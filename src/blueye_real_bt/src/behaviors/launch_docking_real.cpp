// launch_docking_procedure.cpp
#include "blueye_real_bt/behaviors/launch_docking_real.hpp"
#include <cerrno>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>

static const char* SCRIPT =
    "/home/badawi/Desktop/auto-pilot/src/blueye_real_bt/bash_scripts/launch_docking_real.sh";

LaunchDockingProcedure::LaunchDockingProcedure(const std::string& name,
                                               const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config)
{
    cmd_pub_        = g_node->create_publisher<geometry_msgs::msg::WrenchStamped>("/blueye/commands", 10);
    depth_client_   = g_node->create_client<std_srvs::srv::SetBool>("/blueye/depth_hold");
    heading_client_ = g_node->create_client<std_srvs::srv::SetBool>("/blueye/heading_hold");
    stationkeep_client_ = g_node->create_client<std_srvs::srv::SetBool>("/blueye/stationkeep");
}

BT::NodeStatus LaunchDockingProcedure::onStart()
{
    RCLCPP_INFO(g_node->get_logger(), "LaunchDocking: starting docking procedure");

    pid_ = fork();
    if (pid_ < 0) {
        RCLCPP_ERROR(g_node->get_logger(), "LaunchDocking: fork() failed");
        return BT::NodeStatus::FAILURE;
    }
    if (pid_ == 0) {
        setpgid(0, 0);
        execl("/bin/sh", "sh", "-c", SCRIPT, nullptr);
        _exit(127);
    }

    RCLCPP_INFO(g_node->get_logger(), "LaunchDocking: script started (pid %d)", pid_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus LaunchDockingProcedure::onRunning()
{
    int status;
    pid_t result = waitpid(pid_, &status, WNOHANG);

    if (result == 0)
        return BT::NodeStatus::RUNNING;   // script still running

    const pid_t group = pid_;
    pid_ = -1;
    stopGroup(group);                     // docking nodes the script's launch left running
    stopThrusters();

    if (result < 0) {
        RCLCPP_ERROR(g_node->get_logger(), "LaunchDocking: waitpid error");
        return BT::NodeStatus::FAILURE;
    }

    int exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
    if (exit_code == 0) {
        RCLCPP_INFO(g_node->get_logger(), "LaunchDocking: docking complete (charging detected)");
        return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_ERROR(g_node->get_logger(),
                 "LaunchDocking: script exited with code %d", exit_code);
    holdPosition();                       // the takeover handler follows: hold while the operator decides
    return BT::NodeStatus::FAILURE;
}

void LaunchDockingProcedure::onHalted()
{
    if (pid_ <= 0) return;

    RCLCPP_INFO(g_node->get_logger(),
                "LaunchDocking: halted — stopping docking script (pid %d)", pid_);

    // SIGINT the entire process group: stops the shell script AND the
    // ros2 launch docking nodes it spawned (the docking controller among them).
    const pid_t group = pid_;
    kill(-group, SIGINT);

    // Wait up to 6 s for clean shutdown, then force-kill.
    bool shell_done = false;
    for (int i = 0; i < 60 && !shell_done; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        shell_done = waitpid(group, nullptr, WNOHANG) == group;
    }
    if (!shell_done) {
        kill(-group, SIGKILL);
        waitpid(group, nullptr, 0);
    }
    pid_ = -1;

    stopGroup(group);
    stopThrusters();
    holdPosition();
}

// Every process left in the docking step's group (its shell already reaped) is stopped before the thrusters are
// zeroed, so the docking controller sends nothing after the zero. ros2 launch stopped with SIGTERM leaves its nodes
// running; they keep the group of the script.
void LaunchDockingProcedure::stopGroup(pid_t group)
{
    if (kill(-group, 0) == -1 && errno == ESRCH)
        return;
    kill(-group, SIGINT);
    for (int i = 0; i < 50; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (kill(-group, 0) == -1 && errno == ESRCH)
            return;
    }
    RCLCPP_WARN(g_node->get_logger(), "LaunchDocking: docking nodes still running — killing them");
    kill(-group, SIGKILL);
}

void LaunchDockingProcedure::stopThrusters()
{
    geometry_msgs::msg::WrenchStamped zero;
    zero.header.frame_id = "body_frame";
    for (int i = 0; i < 3; i++) {
        zero.header.stamp = g_node->now();
        cmd_pub_->publish(zero);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    RCLCPP_INFO(g_node->get_logger(), "LaunchDocking: thrusters zeroed");
}

void LaunchDockingProcedure::holdPosition()
{
    // Station keeping last: it needs a valid position estimate, and depth and heading hold stay on without one
    for (auto& [client, service] : {std::make_pair(depth_client_, "/blueye/depth_hold"),
                                    std::make_pair(heading_client_, "/blueye/heading_hold"),
                                    std::make_pair(stationkeep_client_, "/blueye/stationkeep")}) {
        if (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(g_node->get_logger(), "LaunchDocking: service not available: %s", service);
            continue;
        }
        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        req->data = true;
        auto future = client->async_send_request(req);
        if (rclcpp::spin_until_future_complete(g_node, future, std::chrono::seconds(2))
            != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(g_node->get_logger(), "LaunchDocking: call timed out: %s", service);
        }
    }
    RCLCPP_INFO(g_node->get_logger(),
                "LaunchDocking: depth and heading hold and station keeping on — drone holds for the operator");
}
