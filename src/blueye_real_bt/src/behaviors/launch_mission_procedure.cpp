#include "blueye_real_bt/behaviors/launch_mission_procedure.hpp"
#include <sstream>
#include <thread>
#include <chrono>
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>
#include <rclcpp/rclcpp.hpp>

// Defined here, extern-declared in the header so main.cpp can read it
bool g_mission_paused = false;

LaunchMissionProcedure::LaunchMissionProcedure(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config) {}

BT::NodeStatus LaunchMissionProcedure::onStart()
{
    // Create publisher on first use
    if (!control_pub_) {
        control_pub_ = g_node->create_publisher<std_msgs::msg::String>(
            "/blueye/mission/control", 10);
    }

    // If a previous run left the script paused, resume it instead of starting fresh
    if (pid_ > 0 && script_paused_) {
        int st;
        if (waitpid(pid_, &st, WNOHANG) == 0) {
            // Process still alive — send resume signal
            auto msg = std_msgs::msg::String();
            msg.data = "resume";
            control_pub_->publish(msg);
            script_paused_ = false;
            RCLCPP_INFO(g_node->get_logger(),
                "LaunchMission: resuming paused mission script (pid %d)", pid_);
            return BT::NodeStatus::RUNNING;
        }
        // Script exited while paused (unexpected)
        RCLCPP_WARN(g_node->get_logger(),
            "LaunchMission: paused script exited — starting fresh");
        pid_ = -1;
        script_paused_ = false;
    }

    // Fresh start
    std::string script_path;
    if (!getInput<std::string>("script_path", script_path))
        script_path = "/home/badawi/Desktop/auto-pilot/src/blueye_real_bt/bash_scripts/launch_mission.sh";

    std::string drone_ip;
    if (!getInput<std::string>("drone_ip", drone_ip))
        drone_ip = "192.168.1.101";

    int max_retries = 5;
    getInput<int>("max_retries", max_retries);

    std::stringstream cmd_ss;
    cmd_ss << script_path
           << " --drone-ip " << drone_ip
           << " --max-retries " << max_retries;
    std::string cmd = cmd_ss.str();

    RCLCPP_INFO(g_node->get_logger(),
                "LaunchMission: starting script — ip=%s retries=%d",
                drone_ip.c_str(), max_retries);
    RCLCPP_INFO(g_node->get_logger(), "LaunchMission: cmd: %s", cmd.c_str());

    pid_ = fork();
    if (pid_ < 0) {
        RCLCPP_ERROR(g_node->get_logger(), "LaunchMission: fork() failed");
        return BT::NodeStatus::FAILURE;
    }
    if (pid_ == 0) {
        setpgid(0, 0);
        execl("/bin/sh", "sh", "-c", cmd.c_str(), nullptr);
        _exit(127);
    }

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus LaunchMissionProcedure::onRunning()
{
    int status;
    pid_t result = waitpid(pid_, &status, WNOHANG);

    if (result == 0)
        return BT::NodeStatus::RUNNING;

    pid_ = -1;

    if (result < 0) {
        RCLCPP_ERROR(g_node->get_logger(), "LaunchMission: waitpid error");
        return BT::NodeStatus::FAILURE;
    }

    int exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
    if (exit_code != 0) {
        RCLCPP_ERROR(g_node->get_logger(),
                     "LaunchMission: script exited with code %d", exit_code);
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(g_node->get_logger(), "LaunchMission: mission completed successfully");
    return BT::NodeStatus::SUCCESS;
}

void LaunchMissionProcedure::onHalted()
{
    if (pid_ <= 0) return;

    if (!g_program_running) {
        // Full BT shutdown — kill the script cleanly
        RCLCPP_INFO(g_node->get_logger(),
                    "LaunchMission: shutdown — sending SIGINT to script (pid %d)", pid_);
        kill(-pid_, SIGINT);
        for (int i = 0; i < 60; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            int st;
            if (waitpid(pid_, &st, WNOHANG) == pid_) {
                pid_ = -1;
                return;
            }
        }
        kill(-pid_, SIGKILL);
        waitpid(pid_, nullptr, 0);
        pid_ = -1;
        return;
    }

    // Takeover path — pause the mission, keep the script alive
    if (!control_pub_) {
        control_pub_ = g_node->create_publisher<std_msgs::msg::String>(
            "/blueye/mission/control", 10);
    }
    auto msg = std_msgs::msg::String();
    msg.data = "pause";
    control_pub_->publish(msg);
    script_paused_ = true;
    // g_mission_paused is NOT set here — EnableJoystick sets it only if the
    // operator accepts, so the REJECT/emergency path never triggers a tree restart.
    RCLCPP_INFO(g_node->get_logger(),
                "LaunchMission: takeover — mission paused (pid %d alive, drone holds depth)",
                pid_);
    // pid_ is intentionally kept so onStart() can resume later
}

void LaunchMissionProcedure::cleanup_if_needed()
{
    if (pid_ <= 0) return;
    RCLCPP_INFO(rclcpp::get_logger("launch_mission"),
                "LaunchMission: cleanup — sending SIGINT to script (pid %d)", pid_);
    kill(-pid_, SIGINT);
    for (int i = 0; i < 60; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        int st;
        if (waitpid(pid_, &st, WNOHANG) == pid_) break;
    }
    pid_ = -1;
    script_paused_ = false;
}
