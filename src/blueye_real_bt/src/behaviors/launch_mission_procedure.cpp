#include "blueye_real_bt/behaviors/launch_mission_procedure.hpp"
#include <sstream>
#include <thread>
#include <chrono>
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>
#include <rclcpp/rclcpp.hpp>

LaunchMissionProcedure::LaunchMissionProcedure(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config) {}

BT::NodeStatus LaunchMissionProcedure::onStart()
{
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

    RCLCPP_INFO(rclcpp::get_logger("launch_mission"),
                "Launching pipeline inspection mission with drone IP: %s, max retries: %d",
                drone_ip.c_str(), max_retries);
    RCLCPP_INFO(rclcpp::get_logger("launch_mission"), "Executing command: %s", cmd.c_str());

    pid_ = fork();
    if (pid_ < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("launch_mission"), "fork() failed");
        return BT::NodeStatus::FAILURE;
    }
    if (pid_ == 0) {
        // Child: become its own process group leader so SIGINT reaches all children
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
        return BT::NodeStatus::RUNNING;  // still running

    pid_ = -1;

    if (result < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("launch_mission"), "waitpid error");
        return BT::NodeStatus::FAILURE;
    }

    int exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
    if (exit_code != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("launch_mission"),
                     "Mission failed with exit code: %d", exit_code);
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(rclcpp::get_logger("launch_mission"), "Mission completed successfully");
    return BT::NodeStatus::SUCCESS;
}

void LaunchMissionProcedure::onHalted()
{
    if (pid_ <= 0) return;

    RCLCPP_INFO(rclcpp::get_logger("launch_mission"),
                "Takeover triggered — sending SIGINT to mission script (pid %d) for clean abort", pid_);

    // SIGINT to the whole process group so Python child also gets it
    kill(-pid_, SIGINT);

    // Wait up to 6 seconds for the script to abort the drone mission and disconnect
    for (int i = 0; i < 60; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        int st;
        if (waitpid(pid_, &st, WNOHANG) == pid_) {
            RCLCPP_INFO(rclcpp::get_logger("launch_mission"),
                        "Mission script exited cleanly after takeover");
            pid_ = -1;
            return;
        }
    }

    RCLCPP_WARN(rclcpp::get_logger("launch_mission"),
                "Mission script did not exit in time — sending SIGKILL");
    kill(-pid_, SIGKILL);
    waitpid(pid_, nullptr, 0);
    pid_ = -1;
}
