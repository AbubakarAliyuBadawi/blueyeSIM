#include "blueye_real_bt/behaviors/launch_undocking_real.hpp"
#include <sstream>
#include <thread>
#include <chrono>
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>
#include <rclcpp/rclcpp.hpp>

LaunchUndockingProcedure::LaunchUndockingProcedure(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config) {}

BT::NodeStatus LaunchUndockingProcedure::onStart()
{
    std::string script_path;
    if (!getInput<std::string>("script_path", script_path))
        script_path = "/home/badawi/Desktop/auto-pilot/src/blueye_real_bt/bash_scripts/launch_undocking_real.sh";

    std::string drone_ip;
    if (!getInput<std::string>("drone_ip", drone_ip))
        drone_ip = "192.168.1.101";

    int reverse_duration = 10;
    getInput<int>("reverse_duration", reverse_duration);

    float reverse_power = 0.4f;
    getInput<float>("reverse_power", reverse_power);

    std::stringstream cmd_ss;
    cmd_ss << script_path
           << " --drone-ip " << drone_ip
           << " --reverse-duration " << reverse_duration
           << " --reverse-power " << reverse_power;
    std::string cmd = cmd_ss.str();

    RCLCPP_INFO(rclcpp::get_logger("launch_undocking"),
                "Launching undocking procedure with drone IP: %s, duration: %d, power: %.2f",
                drone_ip.c_str(), reverse_duration, reverse_power);
    RCLCPP_INFO(rclcpp::get_logger("launch_undocking"), "Executing command: %s", cmd.c_str());

    pid_ = fork();
    if (pid_ < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("launch_undocking"), "fork() failed");
        return BT::NodeStatus::FAILURE;
    }
    if (pid_ == 0) {
        setpgid(0, 0);
        execl("/bin/sh", "sh", "-c", cmd.c_str(), nullptr);
        _exit(127);
    }

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus LaunchUndockingProcedure::onRunning()
{
    int status;
    pid_t result = waitpid(pid_, &status, WNOHANG);

    if (result == 0)
        return BT::NodeStatus::RUNNING;

    pid_ = -1;

    if (result < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("launch_undocking"), "waitpid error");
        return BT::NodeStatus::FAILURE;
    }

    int exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
    if (exit_code != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("launch_undocking"),
                     "Undocking failed with error code: %d", exit_code);
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(rclcpp::get_logger("launch_undocking"), "Undocking completed successfully");
    return BT::NodeStatus::SUCCESS;
}

void LaunchUndockingProcedure::onHalted()
{
    if (pid_ <= 0) return;

    RCLCPP_INFO(rclcpp::get_logger("launch_undocking"),
                "Halting undocking — sending SIGINT to script (pid %d)", pid_);

    kill(-pid_, SIGINT);

    for (int i = 0; i < 40; i++) {
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
}
