// launch_docking_procedure.cpp
#include "blueye_real_bt/behaviors/launch_docking_real.hpp"
#include <chrono>
#include <thread>
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>

static const char* SCRIPT =
    "/home/badawi/Desktop/auto-pilot/src/blueye_real_bt/bash_scripts/launch_docking_real.sh";

LaunchDockingProcedure::LaunchDockingProcedure(const std::string& name,
                                               const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config) {}

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

    pid_ = -1;

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
    return BT::NodeStatus::FAILURE;
}

void LaunchDockingProcedure::onHalted()
{
    if (pid_ <= 0) return;

    RCLCPP_INFO(g_node->get_logger(),
                "LaunchDocking: halted — stopping docking script (pid %d)", pid_);

    // SIGINT the entire process group: stops the shell script AND the
    // ros2 launch docking nodes it spawned.
    kill(-pid_, SIGINT);

    // Wait up to 6 s for clean shutdown, then force-kill.
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
}
