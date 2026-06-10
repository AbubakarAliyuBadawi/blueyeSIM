// launch_docking_procedure.cpp
#include "blueye_real_bt/behaviors/launch_docking_real.hpp"
#include <cstdlib>

LaunchDockingProcedure::LaunchDockingProcedure(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {
}

BT::NodeStatus LaunchDockingProcedure::tick() {
    RCLCPP_INFO(rclcpp::get_logger("launch_docking"), "Launching docking procedure");
    
    std::string cmd = "/home/badawi/Desktop/auto-pilot/src/blueye_real_bt/bash_scripts/launch_docking_real.sh";
    RCLCPP_INFO(rclcpp::get_logger("launch_docking"), "Executing command: %s", cmd.c_str());

    int result = system(cmd.c_str());
    if (result != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("launch_docking"),
                    "Docking procedure failed with error code: %d", result);
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(rclcpp::get_logger("launch_docking"),
               "Docking procedure completed successfully");
    return BT::NodeStatus::SUCCESS;
}