#include "blueye_real_bt/behaviors/goto_waypoint.hpp"
#include <cstdlib>
#include <iomanip>
#include <sstream>
#include <rclcpp/rclcpp.hpp>

extern rclcpp::Node::SharedPtr g_node;

GoToWaypoint::GoToWaypoint(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {
}

BT::NodeStatus GoToWaypoint::tick() {
    // Get parameters from ports
    std::string drone_ip = "192.168.1.101";
    getInput<std::string>("drone_ip", drone_ip);

    std::stringstream cmd_ss;
    cmd_ss << "/home/badawi/Desktop/auto-pilot/src/blueye_real_bt/scripts/goto_waypoint.py ";

    // A waypoint of the site layout (placed by the script from the dock reference), or a fixed coordinate
    std::string site_point;
    getInput<std::string>("site_point", site_point);
    if (!site_point.empty()) {
        cmd_ss << "--site-point " << site_point;
        return run(cmd_ss.str());
    }

    double waypoint_lat = 0.0;
    if (!getInput<double>("waypoint_lat", waypoint_lat)) {
        RCLCPP_ERROR(g_node->get_logger(), "GoToWayPoint: Missing required parameter 'waypoint_lat'");
        return BT::NodeStatus::FAILURE;
    }
    
    double waypoint_lon = 0.0;
    if (!getInput<double>("waypoint_lon", waypoint_lon)) {
        RCLCPP_ERROR(g_node->get_logger(), "GoToWaypoint: Missing required parameter 'waypoint_lon'");
        return BT::NodeStatus::FAILURE;
    }
    
    double depth = 1.0;
    getInput<double>("depth", depth);
    
    // Build command with parameters - simple call to the Python script
    cmd_ss << std::setprecision(10)
           << waypoint_lat << " "
           << waypoint_lon << " "
           << depth << " "
           << drone_ip;
    return run(cmd_ss.str());
}

BT::NodeStatus GoToWaypoint::run(const std::string& cmd) {
    RCLCPP_INFO(g_node->get_logger(), "Executing command: %s", cmd.c_str());
    
    // Execute the waypoint mission script
    int result = system(cmd.c_str());
    if (result != 0) {
        RCLCPP_ERROR(g_node->get_logger(),
                    "Waypoint mission failed with error code: %d", result);
        return BT::NodeStatus::FAILURE;
    }
    
    RCLCPP_INFO(g_node->get_logger(),
               "Waypoint mission completed successfully");
    return BT::NodeStatus::SUCCESS;
}