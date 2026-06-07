// include/blueye_stonefish_bt/behaviors/station_keeping.hpp
#pragma once
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include "mundus_mir_msgs/srv/add_waypoint.hpp"
#include "mundus_mir_msgs/srv/run_waypoint_controller.hpp"
#include "mundus_mir_msgs/srv/go_to_waypoints.hpp"
#include "mundus_mir_msgs/srv/clear_waypoints.hpp"
#include "mundus_mir_msgs/srv/get_waypoint_status.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <chrono>
#include <cmath>
#include <limits>
// Forward declaration of global node
extern rclcpp::Node::SharedPtr g_node;
class StationKeeping : public BT::StatefulActionNode {
public:
StationKeeping(const std::string& name, const BT::NodeConfiguration& config);
static BT::PortsList providedPorts() {
return {
BT::InputPort<int>("duration", "Duration in seconds"),
BT::InputPort<double>("heading", std::numeric_limits<double>::quiet_NaN(), "Desired heading in radians; omit to hold current heading"),
BT::InputPort<double>("x", "X coordinate for station keeping (optional)"),
BT::InputPort<double>("y", "Y coordinate for station keeping (optional)"),
BT::InputPort<double>("z", "Z coordinate for station keeping (optional)"),
BT::InputPort<bool>("altitude_mode", false, "Whether to use altitude mode instead of fixed z"),
BT::InputPort<double>("target_altitude", 2.0, "Target altitude above seafloor in meters")
 };
 }
protected:
BT::NodeStatus onStart() override;
BT::NodeStatus onRunning() override;
void onHalted() override;
private:
 // Time tracking
std::chrono::steady_clock::time_point start_time_;
 // Service clients
rclcpp::Client<mundus_mir_msgs::srv::ClearWaypoints>::SharedPtr clear_client_;
rclcpp::Client<mundus_mir_msgs::srv::AddWaypoint>::SharedPtr add_client_;
rclcpp::Client<mundus_mir_msgs::srv::RunWaypointController>::SharedPtr run_client_;
rclcpp::Client<mundus_mir_msgs::srv::GoToWaypoints>::SharedPtr go_client_;
rclcpp::Client<mundus_mir_msgs::srv::GetWaypointStatus>::SharedPtr status_client_;
 // Odometry for current position/heading fallback
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
double current_x_{0.0}, current_y_{0.0}, current_z_{0.0}, current_yaw_{0.0};
 // Helper methods
bool clearWaypoints();
bool addWaypoint(double heading, bool altitude_mode = false, double target_altitude = 2.0);
bool startWaypointController(bool run);
bool startWaypointExecution();
void stopExecution();
};