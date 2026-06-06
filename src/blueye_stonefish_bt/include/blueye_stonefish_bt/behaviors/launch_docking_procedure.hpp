#pragma once
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include "mundus_mir_msgs/srv/run_waypoint_controller.hpp"

extern rclcpp::Node::SharedPtr g_node;

class LaunchDockingProcedure : public BT::SyncActionNode {
public:
    LaunchDockingProcedure(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override;

private:
    rclcpp::Client<mundus_mir_msgs::srv::RunWaypointController>::SharedPtr stop_waypoint_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr start_docking_client_;
};