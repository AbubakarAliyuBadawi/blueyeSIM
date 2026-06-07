#pragma once
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include "mundus_mir_msgs/srv/run_waypoint_controller.hpp"

extern rclcpp::Node::SharedPtr g_node;

class EnableJoystick : public BT::SyncActionNode {
public:
    EnableJoystick(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        run_client_ = g_node->create_client<mundus_mir_msgs::srv::RunWaypointController>(
            "/blueye/run_waypoint_controller");
    }

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override {
        if (!run_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(g_node->get_logger(), "EnableJoystick: waypoint controller service not available");
            return BT::NodeStatus::FAILURE;
        }
        auto req = std::make_shared<mundus_mir_msgs::srv::RunWaypointController::Request>();
        req->run = false;
        auto future = run_client_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(g_node, future, std::chrono::seconds(3))
            != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(g_node->get_logger(), "EnableJoystick: failed to stop waypoint controller");
            return BT::NodeStatus::FAILURE;
        }
        RCLCPP_INFO(g_node->get_logger(), "Waypoint controller stopped — joystick is now active");
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Client<mundus_mir_msgs::srv::RunWaypointController>::SharedPtr run_client_;
};
