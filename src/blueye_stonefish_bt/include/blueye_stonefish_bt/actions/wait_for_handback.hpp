#pragma once
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include "mundus_mir_msgs/srv/run_waypoint_controller.hpp"

extern rclcpp::Node::SharedPtr g_node;

class WaitForHandback : public BT::StatefulActionNode {
public:
    WaitForHandback(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config), handback_received_(false)
    {
        run_client_ = g_node->create_client<mundus_mir_msgs::srv::RunWaypointController>(
            "/blueye/run_waypoint_controller");
        // Subscribe at construction so handback is never missed regardless of timing.
        sub_ = g_node->create_subscription<std_msgs::msg::Bool>(
            "/blueye/takeover_request/handback", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data) handback_received_ = true;
            });
    }

    static BT::PortsList providedPorts() { return {}; }

protected:
    BT::NodeStatus onStart() override {
        handback_received_ = false;  // reset so previous handbacks don't fire immediately
        RCLCPP_INFO(g_node->get_logger(), "WaitForHandback: joystick control active, waiting for handback...");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (!handback_received_) return BT::NodeStatus::RUNNING;
        _re_enable_waypoint_controller();
        RCLCPP_INFO(g_node->get_logger(), "Handback received — resuming autonomous mission");
        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override {}  // keep subscription alive

private:
    bool handback_received_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
    rclcpp::Client<mundus_mir_msgs::srv::RunWaypointController>::SharedPtr run_client_;

    void _re_enable_waypoint_controller() {
        if (!run_client_->wait_for_service(std::chrono::seconds(2))) return;
        auto req = std::make_shared<mundus_mir_msgs::srv::RunWaypointController::Request>();
        req->run = true;
        run_client_->async_send_request(req);
    }
};
