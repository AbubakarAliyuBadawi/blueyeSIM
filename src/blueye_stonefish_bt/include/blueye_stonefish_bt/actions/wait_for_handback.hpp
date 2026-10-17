#pragma once
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include "mundus_mir_msgs/srv/run_waypoint_controller.hpp"

extern rclcpp::Node::SharedPtr g_node;

// Waits for the operator to hand back control. A hand back is kept until the BN urgency is below the
// takeover threshold; only then does the mission resume (the HUD disables its button until then as well).
class WaitForHandback : public BT::StatefulActionNode {
public:
    WaitForHandback(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config), handback_received_(false), urgency_(0.0), threshold_(0.65)
    {
        urgency_sub_ = g_node->create_subscription<std_msgs::msg::Float64>(
            "/blueye/takeover_request/urgency", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) { urgency_ = msg->data; });
        threshold_sub_ = g_node->create_subscription<std_msgs::msg::Float32>(
            "/blueye/takeover_request/threshold", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) { threshold_ = static_cast<double>(msg->data); });
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
        if (urgency_ >= threshold_) {
            RCLCPP_INFO_THROTTLE(g_node->get_logger(), *g_node->get_clock(), 2000,
                "Handback requested — waiting until urgency %.2f < threshold %.2f", urgency_, threshold_);
            return BT::NodeStatus::RUNNING;
        }
        _re_enable_waypoint_controller();
        RCLCPP_INFO(g_node->get_logger(), "Handback received — resuming autonomous mission");
        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override {}  // keep subscription alive

private:
    bool handback_received_;
    double urgency_;
    double threshold_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr urgency_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr threshold_sub_;
    rclcpp::Client<mundus_mir_msgs::srv::RunWaypointController>::SharedPtr run_client_;

    void _re_enable_waypoint_controller() {
        if (!run_client_->wait_for_service(std::chrono::seconds(2))) return;
        auto req = std::make_shared<mundus_mir_msgs::srv::RunWaypointController::Request>();
        req->run = true;
        run_client_->async_send_request(req);
    }
};
