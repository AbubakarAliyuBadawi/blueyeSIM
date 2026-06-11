#pragma once
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

extern rclcpp::Node::SharedPtr g_node;

// Waits until the operator publishes true on the handback topic.
// Re-enabling auto modes is done via ActivateAutoModes in the XML after this node.
class WaitForHandback : public BT::StatefulActionNode {
public:
    WaitForHandback(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config), handback_received_(false)
    {}

    static BT::PortsList providedPorts() { return {}; }

protected:
    BT::NodeStatus onStart() override {
        handback_received_ = false;
        sub_ = g_node->create_subscription<std_msgs::msg::Bool>(
            "/blueye/takeover_request/handback", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data) handback_received_ = true;
            });
        RCLCPP_INFO(g_node->get_logger(),
            "WaitForHandback: joystick control active, waiting for operator handback...");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (!handback_received_) return BT::NodeStatus::RUNNING;
        sub_.reset();
        RCLCPP_INFO(g_node->get_logger(), "Handback received — resuming autonomous mission");
        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override { sub_.reset(); }

private:
    bool handback_received_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
};
