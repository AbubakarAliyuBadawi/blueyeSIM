#pragma once
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

extern rclcpp::Node::SharedPtr g_node;

class WaitForHumanDecision : public BT::StatefulActionNode {
public:
    WaitForHumanDecision(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config), decision_received_(false), accepted_(false)
    {}

    // Writes decision to blackboard so the BT can branch on it.
    static BT::PortsList providedPorts() {
        return { BT::OutputPort<bool>("decision", "true = accepted, false = rejected") };
    }

protected:
    BT::NodeStatus onStart() override {
        decision_received_ = false;
        accepted_ = false;
        sub_ = g_node->create_subscription<std_msgs::msg::Bool>(
            "/blueye/takeover_request/human_decision", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                accepted_ = msg->data;
                decision_received_ = true;
            });
        RCLCPP_INFO(g_node->get_logger(), "WaitForHumanDecision: waiting for operator response...");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (!decision_received_) return BT::NodeStatus::RUNNING;
        sub_.reset();
        // Write to blackboard so the BT can branch on the decision
        setOutput("decision", accepted_);
        RCLCPP_INFO(g_node->get_logger(),
            "Operator %s takeover", accepted_ ? "ACCEPTED" : "REJECTED");
        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override { sub_.reset(); }

private:
    bool decision_received_;
    bool accepted_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
};
