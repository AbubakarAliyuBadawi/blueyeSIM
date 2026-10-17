#pragma once
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <atomic>

extern rclcpp::Node::SharedPtr g_node;
extern std::atomic<bool> g_decision_pending;   // published on /blueye/takeover_request/pending; the HUD pops up on it

class WaitForHumanDecision : public BT::StatefulActionNode {
public:
    WaitForHumanDecision(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config), decision_received_(false), accepted_(false)
    {
        // Subscribe at construction so messages published during StationKeeping
        // or any other delay before onStart() are never missed.
        sub_ = g_node->create_subscription<std_msgs::msg::Bool>(
            "/blueye/takeover_request/human_decision", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                accepted_ = msg->data;
                decision_received_ = true;
            });
    }

    static BT::PortsList providedPorts() { return {}; }

protected:
    BT::NodeStatus onStart() override {
        // Reset flags so stale messages from before this takeover cycle are ignored.
        decision_received_ = false;
        accepted_ = false;
        g_decision_pending = true;
        RCLCPP_INFO(g_node->get_logger(), "WaitForHumanDecision: waiting for operator response...");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (!decision_received_) return BT::NodeStatus::RUNNING;
        g_decision_pending = false;
        if (accepted_) {
            RCLCPP_INFO(g_node->get_logger(), "Operator ACCEPTED takeover");
            return BT::NodeStatus::SUCCESS;
        }
        RCLCPP_INFO(g_node->get_logger(), "Operator REJECTED takeover — triggering emergency sequence");
        return BT::NodeStatus::FAILURE;
    }

    void onHalted() override { g_decision_pending = false; }  // keep subscription alive across tree resets

private:
    bool decision_received_;
    bool accepted_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
};
