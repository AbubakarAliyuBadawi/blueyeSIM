#pragma once
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

extern rclcpp::Node::SharedPtr g_node;

// Waits until the docking controller reports "docking_complete" on /blueye/docking_controller_status
// (last waypoint reached, or final push into the dock finished). FAILURE after `timeout` seconds.
// A "docking_complete" is only accepted after the controller has been seen active in this attempt, so a
// completion left over from an earlier docking cannot end the wait.
class WaitForDocking : public BT::StatefulActionNode {
public:
    WaitForDocking(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config)
    {
        sub_ = g_node->create_subscription<std_msgs::msg::String>(
            "/blueye/docking_controller_status", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                if (!waiting_) return;
                if (msg->data.rfind("docking_complete", 0) == 0) {
                    if (seen_active_) complete_ = true;
                } else {
                    seen_active_ = true;
                }
            });
    }

    static BT::PortsList providedPorts() {
        return { BT::InputPort<int>("timeout", 300, "Seconds before docking is considered failed") };
    }

protected:
    BT::NodeStatus onStart() override {
        complete_ = false;
        seen_active_ = false;
        waiting_ = true;
        start_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(g_node->get_logger(), "WaitForDocking: waiting for the docking controller to report completion");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - start_).count();
        if (complete_) {
            waiting_ = false;
            RCLCPP_INFO(g_node->get_logger(), "Docking completed after %ld s", static_cast<long>(elapsed));
            return BT::NodeStatus::SUCCESS;
        }
        int timeout = 300;
        getInput("timeout", timeout);
        if (elapsed >= timeout) {
            waiting_ = false;
            RCLCPP_ERROR(g_node->get_logger(), "Docking not completed within %d s", timeout);
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override { waiting_ = false; }

private:
    bool complete_{false};
    bool seen_active_{false};
    bool waiting_{false};
    std::chrono::steady_clock::time_point start_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};
