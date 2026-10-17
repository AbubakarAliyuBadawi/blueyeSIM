#pragma once
#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float32.hpp>

extern rclcpp::Node::SharedPtr g_node;
extern bool g_shadow_mode;

// FAILURE (takeover requested) when the BN urgency reaches the threshold; re-armed below 0.8 x threshold.
// In shadow mode the same crossings are only logged and the mission continues.
class CheckTakeoverRequest : public BT::ConditionNode {
public:
    CheckTakeoverRequest(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config), urgency_(0.0), threshold_(0.65)
    {
        urgency_sub_ = g_node->create_subscription<std_msgs::msg::Float64>(
            "/blueye/takeover_request/urgency", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                urgency_ = msg->data;
            });
        threshold_sub_ = g_node->create_subscription<std_msgs::msg::Float32>(
            "/blueye/takeover_request/threshold", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                threshold_ = static_cast<double>(msg->data);
            });
    }

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override {
        constexpr double rearm_ratio = 0.8;

        if (!takeover_armed_) {
            if (urgency_ < threshold_ * rearm_ratio) {
                takeover_armed_ = true;
                RCLCPP_INFO(g_node->get_logger(),
                    "Takeover re-armed: urgency=%.2f < %.2f * threshold=%.2f",
                    urgency_, rearm_ratio, threshold_);
            } else {
                RCLCPP_INFO_THROTTLE(g_node->get_logger(), *g_node->get_clock(), 5000,
                    "Takeover inhibited until urgency clears: urgency=%.2f, rearm below %.2f",
                    urgency_, threshold_ * rearm_ratio);
                return BT::NodeStatus::SUCCESS;
            }
        }

        if (urgency_ >= threshold_) {
            takeover_armed_ = false;
            if (g_shadow_mode) {
                RCLCPP_WARN(g_node->get_logger(),
                    "Shadow mode: takeover would be requested (urgency=%.2f >= threshold=%.2f), mission continues",
                    urgency_, threshold_);
                return BT::NodeStatus::SUCCESS;
            }
            RCLCPP_WARN_THROTTLE(g_node->get_logger(), *g_node->get_clock(), 2000,
                "Takeover requested: urgency=%.2f >= threshold=%.2f", urgency_, threshold_);
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
    }

private:
    double urgency_;
    double threshold_;
    inline static bool takeover_armed_ = true;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr urgency_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr threshold_sub_;
};
