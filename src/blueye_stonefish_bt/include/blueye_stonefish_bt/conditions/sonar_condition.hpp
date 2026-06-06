#pragma once
#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class CheckSonarStatus : public BT::ConditionNode {
public:
    CheckSonarStatus(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts() {
        return { BT::InputPort<double>("timeout_seconds", 5.0, "Max seconds without FLS data before failure") };
    }

    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr fls_sub_;
    rclcpp::Time last_msg_time_;
    double timeout_seconds_ = 5.0;

    void flsCallback(const sensor_msgs::msg::Image::SharedPtr msg);
};
