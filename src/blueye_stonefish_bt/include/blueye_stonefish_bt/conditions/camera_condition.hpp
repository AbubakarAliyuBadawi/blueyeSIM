#pragma once
#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class CheckCameraStatus : public BT::ConditionNode {
public:
    CheckCameraStatus(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts() {
        return { BT::InputPort<double>("timeout_seconds", 10.0, "Max seconds without camera data before failure") };
    }

    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
    rclcpp::Time last_msg_time_;
    double timeout_seconds_ = 10.0;
    bool camera_ok_ = false;

    void cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg);
};
