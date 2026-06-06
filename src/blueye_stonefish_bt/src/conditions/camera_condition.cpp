#include "blueye_stonefish_bt/conditions/camera_condition.hpp"

CheckCameraStatus::CheckCameraStatus(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config), camera_ok_(false)
{
    node_ = rclcpp::Node::make_shared("camera_check_node");

    double timeout = 10.0;
    getInput("timeout_seconds", timeout);
    timeout_seconds_ = timeout;

    // Stonefish front camera topic
    camera_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
        "/blueye/cam/image_color", 10,
        std::bind(&CheckCameraStatus::cameraCallback, this, std::placeholders::_1));

    last_msg_time_ = node_->now();
}

BT::NodeStatus CheckCameraStatus::tick()
{
    rclcpp::spin_some(node_);
    double elapsed = (node_->now() - last_msg_time_).seconds();

    if (camera_ok_ && elapsed < timeout_seconds_) {
        return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_WARN(node_->get_logger(),
                "Camera: no valid data for %.1f s (timeout %.1f s)", elapsed, timeout_seconds_);
    return BT::NodeStatus::FAILURE;
}

void CheckCameraStatus::cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    last_msg_time_ = node_->now();
    camera_ok_ = (msg->width > 0 && msg->height > 0 && !msg->data.empty());
}
