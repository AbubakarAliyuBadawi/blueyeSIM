#include "blueye_stonefish_bt/conditions/sonar_condition.hpp"

CheckSonarStatus::CheckSonarStatus(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
{
    node_ = rclcpp::Node::make_shared("sonar_check_node");

    double timeout = 5.0;
    getInput("timeout_seconds", timeout);
    timeout_seconds_ = timeout;

    // Stonefish FLS publishes sensor_msgs/Image on /blueye/fls/image
    fls_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
        "/blueye/fls/image", 10,
        std::bind(&CheckSonarStatus::flsCallback, this, std::placeholders::_1));

    last_msg_time_ = node_->now();
}

BT::NodeStatus CheckSonarStatus::tick()
{
    rclcpp::spin_some(node_);
    double elapsed = (node_->now() - last_msg_time_).seconds();

    if (elapsed < timeout_seconds_) {
        return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_WARN(node_->get_logger(),
                "FLS sonar: no data for %.1f s (timeout %.1f s)", elapsed, timeout_seconds_);
    return BT::NodeStatus::FAILURE;
}

void CheckSonarStatus::flsCallback(const sensor_msgs::msg::Image::SharedPtr /*msg*/)
{
    last_msg_time_ = node_->now();
}
