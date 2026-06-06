#include "blueye_stonefish_bt/conditions/system_watchdog.hpp"

SystemWatchdog::SystemWatchdog(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
{
    node_ = rclcpp::Node::make_shared("system_watchdog_node");
    startup_time_ = node_->now();

    int failures = 3;
    getInput("max_failures", failures);
    max_failures_before_reset_ = failures;
    startup_grace_period_ = 10.0;

    rclcpp::Time now = node_->now();
    // Monitor Stonefish odometry (/blueye/odom) and cmd_force
    monitored_topics_["/blueye/odom"] = {"Odometry", now, 10.0, true, false};
    monitored_topics_["/blueye/cmd_force"] = {"Command Force", now, 10.0, false, false};

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/blueye/odom", 10,
        std::bind(&SystemWatchdog::odomCallback, this, std::placeholders::_1));

    cmd_force_sub_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/blueye/cmd_force", 10,
        std::bind(&SystemWatchdog::cmdForceCallback, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "System watchdog started (grace period %.1f s)", startup_grace_period_);
}

BT::NodeStatus SystemWatchdog::tick()
{
    rclcpp::spin_some(node_);

    if (in_startup_grace_period_) {
        double elapsed = (node_->now() - startup_time_).seconds();
        if (elapsed < startup_grace_period_) {
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                "Watchdog grace period (%.1f/%.1f s)", elapsed, startup_grace_period_);
            return BT::NodeStatus::SUCCESS;
        }
        in_startup_grace_period_ = false;
        RCLCPP_INFO(node_->get_logger(), "Watchdog grace period ended");
    }

    std::string failing;
    bool healthy = checkSystemHealth(failing);
    setOutput("failing_topics", failing);

    if (healthy) {
        consecutive_failures_ = 0;
        return BT::NodeStatus::SUCCESS;
    }

    consecutive_failures_++;
    RCLCPP_WARN(node_->get_logger(), "System unhealthy: %s (%d/%d)",
                failing.c_str(), consecutive_failures_, max_failures_before_reset_);
    return BT::NodeStatus::FAILURE;
}

bool SystemWatchdog::checkSystemHealth(std::string& failing_topics)
{
    rclcpp::Time now = node_->now();
    bool all_critical_ok = true;
    failing_topics = "";

    for (auto& [topic, monitor] : monitored_topics_) {
        double elapsed = (now - monitor.last_update).seconds();
        bool ok = monitor.is_active && elapsed < monitor.timeout_seconds;
        if (!ok) {
            if (!failing_topics.empty()) failing_topics += ", ";
            failing_topics += monitor.name + (monitor.is_active
                ? " (timeout: " + std::to_string(elapsed) + "s)"
                : " (no data)");
            if (monitor.is_critical) all_critical_ok = false;
        }
    }
    return all_critical_ok;
}

void SystemWatchdog::odomCallback(const nav_msgs::msg::Odometry::SharedPtr /*msg*/)
{
    monitored_topics_["/blueye/odom"].last_update = node_->now();
    monitored_topics_["/blueye/odom"].is_active = true;
}

void SystemWatchdog::cmdForceCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr /*msg*/)
{
    monitored_topics_["/blueye/cmd_force"].last_update = node_->now();
    monitored_topics_["/blueye/cmd_force"].is_active = true;
}
