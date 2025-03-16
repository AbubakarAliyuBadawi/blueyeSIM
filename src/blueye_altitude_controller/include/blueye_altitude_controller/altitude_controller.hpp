#ifndef BLUEYE_ALTITUDE_CONTROLLER_HPP
#define BLUEYE_ALTITUDE_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <marine_acoustic_msgs/msg/dvl.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

class AltitudeController : public rclcpp::Node {
public:
    AltitudeController();

private:
    // Subscriptions
    rclcpp::Subscription<marine_acoustic_msgs::msg::Dvl>::SharedPtr dvl_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_sub_;
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Parameters
    double target_altitude_;
    double max_z_velocity_;
    bool altitude_control_enabled_;
    double min_depth_;
    double max_depth_;
    double safety_margin_;
    double kp_, ki_, kd_;
    std::string velocity_topic_;
    
    // State variables
    double current_altitude_ = 0.0;
    double current_z_ = 0.0;
    double current_velocity_x_ = 0.0;
    double current_velocity_y_ = 0.0;
    rclcpp::Time last_dvl_time_;
    bool dvl_valid_ = false;
    
    // PID controller state
    double error_sum_ = 0.0;
    double last_error_ = 0.0;
    rclcpp::Time last_control_time_;
    bool first_run_ = true;
    
    // Callback functions
    void dvlCallback(const marine_acoustic_msgs::msg::Dvl::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void enableCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void targetCallback(const std_msgs::msg::Float64::SharedPtr msg);
    
    // Control loop
    void controlLoop();
};

#endif // BLUEYE_ALTITUDE_CONTROLLER_HPP