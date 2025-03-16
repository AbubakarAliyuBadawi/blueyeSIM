#include "blueye_altitude_controller/altitude_controller.hpp"
#include <cmath>
#include <algorithm>

AltitudeController::AltitudeController() : Node("altitude_controller") {
    // Parameters
    declare_parameter("target_altitude", 2.0);       // Default 2 meters
    declare_parameter("max_z_velocity", 0.3);        // Max vertical speed
    declare_parameter("altitude_control_enabled", false);
    declare_parameter("min_depth", 1.0);             // Minimum depth (m)
    declare_parameter("max_depth", 30.0);            // Maximum depth (m)
    declare_parameter("safety_margin", 0.5);         // Safety margin (m)
    declare_parameter("kp", 0.5);                    // Proportional gain
    declare_parameter("ki", 0.1);                    // Integral gain
    declare_parameter("kd", 0.2);                    // Derivative gain
    declare_parameter("velocity_topic", "/blueye/ref_vel");
    
    // Get parameters
    target_altitude_ = get_parameter("target_altitude").as_double();
    max_z_velocity_ = get_parameter("max_z_velocity").as_double();
    altitude_control_enabled_ = get_parameter("altitude_control_enabled").as_bool();
    min_depth_ = get_parameter("min_depth").as_double();
    max_depth_ = get_parameter("max_depth").as_double();
    safety_margin_ = get_parameter("safety_margin").as_double();
    kp_ = get_parameter("kp").as_double();
    ki_ = get_parameter("ki").as_double();
    kd_ = get_parameter("kd").as_double();
    velocity_topic_ = get_parameter("velocity_topic").as_string();
    
    // Subscriptions
    dvl_sub_ = create_subscription<marine_acoustic_msgs::msg::Dvl>(
        "/blueye/dvl", 10, 
        std::bind(&AltitudeController::dvlCallback, this, std::placeholders::_1));
        
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/blueye/odometry_flu/gt", 10,
        std::bind(&AltitudeController::odomCallback, this, std::placeholders::_1));
        
    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/blueye/altitude_control/enable", 10,
        std::bind(&AltitudeController::enableCallback, this, std::placeholders::_1));
        
    target_sub_ = create_subscription<std_msgs::msg::Float64>(
        "/blueye/altitude_control/target", 10,
        std::bind(&AltitudeController::targetCallback, this, std::placeholders::_1));
        
    // Publishers
    velocity_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
        velocity_topic_, 10);
        
    status_pub_ = create_publisher<std_msgs::msg::String>(
        "/blueye/altitude_control/status", 10);
        
    // Timer for control loop
    timer_ = create_wall_timer(
        std::chrono::milliseconds(50),  // 20 Hz control loop
        std::bind(&AltitudeController::controlLoop, this));
        
    RCLCPP_INFO(get_logger(), "Altitude controller initialized with target altitude: %.2f m", 
               target_altitude_);
    RCLCPP_INFO(get_logger(), "Safety limits: min_depth=%.2f m, max_depth=%.2f m, margin=%.2f m",
               min_depth_, max_depth_, safety_margin_);
}

void AltitudeController::dvlCallback(const marine_acoustic_msgs::msg::Dvl::SharedPtr msg) {
    current_altitude_ = msg->altitude;
    last_dvl_time_ = this->now();
    dvl_valid_ = true;
    
    RCLCPP_DEBUG(get_logger(), "DVL update: altitude=%.2f m", current_altitude_);
}

void AltitudeController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_z_ = msg->pose.pose.position.z;
    current_velocity_x_ = msg->twist.twist.linear.x;
    current_velocity_y_ = msg->twist.twist.linear.y;
}

void AltitudeController::enableCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    altitude_control_enabled_ = msg->data;
    if (altitude_control_enabled_) {
        // Reset PID controller state when enabling
        error_sum_ = 0.0;
        last_error_ = 0.0;
        first_run_ = true;
        RCLCPP_INFO(get_logger(), "Altitude control enabled");
    } else {
        RCLCPP_INFO(get_logger(), "Altitude control disabled");
    }
}

void AltitudeController::targetCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    double new_target = msg->data;
    
    // Validate the target is reasonable
    if (new_target < 0.5) {
        RCLCPP_WARN(get_logger(), "Target altitude too low (%.2f m), setting to 0.5 m", new_target);
        new_target = 0.5;
    } else if (new_target > 10.0) {
        RCLCPP_WARN(get_logger(), "Target altitude too high (%.2f m), setting to 10.0 m", new_target);
        new_target = 10.0;
    }
    
    target_altitude_ = new_target;
    RCLCPP_INFO(get_logger(), "Target altitude updated: %.2f m", target_altitude_);
}

void AltitudeController::controlLoop() {
    if (!altitude_control_enabled_) {
        return;
    }
    
    // Check if DVL data is valid
    if (!dvl_valid_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No valid DVL data received yet");
        return;
    }
    
    // Check if DVL data is recent (within 1 second)
    auto time_since_dvl = this->now() - last_dvl_time_;
    if (time_since_dvl.seconds() > 1.0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, 
                           "DVL data too old (%.1f seconds), pausing altitude control", 
                           time_since_dvl.seconds());
        return;
    }
    
    // Calculate depth (negative z is depth in FLU frame)
    double current_depth = -current_z_;
    
    // Status message to publish
    auto status_msg = std::make_unique<std_msgs::msg::String>();
    
    // Check depth safety limits
    if (current_depth < min_depth_ + safety_margin_) {
        // Too close to surface - override with downward movement
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, 
                           "Approaching minimum depth limit (%.2f m)", current_depth);
        
        // Calculate how strongly to respond based on proximity to limit
        double proximity = (min_depth_ - current_depth) / safety_margin_;
        double response = std::min(-0.05, -0.2 * proximity);  // Gentle to stronger response
        
        auto cmd = std::make_unique<geometry_msgs::msg::TwistStamped>();
        cmd->header.stamp = this->now();
        cmd->header.frame_id = "body";
        cmd->twist.linear.z = response;  // Downward movement
        velocity_pub_->publish(std::move(cmd));
        
        status_msg->data = "WARNING: Approaching minimum depth limit";
        status_pub_->publish(std::move(status_msg));
        return;
    }
    
    if (current_depth > max_depth_ - safety_margin_) {
        // Too deep - override with upward movement
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, 
                           "Approaching maximum depth limit (%.2f m)", current_depth);
        
        // Calculate how strongly to respond based on proximity to limit
        double proximity = (current_depth - max_depth_) / safety_margin_;
        double response = std::min(0.05, 0.2 * proximity);  // Gentle to stronger response
        
        auto cmd = std::make_unique<geometry_msgs::msg::TwistStamped>();
        cmd->header.stamp = this->now();
        cmd->header.frame_id = "body";
        cmd->twist.linear.z = response;  // Upward movement
        velocity_pub_->publish(std::move(cmd));
        
        status_msg->data = "WARNING: Approaching maximum depth limit";
        status_pub_->publish(std::move(status_msg));
        return;
    }
    
    // Calculate altitude error (positive if need to go up)
    double error = target_altitude_ - current_altitude_;
    
    // PID control calculation
    auto current_time = this->now();
    double dt = 0.05;  // Default to timer period
    
    if (!first_run_) {
        dt = (current_time - last_control_time_).seconds();
        if (dt <= 0.0 || dt > 0.5) {
            dt = 0.05;  // Use default if time jump detected
        }
    }
    
    // Proportional term
    double p_term = kp_ * error;
    
    // Integral term with anti-windup
    error_sum_ += error * dt;
    error_sum_ = std::clamp(error_sum_, -5.0, 5.0);  // Limit integral windup
    double i_term = ki_ * error_sum_;
    
    // Derivative term
    double d_term = 0.0;
    if (!first_run_) {
        double error_rate = (error - last_error_) / dt;
        d_term = kd_ * error_rate;
    }
    
    // Calculate control output
    double z_velocity = p_term + i_term + d_term;
    
    // Limit the velocity
    z_velocity = std::clamp(z_velocity, -max_z_velocity_, max_z_velocity_);
    
    // Additional safety check based on predicted future depth
    double horizontal_speed = std::sqrt(
        current_velocity_x_ * current_velocity_x_ + 
        current_velocity_y_ * current_velocity_y_
    );
    
    // Adjust safety margin based on horizontal speed
    double dynamic_margin = safety_margin_ + (horizontal_speed * 0.5); // Half-second of travel
    
    // Predict depth after 1 second with current command
    double predicted_depth = current_depth + z_velocity;
    
    if (predicted_depth < min_depth_ + dynamic_margin) {
        // Limit downward movement
        z_velocity = std::max(z_velocity, 0.0);
    }
    
    if (predicted_depth > max_depth_ - dynamic_margin) {
        // Limit upward movement
        z_velocity = std::min(z_velocity, 0.0);
    }
    
    // Publish velocity command
    auto cmd = std::make_unique<geometry_msgs::msg::TwistStamped>();
    cmd->header.stamp = this->now();
    cmd->header.frame_id = "body";
    cmd->twist.linear.z = z_velocity;
    velocity_pub_->publish(std::move(cmd));
    
    // Update state
    last_error_ = error;
    last_control_time_ = current_time;
    first_run_ = false;
    
    // Publish status
    status_msg->data = "Normal: Altitude control active, target=" + 
                      std::to_string(target_altitude_) + "m, current=" + 
                      std::to_string(current_altitude_) + "m";
    status_pub_->publish(std::move(status_msg));
    
    // Log status periodically
    RCLCPP_DEBUG(get_logger(), 
               "Altitude: %.2fm, Target: %.2fm, Error: %.2fm, Command: %.2fm/s", 
               current_altitude_, target_altitude_, error, z_velocity);
}