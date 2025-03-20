#include "mundus_mir_waypoint_controller/mundus_mir_waypoint_controller.hpp"

float calculate_distance(const Waypoint &waypoint, const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Calculate distance between waypoint and current position
    return sqrt(pow(waypoint.x - msg->pose.pose.position.x, 2) + pow(waypoint.y - msg->pose.pose.position.y, 2));
}

double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion &q)
{
    // Convert quaternion to yaw (rotation around Z axis)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

WaypointController::WaypointController() : Node("mundus_mir_waypoint_controller")
{
    // Fetch ROS parameters
    get_ros_parameters();

    // Set up publishers, subscribers and services
    setup_interfaces();
}

WaypointController::~WaypointController()
{
}

void WaypointController::controller_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Check if we should run the controller
    if (!run_controller_)
    {
        return;
    }

    // Check if waypoint following should be run
    if (go_to_waypoint_)
    {

        // Check current waypoint against odometry
        if (calculate_distance(waypoints_.front(), msg) < circle_of_acceptance_)
        {
            // If this is the last waypoint we should go to stationkeep
            if (waypoints_.size() == 1)
            {
                stationary_waypoint_ = waypoints_.front();
                // If the current waypoint does not have a fixed heading, set it to the current heading
                if (!stationary_waypoint_.fixed_heading) {
                    stationary_waypoint_.heading = get_yaw_from_quaternion(msg->pose.pose.orientation); 
                    stationary_waypoint_.fixed_heading = true;
                }
                go_to_waypoint_ = false;
                stationary_waypoint_set_ = true;
            }
            // If we are close to the waypoint, remove it
            waypoints_.erase(waypoints_.begin());
            return;
        }

        // Go to the current waypoint
        go_to_waypoint(msg);
        return;
    }

    // If not station keep on stationary_waypoint
    if (stationary_waypoint_set_)
    {
        station_keep(msg);
        return;
    }
    // If it is not set, it should be set to current pose
    stationary_waypoint_.x = msg->pose.pose.position.x;
    stationary_waypoint_.y = msg->pose.pose.position.y;
    stationary_waypoint_.z = msg->pose.pose.position.z;
    stationary_waypoint_.desired_speed = 0.0;
    stationary_waypoint_.fixed_heading = true;
    stationary_waypoint_.heading = get_yaw_from_quaternion(msg->pose.pose.orientation);
    stationary_waypoint_set_ = true;
}

void WaypointController::dvl_callback(const marine_acoustic_msgs::msg::Dvl::SharedPtr msg)
{
    // Store raw altitude for debugging
    float raw_altitude = msg->altitude;
    
    // Apply filtering to altitude measurements
    if (!altitude_filter_initialized_) {
        // Initialize filter with first valid measurement
        filtered_altitude_ = raw_altitude;
        for (size_t i = 0; i < ALTITUDE_BUFFER_SIZE; ++i) {
            altitude_buffer_.push_back(raw_altitude);
        }
        altitude_filter_initialized_ = true;
    } else {
        // Add new measurement to buffer
        altitude_buffer_.push_back(raw_altitude);
        if (altitude_buffer_.size() > ALTITUDE_BUFFER_SIZE) {
            altitude_buffer_.pop_front();
        }
        
        // Calculate median of buffer (more robust than mean)
        std::vector<float> sorted_altitudes(altitude_buffer_.begin(), altitude_buffer_.end());
        std::sort(sorted_altitudes.begin(), sorted_altitudes.end());
        float median_altitude = sorted_altitudes[sorted_altitudes.size() / 2];
        
        // Apply low-pass filter to median value for even smoother result
        const float alpha = 0.1; // Lower = smoother but more lag
        filtered_altitude_ = alpha * median_altitude + (1.0 - alpha) * filtered_altitude_;
    }
    
    // Use filtered altitude for control
    current_altitude_ = filtered_altitude_;
    
    last_dvl_time_ = this->now();
    dvl_valid_ = true;
    
    RCLCPP_INFO(get_logger(),
                "DVL update: raw=%.2f m, filtered=%.2f m, delta=%.2f m", 
                raw_altitude, filtered_altitude_, raw_altitude - filtered_altitude_);
}

void WaypointController::go_to_waypoint(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Get current waypoint reference
    const Waypoint& current_waypoint = waypoints_.front();
    
    // Create velocity command structure
    geometry_msgs::msg::TwistStamped desired_velocity;
    desired_velocity.header.stamp = msg->header.stamp;
    desired_velocity.header.frame_id = msg->header.frame_id;
    
    // Handle heading control based on fixed_heading flag
    float heading_err = 0.0;
    
    if (!current_waypoint.fixed_heading)
    {
        // If heading is outside tolerance fix it before we start going toward the next waypoint
        // Desired heading is straight towards the waypoint
        float desired_heading = atan2(current_waypoint.y - msg->pose.pose.position.y, 
                                      current_waypoint.x - msg->pose.pose.position.x);
        heading_err = desired_heading - get_yaw_from_quaternion(msg->pose.pose.orientation);

        // Check for minimum signed angle
        if (heading_err > M_PI)
        {
            heading_err -= 2 * M_PI;
        }
        else if (heading_err < -M_PI)
        {
            heading_err += 2 * M_PI;
        }

        if (abs(heading_err) > heading_tolerance_)
        {
            // Rotate towards the correct heading, all other velocity fields are set to 0 by default
            // Since we are outside tolerance "max" speed is used
            desired_velocity.twist.angular.z = heading_err > 0 ? heading_speed_ : -heading_speed_;
            desired_velocity_pub_->publish(desired_velocity);
            return;
        }
    }
    else
    {
        // Fixed heading implementation
        // Calculate heading error
        float current_heading = get_yaw_from_quaternion(msg->pose.pose.orientation);
        heading_err = current_waypoint.heading - current_heading;
        
        // Normalize to [-pi, pi]
        if (heading_err > M_PI) heading_err -= 2 * M_PI;
        if (heading_err < -M_PI) heading_err += 2 * M_PI;
    }
    
    // Calculate position errors in world frame
    float x_err = current_waypoint.x - msg->pose.pose.position.x;
    float y_err = current_waypoint.y - msg->pose.pose.position.y;
    
    // Get current heading for coordinate transformation
    float current_heading = get_yaw_from_quaternion(msg->pose.pose.orientation);
    
    // Transform world frame errors to body frame
    float cos_h = cos(current_heading);
    float sin_h = sin(current_heading);
    float x_err_body = x_err * cos_h + y_err * sin_h;
    float y_err_body = -x_err * sin_h + y_err * cos_h;
    
    // Set x, y velocities
    float speed = current_waypoint.desired_speed;
    desired_velocity.twist.linear.x = x_err_body > 0 ? 
        std::min(speed, speed * x_err_body) : 
        std::max(-speed, -speed * std::abs(x_err_body));
        
    desired_velocity.twist.linear.y = y_err_body > 0 ? 
        std::min(speed, speed * y_err_body) : 
        std::max(-speed, -speed * std::abs(y_err_body));
    
    // Handle z-axis control based on altitude mode
    if (current_waypoint.altitude_mode && dvl_valid_) {
        auto time_since_dvl = this->now() - last_dvl_time_;
        if (time_since_dvl.seconds() <= 1.0) {
            // Calculate altitude error (positive when altitude is too low)
            float altitude_err = current_waypoint.target_altitude - current_altitude_;
            
            // Add debug print BEFORE any processing
            RCLCPP_INFO(get_logger(),
                               "Raw altitude data: current=%.2f, target=%.2f, err=%.2f",
                               current_altitude_, current_waypoint.target_altitude, altitude_err);
            
            // Safety check
            if (current_altitude_ < min_altitude_ + altitude_safety_margin_ && altitude_err < 0) {
                RCLCPP_INFO(get_logger(),
                                  "Altitude too low (%.2f m) - restricting downward movement", 
                                  current_altitude_);
                altitude_err = std::max(0.0f, altitude_err);
            }
            
            // Limit error magnitude to prevent excessive speeds
            float max_error = 2.0f;  // Maximum error to consider for control
            altitude_err = std::max(std::min(altitude_err, max_error), -max_error);
            
            // Calculate z velocity from altitude error
            // Positive error → need to go UP → positive z velocity
            desired_velocity.twist.linear.z = -altitude_err * heave_speed_ / max_error;
            
            // Ensure we respect max speed limits
            desired_velocity.twist.linear.z = std::max(std::min(desired_velocity.twist.linear.z, 
                                                 static_cast<double>(heave_speed_)), 
                                        static_cast<double>(-heave_speed_));
            
            RCLCPP_INFO(get_logger(), 
                              "Altitude control: current=%.2f, target=%.2f, err=%.2f, z_vel=%.2f",
                              current_altitude_, current_waypoint.target_altitude, altitude_err, 
                              desired_velocity.twist.linear.z);
        }
        else
        {
            // DVL data too old, fall back to position-based z control
            RCLCPP_INFO(get_logger(),
                               "DVL data too old (%.1f seconds), using position control for z-axis",
                               time_since_dvl.seconds());
            float z_err = current_waypoint.z - msg->pose.pose.position.z;
            desired_velocity.twist.linear.z = z_err > 0 ? 
                std::min(heave_speed_, heave_speed_ * z_err) : 
                std::max(-heave_speed_, -heave_speed_ * z_err);  // Fixed: using max for negative values
        }
    }
    else
    {
        // Standard position-based z control
        float z_err = current_waypoint.z - msg->pose.pose.position.z;
        desired_velocity.twist.linear.z = z_err > 0 ? 
            std::min(heave_speed_, heave_speed_ * z_err) : 
            std::max(-heave_speed_, -heave_speed_ * z_err);  // Fixed: using max for negative values
    }
    
    // Set angular velocity for heading control
    desired_velocity.twist.angular.z = heading_err > 0 ? 
        std::min(heading_speed_, heading_speed_ * heading_err) : 
        std::max(-heading_speed_, -heading_speed_ * heading_err);  // Fixed: using max for negative values
    
    // Add final debug print to see what velocity command is being published
    RCLCPP_INFO(get_logger(), 
                      "Publishing velocity: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f",
                      desired_velocity.twist.linear.x,
                      desired_velocity.twist.linear.y,
                      desired_velocity.twist.linear.z,
                      desired_velocity.twist.angular.z);
    
    // Publish velocity command
    desired_velocity_pub_->publish(desired_velocity);
}

void WaypointController::station_keep(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Get position errors for x and y as before
    float x_err_world = stationary_waypoint_.x - msg->pose.pose.position.x;
    float y_err_world = stationary_waypoint_.y - msg->pose.pose.position.y;
    float heading_err = stationary_waypoint_.heading - get_yaw_from_quaternion(msg->pose.pose.orientation);
    
    // Transform to body frame
    float current_heading = get_yaw_from_quaternion(msg->pose.pose.orientation);
    float cos_h = cos(current_heading);
    float sin_h = sin(current_heading);
    float x_err = x_err_world * cos_h + y_err_world * sin_h;
    float y_err = -x_err_world * sin_h + y_err_world * cos_h;
    
    // Create velocity command
    geometry_msgs::msg::TwistStamped desired_velocity;
    desired_velocity.header.stamp = msg->header.stamp;
    desired_velocity.header.frame_id = msg->header.frame_id;
    
    // Set x,y velocities
    desired_velocity.twist.linear.x = x_err > 0 ?
        min(station_keep_speed_, station_keep_speed_ * x_err) :
        min(-station_keep_speed_, -station_keep_speed_ * x_err);
        
    desired_velocity.twist.linear.y = y_err > 0 ?
        min(station_keep_speed_, station_keep_speed_ * y_err) :
        min(-station_keep_speed_, -station_keep_speed_ * y_err);
    
    // Handle z-axis based on mode
    if (stationary_waypoint_.altitude_mode && dvl_valid_) {
        auto time_since_dvl = this->now() - last_dvl_time_;
        if (time_since_dvl.seconds() <= 1.0) {
            // Calculate altitude error (positive when altitude is too low)
            float altitude_err = stationary_waypoint_.target_altitude - current_altitude_;
            
            // Add debug print BEFORE any processing
            RCLCPP_INFO(get_logger(),
                       "Raw altitude data: current=%.2f, target=%.2f, err=%.2f",
                       current_altitude_, stationary_waypoint_.target_altitude, altitude_err);
            
            // Safety check
            if (current_altitude_ < min_altitude_ + altitude_safety_margin_ && altitude_err < 0) {
                RCLCPP_INFO(get_logger(),
                          "Altitude too low (%.2f m) - restricting downward movement", 
                          current_altitude_);
                altitude_err = std::max(0.0f, altitude_err);
            }
            
            // Limit error magnitude to prevent excessive speeds
            float max_error = 2.0f;  // Maximum error to consider for control
            altitude_err = std::max(std::min(altitude_err, max_error), -max_error);
            
            // Calculate z velocity from altitude error
            desired_velocity.twist.linear.z = -altitude_err * heave_speed_ / max_error;
            
            // Ensure we respect max speed limits
            desired_velocity.twist.linear.z = std::max(std::min(desired_velocity.twist.linear.z, 
                                         static_cast<double>(heave_speed_)), 
                                static_cast<double>(-heave_speed_));
            
            RCLCPP_INFO(get_logger(), 
                      "Station Keep Altitude control: current=%.2f, target=%.2f, err=%.2f, z_vel=%.2f",
                      current_altitude_, stationary_waypoint_.target_altitude, altitude_err, 
                      desired_velocity.twist.linear.z);
        } else {
            // DVL data too old, fall back to position
            float z_err = stationary_waypoint_.z - msg->pose.pose.position.z;
            desired_velocity.twist.linear.z = z_err > 0 ?
                min(heave_speed_, heave_speed_ * z_err) :
                min(-heave_speed_, -heave_speed_ * z_err);
        }
    } else {
        // Use position for z control
        float z_err = stationary_waypoint_.z - msg->pose.pose.position.z;
        desired_velocity.twist.linear.z = z_err > 0 ?
            min(heave_speed_, heave_speed_ * z_err) :
            min(-heave_speed_, -heave_speed_ * z_err);
    }
    
    // Set heading control
    desired_velocity.twist.angular.z = heading_err > 0 ?
        min(heading_speed_, heading_speed_ * heading_err) :
        min(-heading_speed_, -heading_speed_ * heading_err);
    
    // Add final debug print to see what velocity command is being published
    RCLCPP_INFO(get_logger(), 
              "Station Keep publishing velocity: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f",
              desired_velocity.twist.linear.x,
              desired_velocity.twist.linear.y,
              desired_velocity.twist.linear.z,
              desired_velocity.twist.angular.z);
    
    // Publish command
    desired_velocity_pub_->publish(desired_velocity);
}

string WaypointController::waypoints_to_string()
{
    // Get list of waypoints on a string format
    int cnt = 1;
    ostringstream waypoints_string;
    for (const auto &waypoint : waypoints_)
    {
        waypoints_string << "Waypoint " + to_string(cnt) + "(x: " << waypoint.x
                         << ", y: " << waypoint.y
                         << ", z: " << waypoint.z
                         << ", desired_speed: " << waypoint.desired_speed
                         << ", fixed_heading: " << waypoint.fixed_heading
                         << ", heading: " << waypoint.heading << ")\n";
        cnt++;
    }
    return waypoints_string.str();
}

void WaypointController::setup_interfaces()
{
    // Desired Velocity Publisher
    desired_velocity_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(desired_velocity_topic_, 1);

    // Odometry Subscriber
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(odometry_topic_, 1, bind(&WaypointController::controller_callback, this, placeholders::_1));

    // Waypoint Services
    add_waypoint_srv_ = create_service<mundus_mir_msgs::srv::AddWaypoint>(add_waypoint_srv_name_, bind(&WaypointController::add_waypoint, this, placeholders::_1, placeholders::_2));
    insert_waypoint_srv_ = create_service<mundus_mir_msgs::srv::InsertWaypoint>(insert_waypoint_srv_name_, bind(&WaypointController::insert_waypoint, this, placeholders::_1, placeholders::_2));
    remove_waypoint_srv_ = create_service<mundus_mir_msgs::srv::RemoveWaypoint>(remove_waypoint_srv_name_, bind(&WaypointController::remove_waypoint, this, placeholders::_1, placeholders::_2));
    get_waypoints_srv_ = create_service<mundus_mir_msgs::srv::GetWaypoints>(get_waypoints_srv_name_, bind(&WaypointController::get_waypoints, this, placeholders::_1, placeholders::_2));
    clear_waypoints_srv_ = create_service<mundus_mir_msgs::srv::ClearWaypoints>(clear_waypoints_srv_name_, bind(&WaypointController::clear_waypoints, this, placeholders::_1, placeholders::_2));
    go_to_waypoints_srv_ = create_service<mundus_mir_msgs::srv::GoToWaypoints>(go_to_waypoints_srv_name_, bind(&WaypointController::go_to_waypoints, this, placeholders::_1, placeholders::_2));
    get_waypoint_status_srv_ = create_service<mundus_mir_msgs::srv::GetWaypointStatus>(get_waypoint_status_srv_name_, bind(&WaypointController::get_waypoint_status, this, placeholders::_1, placeholders::_2));
    run_waypoint_controller_srv_ = create_service<mundus_mir_msgs::srv::RunWaypointController>(run_waypoint_controller_srv_name_, bind(&WaypointController::run_waypoint_controller, this, placeholders::_1, placeholders::_2));

    // Add DVL subscription
    dvl_sub_ = create_subscription<marine_acoustic_msgs::msg::Dvl>(dvl_topic_, 10, std::bind(&WaypointController::dvl_callback, this, std::placeholders::_1));
}

void WaypointController::get_ros_parameters()
{
    // Declare parameters
    declare_parameter("odometry_topic", "/odom");
    declare_parameter("desired_vel_topic", "/desired_vel");
    declare_parameter("waypoint_service/add", "/blueye/add_waypoint");
    declare_parameter("waypoint_service/insert", "/blueye/insert_waypoint");
    declare_parameter("waypoint_service/remove", "/blueye/remove_waypoint");
    declare_parameter("waypoint_service/get", "/blueye/get_waypoints");
    declare_parameter("waypoint_service/clear", "/blueye/clear_waypoints");
    declare_parameter("waypoint_service/go_to", "/blueye/go_to_waypoints");
    declare_parameter("waypoint_service/get_waypoint_status", "/blueye/get_waypoint_status");
    declare_parameter("waypoint_service/run_waypoint_controller", "/blueye/run_waypoint_controller");
    declare_parameter("heading_tolerance", 0.3);
    declare_parameter("circle_of_acceptance", 0.5);
    declare_parameter("max_heading_rate", 0.1);
    declare_parameter("max_heave_rate", 0.3);
    declare_parameter("max_station_keep_rate", 0.3);
    declare_parameter("dvl_topic", "/blueye/dvl");
    declare_parameter("min_altitude", 0.5);
    declare_parameter("max_altitude", 10.0);
    declare_parameter("altitude_safety_margin", 0.2);

    // Get parameters
    get_parameter("odometry_topic", odometry_topic_);
    get_parameter("desired_vel_topic", desired_velocity_topic_);
    get_parameter("waypoint_service/add", add_waypoint_srv_name_);
    get_parameter("waypoint_service/insert", insert_waypoint_srv_name_);
    get_parameter("waypoint_service/remove", remove_waypoint_srv_name_);
    get_parameter("waypoint_service/get", get_waypoints_srv_name_);
    get_parameter("waypoint_service/clear", clear_waypoints_srv_name_);
    get_parameter("waypoint_service/go_to", go_to_waypoints_srv_name_);
    get_parameter("waypoint_service/get_waypoint_status", get_waypoint_status_srv_name_);
    get_parameter("waypoint_service/run_waypoint_controller", run_waypoint_controller_srv_name_);
    get_parameter("heading_tolerance", heading_tolerance_);
    get_parameter("circle_of_acceptance", circle_of_acceptance_);
    get_parameter("max_heading_rate", heading_speed_);
    get_parameter("max_heave_rate", heave_speed_);
    get_parameter("max_station_keep_rate", station_keep_speed_);
    get_parameter("dvl_topic", dvl_topic_);
    get_parameter("min_altitude", min_altitude_);
    get_parameter("max_altitude", max_altitude_);
    get_parameter("altitude_safety_margin", altitude_safety_margin_);
}

void WaypointController::add_waypoint(
    const std::shared_ptr<mundus_mir_msgs::srv::AddWaypoint::Request> request,
    std::shared_ptr<mundus_mir_msgs::srv::AddWaypoint::Response> response)
{
    // Add a new waypoint to the back of the waypoint list
    Waypoint new_waypoint;
    new_waypoint.x = request->x;
    new_waypoint.y = request->y;
    new_waypoint.z = request->z;
    new_waypoint.fixed_heading = request->fixed_heading;
    new_waypoint.heading = request->heading;
    
    // New fields
    new_waypoint.altitude_mode = request->altitude_mode;
    if (request->altitude_mode) {
        new_waypoint.target_altitude = request->target_altitude;
        RCLCPP_INFO(get_logger(), "Adding waypoint with altitude mode: target=%.2f m", 
                    request->target_altitude);
    }

    if (!(request->desired_velocity >= 0.0)) {
        new_waypoint.desired_speed = request->desired_velocity;
    }

    waypoints_.push_back(new_waypoint);

    // Set response
    response->accepted = true;
}

void WaypointController::insert_waypoint(const std::shared_ptr<mundus_mir_msgs::srv::InsertWaypoint::Request> request,
                                         std::shared_ptr<mundus_mir_msgs::srv::InsertWaypoint::Response> response)
{
    // Add a new waypoint and insert it at desired index
    Waypoint new_waypoint;
    new_waypoint.x = request->x;
    new_waypoint.y = request->y;
    new_waypoint.z = request->z;
    new_waypoint.fixed_heading = request->fixed_heading;
    new_waypoint.heading = request->heading;
    
    // Add altitude mode parameters
    new_waypoint.altitude_mode = request->altitude_mode;
    if (request->altitude_mode) {
        // Set target altitude, validate it's within safe range
        float target_altitude = request->target_altitude;
        if (target_altitude < min_altitude_) {
            RCLCPP_WARN(get_logger(), "Requested target altitude (%.2f m) is below minimum safe altitude. Using %.2f m instead.",
                      target_altitude, min_altitude_);
            target_altitude = min_altitude_;
        } else if (target_altitude > max_altitude_) {
            RCLCPP_WARN(get_logger(), "Requested target altitude (%.2f m) exceeds maximum allowed. Using %.2f m instead.",
                      target_altitude, max_altitude_);
            target_altitude = max_altitude_;
        }
        
        new_waypoint.target_altitude = target_altitude;
        RCLCPP_INFO(get_logger(), "Inserting waypoint with altitude mode: target=%.2f m at index %d", 
                   target_altitude, request->index);
    }

    if (!(request->desired_velocity >= 0.0))
    {
        new_waypoint.desired_speed = request->desired_velocity;
    }
    
    int index = request->index > 0 ? request->index : 0;
    int amount_of_waypoints = waypoints_.size();
    if (index >= amount_of_waypoints)
    {
        waypoints_.push_back(new_waypoint);
    }
    else
    {
        waypoints_.insert(waypoints_.begin() + index, new_waypoint);
    }

    // Set response
    response->accepted = true;
}

void WaypointController::remove_waypoint(const std::shared_ptr<mundus_mir_msgs::srv::RemoveWaypoint::Request> request,
                                         std::shared_ptr<mundus_mir_msgs::srv::RemoveWaypoint::Response> response)
{
    // Remove waypoint at index
    int size_of_waypoints = waypoints_.size();
    if (size_of_waypoints < request->index || request->index < 0)
    {
        response->accepted = false;
        response->error_code = "Index out of bounds.";
        return;
    }
    waypoints_.erase(waypoints_.begin() + request->index);

    // Set response
    response->accepted = true;
}

void WaypointController::get_waypoints(const std::shared_ptr<mundus_mir_msgs::srv::GetWaypoints::Request> request,
                                       std::shared_ptr<mundus_mir_msgs::srv::GetWaypoints::Response> response)
{
    // Handle request to avoid warinings
    (void) request;
    // Set response
    response->accepted = true;
    response->waypoints = waypoints_to_string();
}

void WaypointController::clear_waypoints(const std::shared_ptr<mundus_mir_msgs::srv::ClearWaypoints::Request> request,
                                         std::shared_ptr<mundus_mir_msgs::srv::ClearWaypoints::Response> response)
{
    // Handle request to avoid warnings
    (void) request;
    // Clear all waypoints
    waypoints_.clear();

    // Set go_to_waypoint variable to false as we no longer have any waypoints
    go_to_waypoint_ = false;
    stationary_waypoint_set_ = false;

    // Set response
    response->accepted = true;
}

void WaypointController::go_to_waypoints(const std::shared_ptr<mundus_mir_msgs::srv::GoToWaypoints::Request> request,
                                         std::shared_ptr<mundus_mir_msgs::srv::GoToWaypoints::Response> response)
{
    // If no waypoints then we cant go to any
    if (waypoints_.size() == 0)
    {
        response->accepted = false;
        response->status_code = "No waypoints to go to.";
        return;
    }
    go_to_waypoint_ = request->run;
    stationary_waypoint_set_ = false;

    // Set response
    response->accepted = true;
}

void WaypointController::get_waypoint_status(const std::shared_ptr<mundus_mir_msgs::srv::GetWaypointStatus::Request> request,
                                             std::shared_ptr<mundus_mir_msgs::srv::GetWaypointStatus::Response> response)
{
    // Handle request to avoid warnings
    (void) request;
    
    // Set response
    response->accepted = true;

    // Build status message
    ostringstream status;
    status << "Controller running: " << run_controller_ << "\n";
    
    if (run_controller_) {
        // Add waypoint information
        status << "Amount of waypoints: " << waypoints_.size() << "\n";
        status << "Currently going to waypoint: " << go_to_waypoint_ << "\n";
        
        // Add DVL information
        status << "DVL valid: " << dvl_valid_;
        if (dvl_valid_) {
            auto time_since_dvl = this->now() - last_dvl_time_;
            status << " (last update: " << std::fixed << std::setprecision(1) 
                   << time_since_dvl.seconds() << "s ago)";
            status << ", Current altitude: " << std::setprecision(2) << current_altitude_ << "m";
        }
        status << "\n";
        
        // Add detailed waypoint information
        if (go_to_waypoint_ && !waypoints_.empty()) {
            const auto& wp = waypoints_.front();
            status << "Current waypoint: x=" << wp.x << ", y=" << wp.y << ", z=" << wp.z;
            status << ", speed=" << wp.desired_speed;
            status << ", fixed_heading=" << wp.fixed_heading;
            if (wp.fixed_heading) {
                status << ", heading=" << wp.heading;
            }
            
            // Add altitude mode information
            if (wp.altitude_mode) {
                status << ", altitude_mode=true";
                status << ", target_altitude=" << wp.target_altitude << "m";
            } else {
                status << ", altitude_mode=false";
            }
            status << "\n";
            
            // Add distance to waypoint if odometry is available
            float distance = 0.0;
            try {
                auto odom = std::make_shared<nav_msgs::msg::Odometry>();
                // TODO: This is not ideal as it blocks, but it's simple for status reporting
                auto sub = this->create_subscription<nav_msgs::msg::Odometry>(
                    odometry_topic_, 1, 
                    [&odom](const nav_msgs::msg::Odometry::SharedPtr msg) { *odom = *msg; });
                
                auto start_time = this->now();
                while ((this->now() - start_time).seconds() < 0.1) {
                    rclcpp::spin_some(this->get_node_base_interface());
                }
                
                distance = calculate_distance(wp, odom);
                status << "Distance to waypoint: " << std::setprecision(2) << distance << "m\n";
            } catch (const std::exception& e) {
                // If we can't get the distance, just skip this part
            }
        } else if (stationary_waypoint_set_) {
            // Add station keeping information
            status << "Station Keep On Position: x=" << stationary_waypoint_.x 
                   << ", y=" << stationary_waypoint_.y << ", z=" << stationary_waypoint_.z;
            status << ", fixed_heading=" << stationary_waypoint_.fixed_heading;
            if (stationary_waypoint_.fixed_heading) {
                status << ", heading=" << stationary_waypoint_.heading;
            }
            
            // Add altitude mode information for station keeping
            if (stationary_waypoint_.altitude_mode) {
                status << ", altitude_mode=true";
                status << ", target_altitude=" << stationary_waypoint_.target_altitude << "m";
            } else {
                status << ", altitude_mode=false";
            }
            status << "\n";
        }
    }
    
    response->status_code = status.str();
}

void WaypointController::run_waypoint_controller(const std::shared_ptr<mundus_mir_msgs::srv::RunWaypointController::Request> request,
                                                 std::shared_ptr<mundus_mir_msgs::srv::RunWaypointController::Response> response)
{
    // Set run_controller variable
    run_controller_ = request->run;

    // Set response
    response->accepted = true;
}