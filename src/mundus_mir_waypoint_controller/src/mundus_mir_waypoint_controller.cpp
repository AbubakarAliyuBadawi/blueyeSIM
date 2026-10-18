#include "mundus_mir_waypoint_controller/mundus_mir_waypoint_controller.hpp"

float clamp_command(float value, float limit)
{
    return std::clamp(value, -limit, limit);
}

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
                // Don't override the waypoint's heading if fixed_heading is true
                if (!waypoints_.front().fixed_heading) {
                    stationary_waypoint_.heading = get_yaw_from_quaternion(msg->pose.pose.orientation);
                }
                stationary_waypoint_.fixed_heading = true;
                go_to_waypoint_ = false;
                stationary_waypoint_set_ = true;
            }
            // If we are close to the waypoint, remove it and reset alignment for the next one
            waypoints_.erase(waypoints_.begin());
            reset_alignment();
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

void WaypointController::go_to_waypoint(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Check heading of the robot if it is not supposed to
    if (!waypoints_.front().fixed_heading)
    {
        // If heading is outside tolerance fix it before we start going toward the next waypoint
        // Desired heading is straight towards the waypoint
        float desired_heading = atan2(waypoints_.front().y - msg->pose.pose.position.y, waypoints_.front().x - msg->pose.pose.position.x);
        float heading_err = desired_heading - get_yaw_from_quaternion(msg->pose.pose.orientation);

        // Check for minimum signed angle
        if (heading_err > M_PI)
        {
            heading_err -= 2 * M_PI;
        }
        else if (heading_err < -M_PI)
        {
            heading_err += 2 * M_PI;
        }

        // While surging the waypoint drifted off the nose (the heading cannot be corrected well while surging):
        // stop and align again, but only if the current heading would pass outside the circle of acceptance.
        // Close to the waypoint the heading error grows quickly even for a small offset that still gets there.
        float distance = calculate_distance(waypoints_.front(), msg);
        bool would_miss = std::abs(heading_err) > M_PI / 2 ||
                          distance * std::sin(std::abs(heading_err)) > 0.8 * circle_of_acceptance_;
        if (heading_aligned_ && std::abs(heading_err) > realign_heading_error_ && would_miss)
        {
            reset_alignment();
            RCLCPP_INFO(get_logger(), "Heading error %.0f deg while surging, %.2f m from the waypoint: stopping to realign",
                        heading_err * 180.0 / M_PI, distance);
        }

        // Rotate in place until the heading is settled on the waypoint, then surge and correct simultaneously
        if (!heading_aligned_)
        {
            geometry_msgs::msg::TwistStamped desired_velocity;
            desired_velocity.header.stamp = msg->header.stamp;
            desired_velocity.header.frame_id = "body_frame";
            if (!align_heading(heading_err, msg->twist.twist.angular.z, desired_velocity))
            {
                desired_velocity_pub_->publish(desired_velocity);
                return;
            }
            heading_aligned_ = true;
            RCLCPP_INFO(get_logger(), "Heading aligned (error %.1f deg), surging %.2f m to the waypoint",
                        heading_err * 180.0 / M_PI, calculate_distance(waypoints_.front(), msg));
        }

        // Surge toward waypoint while continuously correcting heading
        // Stonefish Blueye convention: positive commanded heave moves the vehicle up,
        // which decreases odometry z. Use current - target so z above target commands down
        // and z below target commands up.
        float z_err = msg->pose.pose.position.z - waypoints_.front().z;
        geometry_msgs::msg::TwistStamped desired_velocity;
        desired_velocity.header.stamp = msg->header.stamp;
        desired_velocity.header.frame_id = "body_frame";
        desired_velocity.twist.linear.x = waypoints_.front().desired_speed;
        // TODO - Add a scaling system that takes size of error into consideration more than just linear scaling
        desired_velocity.twist.linear.z = clamp_command(heave_speed_ * z_err, heave_speed_);
        desired_velocity.twist.angular.z = clamp_command(heading_speed_ * heading_err, heading_speed_);
        desired_velocity_pub_->publish(desired_velocity);
    }

    // TODO Publish velocity when heading should be fixed
    else
    {
        // Fixed heading implementation
        // Calculate position errors in world frame
        float x_err = waypoints_.front().x - msg->pose.pose.position.x;
        float y_err = waypoints_.front().y - msg->pose.pose.position.y;
        // Stonefish Blueye convention: positive commanded heave moves the vehicle up,
        // which decreases odometry z.
        float z_err = msg->pose.pose.position.z - waypoints_.front().z;
        
        // Get current heading
        float current_heading = get_yaw_from_quaternion(msg->pose.pose.orientation);
        
        // Calculate heading error
        float heading_err = waypoints_.front().heading - current_heading;
        if (heading_err > M_PI) heading_err -= 2 * M_PI;
        if (heading_err < -M_PI) heading_err += 2 * M_PI;

        // Transform world frame errors to body frame
        float cos_h = cos(current_heading);
        float sin_h = sin(current_heading);
        float x_err_body = x_err * cos_h + y_err * sin_h;
        float y_err_body = -x_err * sin_h + y_err * cos_h;

        // Create velocity command
        geometry_msgs::msg::TwistStamped desired_velocity;
        desired_velocity.header.stamp = msg->header.stamp;
        desired_velocity.header.frame_id = msg->header.frame_id;

        // Scale velocities based on errors
        float speed = waypoints_.front().desired_speed;
        desired_velocity.twist.linear.x = clamp_command(speed * x_err_body, speed);
        desired_velocity.twist.linear.y = clamp_command(-speed * y_err_body, speed);
        desired_velocity.twist.linear.z = clamp_command(heave_speed_ * z_err, heave_speed_);
        
        // Maintain fixed heading
        desired_velocity.twist.angular.z = clamp_command(heading_speed_ * heading_err, heading_speed_);

        RCLCPP_DEBUG(get_logger(), "Fixed heading control - errors (body frame): x=%.2f, y=%.2f, z=%.2f, h=%.2f",
                    x_err_body, y_err_body, z_err, heading_err);

        desired_velocity_pub_->publish(desired_velocity);

    }
}

bool WaypointController::align_heading(float heading_err, float yaw_rate,
                                       geometry_msgs::msg::TwistStamped &desired_velocity)
{
    // Turn: proportional to the error, at least align_min_heading_rate (above the velocity controller's heading-hold
    // deadband). Stop commanding when within heading_tolerance, or earlier when the vehicle, already turning towards
    // the target, would coast the rest of the way in (|yaw rate| * align_coast_time), so it does not overshoot.
    if (!align_waiting_)
    {
        bool coasting_in = yaw_rate * heading_err > 0 && std::abs(heading_err) < std::abs(yaw_rate) * align_coast_time_;
        if (std::abs(heading_err) > heading_tolerance_ && !coasting_in)
        {
            desired_velocity.twist.angular.z = std::copysign(
                std::clamp(align_heading_gain_ * std::abs(heading_err), align_min_heading_rate_, heading_speed_), heading_err);
            return false;
        }
        align_waiting_ = true;
        align_still_ = false;
    }

    // Wait with zero command (the velocity controller's heading hold brakes the turn) until the yaw has been at rest
    // for align_still_time, then check where the heading ended up: within align_heading_tolerance it is aligned,
    // otherwise turn again from rest. Waiting for rest makes this robust to delays between odometry and command.
    const float still_yaw_rate = 0.005;  // rad/s
    if (std::abs(yaw_rate) > still_yaw_rate)
    {
        align_still_ = false;
        return false;
    }
    if (!align_still_)
    {
        align_still_ = true;
        align_still_since_ = now();
        return false;
    }
    if ((now() - align_still_since_).seconds() < align_still_time_)
    {
        return false;
    }
    if (std::abs(heading_err) < align_heading_tolerance_)
    {
        return true;
    }
    align_waiting_ = false;
    return false;
}

void WaypointController::reset_alignment()
{
    heading_aligned_ = false;
    align_waiting_ = false;
    align_still_ = false;
}

void WaypointController::station_keep(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Check all degrees of freedom compared to the stationary waypoint and run correction
    float x_err_world = stationary_waypoint_.x - msg->pose.pose.position.x;
    float y_err_world = stationary_waypoint_.y - msg->pose.pose.position.y;
    // Stonefish Blueye convention: positive commanded heave moves the vehicle up,
    // which decreases odometry z.
    float z_err = msg->pose.pose.position.z - stationary_waypoint_.z;
    float heading_err = stationary_waypoint_.heading - get_yaw_from_quaternion(msg->pose.pose.orientation);
    if (heading_err > M_PI) heading_err -= 2 * M_PI;
    if (heading_err < -M_PI) heading_err += 2 * M_PI;

    // Rotate x_err and y_err by taking current heading into consideration
    float x_err = x_err_world * cos(get_yaw_from_quaternion(msg->pose.pose.orientation)) + y_err_world * sin(get_yaw_from_quaternion(msg->pose.pose.orientation));
    float y_err = -x_err_world * sin(get_yaw_from_quaternion(msg->pose.pose.orientation)) + y_err_world * cos(get_yaw_from_quaternion(msg->pose.pose.orientation));

    // Station keep on the stationary waypoint, TODO Add scaling based on error more than just linear scaling
    geometry_msgs::msg::TwistStamped desired_velocity;
    desired_velocity.header.stamp = msg->header.stamp;
    desired_velocity.header.frame_id = "body_frame";
    desired_velocity.twist.linear.x = clamp_command(station_keep_speed_ * x_err, station_keep_speed_);
    desired_velocity.twist.linear.y = clamp_command(-station_keep_speed_ * y_err, station_keep_speed_);
    desired_velocity.twist.linear.z = clamp_command(heave_speed_ * z_err, heave_speed_);
    // Turn at least station_keep_min_heading_rate (above the velocity controller's heading-hold deadband) until
    // the heading is within station_keep_heading_tolerance; smaller commands would let the heading hold lock the
    // vehicle on its current heading, short of the target.
    float yaw_rate = clamp_command(heading_speed_ * heading_err, heading_speed_);
    if (std::abs(heading_err) > station_keep_heading_tolerance_ && std::abs(yaw_rate) < station_keep_min_heading_rate_)
    {
        yaw_rate = std::copysign(station_keep_min_heading_rate_, heading_err);
    }
    desired_velocity.twist.angular.z = yaw_rate;
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
    declare_parameter("station_keep_min_heading_rate", 0.06);
    declare_parameter("station_keep_heading_tolerance", 0.035);
    declare_parameter("realign_heading_error", 0.26);
    declare_parameter("align_heading_tolerance", 0.026);
    declare_parameter("align_heading_gain", 0.5);
    declare_parameter("align_min_heading_rate", 0.06);
    declare_parameter("align_coast_time", 1.0);
    declare_parameter("align_still_time", 1.0);

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
    get_parameter("station_keep_min_heading_rate", station_keep_min_heading_rate_);
    get_parameter("station_keep_heading_tolerance", station_keep_heading_tolerance_);
    get_parameter("realign_heading_error", realign_heading_error_);
    get_parameter("align_heading_tolerance", align_heading_tolerance_);
    get_parameter("align_heading_gain", align_heading_gain_);
    get_parameter("align_min_heading_rate", align_min_heading_rate_);
    get_parameter("align_coast_time", align_coast_time_);
    get_parameter("align_still_time", align_still_time_);
}

void WaypointController::add_waypoint(const std::shared_ptr<mundus_mir_msgs::srv::AddWaypoint::Request> request,
                                      std::shared_ptr<mundus_mir_msgs::srv::AddWaypoint::Response> response)
{
    // Add a new waypoint to the back of the waypoint list
    Waypoint new_waypoint;
    new_waypoint.x = request->x;
    new_waypoint.y = request->y;
    new_waypoint.z = request->z;
    new_waypoint.fixed_heading = request->fixed_heading;
    new_waypoint.heading = request->heading;
    
    // Fix: Always set the desired_speed to the requested velocity
    new_waypoint.desired_speed = request->desired_velocity;
    
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
    
    // Fix: Always set the desired_speed to the requested velocity
    new_waypoint.desired_speed = request->desired_velocity;
    
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
    reset_alignment();

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

    // TODO Figure out how we want to have this message written
    ostringstream status;
    status << "Controller running: " << run_controller_ << "\n";
    if (run_controller_) {
        status << "Amount of waypoints: " << waypoints_.size() << "\n";
        status << "Currently going to waypoint: " << go_to_waypoint_ << "\n";
        if (go_to_waypoint_)
        {
            status << "Current waypoint: " << waypoints_.front().x << ", " << waypoints_.front().y << ", " << waypoints_.front().z << "\n";
        }
        else {
            status << "Station Keep On Position: " << stationary_waypoint_.x << ", " << stationary_waypoint_.y << ", " << stationary_waypoint_.z << "\n";
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
