#include "blueye_joystick_cpp/blueye_joystick_cpp.hpp"

JoystickController::JoystickController()
  : Node("mundus_mir_blueye_joystick_cpp") {
// Fetch parameters
fetch_ros_parameters();

// Initialize publisher and subscriber

if (!velocity_control_) {
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
      cmd_topic_, 10);
  RCLCPP_INFO(get_logger(), "Publishing to %s", cmd_topic_.c_str());
} else {
  vel_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(vel_topic_, 10);
  RCLCPP_INFO(get_logger(), "Publishing to %s", vel_topic_.c_str());
}

// Lights publisher
lights_pub_ =
    this->create_publisher<std_msgs::msg::Bool>("/blueye/lights", 1);

// Tilt publisher
tilt_pub_ =
    this->create_publisher<std_msgs::msg::Float64>("/blueye/camera_tilt", 1);

    // Control State Clients
    depth_hold_srv_ =
        this->create_client<std_srvs::srv::SetBool>(depth_hold_service_);
heading_hold_srv_ =
    this->create_client<std_srvs::srv::SetBool>(heading_hold_service_);
docking_mode_srv_ =
    this->create_client<std_srvs::srv::SetBool>(docking_mode_service_);
inspection_clock_srv =
    this->create_client<std_srvs::srv::SetBool>(inspection_clock_service_);
inspection_counter_clock_srv =
    this->create_client<std_srvs::srv::SetBool>(inspection_counter_clock_service_);
stationkeep_srv_ =
    this->create_client<std_srvs::srv::SetBool>(stationkeep_service_);
depth_hold_srv_viz =
    this->create_client<std_srvs::srv::SetBool>("/blueye/depth_hold/viz");
heading_hold_srv_viz =
    this->create_client<std_srvs::srv::SetBool>("/blueye/heading_hold/viz");
docking_mode_srv_viz =
    this->create_client<std_srvs::srv::SetBool>("/blueye/dock_mode/viz");
inspection_counter_clock_srv_viz =
    this->create_client<std_srvs::srv::SetBool>("/blueye/inspection_counter_clock/viz");
inspection_clock_srv_viz =
    this->create_client<std_srvs::srv::SetBool>("/blueye/inspection_clock/viz");
align_aruco_srv_ = 
    this->create_client<std_srvs::srv::Trigger>("/blueye/align_aruco");

// Subscribe to the joystick topic
joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    joy_topic_, 1,
    std::bind(&JoystickController::joystick_callback, this,
              std::placeholders::_1));

// Command Timer
timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&JoystickController::send_commands, this));
}

JoystickController::~JoystickController() {
// Do nothing
}

void JoystickController::send_commands() {
// Prepare a twist message
auto wrench_msg = geometry_msgs::msg::WrenchStamped();
auto header = std_msgs::msg::Header();
header.frame_id = "body_frame";
header.stamp = now();
auto wrench = geometry_msgs::msg::Wrench();
wrench.force.x = right_y;
wrench.force.y = right_x;
wrench.force.z = left_y;
wrench.torque.x = 0.0;
wrench.torque.y = 0.0;
wrench.torque.z = left_x;
wrench_msg.header = header;
wrench_msg.wrench = wrench;

// Publish message
if (joystick_active_) {
  cmd_pub_->publish(wrench_msg);
}
}

void JoystickController::fetch_ros_parameters() {
// Fetch parameters
this->declare_parameter("quadratic_mapping", true);
this->declare_parameter("max_thrust_surge", 1.0);
this->declare_parameter("max_thrust_sway", 1.0);
this->declare_parameter("max_thrust_heave", 1.0);
this->declare_parameter("max_thrust_yaw", 1.0);
this->declare_parameter("joy_topic", "/joy");
// this->declare_parameter("cmd_topic", "/blueye/force_cmd");
this->declare_parameter("cmd_topic", "/blueye/commands");

this->declare_parameter("right_joystick_x_axis", 3);
this->declare_parameter("right_joystick_y_axis", 4);
this->declare_parameter("left_joystick_x_axis", 1);
this->declare_parameter("left_joystick_y_axis", 0);
this->declare_parameter("x_button", 0);
this->declare_parameter("circle_button", 1);
this->declare_parameter("square_button", 2);
this->declare_parameter("triangle_button", 3);
this->declare_parameter("dpad_horizontal_axis", 6);
this->declare_parameter("dpad_vertical_axis", 7);

this->declare_parameter("velocity_control", false);
this->declare_parameter("max_velocity_x", 0.5);
this->declare_parameter("max_velocity_y", 0.5);
this->declare_parameter("max_velocity_z", 0.5);
this->declare_parameter("max_angular_velocity_yaw", 1.0); // Radians/s
this->declare_parameter("velocity_topic", "/blueye/velocity_ref");
this->declare_parameter("depth_hold_service", "blueye/depth_hold");
this->declare_parameter("heading_hold_service", "blueye/heading_hold");
this->declare_parameter("docking_mode_service", "blueye/docking_mode");
this->declare_parameter("inspection_clock_service", "blueye/inspection_clock");
this->declare_parameter("inspection_counter_clock_service", "blueye/inspection_counter_clock");
this->declare_parameter("stationkeep_service", "blueye/stationkeep");

this->declare_parameter("deadzone", 0.1);

this->get_parameter("quadratic_mapping", quadratic_mapping_);
this->get_parameter("max_thrust_surge", max_thrust_surge_);
this->get_parameter("max_thrust_sway", max_thrust_sway_);
this->get_parameter("max_thrust_heave", max_thrust_heave_);
this->get_parameter("max_thrust_yaw", max_thrust_yaw_);
this->get_parameter("joy_topic", joy_topic_);
this->get_parameter("cmd_topic", cmd_topic_);
this->get_parameter("depth_hold_service", depth_hold_service_);
this->get_parameter("heading_hold_service", heading_hold_service_);
this->get_parameter("docking_mode_service", docking_mode_service_);
this->get_parameter("inspection_clock_service", inspection_clock_service_);
this->get_parameter("inspection_counter_clock_service", inspection_counter_clock_service_);

this->get_parameter("stationkeep_service", stationkeep_service_);

this->get_parameter("right_joystick_x_axis", right_joystick_x_axis_);
this->get_parameter("right_joystick_y_axis", right_joystick_y_axis_);
this->get_parameter("left_joystick_x_axis", left_joystick_x_axis_);
this->get_parameter("left_joystick_y_axis", left_joystick_y_axis_);
this->get_parameter("x_button", x_button_);
this->get_parameter("circle_button", circle_button_);
this->get_parameter("square_button", square_button_);
this->get_parameter("triangle_button", triangle_button_);
this->get_parameter("dpad_horizontal_axis", dpad_horizontal_axis_);
this->get_parameter("dpad_vertical_axis", dpad_vertical_axis_);


this->get_parameter("velocity_control", velocity_control_);
this->get_parameter("max_velocity_x", max_velocity_x_);
this->get_parameter("max_velocity_y", max_velocity_y_);
this->get_parameter("max_velocity_z", max_velocity_z_);
this->get_parameter("max_angular_velocity_yaw", max_angular_velocity_yaw_);
this->get_parameter("velocity_topic", vel_topic_);

this->get_parameter("deadzone", deadzone_);
}

void JoystickController::joystick_callback(
  const sensor_msgs::msg::Joy::SharedPtr msg) {

// Check buttons to see if we should go to another control state
handle_buttons(msg->buttons, msg->axes);

// Do action based on current state
if (joystick_active_ && !ctrl_state_3_) {

  double desired_surge, desired_sway, desired_heave, desired_yaw;

  if (!velocity_control_) {

    // Calculate desired thrust values
    if (quadratic_mapping_) {
      desired_surge = copysign(1.0, msg->axes[right_joystick_y_axis_]) *
                      pow(msg->axes[right_joystick_y_axis_], 2) *
                      max_thrust_surge_;
      desired_sway = copysign(1.0, msg->axes[right_joystick_x_axis_]) *
                      pow(msg->axes[right_joystick_x_axis_], 2) *
                      max_thrust_sway_;
      desired_heave = copysign(1.0, msg->axes[left_joystick_y_axis_]) *
                      pow(msg->axes[left_joystick_y_axis_], 2) *
                      max_thrust_heave_;
      desired_yaw = copysign(1.0, msg->axes[left_joystick_x_axis_]) *
                    pow(msg->axes[left_joystick_x_axis_], 2) *
                    max_thrust_yaw_;
    } else {
      desired_surge = msg->axes[right_joystick_y_axis_] * max_thrust_surge_;
      desired_sway = msg->axes[right_joystick_x_axis_] * max_thrust_sway_;
      desired_heave = msg->axes[left_joystick_y_axis_] * max_thrust_heave_;
      desired_yaw = msg->axes[left_joystick_x_axis_] * max_thrust_yaw_;
    }

    // Deadzone
    desired_surge = abs(desired_surge) < deadzone_ ? 0.0 : desired_surge;
    desired_sway = abs(desired_sway) < deadzone_ ? 0.0 : desired_sway;
    desired_heave = abs(desired_heave) < deadzone_ ? 0.0 : desired_heave;
    desired_yaw = abs(desired_yaw) < deadzone_ ? 0.0 : desired_yaw;

    // Works with Gabriele's PS4 controller:
    right_y = desired_surge;
    right_x = -desired_sway;
    left_y = -desired_yaw;
    left_x = -desired_heave;

  }

  // Velocity Control
  else if (velocity_control_) {

    // Calculate desired thrust values
    if (quadratic_mapping_) {
      desired_surge = copysign(1.0, msg->axes[right_joystick_y_axis_]) *
                      pow(msg->axes[right_joystick_y_axis_], 2) *
                      max_velocity_x_;
      desired_sway = -copysign(1.0, msg->axes[right_joystick_x_axis_]) *
                      pow(msg->axes[right_joystick_x_axis_], 2) *
                      max_velocity_y_;
      desired_heave = copysign(1.0, msg->axes[left_joystick_y_axis_]) *
                      pow(msg->axes[left_joystick_y_axis_], 2) *
                      max_velocity_z_;
      desired_yaw = -copysign(1.0, msg->axes[left_joystick_x_axis_]) *
                    pow(msg->axes[left_joystick_x_axis_], 2) *
                    max_angular_velocity_yaw_;
    } else {
      desired_surge = msg->axes[right_joystick_y_axis_] * max_velocity_x_;
      desired_sway = -msg->axes[right_joystick_x_axis_] * max_velocity_y_;
      desired_heave = msg->axes[left_joystick_y_axis_] * max_velocity_z_;
      desired_yaw =
          -msg->axes[left_joystick_x_axis_] * max_angular_velocity_yaw_;
    }

    // Deadzone
    desired_surge = abs(desired_surge) < deadzone_ ? 0.0 : desired_surge;
    desired_sway = abs(desired_sway) < deadzone_ ? 0.0 : desired_sway;
    desired_heave = abs(desired_heave) < deadzone_ ? 0.0 : desired_heave;
    desired_yaw = abs(desired_yaw) < deadzone_ ? 0.0 : desired_yaw;

    // Prepare a twist message
    auto odometry_msg = nav_msgs::msg::Odometry();
    auto header = std_msgs::msg::Header();
    header.frame_id = "body_frame";
    header.stamp = now();
    odometry_msg.header = header;

    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = desired_surge;
    twist.linear.y = desired_sway;
    twist.linear.z = desired_heave;
    twist.angular.z = desired_yaw;
    odometry_msg.twist.twist = twist;

    // Publish message
    vel_pub_->publish(odometry_msg);
  }

}
// If we are in control state 1
else if (ctrl_state_1_) {

}
// If we are in control state 2
else if (ctrl_state_2_) {
  // Do a service call or something
}
// If we are in control state 3
else if (ctrl_state_3_) {
  // Do a service call or something
}
}

void JoystickController::handle_buttons(const vector<int32_t> &buttons,
                                        const std::vector<float> &axes) {

// Go through all buttons and check if they are pressed
if (buttons[x_button_] == 1 && !x_is_pressed_) {
  x_is_pressed_ = true;
  joystick_active_ = !joystick_active_;
  if (joystick_active_) {
    RCLCPP_INFO(get_logger(), "Joystick ACTIVATED — publishing commands");
  } else {
    RCLCPP_INFO(get_logger(), "Joystick DEACTIVATED — silent");
  }
} else if (!buttons[x_button_] && x_is_pressed_) {
  x_is_pressed_ = false;
}

// Depth Hold
if (buttons[triangle_button_] == 1 && !triangle_is_pressed_) {
  triangle_is_pressed_ = true;
  ctrl_state_1_ = !ctrl_state_1_;
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  if (ctrl_state_1_) {
    ctrl_state_4_ = false;
    ctrl_state_5_ = false;
    ctrl_state_6_ = false;
    RCLCPP_INFO(get_logger(), "Depth Hold is now active");
    req->data = true;
    auto req_hold = std::make_shared<std_srvs::srv::SetBool::Request>();
    req_hold->data = false;
    // Activate 
    auto resp_hold = depth_hold_srv_->async_send_request(req);
    auto resp_hold_viz = depth_hold_srv_viz->async_send_request(req);
    // Deactivate
    auto resp_dock = docking_mode_srv_->async_send_request(req_hold);
    auto resp_dock_viz = docking_mode_srv_viz->async_send_request(req_hold);
    auto resp_clock = inspection_clock_srv->async_send_request(req_hold);
    auto resp_clock_viz = inspection_clock_srv_viz->async_send_request(req_hold);
    auto resp_counter_clock = inspection_counter_clock_srv->async_send_request(req_hold);
    auto resp_counter_clock_viz = inspection_counter_clock_srv_viz->async_send_request(req_hold);
  } else {
    RCLCPP_INFO(get_logger(), "Depth Hold is now inactive");
    req->data = false;
    auto resp_hold = depth_hold_srv_->async_send_request(req);
    auto resp_hold_viz = depth_hold_srv_viz->async_send_request(req);
  }
} else if (!buttons[triangle_button_] && triangle_is_pressed_) {
  triangle_is_pressed_ = false;
}

// Heading Hold
if (buttons[circle_button_] == 1 && !circle_is_pressed_) {
  circle_is_pressed_ = true;
  ctrl_state_2_ = !ctrl_state_2_;
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  if (ctrl_state_2_) {
    ctrl_state_4_ = false;
    ctrl_state_5_ = false;
    ctrl_state_6_ = false;
    RCLCPP_INFO(get_logger(), "Heading hold is now active");
    req->data = true;
    auto req_hold = std::make_shared<std_srvs::srv::SetBool::Request>();
    req_hold->data = false;
    // Activate
    auto resp_hold2 = heading_hold_srv_->async_send_request(req);
    auto resp_hold_viz_2 = heading_hold_srv_viz->async_send_request(req);

    // Deactivate
    auto resp_dock = docking_mode_srv_->async_send_request(req_hold);
    auto resp_dock_viz = docking_mode_srv_viz->async_send_request(req_hold);
    auto resp_clock = inspection_clock_srv->async_send_request(req_hold);
    auto resp_clock_viz = inspection_clock_srv_viz->async_send_request(req_hold);
    auto resp_counter_clock = inspection_counter_clock_srv->async_send_request(req_hold);
    auto resp_counter_clock_viz = inspection_counter_clock_srv_viz->async_send_request(req_hold);
  } else {
    RCLCPP_INFO(get_logger(), "Heading hold is now inactive");
    req->data = false;
    auto resp_hold_2 = heading_hold_srv_->async_send_request(req);
    auto resp_hold_viz_2 = heading_hold_srv_viz->async_send_request(req);
  }
} else if (!buttons[circle_button_] && circle_is_pressed_) {
  circle_is_pressed_ = false;
}

// Docking mode
if (buttons[square_button_] == 1 && !square_is_pressed_) {
  square_is_pressed_ = true;
  ctrl_state_3_ = !ctrl_state_3_;
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  if (ctrl_state_3_) {
    joystick_active_ = false;
    ctrl_state_2_ = false;
    ctrl_state_1_ = false;
    ctrl_state_4_ = false;
    ctrl_state_5_ = false;
    ctrl_state_6_ = false;

    RCLCPP_INFO(get_logger(), "Docking mode is now active");
    req->data = true;
    auto req_hold = std::make_shared<std_srvs::srv::SetBool::Request>();
    req_hold->data = false;
    auto resp_dock = docking_mode_srv_->async_send_request(req);
    auto resp_dock_viz = docking_mode_srv_viz->async_send_request(req);
    auto resp_hold = depth_hold_srv_->async_send_request(req_hold);
    auto resp_hold_viz = depth_hold_srv_viz->async_send_request(req_hold);
    auto resp_hold_2 = heading_hold_srv_->async_send_request(req_hold);
    auto resp_hold_viz_2 = heading_hold_srv_viz->async_send_request(req_hold);
    auto resp_clock = inspection_clock_srv->async_send_request(req_hold);
    auto resp_clock_viz = inspection_clock_srv_viz->async_send_request(req_hold);
    auto resp_counter_clock = inspection_counter_clock_srv->async_send_request(req_hold);
    auto resp_counter_clock_viz = inspection_counter_clock_srv_viz->async_send_request(req_hold);
  } else {
    RCLCPP_INFO(get_logger(), "Docking mode is now inactive");
    joystick_active_ = true;
    req->data = false;
      auto resp = docking_mode_srv_->async_send_request(req);
      auto resp_viz = docking_mode_srv_viz->async_send_request(req);
    }
  } else if (!buttons[square_button_] && square_is_pressed_) {
    square_is_pressed_ = false;
  }

  // --- Inspection Clockwise (D-Pad RIGHT) ---
  if (axes[dpad_horizontal_axis_] == -1.0 && !dpad_right_pressed_) {
    dpad_right_pressed_ = true;
    ctrl_state_5_ = !ctrl_state_5_;   // add a new ctrl_state_5_ in your class
    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();

    if (ctrl_state_5_) {
      joystick_active_ = false;
      ctrl_state_1_ = false;
      ctrl_state_2_ = false;
      ctrl_state_3_ = false;
      ctrl_state_4_ = false;
      ctrl_state_6_ = false;
      RCLCPP_INFO(get_logger(), "Inspection (clockwise) mode is now active");
      req->data = true;
      auto req_hold = std::make_shared<std_srvs::srv::SetBool::Request>();
      req_hold->data = false;
      // Activate
      auto resp_clock = inspection_clock_srv->async_send_request(req);
      auto resp_clock_viz = inspection_clock_srv_viz->async_send_request(req);
      // Deactivate
      auto resp = docking_mode_srv_->async_send_request(req_hold);
      auto resp_viz = docking_mode_srv_viz->async_send_request(req_hold);
      auto resp_counter_clock = inspection_counter_clock_srv->async_send_request(req_hold);
      auto resp_counter_clock_viz = inspection_counter_clock_srv_viz->async_send_request(req_hold);
      auto resp_hold = depth_hold_srv_->async_send_request(req_hold);
      auto resp_hold_viz = depth_hold_srv_viz->async_send_request(req_hold);
      auto resp_hold_2 = heading_hold_srv_->async_send_request(req_hold);
      auto resp_hold_viz_2 = heading_hold_srv_viz->async_send_request(req_hold);
    } else {
      RCLCPP_INFO(get_logger(), "Inspection (clockwise) mode is now inactive");
      joystick_active_ = true;
      req->data = false;

      auto resp = inspection_clock_srv->async_send_request(req);
      auto resp_viz = inspection_clock_srv_viz->async_send_request(req);
    }
  } else if (axes[dpad_horizontal_axis_] != -1.0 && dpad_right_pressed_) {
    dpad_right_pressed_ = false;
  }

  // --- Inspection Counter-Clockwise (D-Pad LEFT) ---
  if (axes[dpad_horizontal_axis_] == 1.0 && !dpad_left_pressed_) {
    dpad_left_pressed_ = true;
    ctrl_state_6_ = !ctrl_state_6_;   // add a new ctrl_state_6_ in your class
    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();

    if (ctrl_state_6_) {
      joystick_active_ = false;
      ctrl_state_1_ = false;
      ctrl_state_2_ = false;
      ctrl_state_3_ = false;
      ctrl_state_4_ = false;
      ctrl_state_5_ = false;
      RCLCPP_INFO(get_logger(), "Inspection (counter-clockwise) mode is now active");
      req->data = true;
      auto req_hold = std::make_shared<std_srvs::srv::SetBool::Request>();
      req_hold->data = false;
      // Activate
      auto resp_counter_clock = inspection_counter_clock_srv->async_send_request(req);
      auto resp_counter_clock_viz = inspection_counter_clock_srv_viz->async_send_request(req);
      // Deactivate
      auto resp = docking_mode_srv_->async_send_request(req_hold);
      auto resp_viz = docking_mode_srv_viz->async_send_request(req_hold);
      auto resp_clock = inspection_clock_srv->async_send_request(req_hold);
      auto resp_clock_viz = inspection_clock_srv_viz->async_send_request(req_hold);
      auto resp_hold = depth_hold_srv_->async_send_request(req_hold);
      auto resp_hold_viz = depth_hold_srv_viz->async_send_request(req_hold);
      auto resp_hold_2 = heading_hold_srv_->async_send_request(req_hold);
      auto resp_hold_viz_2 = heading_hold_srv_viz->async_send_request(req_hold);

    } else {
      RCLCPP_INFO(get_logger(), "Inspection (counter-clockwise) mode is now inactive");
      joystick_active_ = true;
      req->data = false;

      auto resp = inspection_counter_clock_srv->async_send_request(req);
      auto resp_viz = inspection_counter_clock_srv_viz->async_send_request(req);
    }
  } else if (axes[dpad_horizontal_axis_] != 1.0 && dpad_left_pressed_) {
    dpad_left_pressed_ = false;
  }

  // Tilt logic
  std_msgs::msg::Float64 tilt_msg;
  tilt_msg.data = 0.0;
  if (buttons[4]) {
    tilt_msg.data = 0.5;
  } else if (buttons[5]) {
    tilt_msg.data = -0.5;
  }
  tilt_pub_->publish(tilt_msg);
}
