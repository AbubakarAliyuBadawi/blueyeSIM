#ifndef MUNDUS_MIR_BLUEYE_JOYSTICK_CPP_HPP
#define MUNDUS_MIR_BLUEYE_JOYSTICK_CPP_HPP

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <vector>

using namespace std;

class JoystickController : public rclcpp::Node {

public:
  JoystickController();
  ~JoystickController();

  // Methods
private:
  void joystick_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void fetch_ros_parameters();
  void handle_buttons(const std::vector<int32_t> &buttons,
                    const std::vector<float> &axes);
  void send_commands();

  // Variables
private:
  // ROS
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lights_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr tilt_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Control States
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr depth_hold_srv_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr heading_hold_srv_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr docking_mode_srv_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr inspection_clock_srv;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr inspection_counter_clock_srv;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr stationkeep_srv_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr depth_hold_srv_viz;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr heading_hold_srv_viz;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr docking_mode_srv_viz;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr inspection_counter_clock_srv_viz;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr inspection_clock_srv_viz;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr stationkeep_srv_viz;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr align_aruco_srv_;

  // Configuration file parameters
  bool quadratic_mapping_, velocity_control_;
  double max_thrust_surge_, max_thrust_sway_, max_thrust_heave_,
      max_thrust_yaw_;
  double max_velocity_x_, max_velocity_y_, max_velocity_z_,
      max_angular_velocity_yaw_;
  string joy_topic_, cmd_topic_, vel_topic_;
  string depth_hold_service_, heading_hold_service_, docking_mode_service_,
            stationkeep_service_, inspection_clock_service_,
            inspection_counter_clock_service_;
  double deadzone_;

  // Controller indexes, may vary from joystick to joystick..
  int right_joystick_x_axis_;
  int right_joystick_y_axis_;
  int left_joystick_x_axis_;
  int left_joystick_y_axis_;
  int x_button_;
  int circle_button_;
  int square_button_;
  int triangle_button_;
  int dpad_horizontal_axis_;
  int dpad_vertical_axis_;

  // Control States
  bool joystick_active_ = false;
  bool ctrl_state_1_ = false;
  bool ctrl_state_2_ = false;
  bool ctrl_state_3_ = false;
  bool ctrl_state_4_ = false;
  bool ctrl_state_5_ = false;
  bool ctrl_state_6_ = false;

  // Button states
  bool x_is_pressed_ = false;
  bool triangle_is_pressed_ = false;
  bool circle_is_pressed_ = false;
  bool square_is_pressed_ = false;
  bool dpad_right_pressed_ = false;
  bool dpad_left_pressed_ = false;

  // Last states
  double right_x = 0.0;
  double right_y = 0.0;
  double left_x = 0.0;
  double left_y = 0.0;
};
#endif
