#include "mundus_mir_vehicle_interfaces/blueye_simulator_interface.hpp"

// 

#include <algorithm>

BlueyeThrustAllocator::BlueyeThrustAllocator() : Node("mundus_mir_thruster_allocation") {
    // Fetch parameters
    fetch_ros_parameters();

    // Initialize publisher and subscriber
    thrusters_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(thrusters_topic_, 1);
    desired_force_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(cmd_force_topic_, 10, std::bind(&BlueyeThrustAllocator::desired_force_callback, this, std::placeholders::_1));

}

BlueyeThrustAllocator::~BlueyeThrustAllocator() {

}

void BlueyeThrustAllocator::fetch_ros_parameters() {
    // Fetch parameters
    this->declare_parameter("cmd_force_topic", "/blueye/cmd_force");
    this->declare_parameter("thrusters_topic", "/blueye/thrusters");
    this->declare_parameter("max_thruster_setpoint", 35.0);

    this->get_parameter("cmd_force_topic", cmd_force_topic_);
    this->get_parameter("thrusters_topic", thrusters_topic_);
    this->get_parameter("max_thruster_setpoint", max_thruster_setpoint_);
}

void BlueyeThrustAllocator::desired_force_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {

    // Calculate individual thrusts
    Eigen::VectorXd desired_thrusts(4);
    desired_thrusts(0) = msg->wrench.force.x;
    desired_thrusts(1) = msg->wrench.force.y;
    desired_thrusts(2) = msg->wrench.force.z;
    desired_thrusts(3) = msg->wrench.torque.z;

    Eigen::MatrixXd TAM(4,4);
    TAM << 0.5, 0.5, 0.0, 0.0,
           0.0, 0.0, -1.0, 0.0,
           0.0, 0.0, 0.0, 1.0,
           0.5, -0.5, 0.0, 0.0;

    Eigen::MatrixXd TAM_pseudo_inverse = TAM.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::VectorXd thrust_commands = TAM_pseudo_inverse * desired_thrusts;

    // Publish Stonefish aggregate thruster setpoints in actuator order:
    // [Thruster1, Thruster2, Thruster3, Thruster4].
    std_msgs::msg::Float64MultiArray thrusters_msg;
    thrusters_msg.data.resize(4);
    for (int i = 0; i < 4; ++i) {
        thrust_commands(i) = std::clamp(
            thrust_commands(i), -max_thruster_setpoint_, max_thruster_setpoint_);
        thrusters_msg.data[i] = thrust_commands(i);
    }
    thrusters_pub_->publish(thrusters_msg);
}

// Starts and spins blueye vehicle interface
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BlueyeThrustAllocator>());
    rclcpp::shutdown();
    return -1;
}
