#ifndef MUNDUS_MIR_THRUSTER_ALLOCATION_HPP_
#define MUNDUS_MIR_THRUSTER_ALLOCATION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>


using namespace std;

class BlueyeThrustAllocator : public rclcpp::Node {

    public:
        BlueyeThrustAllocator();
        ~BlueyeThrustAllocator();
    
    private:
        void desired_force_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
        void fetch_ros_parameters();

    private:
            
            // ROS 
            rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr desired_force_sub_;
            rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr thrusters_pub_;
    
            // Configuration file parameters
            string cmd_force_topic_, thrusters_topic_;
            double max_thruster_setpoint_;

};

#endif
