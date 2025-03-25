#ifndef SIMPLE_3D_MAPPER_HPP_
#define SIMPLE_3D_MAPPER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"


class simple_3d_mapper : public rclcpp::Node {

    public:
        simple_3d_mapper();
        ~simple_3d_mapper();

    private:
        void get_ros_parameters();
        void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    private:

        // ROS publishers and subscribers, requires a depth camera and an odometry topic
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;

        // ROS parameters
        std::string odometry_topic_, image_topic_, octomap_topic_;

        // Odometry messages
        nav_msgs::msg::Odometry::SharedPtr last_odometry_msg_;
        bool odometry_received_ = false;

        // Octomap 
        std::shared_ptr<octomap::OcTree> octomap_;

        // OpenCV variables
        cv_bridge::CvImagePtr cv_ptr_;
        double fx_ = 227.0;
        double fy_ = 227.0;
        double cx_ = 160.0;
        double cy_ = 120.0;

};

#endif