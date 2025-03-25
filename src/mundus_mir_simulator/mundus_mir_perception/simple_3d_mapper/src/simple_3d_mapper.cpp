#include "simple_3d_mapper/simple_3d_mapper.hpp"

simple_3d_mapper::simple_3d_mapper() : Node("simple_3d_mapper") {

    // Get ros parameters
    get_ros_parameters();


    // Set up subscribers and publishers
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odometry_topic_, 1,
        std::bind(&simple_3d_mapper::odometry_callback, this,
                  std::placeholders::_1));

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic_, 1,
        std::bind(&simple_3d_mapper::image_callback, this,
                  std::placeholders::_1));

    octomap_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>(
        octomap_topic_, 1);

    // Initialize octomap
    octomap_ = std::make_shared<octomap::OcTree>(0.2);

    RCLCPP_INFO(get_logger(), "Simple 3D mapper node has been initialized.");

}

simple_3d_mapper::~simple_3d_mapper() {}

void simple_3d_mapper::get_ros_parameters() {
    declare_parameter("odometry_topic", "blueye/odometry_frd/gt");
    declare_parameter("image_topic", "blueye/depth_camera/image_raw");
    declare_parameter("octomap_topic", "/blueye/voxelmap");

    odometry_topic_ = get_parameter("odometry_topic")
                         .get_parameter_value()
                         .get<std::string>();

    image_topic_ =    get_parameter("image_topic")
                         .get_parameter_value()
                         .get<std::string>();

    octomap_topic_ =  get_parameter("octomap_topic")
                         .get_parameter_value()
                         .get<std::string>();
}

void simple_3d_mapper::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

    // Store last odometry message
    last_odometry_msg_ = msg;
    odometry_received_ = true;
}

void simple_3d_mapper::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {

    RCLCPP_INFO(get_logger(), "Image Callback");

    // Check if odometry message has been received
    if (!odometry_received_) {
        return;
    }

    // Extracting depth image 
    cv::Mat depth_image;
    try {
        depth_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Projecting depth image to octomap
    for (int v = 0; v < depth_image.rows; v++) {
        for (int u = 0; u < depth_image.cols; u++) {
            // Get depth value
            float d = depth_image.at<float>(v, u);
            if (d == 0 || d > 10.0) {
                continue;
            }

            // Get 3D point
            double Z = d;
            double X = (u - cx_) * Z / fx_;
            double Y = (v - cy_) * Z / fy_;

            // Camera to odom frame
            tf2::Vector3 pt_camera(X, Y, Z);
            tf2::Matrix3x3 R(
                0, 0, 1,
                1, 0, 0,
                0, -1, 0
            );
            tf2::Vector3 pt_odom = R * pt_camera;

            // Transform 3D point to world frame
            tf2::Transform tf_odom_to_world;
            tf2::fromMsg(last_odometry_msg_->pose.pose, tf_odom_to_world);
            tf2::Vector3 pt_world = tf_odom_to_world * pt_odom;

            // Insert point to octomap
            octomap::point3d endpoint(pt_world.x(), pt_world.y(), pt_world.z());
            octomap_->updateNode(endpoint, true, true);
        }
    }

    // Publish current octomap
    octomap_msgs::msg::Octomap octomap_msg;
    if (octomap_msgs::fullMapToMsg(*octomap_, octomap_msg)) { // Dereference octomap_ to pass the actual object
        octomap_msg.header.frame_id = "world"; // Set the appropriate frame ID
        octomap_msg.header.stamp = this->now(); // Set the timestamp
        octomap_pub_->publish(octomap_msg);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to convert octomap to message.");
    }

}