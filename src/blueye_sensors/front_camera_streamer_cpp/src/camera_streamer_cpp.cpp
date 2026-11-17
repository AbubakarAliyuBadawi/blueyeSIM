#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <thread>
#include <atomic>

class BlueyeCameraNode : public rclcpp::Node {
public:
  BlueyeCameraNode() : Node("blueye_camera_node"), is_running_(true) {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/blueye/image", 10);
    open_camera();
    capture_thread_ = std::thread(&BlueyeCameraNode::capture_loop, this);
  }

  ~BlueyeCameraNode() {
    is_running_ = false;
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    cap_.release();
    RCLCPP_INFO(this->get_logger(), "Camera stream closed and thread joined.");
  }

private:
  void open_camera() {
    std::string gst_pipeline =
        "rtspsrc latency=0 location=rtsp://192.168.1.101:8554/test ! decodebin ! videoconvert ! appsink";
    //std::string gst_pipeline =
    //"rtspsrc latency=0 location=rtsp://192.168.1.101:8554/test ! decodebin ! videoconvert ! appsink drop=true max-buffers=1 sync=false";

    cap_.open(gst_pipeline, cv::CAP_GSTREAMER);

    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Unable to open camera stream.");
    }
  }

  void capture_loop() {
    while (rclcpp::ok() && is_running_) {
      cv::Mat frame;
      if (!cap_.read(frame)) {
        RCLCPP_WARN(this->get_logger(), "Failed to read frame.");
        continue;
      }

      auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
      publisher_->publish(*msg);
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  cv::VideoCapture cap_;
  std::thread capture_thread_;
  std::atomic<bool> is_running_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BlueyeCameraNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

