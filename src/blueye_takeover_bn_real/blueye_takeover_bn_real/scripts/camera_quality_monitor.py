#!/usr/bin/env python3
import threading

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String

import cv2
import numpy as np


class CameraQualityMonitor(Node):
    """Classify camera quality using sharpness, brightness, and image latency."""

    def __init__(self):
        super().__init__("camera_quality_monitor")
        self.declare_parameter("source_mode", "rtsp")
        self.declare_parameter("rtsp_url", "rtsp://192.168.1.101:8554/test")
        self.declare_parameter("camera_topic", "/blueye/image")
        self.declare_parameter("quality_topic", "/blueye/camera_quality")
        self.declare_parameter("latency_topic", "/blueye/camera_latency")
        self.declare_parameter("frame_gap_topic", "/blueye/camera_frame_gap")
        self.declare_parameter("excellent_sharpness", 200.0)
        self.declare_parameter("good_sharpness", 50.0)
        self.declare_parameter("poor_sharpness", 10.0)
        self.declare_parameter("excellent_min_brightness", 40.0)
        self.declare_parameter("excellent_max_brightness", 100.0)
        self.declare_parameter("good_min_brightness", 20.0)
        self.declare_parameter("good_max_brightness", 120.0)
        self.declare_parameter("good_latency_s", 0.25)
        self.declare_parameter("poor_latency_s", 1.0)
        self.declare_parameter("good_frame_gap_s", 0.2)
        self.declare_parameter("poor_frame_gap_s", 0.7)

        camera_topic = str(self.get_parameter("camera_topic").value)
        quality_topic = str(self.get_parameter("quality_topic").value)
        latency_topic = str(self.get_parameter("latency_topic").value)
        frame_gap_topic = str(self.get_parameter("frame_gap_topic").value)

        self.bridge = CvBridge()
        self.last_receive_time = None
        self.shutdown_requested = False
        self.quality_pub = self.create_publisher(String, quality_topic, 10)
        self.latency_pub = self.create_publisher(Float32, latency_topic, 10)
        self.frame_gap_pub = self.create_publisher(Float32, frame_gap_topic, 10)

        source_mode = str(self.get_parameter("source_mode").value).lower()
        if source_mode == "ros":
            self.create_subscription(Image, camera_topic, self.image_callback, 10)
            self.get_logger().info(f"Listening to camera images on {camera_topic}")
        else:
            self.rtsp_thread = threading.Thread(target=self.rtsp_loop, daemon=True)
            self.rtsp_thread.start()
            self.get_logger().info(f"Reading camera stream from {self.get_parameter('rtsp_url').value}")

        self.get_logger().info(f"Publishing camera quality on {quality_topic}")

    def image_callback(self, msg: Image):
        receive_time = self.get_clock().now()
        latency_s = self._image_latency_s(msg)
        frame_gap_s = self._frame_gap_s(receive_time)
        self.last_receive_time = receive_time

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warning(f"Could not convert image: {exc}")
            self._publish_quality("Failed", latency_s, frame_gap_s)
            return

        self._process_image(image, latency_s, frame_gap_s)

    def rtsp_loop(self):
        rtsp_url = str(self.get_parameter("rtsp_url").value)
        gst_pipeline = (
            f"rtspsrc location={rtsp_url} latency=50 drop-on-latency=true ! "
            "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
            "video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false"
        )

        cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            self.get_logger().warning("GStreamer RTSP pipeline failed; trying OpenCV FFMPEG")
            cap = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)

        if not cap.isOpened():
            self.get_logger().error(f"Unable to open camera stream: {rtsp_url}")
            self._publish_quality("Failed", 0.0, 0.0)
            return

        try:
            while rclpy.ok() and not self.shutdown_requested:
                ret, image = cap.read()
                receive_time = self.get_clock().now()
                frame_gap_s = self._frame_gap_s(receive_time)
                self.last_receive_time = receive_time

                if not ret:
                    self.get_logger().warning("Unable to fetch camera frame")
                    self._publish_quality("Failed", 0.0, frame_gap_s)
                    continue

                self._process_image(image, 0.0, frame_gap_s)
        finally:
            cap.release()

    def _process_image(self, image, latency_s: float, frame_gap_s: float):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        sharpness = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        brightness = float(np.mean(gray))

        quality = self._quality_from_image(sharpness, brightness)
        quality = self._downgrade_for_timing(quality, latency_s, frame_gap_s)
        self._publish_quality(quality, latency_s, frame_gap_s)

        self.get_logger().info(
            f"Camera {quality}: sharpness={sharpness:.1f}, "
            f"brightness={brightness:.1f}, latency={latency_s:.3f}s, frame_gap={frame_gap_s:.3f}s"
        )

    def _image_latency_s(self, msg: Image) -> float:
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds == 0:
            return 0.0
        return max(0.0, (self.get_clock().now() - stamp).nanoseconds / 1e9)

    def _frame_gap_s(self, receive_time) -> float:
        if self.last_receive_time is None:
            return 0.0
        return max(0.0, (receive_time - self.last_receive_time).nanoseconds / 1e9)

    def _quality_from_image(self, sharpness: float, brightness: float) -> str:
        excellent_sharpness = float(self.get_parameter("excellent_sharpness").value)
        good_sharpness = float(self.get_parameter("good_sharpness").value)
        poor_sharpness = float(self.get_parameter("poor_sharpness").value)
        excellent_min_brightness = float(self.get_parameter("excellent_min_brightness").value)
        excellent_max_brightness = float(self.get_parameter("excellent_max_brightness").value)
        good_min_brightness = float(self.get_parameter("good_min_brightness").value)
        good_max_brightness = float(self.get_parameter("good_max_brightness").value)

        if (
            sharpness > excellent_sharpness
            and excellent_min_brightness < brightness < excellent_max_brightness
        ):
            return "Excellent"
        if sharpness > good_sharpness and good_min_brightness < brightness < good_max_brightness:
            return "Good"
        if sharpness > poor_sharpness:
            return "Poor"
        return "Failed"

    def _downgrade_for_timing(self, quality: str, latency_s: float, frame_gap_s: float) -> str:
        good_latency = float(self.get_parameter("good_latency_s").value)
        poor_latency = float(self.get_parameter("poor_latency_s").value)
        good_frame_gap = float(self.get_parameter("good_frame_gap_s").value)
        poor_frame_gap = float(self.get_parameter("poor_frame_gap_s").value)

        if latency_s >= poor_latency or frame_gap_s >= poor_frame_gap:
            return "Failed"
        if (
            (latency_s >= good_latency or frame_gap_s >= good_frame_gap)
            and quality in ("Excellent", "Good")
        ):
            return "Poor"
        return quality

    def _publish_quality(self, quality: str, latency_s: float, frame_gap_s: float):
        quality_msg = String()
        quality_msg.data = quality
        self.quality_pub.publish(quality_msg)

        latency_msg = Float32()
        latency_msg.data = float(latency_s)
        self.latency_pub.publish(latency_msg)

        frame_gap_msg = Float32()
        frame_gap_msg.data = float(frame_gap_s)
        self.frame_gap_pub.publish(frame_gap_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraQualityMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_requested = True
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
