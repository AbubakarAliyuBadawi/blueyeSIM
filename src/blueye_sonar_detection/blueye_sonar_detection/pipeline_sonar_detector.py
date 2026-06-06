#!/usr/bin/env python3
"""
Pipeline sonar detector — visualise FLS image with pipe detection overlay.

Subscribes  : /blueye/fls           (sensor_msgs/Image, raw FLS sonar image)
Publishes   : /blueye/fls_raw       (sensor_msgs/Image, raw image with first-return profile)
              /blueye/fls_cartesian  (sensor_msgs/Image, top-down Cartesian sonar map)

The ROV does not move. Run this node while the simulator is running and
view the output topics in rqt_image_view or Foxglove.
"""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


# --------------------------------------------------------------------------- #
#  FLS sensor parameters — must match blueye.scn <specs> values               #
# --------------------------------------------------------------------------- #
BEAMS = 512
BINS = 256
MAX_RANGE_M = 5.0
HFOV_DEG = 90.0

RANGE_PER_BIN = MAX_RANGE_M / BINS          # metres per row
DEG_PER_BEAM = HFOV_DEG / BEAMS            # degrees per column
RAD_PER_BEAM = math.radians(DEG_PER_BEAM)

# Bearing of beam 0 and last beam (centred at 0)
BEARING_MIN_RAD = math.radians(-HFOV_DEG / 2.0)

# Detection threshold (0-255).  Bright pixels = strong acoustic return.
# Tune this value — start high to avoid floor noise, lower if pipe is missed.
INTENSITY_THRESHOLD = 60

# Cartesian output image size (pixels) and resolution (m/px)
CART_SIZE_PX = 512
CART_RESOLUTION = MAX_RANGE_M / (CART_SIZE_PX / 2)   # m per pixel


class PipelineSonarDetector(Node):
    def __init__(self):
        super().__init__("pipeline_sonar_detector")

        self.bridge = CvBridge()
        self._first_msg = True

        self.sub_fls = self.create_subscription(
            Image, "/blueye/fls/image", self._fls_callback, 10
        )
        self.pub_raw = self.create_publisher(Image, "/blueye/fls_raw", 10)
        self.pub_cart = self.create_publisher(Image, "/blueye/fls_cartesian", 10)

        self.get_logger().info(
            "Pipeline sonar detector started — waiting for /blueye/fls …"
        )

    # ----------------------------------------------------------------------- #

    def _fls_callback(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")

        if self._first_msg:
            self.get_logger().info(
                f"First FLS message received: shape={img.shape}  encoding={msg.encoding}"
            )
            self._first_msg = False

        # /blueye/fls/image: 256 rows = bins (range, near=row 0), 512 cols = beams (angle)

        self.pub_raw.publish(
            self._build_raw_image(msg.header, img)
        )
        self.pub_cart.publish(
            self._build_cartesian_image(msg.header, img)
        )

    # ----------------------------------------------------------------------- #

    def _first_return_profile(self, img: np.ndarray):
        """
        For each beam (column), return the bin (row) of the first pixel that
        exceeds INTENSITY_THRESHOLD, or None if no return exists.
        """
        profile = []
        for col in range(img.shape[1]):
            hits = np.where(img[:, col] > INTENSITY_THRESHOLD)[0]
            profile.append(int(hits[0]) if len(hits) > 0 else None)
        return profile

    def _profile_to_range_bearing(self, profile):
        """
        Convert (col, row) profile entries to (range_m, bearing_deg) pairs.
        Returns a list of (bearing_deg, range_m) tuples for valid detections.
        """
        points = []
        for col, row in enumerate(profile):
            if row is None:
                continue
            bearing_deg = (col / BEAMS * HFOV_DEG) - HFOV_DEG / 2.0
            range_m = row * RANGE_PER_BIN
            points.append((bearing_deg, range_m))
        return points

    # ----------------------------------------------------------------------- #

    def _build_raw_image(self, header, img: np.ndarray) -> Image:
        """
        Raw FLS image with the first-return profile drawn in green and the
        closest detected point (candidate pipe) highlighted in red.
        """
        vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        profile = self._first_return_profile(img)

        # Draw first-return profile as green dots
        for col, row in enumerate(profile):
            if row is not None:
                cv2.circle(vis, (col, row), 1, (0, 255, 0), -1)

        # Find the closest return — candidate pipe
        valid = [(col, row) for col, row in enumerate(profile) if row is not None]
        if valid:
            min_col, min_row = min(valid, key=lambda p: p[1])
            range_m = min_row * RANGE_PER_BIN
            bearing_deg = (min_col / BEAMS * HFOV_DEG) - HFOV_DEG / 2.0

            cv2.circle(vis, (min_col, min_row), 8, (0, 0, 255), 2)
            cv2.line(vis, (min_col, 0), (min_col, vis.shape[0] - 1), (0, 0, 255), 1)
            cv2.putText(
                vis,
                f"Pipe candidate: {range_m:.2f} m  {bearing_deg:+.1f} deg",
                (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 255),
                1,
            )
        else:
            cv2.putText(
                vis,
                "No detection (threshold too high?)",
                (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 200, 255),
                1,
            )

        # Axis labels
        cv2.putText(vis, "-45 deg", (2, vis.shape[0] - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        cv2.putText(vis, "+45 deg", (vis.shape[1] - 70, vis.shape[0] - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        cv2.putText(vis, "0 m", (2, 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        cv2.putText(vis, f"{MAX_RANGE_M:.0f} m", (2, vis.shape[0] - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

        out = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
        out.header = header
        return out

    # ----------------------------------------------------------------------- #

    def _build_cartesian_image(self, header, img: np.ndarray) -> Image:
        """
        Convert the polar FLS image to a top-down Cartesian map.

        The ROV is at the bottom-centre of the image.
        X axis (horizontal) = sway (positive right).
        Y axis (vertical)   = surge (positive upward in image = forward).
        """
        S = CART_SIZE_PX
        cart = np.zeros((S, S, 3), dtype=np.uint8)

        # ROV position in image pixels — bottom centre
        origin_px = (S // 2, S - 1)

        profile = self._first_return_profile(img)

        # Paint all raw sonar returns as dim grey points
        for col in range(img.shape[1]):
            bearing_rad = BEARING_MIN_RAD + col * RAD_PER_BEAM
            for row in range(img.shape[0]):
                intensity = int(img[row, col])
                if intensity < 20:
                    continue
                range_m = row * RANGE_PER_BIN
                px_x = origin_px[0] + int(range_m * math.sin(bearing_rad) / CART_RESOLUTION)
                px_y = origin_px[1] - int(range_m * math.cos(bearing_rad) / CART_RESOLUTION)
                if 0 <= px_x < S and 0 <= px_y < S:
                    # Blend intensity into the grey channel
                    grey = min(255, intensity)
                    cart[px_y, px_x] = np.maximum(cart[px_y, px_x], [grey // 3, grey // 3, grey // 3])

        # Paint first-return profile in green
        for col, row in enumerate(profile):
            if row is None:
                continue
            bearing_rad = BEARING_MIN_RAD + col * RAD_PER_BEAM
            range_m = row * RANGE_PER_BIN
            px_x = origin_px[0] + int(range_m * math.sin(bearing_rad) / CART_RESOLUTION)
            px_y = origin_px[1] - int(range_m * math.cos(bearing_rad) / CART_RESOLUTION)
            if 0 <= px_x < S and 0 <= px_y < S:
                cart[px_y, px_x] = [0, 255, 0]

        # Highlight closest return (pipe candidate) in red
        valid = [(col, row) for col, row in enumerate(profile) if row is not None]
        if valid:
            min_col, min_row = min(valid, key=lambda p: p[1])
            bearing_rad = BEARING_MIN_RAD + min_col * RAD_PER_BEAM
            range_m = min_row * RANGE_PER_BIN
            px_x = origin_px[0] + int(range_m * math.sin(bearing_rad) / CART_RESOLUTION)
            px_y = origin_px[1] - int(range_m * math.cos(bearing_rad) / CART_RESOLUTION)
            if 0 <= px_x < S and 0 <= px_y < S:
                cv2.circle(cart, (px_x, px_y), 6, (0, 0, 255), -1)

        # Draw ROV marker
        cv2.circle(cart, origin_px, 5, (255, 200, 0), -1)
        cv2.putText(cart, "ROV", (origin_px[0] + 6, origin_px[1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 200, 0), 1)

        # Draw FOV lines
        for deg in (-45, 45):
            rad = math.radians(deg)
            end_x = origin_px[0] + int(MAX_RANGE_M * math.sin(rad) / CART_RESOLUTION)
            end_y = origin_px[1] - int(MAX_RANGE_M * math.cos(rad) / CART_RESOLUTION)
            cv2.line(cart, origin_px, (end_x, end_y), (60, 60, 60), 1)

        # Range rings
        for r_m in [1.0, 2.0, 3.0, 4.0, 5.0]:
            r_px = int(r_m / CART_RESOLUTION)
            cv2.circle(cart, origin_px, r_px, (40, 40, 40), 1)
            cv2.putText(cart, f"{r_m:.0f}m", (origin_px[0] + 2, origin_px[1] - r_px + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.3, (80, 80, 80), 1)

        out = self.bridge.cv2_to_imgmsg(cart, encoding="bgr8")
        out.header = header
        return out


# --------------------------------------------------------------------------- #

def main():
    rclpy.init()
    node = PipelineSonarDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
