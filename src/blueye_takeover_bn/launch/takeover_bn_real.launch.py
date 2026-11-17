"""
Takeover network (v4) on the real Blueye, with the evidence sources of the vehicle:

  handler (blueye_handler)      /blueye/battery (Pose), /blueye/speed, /blueye/altitude, /blueye/sensor/ekf/pose
  Water Linked DVL driver       /blueye/sensor/dvl (marine_acoustic_msgs/Dvl): DVLStatus and altitude
  camera_quality_monitor        CameraQuality, from the drone's RTSP stream
  aruco_visibility_monitor      ArUcoVisibility, from /blueye/aruco/status (pose_estimation_aruco_real_life), relayed
                                by virtual_sensors unless injected on the HUD (no dock at the site)
  behaviour tree, mission.py    /mission_state and /blueye/waypoint_goal (lat/long waypoints as north/east)
  human_model                   Fatigue, Stress
  virtual_sensors               USBLStrength and SonarRange (not fitted: healthy, or as set on the operator HUD's
                                Injected Evidence tab) and Current (not measured: 0)

The dock's position in the handler's frame is not known, so ArUco evidence is used in Undocking and Docking once the
markers have been detected in that phase, and the dock's USBL shadow rule of the simulator is off.

Start after the handler, real_sensors.launch.py and the operator HUD, and restart it for every mission (the
operator model's clock and the network's evidence window start again):
  ros2 launch blueye_takeover_bn takeover_bn_real.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("drone_ip", default_value="192.168.1.101"),
            DeclareLaunchArgument("takeover_threshold", default_value="0.65"),
            DeclareLaunchArgument("expected_aruco_markers", default_value="6",
                                  description="markers on the dock board: all detected = ArUcoVisibility All"),
            Node(
                package="blueye_takeover_bn",
                executable="takeover_bn",
                name="takeover_bayesian_network",
                output="screen",
                parameters=[{
                    "update_rate_hz": 2.0,
                    "default_mission_phase": "Undocking",
                    "use_default_mission_phase": True,
                    "takeover_threshold": LaunchConfiguration("takeover_threshold"),
                    "battery_message": "pose",
                    "dvl_message": "marine_acoustic",
                    "dvl_topic": "/blueye/sensor/dvl",
                    "dvl_degraded_good_beams": 4,
                    "sonar_source": "state",
                    "goal_odometry_topic": "/blueye/sensor/ekf/pose",
                    "aruco_gating": "acquired",
                    "aruco_phases": ["Undocking", "Docking"],
                    "usbl_shadow_radius_m": 0.0,
                    "mask_known_structures": False,
                }],
            ),
            Node(
                package="blueye_takeover_bn",
                executable="virtual_sensors",
                name="virtual_sensors",
                output="screen",
                parameters=[{
                    "publish_rate_hz": 2.0,
                    "usbl_state": "Strong",
                    "sonar_state": "Clear",
                    "current_ms": 0.0,
                }],
            ),
            Node(
                package="blueye_takeover_bn",
                executable="human_model",
                name="human_model",
                output="screen",
                parameters=[{
                    "publish_rate_hz":    2.0,
                    "ramp_duration_s":    900.0,
                    "fatigue_start":      0.15,
                    "fatigue_end":        0.55,
                    "stress_start":       0.15,
                    "stress_end":         0.50,
                    "noise_std":          0.02,
                    # the real tree publishes /mission_state only when the phase changes, so silence is not the
                    # end of a mission here: the clock runs from the first mission start until this launch stops
                    "mission_restart_gap_s": 1.0e9,
                }],
            ),
            Node(
                package="blueye_takeover_bn_real",
                executable="camera_quality_monitor",
                name="camera_quality_monitor",
                output="screen",
                # Thresholds from the first wet run (wet_shadow_20261022_173843, harbour, surface to 2.3 m, dock
                # markers detected at 8-15 m): sharpness (Laplacian variance, 1080p) p5/p50/p95 3.5/5.2/7.7,
                # brightness 134-145. The defaults (Good above 50, brightness below 120) read every frame as Failed.
                parameters=[{
                    "rtsp_url": ["rtsp://", LaunchConfiguration("drone_ip"), ":8554/test"],
                    "excellent_sharpness": 8.0,
                    "good_sharpness": 3.0,
                    "poor_sharpness": 1.5,
                    "excellent_min_brightness": 40.0,
                    "excellent_max_brightness": 170.0,
                    "good_min_brightness": 20.0,
                    "good_max_brightness": 200.0,
                }],
            ),
            Node(
                package="blueye_takeover_bn_real",
                executable="aruco_visibility_monitor",
                name="aruco_visibility_monitor",
                output="screen",
                parameters=[{
                    "expected_markers": LaunchConfiguration("expected_aruco_markers"),
                    # to virtual_sensors, which passes it on to /blueye/aruco_visibility or replaces it
                    "visibility_topic": "/blueye/aruco_visibility_real",
                }],
            ),
        ]
    )
