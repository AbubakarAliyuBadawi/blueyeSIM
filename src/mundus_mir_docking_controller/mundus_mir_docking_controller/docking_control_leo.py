import math
import time
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, Pose
from geometry_msgs.msg import WrenchStamped
from blueye_interfaces.srv import AddWaypoint, ClearWaypoints, InsertWaypoint, RemoveWaypoint
from std_srvs.srv import SetBool


class WaypointHandler():
    '''
    Class constructor
    '''
    def __init__(self):                
                                   #   z        x    y
        self.waypoints_left_loop = [[-0.35, 1.5 , 0.2, np.deg2rad(-98), 0.3], # WP1_r
                                     [1.5, 1.5 , 0.45, np.deg2rad(-145), 0.3], # WP2_r
                                     [1.5, 0.0, 0.45, np.deg2rad(180), 0.16], # WP4_r
                                     #[0.0, 0.0, -0.317, np.deg2rad(180), 0.13], # WP5_r
                                     [0.25, 0.01, 0.3, np.deg2rad(180), 0.16], # WP6_r
                                     [-0.1, 0.01, 0.3, np.deg2rad(180), 0.16], # WP6_r NEW
                                     [-0.4, 0.01, 0.3, np.deg2rad(180), 0.16], # WP6_r NEW
                                     [-0.6497, 0.0, 0.2582, np.deg2rad(180), 0.05]] # WP7_r
        
        self.waypoints_right_loop = [[-0.5, -2.0 , 0.4, np.deg2rad(90), 0.3], # WP1_r
                                     [1.0, -2.0 , 0.45, np.deg2rad(145), 0.3], # WP2_r
                                     [1.0, -1.0, 0.45, np.deg2rad(155), 0.3], # WP3_r
                                     [1.0, 0.0, 0.45, np.deg2rad(-180), 0.16], # WP4_r
                                     #[0.0, 0.0, -0.317, np.deg2rad(-180), 0.13], # WP5_r
                                     [0.25, 0.01, 0.3, np.deg2rad(-180), 0.16], # WP6_r
                                     [-0.1, 0.01, 0.3, np.deg2rad(180), 0.16], # WP6_r NEW
                                     [-0.4, 0.01, 0.3, np.deg2rad(180), 0.16], # WP6_r NEW
                                     [-0.6497, 0.0, 0.2582, np.deg2rad(-180), 0.05]] # WP7_r
        
        self.waypoints_lawnmover_vert_1m =  [[1.0, -0.3 , 0.0, np.deg2rad(-180), 0.05], # WP1_r
                                            [1.0, -0.3 , -0.8, np.deg2rad(-180), 0.05], # WP2_r
                                            [1.0, 0.0, -0.8, np.deg2rad(-180), 0.05], # WP3_r
                                            [1.0, 0.0, 0.0, np.deg2rad(-180), 0.05], # WP4_r
                                            [1.0, 0.3, 0.0, np.deg2rad(-180), 0.05], # WP5_r
                                            [1.0, 0.3, -0.8, np.deg2rad(-180), 0.05]] # WP6_r        
        
                                    #   z    x    y
        self.waypoints_undocking = [
                                    [-0.4, 0.01, 0.3, np.deg2rad(180), 0.16], # WP6_r NEW
                                    [-0.1, 0.01, 0.3, np.deg2rad(180), 0.16], # WP6_r NEW
                                    [0.3, 0.01, 0.3, np.deg2rad(180), 0.3], # WP6_r NEW
                                    [0.8, 0.05, 0.3, np.deg2rad(180), 0.3], # WP6_r NEW
                                    [1.5, 0.05, 0.45, np.deg2rad(180), 0.3], # WP4_r                            
                                    ]

        self.waypoints_docking = [
                                    [1.5, 0.5, 0.45, np.deg2rad(180), 0.16], # WP4_r
                                    [1.0, 0.5, 0.45, np.deg2rad(180), 0.16], # WP4_r
                                    [0.5, 0.2, 0.35, np.deg2rad(180), 0.16], # WP4_r
                                    [0.25, 0.01, 0.35, np.deg2rad(180), 0.16], # WP6_r
                                    [-0.1, 0.01, 0.35, np.deg2rad(180), 0.16], # WP6_r NEW
                                    [-0.2, 0.01, 0.35, np.deg2rad(180), 0.16], # WP6_r NEW
                                    [-0.2, 0.0, 0.19, np.deg2rad(180), 0.16], # WP7_r NEW
                                    [-0.4, 0.0, 0.19, np.deg2rad(180), 0.16], # WP7_r NEW
                                    [-0.662, 0.0, 0.16, np.deg2rad(180), 0.3],
                                    [-0.7, 0.05, 0.1, np.deg2rad(180), 0.05] # WP7_r # WP7_r                           
                                ]

        self.waypoints_inspection_old = [ #   z    x    y
            # Left side loop
            ### Undocking ###
            [-0.4, 0.01, 0.3, np.deg2rad(180), 0.16], # WP6_r NEW
            [-0.1, 0.01, 0.3, np.deg2rad(180), 0.16], # WP6_r NEW
            [0.3, 0.01, 0.3, np.deg2rad(180), 0.3], # WP6_r NEW
            [0.8, 0.0, 0.3, np.deg2rad(180), 0.3], # WP6_r NEW
            [1.5, 0.0, 0.45, np.deg2rad(180), 0.3], # WP4_r
            
            ### Left Side ###
            [1.5, 0.5, 0.45, np.deg2rad(-160), 0.3], # WP4_r
            [1, 1.5 , 0.45, np.deg2rad(-145), 0.3], # WP2_r
            [-0.35, 2.5 , 0.4, np.deg2rad(-98), 0.3], # WP1_r
            [-0.5, 2.5 , 0.4, np.deg2rad(-90), 0.3], # WP5_b_NEW
            [-1.0, 2.5 , 0.4, np.deg2rad(-90), 0.3], # WP5_b_NEW

            ### Back Side ###
            [-2, 2.5 , 0.4, np.deg2rad(-60), 0.3], # WP4_b_NEW
            [-2, 1.0 , 0.4, np.deg2rad(-50), 0.3], # WP4_b_NEW
            [-3.0, -0.0 , 0.4, np.deg2rad(-45), 0.3], # WP3_b_NEW
            [-3.0, -1.0 , 0.4, np.deg2rad(-10), 0.3], # WP2_b_NEW
            [-3.0, -2.5 , 0.4, np.deg2rad(45), 0.3], # WP1_b_NEW
            [-1.5, -3 , 0.4, np.deg2rad(60), 0.3], # WP1_b_NEW
            [-1.5, -3.5 , 0.4, np.deg2rad(75), 0.3], # WP1_r_NEW
            
            ### Right Side ###
            [-1, -3.5 , 0.4, np.deg2rad(90), 0.3], # WP1_r_NEW
            [-0.5, -3.5 , 0.4, np.deg2rad(90), 0.3], # WP1_r_NEW
            [0, -3.5 , 0.4, np.deg2rad(110), 0.3], # WP1_r
            [0.5, -3 , 0.4, np.deg2rad(135), 0.3], # WP1_r


            ### In Front ###
            [1.5, -2.5 , 0.45, np.deg2rad(145), 0.3], # WP2_r
            [1.5, -1.0, 0.45, np.deg2rad(155), 0.3], # WP3_r
            
            ### Docking ###
            [1.5, 0.0, 0.45, np.deg2rad(180), 0.16], # WP4_r
            [1.0, 0.0, 0.45, np.deg2rad(180), 0.16], # WP4_r
            [0.5, 0.0, 0.35, np.deg2rad(180), 0.16], # WP4_r
            #[0.0, 0.0, -0.317, np.deg2rad(180), 0.13], # WP5_r
            [0.25, 0.01, 0.35, np.deg2rad(180), 0.16], # WP6_r
            
            [-0.1, 0.01, 0.35, np.deg2rad(180), 0.16], # WP6_r NEW
            [-0.2, 0.01, 0.35, np.deg2rad(180), 0.16], # WP6_r NEW
            
            [-0.2, 0.0, 0.19, np.deg2rad(180), 0.16], # WP7_r NEW
            [-0.4, 0.0, 0.19, np.deg2rad(180), 0.16], # WP7_r NEW
            #[-0.662, 0.0, -0.18, np.deg2rad(180), 0.16],
            [-0.662, 0.0, 0.16, np.deg2rad(180), 0.3],
            
            [-0.7, 0.05, 0.1, np.deg2rad(180), 0.05] # WP7_r # WP7_r
        ]
        
        """self.counter_clockwise_inspection = [ #   z    x    y          
            # This generally works, creation file is somehow wrong
            [1.9, -0.2, 0.45, np.deg2rad(164.0), 0.3],
            [1.5, -1.2, 0.45, np.deg2rad(139.0), 0.3],
            [1.1, -1.9, 0.45, np.deg2rad(125.0), 0.3],
            [0.9, -2.3, 0.45, np.deg2rad(117.0), 0.3],
            [-0.0, -2.5, 0.45, np.deg2rad(99.0), 0.3],
            [-1.0, -2.5, 0.45, np.deg2rad(81.0), 0.3],
            [-1.9, -2.3, 0.45, np.deg2rad(63.0), 0.3],
            [-2.1, -1.9, 0.45, np.deg2rad(55.0), 0.3],
            [-2.5, -1.2, 0.45, np.deg2rad(41.0), 0.3],
            [-2.9, -0.2, 0.45, np.deg2rad(16.0), 0.3],
            [-2.9, 1.2, 0.45, np.deg2rad(-16.0), 0.3],
            [-2.5, 2.2, 0.45, np.deg2rad(-41.0), 0.3],
            [-2.1, 2.9, 0.45, np.deg2rad(-55.0), 0.3],
            [-1.9, 3.3, 0.45, np.deg2rad(-63.0), 0.3],
            [-1.0, 3.5, 0.45, np.deg2rad(-81.0), 0.3],
            [-0.0, 3.5, 0.45, np.deg2rad(-99.0), 0.3],
            [0.9, 3.3, 0.45, np.deg2rad(-117.0), 0.3],
            [1.1, 2.9, 0.45, np.deg2rad(-125.0), 0.3],
            [1.5, 2.2, 0.45, np.deg2rad(-139.0), 0.3],
            [1.9, 1.2, 0.45, np.deg2rad(-164.0), 0.3],
        """
        self.counter_clockwise_inspection = [ #   z    x    y
            # Half counter_clockwise 2m offset 
            [1.9, 0.2, 0.45, np.deg2rad(-164.0), 0.3],
            [1.9, -1.2, 0.45, np.deg2rad(164.0), 0.3],
            [1.5, -2.2, 0.45, np.deg2rad(139.0), 0.3],
            [1.1, -2.9, 0.45, np.deg2rad(125.0), 0.3],
            [0.9, -3.3, 0.45, np.deg2rad(117.0), 0.3],
            [-0.0, -3.5, 0.45, np.deg2rad(99.0), 0.3],
            [-1.0, -3.5, 0.45, np.deg2rad(81.0), 0.3],
            [-1.9, -3.3, 0.45, np.deg2rad(63.0), 0.3],
            [-2.1, -2.9, 0.45, np.deg2rad(55.0), 0.3],
            [-2.5, -2.2, 0.45, np.deg2rad(41.0), 0.3],
            [-2.1, -2.9, 0.45, np.deg2rad(55.0), 0.3],
            [-1.9, -3.3, 0.45, np.deg2rad(63.0), 0.3],
            [-1.0, -3.5, 0.45, np.deg2rad(81.0), 0.3],
            [-0.0, -3.5, 0.45, np.deg2rad(99.0), 0.3],
            [0.9, -3.3, 0.45, np.deg2rad(117.0), 0.3],
            [1.1, -2.9, 0.45, np.deg2rad(125.0), 0.3],
            [1.5, -2.2, 0.45, np.deg2rad(139.0), 0.3],
            [1.9, -1.2, 0.45, np.deg2rad(164.0), 0.3],
            ]
        
        """self.counter_clockwise_inspection = [ #   z    x    y 
            # Full Inspection
            [2.4, -1.3, 0.45, np.deg2rad(164.0), 0.3],
            [1.9, -2.6, 0.45, np.deg2rad(139.0), 0.3],
            [1.4, -3.3, 0.45, np.deg2rad(125.0), 0.3],
            [1.1, -3.7, 0.45, np.deg2rad(117.0), 0.3],
            [0.1, -4.0, 0.45, np.deg2rad(99.0), 0.3],
            [-1.1, -4.0, 0.45, np.deg2rad(81.0), 0.3],
            [-2.1, -3.7, 0.45, np.deg2rad(63.0), 0.3],
            [-2.4, -3.3, 0.45, np.deg2rad(55.0), 0.3],
            [-2.9, -2.6, 0.45, np.deg2rad(41.0), 0.3],
            [-3.4, -1.3, 0.45, np.deg2rad(16.0), 0.3],
            [-3.4, 0.3, 0.45, np.deg2rad(-16.0), 0.3],
            [-2.9, 1.6, 0.45, np.deg2rad(-41.0), 0.3],
            [-2.4, 2.3, 0.45, np.deg2rad(-55.0), 0.3],
            [-2.1, 2.7, 0.45, np.deg2rad(-63.0), 0.3],
            [-1.1, 3.0, 0.45, np.deg2rad(-81.0), 0.3],
            [0.1, 3.0, 0.45, np.deg2rad(-99.0), 0.3],
            [1.1, 2.7, 0.45, np.deg2rad(-117.0), 0.3],
            [1.4, 2.3, 0.45, np.deg2rad(-125.0), 0.3],
            [1.9, 1.6, 0.45, np.deg2rad(-139.0), 0.3],
            [2.4, 0.3, 0.45, np.deg2rad(-164.0), 0.3],
                                            ]"""
    
        self.clockwise_inspection = [ #   z    x    y
            # Half clockwise 2m offset 
            [1.9, 0.2, 0.45, np.deg2rad(-164.0), 0.3],
            [1.5, 1.2, 0.45, np.deg2rad(-139.0), 0.3],
            [1.1, 1.9, 0.45, np.deg2rad(-125.0), 0.3],
            [0.9, 2.3, 0.45, np.deg2rad(-117.0), 0.3],
            [-0.0, 2.5, 0.45, np.deg2rad(-99.0), 0.3],
            [-1.0, 2.5, 0.45, np.deg2rad(-81.0), 0.3],
            [-1.9, 2.3, 0.45, np.deg2rad(-63.0), 0.3],
            [-2.1, 1.9, 0.45, np.deg2rad(-55.0), 0.3],
            [-2.5, 1.2, 0.45, np.deg2rad(-41.0), 0.3],
            [-2.1, 1.9, 0.45, np.deg2rad(-55.0), 0.3],
            [-1.9, 2.3, 0.45, np.deg2rad(-63.0), 0.3],
            [-1.0, 2.5, 0.45, np.deg2rad(-81.0), 0.3],
            [-0.0, 2.5, 0.45, np.deg2rad(-99.0), 0.3],
            [0.9, 2.3, 0.45, np.deg2rad(-117.0), 0.3],
            [1.1, 1.9, 0.45, np.deg2rad(-125.0), 0.3],
            [1.5, 1.2, 0.45, np.deg2rad(-139.0), 0.3],
            [1.9, 0.2, 0.45, np.deg2rad(-164.0), 0.3],
            ]
        
        """
        self.clockwise_inspection = [ #   z    x    y
            # Full Inspection
            [1.9, 1.2, 0.45, np.deg2rad(-164.0), 0.3],
            [1.5, 2.2, 0.45, np.deg2rad(-139.0), 0.3],
            [1.1, 2.9, 0.45, np.deg2rad(-125.0), 0.3],
            [0.9, 3.3, 0.45, np.deg2rad(-117.0), 0.3],
            [-0.0, 3.5, 0.45, np.deg2rad(-99.0), 0.3],
            [-1.0, 3.5, 0.45, np.deg2rad(-81.0), 0.3],
            [-1.9, 3.3, 0.45, np.deg2rad(-63.0), 0.3],
            [-2.1, 2.9, 0.45, np.deg2rad(-55.0), 0.3],
            [-2.5, 2.2, 0.45, np.deg2rad(-41.0), 0.3],
            [-2.9, 1.2, 0.45, np.deg2rad(-16.0), 0.3],
            [-2.9, -0.2, 0.45, np.deg2rad(16.0), 0.3],
            [-2.5, -1.2, 0.45, np.deg2rad(41.0), 0.3],
            [-2.1, -1.9, 0.45, np.deg2rad(55.0), 0.3],
            [-1.9, -2.3, 0.45, np.deg2rad(63.0), 0.3],
            [-1.0, -2.5, 0.45, np.deg2rad(81.0), 0.3],
            [-0.0, -2.5, 0.45, np.deg2rad(99.0), 0.3],
            [0.9, -2.3, 0.45, np.deg2rad(117.0), 0.3],
            [1.1, -1.9, 0.45, np.deg2rad(125.0), 0.3],
            [1.5, -1.2, 0.45, np.deg2rad(139.0), 0.3],
            [1.9, -0.2, 0.45, np.deg2rad(164.0), 0.3],
            ]
            """
        # Initializing a list of waypoints
        self.waypoints_counter_clockwise_inspection  = self.waypoints_undocking + self.counter_clockwise_inspection + self.waypoints_docking
        self.waypoints_clockwise_inspection  = self.waypoints_undocking + self.clockwise_inspection + self.waypoints_docking

        
    def mod(self, x, y):
        return x - math.floor(x/y) * y
    
    # SSA is the "smallest signed angle" or the smallest difference between two angles.
    # Maps the angle to the interval [-pi, pi)
    def ssa(self, angle):
        return self.mod(angle + math.pi, 2 * math.pi) - math.pi
    
    def fetch_waypoint(self, current_pose):
        current_waypoint = self.waypoints[0]
        distance = np.linalg.norm(np.array(current_waypoint[:3]) - np.array(current_pose[:3]))
        heading_error = abs(self.ssa(current_waypoint[3] - current_pose[3]))
        
        if distance < current_waypoint[4] and heading_error < np.deg2rad(10):
            if len(self.waypoints) > 1:
                self.waypoints.pop(0)
                return True, self.waypoints[0]
        return False, current_waypoint
    
    def clear_waypoints(self, current_pose):
        self.waypoints = [current_pose]

    def append_waypoint(self, waypoint):
        self.waypoints.append(waypoint)

    def insert_waypoint(self, waypoint, index):
        self.waypoints.insert(index, waypoint)
    
    def remove_waypoint(self, index):
        if 0 <= index < len(self.waypoints) > 1:
            self.waypoints.pop(index)
        

class Controller:
    def __init__(self, KP, KI, KD, Integral_Max=0.2):
        self.Kp, self.Ki, self.Kd = KP, KI, KD
        self.integral, self.integral_max = 0.0, Integral_Max
        self.prev_time, self.prev_error = rclpy.time.Time().nanoseconds / 1e9, 0.0
    
    def compute(self, error, velocity, cap=5.0):
        current_time = rclpy.time.Time().nanoseconds / 1e9
        dt = current_time - self.prev_time
        
        self.integral = np.clip(self.integral + error * dt, -self.integral_max, self.integral_max)
        pid_output = self.Kp * error + self.Ki * self.integral - self.Kd * velocity
        
        self.prev_error, self.prev_time = error, current_time
        return np.clip(pid_output, -cap, cap)
 
class DockingNode(Node):
    """
    Node that subscribes to /odometry/filtered,
    calculates error in surge/sway/heave/yaw to the next waypoint,
    and publishes velocity commands to /blueye/ref_vel.
    """
    def __init__(self):
        super().__init__(
            node_name="docking_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=False,             
        )
                     
        # Choose between docking mission or specific waypoint following
        self.docking_mission = True
        
        
        # Timer for the reference filter (low pass filter)
        self.previous_time = time.perf_counter()
        self.first_update = True
        self.x_d_filtered_previous = 0.0
        self.y_d_filtered_previous = 0.0
        self.z_d_filtered_previous = 0.0

        # Control Mode Service
        self.dock_mode = False
        self.inspection_counter_clock = False
        self.inspection_clock = False
        self.undocking = False
        self.dock_mode_servic = self.create_service(SetBool, "/blueye/docking_mode", self.toogle_dock_controller)
        self.inspection_counter_clock_servic = self.create_service(SetBool, "blueye/inspection_counter_clock", self.toogle_inspection_counter_clock_controller)
        self.inspection_clock_servic = self.create_service(SetBool, "blueye/inspection_clock", self.toogle_inspection_clock_controller)



        # Publishers
        self.command_publisher = self.create_publisher(WrenchStamped, "/blueye/commands", 10)
        self.current_waypoint_publisher = self.create_publisher(Pose, "/blueye/current_waypoint", 10)
        self.current_reference_publisher = self.create_publisher(Pose, "/blueye/current_reference", 10)
        self.desired_pose_publisher = self.create_publisher(Pose, "/blueye/desired_pose", 10)
        self.yaw_publisher = self.create_publisher(TwistStamped, "/blueye/yaws", 10)
        
        # Subscribers
        self.battery_subscriber = self.create_subscription(Pose, "/blueye/battery", self.battery_callback, 10)
        self.filtered_odometry_subscriber = self.create_subscription(
            Odometry,
            "/blueye/navigation/current_state",
            self.filtered_odometry_callback,
            1
        )
        self.docking_detected_sub = self.create_subscription(
            Bool,
            '/blueye/docking_station_detected',
            self.docking_detected_callback,
            10
        )
        
        # Internal boolean for docking logic
        self.docking_detected = False

        
        # This shows how much current [A] that goes out or into the battery
        # Register charging state by checking that this is positive
        self.battery_current = -1.0
        
        # Needs adaptive handling of gains, we never know the conditions at
        
        """
        x = sway
        y = heave
        z = surge
        
        """
        self.surge_controller = Controller(1.3, 0.3, 1.8, 0.2) # Tuned, but can be improved
        self.sway_controller  = Controller(1.3, 0.3, 2.6, 0.2)
        # was: Controller(2.0, 0.3, 3.0, 0.2)
        # Old self.heave_controller = Controller(2.3, 0.35, 2.5, 0.2)
        self.heave_controller = Controller(2.7, 0.35, 2.5, 0.2)
        self.yaw_controller   = Controller(1.0, 0.3, 4.0, 0.2)
        
        # Waypoint logic
        self.waypoint_handler = WaypointHandler()
        self.previous_waypoint = []
        self.last_heading = 0.0

        # Variables to store desired and current states
        self.current_x = False # X-position [m] relative to tag frame*
        self.current_y = False # Y-position [m] relative to tag frame*
        self.current_z = False # Z-position [m] relative to tag frame*
        self.current_psi = False # Yaw angle [rad]
        self.current_u = False # Surge velocity
        self.current_v = False # Sway velocity
        self.current_w = False # Heave velocity
        self.current_r = False # Yaw rate
        self.valid_odometry = False # Flag to check if we have received odometry data

        # Set up service calls
        #self.append_waypoint = self.create_service(AppendWaypoint, "blueye/append_waypoint", self.append_waypoint_callback)
        #self.clear_waypoints = self.create_service(ClearWaypoints, "blueye/clear_waypoints", self.clear_waypoints_callback)
        #self.insert_waypoint = self.create_service(InsertWaypoint, "blueye/insert_waypoint", self.insert_waypoint_callback)
        #self.remove_waypoint = self.create_service(RemoveWaypoint, "blueye/remove_waypoint", self.remove_waypoint_callback)

        # Starting timer callback
        # OBS Might need to change the timer rate. How often do we want to update the control commands?
        self.timer = self.create_timer(0.1, self.command_callback)

    def toogle_dock_controller(self, request, response):
        print("Docking Mode: {}".format(request.data))
        self.dock_mode = request.data
        if self.dock_mode:
            self.previous_waypoint = True
            self.waypoint_handler.waypoints = self.waypoint_handler.waypoints_counter_clockwise_inspection
            self.previous_waypoint = self.waypoint_handler.waypoints[0]
        # self.waypoint_handler.waypoints.clear()
        response.success = True
        return response 
    
    def toogle_inspection_counter_clock_controller(self, request, response):
        print("Inspection Counter Clockwise Mode: {}".format(request.data))
        self.inspection_counter_clock = request.data
        if self.inspection_counter_clock:
            self.previous_waypoint = True
            self.waypoint_handler.waypoints = self.waypoint_handler.waypoints_counter_clockwise_inspection
            self.previous_waypoint = self.waypoint_handler.waypoints[0]
        # self.waypoint_handler.waypoints.clear()
        response.success = True
        return response 
    
    def toogle_inspection_clock_controller(self, request, response):
        print("Inspection Clockwise Mode: {}".format(request.data))
        self.inspection_clock = request.data
        if self.inspection_clock:
            self.previous_waypoint = True
            self.waypoint_handler.waypoints = self.waypoint_handler.waypoints_clockwise_inspection
            self.previous_waypoint = self.waypoint_handler.waypoints[0]
        # self.waypoint_handler.waypoints.clear()
        response.success = True
        return response 

    def docking_detected_callback(self, msg: Bool):
        self.docking_detected = msg.data
        #if self.docking_detected:
        #    self.get_logger().info("Docking station detected! Switching to docking logic.")
        
    # Callback for the battery topic
    def battery_callback(self, msg:Pose):
        self.battery_current = msg.position.z

    # Append Waypoint Callback
    def append_waypoint_callback(self, request, response):
        waypoint = [request.x, request.y, request.z]
        self.waypoint_handler.append_waypoint(waypoint)
        response.success = True
        return response
    
    # Clear Waypoints 
    def clear_waypoints_callback(self, request, response):
        waypoint = [self.current_x, self.current_y, self.current_z]
        self.waypoint_handler.clear_waypoints(waypoint)
        response.success = True
        return response
    
    # Insert Waypoint
    def insert_waypoint_callback(self, request, response):
        index = request.index 
        waypoint = [request.x, request.y, request.z]
        self.waypoint_handler.insert_waypoint(waypoint, index)
        response.success = True
        return response

    # Remove Waypoint
    def remove_waypoint_callback(self, request, response):
        index = request.index
        self.waypoint_handler.remove_waypoint(index)
        response.success = True
        return response 
    
    def mod(self, x, y):
        return x - math.floor(x/y) * y
    
    # SSA is the "smallest signed angle" or the smallest difference between two angles.
    # Maps the angle to the interval [-pi, pi)
    def ssa(self, angle):
        return self.mod(angle + math.pi, 2 * math.pi) - math.pi
    
    # Calculate the closest waypoint to the current position
    # 3D distance
    def closest_waypoint(self, current_position, waypoints):
        min_distance = float('inf')
        closest_wp = None
        for i, wp in enumerate(waypoints):
            distance = math.sqrt((current_position[0] - wp[0])**2 + (current_position[1] - wp[1])**2 + (current_position[2] - wp[2])**2)
            if distance < min_distance:
                min_distance = distance
                closest_wp = wp
                closest_i = i
        return closest_wp, closest_i, min_distance
    
    def command_callback(self):
        """
        Main control loop, runs every 0.1 s.
        Determines whether to switch to new waypoint,
        calculates errors in surge/sway/heave/yaw, and publishes control commands.
        """
        if not self.valid_odometry:
            return
        
        # Checks if any guide modes are activated 
        # if in manual return -> if all of the modes are false that means it is manual
        if (not self.dock_mode) and (not self.inspection_counter_clock) and (not self.inspection_clock) :
            return

        # Initialize previous waypoint
        if not self.previous_waypoint:
            self.previous_waypoint = [self.current_x, self.current_y, self.current_z, self.current_psi, 0.3]
            print(f"Setting previous wp: {self.previous_waypoint}")
            if self.docking_mission:
                if self.undocking:
                    self.waypoint_handler.waypoints = self.waypoint_handler.waypoints_undocking
                else:
                    print("we want to dock ....")
                    # Decide which loop (left or right) is closer
                    left_wp, i_left, dist_left = self.closest_waypoint(
                        self.previous_waypoint, self.waypoint_handler.waypoints_left_loop
                    )
                    right_wp, i_right, dist_right = self.closest_waypoint(
                        self.previous_waypoint, self.waypoint_handler.waypoints_right_loop
                    )
                    if dist_left < dist_right:
                        self.waypoint_handler.waypoints = self.waypoint_handler.waypoints_left_loop[i_left:]
                    else:
                        self.waypoint_handler.waypoints = self.waypoint_handler.waypoints_right_loop[i_right:]
            else:
                self.waypoint_handler.waypoints = self.waypoint_handler.waypoints_lawnmover_vert_1m
            potentially_previous_waypoint = self.waypoint_handler.waypoints[0]
        else:
            potentially_previous_waypoint = self.waypoint_handler.waypoints[0]

        # Check for waypoint completion
        update, waypoint = self.waypoint_handler.fetch_waypoint(
            [self.current_x, self.current_y, self.current_z, self.current_psi]
        )
        if update:
            self.previous_waypoint = potentially_previous_waypoint
            self.get_logger().info("Waypoint reached!")

        # Publish current waypoint
        current_waypoint_msg = Pose()
        current_waypoint_msg.position.x = float(waypoint[0])
        current_waypoint_msg.position.y = float(waypoint[1])
        current_waypoint_msg.position.z = float(waypoint[2])
        current_waypoint_msg.orientation.w = float(np.rad2deg(waypoint[3]))
        self.current_waypoint_publisher.publish(current_waypoint_msg)

        if len(waypoint) == 5:
            # Interpolate heading from previous_waypoint heading to new heading
            desired_heading_prev_wp = self.previous_waypoint[3]
            desired_heading_next_wp = waypoint[3]

            path_vector = np.array(waypoint[0:3]) - np.array(self.previous_waypoint[0:3])
            drone_vector = np.array([self.current_x, self.current_y, self.current_z]) - np.array(self.previous_waypoint[0:3])

            # Parametric distance along the line
            denom = np.dot(path_vector, path_vector)
            if abs(denom) < 1e-9:
                t = 0.0
            else:
                t = np.dot(drone_vector, path_vector) / denom
            t = max(0, min(1, t))

            if desired_heading_prev_wp < 0:
                desired_heading_prev_wp += 2.0 * math.pi
            if desired_heading_next_wp < 0:
                desired_heading_next_wp += 2.0 * math.pi

            # Weighted average heading
            psi_d = self.ssa((1 - t) * desired_heading_prev_wp + t * desired_heading_next_wp)

            # ------------------------------------------------------------
            # Low-pass filter for x_d, y_d, z_d
            # ------------------------------------------------------------
            x_d = waypoint[0]
            y_d = waypoint[1]
            z_d = waypoint[2]

            current_time = time.perf_counter()
            dt = current_time - self.previous_time
            T = 7.0  # filter time constant
            alpha = math.exp(-dt / T)

            if self.first_update:
                self.x_d_filtered_previous = self.current_x
                self.y_d_filtered_previous = self.current_y
                self.z_d_filtered_previous = self.current_z
                self.first_update = False

            x_d_filtered = alpha * self.x_d_filtered_previous + (1 - alpha) * x_d
            y_d_filtered = alpha * self.y_d_filtered_previous + (1 - alpha) * y_d
            z_d_filtered = alpha * self.z_d_filtered_previous + (1 - alpha) * z_d

            self.x_d_filtered_previous = x_d_filtered
            self.y_d_filtered_previous = y_d_filtered
            self.z_d_filtered_previous = z_d_filtered
            self.previous_time = current_time

            # Publish filtered reference
            current_reference_msg = Pose()
            current_reference_msg.position.x = float(x_d_filtered)
            current_reference_msg.position.y = float(y_d_filtered)
            current_reference_msg.position.z = float(z_d_filtered)
            current_reference_msg.orientation.w = float(np.rad2deg(psi_d))
            self.current_reference_publisher.publish(current_reference_msg)

            # Calculate error in tag* frame
            delta_x = x_d_filtered - self.current_x
            delta_y = y_d_filtered - self.current_y
            delta_z = z_d_filtered - self.current_z

            # If battery_current > 0, presumably we are charging => no movement
            if self.battery_current <= 0.0:
                # Transform error into body frame
                error_surge = delta_x * math.cos(self.current_psi) + delta_y * math.sin(self.current_psi)
                error_sway = delta_x * math.sin(self.current_psi) - delta_y * math.cos(self.current_psi)
                error_heave = -delta_z  # z-axis sign difference
                error_yaw = self.ssa(-(psi_d - self.current_psi))
            else:
                error_surge = 0.0
                error_sway = 0.0
                error_heave = 0.0
                error_yaw = 0.0

            # Potentially saturate surge/sway if yaw or z error is too large
            limit_heave = 0.3
            limit_yaw = np.deg2rad(4)
            gamma_e = 0.7
            delta_r = 0.0

            if abs(error_heave) > limit_heave and abs(error_yaw) > limit_yaw:
                delta_r = (abs(error_heave) - limit_heave)
            elif abs(error_heave) <= limit_heave and abs(error_yaw) > limit_yaw:
                delta_r = (abs(error_yaw) - limit_yaw)
            elif abs(error_heave) > limit_heave and abs(error_yaw) > limit_yaw:
                delta_r = (abs(error_heave) - limit_heave) + (abs(error_yaw) - limit_yaw)

            lambda_cap = math.exp(delta_r * math.log(gamma_e))

            # Compute final commands
            self.surge = self.surge_controller.compute(error_surge, self.current_u, 5.0)
            self.sway = self.sway_controller.compute(error_sway, -self.current_v, 5.0)
            self.heave = self.heave_controller.compute(error_heave, -self.current_w)
            self.yaw = self.yaw_controller.compute(error_yaw, -self.current_r)

            # Publish twist command
            wrench_msg = WrenchStamped()
            wrench_msg.header.stamp = self.get_clock().now().to_msg()
            wrench_msg.header.frame_id = "body_frame"

            # Minus sign because the eskf waypoints are in NED instead of ENU
            wrench_msg.wrench.force.x = float(self.surge)
            wrench_msg.wrench.force.y = -float(self.sway)
            wrench_msg.wrench.force.z = -float(self.heave) 
            wrench_msg.wrench.torque.z = -float(self.yaw) + 0.15
            
            #print("Surge: {}, Sway: {}, Heave: {} and Yaw; {}".format(self.surge, self.sway, self.heave, self.yaw))


            self.command_publisher.publish(wrench_msg)
            #print("Published Wrench Message")

        else:
            self.get_logger().warn("Received waypoint missing five elements [x, y, z, yaw, threshold].")
    
    def filtered_odometry_callback(self, msg: Odometry):
        self.current_x, self.current_y, self.current_z = msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        self.current_psi = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        #print("Current yaw: {}".format(self.current_psi))
        self.current_u, self.current_v, self.current_w = msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z
        self.current_r = msg.twist.twist.angular.z
        self.valid_odometry = True

       
def main(args=None):
    print("Docking node started ....")
    rclpy.init()
    controller_node = DockingNode()
    rclpy.spin(controller_node)
    controller_node.keyboard.stop()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
