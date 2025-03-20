#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
import numpy as np

class DepthVisualizer(Node):
    def __init__(self):
        super().__init__('depth_visualizer')
        
        # History lists: time (in seconds) and reversed depth (i.e. -z)
        self.time_history = []
        self.trajectory_depth = []  # will store -z values
        
        self.max_trajectory_points = 1000  # maximum number of samples
        
        # Setup a 2D plot: x-axis = time, y-axis = depth (reversed)
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Depth (m)')
        self.ax.set_title('ROV Depth Over Time (Reversed)')
        
        # Initialize the red depth line and blue marker for current depth
        self.depth_line, = self.ax.plot([], [], color='red', linewidth=2, label='Depth')
        self.current_marker, = self.ax.plot([], [], 'bo', markersize=8, label='Current Depth')
        self.ax.legend()
        
        # Subscribe to the odometry topic (using /blueye/odometry_frd/gt)
        self.subscription = self.create_subscription(
            Odometry,
            '/blueye/odometry_frd/gt',
            self.odometry_callback,
            10
        )
        
        # Create animation that updates the plot every 100 ms
        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=100, blit=False)
        plt.show(block=False)
    
    def odometry_callback(self, msg):
        # Extract timestamp from header (in seconds)
        time_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Extract the depth (z coordinate) and reverse it (so descending appears lower)
        pos = msg.pose.pose.position
        reversed_depth = -pos.z
        
        self.time_history.append(time_stamp)
        self.trajectory_depth.append(reversed_depth)
        
        # Limit history size to prevent overflow
        if len(self.trajectory_depth) > self.max_trajectory_points:
            self.time_history.pop(0)
            self.trajectory_depth.pop(0)
    
    def update_plot(self, frame):
        if self.time_history and self.trajectory_depth:
            t = np.array(self.time_history)
            d = np.array(self.trajectory_depth)
            
            # Update the red line for depth history and the blue marker for current depth
            self.depth_line.set_data(t, d)
            self.current_marker.set_data(t[-1], d[-1])
            
            # Adjust x-axis to span the recorded time and add a small buffer at the end
            self.ax.set_xlim(t[0], t[-1] + 1)
            # Adjust y-axis based on current data range with a small margin
            d_min = np.min(d)
            d_max = np.max(d)
            self.ax.set_ylim(d_min - 1, d_max + 1)
        
        return self.depth_line, self.current_marker

def main(args=None):
    rclpy.init(args=args)
    visualizer = DepthVisualizer()
    
    # Run the ROS spin loop in a separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(visualizer,), daemon=True)
    spin_thread.start()
    
    try:
        plt.show()  # Keeps the matplotlib window open
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
