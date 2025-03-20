#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading

class MatplotlibVisualizer(Node):
    def __init__(self):
        super().__init__('matplotlib_visualizer')
        
        # ROV initial position and orientation (using x, y, z; we only use x and y for 2D)
        self.rov_position = np.array([-180.0, 130.0, 193.0])
        self.rov_orientation = np.array([0.0, 0.0, 0.0, 1.0])
        
        # Trajectory: store positions over time
        self.trajectory = [self.rov_position.copy()]
        self.max_trajectory_points = 100

        # Static elements (2D: use x and y only)
        self.docking_station = np.array([-180.0, 130.0, 193.0])
        self.pipeline = np.array([
            [-175.0, 125.0, 194.0],
            [-170.0, 126.0, 194.5],
            [-165.0, 127.0, 195.0]
        ])
        self.waypoints = np.array([
            [-175.0, 125.0, 194.0],
            [-170.0, 126.0, 194.5]
        ])
        
        # Setup 2D plot
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111)  # 2D axes
        
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('ROV Mission Visualization (2D)')
        
        # Plot static elements once
        self.ax.scatter(
            self.docking_station[0], self.docking_station[1],
            color='red', s=200, marker='s', label='Docking Station'
        )
        self.ax.plot(
            self.pipeline[:, 0], self.pipeline[:, 1],
            color='orange', linewidth=3, label='Pipeline'
        )
        if self.waypoints.size > 0:
            self.ax.scatter(
                self.waypoints[:, 0], self.waypoints[:, 1],
                color='green', s=100, marker='^', label='Waypoints'
            )
        
        # Initialize trajectory (red line) and current ROV marker
        traj_array = np.array(self.trajectory)
        self.trajectory_line, = self.ax.plot(
            traj_array[:, 0], traj_array[:, 1],
            color='red', linewidth=2, label='Trajectory'
        )
        self.rov_marker, = self.ax.plot(
            self.rov_position[0], self.rov_position[1],
            'bo', markersize=8, label='ROV'
        )
        
        # Set fixed axis limits (optional)
        self.ax.set_xlim(self.rov_position[0] - 50, self.rov_position[0] + 50)
        self.ax.set_ylim(self.rov_position[1] - 50, self.rov_position[1] + 50)
        self.ax.legend()
        
        # Subscribe to ROV odometry topic
        self.subscription = self.create_subscription(
            Odometry,
            '/blueye/odometry_frd/gt',
            self.odometry_callback,
            10
        )
        
        # Create animation: update_plot is called every 100 ms
        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=100, blit=False
        )
        
        # Show plot in non-blocking mode
        plt.show(block=False)
    
    def odometry_callback(self, msg):
        # Update ROV position (x, y, z) and orientation
        pos = msg.pose.pose.position
        quat = msg.pose.pose.orientation
        self.rov_position = np.array([pos.x, pos.y, pos.z])
        self.rov_orientation = np.array([quat.x, quat.y, quat.z, quat.w])
        
        # Append new position to the trajectory history
        self.trajectory.append(self.rov_position.copy())
        if len(self.trajectory) > self.max_trajectory_points:
            self.trajectory.pop(0)
    
    def update_plot(self, frame):
        # Convert trajectory history to numpy array
        traj_array = np.array(self.trajectory)
        # Update the trajectory line (x and y only)
        self.trajectory_line.set_data(traj_array[:, 0], traj_array[:, 1])
        # Update the ROV marker position (x and y only)
        self.rov_marker.set_data(self.rov_position[0], self.rov_position[1])
        # (Optional) Adjust axis limits to follow the ROV if desired:
        # self.ax.set_xlim(self.rov_position[0] - 50, self.rov_position[0] + 50)
        # self.ax.set_ylim(self.rov_position[1] - 50, self.rov_position[1] + 50)
        return self.trajectory_line, self.rov_marker

def main(args=None):
    rclpy.init(args=args)
    visualizer = MatplotlibVisualizer()
    
    # Run ROS spin in a separate thread so the callbacks are processed
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
