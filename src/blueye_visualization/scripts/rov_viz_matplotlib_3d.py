#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (for 3D projection)
import matplotlib.animation as animation
import threading

class MatplotlibVisualizer(Node):
    def __init__(self):
        super().__init__('matplotlib_visualizer')
        
        # ROV initial position and orientation
        self.rov_position = np.array([-180.0, 130.0, 193.0])
        self.rov_orientation = np.array([0.0, 0.0, 0.0, 1.0])
        
        # Trajectory: store positions over time
        self.trajectory = [self.rov_position.copy()]
        self.max_trajectory_points = 100

        # Static elements
        self.docking_station = np.array([-180.0, 130.0, 193.0])
        self.pipeline = np.array([
            [-175.0, 125.0, 194.0],
            [-170.0, 126.0, 194.5],
            [-165.0, 127.0, 195.0]
        ])
        
        # Waypoints
        self.waypoints = np.array([
            [-175.0, 125.0, 194.0],
            [-170.0, 126.0, 194.5]
        ])
        
        # Setup plot (create figure and 3D axes)
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Plot static elements once
        self.ax.scatter(
            self.docking_station[0], self.docking_station[1], self.docking_station[2],
            color='red', s=200, marker='s', label='Docking Station'
        )
        self.ax.plot(
            self.pipeline[:, 0], self.pipeline[:, 1], self.pipeline[:, 2],
            color='orange', linewidth=3, label='Pipeline'
        )
        if self.waypoints.size > 0:
            self.ax.scatter(
                self.waypoints[:, 0], self.waypoints[:, 1], self.waypoints[:, 2],
                color='green', s=100, marker='^', label='Waypoints'
            )
        
        # Initialize trajectory line and current position marker
        self.trajectory_line, = self.ax.plot([], [], [], color='cyan', linewidth=1, label='Trajectory')
        # For scatter, we cannot update data directly so we will remove and re-add it
        self.rov_scatter = self.ax.scatter(
            self.rov_position[0], self.rov_position[1], self.rov_position[2],
            color='blue', s=100, marker='o', label='ROV'
        )
        
        # Set axis labels and title
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Z (m)')
        self.ax.set_title('ROV Mission Visualization')
        
        # Set fixed axis limits based on initial position (to prevent auto-scaling)
        center = self.rov_position
        self.ax.set_xlim(center[0] - 20, center[0] + 20)
        self.ax.set_ylim(center[1] - 20, center[1] + 20)
        self.ax.set_zlim(center[2] - 10, center[2] + 10)
        
        self.ax.legend()
        
        # Subscribe to ROV position topic
        self.subscription = self.create_subscription(
            Odometry,
            '/blueye/odometry_frd/gt',
            self.odometry_callback,
            10
        )
        
        # Create animation that calls update_plot periodically
        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=100, blit=False
        )
        
        # Show plot without blocking the ROS spin thread
        plt.show(block=False)
    
    def odometry_callback(self, msg):
        # Update ROV position and orientation from incoming odometry message
        pos = msg.pose.pose.position
        quat = msg.pose.pose.orientation
        
        self.rov_position = np.array([pos.x, pos.y, pos.z])
        self.rov_orientation = np.array([quat.x, quat.y, quat.z, quat.w])
        
        # Append new position to the trajectory history
        self.trajectory.append(self.rov_position.copy())
        if len(self.trajectory) > self.max_trajectory_points:
            self.trajectory.pop(0)
    
    def update_plot(self, frame):
        # Update trajectory line with the current history
        traj_array = np.array(self.trajectory)
        self.trajectory_line.set_data(traj_array[:, 0], traj_array[:, 1])
        self.trajectory_line.set_3d_properties(traj_array[:, 2])
        
        # Update current ROV position: remove previous scatter and re-plot
        try:
            self.ax.collections.remove(self.rov_scatter)
        except ValueError:
            # In case the collection was already removed
            pass
        self.rov_scatter = self.ax.scatter(
            self.rov_position[0], self.rov_position[1], self.rov_position[2],
            color='blue', s=100, marker='o', label='ROV'
        )
        
        # Optionally update axis limits if you wish to follow the ROV
        center = self.rov_position
        self.ax.set_xlim(center[0] - 20, center[0] + 20)
        self.ax.set_ylim(center[1] - 20, center[1] + 20)
        self.ax.set_zlim(center[2] - 10, center[2] + 10)
        
        return self.trajectory_line, self.rov_scatter

def main(args=None):
    rclpy.init(args=args)
    visualizer = MatplotlibVisualizer()
    
    # Start ROS spin loop in a separate thread so callbacks are processed
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
