#!/usr/bin/env python3

import sys
from threading import Thread
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from mundus_mir_msgs.msg import BatteryStatus
from mundus_mir_msgs.msg import ReturnRecommendation
from std_msgs.msg import Float64

from PyQt6.QtWidgets import (
    QApplication,
    QMainWindow,
    QVBoxLayout,
    QWidget,
    QLabel,
)
from PyQt6.QtCore import QTimer
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

class BlueyeGUI(Node):
    def __init__(self):
        super().__init__('blueye_gui')
        
        # Initialize return recommendation data
        self.return_recommendation = {
            'should_return': False,
            'current_battery_level': 100.0,
            'distance_to_dock': 0.0,
            'current_speed': 0.0,
            'current_consumption_rate': 0.0,
            'estimated_return_energy': 0.0,
            'estimated_time_to_return': 0.0,
            'minimum_battery_needed': 0.0,
            'safety_margin_percent': 0.0,
            'battery_safety_threshold': 20.0
        }
        # Hardcoded waypoints - convert to numpy arrays right away
        self.waypoints = np.array([
            [-8.5, 8.54, 95.3],    # Docking station
            [-18.0, -25.0, 90.0],  # Pipeline point 1
            # [20.0, -25.0, 90.0]   # Pipeline point 2
            [20.0, 55.0, 90.0]   # Pipeline point 2

        ])
        
        # Initialize position history with empty numpy arrays
        self.position_history = {
            'x': np.array([self.waypoints[0][0]]),  # Start at first waypoint
            'y': np.array([self.waypoints[0][1]]),
            'z': np.array([self.waypoints[0][2]])
        }
        
        # Initialize battery status
        self.battery_level = 100.0
        
        # Create subscriptions
        self.create_subscription(
            Odometry,
            '/blueye/odometry_frd/gt',
            self.odometry_callback,
            10
        )
        
        self.create_subscription(
            BatteryStatus,
            '/blueye/battery',
            self.battery_callback,
            10
        )
        
        self.create_subscription(
            ReturnRecommendation,
            '/blueye/return_recommendation',
            self.return_recommendation_callback,
            10
        )
        
        # Set maximum history length to prevent memory overflow
        self.max_history_length = 1000
        
                # Add distance tracking
        self.distance_to_dock = 0.0
        
        # Add subscription to distance
        self.create_subscription(
            Float64,
            '/blueye/distance_to_dock',
            self.distance_callback,
            10
        )

    def distance_callback(self, msg):
        """Handle incoming distance data"""
        self.distance_to_dock = msg.data

    def odometry_callback(self, msg):
        """Handle incoming odometry data"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        # Convert to numpy arrays and append
        self.position_history['x'] = np.append(self.position_history['x'], x)
        self.position_history['y'] = np.append(self.position_history['y'], y)
        self.position_history['z'] = np.append(self.position_history['z'], z)

    def battery_callback(self, msg):
        """Handle incoming battery status"""
        self.battery_level = msg.state_of_charge * 100

    def return_recommendation_callback(self, msg):
        """Handle incoming return recommendation data"""
        self.return_recommendation['should_return'] = msg.should_return
        self.return_recommendation['current_battery_level'] = msg.current_battery_level
        self.return_recommendation['distance_to_dock'] = msg.distance_to_dock
        self.return_recommendation['current_speed'] = msg.current_speed
        self.return_recommendation['current_consumption_rate'] = msg.current_consumption_rate
        self.return_recommendation['estimated_return_energy'] = msg.estimated_return_energy
        self.return_recommendation['estimated_time_to_return'] = msg.estimated_time_to_return
        self.return_recommendation['minimum_battery_needed'] = msg.minimum_battery_needed
        self.return_recommendation['safety_margin_percent'] = msg.safety_margin_percent
        self.return_recommendation['battery_safety_threshold'] = msg.battery_safety_threshold
        
    
class ROVPlotCanvas(FigureCanvas):
    def __init__(self, parent=None, width=100, height=10, dpi=100):
        # Create figure with tight layout to use maximum space
        fig = plt.figure(figsize=(width, height), dpi=dpi)
        fig.tight_layout()
        
        # Create 3D axes with more space
        self.axes = fig.add_subplot(111, projection='3d', computed_zorder=False)
        super().__init__(fig)
        
        # Set fixed view limits with some padding around waypoints
        self.x_limits = [-100, 100]
        self.y_limits = [-100, 100]
        self.z_limits = [85, 100]
        
        self.axes.set_xlabel('X [m]')
        self.axes.set_ylabel('Y [m]')
        self.axes.set_zlabel('Depth [m]')
        self.axes.set_title('ROV Position and Waypoints')
        
        # Initialize plot elements with initial dummy data
        self.path_line, = self.axes.plot3D([], [], [], 'b-', label='Blueye Path', linewidth=2, zorder=1)
        self.current_pos, = self.axes.plot3D([], [], [], 'yo', label='Blueye Position', 
                                           markersize=10, zorder=3)
        self.waypoints_plots = []  # Will store separate scatter plots for different waypoint types
        
        self.axes.set_xlim(self.x_limits)
        self.axes.set_ylim(self.y_limits)
        self.axes.set_zlim(self.z_limits[1], self.z_limits[0])  # Flip Z axis
        
        # Set the view angle for better visualization
        self.axes.view_init(elev=25, azim=45)
        
        # Make the grid appear behind all plots
        self.axes.grid(True, zorder=0)
        self.axes.legend()

    def plot_waypoints(self, waypoints):
        """Plot the waypoints with different symbols and labels"""
        if not self.waypoints_plots:
            # Plot docking station (first waypoint) with a square
            dock_scatter = self.axes.scatter(
                [waypoints[0, 0]],
                [waypoints[0, 1]],
                [waypoints[0, 2]],
                c='r',
                marker='s',  # square marker
                s=100,
                label='Docking Station',
                zorder=2
            )
            self.waypoints_plots.append(dock_scatter)
            
            # Add text label for docking station
            self.axes.text(
                waypoints[0, 0], 
                waypoints[0, 1], 
                waypoints[0, 2] + 1,  # Slightly above the point
                'Docking Station',
                zorder=2
            )
            
            # Plot pipeline points (remaining waypoints) with diamonds
            pipeline_scatter = self.axes.scatter(
                waypoints[1:, 0],
                waypoints[1:, 1],
                waypoints[1:, 2],
                c='g',
                marker='s',  # diamond marker
                s=100,
                label='Pipeline',
                zorder=2
            )
            self.waypoints_plots.append(pipeline_scatter)
            
            # # Add text labels for pipeline points
            # for i, point in enumerate(waypoints[1:], 1):
            #     self.axes.text(
            #         point[0],
            #         point[1],
            #         point[2] - 2, 
            #         f'Pipeline {i}',
            #         zorder=2
            #     )
            
            self.axes.legend(loc='upper right')

    def update_plot(self, position_history):
        """Update the plot with new position data"""
        try:
            if len(position_history['x']) > 0:
                # Convert lists to numpy arrays for plotting
                x = np.array(position_history['x'])
                y = np.array(position_history['y'])
                z = np.array(position_history['z'])
                
                # Update path line - use the full history
                self.path_line.set_data_3d(x, y, z)
                
                # Update current position
                self.current_pos.set_data_3d(
                    [x[-1]],
                    [y[-1]],
                    [z[-1]]
                )
                
                self.draw_idle()
        except Exception as e:
            print(f"Error updating plot: {e}")
            
            
class MainWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Blueye ROV Visualization")
        
        # Create main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout(main_widget)
        
        # Create plot canvas
        self.canvas = ROVPlotCanvas(main_widget)
        layout.addWidget(self.canvas)
        
        # Plot waypoints
        self.canvas.plot_waypoints(self.node.waypoints)
        
        # Create battery status label
        self.battery_label = QLabel("Battery Level: 100%")
        layout.addWidget(self.battery_label)
        
        # Create position label
        self.position_label = QLabel("Position: No data")
        layout.addWidget(self.position_label)
        
        # Create distance label
        self.distance_label = QLabel("Distance to Dock: No data")
        layout.addWidget(self.distance_label)
        
        # Set up update timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(10)  # Update every 100ms
        
        self.return_status_label = QLabel("Return Status: Not Available")
        layout.addWidget(self.return_status_label)
        
        self.energy_info_label = QLabel("Energy Information: Not Available")
        layout.addWidget(self.energy_info_label)
        
        self.consumption_label = QLabel("Consumption Information: Not Available")
        layout.addWidget(self.consumption_label)
        
        # Set window size
        self.resize(1600, 1000)
        # self.setMinimumSize(800, 800)

    def update_gui(self):
        """Update the GUI elements"""
        # Update plot
        self.canvas.update_plot(self.node.position_history)
        
        # Update battery label
        self.battery_label.setText(f"Battery Level: {self.node.battery_level:.2f}%")
        
        # Update position label
        if len(self.node.position_history['x']) > 0:
            pos_x = self.node.position_history['x'][-1]
            pos_y = self.node.position_history['y'][-1]
            pos_z = self.node.position_history['z'][-1]
            self.position_label.setText(
                f"Position: X: {pos_x:.2f}m, Y: {pos_y:.2f}m, Depth: {pos_z:.2f}m"
            )
        # Update distance label
        self.distance_label.setText(f"Distance to Dock: {self.node.distance_to_dock:.2f}m")

        self.canvas.update_plot(self.node.position_history)
        self.battery_label.setText(f"Battery Level: {self.node.battery_level:.2f}%")
        
        if len(self.node.position_history['x']) > 0:
            pos_x = self.node.position_history['x'][-1]
            pos_y = self.node.position_history['y'][-1]
            pos_z = self.node.position_history['z'][-1]
            self.position_label.setText(
                f"Position: X: {pos_x:.2f}m, Y: {pos_y:.2f}m, Depth: {pos_z:.2f}m"
            )
        
        self.distance_label.setText(f"Distance to Dock: {self.node.distance_to_dock:.2f}m")

        # Add return recommendation updates
        status_color = "red" if self.node.return_recommendation['should_return'] else "green"
        self.return_status_label.setText(
            f"Return Status: <span style='color: {status_color};'>"
            f"{'RETURN RECOMMENDED' if self.node.return_recommendation['should_return'] else 'CONTINUE MISSION'}</span>"
        )
        
        self.energy_info_label.setText(
            f"Energy Information:\n"
            f"  Estimated Return Energy: {self.node.return_recommendation['estimated_return_energy']:.2f}%\n"
            f"  Minimum Battery Needed: {self.node.return_recommendation['minimum_battery_needed']:.2f}%\n"
            f"  Safety Margin: {self.node.return_recommendation['safety_margin_percent']:.2f}%"
        )
        
        self.consumption_label.setText(
            f"Consumption Information:\n"
            f"  Current Speed: {self.node.return_recommendation['current_speed']:.2f} m/s\n"
            f"  Consumption Rate: {self.node.return_recommendation['current_consumption_rate']:.3f}%/s\n"
            f"  Estimated Return Time: {self.node.return_recommendation['estimated_time_to_return']:.2f}s"
        )

def main(args=None):
    rclpy.init(args=args)
    node = BlueyeGUI()
    
    # Set up the executor in a separate thread
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    # Set up the Qt application
    app = QApplication(sys.argv)
    window = MainWindow(node)
    window.show()
    
    try:
        app.exec()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()