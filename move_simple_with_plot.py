import math
import csv
from datetime import datetime
import os
from typing import List, Tuple, Dict

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import tf_transformations

class MoveSimple(Node):
    def __init__(self):
        super().__init__('move_simple')
        # Existing initializations
        self.current_pose = None
        self.current_twist_linear = None
        self.current_twist_angular = None
        self.send_once = True
        self.stop_reached = False
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.x_stop = 0.0
        self.y_stop = 0.0
        self.timer = None

        # Add path tracking
        self.trajectory_data = []
        self.current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = f'robot_trajectory_{self.current_time}.csv'
        
        self._create_subscribers()
        self._create_publishers()
        self.get_logger().info('MoveSimple node initialized with parameters')

    def log_position(self):
        """Log current position and calculate error from ideal path"""
        if self.current_pose is not None:
            current_x = self.current_pose.position.x
            current_y = self.current_pose.position.y
            
            # Calculate error (distance from ideal straight line path)
            error = self.calculate_path_error(current_x, current_y)
            
            data_point = {
                'timestamp': self.get_clock().now().to_msg().sec,
                'robot_x': current_x,
                'robot_y': current_y,
                'ideal_x': current_x,  # Ideal x position on straight line
                'ideal_y': 0.0,        # Ideal y should be 0 for straight line
                'error': error,
                'linear_vel': self.current_twist_linear.x,
                'angular_vel': self.current_twist_angular.z
            }
            self.trajectory_data.append(data_point)

    def _create_subscribers(self):
        self.pose_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
            
        
    def _create_publishers(self):
        self.vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
        
    def set_cmd_vel(self, linear_vel: float, angular_vel: float):
        self.linear_vel = linear_vel
        self.angular_vel = angular_vel
        
    def set_stop(self, x_stop: float, y_stop: float):
        self.x_stop = x_stop
        self.y_stop = y_stop

    def send_velocity_command(self):
        """Send a single velocity command and then stop"""
        msg = Twist()
        msg.linear.x = self.linear_vel
        msg.angular.z = self.angular_vel
        self.vel_publisher.publish(msg)

    def calculate_path_error(self, current_x: float, current_y: float) -> float:
        """Calculate error from ideal straight line path"""
        # For straight line motion, error is simply the y-coordinate
        return abs(current_y)

    def save_trajectory_data(self):
        """Save trajectory data to CSV file"""
        try:
            with open(self.csv_filename, 'w', newline='') as csvfile:
                fieldnames = ['timestamp', 'robot_x', 'robot_y', 'ideal_x', 'ideal_y', 
                            'error', 'linear_vel', 'angular_vel']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                for data_point in self.trajectory_data:
                    writer.writerow(data_point)
            self.get_logger().info(f'Trajectory data saved to {self.csv_filename}')
        except Exception as e:
            self.get_logger().error(f'Error saving trajectory data: {e}')

    def control_loop(self):
        """Modified control loop to include position logging"""
        if self.current_pose is not None and not self.stop_reached:
            position_reached = abs(self.current_pose.position.x - self.x_stop) < 0.1
            
            # Log position data
            self.log_position()
            
            self.get_logger().info(
                f'Current position: x={self.current_pose.position.x:.2f}, '
                f'y={self.current_pose.position.y:.2f}, '
                f'Forward vel={self.current_twist_linear.x:.2f}, '
                f'Rotation={self.current_twist_angular.z:.2f}'
            )
            
            if position_reached:
                self.stop_control_loop()
                self.save_trajectory_data()
                self.get_logger().info(
                    f'Stop condition reached at x={self.current_pose.position.x:.2f}'
                )
                return

            if self.send_once:
                self.send_once = False
                self.send_velocity_command()
                self.get_logger().info(
                    f'Started moving with vel={self.linear_vel:.2f}, '
                    f'ang={self.angular_vel:.2f}'
                )


    def stop_control_loop(self):
        """Stop the control loop and cleanup"""
        try:
            self.stop_reached = True
            if self.timer:
                self.timer.cancel()
                self.timer.destroy()
            # Send final stop command
            self.set_cmd_vel(0.0, 0.0)
            self.send_velocity_command()
            # Save the trajectory data
            self.save_trajectory_data()
        except Exception as e:
            self.get_logger().error(f'Error stopping control loop: {e}')


    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.current_twist_linear = msg.twist.twist.linear
        self.current_twist_angular = msg.twist.twist.angular

    def __del__(self):
        """Cleanup when object is deleted"""
        if hasattr(self, 'timer') and self.timer:
            self.timer.destroy()


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Robot Simple Controller')
    parser.add_argument('--x-vel', type=float, default=0.5, help='Linear velocity')
    parser.add_argument('--z-ang', type=float, default=0.0, help='Rotation of the robot')
    parser.add_argument('--x-stop', type=float, default=10.0, help='X coordinate to stop at')
    args = parser.parse_args()
    
    rclpy.init()
    node = MoveSimple()
    node.set_cmd_vel(args.x_vel, args.z_ang)
    node.set_stop(args.x_stop, 0)
    
    timer_period = 0.1  # seconds
    timer = node.create_timer(timer_period, node.control_loop)
    node.timer = timer
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:   
        node.save_trajectory_data()  # Save data before shutting down
        node.stop_control_loop()
        node.destroy_node()
        rclpy.shutdown()
        node.get_logger().info('Node destroyed and shutdown complete')

if __name__ == '__main__':
    main()