import math
import csv
import argparse
from datetime import datetime
import os
from typing import List, Tuple, Dict
from enum import Enum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import tf_transformations

# move_simple.py --x-vel 0.5 --z-ang 0.0 --x-stop 1.0

class MoveSimple(Node):
    def __init__(self):
        super().__init__('move_simple')
        self.current_pose = None
        self.current_twist_linear = None
        self.current_twist_angular = None
        self.send_once = True
        self.stop_reached = False
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.x_stop = 0.0
        self.y_stop = 0.0
        self.timer = None  # Add timer reference
        self._create_subscribers()
        self._create_publishers()
        self.get_logger().info('MoveSimple node initialized with parameters')

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
        except Exception as e:
            self.get_logger().error(f'Error stopping control loop: {e}')
        
    def control_loop(self):
        """Timer callback for control loop"""
        if self.current_pose is not None and not self.stop_reached:
            # Check both position and velocity for stop condition
            position_reached = abs(self.current_pose.position.x - self.x_stop) < 0.1
            # velocity_low = abs(self.current_twist_linear.x) < 0.01
            self.get_logger().info(
                f'Current position: x={self.current_pose.position.x:.2f}, '
                f'y={self.current_pose.position.y:.2f}, '
                f'Foward vel={self.current_twist_linear.x:.2f}, '
                f'Rotation={self.current_twist_angular.z:.2f}'
            )
            if position_reached:
                self.stop_control_loop()
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
    # Initialize the ROS 2 node
    rclpy.init()
    node = MoveSimple()
    node.set_cmd_vel(args.x_vel, args.z_ang)
    node.set_stop(args.x_stop, 0)  # now just working with stop
    
    # Create a timer to run the control loop
    timer_period = 0.1  # seconds
    timer = node.create_timer(timer_period, node.control_loop)
    node.timer = timer  # Store timer reference
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:   
        node.stop_control_loop()
        node.destroy_node()
        rclpy.shutdown()
        node.get_logger().info('Node destroyed and shutdown complete')


    


if __name__ == '__main__':
    main()