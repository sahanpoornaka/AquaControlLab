#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class RobotDriver(Node):
    def __init__(self, goals):
        super().__init__('robot_driver')
        
        # PID control variables for heading
        self.heading_error_prev = 0.0     # Previous heading error
        self.heading_error_sum = 0.0      # Accumulated heading error
        self.heading_error_sum_max = 2.0  # Anti-windup limit
        
        # PID control variables for velocity
        self.dist_error_prev = 0.0        # Previous distance error
        self.dist_error_sum = 0.0         # Accumulated distance error
        self.dist_error_sum_max = 2.0     # Anti-windup limit
        
        # PID gains for heading
        self.kp_heading = 0.5             # Proportional gain
        self.ki_heading = 0.03            # Integral gain
        self.kd_heading = 0.05            # Differential gain
        
        # PID gains for velocity
        self.kp_linear = 0.2              # Proportional gain 
        self.ki_linear = 0.03             # Integral gain
        self.kd_linear = 0.05             # Differential gain
        
        # Velocity 
        self.vel_clip = 0.5           # Maximum linear velocity
        
        # Time tracking
        self.last_time = self.get_clock().now()
        
        # Initialize variables
        self.i = 0
        self.max_i = len(goals)
        self.goal = goals
        self.x = self.goal[self.i][0]
        self.y = self.goal[self.i][1]
        self.x_start = 0.0
        self.y_start = 0.0
        self.start_error = 0.0
        self.error_sum = 0.0
        self.finish = False        
        # Set up publisher and subscriber
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry, '/odom', self.pid_callback, 10)
        
        # Initialize velocity command
        self.base_cmd = Twist()
        self.base_cmd.linear.x = 0.0
        self.base_cmd.linear.y = 0.0
        self.base_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(self.base_cmd)

    def pid_callback(self, msg):
        # Handle finish condition
        if self.finish:
            # Send a final stop command
            stop_cmd = Twist()
            stop_cmd.linear.x, stop_cmd.angular.z = 0.0, 0.0 
            self.cmd_vel_pub.publish(stop_cmd)
            # Request shutdown of the node
            self.get_logger().info("Shutting down...")
            self.destroy_node()
            return rclpy.shutdown()
    
        # Get current time and calculate dt
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Convert to seconds
        self.last_time = current_time
        
        # Get current position
        x_curr = msg.pose.pose.position.x
        y_curr = msg.pose.pose.position.y
        
        # Log current position and goal
        self.get_logger().info(f"X: [{x_curr}] and Y: [{y_curr}] and Goal: {{X:{self.x}, Y:{self.y}}}")
        
        # Extract orientation quaternion
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # Convert quaternion to yaw (theta)
        current_theta = math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
        
        # Calculate distance error
        dist = math.sqrt((self.x - x_curr)**2 + (self.y - y_curr)**2)
        
        # Calculate velocity PID components
        
        # Derivative term
        if dt > 0:
            dist_error_diff = (dist - self.dist_error_prev) / dt
        else:
            dist_error_diff = 0.0
        self.dist_error_prev = dist  # Store for next iteration
        
        # Integral term with anti-windup
        self.dist_error_sum += dist * dt
        self.dist_error_sum = max(-self.dist_error_sum_max, 
                            min(self.dist_error_sum_max, self.dist_error_sum))
        
        # Create velocity command with PID
        self.base_cmd.linear.x = (self.kp_linear * dist + 
                               self.ki_linear * self.dist_error_sum + 
                               self.kd_linear * dist_error_diff)
        
        # Clip linear velocity
        self.base_cmd.linear.x = max(self.vel_clip, min(self.vel_clip, self.base_cmd.linear.x))
        
        # Log error metrics for forward velocity
        self.get_logger().info(f"Vel PID - P: {self.kp_linear * dist:.3f}, " +
                            f"I: {self.ki_linear * self.dist_error_sum:.3f}, " +
                            f"D: {self.kd_linear * dist_error_diff:.3f}")
        
        # Calculate target heading (in radians)
        target_theta = math.atan2(self.y - y_curr, self.x - x_curr)
        
        # Calculate heading error (shortest angle)
        heading_error = math.atan2(math.sin(target_theta - current_theta), 
                                 math.cos(target_theta - current_theta))
        
        # Calculate differential term (rate of change)
        if dt > 0:
            heading_error_diff = (heading_error - self.heading_error_prev) / dt
        else:
            heading_error_diff = 0.0
            
        # Update heading error for next iteration
        self.heading_error_prev = heading_error
        
        # Update integral term with anti-windup
        self.heading_error_sum += heading_error * dt
        # Apply anti-windup (limit accumulated error)
        self.heading_error_sum = max(-self.heading_error_sum_max, 
                                min(self.heading_error_sum_max, self.heading_error_sum))
        
        # PID control for rotation
        self.base_cmd.angular.z = (self.kp_heading * heading_error + 
                                self.ki_heading * self.heading_error_sum + 
                                self.kd_heading * heading_error_diff)
        
        # Clip angular velocity
        self.base_cmd.angular.z = max(-1.0, min(1.0, self.base_cmd.angular.z))
        
        # Log PID components for debugging
        self.get_logger().info(f"Heading PID Components - P: {self.kp_heading * heading_error:.3f}, " +
                            f"I: {self.ki_heading * self.heading_error_sum:.3f}, " +
                            f"D: {self.kd_heading * heading_error_diff:.3f}")
        
        # Log angles for debugging
        self.get_logger().info( f"Current θ: {current_theta*180/math.pi:.1f}° | " +
                              f"Target θ: {target_theta*180/math.pi:.1f}° | " +
                              f"Error: {heading_error*180/math.pi:.1f}° | " +
                              f"Heading Error: {heading_error}" ) 
        
        # Check if goal is reached and update to next waypoint
        if dist < 0.15:
            # Goal reached logic
            self.i += 1
            self.get_logger().info(" MOVING TO NEXT WAYPOINT !!!!!!")
            
            # Stop robot
            self.base_cmd.linear.x = 0.0
            self.base_cmd.angular.z = 0.0
            
            # Update to next waypoint
            if self.max_i > self.i:
                self.x = self.goal[self.i][0]
                self.y = self.goal[self.i][1]
            else:
                self.get_logger().info("Al GOAL(s) REACHED")
                self.finish = True
                
        
        # Store distance for debugging
        self.base_cmd.linear.y = dist  # Just for logging
        
        # Publish command
        self.cmd_vel_pub.publish(self.base_cmd)


def main(args=None):
    rclpy.init(args=args)
    
    # Define waypoints
    
    pos = [[0.0, 10.0], [10, 10.0]] 
    
    # Create and run node
    robot_driver = RobotDriver(pos)
    rclpy.spin(robot_driver)
    
    # Cleanup
    robot_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()