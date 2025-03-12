#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import csv
import argparse
from datetime import datetime
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class RobotDriver(Node):
    def __init__(self, goals):
        super().__init__('robot_driver')

        # PID Control Variables
        self.heading_error_prev = 0.0
        self.heading_error_sum = 0.0
        self.heading_error_sum_max = 2.0  # Anti-windup limit

        self.dist_error_prev = 0.0
        self.dist_error_sum = 0.0
        self.dist_error_sum_max = 2.0  # Anti-windup limit

        # PID Gains
        self.kp_heading, self.ki_heading, self.kd_heading = 0.5, 0.03, 0.05
        self.kp_linear, self.ki_linear, self.kd_linear = 0.2, 0.03, 0.05

        # Max Velocity Limits
        self.vel_clip = 0.5

        # Time Tracking
        self.last_time = self.get_clock().now()

        # Waypoints & Goal Tracking
        self.goals = goals
        self.i = 0
        self.max_i = len(goals)
        self.x, self.y = self.goals[self.i]
        self.finish = False
        self.current_pose = None

        # ROS 2 Publishers & Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.pid_callback, 10)

        # Initialize Velocity Command
        self.base_cmd = Twist()
        self.cmd_vel_pub.publish(self.base_cmd)

        # Data Logging
        self.trajectory_data = []
        self.current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = f'robot_trajectory_data_{self.current_time}.csv'

        self.get_logger().info("PID Navigation Initialized.")

    def pid_callback(self, msg):
        """Callback function for PID control based on odometry data"""
        self.current_pose = msg.pose.pose
        self.current_twist_linear = msg.twist.twist.linear
        self.current_twist_angular = msg.twist.twist.angular

        if self.finish:
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            self.get_logger().info("Shutting down...")
            rclpy.shutdown()
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Convert to seconds
        self.last_time = current_time

        x_curr, y_curr = msg.pose.pose.position.x, msg.pose.pose.position.y
        qx, qy, qz, qw = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        current_theta = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

        dist = math.sqrt((self.x - x_curr) ** 2 + (self.y - y_curr) ** 2)
        target_theta = math.atan2(self.y - y_curr, self.x - x_curr)
        heading_error = math.atan2(math.sin(target_theta - current_theta), math.cos(target_theta - current_theta))

        dist_error_diff = (dist - self.dist_error_prev) / dt if dt > 0 else 0.0
        self.dist_error_prev = dist
        self.dist_error_sum = max(-self.dist_error_sum_max, min(self.dist_error_sum_max, self.dist_error_sum + dist * dt))

        heading_error_diff = (heading_error - self.heading_error_prev) / dt if dt > 0 else 0.0
        self.heading_error_prev = heading_error
        self.heading_error_sum = max(-self.heading_error_sum_max, min(self.heading_error_sum_max, self.heading_error_sum + heading_error * dt))

        self.base_cmd.linear.x = max(-self.vel_clip, min(self.vel_clip, self.kp_linear * dist + self.ki_linear * self.dist_error_sum + self.kd_linear * dist_error_diff))
        self.base_cmd.angular.z = max(-1.0, min(1.0, self.kp_heading * heading_error + self.ki_heading * self.heading_error_sum + self.kd_heading * heading_error_diff))

        self.log_position(dist, heading_error, dist_error_diff, heading_error_diff)

        if dist < 0.15:
            self.i += 1
            if self.i < self.max_i:
                self.x, self.y = self.goals[self.i]
            else:
                self.get_logger().info("Final waypoint reached. Stopping robot.")
                self.finish = True
                self.save_trajectory_data()
                stop_cmd = Twist()
                self.cmd_vel_pub.publish(stop_cmd)
                rclpy.shutdown()

        self.cmd_vel_pub.publish(self.base_cmd)

    def log_position(self, dist, heading_error, dist_error_diff, heading_error_diff):
        """Log robot data for visualization"""
        current_x, current_y = self.current_pose.position.x, self.current_pose.position.y
        data_point = {
            'timestamp': self.get_clock().now().to_msg().sec,
            'waypoint_number': self.i,
            'robot_x': current_x,
            'robot_y': current_y,
            'goal_x': self.x,
            'goal_y': self.y,
            'distance_error': dist,
            'heading_error': heading_error,
            'linear_velocity': self.base_cmd.linear.x,
            'angular_velocity': self.base_cmd.angular.z,
            'P_linear': self.kp_linear * dist,
            'I_linear': self.ki_linear * self.dist_error_sum,
            'D_linear': self.kd_linear * dist_error_diff,
            'P_heading': self.kp_heading * heading_error,
            'I_heading': self.ki_heading * self.heading_error_sum,
            'D_heading': self.kd_heading * heading_error_diff
        }
        self.trajectory_data.append(data_point)

    def save_trajectory_data(self):
        """Save logged data to a CSV file"""
        with open(self.csv_filename, 'w', newline='') as csvfile:
            fieldnames = self.trajectory_data[0].keys()
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.trajectory_data)
        self.get_logger().info(f"Data saved to {self.csv_filename}")


def generate_waypoints(shape, size_x, size_y=None):
    """Generate predefined waypoints for movement"""
    if shape == "linear":
        return [[i, 0] for i in range(size_x + 1)]
    elif shape == "square":
        return [[0, 0], [size_x, 0], [size_x, size_x], [0, size_x], [0, 0]]
    elif shape == "rectangle":
        return [[0, 0], [size_x, 0], [size_x, size_y], [0, size_y], [0, 0]]
    elif shape == "zig-zag":
        return [[i, (i % 2) * size_y] for i in range(0, size_x + 1, 2)]
    else:
        raise ValueError("Invalid shape. Choose from: linear, square, rectangle, zig-zag.")


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description="Control TurtleBot3 with PID and predefined paths.")
    parser.add_argument("shape", choices=["linear", "square", "rectangle", "zig-zag"], help="Shape of movement")
    parser.add_argument("size_x", type=int, help="Primary dimension size")
    parser.add_argument("size_y", type=int, nargs="?", default=None, help="Secondary dimension size (optional for rectangles and zig-zags)")

    args = parser.parse_args()
    waypoints = generate_waypoints(args.shape, args.size_x, args.size_y)

    robot_driver = RobotDriver(waypoints)
    try:
        rclpy.spin(robot_driver)
    except KeyboardInterrupt:
        pass
    finally:
        robot_driver.save_trajectory_data()
        robot_driver.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

# To RUN
# python script.py square 4
# python script.py rectangle 4 2
# python script.py zig-zag 6 2
# python script.py linear 10