#! /usr/bin/env python3

import math
import csv
import argparse
from datetime import datetime
import os
from typing import List, Tuple, Dict
from enum import Enum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import tf_transformations

class PathType(Enum):
    SQUARE = "square"
    ZIGZAG = "zigzag"

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.current_pose = None
        self._create_subscribers()

    def _create_subscribers(self):
        self.pose_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.pose_callback,
            10)

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

class PathPlanner:
    @staticmethod
    def create_pose_stamped(navigator: BasicNavigator, x: float, y: float, z: float = 0.0) -> PoseStamped:
        pose = PoseStamped()
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, z)
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0  
        pose.pose.orientation.x = q_x
        pose.pose.orientation.y = q_y
        pose.pose.orientation.z = q_z
        pose.pose.orientation.w = q_w
        return pose

    @staticmethod
    def generate_square_path(navigator: BasicNavigator, size: float = 1.0) -> List[PoseStamped]:
        """Generate waypoints for a square path"""
        waypoints = [
            (size, size, 3.14),      # Top right down (180)
            (-size, size, 4.71),     # Top left (270)
            (-size, -size, 0),    # Bottom left (0)
            (size, -size, 1.57),     # Bottom right (90)
            (size, size, 1.57)       # Back to top right to close the square (90)
        ]
        return [PathPlanner.create_pose_stamped(navigator, x, y, z) for x, y, z in waypoints]
    
    @staticmethod
    def generate_zigzag_path(navigator: BasicNavigator, size: float = 2.0) -> List[PoseStamped]:
        """Generate waypoints for a zigzag path
        
        Args:
            navigator: BasicNavigator instance
            size: Size of the path (default: 2.0)
            
        Returns:
            List[PoseStamped]: List of pose stamped waypoints
        """
        waypoints = [
            (-size, size, 0.0),      # Top left
            (0.0, size, 0.0),        # Top middle
            (size, size, -1.57),     # Top right (pointing down)
            (size, 0.0, 3.14),       # Middle right (pointing left)
            (0.0, 0.0, 3.14),        # Center (pointing left)
            (-size, 0.0, -1.57),     # Middle left (pointing down)
            (-size, -size, 0.0),     # Bottom left (pointing right)
            (0.0, -size, 0.0),       # Bottom middle (pointing right)
            (size, -size, 0.0)       # Bottom right
        ]
        return [PathPlanner.create_pose_stamped(navigator, x, y, z) for x, y, z in waypoints]

    @staticmethod
    def generate_path(navigator: BasicNavigator, path_type: PathType, size: float) -> List[PoseStamped]:
        """Generate waypoints based on the specified path type
        
        Args:
            navigator: BasicNavigator instance
            path_type: Type of path to generate (square or zigzag)
            size: Size of the path
            
        Returns:
            List[PoseStamped]: List of pose stamped waypoints
        """
        if path_type == PathType.SQUARE:
            return PathPlanner.generate_square_path(navigator, size)
        elif path_type == PathType.ZIGZAG:
            return PathPlanner.generate_zigzag_path(navigator, size)
        else:
            raise ValueError(f"Unknown path type: {path_type}")

class ErrorCalculator:
    @staticmethod
    def calculate_perpendicular_distance(point: Tuple[float, float], 
                                       line_start: Tuple[float, float], 
                                       line_end: Tuple[float, float]) -> float:
        """Calculate perpendicular distance from a point to a line segment."""
        x, y = point
        x1, y1 = line_start
        x2, y2 = line_end
        
        line_length = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        if line_length == 0:
            return math.sqrt((x - x1)**2 + (y - y1)**2)
        
        numerator = abs((x2 - x1)*(y1 - y) - (x1 - x)*(y2 - y1))
        return numerator / line_length

    @staticmethod
    def get_current_segment(current_waypoint: int, goal_poses: List[PoseStamped]) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        """Determine the current line segment the robot should be on."""
        if current_waypoint == 0:
            return (0.0, 0.0), (goal_poses[0].pose.position.x, goal_poses[0].pose.position.y)
        return ((goal_poses[current_waypoint-1].pose.position.x, goal_poses[current_waypoint-1].pose.position.y),
                (goal_poses[current_waypoint].pose.position.x, goal_poses[current_waypoint].pose.position.y))
    
    @staticmethod
    def calculate_total_error(trajectory_data: List[Dict]) -> Tuple[float, float]:
        """Calculate total and average error from trajectory data.
        
        Args:
            trajectory_data: List of dictionaries containing trajectory information
            
        Returns:
            Tuple[float, float]: (total_error, average_error)
        """
        if not trajectory_data:
            return 0.0, 0.0
            
        # Skip Invalid Data
        valid_data = [point for point in trajectory_data if point['waypoint_number'] != 1]

        # TODO We Can add a Logic to Ignore the Error Cases in the Corners with a Buffer Radius Here
        
        total_error = sum(point['error'] for point in valid_data)
        avg_error = total_error / len(valid_data) if valid_data else 0.0
        
        return total_error, avg_error

class DataLogger:
    def __init__(self):
        self.trajectory_data = []
        self.current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = f'robot_trajectory_{self.current_time}.csv'

    def add_data_point(self, data_point: Dict):
        self.trajectory_data.append(data_point)

    def save_to_csv(self):
        try:
            with open(self.csv_filename, 'w', newline='') as csvfile:
                fieldnames = ['timestamp', 'robot_x', 'robot_y', 
                            'ideal_line_start_x', 'ideal_line_start_y',
                            'ideal_line_end_x', 'ideal_line_end_y',
                            'error', 'waypoint_number']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                for data_point in self.trajectory_data:
                    writer.writerow(data_point)
            print(f"\nTrajectory data saved to {self.csv_filename}")
        except Exception as e:
            print(f"Error saving trajectory data: {e}")

class NavigationController:
    def __init__(self, path_type: PathType):
        self.navigator = BasicNavigator()
        self.waypoint_follower = WaypointFollower()
        self.data_logger = DataLogger()
        self.error_calculator = ErrorCalculator()
        self.path_type = path_type

    def initialize_navigation(self):
        initial_pose = PathPlanner.create_pose_stamped(self.navigator, 0.0, 0.0)
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()

    def execute_path(self, goal_poses: List[PoseStamped]):
        self.navigator.followWaypoints(goal_poses)
        
        i = 0
        total_error = 0.0
        num_samples = 0
        
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self.waypoint_follower)
            i += 1
            feedback = self.navigator.getFeedback()
            
            if feedback and i % 5 == 0 and self.waypoint_follower.current_pose is not None:
                self._process_navigation_feedback(feedback, goal_poses, total_error, num_samples)

    def _process_navigation_feedback(self, feedback, goal_poses, total_error, num_samples):
        current_pos = (
            self.waypoint_follower.current_pose.position.x,
            self.waypoint_follower.current_pose.position.y
        )
        
        current_segment = self.error_calculator.get_current_segment(feedback.current_waypoint, goal_poses)
        distance = self.error_calculator.calculate_perpendicular_distance(
            current_pos, current_segment[0], current_segment[1]
        )
        
        total_error += distance
        num_samples += 1

        self._log_data_point(feedback, current_pos, current_segment, distance)
        self._print_status(feedback, current_pos, distance, total_error, num_samples, len(goal_poses))

    def _log_data_point(self, feedback, current_pos, current_segment, distance):
        data_point = {
            'timestamp': self.navigator.get_clock().now().to_msg().sec,
            'robot_x': current_pos[0],
            'robot_y': current_pos[1],
            'ideal_line_start_x': current_segment[0][0],
            'ideal_line_start_y': current_segment[0][1],
            'ideal_line_end_x': current_segment[1][0],
            'ideal_line_end_y': current_segment[1][1],
            'error': distance,
            'waypoint_number': feedback.current_waypoint + 1
        }
        self.data_logger.add_data_point(data_point)

    @staticmethod
    def _print_status(feedback, current_pos, distance, total_error, num_samples, total_waypoints):
        print(
            f'Executing current waypoint: {feedback.current_waypoint + 1}/{total_waypoints}\n'
            f'Current position: ({current_pos[0]:.2f}, {current_pos[1]:.2f})\n'
            f'Current error: {distance:.4f}\n'
            f'Average error so far: {(total_error/num_samples):.4f}'
        )

    def cleanup(self):
        self.data_logger.save_to_csv()
        self.waypoint_follower.destroy_node()
        self.navigator.lifecycleShutdown()

def main():
    # Parse command line arguments
    import argparse
    parser = argparse.ArgumentParser(description='Robot Path Navigation')
    parser.add_argument('--path-type', type=str, choices=['square', 'zigzag'],
                      default='square', help='Type of path to follow')
    parser.add_argument('--size', type=float, default=1.0,
                      help='Size of the path')
    args = parser.parse_args()

    rclpy.init()

    # Convert string to PathType enum
    path_type = PathType(args.path_type)
    
    controller = NavigationController(path_type)
    controller.initialize_navigation()
    
    # Generate and execute path
    goal_poses = PathPlanner.generate_path(controller.navigator, path_type, args.size)
    controller.execute_path(goal_poses)

    # Calculate and display the total error
    total_error, avg_error = ErrorCalculator.calculate_total_error(
        controller.data_logger.trajectory_data
    )
    
    # Print error statistics with formatting
    print("\n" + "="*50)
    print(f"NAVIGATION COMPLETE - ERROR STATISTICS ({path_type.value.upper()} PATH)")
    print("="*50)
    print(f"Total Error: {total_error:.4f} meters")
    print(f"Average Error: {avg_error:.4f} meters")
    print("="*50 + "\n")
    
    controller.cleanup()
    rclpy.shutdown()

if __name__ == '__main__':
    main()