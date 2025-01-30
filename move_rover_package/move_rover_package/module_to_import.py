#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from move_rover_action.action import MoveRover
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math


class SimpleRoverActionServer(Node):
    def __init__(self):
        super().__init__('simple_rover_action_server')
        self._action_server = ActionServer(
            self,
            MoveRover,
            'move_rover',
            self.execute_callback,
            goal_callback=self.handle_goal,
            cancel_callback=self.handle_cancel
        )
        self.publisher = self.create_publisher(Twist, '/model/vehicle_blue/cmd_vel', 10)
        self.odom_subscription = self.create_subscription(Odometry, '/model/vehicle_blue/odometry', self.odom_callback, 10)
        self.lidar_subscription = self.create_subscription(LaserScan, '/lidar', self.lidar_callback, 10)

        self.current_position = 0.0
        self.start_position = None
        self.obstacle_detected = False
        self.safe_distance = 1.0
        self.obstacle_direction = None

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position.x

    def lidar_callback(self, msg):
        angle_increment = msg.angle_increment
        for i, distance in enumerate(msg.ranges):
            angle = msg.angle_min + i * angle_increment
            if -math.pi / 6 <= angle <= math.pi / 6 and distance < self.safe_distance:
                self.obstacle_detected = True
                self.obstacle_direction = angle
                return
        self.obstacle_detected = False
        self.obstacle_direction = None

    def handle_goal(self, goal_handle):
        self.get_logger().info('Received a new goal request.')
        return GoalResponse.ACCEPT

    def handle_cancel(self, goal_handle):
        self.get_logger().info('Goal cancellation requested.')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        target_distance = goal_handle.request.distance
        self.start_position = self.current_position
        feedback = MoveRover.Feedback()
        cmd = Twist()

        while abs(self.current_position - self.start_position) < target_distance:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled.')
                cmd.linear.x = 0.0
                self.publisher.publish(cmd)
                return MoveRover.Result(success=False)

            if self.obstacle_detected:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.5 if self.obstacle_direction and self.obstacle_direction < 0 else -0.5
            else:
                cmd.linear.x = 0.5
                cmd.angular.z = 0.0

            self.publisher.publish(cmd)
            distance_covered = abs(self.current_position - self.start_position)
            feedback.progress = f'{distance_covered:.2f} out of {target_distance:.2f} meters covered.'
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(feedback.progress)
            rclpy.spin_once(self, timeout_sec=0.1)

        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.publisher.publish(cmd)
        goal_handle.succeed()
        self.get_logger().info('Goal completed successfully.')
        return MoveRover.Result(success=True)