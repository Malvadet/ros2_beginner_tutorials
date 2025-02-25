#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

class TurtleBot(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')

        # Publisher to control turtle's velocity
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Subscriber to receive turtle's pose
        self.pose_subscriber = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.update_pose,
            10
        )

        self.pose = Pose()
        self.timer = self.create_timer(0.1, self.control_loop)  # Timer for the control loop
        self.goal_pose = None
        self.distance_tolerance = 0.1  # Default tolerance

    def update_pose(self, data):
        """Callback function to update turtle's pose."""
        self.pose = data

    def euclidean_distance(self, goal_pose):
        """Calculates Euclidean distance between current pose and goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        """Calculates linear velocity."""
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """Calculates steering angle."""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=6):
        """Calculates angular velocity."""
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def control_loop(self):
        """Main control loop to move the turtle towards the goal."""
        if self.goal_pose is None:
            return  # No goal set yet

        vel_msg = Twist()
        
        if self.euclidean_distance(self.goal_pose) >= self.distance_tolerance:
            vel_msg.linear.x = self.linear_vel(self.goal_pose)
            vel_msg.angular.z = self.angular_vel(self.goal_pose)
        else:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.goal_pose = None  # Reset goal after reaching it
            self.get_logger().info("Goal reached!")

        self.velocity_publisher.publish(vel_msg)

    def set_goal(self, x, y, tolerance=0.1):
        """Sets a new goal for the turtle."""
        self.goal_pose = Pose()
        self.goal_pose.x = x
        self.goal_pose.y = y
        self.distance_tolerance = tolerance
        self.get_logger().info(f"Moving to goal: x={x}, y={y} with tolerance={tolerance}")


def main(args=None):
    rclpy.init(args=args)
    turtlebot = TurtleBot()

    try:
        x_goal = float(input("Set your x goal: "))
        y_goal = float(input("Set your y goal: "))
        tolerance = float(input("Set your tolerance: "))
        turtlebot.set_goal(x_goal, y_goal, tolerance)

        rclpy.spin(turtlebot)

    except KeyboardInterrupt:
        turtlebot.get_logger().info("Program interrupted.")

    finally:
        turtlebot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
