# todo
# [x] clean up the code
# [x] fix the rotation bug
# [x] make a readme.md for the paths
# [x] find the teleop command and put that in tips
# [x] publish distance and angular errors and visualize in rqt_plot
# [ ] collect ros bag


#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import math
import argparse
from collections import namedtuple
import sys

PathPoint = namedtuple("PathPoint", ["x", "y", "theta"])


class TurtleBotController(Node):
    def __init__(self, path: list[PathPoint] = [], circular_path=False):
        super().__init__('turtlebot_controller')

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ae_pulisher = self.create_publisher(Float32, '/angular_error', 10)
        self.le_publisher = self.create_publisher(Float32, '/linear_error', 10)
        self.odom_subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # PID controller parameters for angular
        self.kp_angular = 0.4

        # PID controller parameters for linear
        self.kp = 0.12
        self.ki = 0.0001
        self.kd = 0.015
        self.prev_error = 0.0
        self.integral = 0.0

        # robot state
        self.reached_xy = False

        self.circular_path = circular_path
        # disable PD controll when havin a circular path
        if circular_path:
            self.ki = 0
            self.kd = 0
            self.kp = 1.3*self.kp

        self.path = path
        self.goal_index = 0

        # target x-coordinate
        self.target_x = self.path[self.goal_index].x
        # target y-coordinate
        self.target_y = self.path[self.goal_index].y
        # target yaw/theta
        self.target_yaw = self.path[self.goal_index].theta
        self.get_logger().info(
            f'>>>>> new goal: ({self.path[self.goal_index]})')

    def calculate_yaw(self, odom: Odometry):
        # Calculate the Yaw
        quaternion = odom.pose.pose.orientation
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        if yaw < 0:
            # fix the singularity
            yaw = yaw + 2*math.pi
        return yaw

    def calculate_steering_angle(self, odom: Odometry):
        # Calculate the angle to the target
        dy = self.target_y - odom.pose.pose.position.y
        dx = self.target_x - odom.pose.pose.position.x

        steering_angle = math.atan2(dy, dx)
        if steering_angle < 0:
            # fix the singularity
            steering_angle = steering_angle + 2*math.pi
        return steering_angle

    def calculate_distance(self, odom: Odometry):
        # Get the current position from the /odom topic
        current_x = odom.pose.pose.position.x
        current_y = odom.pose.pose.position.y

        # Calculate the distance to the target
        distance = math.sqrt((self.target_x - current_x) **
                             2 + (self.target_y - current_y)**2)
        return distance

    def calculate_angular_diff(self, target, current):
        # calculate the angular difference and correct it to alway take the shorted/smallest difference
        diff = target - current
        if diff > 0:
            corr = diff - 2*math.pi
        elif abs(diff) < 0.001:
            return 0.0
        else:
            corr = diff + 2*math.pi

        if abs(diff) < abs(corr):
            return diff
        else:
            return corr

    def odom_callback(self, odom: Odometry):
        # init command message
        cmd_msg = Twist()

        # Calculate the distance to the target
        distance_error = self.calculate_distance(odom)

        # Calculate yaw from odometry message which contains quaternions
        current_yaw = self.calculate_yaw(odom)
        # orientation_error = self.target_yaw - current_yaw
        angular_error = self.calculate_angular_diff(
            self.target_yaw, current_yaw)

        ############################################ STATE1: DISTANCE COTROL ###########################################
        if distance_error > 0.05 and not self.reached_xy:
            # Calculate the angle to the target
            sa = self.calculate_steering_angle(odom)
            steering_error = self.calculate_angular_diff(sa, current_yaw)

            # Create and publish the Twist message
            cmd_msg.angular.z = self.kp_angular * steering_error

            # make the path straight in case the path is not circular
            if (abs(steering_error) < 0.05) or self.circular_path:
                # Calculate the PID controller output
                self.integral += distance_error
                linear_vel = self.kp * distance_error + self.ki * \
                    self.integral + self.kd * \
                    (distance_error - self.prev_error)
                self.prev_error = distance_error
                cmd_msg.linear.x = linear_vel
            else:
                cmd_msg.linear.x = 0.0

            self.cmd_vel_publisher.publish(cmd_msg)
            errl = Float32()
            errl.data = distance_error
            self.le_publisher.publish(errl)

        ######################################## STATE2: ORIENTATION COTROL ############################################
        elif abs(angular_error) > 0.01:
            self.reached_xy = True
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = self.kp_angular * angular_error
            self.cmd_vel_publisher.publish(cmd_msg)
            erra = Float32()
            erra.data = angular_error
            self.ae_pulisher.publish(erra)

        ############################# STATE3: REACHING THE GOAL, AND PREPARING NEXT GOAL ###############################
        else:
            self.reached_xy = False
            # Stop the robot when the goal is reached
            cmd_msg = Twist()
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0
            self.cmd_vel_publisher.publish(cmd_msg)
            self.get_logger().info(
                f'* reached goal: {self.path[self.goal_index]}')

            # go to the new goal
            self.goal_index = self.goal_index + 1
            if self.goal_index < len(self.path):
                # target x-coordinate
                self.target_x = self.path[self.goal_index].x
                # target y-coordinate
                self.target_y = self.path[self.goal_index].y
                # target yaw/theta
                self.target_yaw = self.path[self.goal_index].theta
                self.get_logger().info(
                    f'>>>>> new goal: ({self.path[self.goal_index]})')
            else:
                self.get_logger().info('Final destination reached !!!!!!!! (^_^) !!!!!!!!')
                sys.exit()


def main(args=None):
    rclpy.init(args=args)

    # Define command-line arguments for the list of coordinates
    parser = argparse.ArgumentParser()
    parser.add_argument('--path', type=str,
                        help='List of coordinates as x,y,theta;x,y,theta;...')
    parser.add_argument('--circular', type=bool, default=False,
                        help='allow cicular paths instead of forcing straight paths')
    args, _ = parser.parse_known_args()

    if args.path:
        path = [PathPoint(*map(float, point.split(',')))
                for point in args.path.split(';')]
        node = TurtleBotController(path=path, circular_path=args.circular)
        rclpy.spin(node)
        rclpy.shutdown()


if __name__ == '__main__':
    main()
