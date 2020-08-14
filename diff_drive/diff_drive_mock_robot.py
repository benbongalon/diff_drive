#! /usr/bin/env python
from __future__ import division

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from math import pi, asin
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from src.diff_drive_module import mock_robot

class MockRobotNode(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('diff_drive_mock_robot')
        self.robot = mock_robot.MockRobot()
        self.leftSpeed = 0
        self.rightSpeed = 0
        qos_profile = QoSProfile(depth=10)
        self.leftPub = self.create_publisher(Int32, 'lwheel_ticks', qos_profile)
        self.rightPub = self.create_publisher(Int32, 'rwheel_ticks', qos_profile)

        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        self.create_subscription(Int32, 'lwheel_desired_rate', self.leftCallback, qos_profile)
        self.create_subscription(Int32, 'rwheel_desired_rate', self.rightCallback, qos_profile)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('rate', 10.0),
                ('timeout', 0.5)
            ]
        )
        self.rate = float(self.get_parameter('rate').value)
        self.timeout = float(self.get_parameter('timeout').value)

        rate = self.create_rate(self.rate)
        self.lastTime = self.get_clock().now()
        while rclpy.ok():
            self.publish()
            rate.sleep()

    def publish(self):
        newTime = self.get_clock().now()
        diffTime = newTime - self.lastTime
        self.lastTime = newTime

        if diffTime.nanoseconds / 1.e9 > self.timeout:
            self.robot.setSpeeds(0, 0)
    
        try:
            self.robot.updateRobot(diffTime.nanoseconds / 1.e9)
        except Exception as ex:
            self.get_logger().error("Got exception updating robot: {}".format(ex))
            raise
        ticks = self.robot.getTicks()
        self.leftPub.publish(Int32(data=ticks.left))
        self.rightPub.publish(Int32(data=ticks.right))

    def leftCallback(self, leftSpeed):
        self.leftSpeed = leftSpeed.data
        self.robot.setSpeeds(self.leftSpeed, self.rightSpeed)

    def rightCallback(self, rightSpeed):
        self.rightSpeed = rightSpeed.data
        self.robot.setSpeeds(self.leftSpeed, self.rightSpeed)


def main():
    node = MockRobotNode()


if __name__ == '__main__':
    main()
