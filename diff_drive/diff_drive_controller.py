#! /usr/bin/env python
from __future__ import division

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

from src.diff_drive_module import Controller

class ControllerNode(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('diff_drive_controller')
        self.controller = Controller()
        self.linearVelocity = 0.0
        self.angularVelocity = 0.0

        qos_profile = QoSProfile(depth=1)
        self.leftPub = self.create_publisher(Int32, 'lwheel_desired_rate', qos_profile)
        self.rightPub = self.create_publisher(Int32, 'rwheel_desired_rate', qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        qos_profile = QoSProfile(depth=10)
        self.create_subscription(Twist, 'cmd_vel', self.twistCallback, qos_profile)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('ticks_per_meter', 1000),
                ('wheel_separation', 100),
                ('max_motor_speed', 1.0),
                ('rate', 10.0),
                ('timeout', 0.2)
            ]
        )
        self.ticksPerMeter = float(self.get_parameter('ticks_per_meter').value)
        self.wheelSeparation = float(self.get_parameter('wheel_separation').value)
        self.maxMotorSpeed = int(self.get_parameter('max_motor_speed').value)
        self.rate = float(self.get_parameter('rate').value)
        self.timeout = float(self.get_parameter('timeout').value)

        self.controller.setWheelSeparation(self.wheelSeparation)
        self.controller.setTicksPerMeter(self.ticksPerMeter)
        self.controller.setMaxMotorSpeed(self.maxMotorSpeed)

        rate = self.create_rate(self.rate)
        self.lastTwistTime = self.get_clock().now()
        while rclpy.ok():
            self.publish()
            rate.sleep()

    def publish(self):
        if (self.get_clock().now() - self.lastTwistTime).nanoseconds / 1.e9 < self.timeout:
            speeds = self.controller.getSpeeds(self.linearVelocity,
                                               self.angularVelocity)
            self.leftPub.publish(Int32(data=speeds.left))
            self.rightPub.publish(Int32(data=speeds.right))
        else:
            self.leftPub.publish(Int32(data=0))
            self.rightPub.publish(Int32(data=0))

    def twistCallback(self, twist):
        self.linearVelocity = twist.linear.x
        self.angularVelocity = twist.angular.z
        self.lastTwistTime = self.get_clock().now()


def main():
    node = ControllerNode()

if __name__ == '__main__':
    main()

