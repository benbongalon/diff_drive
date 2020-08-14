#! /usr/bin/env python
from __future__ import division

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster, TransformStamped
from math import sin, cos
from src.diff_drive_module.pose import Pose, euler_from_quaternion
from src.diff_drive_module import odometry


class OdometryNode(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('diff_drive_odometry')
        self.odometry = odometry.Odometry()
        qos_profile = QoSProfile(depth=10)
        self.odomPub = self.create_publisher(Odometry, 'odom', qos_profile)
        self.tfPub = TransformBroadcaster(self, qos=qos_profile)

        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        self.create_subscription(Int32, "lwheel_ticks", self.leftCallback, qos_profile)
        self.create_subscription(Int32, "rwheel_ticks", self.rightCallback, qos_profile)
        self.create_subscription(PoseWithCovarianceStamped,"initialpose", 
                                 self.on_initial_pose, qos_profile)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('ticks_per_meter', 1000),
                ('wheel_separation', 100),
                ('rate', 10.0),
                ('base_frame_id', 'base_link'),
                ('odom_frame_id', 'odom'),
                ('encoder_min', -32768),
                ('encoder_max', 32767)
            ]
        )
        self.ticksPerMeter = int(self.get_parameter('ticks_per_meter').value)
        self.wheelSeparation = float(self.get_parameter('wheel_separation').value)
        self.rate = float(self.get_parameter('rate').value)
        self.baseFrameID = self.get_parameter('base_frame_id').value
        self.odomFrameID = self.get_parameter('odom_frame_id').value
        self.encoderMin = int(self.get_parameter('encoder_min').value)
        self.encoderMax = int(self.get_parameter('encoder_max').value)

        self.odometry.setWheelSeparation(self.wheelSeparation)
        self.odometry.setTicksPerMeter(self.ticksPerMeter)
        self.odometry.setEncoderRange(self.encoderMin, self.encoderMax)
        self.odometry.setTime(self.get_clock().now())

        rate = self.create_rate(self.rate)
        while rclpy.ok():
            self.publish()
            rate.sleep()

    def publish(self):
        now = self.get_clock().now()
        self.odometry.updatePose(now)
        pose = self.odometry.getPose()

        #q = quaternion_from_euler(0, 0, pose.theta)
        #self.tfPub.sendTransform(
        #    (pose.x, pose.y, 0),
        #    (q[0], q[1], q[2], q[3]),
        #    now,
        #    self.baseFrameID,
        #    self.odomFrameID
        #)

        # Translate it
        odom_trans = TransformStamped()
        odom_trans.header.stamp = now.to_msg()
        odom_trans.header.frame_id = self.odomFrameID
        odom_trans.child_frame_id = self.baseFrameID
        odom_trans.transform.translation.x = float(pose.x)
        odom_trans.transform.translation.y = float(pose.y)
        odom_trans.transform.translation.z = 0.0
        self.tfPub.sendTransform(odom_trans)

        #odom = Odometry()
        #odom.header.stamp = now
        #odom.header.frame_id = self.odomFrameID
        #odom.child_frame_id = self.baseFrameID
        #odom.pose.pose.position.x = pose.x
        #odom.pose.pose.position.y = pose.y
        #odom.pose.pose.orientation.x = q[0]
        #odom.pose.pose.orientation.y = q[1]
        #odom.pose.pose.orientation.z = q[2]
        #odom.pose.pose.orientation.w = q[3]
        #odom.twist.twist.linear.x = pose.xVel
        #odom.twist.twist.angular.z = pose.thetaVel
        #self.odomPub.publish(odom)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odomFrameID
        odom.child_frame_id = self.baseFrameID
        odom.pose.pose.position.x = pose.x
        odom.pose.pose.position.y = pose.y
        odom.pose.pose.position.z = 0.0
        quaternion = Quaternion(x=0.0,
                                y=0.0,
                                z=sin(pose.theta / 2.0),
                                w=cos(pose.theta / 2.0))
        odom.pose.pose.orientation = quaternion
        odom.twist.twist.linear.x = pose.xVel
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = pose.thetaVel
        self.odomPub.publish(odom)

    def on_initial_pose(self, msg):
        q = [msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q)

        pose = Pose()
        pose.x = msg.pose.pose.position.x
        pose.y = msg.pose.pose.position.y
        pose.theta = yaw

        self.get_logger().info('Setting initial pose to %s', pose)
        self.odometry.setPose(pose)

    def leftCallback(self, msg):
        self.odometry.updateLeftWheel(msg.data)

    def rightCallback(self, msg):
        self.odometry.updateRightWheel(msg.data)


def main():
    node = OdometryNode()

if __name__ == '__main__':
    main()


