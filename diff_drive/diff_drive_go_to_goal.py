#! /usr/bin/env python
from __future__ import division

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from math import pi
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Bool
#import actionlib

from src.diff_drive_module import goal_controller
from src.diff_drive_module.pose import Pose, euler_from_quaternion
from src.diff_drive_module.msg import GoToPoseAction, GoToPoseGoal, GoToPoseResult


class GoToGoalNode(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('diff_drive_go_to_goal')
        self.controller = goal_controller.GoalController()
        self.action_name = 'diff_drive_go_to_goal'
        self.action_server \
            = actionlib.SimpleActionServer(self.action_name, GoToPoseAction,
                                           execute_cb=self.on_execute,
                                           auto_start=False)
        self.action_client = actionlib.SimpleActionClient(
            'diff_drive_go_to_goal', GoToPoseAction)

        qos_profile = QoSProfile(depth=10)
        self.dist_pub = rclpy.create_publisher(Float32, 'distance_to_goal', qos_profile)
        self.twist_pub = rclpy.create_publisher(Twist, 'cmd_vel', qos_profile)

        self.node_name = self.get_name()
        self.get_logger().info("{0} started".format(self.node_name))

        self.create_subscription(Odometry, 'odom', self.on_odometry, qos_profile)
        self.create_subscription(PoseStamped, 'move_base_simple/goal', self.on_goal, qos_profile)

        qos_profile = QoSProfile(depth=1)
        self.goal_achieved_pub = self.create_publisher(Bool, 'goal_achieved', qos_profile)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('rate', 10.0),
                ('kP', 3.0),
                ('kA', 8.0),
                ('kB', -1.5),
                ('linear_tolerance', 0.05),
                ('angular_tolerance', 3.0 / 180.0*pi),
                ('max_linear_speed', 0.2),
                ('min_linear_speed', 0),
                ('max_angular_speed', 1.0),
                ('min_angular_speed', 0),
                ('max_linear_acceleration', 0.1),
                ('max_angular_acceleration', 0.3),
                ('forwardMovementOnly', True)
            ]
        )
        rate = float(self.get_parameter('rate').value)
        self.rate = self.create_rate(self.rate)
        self.dT = 1.0 / rate

        self.kP = float(self.get_parameter('kP').value)
        self.kA = float(self.get_parameter('kA').value)
        self.kB = float(self.get_parameter('kB').value)
        self.controller.set_constants(self.kP, self.kA, self.kB)

        self.controller.set_linear_tolerance(
            float(self.get_parameter('linear_tolerance').value))
        self.controller.set_angular_tolerance(
            float(self.get_parameter('angular_tolerance').value))

        self.controller.set_max_linear_speed(
            float(self.get_parameter('max_linear_speed').value))
        self.controller.set_min_linear_speed(
            float(self.get_parameter('min_linear_speed', 0).value))
        self.controller.set_max_angular_speed(
            float(self.get_parameter('max_angular_speed').value))
        self.controller.set_min_angular_speed(
            float(self.get_parameter('min_angular_speed').value))
        self.controller.set_max_linear_acceleration(
            float(self.get_parameter('max_linear_acceleration').value))
        self.controller.set_max_angular_acceleration(
            float(self.get_parameter('max_angular_acceleration').value))

        # Set whether to allow movement backward. Backward movement is
        # safe if the robot can avoid obstacles while traveling in
        # reverse. We default to forward movement only since many
        # sensors are front-facing.
        self.controller.set_forward_movement_only(
            bool(self.get_parameter('forwardMovementOnly').value))

        self.init_pose()
        self.goal = None

        self.action_server.start()
        rospy.spin()

    def on_execute(self, goal):
        self.goal = self.get_angle_pose(goal.pose.pose)
        rclpy.get_logger().info('Goal: (%f,%f,%f)', self.goal.x, self.goal.y,
                      self.goal.theta)

        success = True
        while rclpy.ok() and self.goal is not None:
            # Allow client to preempt the goal.
            if self.action_server.is_preempt_requested():
                rclpy.get_logger().info('Goal preempted')
                self.send_velocity(0, 0)
                self.action_server.set_preempted()
                success = False
                break
            self.publish()
            self.rate.sleep()

        result = GoToPoseResult()
        result.success = success
        self.action_server.set_succeeded(result)

    def init_pose(self):
        self.pose = pose.Pose()
        self.pose.x = 0
        self.pose.y = 0
        self.pose.theta = 0

    def publish(self):
        if self.controller.at_goal(self.pose, self.goal):
            desired = pose.Pose()
        else:
            desired = self.controller.get_velocity(self.pose, self.goal,
                                                   self.dT)

        # if self.goal is not None \
        #    and (desired.xVel!=0.0 or desired.thetaVel!=0.0):
        #     rospy.loginfo(
        #         'current=(%f,%f,%f) goal=(%f,%f,%f)  xVel=%f thetaVel=%f',
        #         self.pose.x, self.pose.y, self.pose.theta,
        #         self.goal.x, self.goal.y, self.goal.theta,
        #         desired.xVel, desired.thetaVel)

        d = self.controller.get_goal_distance(self.pose, self.goal)
        self.dist_pub.publish(d)

        self.send_velocity(desired.xVel, desired.thetaVel)

        # Forget the goal if achieved.
        if self.controller.at_goal(self.pose, self.goal):
            rclpy.get_logger().info('Goal achieved')
            self.goal = None
            msg = Bool()
            msg.data = True
            self.goal_achieved_pub.publish(msg)

    def send_velocity(self, xVel, thetaVel):
        twist = Twist()
        twist.linear.x = xVel
        twist.angular.z = thetaVel
        self.twist_pub.publish(twist)

    def on_odometry(self, newPose):
        self.pose = self.get_angle_pose(newPose.pose.pose)

    def on_goal(self, goal):
        self.action_client.wait_for_server()
        action_goal = GoToPoseGoal()
        action_goal.pose.pose = goal.pose
        self.action_client.send_goal(action_goal)

    def get_angle_pose(self, quaternion_pose):
        q = [quaternion_pose.orientation.x,
             quaternion_pose.orientation.y,
             quaternion_pose.orientation.z,
             quaternion_pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q)

        angle_pose = pose.Pose()
        angle_pose.x = quaternion_pose.position.x
        angle_pose.y = quaternion_pose.position.y
        angle_pose.theta = yaw
        return angle_pose


def main():
    node = GoToGoalNode()

if __name__ == '__main__':
    main()

