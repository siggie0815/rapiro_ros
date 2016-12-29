#!/usr/bin/env python

import math
from reportlab.pdfbase._cidfontdata import typeFaces_chs

import geometry_msgs.msg as geometry_msgs
import nav_msgs.msg as nav_msgs
import rapiro_msgs.msg as rapiro_msgs
import rospy
import tf.transformations
import tf2_ros as tf2


def isclose(a, b, rel_tol=1e-9, abs_tol=.0):
    abs(a - b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)


class RapiroOdom:
    BOTH = 0
    LEFT = 1
    RIGHT = 2

    def __init__(self):
        # state variables
        self.last_stand = RapiroOdom.BOTH
        self.last_r_yaw = math.pi / 2.
        self.last_l_yaw = math.pi / 2.
        self.last_time = rospy.Time.now()

        # all TF stuff
        self.tf_pub = tf2.TransformBroadcaster()
        self.tf_msg = geometry_msgs.TransformStamped()
        self.tf_msg.header.frame_id = 'odom'
        self.tf_msg.child_frame_id = 'base_link'
        self.tf_msg.transform.rotation.w = 1.

        # all Odometry stuff
        self.odom_pub = rospy.Publisher("odom", nav_msgs.Odometry, queue_size=10)
        self.odom_msg = nav_msgs.Odometry()
        self.odom_msg.header.frame_id = 'odom'
        self.odom_msg.child_frame_id = 'base_link'
        self.odom_msg.pose.pose.orientation.w = 1.

    def new_joint_state(self, joint_state):
        now = rospy.Time.now()

        # find out on wich foot we are standing
        r_foot_p = joint_state.position[rapiro_msgs.JointIDs.R_FOOT_P]
        l_foot_p = joint_state.position[rapiro_msgs.JointIDs.L_FOOT_P]
        stand = RapiroOdom.BOTH
        # right foot
        if r_foot_p > math.pi / 2. and l_foot_p > r_foot_p:
            stand = RapiroOdom.RIGHT
        # left foot
        if l_foot_p < math.pi / 2. and r_foot_p < l_foot_p:
            stand = RapiroOdom.LEFT

        yaw = math.pi / 2.
        last_yaw = yaw
        l = .0

        # get Yaw angle and l, corresponding to current stand
        r_foot_y = joint_state.position[rapiro_msgs.JointIDs.R_FOOT_Y]
        l_foot_y = joint_state.position[rapiro_msgs.JointIDs.L_FOOT_Y]

        # if we are or were standing on the right
        if stand == RapiroOdom.RIGHT or stand == RapiroOdom.BOTH and self.last_stand == RapiroOdom.RIGHT:
            yaw = r_foot_y
            last_yaw = self.last_r_yaw
            l = .027
        # if we are or were standing on the left
        if stand == RapiroOdom.LEFT or stand == RapiroOdom.BOTH and self.last_stand == RapiroOdom.LEFT:
            yaw = joint_state.position[rapiro_msgs.JointIDs.L_FOOT_Y]
            last_yaw = self.last_l_yaw
            l = -.027

        # compute local deltas
        dx = (math.cos(yaw) * l) - (math.cos(last_yaw) * l)
        dy = (math.sin(yaw) * l) - (math.sin(last_yaw) * l)
        dtheta = (yaw - last_yaw) * .75

        # compute global deltas
        [r, p, theta] = tf.transformations.euler_from_quaternion([self.tf_msg.transform.rotation.x,
                                                                  self.tf_msg.transform.rotation.y,
                                                                  self.tf_msg.transform.rotation.z,
                                                                  self.tf_msg.transform.rotation.w])
        dx_g = dx * math.cos(theta) - dy * math.sin(theta)
        dy_g = dx * math.sin(theta) + dy * math.cos(theta)
        q = tf.transformations.quaternion_from_euler(r, p, theta + dtheta)

        # create messages
        self.tf_msg.header.stamp = now
        self.tf_msg.transform.translation.x += dx_g
        self.tf_msg.transform.translation.y += dy_g
        self.tf_msg.transform.rotation.x = q[0]
        self.tf_msg.transform.rotation.y = q[1]
        self.tf_msg.transform.rotation.z = q[2]
        self.tf_msg.transform.rotation.w = q[3]

        self.odom_msg.header.stamp = now
        self.odom_msg.pose.pose.position.x = self.tf_msg.transform.translation.x
        self.odom_msg.pose.pose.position.y = self.tf_msg.transform.translation.y
        self.odom_msg.pose.pose.orientation = self.tf_msg.transform.rotation
        dt = (now - self.last_time).to_sec()
        self.odom_msg.twist.twist.linear.x = dx_g / dt
        self.odom_msg.twist.twist.linear.y = dy_g / dt
        self.odom_msg.twist.twist.angular.z = dtheta / dt

        # send TF and odom msg
        self.tf_pub.sendTransform(self.tf_msg)
        self.odom_pub.publish(self.odom_msg)

        # save state
        self.last_stand = stand
        self.last_r_yaw = r_foot_y
        self.last_l_yaw = l_foot_y
        self.last_time = now
