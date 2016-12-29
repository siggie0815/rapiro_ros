#!/usr/bin/env python

import math

import rospy
import rapiro_msgs.msg as rapiro
import sensor_msgs.msg as sensor
import trajectory_msgs.msg as traj


class RapiroMsgRelay:
    """
         RapiroMsgRelay translates standard messages used to communicate in the ROS world to lightweight Rapiro ROS
         messages used for rosserial communication with the Arduino controller.
    """

    # mapping from joint names to IDs and vice versa
    joint_names = {rapiro.JointIDs.HEAD: 'head_jnt', 'head_jnt': rapiro.JointIDs.HEAD,
                   rapiro.JointIDs.WAIST: 'waist_jnt', 'waist_jnt': rapiro.JointIDs.WAIST,
                   rapiro.JointIDs.R_SHLD_R: 'r_shld_r_jnt', 'r_shld_r_jnt': rapiro.JointIDs.R_SHLD_R,
                   rapiro.JointIDs.R_SHLD_P: 'r_shld_p_jnt', 'r_shld_p_jnt': rapiro.JointIDs.R_SHLD_P,
                   rapiro.JointIDs.R_HAND: 'r_hand_jnt', 'r_hand_jnt': rapiro.JointIDs.R_HAND,
                   rapiro.JointIDs.L_SHLD_R: 'l_shld_r_jnt', 'l_shld_r_jnt': rapiro.JointIDs.L_SHLD_R,
                   rapiro.JointIDs.L_SHLD_P: 'l_shld_p_jnt', 'l_shld_p_jnt': rapiro.JointIDs.L_SHLD_P,
                   rapiro.JointIDs.L_HAND: 'l_hand_jnt', 'l_hand_jnt': rapiro.JointIDs.L_HAND,
                   rapiro.JointIDs.R_FOOT_Y: 'r_foot_y_jnt', 'r_foot_y_jnt': rapiro.JointIDs.R_FOOT_Y,
                   rapiro.JointIDs.R_FOOT_P: 'r_foot_p_jnt', 'r_foot_p_jnt': rapiro.JointIDs.R_FOOT_P,
                   rapiro.JointIDs.L_FOOT_Y: 'l_foot_y_jnt', 'l_foot_y_jnt': rapiro.JointIDs.L_FOOT_Y,
                   rapiro.JointIDs.L_FOOT_P: 'l_foot_p_jnt', 'l_foot_p_jnt': rapiro.JointIDs.L_FOOT_P}

    def __init__(self):
        self.odom = None

        # Range translation Rapiro -> ROS
        self.range_msg = sensor.Range()
        self.range_msg.header.frame_id = 'usr_frame'
        self.range_msg.radiation_type = sensor.Range.ULTRASOUND
        self.range_msg.field_of_view = math.radians(45.0)
        self.range_msg.min_range = 0.02
        self.range_msg.max_range = 4.0
        self.range_pub = rospy.Publisher('range', sensor.Range, queue_size=10)
        self.range_sub = rospy.Subscriber('range_raw', rapiro.Range, self.range_cb)

        # JointState translation Rapiro -> ROS
        self.joint_state_msg = sensor.JointState()
        self.joint_state_msg.name = [self.joint_names[id] for id in range(rapiro.JointIDs.N_JONTS)]
        self.joint_state_pub = rospy.Publisher('joint_states', sensor.JointState, queue_size=10)
        self.joint_state_sub = rospy.Subscriber('joint_states_raw', rapiro.JointState, self.joint_state_cb)

        # JointTrajectroy translation ROS -> Rapiro
        self.active_trajectory = None
        self.trajectory_msg = None
        self.trajectory_pub = rospy.Publisher('traj_cmd_raw', rapiro.Trajectory, queue_size=10)
        self.trajectory_sub = rospy.Subscriber('traj_cmd', traj.JointTrajectory, self.trajectory_cb)

    def range_cb(self, in_msg):
        self.range_msg.header.stamp = in_msg.stamp
        self.range_msg.range = in_msg.range / 100.0
        self.range_pub.publish(self.range_msg)

    def joint_state_cb(self, in_msg):
        if len(in_msg.pos) != len(self.joint_state_msg.name):
            rospy.logwarn('Number of JointStates do not match!')
        self.joint_state_msg.header.stamp = in_msg.stamp
        self.joint_state_msg.position = [math.radians(p) for p in bytearray(in_msg.pos)]
        self.joint_state_pub.publish(self.joint_state_msg)
        if self.odom:
            self.odom.new_joint_state(self.joint_state_msg)

    def trajectory_cb(self, traj_msg):
        self.trajectory_msg = rapiro.Trajectory()
        self.trajectory_msg.stamp = traj_msg.header.stamp
        if len(traj_msg.points) and len(traj_msg.joint_names):
            self.active_trajectory = traj_msg
            self.trajectory_msg.id = [self.joint_names[id] for id in self.active_trajectory.joint_names]
            self.trajectory_execution_cb(None)
        else:
            self.trajectory_pub.publish(self.trajectory_msg)
            self.trajectory_msg = None

    def trajectory_execution_cb(self, event):
        cur_traj_pt = self.active_trajectory.points.pop(0)
        self.trajectory_msg.tgt_pos = [round(math.degrees(p)) for p in cur_traj_pt.positions]
        self.trajectory_msg.tgt_time = [round(cur_traj_pt.time_from_start.to_sec() * 10.0)]
        self.trajectory_pub.publish(self.trajectory_msg)
        if len(self.active_trajectory.points):
            rospy.Timer(cur_traj_pt.time_from_start, self.trajectory_execution_cb, oneshot=True)
        else:
            self.active_trajectory = None
            self.trajectory_msg = None

    def set_odom(self, odom):
        self.odom = odom
