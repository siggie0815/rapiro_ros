#!/usr/bin/env python

import copy

import rospy
import rostopic
import std_msgs.msg as std_msgs
import trajectory_msgs.msg as trajectory_msgs


class RapiroMotionCmd:
    """
         RapiroMotionCmd takes motion commands as strings, gets the motion command form parameter server
         and passes the JointTrajectories to the RapiroMessageRelay to be executed.
    """

    def __init__(self, relay):
        self.relay = relay
        self.cmd_sub = rospy.Subscriber('rapiro_cmd', std_msgs.String, self.cmd_cb)
        self.timer = None
        self.trajectory_msg = trajectory_msgs.JointTrajectory()

    def cmd_cb(self, cmd_msg):
        cmd_string = cmd_msg.data

        rospy.loginfo('Trying to get motion under ~motions/' + cmd_string + ': '
                      + str(rospy.has_param('~motions/' + cmd_string)))
        if rospy.has_param('~motions/' + cmd_string):
            # cancel running repeat timer
            if self.timer:
                self.timer.shutdown()
                self.timer = None
            # get motion trajectory
            command = rospy.get_param('~motions/' + cmd_string)
            repeat = command['repeat']
            self.trajectory_msg = trajectory_msgs.JointTrajectory()
            rostopic._fillMessageArgs(self.trajectory_msg, command['motion'])

            # execute motion
            if repeat:
                period = sum([p.time_from_start.secs + p.time_from_start.nsecs / 1000000000.0
                              for p in self.trajectory_msg.points])
                self.timer = rospy.Timer(rospy.Duration(period), self.cmd_repeat_cb)
            self.trajectory_msg.header.stamp = rospy.Time.now()
            self.relay.trajectory_cb(copy.deepcopy(self.trajectory_msg))
        else:
            if cmd_string == 'init':
                self.trajectory_msg = trajectory_msgs.JointTrajectory()
                self.trajectory_msg.header.stamp = rospy.Time.now()
                self.relay.trajectory_cb(copy.deepcopy(self.trajectory_msg))

    def cmd_repeat_cb(self, event):
        self.trajectory_msg.header.stamp = rospy.Time.now()
        self.relay.trajectory_cb(copy.deepcopy(self.trajectory_msg))
