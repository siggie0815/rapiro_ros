#!/usr/bin/env python

import rospy
import rapiro_cmd

if __name__ == '__main__':
    rospy.init_node('rapiro_cmd_node')
    rospy.loginfo('Starting rapiro_cmd_node')

    relay = rapiro_cmd.RapiroMsgRelay()
    relay.set_odom(rapiro_cmd.RapiroOdom())

    cmd = rapiro_cmd.RapiroMotionCmd(relay)

    rospy.spin()