#! /usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs
import tf

import time

time.sleep(10)

 # start moveit
moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
rospy.init_node('go_init', anonymous=True)
robot = moveit_commander.robot.RobotCommander()

# add planning group
arm_group = moveit_commander.move_group.MoveGroupCommander("panda_arm")

arm_group.set_named_target('ready')
arm_group.go()

moveit_commander.roscpp_initializer.roscpp_shutdown()