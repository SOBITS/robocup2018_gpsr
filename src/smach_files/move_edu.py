#!/usr/bin/env python
# -*- coding:utf-8 -*-
import rospy
import cv2
import sys
from std_msgs.msg import *
from subprocess import *
from turtlebot_edu.srv import *
from turtlebot_edu.msg import *
from geometry_msgs.msg import Pose

def processing(msg):
    rospy.wait_for_service('/waypoint_nav/move_ctrl')

    # dynamixelを動かすnodeへ角度をpub 2048が並行　2200が斜め下を向く
    pub_joint_pose = rospy.Publisher("/joint_goal", dynamixel_pos_id, queue_size=10)
    joint = dynamixel_pos_id()
    joint.position = 2200
    joint.id = 1
    #print joint
    rospy.sleep(1)
    pub_joint_pose.publish(joint)


    try:
        move_ctrl = rospy.ServiceProxy('/waypoint_nav/move_ctrl', waypoint_nav)
        location_name = String()
        location_pose = Pose()

        flag = move_ctrl(msg, location_pose)
        return flag.result_text

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    processing()
