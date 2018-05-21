#!/usr/bin/env python
# -*- coding:utf-8 -*-
import rospy, rosparam
import cv2
import sys
from std_msgs.msg import *
from subprocess import *
from turtlebot_edu.srv import *
from turtlebot_edu.msg import *
from geometry_msgs.msg import Pose

def processing(msg):
    move_flag = rosparam.get_param("/gpsr/do_move")

    if not msg:
        rospy.logerr("GPSR : Move -> No destination")

    # command_analyzerで既に登録してあるlocationのリストを読み込む
    location_list = rosparam.load_file("/home/sobit-x3/catkin_ws/src/command_analyzer/dataForGPSR/location_list_gpsr.yaml")

    rospy.wait_for_service('/waypoint_nav/move_ctrl')

    if move_flag == False:
        rospy.logwarn("GPSR : skipped moving")
        return "SUCCESS"

    else:
        # dynamixelを動かすnodeへ角度をpub 2048が並行　2200が斜め下を向く
        pub_joint_pose = rospy.Publisher("/joint_goal", dynamixel_pos_id, queue_size=10)
        joint = dynamixel_pos_id()
        joint.position = 2400
        joint.id = 3
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
