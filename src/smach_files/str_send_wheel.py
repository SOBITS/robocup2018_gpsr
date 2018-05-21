#!/usr/bin/env python
# -*- coding:utf-8 -*-
import rospy
import sys
from turtlebot_edu.srv import *
from door_open_detector.srv import *
from subprocess import *
from std_msgs.msg import String, Bool

#移動用のサービスを定義
def wheel_send_order(str_order):
    service_odom = rospy.ServiceProxy('/robot_ctrl/odom_base_ctrl', odom_base)
    try:
        res = service_odom(str_order)
        return res.res_str
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s"%e)

def processing():
    pub_word = rospy.Publisher('/speech_word', String, queue_size=1)

    #移動用のサービスが立ち上がるまで待つ
    rospy.wait_for_service('/robot_ctrl/odom_base_ctrl')

    #200ｃｍ前進
    wheel_send_order("S:200")



if __name__ == "__main__":
    processing()
