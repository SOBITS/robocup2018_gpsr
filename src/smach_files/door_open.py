#!/usr/bin/env python
# -*- coding:utf-8 -*-
import rospy
import sys
from turtlebot_edu.srv import *
from door_open_detector.srv import *
from subprocess import *
from std_msgs.msg import String, Bool

#Global変数
door_state = Bool()

# ドアの状態（開いているか閉じているか）をsubscribeするcallback
def door_open_detector_cb(msg):
    global door_state
    door_state = msg.data

#移動用のサービスを定義
def wheel_send_order(str_order):
    service_odom = rospy.ServiceProxy('/robot_ctrl/odom_base_ctrl', odom_base)
    try:
        res = service_odom(str_order)
        return res.res_str
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s"%e)

def processing():
    global door_state
    sub_door_status = rospy.Subscriber('/door_open_detector/door_open_state', Bool, door_open_detector_cb)
    pub_word = rospy.Publisher('/speech_word', String, queue_size=1)


    Popen(['roslaunch','door_open_detector','door_open_detector.launch'])

    #移動用のサービスが立ち上がるまで待つ
    rospy.wait_for_service('/robot_ctrl/odom_base_ctrl')

    while not rospy.is_shutdown():
        if door_state == True:
            rospy.loginfo("GPSR state : Doar_open -> True")
            break
        else:
            rospy.loginfo("GPSR state : Doar_open -> False")
            rospy.sleep(1)

    pub_word.publish('I detected that a door opened.')
    rospy.sleep(3)

    #部屋に入るために１００ｃｍ前進
    wheel_send_order("S:100")

    sub_door_status.unregister()

    Popen(['rosnode','kill','/door_open_detector'])


if __name__ == "__main__":
    processing()
