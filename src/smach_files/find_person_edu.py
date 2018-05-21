#!/usr/bin/env python
# -*- coding:utf-8 -*-
import rospy
import sys
from std_msgs.msg import String, UInt8
from subprocess import Popen
from turtlebot_edu.srv import *
from turtlebot_edu.msg import *
from ssd_node.msg import *

#Global変数
objects_name = stringArray()
trials_num = 20 #顔検出の試行回数
objects_num = UInt8()

def name_cb(msg):
    global objects_name
    objects_name = msg.data

def num_cb(msg):
    global objects_num
    objects_num = msg.data

def processing():
    rospy.init_node("ssd_sub", anonymous=True)
    global objects_num
    global trials_num

    sub_num = rospy.Subscriber("/ssd_object_detect/objects_num", UInt8, num_cb)
    pub_word = rospy.Publisher('/speech_word', String, queue_size=1)

    rospy.loginfo("GPSR : Find_person -> start")
    Popen(['roslaunch','ssd_node','face_detect.launch'])

    rospy.sleep(3)

    trials_count = 1 #人検出の試行回数と比べるための変数

    while not rospy.is_shutdown():
        pub_word.publish("I'm looking for a person.")
        rospy.loginfo("GPSR : Find_person -> " + "person detection("+str(trials_count) + "回目)")

        if objects_num == 1: #face_detectで検出できるのは顔のみ、したがって検出物体が１ならばそれは顔である（と推測される
            rospy.loginfo("GPSR : Find_person -> find a person")
            pub_word.publish("I found a person.")
            Popen(["rosnode", "kill", "/ssd_object_detect/ssd_node"])
            return "Found"
            break

        elif trials_count == trials_num: # 顔の検出に失敗
            rospy.loginfo("GPSR : Find_person -> can NOT find a person")
            pub_word.publish("Sorry, I could not find a person.")
            Popen(["rosnode", "kill", "/ssd_object_detect/ssd_node"])
            return "Not found"
            break

        else:
            trials_count += 1
            rospy.loginfo("GPSR : Find_person -> looking for a person")
            #####################
            #何かしらの頭を動かす処理#
            #####################
            rospy.sleep(3)


if __name__ == "__main__":
    processing()
