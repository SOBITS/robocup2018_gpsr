#!/usr/bin/env python
# coding: utf-8
import rospy, rosparam
import sys

from std_msgs.msg import *
from subprocess import Popen
from turtlebot_edu.srv import *
from turtlebot_edu.msg import *
from google_speech_recognition.msg import *

# ----- Global ----- #
Follow_me_start_flag = Bool()
# ------------------ #

def Follow_me_start_flag_cb(msg):
    global Follow_me_start_flag
    Follow_me_start_flag = msg.data

def processing():
    global Follow_me_start_flag

    pub_word = rospy.Publisher("/speech_word", String, queue_size=1)
    pub_follow_me_start_flag = rospy.Publisher("/target_follower_group/follow_ctrl", Bool, queue_size=1)

    sub_flag = rospy.Subscriber("/target_follower_group/follow_ctrl", Bool, Follow_me_start_flag_cb)

    Popen(["roslaunch", "target_scan_follower", "target_scan_follower.launch"])
    rospy.sleep(3)

    pub_word.publish("I will follow you.")
    rospy.sleep(2)
    pub_word.publish("Please say 'start!', and if you want me stop following, please say 'stop!'.")

    Popen(["roslaunch", "voice_trigger", "voice_trigger_follow_me.launch"])
    rospy.sleep(2)

    while not rospy.is_shutdown():
        if Follow_me_start_flag == True:
            rospy.loginfo("GPSR : Follow_me -> start following")
            break

        else:
            rospy.loginfo("GPSR : Follow_me -> waiting to say 'start!'.")
            rospy.sleep(3)



    while not rospy.is_shutdown():
        if Follow_me_start_flag == False:
            rospy.loginfo("GPSR : Follow_me -> stop following")

            Popen(["rosnode", "kill", "/obstacle_extractor"])
            Popen(["rosnode", "kill", "/obstacle_publisher"])
            Popen(["rosnode", "kill", "/obstacle_tracker"])
            Popen(["rosnode", "kill", "/target_follower_group/target_scan_follower"])
            Popen(["rosnode", "kill", "/julius_follow_me"])
            Popen(["rosnode", "kill", "/socket_client_follow_me"])

            rospy.sleep(2)
            Popen(["rosnode", "cleanup"])
            rospy.sleep(1)

            break

        else:
            rospy.loginfo("GPSR : Follow_me -> Following")
            rospy.sleep(1)


    sub_flag.unregister()



if __name__ == "__main__":
    rospy.init_node("follow_me_test")
    processing()
