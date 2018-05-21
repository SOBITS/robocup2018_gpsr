#!/usr/bin/env python
# coding: utf-8
import rospy
from turtlebot_edu.srv import *
from turtlebot_edu.msg import *
from std_msgs.msg import *


def processing(Report_word):
    pub_word = rospy.Publisher("/speech_word", String, queue_size=1)

    if len(Report_word) == 0:
        rospy.logwarn("GPSR : Report -> Report_word Empty")

    else:
        rospy.loginfo("GPSR : Report -> [%s]", Report_word)
        pub_word.publish(Report_word)

    rospy.sleep(5)

    Report_word = ""

if __name__ == "__main__":
    rospy.init_node("report_test")
    Report_word = "Hello, world!"
    processing(Report_word)
