#!/usr/bin/env python
# -*- coding:utf-8 -*-
import rospy
import sys
from std_msgs.msg import String, Bool
from turtlebot_edu.srv import *
from turtlebot_edu.msg import *
from subprocess import Popen
from command_analyzer.msg import *
from nltk_command_analyzer.msg import *

Command = Command_data()
Get_cmd_flag = Bool()

def Get_cmd_cb(msg):
    global Command, Get_cmd_flag
    Command.data = msg.data
    Get_cmd_flag = True

def processing():
    global Command
    global Get_cmd_flag

    sub_cmd = rospy.Subscriber("/command_data", Command_data, Get_cmd_cb)
    pub_word = rospy.Publisher("/speech_word", String, queue_size=1)
    pub_joint_pose = rospy.Publisher("/joint_goal", dynamixel_pos_id, queue_size=10)

    #command_analyzer_for_gpsrを立ち上げる
    Popen(['roslaunch', 'command_analyzer', 'command_analyzer_for_gpsr.launch'])

    # 音声認識のタイミングを調整するプログラム
    Popen(['rosrun', 'gpsr', 'recognition_start_controller.py'])
    rospy.sleep(1)

    # 頭を並行にするjointを設定(joint_parallel)
    joint_parallel = dynamixel_pos_id()
    joint_parallel.position = 2048
    joint_parallel.id = 3

    # 頭を斜め上にするjointを設定(joint_bottom)
    joint_top = dynamixel_pos_id()
    joint_top.position = 1900
    joint_top.id = 3

    # 顔を斜め上に上げる
    pub_joint_pose.publish(joint_top)
    rospy.sleep(1)

    Get_cmd_flag = False

    pub_word.publish('please give me a command')

    while not rospy.is_shutdown():
        if Get_cmd_flag == True:
            break
        else:
            pass

    Popen(['rosnode','kill','/command_selector'])
    Popen(['rosnode','kill','/command_analyzer_for_gpsr'])
    Popen(['rosnode', 'kill', '/recognition_start_controller'])
    Popen(['rosnode', 'cleanup'])

    sub_cmd.unregister()

    # 顔をまっすぐにする
    pub_joint_pose.publish(joint_parallel)


    return Command


if __name__ == "__main__":
    rospy.init_node("wait_command")
    processing()
