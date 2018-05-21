#!/usr/bin/env python
# -*- coding:utf-8 -*-
import rospy
import sys
import csv
import rosparam
import roslib.packages

from subprocess import *
from std_msgs.msg import String, Bool
from turtlebot_edu.srv import *
from turtlebot_edu.msg import *
from google_speech_recognition.msg import *

pkg_name = rospy.get_param('/gpsr/pkg_name')
csv_filename = rospy.get_param('/gpsr/csv_filename')
pkg_path = roslib.packages.get_pkg_dir(pkg_name)
csv_filepath = pkg_path + '/config/' + csv_filename

# ----- Global変数 ----- #
Recog_word = SpeechRecognitionCandidates()
Ans_list = []
Find_ans_flag = Bool()
# --------------------- #


def Recog_word_cb(msg):
    global Recog_word
    if len(msg.transcript) == 0:
        pass
    else:
        Recog_word = msg.transcript[0]

def Read_csv_file():
    global csv_filepath
    global Ans_list

    try:
        f = open(csv_filepath, 'r')
        dataReader = csv.reader(f)
        for row in dataReader:
            Ans_list.append(row)
            #rospy.loginfo(row)
        rospy.loginfo("GPSR : Answer -> Read csv file [SUCCESS]")

    except:
        rospy.logerr("GPSR : Answer -> Read csv file [ERROR]")

def processing():
    global Recog_word
    global Ans_list
    global Find_ans_flag

    Find_ans_flag = False

    pub_word = rospy.Publisher("/speech_word", String, queue_size=1)
    pub_recognition_start_flag = rospy.Publisher("/recognition_start", Bool, queue_size=1)
    pub_joint_pose = rospy.Publisher("/joint_goal", dynamixel_pos_id, queue_size=10)

    sub_word = rospy.Subscriber("/recognition_word_array", SpeechRecognitionCandidates, Recog_word_cb) # 音声認識した文字列のsubscribe

    pub_recognition_start_flag.publish(False)

    r = Read_csv_file() #csvファイルの読み込み

    # 頭を並行にするjointを設定(joint_parallel)
    joint_parallel = dynamixel_pos_id()
    joint_parallel.position = 2048
    joint_parallel.id = 3

    # 頭を斜め上にするjointを設定(joint_bottom)
    joint_top = dynamixel_pos_id()
    joint_top.position = 1900
    joint_top.id = 3

    rospy.loginfo("GPSR : Answer -> start ")

    pub_joint_pose.publish(joint_top)
    rospy.sleep(1)


    while not rospy.is_shutdown():
        rospy.loginfo("GPSR : Answer -> waiting for a question")
        rospy.sleep(1)
        pub_word.publish('Prease give me a question.')
        pub_recognition_start_flag.publish(False)
        rospy.sleep(2)
        pub_recognition_start_flag.publish(True)
        rospy.sleep(3)

        if len(str(Recog_word)) == 29: # 29はRecog_wordの認識結果が空だったときに文字列の長さ
            rospy.logwarn("GPSR : Answer -> Recog_word is Empty")
            rospy.sleep(1)

        else:
            Find_ans_flag = False
            rospy.loginfo("GPSR : Answer -> Recog_word[%s]", str(Recog_word))
            for i in range(len(Ans_list)):

                for j in range(len(Ans_list[i])-1):
                    if Ans_list[i][j+1] in str(Recog_word):
                        if Find_ans_flag == True:
                            pass
                        else:
                            rospy.loginfo("GPSR : Answer -> Found keyword in Recog_word")
                            Find_ans_flag = True
                            rospy.sleep(1)
                    else:
                        pub_recognition_start_flag.publish(False)
                        break

                if Find_ans_flag == True:
                    ans_word = str(Ans_list[i][0])
                    rospy.loginfo("GPSR : Answer -> Answer[%s]", ans_word)
                    pub_word.publish(ans_word)
                    rospy.sleep(len(ans_word)/30)
                    rospy.sleep(3)
                    break

        if Find_ans_flag == True:
            Find_ans_flag = False
            break

        rospy.sleep(3)
        pub_recognition_start_flag.publish(False)

    rospy.loginfo("GPSR : Answer -> finish")
    sub_word.unregister()

    pub_joint_pose.publish(joint_parallel)
    rospy.sleep(1)

if __name__ == "__main__":
    rospy.init_node("answer_test")
    processing()
