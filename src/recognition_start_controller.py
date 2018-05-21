#!/usr/bin/env python
# coding: utf-8
import rospy, rosparam
import sys

from std_msgs.msg import *
from subprocess import Popen
from object_recog.srv import *
from object_recog.msg import *
from nltk_command_analyzer.msg import *
from google_speech_recognition.msg import *


# Global変数 #
Picotts_flag = Bool()
Get_cmd_flag = Bool()
Picotts_word = String()
Old_picotts_word = String()
Recog_word_flag = Bool()

def Picotts_flag_cb(msg): # picottsでの発話が終わったらTrueが送られる
    global Picotts_flag
    Picotts_flag = True

def Get_cmd_flag_cb(msg): # 命令を聞き取れたかどうかをflagで制御
    global Get_cmd_flag
    Get_cmd_flag = True

def Picotts_word_cb(msg): # picottsで発話した文字列を格納する
    global Picotts_word
    Picotts_word = msg.data

def Recog_word_flag_cb(msg):
    global Recog_word_flag
    if len(msg.transcript) != 0: # 音声認識した文字列が空じゃない場合
        Recog_word_flag = True

def processing():
    global Picotts_flag
    global Picotts_word, Old_picotts_word
    global Get_cmd_flag
    global Recog_word_flag

    rospy.init_node("recognition_start_controller")

    pub_recognition_start_flag = rospy.Publisher("/recognition_start", Bool, queue_size=1)
    pub_word = rospy.Publisher("/speech_word", String, queue_size=10)

    sub_picotts_flag = rospy.Subscriber("/speech_end_flag", Bool, Picotts_flag_cb)
    sub_picotts_word = rospy.Subscriber("/speech_word", String, Picotts_word_cb)
    sub_get_cmd_flag = rospy.Subscriber("/command_data", Command_data, Get_cmd_flag_cb)
    sub_recog_word_flag = rospy.Subscriber("/recognition_word_array", SpeechRecognitionCandidates, Recog_word_flag_cb)

    Picotts_flag = False
    counter = 0 #返事がなかった場合の処理用のカウンター


    while not rospy.is_shutdown():
        rospy.sleep(1)
        pub_recognition_start_flag.publish(False)

        # picotts_word が空の場合を避けるための処理
        if len(str(Picotts_word)) != 0 or str(Picotts_word) != "data: ''":
            Old_picotts_word = Picotts_word

        if Get_cmd_flag == True:
            pub_recognition_start_flag.publish(False)
            rospy.loginfo("GPSR : Wait_command -> Get Command Succsess")
            break

        elif Picotts_flag == True: # picottsでの発話終了
            Picotts_flag = False
            rospy.loginfo("GPSR : Wait_command -> /speech_word [%s]", str(Old_picotts_word))

            if str(Old_picotts_word) == "please give me a command":
                pub_recognition_start_flag.publish(True)

            elif str(Old_picotts_word) == "Sorry, please repeat a full sentence.":
                pub_recognition_start_flag.publish(True)

            elif str(Old_picotts_word).find("is that correct?") != -1:
                pub_recognition_start_flag.publish(True)

            elif str(Old_picotts_word).find("Pardon me?") != -1:
                pub_recognition_start_flag.publish(True)

            elif str(Old_picotts_word).find("?") != -1:
                pub_recognition_start_flag.publish(True)

            elif str(Old_picotts_word) == "start GPSR program":
                pass

            elif str(Old_picotts_word) == "data: ''":
                pass

            else: # picottsで発話した文字列が想定外
                rospy.logwarn("GPSR : wait_command -> Unexpected picotts word [%s]", str(Old_picotts_word))

            rospy.sleep(10)
            if Recog_word_flag == False: # 音声認識出来なかった場合
                pub_word.publish(str(Picotts_word))


        rospy.sleep(1)

    # ----- while文終了 ----- #

    Picotts_flag = Bool()
    Get_cmd_flag = Bool()
    Picotts_word = String()
    Old_picotts_word = String()
    Recog_word_flag = Bool()


    sub_picotts_flag.unregister()
    sub_picotts_word.unregister()
    sub_get_cmd_flag.unregister()
    sub_recog_word_flag.unregister()
if __name__ == "__main__":
    processing()
