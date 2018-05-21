#!/usr/bin/env python
# coding: utf-8
import rospy
from turtlebot_edu.srv import *
from turtlebot_edu.msg import *
from google_speech_recognition.msg import *
from std_msgs.msg import String, Bool
from subprocess import Popen

# global変数宣言
recog_word = SpeechRecognitionCandidates()
recog_name = ""

def Recog_word_cb(msg):
    global recog_word
    if len(msg.transcript) == 0:
        pass
    else:
        recog_word = msg.transcript[0]

def processing(target, option):
    global recog_word
    pub_word = rospy.Publisher("/speech_word", String, queue_size=1)
    sub_word = rospy.Subscriber("/recognition_word_array", SpeechRecognitionCandidates, Recog_word_cb)
    pub_recognition_start_flag = rospy.Publisher("/recognition_start", Bool, queue_size=1)
    pub_joint_pose = rospy.Publisher("/joint_goal", dynamixel_pos_id, queue_size=10)

    pub_recognition_start_flag.publish(False)
    rospy.sleep(2)

    # 頭を並行にするjointを設定(joint_parallel)
    joint_parallel = dynamixel_pos_id()
    joint_parallel.position = 2048
    joint_parallel.id = 3

    # 頭を斜め上にするjointを設定(joint_bottom)
    joint_top = dynamixel_pos_id()
    joint_top.position = 1900
    joint_top.id = 3


    pub_joint_pose.publish(joint_top)
    rospy.sleep(1)


    if not target:
        rospy.logwarn("GPSR : Ask -> target Empty")
        if not option:
            rospy.logerr("GPSR : Ask -> target & option Empty")
        else:
            pass

    # 名前を尋ねる処理
    elif target == "name":
        while not rospy.is_shutdown():
            rospy.loginfo("GPSR : Ask -> target [name]")
            pub_word.publish("Please tell me your name.")
            rospy.sleep(2)
            pub_recognition_start_flag.publish(True)
            rospy.sleep(4)

            # 聞いた文字列が空
            if len(str(recog_word)) == 0:
                rospy.logwarn("GPSR : Ask -> recog_word is Empty")
                rospy.sleep(2)

            else:
                name_list = str(recog_word).split() # 操作しやすくするためにリスト化

                # 文字列から名前だけを抽出
                if "." in name_list:
                    name_list.remove(".")

                if "My" in name_list:
                    name_list.remove("My")

                if "name" in name_list:
                    name_list.remove("name")

                if "is" in name_list:
                    name_list.remove("is")

                if "my" in name_list:
                    name_list.remove("my")

                if "I" in name_list:
                    name_list.remove("I")

                if "am" in name_list:
                    name_list.remove("am")

                if "'m" in name_list:
                    name_list.remove("'m")

                if "called" in name_list:
                    name_list.remove("called")

                if "." in name_list:
                    name_list.remove(".")

                if "," in name_list:
                    name_list.remove(",")

                #if "." in name_list:
                    #name_list.remove(".")

                recog_name = name_list[0]
                rospy.loginfo("GPSR : Ask -> name:[%s]", recog_name)

                if str(recog_name) == "transcript:":
                    rospy.logwarn("GPSR : Ask -> name:[Empty]")
                    rospy.sleep(5)

                else:
                    while not rospy.is_shutdown(): # 名前の確認とyes/noが返ってくるまでループ
                        # 名前の確認
                        pub_word.publish("Let me confirm your name.")
                        rospy.sleep(2)
                        pub_word.publish("You are " + recog_name + ", aren't you? Yes or No.")
                        rospy.sleep(5)
                        pub_recognition_start_flag.publish(True)
                        rospy.sleep(3)

                        # 返事がない
                        if len(str(recog_word)) == 0:
                            rospy.logwarn("GPSR : Ask -> recog_word is Empty")
                            rospy.sleep(2)

                        elif str(recog_word) == "yes" or str(recog_word) == "Yes":
                            break

                        elif str(recog_word) == "no" or str(recog_word) == "No":
                            break

                        else: # yes/no 以外の文字列が返ってきた
                            rospy.logwarn("GPSR : Ask -> please say 'yes' or 'no'")
                            #recog_word = ""
                            pub_word.publish("Please say 'yes' or 'No'.")
                            rospy.sleep(5)

                if str(recog_word) == "yes" or str(recog_word) == "Yes":
                    rospy.loginfo("GPSR : Ask -> ask name Success")
                    pub_word.publish("Thank you, " + recog_name + ".")
                    return_value = "The name of the person is " + recog_name +"."
                    break

                elif str(recog_word) == "no" or str(recog_word) == "No":
                    rospy.logwarn("GPSR : Ask -> ask name Failure")
                    pub_word.publish("I'm sorry")
                    #recog_word = ""
                    rospy.sleep(3)






        else:
            pass

    pub_recognition_start_flag.publish(False)
    rospy.sleep(1)

    pub_joint_pose.publish(joint_parallel)
    rospy.sleep(1)

    sub_word.unregister() # Subscriberの削除

    return return_value





if __name__ == "__main__":
    rospy.init_node("ask_test")
    target = "name"
    option = ""
    while not rospy.is_shutdown():
        processing(target, option)
