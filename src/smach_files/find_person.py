#!/usr/bin/env python
# coding: utf-8
import rospy, rosparam
import cv2
import sys
import numpy as np
from skybiometry_ros.msg import *
from skybiometry_ros.srv import *
from ssd_node.msg import *
from turtlebot_edu.srv import *
from turtlebot_edu.msg import *
from std_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from subprocess import Popen
from google_speech_recognition.msg import *

# ----- Global ----- #
Image_raw = Image()
Image_raw_flag = Bool()
Rect_info = BoundingBoxes()
Rect_info_flag = Bool()
Recog_word = SpeechRecognitionCandidates()
Neck_swing = False
# ------------------ #

# 画像取得
def image_raw(data):
    global Image_raw
    global Image_raw_flag
    Image_raw = data
    Image_raw_flag = True

# 顔の座標を取得
def face_rect(data):
    global Rect_info
    global Rect_info_flag
    Rect_info = data
    Rect_info_flag = True

# Goole_speech_recognitionから得た文字列を格納
def Recog_word_cb(msg):
    global Recog_word
    if len(msg.transcript) == 0:
        pass
    else:
        Recog_word = msg.transcript[0]

# skybiometryに画像データを送る
def skybiometry_client(images):
    rospy.loginfo("wait for get_face_properties")
    rospy.wait_for_service("/get_face_properties")
    try:
        send_sky = rospy.ServiceProxy("/get_face_properties", GetFaceProperties)
        sky_ans = send_sky(images)
        return sky_ans
    except rospy.ServiceException, e:
        return False

def processing(target, option):
    global Image_raw
    global Image_raw_flag
    global Rect_info
    global Rect_info_flag
    global Recog_word
    global Neck_swing

    Image_raw_flag = False
    Rect_info_flag = False

    Popen(['roslaunch','ssd_node','face_detect.launch'])

    # Publisher
    pub_word = rospy.Publisher("/speech_word", String, queue_size=10)
    pub_face_detection_flag = rospy.Publisher("/ssd_object_detect/detect_ctrl", Bool, queue_size = 10)
    pub_img = rospy.Publisher("/GPSR/Find_person/Trimed_Face", Image, queue_size=10)
    pub_joint_pose = rospy.Publisher("/joint_goal", dynamixel_pos_id, queue_size=10)
    pub_recognition_start_flag = rospy.Publisher("/recognition_start", Bool, queue_size=1)

    # Subscriber
    sub_image = rospy.Subscriber("/ssd_object_detect/detect_result", Image, image_raw) # 画像のsubscribe
    sub_rect = rospy.Subscriber("/ssd_object_detect/objects_rect", BoundingBoxes, face_rect)	#顔座標のsubscribe
    sub_word = rospy.Subscriber("/recognition_word_array", SpeechRecognitionCandidates, Recog_word_cb) # 音声認識した文字列のsubscribe

    pub_face_detection_flag.publish(False) # SSDは最初は止めておく

    trimed_images = [] # トリミングした顔画像を格納する配列

    people_num = 0
    male_num = 0
    female_num = 0

    #　頭を並行にするjointを設定(joint_parallel)
    joint_parallel = dynamixel_pos_id()
    joint_parallel.position = 2048
    joint_parallel.id = 3

    # 頭を斜め上にするjointを設定(joint_bottom)
    joint_top = dynamixel_pos_id()
    joint_top.position = 1900
    joint_top.id = 3

    # 首を真ん中にするjointを設定(joint_neck)
    joint_neck_center = dynamixel_pos_id()
    joint_neck_center.position = 2048
    joint_neck_center.id = 4

    joint_neck_right = dynamixel_pos_id()
    joint_neck_right.position = 1948
    joint_neck_right.id = 4

    joint_neck_left = dynamixel_pos_id()
    joint_neck_left.position = 2148
    joint_neck_left.id = 4

    pub_joint_pose.publish(joint_neck_center)
    rospy.sleep(1)



    # 頭を並行にする
    pub_joint_pose.publish(joint_parallel)
    rospy.sleep(1)

    trial_num = 10
    trial_count = 0

    pub_word.publish("Please stand in front of me.")

    # ----- 顔検出 ---- #
    rospy.loginfo("GPSR : Find_person -> start")
    pub_face_detection_flag.publish(True)


    rospy.sleep(3)

    while not rospy.is_shutdown():
        if Image_raw_flag == True and Rect_info_flag == True:
            rospy.loginfo("Get face rect ")
            pub_face_detection_flag.publish(False)
            Image_raw_flag = False # 初期化
            Rect_info_flag = False # 初期化
            break

        elif trial_num == trial_count:
            rospy.logerr("GPSR : Find_person -> Failure to get face_rect")
            return False

        else:
            rospy.logwarn("No face rect [%s]", str(trial_count))
            if Neck_swing == True:#首を降る処理
                if trial_count %2 == 0:
                    pub_joint_pose.publish(joint_neck_right)
                    rospy.sleep(1)
                else:
                    pub_joint_pose.publish(joint_neck_left)
                    rospy.sleep(1)

            trial_count += 1
            pub_joint_pose.publish(joint_top)
            rospy.sleep(3)

        print Rect_info

    face_rect_num = int(len(Rect_info.boundingBoxes)) # 検出した顔の数

    # 画像から顔の部分だけをトリミング
    for i in range(face_rect_num):
        try:
            cv_image = CvBridge().imgmsg_to_cv2(Image_raw, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge convert error")
            return False

        # トリミング
        trimed_cv_image = cv_image[Rect_info.boundingBoxes[i].y : Rect_info.boundingBoxes[i].y + Rect_info.boundingBoxes[i].height, Rect_info.boundingBoxes[i].x : Rect_info.boundingBoxes[i].x + Rect_info.boundingBoxes[i].width]


        orgHeight, orgWidth = trimed_cv_image.shape[:2]

        #cv2.imshow('image', trimed_cv_image)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()

        # ROSで扱えるように画像を変換
        try:
            trimed_ros_image = CvBridge().cv2_to_imgmsg(trimed_cv_image, "rgb8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge convert error")
            return False

        if orgHeight <= 10 or orgWidth <= 10: # あまりにも小さいとskybiometryで識別できないので予め除く
            rospy.logerr("GPSR : Find_person -> face rect is too small")

        else:
            rospy.loginfo("Image_size:[%s, %s]", str(orgHeight), str(orgWidth))

            pub_img.publish(trimed_ros_image)
            trimed_images.append(trimed_ros_image)

    people_num = int(len(Rect_info.boundingBoxes)) # skybiometryに送った画像の枚数

    if people_num == 1: pub_word.publish("I got a face rect.")
    else: pub_word.publish("I got " + str(people_num) + " face rects.")

    del Rect_info.boundingBoxes[:] # 初期化

    rospy.sleep(3)

# ---------- targetとoptionに合わせた処理の実行 ---------- #
    if option == "name": # 名前判定
        rospy.loginfo("GPSR : Find_person -> option [name]")
        if str(target) == "someone" or str(target) == "operator": # 単なる人検出
            rospy.loginfo("GPSR : Find_person -> Found people !")


        else: # 名前の確認
            target_name = target
            rospy.loginfo("GPSR : Find_person -> name check")
            ##### 名前を確認する処理 #####
            while not rospy.is_shutdown():
                rospy.loginfo("GPSR : Find_person -> ask the name")
                pub_word.publish("Please tell me your name")
                rospy.sleep(2)
                pub_recognition_start_flag.publish(True)
                rospy.sleep(4)

                # 聞いた文字列が空
                if len(str(Recog_word)) == 0:
                    rospy.logwarn("GPSR : Ask -> recog_word is Empty")
                    rospy.sleep(2)

                else:
                    name_list = str(Recog_word).split() # 操作しやすくするためにリスト化
                    # 文字列から名前だけを抽出/余計な部分を削除
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
                    rospy.loginfo("GPSR : Find_person -> name:[%s]", recog_name)

                    rospy.loginfo(recog_name)

                    if str(recog_name) == "transcript:":
                        rospy.logwarn("GPSR : Find_person -> recog_name is Empty")
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
                            if len(str(Recog_word)) == 0:
                                rospy.logwarn("GPSR : Find_person -> ask name [response is Empty]")
                                rospy.sleep(2)

                            elif str(Recog_word) == "yes" or str(Recog_word) == "Yes":
                                break

                            elif str(Recog_word) == "no" or str(Recog_word) == "No":
                                break

                            else:
                                rospy.logwarn("GPSR : Find_person -> ask name [response is ???]")
                                pub_word.publish("Please say 'yes' or 'No'.")
                                rospy.sleep(5)

                        if str(Recog_word) == "yes" or str(Recog_word) == "Yes":
                            rospy.loginfo("GPSR : Find_person -> ask name [Success]")
                            pub_word.publish("Thank you, " + recog_name + ".")
                            break

                        elif str(Recog_word) == "no" or str(Recog_word) == "No":
                            rospy.logwarn("GPSR : Find_person -> ask name [Failure]")
                            pub_word.publish("I'm sorry")
                            rospy.sleep(3)



                        #####
                        ##### target_nameとRecog_wordの比較で検出するのもありかも
                        #####

    # 性別判定
    if target == "male" or target == "female":
        target_gender = target
        rospy.loginfo("GPSR : Find_person -> detect gender")

        # skybiometryに画像データを送る
        result = skybiometry_client(trimed_images)

        if result == False: # 男女判定に失敗
            rospy.logerr("GPSR : Find_person -> detect gender [Failure]")
            pub_word.publish("I could not detect gender.")
            #return False
            return "I could not detect gender."

        else:
            # 男女カウント
            for i in range(people_num):
                if result.properties_array[i].gender == 0:
                    male_num += 1
                elif result.properties_array[i].gender == 1:
                    female_num += 1
                else:
                    pass

        # ターゲットが男性
        if target_gender == "male":
            rospy.loginfo("GPSR : Find_person -> detect gender [target is Male]")
            if male_num == 0: pub_word.publish("I could not find male person.")
            elif male_num == 1: pub_word.publish("I found one male person.")
            else: pub_word.publish("I found male people.")

        # ターゲットが女性
        elif target_gender == "female":
            rospy.loginfo("GPSR : Find_person -> detect gender [target is Female]")
            if female_num == 0: pub_word.publish("I could not find female person.")
            elif female_num == 1: pub_word.publish("I found female person.")
            else: pub_word.publish("I found female people.")

        else:
            rospy.logerr("GPSR : Find_person -> detect gender [target gender is Empty]")


    if option == "count": # 男性/女性の人数を返す
        rospy.loginfo("GPSR : Find_person -> option [count]")
        if target == "male":
            if male_num == 1: return "I found one male person."
            elif male_num == 0: return "There are no male people."
            else: return "I found " + str(male_num) + " male people."

        elif target == "female":
            if female_num == 1: return "I found one female person."
            elif female_num == 0: return "There are no female people."
            else: return "I found " + str(female_num) + " female people."

        else:
            rospy.logwarn("GPSR : Find_person -> count target gender is Empty")
            if people_num == 1: return "I found one person."
            else: return "I found " + str(people_num) + " people."


    elif option == "judge": # 返り値として話してほしい文字列を返す
        rospy.loginfo("GPSR : Find_person -> option [judge]")
        if male_num > female_num: # 男性と判定
            rospy.loginfo("GPSR : Find_person -> judge[male]")
            return "The gender of the person is male."

        elif female_num > male_num: # 女性と判定
            rospy.loginfo("GPSR : Find_person -> judge[female]")
            return "The gender of the person is female."

        else: # 判定失敗
            rospy.logerr("GPSR : Find_person -> judge[Failure]")
            return "I could not detect the gender of the person."

    else: # "Found" を返す
        return "Found"

    del trimed_images[:] # 配列の初期化

    trial_count = 0

    Popen(["rosnode", "kill", "/ssd_object_detect/ssd_node"])
    Popen(["rosnode", "cleanup"])


if __name__ == "__main__":
    rospy.init_node("test")
    target="male"
    option="count"
    while not rospy.is_shutdown():
        result = processing(target, option)
        print result
        rospy.sleep(3)
