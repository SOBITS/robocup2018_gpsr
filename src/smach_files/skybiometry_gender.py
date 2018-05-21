#!/usr/bin/env python
# coding: utf-8
import rospy, rosparam
import cv2
import sys
import numpy as np
from skybiometry_ros.msg import FaceProperties
from skybiometry_ros.srv import *
from ssd_node.msg import *
from turtlebot_edu.srv import *
from turtlebot_edu.msg import *
from std_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# ----- Global ----- #
Image_raw = Image()
Image_raw_flag = Bool()
Rect_info = BoundingBoxes()
Rect_info_flag = Bool()
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

# skybiometryに画像データを送る
def skybiometry_client(image):
    rospy.wait_for_service('/get_face_properties')
	try:
		send_sky = rospy.ServiceProxy('/get_face_properties', GetFaceProperties)
		sky_ans = send_sky(images)
		return sky_ans
	except rospy.ServiceException, e:
		return -1

def processing():
    global Image_raw
    global Image_raw_flag
    global Rect_info
    global Rect_info_flag

    Image_raw_flag = False
    Rect_info_flag = False

    pub_word = rospy.Publisher("/speech_word", String, queue_size=10)
    pub_face_detection_flag = rospy.Publisher("/ssd_object_detect/detect_ctrl", Bool, queue_size = 10)

    pub_face_detection_flag.publish(False) # SSDは最初は止めておく

    trimed_image = []

    # ----- 顔検出 && 男女判定 ----- #
    pub_face_detection_flag.publish(True)
    sub_image = rospy.Subscriber("/ssd_object_detect/detect_result", Image, image_raw) # 画像のsubscribe
    sub_rect = rospy.Subscriber("/ssd_object_detect/objects_rect", BoundingBoxes, face_rect)	#顔座標のsubscribe

    while not rospy.is_shutdown():
        if Rect_info_flag == True:
            rospy.loginfo("Get face rect ")
            break
        else:
            rospy.logwarn("No face rect")
            rospy.sleep(3)

    people_num = int(len(Rect_info.boundingBoxes))



if __name__ == "__main__":
