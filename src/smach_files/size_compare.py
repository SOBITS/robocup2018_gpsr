#!/usr/bin/env python
# coding: utf-8
import rospy, rosparam
import sys

from std_msgs.msg import *
from subprocess import Popen
from object_recog.srv import *
from object_recog.msg import *

#===== Global変数 =====#
Rect_array = []
Object_name_list = []
#======================#

def chunked(iterable, n): # リストを４つごとに区切る関数
    return [iterable[x:x + n] for x in range(0, len(iterable), n)]

def rect_array_cb(req):
    global Rect_array
    rospy.wait_for_service('object_rect')
    rect = list(req.data)
    Rect_array = chunked(rect, 4)

def object_name_list_cb(msg):
    global Object_name_list
    Object_name_list = list(msg.data)

def processing(SIZE):
    global Rect_array
    global Object_name_list

    rect_srv = rospy.ServiceProxy("object_rect", rect_to_str)
    rect_sub = rospy.Service("object_rect", rect_to_str, rect_array_cb)

    object_name_list_sub = rospy.Subscriber("/detect_object_name_array", stringArray, object_name_list_cb)

    # 既に物体検出のノードが動いているとうまく動かないので一度切る
    Popen(["rosnode", "kill",  "/pcl_obj_detect"])
    Popen(["rosnode", "kill", "/caffe_obj_classification"])
    Popen(["rosnode", "cleanup"])

    rospy.sleep(2)

    # object_recogを再び立ち上げる
    Popen(['roslaunch', 'object_recog', 'ObjectDetect.launch'])

    rospy.sleep(3)

    while not rospy.is_shutdown():
        if len(Object_name_list) == 0:
            rospy.logwarn("There is no object")
            rospy.sleep(1)

        elif SIZE == "big":
            max_size = 0 # 句形の大きさの設定（最初は0）
            element_num = 0 # 配列の要素番号
            for i in range(len(Rect_array)):
                i_size = int(Rect_array[i][2])*int(Rect_array[i][3])
                if i_size > max_size:
                    max_size = i_size
                    element_num = i
                else:
                    pass

            return str(Object_name_list[element_num])
            break

        elif SIZE == "small":
            min_size = 100000 # 句形の大きさの設定（最初は10000）
            element_num = 0 # 配列の要素番号
            for i in range(len(Rect_array)):
                i_size = int(Rect_array[i][2])*int(Rect_array[i][3])
                if i_size < min_size:
                    min_size = i_size
                    element_num = i
                else:
                    pass

            return str(Object_name_list[element_num])
            break

        else:
            rospy.logerr("big/small only")
            return False
# ====================== while文終了 ======================== #



    Popen(["rosnode", "kill",  "/pcl_obj_detect"])
    rospy.sleep(1)
    Popen(["rosnode", "kill", "/caffe_obj_classification"])
    rospy.sleep(1)
    Popen(["rosnode", "cleanup"])

    rect_sub.unregister()
    object_name_list_sub.unregister()



if __name__ == "__main__":
    processing("big")
