#!/usr/bin/env python
# -*- coding:utf-8 -*-
import rospy
import cv2
import sys

def processing(img_filepath):
    rospy.loginfo("GPSR state : Key_trigger -> start")
    rospy.loginfo("key_trigger image file: " + img_filepath)

    img = cv2.imread(img_filepath, 1)
    
    while not rospy.is_shutdown():
        cv2.imshow('Please Push Enter Key.', img)
        key = cv2.waitKey(1000)
        if key == 13:#enter
            rospy.loginfo("GPSR state : Key_trigger -> OK")
            cv2.destroyAllWindows()
            break
        else:
            rospy.loginfo("GPSR state : Key_trigger -> waiting")

if __name__ == "__main__":
    img_filepath = "~/catkin_ws/src/gpsr/config/key_trigger.JPG"
    processing(img_filepath)
