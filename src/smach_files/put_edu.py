#!/usr/bin/env python
# coding: utf-8
import rospy
from std_msgs.msg import String,Bool
from turtlebot_edu.srv import *

def processing(right_hand_grasp,left_hand_grasp):						#呼び出される関数
	print "put_motion start. #############"

	rospy.wait_for_service('/put_motion_edu_service')
	try:
		put = rospy.ServiceProxy("/put_motion_edu_service", put_motion_edu)
		flag = put(right_hand_grasp.data,left_hand_grasp.data)
		return flag
	except rospy.ServiceException, e:
	    print "Service call failed: %s"%e

	return "?"



if __name__ == '__main__':	#この部分は「$ python sub.py」の時には実行される
	print "do processing() @put_motion.py"
	processing()
