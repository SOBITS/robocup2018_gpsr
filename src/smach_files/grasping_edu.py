#!/usr/bin/env python
# coding: utf-8
import rospy
from turtlebot_edu.srv import *



def processing(msg):						#呼び出される関数
	print "grasping start. #############"
	rospy.wait_for_service('grasp_obj_edu_service')
	try:
		grasp = rospy.ServiceProxy('grasp_obj_edu_service', grasp_ctr_edul)
		print "把持対象物体　[%s]"%(msg.data)
		flag = grasp(msg.data)
		return flag
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e



if __name__ == '__main__':	#この部分は「$ python sub.py」の時には実行される
	print "do processing() @grasping.py"
	processing()
