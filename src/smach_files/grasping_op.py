#!/usr/bin/env python
# coding: utf-8
import rospy
from turtlebot_op.srv import *



def processing(target_obj):						#呼び出される関数
	rospy.wait_for_service('grasp_obj_op_service')
	try:
		grasp = rospy.ServiceProxy('grasp_obj_op_service', grasp_ctrl_op)
		print "把持対象物体　[%s]"%(target_obj)
		flag = grasp(target_obj)
		return flag
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

if __name__ == '__main__':	#この部分は「$ python sub.py」の時には実行される
	print "do processing() @grasping.py"
	processing()

"""
def execute(self, userdata):
	global object_name,right_hand_grasp,left_hand_grasp
	print "============= Grasping ============="
	object_name.data = "potato_chips"#把持対象物体 個数確認すること(len関数を使用する)
	g = grasping_op.processing(object_name)
	#print "掴むモーションができたか [%s]"%(g.res_bool)
	print "右手に物を掴んでいるか [%s]"%(g.right_hand_grasp)
	print "左手に物を掴んでいるか [%s]"%(g.left_hand_grasp)
	#物体を掴んでいるか否か
	right_hand_grasp.data = g.right_hand_grasp
	left_hand_grasp.data = g.left_hand_grasp
	print "================================"
	if g.res_bool==True:
		return 'grasping'
	else:
		return 'again'
"""
