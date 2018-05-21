#!/usr/bin/env python
# coding: utf-8
import rospy
from std_msgs.msg import String,Bool
from turtlebot_op.srv import *

def processing(right_hand_grasp,left_hand_grasp):
    if not right_hand_grasp:
        rospy.logerr('GPSR : Put_object -> right hand grasp flag Empty')

    if not left_hand_grasp:
        rospy.logerr('GPSR : Put_object -> left hand grasp flag Empty')

    rospy.loginfo("GPSR : Put_object -> put motion start ")

    pub_word = rospy.Publisher("/speech_word", String, queue_size=1)

    rospy.wait_for_service('/put_motion_op_service')

    pub_word.publish("Here you are")

    try:
        put = rospy.ServiceProxy("/put_motion_op_service", put_motion_op)
        flag = put(right_hand_grasp.data,left_hand_grasp.data)
        #print flag
        #rospy.loginfo(str(flag))
        return flag

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == '__main__':    #この部分は「$ python sub.py」の時には実行される
    print "do processing() @put_motion.py"
    processing()

"""
global right_hand_grasp,left_hand_grasp
print "============= Put_motion ============="
p = put_op.processing(right_hand_grasp,left_hand_grasp)
#物体を置いたあとの掴んでいるか否か
print "右手に物を掴んでいるか [%s]"%(p.after_right_hand_grasp)
print "左手に物を掴んでいるか [%s]"%(p.after_left_hand_grasp)
right_hand_grasp.data = p.after_right_hand_grasp
left_hand_grasp.data = p.after_left_hand_grasp
print "============= FINISH ============="
return 'put_motion'
"""
