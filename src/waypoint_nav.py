#!/usr/bin/env python
# coding: utf-8

"""
	@RoboCup
	coded by kugou on 2017/4/1
"""

import roslib; roslib.load_manifest('turtlebot_teleop')
import rospy
import datetime
from std_msgs.msg import String,Int16,Int32,Float64
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
import sys, termios, re, tf
from subprocess import *
from std_srvs.srv import Empty, EmptyResponse
import math
from tf2_msgs.msg import *
from tf2_ros import *

allow_distance = 2.0

class guide:
	def __init__(self):

		# Initial pose
		self.initial_pose = PoseWithCovarianceStamped()
		self.initial_pose.header.frame_id = "map"
		self.initial_pose.pose.pose = Pose(Point(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.023, 0.999))
		self.pub_initial_pose = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)
		self.pub_initial_pose.publish(self.initial_pose)
	
		self.pub_move_ctrl = rospy.Publisher('/move_ctrl', String, queue_size=1)#arrive_check
		self.pub_arrive_msg = rospy.Publisher('/arrive_msg', String, queue_size=1)
		self.pub_robot_location_pose = rospy.Publisher('/robot_location_pose', Pose, queue_size=1)
		self.sub_move_ctrl = rospy.Subscriber('/move_ctrl', String, self.move_cb)
		self.sub_tf = rospy.Subscriber('/tf', TFMessage, self.tf_cb)
		self.sub_initial_ctrl = rospy.Subscriber('/initial_ctrl', String, self.initial_cb)
		self.odom_point = Point()
		
		self.listener = tf.TransformListener()

		rospy.loginfo('waypoint_nav_initialize_ok')

		self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		self.move_base.wait_for_server(rospy.Duration(100))
		
	def set_position(self, target_name):
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.pose.position.x = rospy.get_param('/' + target_name + '_translation_x')
		goal.target_pose.pose.position.y = rospy.get_param('/' + target_name + '_translation_y')
		goal.target_pose.pose.position.z = rospy.get_param('/' + target_name + '_translation_z')
		goal.target_pose.pose.orientation.x = rospy.get_param('/' + target_name + '_rotation_x')
		goal.target_pose.pose.orientation.y = rospy.get_param('/' + target_name + '_rotation_y')
		goal.target_pose.pose.orientation.z = rospy.get_param('/' + target_name + '_rotation_z')
		goal.target_pose.pose.orientation.w = rospy.get_param('/' + target_name + '_rotation_w')
		return goal

	def set_initial_position(self, target_name):
		initial = PoseWithCovarianceStamped()
		initial.header.frame_id = "map"
		initial.pose.pose.position.x = rospy.get_param('/' + target_name + '_translation_x')
		initial.pose.pose.position.y = rospy.get_param('/' + target_name + '_translation_y')
		initial.pose.pose.position.z = rospy.get_param('/' + target_name + '_translation_z')
		initial.pose.pose.orientation.x = rospy.get_param('/' + target_name + '_rotation_x')
		initial.pose.pose.orientation.y = rospy.get_param('/' + target_name + '_rotation_y')
		initial.pose.pose.orientation.z = rospy.get_param('/' + target_name + '_rotation_z')
		initial.pose.pose.orientation.w = rospy.get_param('/' + target_name + '_rotation_w')
		return initial

	def tf_cb(self,data):
		if True == self.listener.canTransform('map', 'base_footprint', rospy.Time(0)):
			(trans,rot) = self.listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
			self.odom_point.x = trans[0]
			self.odom_point.y = trans[1]
			#print self.odom_point
			position = Pose()
			position.position.x = trans[0]
			position.position.y = trans[1]
			position.position.z = trans[2]
			position.orientation.x = rot[0]
			position.orientation.y = rot[1]
			position.orientation.z = rot[2]
			position.orientation.w = rot[3]
			self.pub_robot_location_pose.publish(position)


	def arrive_check(self,target_name):#到着判定の確認
		target_point = self.set_position(target_name)
		temp_distance = ((target_point.target_pose.pose.position.x - self.odom_point.x)**2) + ((target_point.target_pose.pose.position.y - self.odom_point.y)**2)
		distance = math.sqrt(temp_distance)
		rospy.loginfo("Distance =  %.3f m", distance)
		if distance < allow_distance:#目標地点との距離				
			rospy.loginfo('arrival:' + target_name)
			self.pub_arrive_msg.publish(target_name)
		else:
			self.pub_move_ctrl.publish(target_name)


	def move_cb(self,msg):
		print "move to " + msg.data
		goal = self.set_position(msg.data)
		self.move_base.send_goal(goal)
		waiting = self.move_base.wait_for_result(rospy.Duration(300))
		if waiting == 1:
			self.arrive_check(msg.data)

	def initial_cb(self,msg):
		print "set initial pose " + msg.data
		target_pose = self.set_initial_position(msg.data)
		self.pub_initial_pose.publish(target_pose)


if __name__ == '__main__':
	rospy.init_node('waypoint_nav_test')
	rospy.loginfo('waypoint_nav_start')

	gd = guide()
	
	rospy.spin()
