#!/usr/bin/env python
# -*- coding:utf-8 -*-
import rospy
import smach
import smach_ros
import csv
import sys
import termios
import re
import tf
import cv2
import roslib.packages
from subprocess import *
from std_msgs.msg import *
from geometry_msgs.msg import *

from nltk_command_analyzer.msg import *
from turtlebot_edu.msg import dynamixel_pos_id
from object_recog.msg import stringArray
from turtlebot_edu.srv import *
from door_open_detector.srv import *

from smach_files import *

#Global変数
Command = Command_data()
Report_word = ''
right_hand_grasp = Bool()
left_hand_grasp = Bool()

class Key_trigger(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['door_open'])
		self.pub_word = rospy.Publisher("/speech_word", String, queue_size=10)
		self.img_filepath = rospy.get_param('/gpsr/key_trigger_img_path')

	def execute(self, userdata):
		#詳細な処理の内容は./smach_files/key_trigger.pyに記述
		k = key_trigger.processing(self.img_filepath)
		self.pub_word.publish('start GPSR program')
		rospy.sleep(3)
		return 'door_open'


class Door_open(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['wait_command', 'door_open'])
		self.pub_word = rospy.Publisher("/speech_word", String, queue_size=1)

	def execute(self, userdata):
		#詳細な処理の内容は./smach_files/door_open.pyに記述
		d = door_open.processing()
		rospy.loginfo("GPSR state : Doar_open -> True")
		return 'wait_command'


class Wait_command(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['again','check_point'])
		self.sub = rospy.Subscriber("/command_data", Command_data, self.Get_cmd_cb)
		self.pub_word = rospy.Publisher("/speech_word", String, queue_size=1)
		self.get_cmd = False
		self.pub_recognition_start_flag = rospy.Publisher("/recognition_start", Bool, queue_size=1)

	def Get_cmd_cb(self, msg):
		global Command
		Command.data = msg.data
		self.get_cmd = True

	def execute(self, userdata):
		#command_analyzer_for_gpsrを立ち上げる
		Popen(['roslaunch', 'command_analyzer', 'command_analyzer_for_gpsr.launch'])
		self.get_cmd = False

		self.pub_word.publish('please give me a command')
		rospy.sleep(2)
		self.pub_recognition_start_flag.publish(True)
		rospy.sleep(5)

		while not rospy.is_shutdown():
			if self.get_cmd == True:
				self.pub_recognition_start_flag.publish(False)
				rospy.loginfo("GPSR : Wait_command -> Get Command Succsess")
				break
			else:
				self.pub_recognition_start_flag.publish(True)
				rospy.loginfo("GPSR : Wait_command -> Waiting Command")
				rospy.sleep(5)

		Popen(['rosnode','kill','/rosbridge_websocket'])
		Popen(['rosnode','kill','/command_selector'])
		Popen(['rosnode','kill','/command_analyzer_for_gpsr'])
		Popen(['rosnode', 'kill', '/rosapi'])

		#一応、wait_commandのstateで行う処理の内容を./smach_files/wait_command.pyに記述したがグローバル変数を扱う部分があるためうまく動かなかった
		#w = wait_command.processing()
		return 'check_point'


class Check_point(smach.State):
	def __init__(self):
		smach.State.__init__(self,
		outcomes=['move', 'find_person', 'follow_me', 'object_recognition', 'object_grasp', 'answer', 'put_object', 'find_object', 'report'])

	def execute(self, userdata):
		global Command
		rospy.loginfo("GPSR: Check_point --> " + str(Command.data[0].state))
		return Command.data[0].state


class Move(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['again','finish'])
		self.pub_word = rospy.Publisher("/speech_word", String, queue_size=1)
		self.pub_target_place = rospy.Publisher("/waypoint_nav/move_ctrl", String, queue_size=1)
		self.arrive_msg = ''
		self.target_place = ''

	def Arrive_cb(self, msg):
		self.arrive_msg = msg.data
		rospy.loginfo('GPSR : Moving -> arrive_msg: %s', self.arrive_msg)

	def execute(self, userdata):
		global Command
		self.target_place = Command.data[0].target
		self.pub_word.publish('i will move to '+ self.target_place)
		rospy.sleep(3)
		self.pub_target_place.publish(self.target_place)

		while not rospy.is_shutdown():
			m = move.processing(self.target_place)
			if m == "SUCCESS":
				rospy.loginfo('GPSR : Moving -> Arrive')
				self.pub_word.publish("I have arrived at " + self.target_place + ".")
				break
			else:
				rospy.loginfo('GPSR : Moving -> moving now')
				rospy.sleep(2)

		return 'finish'


class Find_person(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['again','finish'])
		self.pub_word = rospy.Publisher("/speech_word", String, queue_size=1)
	

	def execute(self, userdata):
		Popen(['roslaunch', 'person_detector', 'PersonDetect.launch'])
		while not rospy.is_shutdown():
			self.pub_word.publish("I'm looking for a person.")
			#処理の詳細は./smach_files/find_person.pyを参照、人検出に成功すると「found person」と返す関数
			f = find_person.processing(self.person_num)
			if f == "found person":
				return 'finish'
				break
			else:
				pass


class Follow_me(smach.State):#Must re wrote
	def __init__(self):
		smach.State.__init__(self, outcomes=['again','finish'])
	#	self.sub_word = rospy.Subscriber("/recongition_word", String, self.Voice_cb)
		self.pub_word = rospy.Publisher("/speech_word", String, queue_size=1)
		self.follow_finished = False
		self.follow_started = False

	#def Voice_cb(self, data):
	#	if 'Follow' in data.data or 'follow' in data.data or 'for me' in data.data:
	#		self.pub_word.publish('i will follow you. Please stand in front of me')
	#		rospy.sleep(3)
	#		self.follow_started = True
	#	elif 'Finish' in data.data or 'Stop' in data.data or 'stop' in data.data or 'finish' in data.data or 'Arrive' in data.data:
	#		self.pub_word.publish('I will finish following person.')
	#		rospy.sleep(3)
	#		self.follow_finished = True

	def execute(self, userdata):
		self.pub_word.publish('I will follow you. Please say follow me.')
		rospy.loginfo('I will follow you. Please say follow me.')

		self.follow_finished = False
		self.follow_started = False
		#while not rospy.is_shutdown():
		#	if self.follow_started == True:
		#		break;
		#	else:
		#		rospy.loginfo('Please say follow me')
		#		rospy.sleep(1)

		#Popen(['roslaunch', 'turtlebot_follower', 'follower.launch'])

		self.pub_word.publish('I will follow you.')
		rospy.loginfo('following person start')

		#while not rospy.is_shutdown():
		#	if self.follow_finished == True:
		#		break
		#	else:
		#		rospy.sleep(1)
		#		rospy.loginfo('following person finished')

		return 'finish'


class Object_recognition(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['again','finish'])
		self.pub_word = rospy.Publisher("/speech_word", String, queue_size=1)
		self.sub_object_name = rospy.Subscriber("/detect_object_name_array", stringArray, self.Detect_object_name)
		self.trials_num = 10#試行回数
		self.detect_count = 0
		self.target_obj = ''
		self.detect_obj = []

	def Detect_object_name(self, msg):
		self.detect_obj = msg.data
		self.detect_count += 1

	def execute(self, userdata):
		global Command
		self.target_obj = Command.data[0].target

		ob = find_object.processing(self.target_obj)

		if ob == 'OK':
			return 'finish'

		else:
			return 'again'

#		if not self.target_obj:
#			rospy.logerr('GPSR : Obj_recog -> Target Object Empty')
#			return 'finish'

		#顔を下に向ける処理？

#		self.detect_count = 0
#		self.detect_obj = []
#		rospy.loginfo('GPSR : Obj_recog -> target_obj : %s', self.target_obj)
#		Popen(['roslaunch', 'object_recog', 'ObjectDetect.launch'])

#		while not rospy.is_shutdown():
#			if self.target_obj in self.detect_obj:
#				rospy.loginfo('GPSR : Obj_recog -> detect %s', self.target_obj)
#				self.pub_word.publish('i found ' + self.target_obj)
#				break
#			elif self.detect_count > self.trials_num:#タイムアウト,物が見つからない
				#顔の角度を変える的な処理
#				pass
#			else:
#				rospy.loginfo('GPSR : Obj_recog -> detecting now')
#				rospy.sleep(1)

#		Popen(['rosnode', 'kill', '/pcl_obj_detect'])
#		Popen(['rosnode', 'kill', '/caffe_obj_classification'])
		#Change Next State
#		return 'finish'


class Object_grasp(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['again','finish'])
		self.sub = rospy.Subscriber('/grasp_flag', Bool, self.Grasp)
		self.pub_grasp_ctrl = rospy.Publisher("/grasp_ctrl", String, queue_size=10)
		self.pub_word = rospy.Publisher("/speech_word", String, queue_size=1)
		self.target_obj = ''
		self.grasp_judge_msg = False

	def Grasp(self, msg):
		self.grasp_judge_msg = msg.data

	def execute(self, userdata):
		"""
		global Command
		self.grasp_judge_msg = False
		self.target_object = Command.data[0].target
		if not self.target_object:
			rospy.logerr('GPSR : Object_grasp -> Target Object Empty')
			return 'finish'

		rospy.loginfo('GPSR :Object_grasp -> target_obj : %s', self.target_object)
		self.pub_grasp_ctrl.publish(self.target_object)

		while not rospy.is_shutdown():
			if self.grasp_judge_msg == True:
				rospy.loginfo('GPSR : Object_grasp -> grasped %s', self.target_object)
				self.pub_word.publish('I grasped ' + self.target_object)
				break
			else:
				rospy.loginfo('GPSR : Object_grasp -> grasping now')
				rospy.sleep(1)

		#Change Next State
		return 'finish'
		"""
		rospy.loginfo("GPSR : object_grasp -> grasping start")
		global Command
		global right_hand_grasp
		global left_hand_grasp

		self.target_obj = Command.data[0].target
		rospy.loginfo("target_obj: " + str(self.target_obj))

		# 処理の詳細は./smach_files/grasping_op.pyを参照
		g = grasping_op.processing(self.target_obj)

		# 確認用
		print "掴むモーションができたか [%s]"%(g.res_bool)
		print "右手に物を掴んでいるか [%s]"%(g.right_hand_grasp)
		print "左手に物を掴んでいるか [%s]"%(g.left_hand_grasp)

		#物体を掴んでいるか否か
		right_hand_grasp.data = g.right_hand_grasp
		left_hand_grasp.data = g.left_hand_grasp

		while not rospy.is_shutdown():
			# 物体把持に成功
			if g.res_bool == True:
				rospy.loginfo('GPSR : Object_grasp -> grasped %s', self.target_obj)
				self.pub_word.publish("I'm grasping.")
				break

			# 物体把持に失敗
			else:
				rospy.loginfo('GPSR : Object_grasp -> grasping now')
				rospy.sleep(1)

		return 'finish'


class Answer(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['again','finish'])
		self.sub_word = rospy.Subscriber("/recognition_word", String, self.Voice_cb)
		self.pub_word = rospy.Publisher("/speech_word", String, queue_size=1)
		self.recog_msg = ''
		self.ans_list = []

		self.pkg_name = rospy.get_param('/gpsr/pkg_name')
		self.csv_filename = rospy.get_param('/gpsr/csv_filename')
		self.pkg_path = roslib.packages.get_pkg_dir(self.pkg_name)
		self.csv_filepath = self.pkg_path + '/config/' + self.csv_filename
		try:
			f = open(self.csv_filepath, 'r')
			dataReader = csv.reader(f)
			for row in dataReader:
				self.ans_list.append(row)
				rospy.loginfo(row)
		except:
			rospy.logerr('GPSR : Answer -> load csv file error')

	def Voice_cb(self, msg):
		self.recog_msg = msg.data

	def execute(self, userdata):

		self.pub_word.publish('Prease give me a question.')
		rospy.sleep(3)
		self.recog_msg = ''

		while not rospy.is_shutdown():
			if self.recog_msg != '':
				for i in range(len(self.ans_list)):
					find_flag = False
					for key in range(len(self.ans_list[i])-1):
						if self.ans_list[i][key+1] in self.recog_msg_check.data:
							#Find keyword
							find_flag = True
						else:
							break
					if find_flag == True:
						line = String()
						line.data = self.ans_list[i][0]
						self.pub_word_ans.publish(line)
						rospy.loginfo('GPSR : Answer -> %s', self.ans_list[i][0])

						#Change Next State
						return 'finish'


class Report(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['again','finish'])
		self.pub_word = rospy.Publisher("/speech_word", String, queue_size=1)

	def execute(self, userdata):

		global Command
		global Report_word
		target = Command.data[0].target

		if target == 'report_sentence':
			self.pub_word.publish('I will report it.' + Report_word)
		else:
			self.pub_word.publish('I will report it.' + Report_word)

		#Change Next State
		return 'finish'


class Put_object(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['again','finish'])
		#self.sub = rospy.Subscriber('/place_judge', String, self.Place)
		#self.pub_place_ctrl = rospy.Publisher("/place_ctrl", String, queue_size=10)
		self.pub_word = rospy.Publisher("/speech_word", String, queue_size=1)
		#self.place_judge_msg = ''
		self.target_obj = ''

	#def Place(self, msg):
		#self.place_judge_msg = msg.data
		#rospy.loginfo('GPSR : Put_object -> place_judge = %s', self.place_judge_msg)

	def execute(self, userdata):
		global Command
		global right_hand_grasp
		global left_hand_grasp
		"""
		self.place_judge_msg = ''
		self.target_obj = Command.data[0].target
		self.pub_place_ctrl.publish(self.target_obj)

		while not rospy.is_shutdown():
			if self.target_obj in self.place_judge_msg:
				rospy.loginfo('GPSR : Put_object -> place %s Succsess', self.target_obj)
				self.pub_word.publish('i placed ' + self.target_obj)
				break
			else:
				rospy.loginfo('GPSR : Put_object -> placing now')
				rospy.sleep(1)
		#Change Next State
		return 'finish'
		"""
		self.pub_word.publish("I will put the object.")
		rospy.loginfo('GPSR : Put_object -> start')

		self.target_obj = Command.data[0].target
		p = put_op.processing(right_hand_grasp,left_hand_grasp)

		#物体を置いたあとの掴んでいるか否か
		print "右手に物を掴んでいるか [%s]"%(p.after_right_hand_grasp)
		print "左手に物を掴んでいるか [%s]"%(p.after_left_hand_grasp)
		right_hand_grasp.data = p.after_right_hand_grasp
		left_hand_grasp.data = p.after_left_hand_grasp

		print "右手に物を掴んでいるか [%s]"%(right_hand_grasp)
		print "左手に物を掴んでいるか [%s]"%(left_hand_grasp)

		rospy.loginfo('GPSR : Put_object -> success')
		return 'finish'


class Find_object(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['again','finish'])

	def execute(self, userdata):
		global Command
		self.target_obj = Command.data[0].target

		ob = find_object.processing(self.target_obj)

		if ob == 'OK':
			return 'finish'

		else:
			return 'again'



class Is_finish(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['not_finish','finish'])
		self.pub_word = rospy.Publisher("/speech_word", String, queue_size=1)

	def execute(self, userdata):
		global Command
		if len(Command.data) >= 1:
			Command.data.pop(0)
			return 'not_finish'
		else:
			self.pub_word.publish('i will finish GPSR program')
			rospy.sleep(1)
			return 'Finish'




def main():
	rospy.init_node('gpsr_main')
	rospy.loginfo('gpsr main start')

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['Finish'])

	with sm:
		smach.StateMachine.add('Key_trigger', Key_trigger(),
								transitions={'door_open':'Door_open'})

		smach.StateMachine.add('Door_open', Door_open(),
								transitions={'wait_command':'Wait_command',
										'door_open':'Door_open'})

		smach.StateMachine.add('Wait_command', Wait_command(),transitions={'again':'Wait_command',
										'check_point':'Check_point'})

		smach.StateMachine.add("Check_point", Check_point(), transitions={'move':'Move',
										'find_person':'Find_person',
										'follow_me':'Follow_me',
										'object_recognition':'Object_recognition',
										'object_grasp':'Object_grasp',
										'answer':'Answer',
										'put_object':'Put_object',
										'find_object':'Find_object',
										'report':'Report'})

		smach.StateMachine.add('Move', Move(), transitions={'again':'Move',
									'finish':'Is_finish'})

		smach.StateMachine.add('Find_person', Find_person(), transitions={'again':'Find_person',
										'finish':'Is_finish'})

		smach.StateMachine.add('Object_recognition', Object_recognition(), transitions={'again':'Object_recognition',
												'finish':'Is_finish'})

		smach.StateMachine.add('Follow_me', Follow_me(), transitions={'again':'Follow_me',
										'finish':'Is_finish'})

		smach.StateMachine.add('Object_grasp', Object_grasp(), transitions={'again':'Object_grasp',
											'finish':'Is_finish'})

		smach.StateMachine.add('Answer', Answer(), transitions={'again':'Answer',
									'finish':'Is_finish'})

		smach.StateMachine.add('Report', Report(), transitions={'again':'Report',
									'finish':'Is_finish'})

		smach.StateMachine.add('Put_object', Put_object(), transitions={'again':'Put_object',
										'finish':'Is_finish'})

		smach.StateMachine.add('Find_object', Find_object(), transitions={'again':'Find_object',
										'finish':'Is_finish'})
		smach.StateMachine.add('Is_finish', Is_finish(), transitions={'not_finish':'Check_point',
									'finish':'Finish'})


	# Create and start the introspection server
	sis = smach_ros.IntrospectionServer('server_name', sm, '/GPSR_TASK')
	sis.start()

	# Execute SMACH plan
	outcome = sm.execute()
	rospy.spin()
	sis.stop()




if __name__ == '__main__':
	main()
