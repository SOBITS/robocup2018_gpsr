#!/usr/bin/env python
# -*- coding:utf-8 -*-
import rospy, rosparam
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

# ----- Global変数 ----- #
Command = Command_data()
Report_word = ""
right_hand_grasp = Bool()
left_hand_grasp = Bool()
Now_place = ""
Task_times = 3 # 実行する命令の回数
Task_count = 1 # 実行した命令の回数をカウントするための変数
# --------------------- #

# 各ステートを実行するか否かをyamlファイルから読み込む
key_triger_flag = rosparam.get_param("/gpsr/do_key_trigger")
door_open_flag = rosparam.get_param("/gpsr/do_door_open")
move_flag = rosparam.get_param("/gpsr/do_move")
find_person_flag = rosparam.get_param("/gpsr/do_find_person")
object_grasp_flag = rosparam.get_param("/gpsr/do_object_grasp")
follow_me_flag = rosparam.get_param("/gpsr/do_follow_me")
answer_flag = rosparam.get_param("/gpsr/do_answer")
report_flag = rosparam.get_param("/gpsr/do_report")
ask_flag = rosparam.get_param("/gpsr/do_ask")
guide_announce_flag = rosparam.get_param("/gpsr/do_guide_announce")
put_object_flag = rosparam.get_param("/gpsr/do_put_object")
find_object_flag = rosparam.get_param("/gpsr/do_find_object")
# ---------------------------- #

class Key_trigger(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['door_open'])
		self.pub_word = rospy.Publisher("/speech_word", String, queue_size=10)
		self.img_filepath = rospy.get_param('/gpsr/key_trigger_img_path')

	def execute(self, userdata):
		global key_triger_flag

		if key_triger_flag == False:
			rospy.logwarn("GPSR : Key_trigger -> skipped")
			return "door_open"

		else:
			#詳細な処理の内容は./smach_files/key_trigger.pyに記述
			k = key_trigger.processing(self.img_filepath)
			self.pub_word.publish('start GPSR program')
			rospy.sleep(3)
			return 'door_open'


class Door_open(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['wait_command', 'door_open'])
		self.pub_word = rospy.Publisher("/speech_word", String, queue_size=1)
		self.pub_target_place = rospy.Publisher("/waypoint_nav/move_ctrl", String, queue_size=1)
		self.target_place = ""

	def execute(self, userdata):
		global door_open_flag

		if door_open_flag == False:
			rospy.logwarn("GPSR : Door_open -> skipped")
			return "wait_command"

		else:
			#詳細な処理の内容は./smach_files/door_open.pyに記述
			d = door_open.processing()
			rospy.loginfo("GPSR state : Doar_open -> True")
			#return 'wait_command'

		# door_openが終わったら命令を聞く場所へ移動する処理
		self.target_place = "wait position"
		self.pub_word.publish("I will go to wait position.")

		self.pub_target_place.publish(self.target_place)
		while not rospy.is_shutdown():
			m = move_op.processing(self.target_place) # wait positionへ到着
			if m == "SUCCESS":
				rospy.loginfo('GPSR : Door_open -> Arrive at wait position')
				break
			else:
				rospy.loginfo('GPSR : Door_open -> Moving to wait position')
				rospy.sleep(3)

		return "wait_command"

class Wait_command(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['again','check_point'])

	def execute(self, userdata):
		global Command
		Command = wait_command.processing()

		return 'check_point'


class Check_point(smach.State):
	def __init__(self):
		smach.State.__init__(self,
		outcomes=['move', 'find_person', 'follow_me', 'object_grasp', 'answer', 'put_object', 'find_object', 'report', 'guide_announce', 'is_finish', 'ask'])

	def execute(self, userdata):
		global Command

		if len(Command.data) != 0:
			rospy.loginfo("GPSR: Check_point -> " + str(Command.data[0].state))
			rospy.sleep(1)
			return Command.data[0].state

		else:
			rospy.logwarn("GPSR: Check_point -> There is no Command")
			rospy.sleep(1)
			return 'is_finish'


class Move(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['again','finish'])
		self.pub_word = rospy.Publisher("/speech_word", String, queue_size=1)
		self.pub_target_place = rospy.Publisher("/waypoint_nav/move_ctrl", String, queue_size=1)
		self.target_place = ""

	def execute(self, userdata):
		global Command
		global move_flag
		global Now_place

		self.target_place = Command.data[0].target
		self.pub_word.publish('I will move to '+ self.target_place)
		rospy.sleep(3)

		if move_flag == False: # ステートを飛ばす
			rospy.logwarn("GPSR : Move -> skipped")
			rospy.loginfo('GPSR : Moving -> Arrive')
			self.pub_word.publish("I have arrived at " + self.target_place + ".")

		else:
			self.pub_target_place.publish(self.target_place)
			while not rospy.is_shutdown():
				m = move_op.processing(self.target_place)
				if m == "SUCCESS":
					rospy.loginfo('GPSR : Moving -> Arrive')
					self.pub_word.publish("I have arrived at " + self.target_place + ".")
					break
				else:
					rospy.loginfo('GPSR : Moving -> moving now')
					rospy.sleep(3)

		Now_place = self.target_place

		self.target_place = "" # 目的地の初期化
		return 'finish'


class Find_person(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['again','finish'])
		self.pub_word = rospy.Publisher("/speech_word", String, queue_size=1)

	def execute(self, userdata):
		global Command
		global find_person_flag
		global Report_word
		global Now_place

		target = str(Command.data[0].target)
		option = str(Command.data[0].option)

		if find_person_flag == False:
			rospy.logwarn("GPSR : Find_person -> skipped")
			return "finish"

		else:
			#処理の詳細は./smach_files/find_person_op.pyを参照、人検出に成功すると「Found」と返す関数
			f = find_person.processing(target, option)

			target = "" #初期化
			option = "" #初期化

			if f == "Found":
				return 'finish'
			elif f == False:
				return "again"
			else:
				Report_word = f
				return 'finish'


class Follow_me(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['again','finish'])

	def execute(self, userdata):
		global follow_me_flag

		if follow_me_flag == False:
			rospy.logwarn("GPSR : Follow_me -> skipped")
			return "finish"

		else:
			f_m = follow_me.processing()
			return 'finish'


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
		rospy.loginfo("GPSR : object_grasp -> grasping start")
		global Command
		global right_hand_grasp
		global left_hand_grasp
		global object_grasp_flag

		if object_grasp_flag == False:
			rospy.logwarn("GPSR : Object_grasp -> skipped")
			return "finish"

		else:
			self.target_obj = Command.data[0].target
			rospy.loginfo("target_obj: " + str(self.target_obj))

			# 処理の詳細は./smach_files/grasping_op.pyを参照
			g = grasping_op.processing(self.target_obj)

			#物体を掴んでいるか否か
			#right_hand_grasp.data = g.right_hand_grasp
			#left_hand_grasp.data = g.left_hand_grasp

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

	def execute(self, userdata):
		global answer_flag

		if answer_flag == False:
			rospy.logwarn("GPSR : Answer -> skipped")
			return "finish"

		else:
			ans = answer.processing()
			return "finish"


class Report(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['again','finish'])

	def execute(self, userdata):
		global Command
		global Report_word
		global report_flag

		report_target = Command.data[0].target

		if report_flag == False:
			rospy.logwarn("GPSR : Report -> skipped")
			return "finish"

		else:
			if str(report_target) == "report_sentence":
				re = report.processing(Report_word)
			else:
				re = report.processing(report_target)
			return 'finish'

		Report_word = "" # 初期化
		report_target = "" # 初期化


class Ask(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['again','finish'])
		self.pub_word = rospy.Publisher("/speech_word", String, queue_size=1)

	def execute(self, userdata):
		global Command
		global ask_flag
		global Report_word

		if ask_flag == False:
			rospy.logwarn("GPSR : Ask -> skipped")
			return "finish"

		else:
			target = str(Command.data[0].target)
			option = str(Command.data[0].option)

			# ask.processingは返り値が Report_word で発話させたい文章
			a = ask.processing(target, option)
			Report_word = a

			#Change Next State
			return 'finish'


class Guide_announce(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['again','finish'])
		self.pub_word = rospy.Publisher("/speech_word", String, queue_size=1)

	def execute(self, userdata):
		global Command
		rospy.loginfo("GPSR : Guide_announce -> start")
		destination = str(Command.data[1].target) # 目的地は次の命令のtarget

		self.pub_word.publish("I guide you to the " + destination + " .")
		rospy.sleep(1)
		self.pub_word.publish("Please, follow me.")
		rospy.sleep(3)
		return 'finish'


class Put_object(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['again','finish'])
		self.pub_word = rospy.Publisher("/speech_word", String, queue_size=1)
		self.target = ""

	def execute(self, userdata):
		global Command
		global right_hand_grasp
		global left_hand_grasp
		global put_object_flag

		if put_object_flag == False:
			rospy.logwarn("GPSR : Put_object -> skipped")
			return "finish"

		else:
			self.target = Command.data[0].target
			self.pub_word.publish("I will put the " + str(self.target) + ".")

			p = put_op.processing(right_hand_grasp,left_hand_grasp)

			right_hand_grasp.data = p.after_right_hand_grasp
			left_hand_grasp.data = p.after_left_hand_grasp

			rospy.loginfo('GPSR : Put_object -> success')
			self.target = "" # 初期化
			return 'finish'


class Find_object(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['again','finish','wait_command'])
		self.pub_target_place = rospy.Publisher("/waypoint_nav/move_ctrl", String, queue_size=1)
		self.pub_word = rospy.Publisher("/speech_word", String, queue_size=1)
		self.target_obj = ""
		self.target_place = ""
		self.option = ""

	def execute(self, userdata):
		global Command
		global Report_word
		global find_object_flag
		global Now_place

		if find_object_flag == False:
			rospy.logwarn("GPSR : Find_object -> skipped")
			return "finish"

		else:
			self.target_obj = Command.data[0].target
			self.option = Command.data[0].option
			#self.target_place = "starting_position" # もし見つからなかったら命令者の元に帰る
			self.target_place = "wait position"

			fb = find_object.processing(self.target_obj, self.option, Now_place)

			# 別のファイルで記述した部分で、nodeを削除する処理を組んだがうまく反映されないので、smach側で消去する
			Popen(["rosnode", "kill",  "/pcl_obj_detect", "/caffe_obj_classification"])
			Popen(["rosnode", "cleanup"])

			if fb == 'OK': # target_objが見つかったら”OK”と返す
				return 'finish'

			elif fb == 'not OK':
				Command = Command_data() # 命令のステートを全部消去する
				rospy.loginfo("GPSR : Find_object -> All command states were deleted.")

				rospy.loginfo("GPSR : Find_object -> return to the oparator")
				self.pub_word.publish("I couldn't find" + str(self.target_obj) + ", so I return to the oparator.")

				# oparatorの場所に戻る処理
				while not rospy.is_shutdown():
					m = move_op.processing(self.target_place)
					if m == "SUCCESS":
						rospy.loginfo('GPSR : Find_object -> Arrive at the oparator')
						self.pub_word.publish("I have returned to the oparator.")
						rospy.sleep(1)
						break
					else:
						rospy.loginfo('GPSR : Find_object -> moving to the oparator')
						rospy.sleep(2)

				self.pub_word.publish("Sorry, I couldn't find " + str(self.target_obj) + ".")
				return "wait_command"

			# 物体の個数 / 個数が返ってくる場合は返り値はint型
			elif isinstance(fb, int) == True:
				Report_word = "There are " + str(fb) + " " + str(self.target_obj) +  "."
				return "finish"

			# "OK", "not OK"以外のstr型が返ってくる場合はReport_wordで発話してほしい文字列
			elif isinstance(fb, str) == True:
				Report_word = fb
				return "finish"

			else:
				return 'again'


class Is_finish(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['not_finish','finish','wait_command'])
		self.pub_word = rospy.Publisher("/speech_word", String, queue_size=1)
		self.pub_target_place = rospy.Publisher("/waypoint_nav/move_ctrl", String, queue_size=1)
		self.target_place = ''

	#移動用のサービスを定義
	def wheel_send_order(self, str_order):
	    service_odom = rospy.ServiceProxy('/robot_ctrl/odom_base_ctrl', odom_base)
	    try:
	        res = service_odom(str_order)
	        return res.res_str
	    except rospy.ServiceException, e:
	        rospy.logerr("Service call failed: %s"%e)

	def execute(self, userdata):
		global Command
		global Task_times, Task_count
		global Now_place

		rospy.loginfo(Command)

		# 命令のステートがまだ残っている
		if len(Command.data) >= 1:
			Command.data.pop(0)
			rospy.loginfo('GPSR : Is_finish -> not finish')
			return 'not_finish'

		# 命令のステートがすべて終了
		else:
			rospy.loginfo("GPSR : Is_finish -> " + str(Task_count) + "回目のタスクが終了")

			# ３回目のタスクが終了　ー＞　GPSR終了
			if Task_times == Task_count:
				rospy.loginfo("GPSR : Is_finish -> go to the exit")
				self.pub_word.publish('I go to the exit.')
				self.target_place = "exit"
				self.pub_target_place.publish(self.target_place)

				rospy.sleep(2)

				while not rospy.is_shutdown():
					m = move_op.processing(self.target_place)
					if m == "SUCCESS":
						rospy.loginfo('GPSR : Is_finish -> Arrive at the exit')
						self.pub_word.publish("I have gone out of the room.")
						#######################
						# 決め打ちで外に出る操作？ #
						#部屋から出るために１００ｃｍ前進
					    #wheel_send_order("S:100")
						#######################
						break
					else:
						rospy.loginfo('GPSR : Is_finish -> moving to the exit')
						rospy.sleep(3)

				rospy.sleep(3)
				self.pub_word.publish('GPSR program is finished.')
				return 'finish'

			#　GPSRはまだ終わらない
			else:
				Task_count += 1
				rospy.loginfo('GPSR : Is_finish -> return to the oparator')
				self.pub_word.publish('I return to the oparator.')
				#self.target_place = "starting_position"
				self.target_place = "wait position"
				self.pub_target_place.publish(self.target_place)

				rospy.sleep(2)

				while not rospy.is_shutdown():
					m = "SUCCESS"
					if m == "SUCCESS":
						rospy.loginfo('GPSR : Is_finish -> Arrive at the oparator')
						self.pub_word.publish("I have returned to the oparator.")
						rospy.sleep(1)
						break
					else:
						rospy.loginfo('GPSR : Is_finish -> moving to the oparator')
						rospy.sleep(3)

				Now_place = self.target_place

				return 'wait_command'


#--------------------------------------------#
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
										'object_grasp':'Object_grasp',
										'answer':'Answer',
										'put_object':'Put_object',
										'find_object':'Find_object',
										'guide_announce':'Guide_announce',
										'is_finish' : 'Is_finish',
										'report':'Report',
										'ask':'Ask'})

		smach.StateMachine.add('Move', Move(), transitions={'again':'Move',
									'finish':'Is_finish'})

		smach.StateMachine.add('Find_person', Find_person(), transitions={'again':'Find_person',
										'finish':'Is_finish'})

		smach.StateMachine.add('Follow_me', Follow_me(), transitions={'again':'Follow_me',
										'finish':'Is_finish'})

		smach.StateMachine.add('Object_grasp', Object_grasp(), transitions={'again':'Object_grasp',
											'finish':'Is_finish'})

		smach.StateMachine.add('Answer', Answer(), transitions={'again':'Answer',
									'finish':'Is_finish'})

		smach.StateMachine.add('Report', Report(), transitions={'again':'Report',
									'finish':'Is_finish'})

		smach.StateMachine.add('Ask', Ask(), transitions={'again':'Ask',
									'finish':'Is_finish'})

		smach.StateMachine.add('Guide_announce', Guide_announce(), transitions={'again':'Guide_announce',
									'finish':'Is_finish'})

		smach.StateMachine.add('Put_object', Put_object(), transitions={'again':'Put_object',
										'finish':'Is_finish'})

		smach.StateMachine.add('Find_object', Find_object(), transitions={'again':'Find_object',
										'finish':'Is_finish', 'wait_command':'Wait_command'})

		smach.StateMachine.add('Is_finish', Is_finish(), transitions={'not_finish':'Check_point',
									'wait_command':'Wait_command', 'finish':'Finish'})


	# Create and start the introspection server
	sis = smach_ros.IntrospectionServer('server_name', sm, '/GPSR_TASK')
	sis.start()

	# Execute SMACH plan
	outcome = sm.execute()
	rospy.spin()
	sis.stop()




if __name__ == '__main__':
	main()
