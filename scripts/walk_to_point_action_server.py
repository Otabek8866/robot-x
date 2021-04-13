#!/usr/bin/env python3

#main module to communicate with ROS
import rospy

#importing built-in modules
import math
import threading

#actionlib module to create an Action server
import actionlib

#from robot_x package, import speech service msg type
from robot_x.srv import SpeechToText

#from standard messages, "Float32" msg type is imported
from std_msgs.msg import Float32

#custom action msg types for action server
from robot_x.msg import WalkToPointAction
from robot_x.msg import WalkToPointResult
from robot_x.msg import WalkToPointFeedback

#custom msg types, for updating robot position data
from robot_x.msg import PositionAndDestination
from robot_x.msg import RobotPosition


#creating a class for Action Server
class Walking:

	def __init__(self, pub1, pub2):
		#creating an Action server with name: 'walk_to_point'
		#received msg type -> WalkToPointAction; callback func -> reach_destination
		self.action_server = actionlib.SimpleActionServer('walk_to_point', WalkToPointAction, self.reach_destination)
		#creating a ROS service client; service name -> 'speech'
		#sending msg type -> SpeechToText
		self.speech_service = rospy.ServiceProxy('speech', SpeechToText)

		#assigning created publisher to the class object ('speed_change'&'position_change')
		self.pub_speed_change = pub1
		self.pub_pos_change = pub2
		#creating speed variable
		self.cur_speed = 0
		#getting max speed value from rosparam server
		self.robot_max_speed = rospy.get_param("robot_max_speed")
		#setting a publishing rate
		self.publisher_rate = rospy.Rate(20)

		#creating current and desired position objects
		self.robot_cur_point = RobotPosition()
		self.robot_goal_point = RobotPosition()
		

	#callback funtion of Action server
	def reach_destination(self, goal):

		#creating two threads, to publish changes in speed and position while doing the action
		#taget is publisher function
		self.thread_speed = threading.Thread(target=self.pub_change_speed)
		self.thread_position = threading.Thread(target=self.pub_change_position)
		
		#to keep threads alive, set flag True 
		self.thread_flag = True

		#from action client msg, extract current position of robot
		self.robot_cur_point.pos_x = goal.destin.cur_x
		self.robot_cur_point.pos_y = goal.destin.cur_y

		#from action client msg, extract desired position values
		self.robot_goal_point.pos_x = goal.destin.dest_x
		self.robot_goal_point.pos_y = goal.destin.dest_y

		#calculating the distance to goal and the time to reach
		distance = round(math.dist([self.robot_cur_point.pos_x, self.robot_cur_point.pos_y], [self.robot_goal_point.pos_x, self.robot_goal_point.pos_y]), 1)
		time_to_reach = int(distance/self.robot_max_speed)
		
		#Informing the user about distance and time to reach
		print("Distance: {} meters || Estimated Time: {} s\n".format(distance, time_to_reach))
		#using speech service client, send texts to Speech Server
		self.speech_service("Distance is {} meters and the estimated time to reach is {} seconds".format(distance, time_to_reach))
		self.speech_service("Action is started")

		#We assume robot walks in 45 angle to the desired position
		#Defining x_step, each second robot walks x_step distance in x coordinate
		if self.robot_cur_point.pos_x < self.robot_goal_point.pos_x:
			step_x = (self.robot_goal_point.pos_x - self.robot_cur_point.pos_x)/time_to_reach
		else:
			step_x = (self.robot_cur_point.pos_x - self.robot_goal_point.pos_x)/time_to_reach
			step_x *= -1

		#Defining y_step, each second robot walks y_step distance in y coordinate
		if self.robot_cur_point.pos_y < self.robot_goal_point.pos_y:
			step_y = (self.robot_goal_point.pos_y - self.robot_cur_point.pos_y)/time_to_reach
		else:
			step_y = (self.robot_cur_point.pos_y - self.robot_goal_point.pos_y)/time_to_reach
			step_y *= -1
		
		#setting current speed values to max speed value
		self.cur_speed = self.robot_max_speed

		#starting threads, they start to publish changes in speed and position independently
		self.thread_speed.start()
		self.thread_position.start()
		
		#interval value, how often to inform users about the distance left
		if time_to_reach < 10:
			interval = 2
		elif time_to_reach < 20:
			interval = 3
		else:
			interval = 4

		#walking to the destination is started
		for i in range(time_to_reach):
			if i%interval == 0:
				#calculating the distance left and sending it to Action client as feedback
				distance = math.dist([self.robot_cur_point.pos_x, self.robot_cur_point.pos_y], [self.robot_goal_point.pos_x, self.robot_goal_point.pos_y])
				self.action_server.publish_feedback(WalkToPointFeedback(distance))

			#sleep 1 second
			rospy.sleep(1)
			
			#increase or decrease the postion cordinates with step_x and step_y values 
			self.robot_cur_point.pos_x += step_x
			self.robot_cur_point.pos_y += step_y

		#after Robot reaches destination, update current position values with destination position values
		self.robot_cur_point.pos_x = self.robot_goal_point.pos_x
		self.robot_cur_point.pos_y = self.robot_goal_point.pos_y

		#setting the speed value to 0
		self.cur_speed = 0

		#waiting threads to publish the last changed values in speed and position
		rospy.sleep(1)

		#setting thread_flag False to stop threads
		self.thread_flag = False

		#wait a little to make sure threads stopped successfully
		rospy.sleep(0.5)

		#add threads to the main thread
		self.thread_speed.join()
		self.thread_position.join()

		#creating Action result object
		result = WalkToPointResult()
		#assigning the time spent to reach the destination
		result.time_spent = time_to_reach
		#answering to client with Action Result msg 
		self.action_server.set_succeeded(result)


	#thread function to publish changes in current speed of robot
	def pub_change_speed(self):

		#checking thread_flag value
		while self.thread_flag:
			#publishing changes
			self.pub_speed_change.publish(self.cur_speed)
			#wait 0.05 second (20 Hz)
			self.publisher_rate.sleep()


	#thread function to publish changes in current position of robot
	def pub_change_position(self):

		#checking thread_flag value
		while self.thread_flag:
			#publishing changes
			self.pub_pos_change.publish(self.robot_cur_point)
			#wait 0.05 second (20 Hz)
			self.publisher_rate.sleep()


if __name__ == '__main__':
	
	try:
		#initializing a new node
		rospy.init_node('walk_to_point_action_server_node')

		#creating a publisher class object to publish speed changes
		#'change_speed' -> topic name; Float32 -> msg type; 'queue_size' -> outgoing message queue used for asynchronous publishing
		pub_speed = rospy.Publisher('change_speed', Float32, queue_size=10)

		#creating a publisher class object to publish position changes
		#'change_position' -> topic name; RobotPosition -> msg type; 'queue_size' -> outgoing message queue used for asynchronous publishing
		pub_position = rospy.Publisher('change_position', RobotPosition, queue_size=10)

		#creating Walking class object, passing publisher objects as args
		server = Walking(pub_speed, pub_position)
		
		#to keep alive the node, continuous spinning
		rospy.spin()

	#only ROS related exceptions will be captured
	except rospy.ROSInterruptException as e:
		print(e)
