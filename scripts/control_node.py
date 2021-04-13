#!/usr/bin/env python3

#main module to communicate with ROS
import rospy

#actionlib module to create an Action client
import actionlib

#from standard messages, "Float32" msg type is imported
from std_msgs.msg import Float32

#to use speech service, import created msg type
from robot_x.srv import SpeechToText

#importing created msg types to use for Action client and Subscriber to position topic
from robot_x.msg import RobotPosition
from robot_x.msg import PositionAndDestination

#msg types to communicate with Action Server
from robot_x.msg import WalkToPointAction
from robot_x.msg import WalkToPointResult
from robot_x.msg import WalkToPointFeedback
from robot_x.msg import WalkToPointGoal

#importing built-in modules
import os
import math


#creating menu content to send speech service
menu = """
****** Select one of [1,2,3,4,5] ******

---> 1. Current Temperature
---> 2. Current Humidity
---> 3. Current Robot Position
---> 4. Walk to a Point
---> 5. Robot returns to the base
"""

#main texts used for speech service
main_texts = {
	"intro":"I am your Voice Assistant to control this robot.",
	#name will be assigned when the node is launched
	"name": None,
	"temp":"The current temperature sensed by the robot is {} celcius.",
	"humid":"The current humidity sensed by the robot is {} percent.",
	"select":"Please select on of the followings",
	"speed":"The speed of robot is {} mps",
	"position":"The position of the robot in 2 dimentional system is {} to {}",
	"return":"Robot is returning to the base"
}


#Initializing Global variables with default and rosparam server values
CUR_TEMP = 0
CUR_HUMID = 0
CUR_ROB_POS = [rospy.get_param('initial_post_x'), rospy.get_param('initial_post_y')]
ROB_BASE_POINT = [rospy.get_param('initial_post_x'), rospy.get_param('initial_post_y')]
CUR_ROB_SPEED = 0
ROB_MAX_SPEED = rospy.get_param("robot_max_speed")

#getting the grid border values, this can be changed in robot_x.launch file
MAX_CORDINATE = rospy.get_param("max_cordinate")
MIN_CORDINATE = rospy.get_param("min_cordinate")
# ------------------------------------------------------


#funtion to set up the environment and initialize values 
def setup():
	#accessing global variables
	global main_texts
	global menu

	#clear the terminal screen
	clear()
	#asking the user's name
	user_name = input("Please enter your name: ")
	#updating the global variable
	main_texts['name'] = user_name
	
	#putting complete main_texts dict values in rosparam server so that other node can use them for speech service
	for key in main_texts.keys():
		rospy.set_param(key, main_texts[key])

	#clear the terminal screen
	clear()
	
	#greeting with user
	speech_to_text("Hello " + main_texts['name'])
	#introducing the interface
	speech_to_text(main_texts['intro'])


#main function of Control Node
def main():
	#accessing global variable
	global main_texts

	#main loop of the Control Node
	while True:
		#printing menu variable to the teraminal to communicate with user
		print(menu)
		#using speech service
		speech_to_text(main_texts['select'])
		
		#safety loop to make sure that user input is valid 
		check = True
		while check:
			try:
				#getting the user input
				choice = int(input("Select: "))
				#cheking the input in this list, otherwise AssertionError raised
				assert choice in [1, 2, 3, 4, 5]
				check = False
			except:
				#using speech service
				speech_to_text("Please choose a valid number")
		#clear the terminal screen
		clear()

		#if choice is 1, inform the user about the temperature
		if choice == 1:
			print("Current Temperature: {} C".format(CUR_TEMP))
			speech_to_text(main_texts['temp'].format(CUR_TEMP))

		#if choice is 2, inform the user about the humidity
		elif choice == 2:
			print("Current Humidity: {} %".format(CUR_HUMID))
			speech_to_text(main_texts['humid'].format(CUR_HUMID))
		
		#if choice is 2, inform the user about the current position of robot
		elif choice == 3:
			print("Current Position: X = {}  Y = {}".format(CUR_ROB_POS[0], CUR_ROB_POS[1]))
			speech_to_text(main_texts['position'].format(CUR_ROB_POS[0], CUR_ROB_POS[1]))

		#if choice is 4, use action client to go to the desired position
		elif choice == 4:
			#creating the goal message
			goal = PositionAndDestination()
			print("Cordinations of Destination")
			#using speech service
			speech_to_text("Enter the cordinations")
			
			#to get the desired values from user, use check_input function
			x = check_input(" X ---> ")
			y = check_input(" Y ---> ")
			
			#setting goal msg data
			goal.dest_x = x
			goal.dest_y = y
			goal.cur_x = CUR_ROB_POS[0]
			goal.cur_y = CUR_ROB_POS[1]

			#calling the walk_to_point to create an Action Client
			walk_to_point(goal)

		#if choice is 5, use action client to go to the base point of robot (specified in the launch file)
		else:
			#creating the goal message
			goal = PositionAndDestination()
			#setting goal msg data
			goal.dest_x = 0
			goal.dest_y = 0
			goal.cur_x = CUR_ROB_POS[0]
			goal.cur_y = CUR_ROB_POS[1]

			#calling the walk_to_point to create an Action Client
			walk_to_point(goal)
			

#Walk To Point Action client function
def walk_to_point(message):
	#accessing global variables
	global ROB_MAX_SPEED
	global CUR_ROB_POS

	#checking if the desired position and current position are the same
	if message.dest_x == message.cur_x and message.dest_y == message.cur_y:
		#if so, informing the user using speech service
		speech_to_text("Robot is in the desired position")
	#if no, creating a action client
	else:
		#creating action client; action name -> 'walk_to_point'
		#msg type -> WalkToPointAction
		walk_client = actionlib.SimpleActionClient('walk_to_point', WalkToPointAction)
		#waiting for Action server if it is busy with other task
		walk_client.wait_for_server()
		#creating a goal message
		destination = WalkToPointGoal(message)
		
		#Informing the user about distance and time to reach
		distance = round(math.dist([message.cur_x, message.cur_y], [message.dest_x, message.dest_y]), 1)
		time_to_reach = int(distance/ROB_MAX_SPEED)
		print("Distance: {} meters || Estimated Time: {} s".format(distance, time_to_reach))

		#sending the goal to the Action server, feedback callback function -> process_feedback
		#Feedback callback function creates a thread for itself
		walk_client.send_goal(destination, feedback_cb = process_feedback)
		#waiting for the Action server to finish the tast
		walk_client.wait_for_result()
		#to make sure, there is no audio congestion
		rospy.sleep(0.5)
		#informing the user using speech service
		speech_to_text("Robot reached to the destination")

		#clear the terminal screen
		clear()

		#printing the current coordination of robot and informing with speech service
		print("Current Cordinations: X = {}  Y = {}".format(CUR_ROB_POS[0], CUR_ROB_POS[1]))
		speech_to_text(main_texts['position'].format(CUR_ROB_POS[0], CUR_ROB_POS[1]))


#action client feedback callback funtion
def process_feedback(feedback):
	#accessing gloabl variable
	global CUR_ROB_SPEED
	#printing and informing user the disatnce to reach the destination
	print("Current Distance: {} m || Speed: {} m/s".format(round(feedback.distance, 1), CUR_ROB_SPEED))
	speech_to_text("{} meters".format(round(feedback.distance, 1)))


#speech service client function
def speech_to_text(text):
	#setting the loop cheking value False, if we receive False response from Service Server, we will ask again with same values
	#there might be Internet connection problems
	result = False
	while not result:
		#creating a ServiceProxy class object to use speech service
		#service name -> 'speech'; msg type -> SpeechToText
		proxy_client = rospy.ServiceProxy('speech', SpeechToText)
		#waiting for a response from the Service Server node
		response = proxy_client(text)
		#getting the result from responce
		result = response.result


#callback function for the subscriber of '/temp' topic
def temp_sub(val):
	#accessing the global variable
	global CUR_TEMP
	#updating the global variable value; round funtion is to round the float number
	CUR_TEMP = round(val.data, 1)


#callback function for the subscriber of '/humid' topic
def humid_sub(val):
	#accessing the global variable
	global CUR_HUMID
	#updating the global variable value; round funtion is to round the float number
	CUR_HUMID = round(val.data, 1)


#callback function for the subscriber of '/position' topic
def rob_pos_sub(val):
	#accessing the global variable
	global CUR_ROB_POS
	#updating the global variable value; round funtion is to round the float number
	CUR_ROB_POS[0] = round(val.pos_x)
	CUR_ROB_POS[1] = round(val.pos_y)


#callback function for the subscriber of '/speed' topic
def rob_speed_sub(val):
	#accessing the global variable
	global CUR_ROB_SPEED
	#updating the global variable value; round funtion is to round the float number
	CUR_ROB_SPEED = round(val.data, 1)


#cleaning the terminal window function
def clear():
	# for mac and linux (os.name is 'posix')
	if os.name == 'posix':
		_ = os.system('clear')
		print()
	else:
	# for windows platfrom
  		_ = os.system('cls')
  		print()


#checking function to validate user input values
def check_input(asked):
	#accessing the global variable
	#the max and min values of the grid plain in which robot operates
	global MAX_CORDINATE
	global MIN_CORDINATE
	#Flag variable to stop the loop if user input is valid
	flag = True

	while flag:
		try:
			#asking input from user
			number = float(input(asked))
			#checking the input value is between max and min grid values
			assert MIN_CORDINATE <= number and number <= MAX_CORDINATE, "Select from {} to {}"
			#if there is no AssertionError, stop the loop
			flag = False

		#if AssertionError, inform the user abot max and min grid values
		except AssertionError as msg:
			#using speech service
			speech_to_text(str(msg).format(MIN_CORDINATE, MAX_CORDINATE))
		
		#if any other exception, ask user to enter valid input
		except:
			#using speech service
			speech_to_text("Enter a valid number")

	#return the valid user input
	return number


if __name__ == "__main__":
	try:
		#initializing a new node
		rospy.init_node("control_node")

		#creating a subscriber for the topic "temp"
		#received data type -> Float32; callback func -> temp_sub
		rospy.Subscriber('temp', Float32, temp_sub)
		#creating a subscriber for the topic "humid"
		#received data type -> Float32; callback func -> humid_sub
		rospy.Subscriber('humid', Float32, humid_sub)
		#creating a subscriber for the topic "position"
		#received data type -> RobotPosition; callback func -> rob_pos_sub
		rospy.Subscriber('position', RobotPosition, rob_pos_sub)
		#creating a subscriber for the topic "speed"
		#received data type -> Float32; callback func -> rob_speed_sub
		rospy.Subscriber('speed', Float32, rob_speed_sub)

		#ROS Subscribers create threads for callback function
		#So, there is no need to create new threads 

		#launching setup function to set up the environment
		setup()
		#main function of Control Node, this is the main loop
		main()

		#to keep alive the node, continuous spinning
		rospy.spin()

	#only ROS related exceptions will be captured
	except rospy.ROSInterruptException as e:
		print(e)

