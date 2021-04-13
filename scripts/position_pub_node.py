#!/usr/bin/env python3

#main module to communicate with ROS
import rospy

#import the customized message type for Robot X (has two values: pos_x, pos_y (Float32))
from robot_x.msg import RobotPosition

#Create a global variable with the custom message
POSITION = RobotPosition()
#Getting initial values from rosparam server and assigning (Initial position)
POSITION.pos_x = rospy.get_param('initial_post_x')
POSITION.pos_y = rospy.get_param('initial_post_y')


#callback function for the subscriber of '/change_position' topic
def update_position(data):
	#accessing the global variable
	global POSITION
	#updating the global variable value
	POSITION = data


#publisher function
def pub_position_val():
	#accessing the global variable
	global POSITION

	#creating a publisher class object to publish position values
	#'position' -> topic name; RobotPosition -> msg type; 'queue_size' -> outgoing message queue used for asynchronous publishing
	position_pub = rospy.Publisher('position', RobotPosition, queue_size=10)

	#creating a Rate class object, at 10Hz, 10 times per second
	rate = rospy.Rate(10)

	#checking the ros master is alive
	while not rospy.is_shutdown():
		#publishing the global variable; msg type -> RobotPosition
		position_pub.publish(POSITION)
		#sleeping 0.1 seconds since Rate is 5 Hz
		rate.sleep()


if __name__ == "__main__":
	try:
		#initializing a new node
		rospy.init_node('position_pub_node')

		#creating a subscriber for the topic "change_position"
		#received data type -> RobotPosition; callback func -> update_position
		rospy.Subscriber("change_position", RobotPosition, update_position)

		#main function for publisher
		pub_position_val()
		
		#to keep alive the node, continuous spinning
		rospy.spin()

	#only ROS related exceptions will be captured
	except rospy.ROSInterruptException as e:
		print(e)