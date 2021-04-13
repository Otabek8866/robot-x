#!/usr/bin/env python3

#main module to communicate with ROS
import rospy

#from standard messages, "Float32" msg type is imported
from std_msgs.msg import Float32

# Setting the initial speed val to 0 (global variable)
SPEED = 0


#callback function for the subscriber of '/change_speed' topic
def update_speed(val):
	#accessing the global variable
	global SPEED
	#updating the global variable value; round funtion is to round the float number
	SPEED = round(val.data, 1)


#publisher function
def pub_speed_val():
	#accessing the global variable
	global SPEED

	#creating a publisher class object to publish speed values
	#'speed' -> topic name; Float32 -> msg type; 'queue_size' -> outgoing message queue used for asynchronous publishing
	speed_pub = rospy.Publisher('speed', Float32, queue_size=10)

	#creating a Rate class object, at 10Hz, 10 times per second
	rate = rospy.Rate(10)

	#checking the ros master is alive
	while not rospy.is_shutdown():
		#publishing the global variable; msg type -> Float32
		speed_pub.publish(SPEED)
		#sleeping 0.1 seconds since Rate is 5 Hz
		rate.sleep()


if __name__ == "__main__":
	try:
		#initializing a new node
		rospy.init_node('speed_pub_node')

		#creating a subscriber for the topic "change_speed"
		#received data type -> Float32; callback func -> update_speed
		rospy.Subscriber("change_speed", Float32, update_speed)

		#main function for publisher
		pub_speed_val()
		
		#to keep alive the node, continuous spinning
		rospy.spin()

	#only ROS related exceptions will be captured
	except rospy.ROSInterruptException as e:
		print(e)