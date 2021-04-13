#!/usr/bin/env python3

#main module to communicate with ROS
import rospy

#from standard messages, "Float32" msg type is imported
from std_msgs.msg import Float32

#random module to generate random values
import random


#publisher function
def pub_temp_val():

	#initializing a new node
	rospy.init_node('temp_pub_node')

	#creating a publisher class object to publish temperature values
	#'temp' -> topic name; Float32 -> value type; 'queue_size' -> outgoing message queue used for asynchronous publishing
	temp_pub = rospy.Publisher('temp', Float32, queue_size=10)

	#creating a Rate class object, at 5Hz, 5 times per second
	rate = rospy.Rate(5)

	#checking the ros master is alive
	while not rospy.is_shutdown():
		#generating a random value between 20 and 25
		val = random.random()*5 + 20
		#publishing the value
		temp_pub.publish(val)
		#sleeping 0.2 seconds since Rate is 5 Hz
		rate.sleep()


if __name__ == "__main__":
	try:
		#starting function
		pub_temp_val()

	#only ROS related exceptions will be captured
	except rospy.ROSInterruptException as e:
		print(e)