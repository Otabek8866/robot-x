#!/usr/bin/env python3

#main module to communicate with ROS
import rospy

#from robot_x package, import speech service msg types
from robot_x.srv import SpeechToText, SpeechToTextResponse

#importing os module
import os
# importing third-party libraries
import playsound		#for playing the audio file
from gtts import gTTS   #for converting texts to speech

#getting the current working directory path, a global variable
path = os.getcwd()


#main function for service requests
def speak(req):
	#accessing the global variable
	global path
	#variable to check the service request processed successfully
	answer = True
	try:
		#getting text for req msg (SpeechToText type)
		text = req.text_to_speech
		#generating speech for text; make speech fast
		tts = gTTS(text=text, slow=False)
		#generating file name and location variable
		filename = path + "audio.mp3"
		#saving audio file in the working directory (downloading audio from Internet)
		tts.save(filename)
		#playing audio file
		playsound.playsound(filename)
	
	#catching any kind of exceptions	
	except Exception as e:
		#converting exception info into str and printing in ROS loginfo 
		rospy.loginfo(str(e))
		#request not processed, try again
		answer = False
		#printing info for user
		print("Error while playing sound")

	#if no exception, return answer variable by converting SpeechToTextResponse msg (custom msg)
	return SpeechToTextResponse(answer)


if __name__ == "__main__":
	try:
		#initializing a new node
		rospy.init_node("speech_gen_service_node")
		#starting a ROS service; service name -> 'speech'
		#received msg type -> SpeechToText; callback func -> speak
		rospy.Service("speech", SpeechToText, speak)
		#to keep alive the node, continuous spinning
		rospy.spin()

	#only ROS related exceptions will be captured
	except rospy.ROSInterruptException as e:
		#converting exception info into str and printing in ROS loginfo 
		rospy.loginfo(str(e))
