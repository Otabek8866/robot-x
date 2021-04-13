# robot_x
Robot_x is a Voice Interface Robot

                                   **Voice Interface Robot X in ROS**

                                    -------**(User Manual)**----------

                                           **INTRODUCTION**

Robot X is an imaginary robot with wheels and some sensors. The main operating system of this robot is created in ROS. It operates in a 2D bordered plain. Robot X has a voice interface that informs the user about its status and sensory data. Users can change the grid plain dimensions and the maximum speed of Robot X in robot_x.launch file. The code-specific comments are provided in all files.
Robot X functions are dependent on the Internet and speakers of the host machine. Please make sure that they are working properly.

![image](https://user-images.githubusercontent.com/55482580/114525488-0c1c0380-9c3e-11eb-896f-2de0658c1877.png)

                                 Fig 1. 2D grid plain in which the robot operates



                                      **  **The functions of Robot X:****

1.	The robot has a temperature sensor and when asked, it replies with the current temperature value.
2.	The robot has a humidity sensor and when asked, it replies with the current humidity value.
3.	It tells its current position in the predefined grid.
4.	It can walk to a point and come back when asked.


                                        **Created nodes for Robot X**:

1.	/control_node
2.	/humid_pub_node
3.	/position_pub_node
4.	/speech_gen_service_node
5.	/speed_pub_node
6.	/temp_pub_node
7.	/walk_to_point_action_server_node

                                        **Created topics for Robot X**

1.	/change_position
2.	/change_speed
3.	/humid
4.	/position
5.	/speed
6.	/temp
7.	/walk_to_point/cancel
8.	/walk_to_point/feedback
9.	/walk_to_point/goal
10.	/walk_to_point/result
11.	/walk_to_point/status

                                      **Created messages for Robot X:**

1.	robot_x/WalkToPointAction
2.	robot_x/WalkToPointActionFeedback
3.	robot_x/WalkToPointActionGoal
4.	robot_x/WalkToPointActionResult
5.	robot_x/WalkToPointFeedback
6.	robot_x/WalkToPointGoal
7.	robot_x/WalkToPointResult
8.	robot_x/PositionAndDestination
9.	robot_x/RobotPosition


                              **Created ROSparam server parameters for Robot X:**

1.	 /humid
2.	/initial_post_x
3.	/initial_post_y
4.	/intro
5.	/max_cordinate
6.	/min_cordinate
7.	/name
8.	/position
9.	/return
10.	/robot_max_speed
11.	/robot_name
12.	/rosdistro
13.	/select
14.	/speed
15.	/temp

Some parameters are initialized when the launch file is called. Additionally, other global ROS parameters are created by nodes when they are launched. Some parameters can be accessed during nodes running. ROS has one master to control all the global parameters, nodes, topics, actions and services. The master and nodes do not have to be in one host of computer, yet there has to be only one master in the network. All of the above is to show how flexible ROS is.


                                              **INSTALLATION**


1.	The package or Robot X should be put in the catkin workspace and source folder as shown: home/{user}/catkin_ws/src/
All the files in robot_x/scripts/ should be executable first with the following command: chmod +x home/{user}/catkin_ws/src/*.py
2.	Inside robot_x folder, install_dependecies.sh file is placed. First, we need to make this file executable by the following command: chmod +x *.sh To run this command, open a new terminal in robot_x folder and type: ./install_dependecies.sh
3.	This script file installs all the third-party libraries used to build this package. Caution: after installation is complete, it reboots the OS. If not needed, change install_dependecies.sh by deleting the last line.
4.	Source your working directory by typing: source devel/setup.bash
5.	To build the robot_x package dependencies, type this command: catkin_make
6.	Alternatively, the package can be downloaded from Github with the following command: git clone https://github.com/Otabek8866/robot_x
We still need make some files executable mentioned above in (1, 2).


                                                **RUNNING**


1.	Open a new terminal and go to the working directory: _cd catkin_ws/_
2.	Source the working directory: source devel/setup.bash
3.	Launch Robot X with the following launch file: roslaunch robot_x robot_x.launch
4.	To access the voice interface, open a new terminal window and go to the working directory: cd catkin_ws/   and source the environment: source devel/setup.bash
5.	Then run the following command: rosrun robot_x control_node.py

The user input interface is protected sufficiently so that users are only allowed to enter a valid input. 
Please make sure that you are connected to the Internet and the speakers of the current host machine are working properly. You should be able to hear voice information.


                                            **WORKFLOW DIAGRAM**
                                            

The following diagram describes the logical workflow of Robot X.

![image](https://user-images.githubusercontent.com/55482580/114526214-ada35500-9c3e-11eb-853c-994070499d0c.png)

                                Fig 2. Logical Workflow Diagram


**Robot X has 7 nodes (mentioned in the Introduction chapter).**

1.	Temperature Publisher Node (/temp_pub_node) and Humidity Publisher Node (humid_pub_node) are created using rospy.Publisher() class. Humidity and temperature are continuous stream data and they do not require high bandwidth across the network. To publish, these nodes use ROS topics that are designed for continuous stream data. Topics can have one-to-one, one-to-many, many-to-one and many-to-main connections which means a topic is a channel of continuous information. These nodes cover the 1st Mini Project (a simple Pub/Sub node). Their values are generated randomly.
2.	Speed Publisher Node (speed_pub_node) and Position Publisher Node (position_pub_node) are quite similar to the nodes described above yet have some complexity. These nodes are publishers and subscribers at the same time. These nodes store the speed and position data. If Robot X takes an action, these nodes subscribe to the changes published by Action Server Node. These nodes are created to add some complexity to Robot X.
3.	Speech Generating Service Node (speech_gen_service_node) is a ROS service server node that receives texts and converts them into speech. ROS services are designed for short-term tasks. Like action nodes, service nodes do a specific task yet in a short time. Action nodes are designed for long term tasks and they provide the current status of a task required by an action client. Walk To Point Action Server Node and Control Node both use Speech Generating Service Node. To prevent long-term waiting problems, ROS service nodes can be the best choice for speech generation. This node covers the 3rd Mini Project (ROS service nodes).
4.	Walk To Point Action Node (walk_to_point_action_server_node) is a ROS action server node. When the user asks Robot X to go to a specific place, this node does this task. ROS action nodes are used for long-term result-oriented tasks, which means an action server is occupied until it finishes the task given previously. Walk To Point Action node ensures the robot to go the desired position that might take much time based on distance. From this perspective, ROS action is used for this node. This node has also two publisher functions, which means this node uses three threads to publish and do a walking task. This node covers the 4th Mini Project (ROS action nodes).
5.	Control Node (control_node) is the most complex node in the package. This node works as a Subscriber, Service Client, Action Client, Voice Interface and the Main Logic of Robot X. It subscribes to 4 topics and there is no need to create threads for each subscription since rospy.Subscriber() class creates a new thread for each callback function. Control Node acts as a ROS service client by using the speech generating service. In the same way, it acts as a ROS action client for Walk To Point Action Node. 
P.S: The voice recognition system was tested, but due to some technical problems in Ubuntu, it was not deployed. Yet, it can work on Windows OS.
6.	This package has robot_x.launch file. When this file is called by roslaunch command, it initializes the first global rosparam server parameters. It initializes all the nodes except for the control node. The control node should be launched separately to get the terminal interface. This part covers the 2nd Mini Project (Launch files).
7.	Some custom message types are created (PositionAndDestination and RobotPosition) to show the understanding of message types. These message types are used by Action and Publisher nodes. These message types can be easily replaced by other existing ones (like geometry_msg/Twist). The idea was to show the understanding of Robot Operating System (ROS).
