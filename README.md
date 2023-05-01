# Human_Robot_Imitation
Brendan's Final Year Project
Current Version is able to enable imitation on UR10 Robot arm

The program is run in ROS Noetic.

HOW TO RUN THE CODE:

1. git clone the package.

2. roslaunch robot_control_simulation ur_robot_drivers.launch (make sure you change the robot ip in the launch file)

#TO LAUNCH THE CAMERA STREAM WITH MEDIAPIPE#

In a new terminal run:

roslaunch mediapipe_holistic_ros mediapipe_holistic_ros.launch

#To throttle the publish rate of pose angles to 0.5Hz for the program to work#

In a new terminal run:

rosrun topic_tools throttle messages /pose_angles 0.5

#to make the robot start to imitate#

In a new terminal run:

rosrun mediapipe_holistic_ros move_robot.py

#To make the grippers move#

In a new terminal run:

rosrun mediapipe_holistic_ros gripperControl.py
