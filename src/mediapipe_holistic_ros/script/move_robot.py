#!/usr/bin/env python3

#---------------Import Libraries and msgs---------------:
from dataclasses import dataclass
from operator import truediv
from sre_constants import NOT_LITERAL_IGNORE
import rospy
import time
import numpy as np
from std_msgs.msg import *
from geometry_msgs.msg import *
from tf.transformations import euler_from_quaternion
from collections import Counter
from control_lib import UR_Controller
from sensor_msgs.msg import JointState

ur_con = UR_Controller()

def callback(msg):
    ur_script = rospy.Publisher('/ur_hardware_interface/script_command', String, queue_size=10)
    global pose_angles
    pose_angles = msg.data
    waypoint = [1.5708, -pose_angles[0], -pose_angles[1] , -1.5708, 1.5708, 0]
    command = ur_con.generate_move_j(waypoint)
    ur_script.publish(command)
    #rate.sleep()

def callback2(data):
    fingercount=data.data

def listner():
    print ("Received pose angles, starting to move!")
    rospy.Subscriber('pose_angles',Float32MultiArray, callback ,queue_size=10,tcp_nodelay=True)
    rate.sleep()
    print('done moving!')


def home_position():
    ur_script = rospy.Publisher('/ur_hardware_interface/script_command', String, queue_size=10)
    home_waypoint = [1.5708, 1.5708, 1.5708 , -1.5708, -1.5708, 0]                        #The robot's home waypoint is set as follows
    command = ur_con.generate_move_j(home_waypoint)                                                 #Generate a command to move the robot to the home position with the help of home waypoint.
    ur_script.publish(command)   
    print("Home position reached!")
 

#---------------Initialise-------------------------------------------------------------------:
print("Please Wait While System Starts Up...")
rospy.init_node("move_robot", anonymous=True)
rate = rospy.Rate(0.1)
print("System Started")
home_position()

if __name__ == '__main__':  
    
    while not rospy.is_shutdown():
    #print('Robot is starting to imitate')
        listner()    
        # rospy.spin()

        


