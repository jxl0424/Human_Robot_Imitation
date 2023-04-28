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
import RG_gripper

ur_con = UR_Controller()
RG6 = RG_gripper.RG_Message_Generator("RG6")
new_finger_count = 0
done_moving = False

def callback2(data):
    fingerCount = data.data
    global new_finger_count
    if fingerCount == 5:
        pub_command(110,25)
        new_finger_count = fingerCount
    if fingerCount == 0:
        pub_command(20,25)
        new_finger_count = fingerCount
    else:
        pass
  
def pub_command(dist,force):
    ur_script = rospy.Publisher('/ur_hardware_interface/script_command', String, queue_size=2)
    ur_script.publish(RG6.message(dist,force))
    
def gripperController(fingerCount):
    print('Received finger count!')
    rospy.Subscriber('fingercount',Int32, callback2,queue_size=1)
    new_finger_count = fingerCount
    if new_finger_count != fingerCount:
        rospy.Subscriber('fingercount',Int32, callback2,queue_size=1)
    else:
        pass

def callback(msg):
    global done_moving
    ur_script = rospy.Publisher('/ur_hardware_interface/script_command', String, queue_size=2)
    pose_angles = msg.data
    waypoint = [1.571, -pose_angles[0], pose_angles[1] , -1.571, -1.571, 0]
    command = ur_con.generate_move_j(waypoint)
    ur_script.publish(command)
    done_moving = True

def listner():
    
    print ("Received pose angles, starting to move!")
    rospy.Subscriber('pose_angles_throttle',Float32MultiArray, callback ,queue_size=2)
    #rate.sleep()
    
    print('done moving!')

#---------------Initialise-------------------------------------------------------------------:
print("Please Wait While System Starts Up...")
rospy.init_node("move_robot", anonymous=False)
print("System Started")

if __name__ == '__main__':   
    while not rospy.is_shutdown():
        listner()  
        gripperController(new_finger_count)
        rospy.spin()
        #rate.sleep()
        



