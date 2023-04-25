#!/usr/bin/env python3

import rospy
import RG_gripper
from std_msgs.msg import *

def callback(msg):
    ur_script = rospy.Publisher('/ur_hardware_interface/script_command', String, queue_size=10)
    fingerCount = msg.data
    if fingerCount == 5:
        print (RG6.message(30, 25)) #just generate the snippet or:
        ur_script.publish(RG6.message(30, 25)) #generate and send to robot
    elif fingerCount ==0:
        print (RG6.message(110, 25)) #just generate the snippet or:
        ur_script.publish(RG6.message(110, 25)) #generate and send to robot     
    else:
        pass  

def gripperController():
    print('Received finger count!')
    rospy.Subscriber('fingercount',Int32, callback ,queue_size=10,tcp_nodelay=True)
    print("Gripper done moving")

rospy.init_node("gripper_control", anonymous=True)

if __name__ == '__main__':    
    while not rospy.is_shutdown():
        RG6 = RG_gripper.RG_Message_Generator("RG6")
        gripperController()
