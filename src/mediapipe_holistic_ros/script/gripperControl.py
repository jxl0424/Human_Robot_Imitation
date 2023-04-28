#!/usr/bin/env python3

import rospy
import RG_gripper
from std_msgs.msg import *

RG6 = RG_gripper.RG_Message_Generator("RG6")
new_finger_count = 0

def callback(msg):
    fingerCount = msg.data
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
    ur_script = rospy.Publisher('/ur_hardware_interface/script_command', String, queue_size=1)
    ur_script.publish(RG6.message(dist,force))
    
def gripperController(fingerCount):
    print('Received finger count!')
    rospy.Subscriber('fingercount',Int32, callback,queue_size=1)
    new_finger_count = fingerCount
    if new_finger_count != fingerCount:
        rospy.Subscriber('fingercount',Int32, callback,queue_size=1)
    else:
        pass

rospy.init_node("gripper_control", anonymous=True)
rate = rospy.Rate(10)

if __name__ == '__main__':   
    while not rospy.is_shutdown():       
        print(new_finger_count)
        gripperController(new_finger_count)
        rate.sleep()
