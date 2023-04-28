#!/usr/bin/env python3

#---------------Import Libraries and msgs---------------:

import rospy
import time
import math
import numpy as np
from std_msgs.msg import *
from geometry_msgs.msg import *
import tf2_msgs.msg
from tf.transformations import euler_from_quaternion
from control_lib import UR_Controller
from RG_gripper import RG_Message_Generator
import RG_gripper


#---------------Initialise---------------:

print("Please Wait While System Starts Up...")
rospy.init_node("move_j_example", anonymous=False)
play = rospy.Publisher('/trajectory_generator/play', String, queue_size=10)
ur_script = rospy.Publisher('/ur_hardware_interface/script_command', String, queue_size=10)
ur_con = UR_Controller()
time.sleep(2)
print("System Started")

#ROBOT_IP = "192.168.0.50"
#---------------Define Variables---------------:

#Move J's are faster, since precision isn't the goal
max_acc = 1.0
max_vel = 1.0
header = "def myProg():"
footer = "\nend"

# robot_startposition = (math.radians(-118),
#                     math.radians(-63),
#                     math.radians(-93),
#                     math.radians(-20),
#                     math.radians(88),
#                     math.radians(0))

#Create points for Move J's by converting joint angles into rads
#waypoint1 = [-0.29635690698863715, -1.7847736930894016, -1.4852751934471744, -1.4051645807806348, 1.577428577952475, -1.8694221618111264]

#waypoint2 = [0.13351768777756623, -1.4756758825612055, -1.7125170620568362, -1.4864969239235706, 1.5482815794441696, -1.4395475670449231]
#
#waypoint3 = [0.72640603468004, -1.4236650708517744, -1.560847950058529, -1.7058848108992577, 1.5320500174006224, -0.8459610884416515]
#
#waypoint3 = [0.31122102291068127, -1.5520107329106074, 1.2436094632753534, 1.3910114591616782,  -1.9203444369822998,  -1.003157461437168432]
#----------------------------First Grip-------------------------------------------------------#
waypoint3 = [1.5707963268, -1.1571532941, -2.2781782726, -1.2768828808, 1.5707963268, 0]

waypoint4 = [1.5714944585,  -1.6060519777, -2.2294835865, -0.8953539063, 1.5734143207, 0.0340339204] # gripping first cube 

waypoint5 = [1.5707963268, -1.3798573066, -2.0704840916, -1.2774064795, 1.5707963268, 0]

waypoint6 = [0, -1.3798573066, -2.0704840916, -1.2774064795, 1.5707963268, 0]

waypoint7 = [0, -1.5851080267, -2.2324506462, -0.9297368925, 1.5699236622, 0]

waypoint8 = [0, -1.3798573066, -2.0704840916, -1.2774064795, 1.5707963268, 0]
#----------------------------------------------------------------------------------------------#
home_waypoint = [0, -1.1571532941, -2.2781782726, -1.2768828808, 1.5707963268, 0]
#--------------------------------Second grip---------------------------------------------------#
waypoint9 =[1.3962634016, -1.1571532941, -2.2781782726, -1.2768828808, 1.5707963268, 0]

waypoint10 =[1.1873474901, -1.6992525597, -2.1362830044, -0.8941321758, 1.5711453926, 0]

waypoint11 =[1.3962634016, -1.3798573066, -2.0704840916, -1.2774064795, 1.5707963268, 0]

waypoint12 =[0, -1.3798573066, -2.0704840916, -1.2774064795, 1.5707963268, 0]

waypoint13 =[0, -1.5149457907, -2.1708405236, -1.0615092511, 1.5707963268, 0]

waypoint14 =[0, -1.3798573066, -2.0704840916, -1.2774064795, 1.5707963268, 0]
#-----------------------------------------------------------------------------------------------#

#-------------------------------Third grip----------------------- -------------------------------#
waypoint15 =[1.308996939, -1.1571532941, -2.2781782726, -1.2768828808, 1.5707963268, 0]
waypoint16 =[0.9208357134, -1.7350318094, -2.1020745511, -0.8923868465, 1.5716689914, 0]
waypoint17 =[1.308996939, -1.3798573066, -2.0704840916, -1.2774064795, 1.5707963268, 0]
waypoint18 =[0, -1.3798573066, -2.0704840916, -1.2774064795, 1.5707963268, 0]
waypoint19 =[0,-1.5008086238, -2.0753710135, -1.2049753156, 1.5707963268, 0]
waypoint20 =[0, -1.3798573066, -2.0704840916, -1.2774064795, 1.5707963268, 0]



#home_waypoint = [-1.58825, -1.71042, -2.19911, -0.802851, 1.58825, -0.03106686]
#home_waypoint = [-0.15184364492350666, -1.7163567864112237, -1.6287412579611082, -1.3281955607676847, 1.5732397877476887, -1.7247343668207964]

robot_path = [home_waypoint]
#Add them to ordered list to move to them one by one
#waypoints = [waypoint1, waypoint2, waypoint3, home_waypoint]
waypoints = [waypoint3, waypoint4, waypoint5, waypoint6, waypoint7, home_waypoint]
time.sleep(5)

def home_robot():
	header = "def myProg():"
	footer = "\nend"
	home_waypoint = [-1.58825, -1.71042, -2.19911, -0.802851, 1.58825, -0.03106686]
	move_msg ="\nmovej({},a={},v={},t={},r={})".format(home_waypoint, max_acc ,max_vel,0,0)
	command = header + move_msg + footer
	return command

def move_j_pose(pose):
	header = "def myProg():"
	footer = "\nend"
	x = pose.position.x
	y = pose.position.y
	z = pose.position.z
	rx, ry, rz = UR_Controller().convert_to_euler(pose.orientation)
	pose_str = '['+str(x)+ ',' +str(y)+ ',' +str(z)+ ',' +str(rx)+ ',' +str(ry)+ ',' +str(rz) +']'
	command = header + '\n\tmovej(p{}, a={}, v={}, t=0, r=0)'.format(pose_str, max_acc, max_vel) + footer
	return command
"""
ros@ubuntu:~$ 
def program():
	set_standard_digital_out(3, True)
	global var_1 = (encoder_get_tick_count(0))
	halt
	end
"""

#---------------Main Code---------------:


## Move using local function and waypoint
command = home_robot()
ur_script.publish(command)

## Move using local function and pose
my_pos = ur_con.get_pose()
#my_pos.position.x += 0.31122102291068127
command = move_j_pose(my_pos) 
ur_script.publish(command)
print(my_pos)
time.sleep(3)





# my_pos = ur_con.get_pose()
# my_pos.position.y -= -0.5520107329106074
# command = move_j_pose(my_pos)
# ur_script.publish(command)
# print(my_pos)
# #time.sleep(1)



# my_pos = ur_con.get_pose()
# my_pos.orientation.x += 0.3910114591616782
# command = move_j_pose(my_pos)
# ur_script.publish(command)
# print(my_pos)
# #time.sleep(5)

# my_pos = ur_con.get_pose()
# my_pos.orientation.y -= -0.9203444369822998
# command = move_j_pose(my_pos)
# ur_script.publish(command)
# print(my_pos)
# #time.sleep(1)

# my_pos = ur_con.get_pose()
# my_pos.orientation.z -= -0.003157461437168432
# command = move_j_pose(my_pos)
# ur_script.publish(command)
# print(my_pos)
# #time.sleep(1)
#encoder_enable_pulse_decode(0, 1, 8, 9)
#conveyor_pulse_decode(1,0,1)

# ## Move using lib and waypoint
#command = ur_con.generate_move_j(home_waypoint)
#ur_script.publish(command)
# print(command)

# ## Move using lib and waypoint sequence
###command = ur_con.generate_move_j(waypoints, sequence=True)
#ur_script.publish(command)
#time.sleep(3)

command = ur_con.generate_move_j(waypoint3)

ur_script.publish(command)
time.sleep(3)

RG6 = RG_gripper.RG_Message_Generator("RG6")
ur_script.publish(RG6.message(110, 25)) #generate and send to robot
print("gripper open for the correct size") # I open the gripper in specific size because the real robot it change automatically the gripping 
time.sleep(3)

command = ur_con.generate_move_j(waypoint4)

#command = ur_con.generate_move_j(home_waypoint)# home

ur_script.publish(command)
time.sleep(5)  # Time between the waypoints necessary to make the gripper to reach on  grab

RG6 = RG_gripper.RG_Message_Generator("RG6")
ur_script.publish(RG6.message(110, 25)) #generate and send to robot
print("gripper open")
time.sleep(3)
print("gripper close")

command = ur_con.generate_move_j(waypoint5)

ur_script.publish(command)
time.sleep(5)


command = ur_con.generate_move_j(waypoint6)

ur_script.publish(command)
time.sleep(5)


command = ur_con.generate_move_j(waypoint7)

ur_script.publish(command)
time.sleep(5)

command = ur_con.generate_move_j(waypoint8)

ur_script.publish(command)
time.sleep(5)
#--------------------------#
command = ur_con.generate_move_j(home_waypoint)

ur_script.publish(command)
time.sleep(5)
#-------------------------#
command = ur_con.generate_move_j(waypoint9)

ur_script.publish(command)
time.sleep(5)

command = ur_con.generate_move_j(waypoint10)

ur_script.publish(command)
time.sleep(5)

command = ur_con.generate_move_j(waypoint11)

ur_script.publish(command)
time.sleep(5)

command = ur_con.generate_move_j(waypoint12)

ur_script.publish(command)
time.sleep(5)

command = ur_con.generate_move_j(waypoint13)

ur_script.publish(command)
time.sleep(5)

command = ur_con.generate_move_j(waypoint14)

ur_script.publish(command)
time.sleep(5)

#--------------------------#
command = ur_con.generate_move_j(home_waypoint)

ur_script.publish(command)
time.sleep(5)
#-------------------------#

command = ur_con.generate_move_j(waypoint15)

ur_script.publish(command)
time.sleep(5)

command = ur_con.generate_move_j(waypoint16)

ur_script.publish(command)
time.sleep(5)

command = ur_con.generate_move_j(waypoint17)

ur_script.publish(command)
time.sleep(5)

command = ur_con.generate_move_j(waypoint18)

ur_script.publish(command)
time.sleep(5)

command = ur_con.generate_move_j(waypoint19)

ur_script.publish(command)
time.sleep(5)

command = ur_con.generate_move_j(waypoint20)

ur_script.publish(command)
time.sleep(5)


## Move using lib and pose
#my_pos = ur_con.get_pose()
#my_pos.position.x += 0.2
#command = ur_con.generate_move_j(my_pos, pose_msg=True)
#ur_script.publish(command)


#command = ur_con.generate_move_j(home_waypoint)
#ur_script.publish(command)

#play_move_j_pose(home_waypoint)
#time.sleep(5)



