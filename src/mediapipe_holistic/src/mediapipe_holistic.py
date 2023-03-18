#!/usr/bin/python3

import rospy
import math
import cv2
import numpy as np
import mediapipe as mp
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from mediapipe_holistic_ros.msg import  MediaPipeHolistic
from mediapipe_holistic_ros.msg  import  MediaPipePose
import time 
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt 
import PIL

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_holistic = mp.solutions.holistic
mp_hands = mp.solutions.hands

#####
input_usb_cam_topic = rospy.get_param("/mediapipe_holistic_ros/input_usb_cam_topic","/usb_cam/image_raw")
output_image_topic = rospy.get_param("/mediapipe_holistic_ros/output_image_topic","/mediapipe_holistic/output_image")
output_mediapipe_topic = rospy.get_param("/mediapipe_holistic_ros/output_mediapipe_topic","/MediaPipePose/holistic/landmarks")
pub_landmark_output = rospy.get_param("/mediapipe_holistic_ros/pub_landmark_output",True)
pub_image_output = rospy.get_param("/mediapipe_holistic_ros/pub_image_output",True)
view_image = rospy.get_param("/mediapipe_holistic_ros/open_view_image",True)
#####

def apply_landmark(image, results):     
        if (pub_image_output == True or view_image == True):
                # Draw landmark annotation on the image.
                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                #mp_drawing.draw_landmarks(
                        #image,
                        #results.face_landmarks,
                        #mp_holistic.FACEMESH_CONTOURS,
                        #landmark_drawing_spec=None,
                        #connection_drawing_spec=mp_drawing_styles
                        #.get_default_face_mesh_contours_style())
                # mp_drawing.draw_landmarks(
                #         image,
                #         results.pose_landmarks,
                #         mp_holistic.POSE_CONNECTIONS,
                #         landmark_drawing_spec=mp_drawing_styles
                #         .get_default_pose_landmarks_style())
                #mp_drawing.draw_landmarks(
                        #image,
                        #results.right_hand_landmarks,
                        #mp_hands.HAND_CONNECTIONS,
                        #mp_drawing_styles.get_default_hand_landmarks_style(),
                        #mp_drawing_styles.get_default_hand_connections_style())       
                mp_drawing.draw_landmarks(
                        image,
                        results.left_hand_landmarks,
                        mp_hands.HAND_CONNECTIONS,
                        mp_drawing_styles.get_default_hand_landmarks_style(),
                        mp_drawing_styles.get_default_hand_connections_style())
                        
        if (pub_image_output == True):
                image_message = bridge.cv2_to_imgmsg(image, encoding="passthrough")
                publisher_output_image.publish(image_message) 
        
        if (view_image == True):
                cv2.imshow('MediaPipe Holistic', cv2.flip(image, 1))
                if cv2.waitKey(5) & 0xFF == 27:
                        print("OK")                          
        return image        
                             
def pub_results(results):
        if (pub_landmark_output == True):
                landmarks = MediaPipeHolistic() 

                #face_landmarks
                if results.face_landmarks:
                        p_id = 0
                        for i in results.face_landmarks.landmark:
                                pose = MediaPipePose()
                                pose.id = p_id
                                pose.x = i.x
                                pose.y = i.y
                                pose.z = i.z
                                p_id += 1
                                landmarks.face_landmarks.append(pose)
                #left_hand_landmarks
                if results.left_hand_landmarks:
                        p_id = 0
                        for i in results.left_hand_landmarks.landmark:
                                pose = MediaPipePose()
                                pose.id = p_id
                                pose.x = i.x
                                pose.y = i.y
                                pose.z = i.z
                                p_id += 1
                                landmarks.left_hand_landmarks.append(pose)  
                                
                #right_hand_landmarks
                if results.right_hand_landmarks:
                        p_id = 0
                        for i in results.right_hand_landmarks.landmark:
                                pose = MediaPipePose()
                                pose.id = p_id
                                pose.x = i.x
                                pose.y = i.y
                                pose.z = i.z
                                p_id += 1
                                landmarks.right_hand_landmarks.append(pose) 
                                
                                
                #pose_landmarks
                if results.pose_landmarks:
                        p_id = 0
                        for i in results.pose_landmarks.landmark:
                                pose = MediaPipePose()
                                pose.id = p_id
                                pose.x = i.x
                                pose.y = i.y
                                pose.z = i.z
                                pose.visibility = i.visibility
                                p_id += 1
                                landmarks.pose_landmarks.append(pose) 
                                
                #pose_world_landmarks
                if results.pose_world_landmarks:
                        p_id = 0
                        for i in results.pose_world_landmarks.landmark:
                                pose = MediaPipePose()
                                pose.id = p_id
                                pose.x = i.x
                                pose.y = i.y
                                pose.z = i.z
                                pose.visibility = i.visibility
                                p_id += 1
                                landmarks.pose_world_landmarks.append(pose)                 
                                                        
                publisher_output_mediapipe.publish(landmarks) 

def calculate_angle_pose(a,b,c):
    a = np.array(a) # First
    b = np.array(b) # Mid
    c = np.array(c) # End
    
    radians = np.arctan2(c[1]-b[1], c[0]-b[0]) - np.arctan2(a[1]-b[1], a[0]-b[0])
    angle = np.abs(radians*180.0/np.pi)
    
    if angle >180.0:
        angle = 360-angle
        
    return angle 
 
def calculate_angle_hand(results,joint_list):
        hand_landmark = results
        angles =[]
        for joint in joint_list:
                a = np.array([hand_landmark[joint[0]].x, hand_landmark[joint[0]].y]) # First coord
                b = np.array([hand_landmark[joint[1]].x, hand_landmark[joint[1]].y]) # Second coord
                c = np.array([hand_landmark[joint[2]].x, hand_landmark[joint[2]].y]) # Third coord
                radians = np.arctan2(c[1] - b[1], c[0]-b[0]) - np.arctan2(a[1]-b[1], a[0]-b[0])
                angle = np.abs(radians*180.0/np.pi)
            
                if angle > 180.0:                
                        angle = 360-angle
                        
                angles.append(angle)  
                      
        return angles 
                 
def data_plot(data_angles):
        plt.plot(data_angles)
        plt.ylabel('angles')
        plt.savefig('/home/ros/Human-Robot-Imitation/src/mediapipe_holistic_ros/mediapipe_holistic_ros/Angles/hand-angles.png')	

if __name__ == '__main__':
        rospy.init_node('MediaPiPeHolistic')        
      
        #Global Topics
        bridge = CvBridge()
        
        #Sub and Pub
        publisher_output_image = rospy.Publisher(output_image_topic, Image)
        publisher_output_mediapipe = rospy.Publisher(output_mediapipe_topic, MediaPipeHolistic)
        
        with mp_holistic.Holistic(
            enable_segmentation=True,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5) as holistic:
          while not rospy.is_shutdown():
          
                try:
                        data = rospy.wait_for_message(input_usb_cam_topic, Image, timeout=10)
                        image = bridge.imgmsg_to_cv2(data, desired_encoding='bgra8')
                except:
                        print("Image read error")
                        continue                
                
                #apply holistic
                image.flags.writeable = False
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                results = holistic.process(image)
                
                #apply landmark                                      
                image = apply_landmark(image, results)  
                pub_results(results)
                try:
                     joint_list = [[6,5,0],[12,9,0],[16,13,0],[18,17,0]]    
                     #joint_list = [[6,5,0]]          
                     hand_landmarks = results.left_hand_landmarks.landmark
                     thumb_tip = [hand_landmarks[8].y]
                     #index_tip = [hand_landmarks[mp_holistic.HandLandmark.INDEX_FINGER_TIP].y]
                     #middle_tip = [hand_landmarks[mp_holistic.hands.HandLandmark.INDEX_FINGER_TIP].y]
                     #ring_tip = [hand_landmarks[mp_holistic.hands.HandLandmark.INDEX_FINGER_TIP].y]
                     #pinky_tip = [hand_landmarks[mp.solutions.hands.HandLandmark.INDEX_FINGER_TIP].y]
                     
                     for i, joints in enumerate(joint_list):
                             hand_angles = calculate_angle_hand(hand_landmarks,joint_list)                     	
                             #print(hand_angles)
                             #data_plot(hand_landmark)                    	
                             data_plot(hand_angles)
                             #print(data_angle)
                             #plt.show()
                             #plt.plot(data_angle)
                             #plt.show()
                             #print("hand angles",i," are: ", hand_angles)

                     
                     #print("position of tip of thumb finger is:",thumb_tip)
                     #time.sleep(0.05)
                     #print("position of tip of index finger is:",index_tip)
                     #time.sleep(0.05)
                     #print("position of tip of middle finger is:",middle_tip)
                     #time.sleep(0.05)
                     #print("position of tip of ring finger is:",ring_tip)
                     #time.sleep(0.05)
                     #print("position of tip of pinky finger is:",pinky_tip)
                except:
                       print("Fingers Not Found")
                # user_choice=input("Arm angles or hand gestures?(A or B):")
                # if user_choice == "A" or "a":
                #         try:
                #                 pose_landmarks = results.pose_landmarks.landmark
                #                 shoulder = [pose_landmarks[mp.solutions.holistic.PoseLandmark.RIGHT_SHOULDER].x,landmarks[mp.solutions.holistic.PoseLandmark.RIGHT_SHOULDER].y]
                #                 elbow = [pose_landmarks[mp.solutions.holistic.PoseLandmark.RIGHT_ELBOW].x,landmarks[mp.solutions.holistic.PoseLandmark.RIGHT_ELBOW].y]
                #                 wrist = [pose_landmarks[mp.solutions.holistic.PoseLandmark.RIGHT_WRIST].x,landmarks[mp.solutions.holistic.PoseLandmark.RIGHT_WRIST].y]
                #                 hip = [pose_landmarks[mp.solutions.holistic.PoseLandmark.RIGHT_HIP].x,landmarks[mp.solutions.holistic.PoseLandmark.RIGHT_HIP].y] 
                #                 index = [pose_landmarks[mp.solutions.holistic.PoseLandmark.RIGHT_INDEX].x,landmarks[mp.solutions.holistic.PoseLandmark.RIGHT_INDEX].y] 
                #                 elbow_angle = calculate_angle(shoulder,elbow,wrist)    
                #                 shoulder_angle = calculate_angle(hip,shoulder,elbow)
                #                 wrist_angle = calculate_angle(elbow,wrist,index)            	
                #                 print("The angle of elbow is: " , elbow_angle)
                #                 time.sleep(0.05)
                #                 print("The angle of shoulder is: " , shoulder_angle)
                #                 time.sleep(0.05)
                #                 print("The angle of wrist is: " , wrist_angle)		        	
                #         except:
                #                 print("Angle Not Found")        

                        
                
                
