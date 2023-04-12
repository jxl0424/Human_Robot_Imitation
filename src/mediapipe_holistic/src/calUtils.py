#!/usr/bin/python3

import numpy as np
import mediapipe as mp
from mediapipe_holistic_ros.msg import  MediaPipeHolistic
from mediapipe_holistic_ros.msg  import  MediaPipePose
import time 
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt 


def calculate_angle_hand(results,joint_list):
        hand_landmark = results
        angle_degree = []
        for joint in joint_list:
                a = np.array([hand_landmark[joint[0]].x, hand_landmark[joint[0]].y]) # First coord
                b = np.array([hand_landmark[joint[1]].x, hand_landmark[joint[1]].y]) # Second coord
                c = np.array([hand_landmark[joint[2]].x, hand_landmark[joint[2]].y]) # Third coord
                radians = np.arctan2(c[1] - b[1], c[0]-b[0]) - np.arctan2(a[1]-b[1], a[0]-b[0])
                angle = np.abs(radians*180.0/np.pi)
                if angle > 180:
                    angle = 360-angle                    
                angle_degree.append(angle)
                
        return angle_degree 
    
def calculate_angle_pose(a,b,c):
    a = np.array(a) # First
    b = np.array(b) # Mid
    c = np.array(c) # End    
    radians = np.arctan2(c[1]-b[1], c[0]-b[0]) - np.arctan2(a[1]-b[1], a[0]-b[0])
    return radians 

def data_plot_hand(data_angles_hand):
        plt.plot(data_angles_hand)
        plt.ylabel('hand_angles')
        plt.savefig('/Users/brend/Downloads/Notebooks-20221025/hand_angles.png')
        plt.clf()
        
def data_plot_pose(data_angles_pose):
    plt.plot(data_angles_pose)
    plt.ylabel('pose_angles')
    plt.savefig('/Users/brend/Downloads/Notebooks-20221025/pose_angles.png')
    plt.clf()

def countFingers(image, results):
    mp_hands = mp.solutions.hands     
    # Initialize a dictionary to store the count of fingers of both hands.
    count = 0
    
    # Store the indexes of the tips landmarks of each finger of a hand in a list.
    fingers_tips_ids = [mp_hands.HandLandmark.INDEX_FINGER_TIP, mp_hands.HandLandmark.MIDDLE_FINGER_TIP,
                        mp_hands.HandLandmark.RING_FINGER_TIP, mp_hands.HandLandmark.PINKY_TIP]
    
    # Initialize a dictionary to store the status (i.e., True for open and False for close) of each finger of both hands.
    fingers_statuses = {'LEFT_THUMB': False, 'LEFT_INDEX': False, 'LEFT_MIDDLE': False,'LEFT_RING': False, 'LEFT_PINKY': False}
    
    # Retrieve the landmarks of the found hand.
    hand_landmarks =  results.left_hand_landmarks
        
    # Iterate over the indexes of the tips landmarks of each finger of the hand.
    for tip_index in fingers_tips_ids:          
        # Retrieve the label (i.e., index, middle, etc.) of the finger on which we are iterating upon.
        finger_name = tip_index.name.split("_")[0]
            
        # Check if the finger is up by comparing the y-coordinates of the tip and pip landmarks.
        if (hand_landmarks.landmark[tip_index].y < hand_landmarks.landmark[tip_index - 2].y):
                
            # Update the status of the finger in the dictionary to true.
            fingers_statuses["LEFT_"+finger_name] = True
                
            # Increment the count of the fingers up of the hand by 1.
            count += 1
        
    # Retrieve the y-coordinates of the tip and mcp landmarks of the thumb of the hand.
    thumb_tip_x = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].x
    thumb_mcp_x = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP - 2].x
        
    # Check if the thumb is up by comparing the hand label and the x-coordinates of the retrieved landmarks.
    if (thumb_tip_x < thumb_mcp_x):
        
        # Update the status of the thumb in the dictionary to true.
        fingers_statuses["LEFT_THUMB"] = True
            
        # Increment the count of the fingers up of the hand by 1.
        count += 1

    # Return the status of each finger and the count of the fingers 
    return fingers_statuses, count
