#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import random
import os
import pandas as pd
import csv
import time
import numpy as np
from threading import Thread
from scipy import signal


class RobotCommands():

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        group_name = "manipulator"
        self.group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
        
        
        plan_joints = [0] * 6

        plan_joints = self.group.get_current_joint_values()

        self.return_to_home()

        plan_joints = self.move_to_initial_position()

        plan_joints = self.move_down(plan_joints)
        self.plan_goal(plan_joints)


        while(True):
            self.handle_buffer(plan_joints)


    def perform_movement(self, rotations, plan_joints):

        
        alpha_rotation = rotations[0]
        beta_rotation = rotations[1]
        theta_rotation = rotations[2]

        # Alpha
        # Controls Rotation
        plan_joints = self.rotate_left(alpha_rotation / 4, plan_joints)
        plan_joints = self.circle_left(alpha_rotation / 4, plan_joints)


        # Beta
        # Used to control left and right
        plan_joints = self.move_left(beta_rotation, plan_joints)


        # Theta
        # Controls up & downs
        plan_joints = self.move_forward(theta_rotation, plan_joints)

        self.plan_goal(plan_joints)



        


    
    def handle_buffer(self, plan_joints) -> None:

        # Constants
        FILE_NAME: str = '/CSVs/toby.csv'
        WAIT_TIME: float = 0.1

        # Opening the file in read mode
        path: str = os.path.normpath(os.getcwd() + os.sep + os.pardir) + FILE_NAME
        file = open(path, 'r')

        # Create's a CSV Reader 
        csv_reader: csv._reader
        csv_reader = csv.reader(file)

        # Attempt to skip the header row
        try:
            next(csv_reader)
        except StopIteration:
            self.after(50, self.handle_buffer)

        buffer: list[list[float]]
        buffer = []

        while(True):
            
            try:
                while(len(buffer) < 256):

                    current_row = next(csv_reader)

                    # Convert the elements to float
                    current_row: list[float] = [float(value) for value in current_row[1:]]
                    buffer.append(current_row)

                # calculate psd from buffer
                mapped_rotations: list = self.calculatePSD(buffer)

                # make robot draw with these rotations
                self.perform_movement(mapped_rotations, plan_joints)

                # Reset the buffer
                buffer = []

                # Sleep
                time.sleep(WAIT_TIME)
                
            # If there is no next row, sleep or perform other desired actions
            except StopIteration:
                time.sleep(WAIT_TIME)

            except KeyboardInterrupt:
                print('Closing!')
        
        return None


    def calculatePSD(self, buffer):

        # Convert buffer to a numpy array
        buffer_array = np.array(buffer)

        # Transpose the array to have channels in columns
        buffer_array = np.transpose(buffer_array)

        # Calculate PSD for each channel
        freqs, psd = signal.welch(buffer_array, fs=256)

        # Calculate alpha, beta, and theta bands
        alpha_band = np.mean(psd[:, (freqs >= 8) & (freqs <= 12)], axis=1)
        beta_band = np.mean(psd[:, (freqs >= 12) & (freqs <= 30)], axis=1)
        theta_band = np.mean(psd[:, (freqs >= 4) & (freqs <= 8)], axis=1)

        alpha = np.mean(alpha_band)
        beta = np.mean(beta_band)
        theta = np.mean(theta_band)

        # Map alpha, beta, and theta values to rotation range
        max_alpha = np.max(alpha_band)
        mapped_alpha = (alpha / max_alpha) * (pi/2) - (pi/4)

        max_beta = np.max(beta_band)
        mapped_beta = (beta / max_beta) * (pi/2) - (pi/4)

        max_theta = np.max(theta_band)
        mapped_theta = (theta / max_theta) * (pi/2) - (pi/4)

        # Create an array of mapped rotations
        mapped_rotations = [mapped_alpha, mapped_beta, mapped_theta]
        print(mapped_rotations)

        return mapped_rotations




        

    def move_to_initial_position(self):    
        print("initial")  

        joint_goal = self.group.get_current_joint_values()    

        joint_goal[0] = pi/2
        joint_goal[1] = 0
        joint_goal[3] = 0
        joint_goal[4] = pi
        joint_goal[5] = pi/4

        self.plan_goal(joint_goal)

        return joint_goal
    

        
    def move_down(self, plan_joints):

        plan_joints[1] += pi/4
        plan_joints[2] -= pi/4

        return plan_joints

    
        

    def move_left(self, amount, plan_joints):

        plan_joints[0] += amount
        plan_joints[4] -= amount

        return plan_joints



    def move_forward(self, amount, plan_joints):

        plan_joints[4] -= amount
        plan_joints[0] -= amount / 8

        return plan_joints



    def rotate_left(self, amount, plan_joints):

        plan_joints[0] += amount
        return plan_joints


    def circle_left(self, amount, plan_joints):

        plan_joints[4] += amount
        return plan_joints



    def return_to_home(self):
        print("return to home")
        joint_goal = self.group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -pi/2
        joint_goal[2] = 0
        joint_goal[3] = -pi/2
        joint_goal[4] = 0
        joint_goal[5] = 0

        self.plan_goal(joint_goal)



    def plan_goal(self, joint_goal):

        # check overflow
        for x in range(0, len(joint_goal)):

            if(joint_goal[x] < -(2 * pi)):
                joint_goal[x] += 2 * pi

            if(joint_goal[x] > (2 * pi)):
                joint_goal[x] -= 2 * pi

        # keep pen on page
        if(joint_goal[0] < pi/4):
            joint_goal[0] += pi/4

        if(joint_goal[0] > (3 * pi)/4):
            joint_goal[0] -= pi/4


        self.group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.group.stop()

robot = RobotCommands()