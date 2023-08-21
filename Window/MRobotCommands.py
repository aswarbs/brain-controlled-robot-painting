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
from random import choice


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
        
        
        self.iterations = 10
        self.rotation_per_iteration = pi/self.iterations
        self.direction = 1
        self.orientation = 1
        self.small_direction = 1
        self.current_rotation = 0.0
        self.total_rotation = 0
        self.significant_difference = 0.1

        self.prev_alpha = 0.5
        self.prev_beta = 0.5

        self.plan_joints = [0] * 6

        self.move_to_initial_position()
        self.plan_goal()

        self.move_down()
        self.plan_goal()




        self.readLoop()

        

        while True:
            self.move_backward(pi/16 * self.orientation)
            self.plan_goal()

    def move_backward(self, amount):

        if((self.plan_joints[1] + amount) > ((3 * pi)/8) and amount > 0):
            amount *= -1
            self.orientation *= -1

        elif((self.plan_joints[1] + amount) < ((5 * pi)/16) and amount < 0):
            amount *= -1
            self.orientation *= -1


        self.plan_joints[1] += amount
        self.plan_joints[2] -= (3 * amount)/2
        self.plan_joints[3] += amount/2

    def readLoop(self):
            




        with open('mapped_rotations.csv', 'r') as file:
            csv_reader = csv.reader(file)

            while True:
                print("reading...")
                try:
                    current_row = next(csv_reader)

                    # Convert the elements to float
                    current_row = [float(value) for value in current_row]


                    print("current row: ", current_row)

                    self.penLoop(current_row)

                except StopIteration:
                    # If there is no next row, sleep or perform other desired actions
                    time.sleep(0.01)
                    # You can also break the loop if you don't want to continue reading the file
                    # break
                    print("WAITING")

    def penLoop(self, rotations):



        alpha_rotation = rotations[0] * pi/2
        beta_rotation = rotations[1] * (pi / 4)

        alpha_difference = rotations[0] - self.prev_alpha
        beta_difference = rotations[1] - self.prev_beta

        if(alpha_difference < -self.significant_difference):
            # alpha is lower
            # the small circle will oscillate
            self.small_direction *= -1

        if(alpha_difference > self.significant_difference):
            alpha_rotation *= 2

        if(beta_difference > self.significant_difference):
            # beta is higher
            # the large circle will oscillate
            self.direction *= -1

        if(beta_difference < -self.significant_difference):
            beta_rotation *= 2


        self.circle_left(alpha_rotation * self.small_direction)

        self.rotate_left(beta_rotation * self.direction)

        """noise = rotations[2]
        noise = noise * (pi/8)
        noise = noise - (pi/16)
        # -pi/16 < noise < pi/16
            
        self.move_backward(noise)"""

        self.plan_goal()

        self.prev_alpha = rotations[0]
        self.prev_beta = rotations[1]



        print("done")



    def move_to_initial_position(self):    

        self.plan_joints[0] = random.uniform(pi/4, pi/2)
        self.plan_joints[1] = 0
        self.plan_joints[3] = 0
        self.plan_joints[4] = random.uniform(0, pi)
        self.plan_joints[5] = pi/4


        
    def move_down(self):

        self.plan_joints[1] += pi/4
        self.plan_joints[2] -= pi/4
        
    def move_up(self):
        self.plan_joints[1] -= pi/4
        self.plan_joints[2] += pi/4


    def move_left(self, amount):

        self.plan_joints[0] += amount
        self.plan_joints[4] -= amount






    def rotate_left(self, amount):



        self.plan_joints[0] += (amount)

        # keep pen on page
        if((self.plan_joints[0] + amount < pi/4) and (amount < 0)):
            self.direction *= -1

        if((self.plan_joints[0] + amount > pi/2) and (amount > 0)):
            self.direction *= -1


    def circle_left(self, amount):

        
        if(self.plan_joints[4] + amount > (3 * pi)/2):
            amount *= -1
            self.small_direction *= -1

        elif(self.plan_joints[4] + amount < -(3 * pi)/2):
            amount *= -1
            self.small_direction *= -1

        self.plan_joints[4] += amount





    def return_to_home(self):
        self.plan_joints[0] = 0
        self.plan_joints[1] = -pi/2
        self.plan_joints[2] = 0
        self.plan_joints[3] = -pi/2
        self.plan_joints[4] = 0
        self.plan_joints[5] = 0

        self.plan_goal()





        
    def plan_goal(self):

        # Set a positive velocity for the joint (pi/8 radians per second for example, adjust as needed).
        joint_velocity = pi / 8


        # Move the joint to the target configuration using velocity control.
        self.group.go(self.plan_joints, wait=True)


        print("gone")

        # Calling ``stop()`` ensures that there is no residual movement.
        self.group.stop()

        print("stopped!")


robot = RobotCommands()