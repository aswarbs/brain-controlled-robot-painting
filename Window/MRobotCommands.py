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
        self.small_direction = 1

        self.plan_joints = [0] * 6

        self.plan_joints = self.group.get_current_joint_values()

        self.return_to_home()

        self.move_to_initial_position()

        self.move_down()
        self.plan_goal()


        self.readLoop()

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

        
        alpha_rotation = (rotations[0] * pi/2) - pi/4 #between -pi/4 and pi/4
        beta_rotation = (rotations[1] * (2 * pi))

        self.rotate_left(self.rotation_per_iteration * self.direction)
        self.circle_left(beta_rotation)
        self.small_direction *= -1

        noise = rotations[2]

        """if(noise > 0.5):
            # there is a lot of noise.. penup / pendown?
            self.move_forward(noise * self.small_direction)

            self.plan_goal()"""

        print(alpha_rotation, beta_rotation, "rotations")



        """noise = rotations[2]

        functions = [self.move_forward, self.move_left]


        if(noise > 0.5):
            # there is a lot of noise.. penup / pendown?

            print("noise: ", noise)

            choice(functions)(noise)
            self.plan_goal()



        # Alpha
        # Controls Rotation
        self.rotate_left(alpha_rotation)
        self.circle_left(beta_rotation)"""





        self.plan_goal()

        print("done")


    def move_to_initial_position(self):    

        self.plan_joints = self.group.get_current_joint_values()    

        self.plan_joints[0] = pi/2
        self.plan_joints[1] = 0
        self.plan_joints[3] = 0
        self.plan_joints[4] = pi/2
        self.plan_joints[5] = pi/4

        self.plan_goal()
    

        
    def move_down(self):

        self.plan_joints[1] += pi/4
        self.plan_joints[2] -= pi/4
    
        

    def move_left(self, amount):

        self.plan_joints[0] += amount
        self.plan_joints[4] -= amount



    def move_forward(self, amount):

        self.plan_joints[4] -= amount
        self.plan_joints[0] -= amount / 8



    def rotate_left(self, amount):

        self.plan_joints[0] += amount


    def circle_left(self, amount):

        self.plan_joints[4] += amount



    def return_to_home(self):
        self.plan_joints = self.group.get_current_joint_values()
        self.plan_joints[0] = 0
        self.plan_joints[1] = -pi/2
        self.plan_joints[2] = 0
        self.plan_joints[3] = -pi/2
        self.plan_joints[4] = 0
        self.plan_joints[5] = 0

        self.plan_goal()


    def check_boundaries(self):
        """Check any boundaries set."""

        # FIRST CHECK THE PAGE / JOINT BOUNDARIES

        # check overflow
        for x in range(0, len(self.plan_joints )):

            if(self.plan_joints [x] < -(2 * pi)):
                self.plan_joints [x] += (2 *pi)

            if(self.plan_joints [x] > (2 * pi)):
                self.plan_joints [x] -= (2*pi)

        # keep pen on page
        if(self.plan_joints[0] < pi/4):
            #self.plan_joints[0] += pi/4
            self.direction *= -1

        if(self.plan_joints[0] > ((5 * pi)/8)):
            #self.plan_joints[0] -= pi/4
            self.direction *= -1


        # ADDITIONAL OPTIONAL BOUNDARIES HERE



    def plan_goal(self):

        self.check_boundaries()

        print("joints: ", self.plan_joints)

        self.group.go(self.plan_joints, wait=True)

        print("gone")

        # Calling ``stop()`` ensures that there is no residual movement
        self.group.stop()

        print("stopped!")

robot = RobotCommands()