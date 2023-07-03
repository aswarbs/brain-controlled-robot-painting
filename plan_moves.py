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

        self.plan_joints = self.move_to_initial_position()

        self.plan_joints = self.move_down(plan_joints)
        self.plan_goal(plan_joints)



    def penLoop(self, rotations):

        
        alpha_rotation = rotations[0]
        beta_rotation = rotations[1]
        theta_rotation = rotations[2]

        # Alpha
        # Controls Rotation
        self.plan_joints = self.rotate_left(alpha_rotation / 4)
        self.plan_joints = self.circle_left(alpha_rotation / 4)


        # Beta
        # Used to control left and right
        self.plan_joints = self.move_left(beta_rotation)


        # Theta
        # Controls up & downs
        self.plan_joints = self.move_forward(theta_rotation)

        self.plan_goal()




    def move_to_initial_position(self):    
        print("initial")  

        self.plan_joints = self.group.get_current_joint_values()    

        self.plan_joints [0] = pi/2
        self.plan_joints [1] = 0
        self.plan_joints [3] = 0
        self.plan_joints [4] = pi
        self.plan_joints [5] = pi/4

        self.plan_goal()
    

        
    def move_down(self):

        self.plan_joints [1] += pi/4
        self.plan_joints [2] -= pi/4

    
        

    def move_left(self, amount):

        self.plan_joints [0] += amount
        self.plan_joints [4] -= amount



    def move_forward(self, amount):

        self.plan_joints[4] -= amount
        self.plan_joints [0] -= amount / 8



    def rotate_left(self, amount):

        self.plan_joints [0] += amount


    def circle_left(self, amount):

        self.plan_joints [4] += amount



    def return_to_home(self):
        print("return to home")
        self.plan_joints = self.group.get_current_joint_values()
        self.plan_joints [0] = 0
        self.plan_joints [1] = -pi/2
        self.plan_joints [2] = 0
        self.plan_joints [3] = -pi/2
        self.plan_joints [4] = 0
        self.plan_joints [5] = 0

        self.plan_goal()


    def check_boundaries(self):
        """Check any boundaries set."""

        # FIRST CHECK THE PAGE / JOINT BOUNDARIES

        # check overflow
        for x in range(0, len(self.plan_joints )):

            if(self.plan_joints [x] < -(2 * pi)):
                self.plan_joints [x] += 2 * pi

            if(self.plan_joints [x] > (2 * pi)):
                self.plan_joints [x] -= 2 * pi

        # keep pen on page
        if(self.plan_joints [0] < pi/4):
            self.plan_joints [0] += pi/4

        if(self.plan_joints [0] > (3 * pi)/4):
            self.plan_joints [0] -= pi/4


        # ADDITIONAL OPTIONAL BOUNDARIES HERE



    def plan_goal(self):

        self.check_boundaries()

        self.group.go(self.plan_joints , wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.group.stop()