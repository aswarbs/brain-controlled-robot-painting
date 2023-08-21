#!/usr/bin/env python3
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
import mne
import turtle
import random
import io
import pandas as pd
import numpy as np
from tkinter import *
from tkinter import filedialog
import os
import glob
import csv


# Open the file in read mode
with open('mapped_rotations.csv', 'r') as file:
    # Read the first line
	csv_reader = csv.reader(file)
	psds = next(csv_reader)
	print(psds)

alpha = float(psds[0])
beta = float(psds[1])



moveit_commander.roscpp_initialize(sys.argv)
#rospy.init_node('/my_node', anonymous=True)


rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander(robot_description="robot_description", ns="/ur3e")
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("manipulator")
display_trajectory_publisher = rospy.Publisher('/ur3e/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)



def tired():
	group.set_named_target("start_circle")
	plan1 = group.plan()
	group.execute(plan1[1])

	group.set_named_target("draw_circle")
	plan2 = group.plan()
	group.execute(plan2[1])

	group.set_named_target("pen_up_circle")
	plan3 = group.plan()
	group.execute(plan3[1])
	
	group.set_named_target("new_up")
	plan6 = group.plan()
	group.execute(plan6[1])

	group.set_named_target("face1_eyebrow1_start")
	plan4 = group.plan()
	group.execute(plan4[1])

	group.set_named_target("face1_eyebrow1_end")
	plan5 = group.plan()
	group.execute(plan5[1])

	group.set_named_target("pen_up")
	plan6 = group.plan()
	group.execute(plan6[1])
	
	

	group.set_named_target("face1_eyebrow2_start")
	plan6 = group.plan()
	group.execute(plan6[1])

	group.set_named_target("face1_eyebrow2_end")
	plan7 = group.plan()
	group.execute(plan7[1])

	group.set_named_target("pen_up")
	plan8 = group.plan()
	group.execute(plan8[1])

	group.set_named_target("face1_eye1_start")
	plan9 = group.plan()
	group.execute(plan9[1])

	group.set_named_target("face1_eye1_end")
	plan10 = group.plan()
	group.execute(plan10[1])

	
	group.set_named_target("pen_up")
	plan11 = group.plan()
	group.execute(plan11[1])

	
	group.set_named_target("face1_eye2_start")
	plan12 = group.plan()
	group.execute(plan12[1])

	
	group.set_named_target("face1_eye2_end")
	plan13 = group.plan()
	group.execute(plan13[1])

	
	group.set_named_target("pen_up_mouth")
	plan14 = group.plan()
	group.execute(plan14[1])

	
	group.set_named_target("face1_mouth_start")
	plan15 = group.plan()
	group.execute(plan15[1])

	
	group.set_named_target("face1_mouth_middle")
	plan16 = group.plan()
	group.execute(plan16[1])

	
	group.set_named_target("face1_mouth_end")
	plan17 = group.plan()
	group.execute(plan17[1])

	
	group.set_named_target("home")
	plan18 = group.plan()
	group.execute(plan18[1])

	moveit_commander.roscpp_shutdown()
	
def relaxed():
	group.set_named_target("start_circle")
	plan1 = group.plan()
	group.execute(plan1[1])

	
	group.set_named_target("draw_circle")
	plan2 = group.plan()
	group.execute(plan2[1])

	
	group.set_named_target("pen_up")
	plan3 = group.plan()
	group.execute(plan3[1])

	
	group.set_named_target("face2_eyebrow1_start")
	plan4 = group.plan()
	group.execute(plan4[1])

	
	group.set_named_target("face2_eyebrow1_middle")
	plan5 = group.plan()
	group.execute(plan5[1])

	
	group.set_named_target("face2_eyebrow1_end")
	plan6 = group.plan()
	group.execute(plan6[1])

	
	group.set_named_target("pen_up")
	plan6 = group.plan()
	group.execute(plan6[1])

	
	group.set_named_target("face2_eyebrow2_start")
	plan7 = group.plan()
	group.execute(plan7[1])

	
	group.set_named_target("face2_eyebrow2_middle")
	plan8 = group.plan()
	group.execute(plan8[1])

	
	group.set_named_target("face2_eyebrow2_end")
	plan9 = group.plan()
	group.execute(plan9[1])

	group.set_named_target("pen_up")
	plan10 = group.plan()
	group.execute(plan10[1])

	
	group.set_named_target("face2_eye1_start")
	plan11 = group.plan()
	group.execute(plan11[1])

	
	group.set_named_target("face2_eye1_middle")
	plan12 = group.plan()
	group.execute(plan12[1])

	
	group.set_named_target("face2_eye1_end")
	plan13 = group.plan()
	group.execute(plan13[1])

	
	group.set_named_target("pen_up")
	plan14 = group.plan()
	group.execute(plan14[1])

	
	group.set_named_target("face2_eye2_start")
	plan15 = group.plan()
	group.execute(plan15[1])

	
	group.set_named_target("face2_eye2_middle")
	plan16 = group.plan()
	group.execute(plan16[1])

	
	group.set_named_target("face2_eye2_end")
	plan17 = group.plan()
	group.execute(plan17[1])

	
	"""group.set_named_target("pen_up_mouth")
	plan18 = group.plan()
	group.execute(plan18[1])
	
	
	group.set_named_target("face2_mouth_start")
	plan18 = group.plan()
	group.execute(plan18[1])
	
	
	group.set_named_target("face2_mouth_middle")
	plan18 = group.plan()
	group.execute(plan18[1])
	
	
	group.set_named_target("face2_mouth_end")
	plan18 = group.plan()
	group.execute(plan18[1])"""

	group.set_named_target("pen_up")
	plan14 = group.plan()
	group.execute(plan14[1])
		
	group.set_named_target("f4_mouth_start")
	plan10 = group.plan()
	group.execute(plan10[1])

	group.set_named_target("f4_mouth_end")
	plan11 = group.plan()
	group.execute(plan11[1])
	
	
	group.set_named_target("home")
	plan18 = group.plan()
	group.execute(plan18[1])

	moveit_commander.roscpp_shutdown()
	
def anxiety():
	group.set_named_target("start_circle")
	plan1 = group.plan()
	group.execute(plan1[1])

	
	group.set_named_target("draw_circle")
	plan2 = group.plan()
	group.execute(plan2[1])

	
	group.set_named_target("eb_up")
	plan3 = group.plan()
	group.execute(plan3[1])

	
	group.set_named_target("f3_eb1_start")
	plan4 = group.plan()
	group.execute(plan4[1])

	
	group.set_named_target("f3_eb1_middle")
	plan5 = group.plan()
	group.execute(plan5[1])

	
	group.set_named_target("f3_eb1_end")
	plan6 = group.plan()
	group.execute(plan6[1])

	
	group.set_named_target("eb_up")
	plan6 = group.plan()
	group.execute(plan6[1])

	
	group.set_named_target("fc3_eb2_start")
	plan7 = group.plan()
	group.execute(plan7[1])

	
	group.set_named_target("f3_eb2_mid")
	plan8 = group.plan()
	group.execute(plan8[1])

	
	group.set_named_target("f3_eb2_end")
	plan9 = group.plan()
	group.execute(plan9[1])

	group.set_named_target("eb_up")
	plan10 = group.plan()
	group.execute(plan10[1])

	
	group.set_named_target("f3_eye1_start")
	plan11 = group.plan()
	group.execute(plan11[1])

	
	group.set_named_target("f3_eye1_end")
	plan12 = group.plan()
	group.execute(plan12[1])

	
	group.set_named_target("eb_up")
	plan13 = group.plan()
	group.execute(plan13[1])

	
	group.set_named_target("f3_eye2_start")
	plan14 = group.plan()
	group.execute(plan14[1])

	
	group.set_named_target("f3_eye2_end")
	plan15 = group.plan()
	group.execute(plan15[1])

	
	group.set_named_target("eb_up")
	plan16 = group.plan()
	group.execute(plan16[1])

	
	group.set_named_target("f3_m1")
	plan17 = group.plan()
	group.execute(plan17[1])

	
	group.set_named_target("f3_m2")
	plan18 = group.plan()
	group.execute(plan18[1])
	
	
	group.set_named_target("f3_m3")
	plan18 = group.plan()
	group.execute(plan18[1])
	
	
	group.set_named_target("f3_m4")
	plan18 = group.plan()
	group.execute(plan18[1])
	
	
	group.set_named_target("f3_m5")
	plan18 = group.plan()
	group.execute(plan18[1])
	
	
	group.set_named_target("home")
	plan18 = group.plan()
	group.execute(plan18[1])

	moveit_commander.roscpp_shutdown()
	
def sleepy():
	group.set_named_target("start_circle")
	plan1 = group.plan()
	group.execute(plan1[1])

	
	group.set_named_target("draw_circle")
	plan2 = group.plan()
	group.execute(plan2[1])

	
	group.set_named_target("pen_up")
	plan3 = group.plan()
	group.execute(plan3[1])

	
	group.set_named_target("face1_eye1_start")
	plan4 = group.plan()
	group.execute(plan4[1])

	
	group.set_named_target("face1_eye1_end")
	plan5 = group.plan()
	group.execute(plan5[1])

	
	group.set_named_target("pen_up")
	plan6 = group.plan()
	group.execute(plan6[1])

	
	group.set_named_target("face1_eye2_start")
	plan6 = group.plan()
	group.execute(plan6[1])

	
	group.set_named_target("face1_eye2_end")
	plan7 = group.plan()
	group.execute(plan7[1])

	
	group.set_named_target("pen_up")
	plan8 = group.plan()
	group.execute(plan8[1])

	
	"""group.set_named_target("mouth_up")
	plan8 = group.plan()
	group.execute(plan8[1])
	
	
	group.set_named_target("s1")
	plan9 = group.plan()
	group.execute(plan9[1])

	group.set_named_target("s2")
	plan10 = group.plan()
	group.execute(plan10[1])

	
	group.set_named_target("s3")
	plan11 = group.plan()
	group.execute(plan11[1])

	
	group.set_named_target("s4")
	plan12 = group.plan()
	group.execute(plan12[1])
	
	
	group.set_named_target("s1")
	plan12 = group.plan()
	group.execute(plan12[1])
	
	
	group.set_named_target("mouth_up")
	plan8 = group.plan()
	group.execute(plan8[1])"""

		
	group.set_named_target("f4_mouth_start")
	plan10 = group.plan()
	group.execute(plan10[1])

	group.set_named_target("f4_mouth_end")
	plan11 = group.plan()
	group.execute(plan11[1])


	
	group.set_named_target("pen_z_up")
	plan13 = group.plan()
	group.execute(plan13[1])

	
	group.set_named_target("z1_1")
	plan14 = group.plan()
	group.execute(plan14[1])

	
	group.set_named_target("z1_2")
	plan15 = group.plan()
	group.execute(plan15[1])

	
	group.set_named_target("z1_3")
	plan16 = group.plan()
	group.execute(plan16[1])

	
	group.set_named_target("z1_4")
	plan17 = group.plan()
	group.execute(plan17[1])

	
	group.set_named_target("pen_z_up")
	plan18 = group.plan()
	group.execute(plan18[1])
	
	
	group.set_named_target("z2_1")
	plan18 = group.plan()
	group.execute(plan18[1])
	
	
	group.set_named_target("z2_2")
	plan18 = group.plan()
	group.execute(plan18[1])
	
	
	group.set_named_target("z2_3")
	plan18 = group.plan()
	group.execute(plan18[1])
	
	
	group.set_named_target("z2_4")
	plan18 = group.plan()
	group.execute(plan18[1])
	
	
	group.set_named_target("pen_z_up")
	plan18 = group.plan()
	group.execute(plan18[1])
	
	
	"""group.set_named_target("z3_1")
	plan19 = group.plan()
	group.execute(plan19[1])
	
	
	group.set_named_target("z3_2")
	plan20 = group.plan()
	group.execute(plan20[1])
	
	
	group.set_named_target("z3_3")
	plan21 = group.plan()
	group.execute(plan21[1])
	
	
	group.set_named_target("z3_4")
	plan22 = group.plan()
	group.execute(plan22[1])"""
	
	
	group.set_named_target("home")
	plan23 = group.plan()
	group.execute(plan23[1])

	moveit_commander.roscpp_shutdown()
	
def confusion():
	group.set_named_target("start_circle")
	plan1 = group.plan()
	group.execute(plan1[1])

	
	group.set_named_target("draw_circle")
	plan2 = group.plan()
	group.execute(plan2[1])

	
	group.set_named_target("eb_up")
	plan3 = group.plan()
	group.execute(plan3[1])

	
	group.set_named_target("f3_eye1_start")
	plan4 = group.plan()
	group.execute(plan4[1])

	
	group.set_named_target("f3_eye1_end")
	plan5 = group.plan()
	group.execute(plan5[1])

	
	group.set_named_target("eb_up")
	plan6 = group.plan()
	group.execute(plan6[1])

	
	group.set_named_target("f3_eye2_start")
	plan7 = group.plan()
	group.execute(plan7[1])

	
	group.set_named_target("f3_eye2_end")
	plan8 = group.plan()
	group.execute(plan8[1])

	
	group.set_named_target("eb_up")
	plan9 = group.plan()
	group.execute(plan9[1])

	
	group.set_named_target("f4_mouth_start")
	plan10 = group.plan()
	group.execute(plan10[1])

	group.set_named_target("f4_mouth_end")
	plan11 = group.plan()
	group.execute(plan11[1])

	
	group.set_named_target("eb_up")
	plan12 = group.plan()
	group.execute(plan12[1])

	
	group.set_named_target("q1")
	plan13 = group.plan()
	group.execute(plan13[1])

	
	group.set_named_target("q2")
	plan14 = group.plan()
	group.execute(plan14[1])

	
	group.set_named_target("q3")
	plan15 = group.plan()
	group.execute(plan15[1])

	
	group.set_named_target("q4")
	plan16 = group.plan()
	group.execute(plan16[1])

	
	group.set_named_target("q5")
	plan17 = group.plan()
	group.execute(plan17[1])

	
	group.set_named_target("eb_up")
	plan18 = group.plan()
	group.execute(plan18[1])

	
	group.set_named_target("q6")
	plan19 = group.plan()
	group.execute(plan19[1])
	
	
	group.set_named_target("q7")
	plan20 = group.plan()
	group.execute(plan20[1])
	
	
	group.set_named_target("eb_up")
	plan21 = group.plan()
	group.execute(plan21[1])
	
	
	group.set_named_target("q2_1")
	plan22 = group.plan()
	group.execute(plan22[1])
	
	
	group.set_named_target("q2_2")
	plan23 = group.plan()
	group.execute(plan23[1])
	
	
	group.set_named_target("q2_3")
	plan24 = group.plan()
	group.execute(plan24[1])
	
	
	group.set_named_target("q2_4")
	plan25 = group.plan()
	group.execute(plan25[1])
	
	
	group.set_named_target("q2_5")
	plan26 = group.plan()
	group.execute(plan26[1])
	
	
	group.set_named_target("eb_up")
	plan21 = group.plan()
	group.execute(plan21[1])
	
	
	group.set_named_target("q2_6")
	plan27 = group.plan()
	group.execute(plan27[1])
	
	
	group.set_named_target("q2_7")
	plan28 = group.plan()
	group.execute(plan28[1])
	
	
	group.set_named_target("eb_up")
	plan29 = group.plan()
	group.execute(plan29[1])
	
	
	group.set_named_target("q3_1")
	plan30 = group.plan()
	group.execute(plan30[1])
	
	
	group.set_named_target("q3_2")
	plan31 = group.plan()
	group.execute(plan31[1])
	
	
	group.set_named_target("q3_3")
	plan32 = group.plan()
	group.execute(plan32[1])
	
	
	group.set_named_target("q3_4")
	plan33 = group.plan()
	group.execute(plan33[1])
	
	
	group.set_named_target("q3_5")
	plan34 = group.plan()
	group.execute(plan34[1])
	
	
	group.set_named_target("eb_up")
	plan35 = group.plan()
	group.execute(plan35[1])
	
	
	group.set_named_target("q3_6")
	plan36 = group.plan()
	group.execute(plan36[1])
	
	
	group.set_named_target("q3_7")
	plan37 = group.plan()
	group.execute(plan37[1])

	
	group.set_named_target("home")
	plan38 = group.plan()
	group.execute(plan38[1])
	moveit_commander.roscpp_shutdown()
	
def thinking():
	group.set_named_target("start_circle")
	plan1 = group.plan()
	group.execute(plan1[1])

	
	group.set_named_target("draw_circle")
	plan2 = group.plan()
	group.execute(plan2[1])

	
	group.set_named_target("pen_up_circle")
	plan3 = group.plan()
	group.execute(plan3[1])
	
	
	group.set_named_target("pen_up")
	plan7 = group.plan()
	group.execute(plan7[1])

	
	group.set_named_target("face2_eyebrow1_start")
	plan4 = group.plan()
	group.execute(plan4[1])

	
	group.set_named_target("face2_eyebrow1_middle")
	plan5 = group.plan()
	group.execute(plan5[1])

	
	group.set_named_target("face2_eyebrow1_end")
	plan6 = group.plan()
	group.execute(plan6[1])

	
	group.set_named_target("pen_up")
	plan7 = group.plan()
	group.execute(plan7[1])

	
	group.set_named_target("face2_eyebrow2_start")
	plan8 = group.plan()
	group.execute(plan8[1])

	
	group.set_named_target("face2_eyebrow2_middle")
	plan9 = group.plan()
	group.execute(plan9[1])

	
	group.set_named_target("face2_eyebrow2_end")
	plan10 = group.plan()
	group.execute(plan10[1])

	group.set_named_target("pen_up")
	plan11 = group.plan()
	group.execute(plan11[1])

	group.set_named_target("eb_up")
	plan11 = group.plan()
	group.execute(plan11[1])

	
	group.set_named_target("f3_eye1_start")
	plan12 = group.plan()
	group.execute(plan12[1])

	
	group.set_named_target("f3_eye1_end")
	plan13 = group.plan()
	group.execute(plan13[1])

	
	group.set_named_target("eb_up")
	plan14 = group.plan()
	group.execute(plan14[1])

	
	group.set_named_target("f3_eye2_start")
	plan15 = group.plan()
	group.execute(plan15[1])

	
	group.set_named_target("f3_eye2_end")
	plan16 = group.plan()
	group.execute(plan16[1])

	
	group.set_named_target("eb_up")
	plan17 = group.plan()
	group.execute(plan17[1])

	
	group.set_named_target("f4_mouth_start")
	plan18 = group.plan()
	group.execute(plan18[1])

	
	group.set_named_target("f4_mouth_end")
	plan19 = group.plan()
	group.execute(plan19[1])
	
	
	group.set_named_target("pen_up_mouth")
	plan20 = group.plan()
	group.execute(plan20[1])
	
	
	group.set_named_target("square1_1")
	plan21 = group.plan()
	group.execute(plan21[1])
	
	
	group.set_named_target("square1_2")
	plan22 = group.plan()
	group.execute(plan22[1])
	
	
	group.set_named_target("square1_3")
	plan23 = group.plan()
	group.execute(plan23[1])
	
	
	group.set_named_target("square1_4")
	plan24 = group.plan()
	group.execute(plan24[1])
	
	
	group.set_named_target("square1_1")
	plan21 = group.plan()
	group.execute(plan21[1])
	
	
	group.set_named_target("square_up")
	plan25 = group.plan()
	group.execute(plan25[1])
	
	
	group.set_named_target("square2_1")
	plan26 = group.plan()
	group.execute(plan26[1])
	
	
	group.set_named_target("square2_2")
	plan21 = group.plan()
	group.execute(plan21[1])
	
	
	group.set_named_target("square2_3")
	plan27 = group.plan()
	group.execute(plan27[1])
	
	
	group.set_named_target("square2_4")
	plan28 = group.plan()
	group.execute(plan28[1])
	
	
	group.set_named_target("square2_1")
	plan29 = group.plan()
	group.execute(plan29[1])
	
	
	group.set_named_target("square_up")
	plan30 = group.plan()
	group.execute(plan30[1])
	
	
	group.set_named_target("square3_1")
	plan31 = group.plan()
	group.execute(plan31[1])
	
	
	group.set_named_target("square3_2")
	plan32 = group.plan()
	group.execute(plan32[1])
	
	
	group.set_named_target("square3_3")
	plan33 = group.plan()
	group.execute(plan33[1])
	
	
	group.set_named_target("square3_4")
	plan34 = group.plan()
	group.execute(plan34[1])
	
	
	group.set_named_target("square3_1")
	plan35 = group.plan()
	group.execute(plan35[1])
	
	
	group.set_named_target("home")
	plan36 = group.plan()
	group.execute(plan36[1])


# change these if beta increases on tests

if beta < 0.05:
	print("sleepy")
	sleepy()

elif beta < 0.1:
	print("tired")
	tired()

elif beta < 0.15:
	print("relaxed")
	relaxed()

elif beta < 0.2:
	print("thinking")
	thinking()

elif beta < 0.25:
	print("anxiety")
	anxiety()

elif beta  >= 0.3:
	print("confused")
	confusion()
	






