import turtle
from tkinter import *
import random
from math import cos, sin, pi
from Window import *
import numpy as np
import math
from threading import Thread
import csv
import time

class FrameSimulation(FrameBase):

    """
    A frame to allow the user to apply a simulation of the pen's trajectory using a specified CSV.
    """
    def __init__(self, master: Tk, main_window, parent_window, **args):
        self.master_window = parent_window
        super().__init__(master, main_window)
        self.pack_propagate(0)
        self.configure(highlightthickness=0)

        self.MAX_ANGLE = 360
        
        self.color_index = 0 
        self.colors = parent_window.frames["filters_frame"].colours
        
        self.forward = 3  # default speed of line
        self.stop = False
        self.direction = 1
        
        self.canvas = Canvas(self)
        self.canvas.config(bd=0, highlightthickness=0)
        self.canvas.pack(fill=BOTH, expand=TRUE)
        self.pack(fill=BOTH, expand=TRUE)
        self.pen = turtle.RawTurtle(self.canvas)  # create pen in canvas 
        self.penSpawn()  # spawns in top left corner

        self.current_angle = 0

        self.small_x_boundary = -50
        self.large_x_boundary = 500
        self.small_y_boundary = -500
        self.large_y_boundary = -50

        self.trajectory = self.master_window.get_trajectory()
        if(self.trajectory == "Square"):
            self.initialise_square()

        thread = Thread(target=self.readLoop)
        thread.start()


    def readLoop(self):


        with open('mapped_rotations.csv', 'r') as file:
            csv_reader = csv.reader(file)

            while True:
                try:
                    current_row = next(csv_reader)

                    # Convert the elements to float
                    current_row = [float(value) for value in current_row]

                    self.penLoop(current_row)

                except StopIteration:
                    # If there is no next row, sleep or perform other desired actions
                    time.sleep(0.01)
                    # You can also break the loop if you don't want to continue reading the file
                    # break

        

    def handle_blink(self):
        # penup / pendown
        print("handling blink")
        if(self.pen.isdown()):
            self.pen.penup()
        else:
            self.pen.pendown()

        
    def penStop(self):
        """
        Stops the simulation on press of the pause button.
        """
        self.stop = True  # when stop button is pressed
    
    def penSpawn(self):
        """
        Spawns the pen to the screen in the middle of the canvas.
        """
        self.pen.clear()
        self.pen.speed(1)
        self.pen.width(2)
        self.pen.color(self.colors[0])
        canvas_width = self.canvas.winfo_width()
        canvas_height = self.canvas.winfo_height()
        x = canvas_width / 2
        y = canvas_height / 2
        turtle_x = x - canvas_width / 2
        turtle_y = canvas_height / 2 - y
        self.pen.penup()
        self.pen.goto(turtle_x, turtle_y)
        self.pen.pendown()




    def penBlink(self):
        """
        Controls the current colour of the pen.
        """
        self.color_index = (self.color_index + 1) % len(self.colors)  # goes to next colour in rainbow
        self.pen.color(self.colors[self.color_index])
    
    def penLoop(self, mapped_rotations) -> None:
        """
        The operations performed by the pen on every loop.
        """

        # instead of random movements, base them on mapped_rotations.
        # each rotation between 0 and 1
        # 3 rotations - need to convert to degrees

        alpha_percentage = mapped_rotations[0]
        alpha_curvature = int(alpha_percentage * 3) + 1
        # curvature is an int between 1 and 3

        current_angle = self.pen.heading()
        beta_angle = mapped_rotations[1] * self.MAX_ANGLE


        self.forward_amount = mapped_rotations[2] * 100


        # make the angle switch direction each pass
        beta_angle *= self.direction
        

        angle_per_segment = beta_angle / alpha_curvature
        forward_per_segment = self.forward_amount / alpha_curvature

        prev_angle = self.current_angle

        

        for current_segment in range(0, alpha_curvature):
            current_angle = prev_angle + angle_per_segment
            self.pen.setheading(current_angle)
            self.move_forward(forward_per_segment)
            prev_angle = current_angle

        angle = self.current_angle - current_angle
        angle_per_segment = angle / alpha_curvature

        for current_segment in range(0, alpha_curvature - 1):
            current_angle = prev_angle + angle_per_segment
            self.pen.setheading(current_angle)
            self.move_forward(forward_per_segment)
            prev_angle = current_angle



        # PERFORM TRAJECTORY BOUNDARY CHECK
        if(self.trajectory == "Square"):
            
            if(self.perform_square() == True):
                self.switch_side()

        self.boundary_check()
        self.direction *= -1 # oscillate

        print(self.pen.xcor(), self.pen.ycor())

        

        self.master_window.signal_done()


    def initialise_square(self):

        self.BOUNDARY = 100
        self.ending_x = 0
        self.ending_y = 0

        self.current_angle = 90

        self.switch_side()

    def boundary_check(self):
        if(self.pen.xcor() < self.small_x_boundary or self.pen.xcor() > self.large_x_boundary or self.pen.ycor() < self.small_y_boundary or self.pen.ycor() > self.large_y_boundary):
            self.pen.setheading(self.pen.heading() + 180) # turn pen around
            self.move_forward(self.forward_amount)
            print("yes")



    def switch_side(self):
        self.starting_x = self.ending_x
        self.starting_y = self.ending_y
        self.current_angle -= 90

        if(self.starting_x == 0 and self.starting_y == 0):
            self.ending_x = 400
            self.ending_y = 0
            self.changing_x = True
            self.increasing = True


        elif(self.starting_x == 400 and self.starting_y == 0):
            self.ending_x = 400
            self.ending_y = -400
            self.changing_x = False
            self.increasing = False

        elif(self.starting_x == 400 and self.starting_y == -400):
            self.ending_x = 0
            self.ending_y = -400
            self.changing_x = True
            self.increasing = False

        else:
            self.ending_x = 0
            self.ending_y = 0
            self.changing_x = False
            self.increasing = True

    
        self.small_x_boundary = min(self.starting_x, self.ending_x) - self.BOUNDARY
        self.large_x_boundary = max(self.starting_x, self.ending_x) + self.BOUNDARY
        self.small_y_boundary = min(self.starting_y, self.ending_y) - self.BOUNDARY
        self.large_y_boundary = max(self.starting_y, self.ending_y) + self.BOUNDARY

        
        self.pen.setheading(self.current_angle)
        self.MAX_ANGLE = 90



    def perform_square(self):


        if(self.changing_x):

            if(self.has_coordinate_reached_goal(self.pen.xcor(), self.ending_x, self.increasing)):
                return True
        else:
            if(self.has_coordinate_reached_goal(self.pen.ycor(), self.ending_y, self.increasing)):
                return True
            




        return False


    def has_coordinate_reached_goal(self, current_variable, target_variable, increasing):
        if increasing:
            return current_variable >= target_variable
        else:
            return current_variable <= target_variable




# height boundary: store current position

    def perform_circle(self):
        self.pen.setheading(-135)
        self.rotate_left(360)

    
    def move_forward(self, amount):
        """Move the pen forward by the specified amount."""
        self.pen.forward(amount)
    
    def move_left(self, amount):
        """Turn the pen left and move forward by the specified amount."""
        self.pen.left(90)
        self.pen.forward(amount)
    
    def circle_left(self, amount):
        """Make a small left circle movement with the pen."""
        self.pen.circle(25, amount)
    
    def rotate_left(self, amount):
        """Make a large left circle movement with the pen."""
        self.pen.circle(250, amount)
    