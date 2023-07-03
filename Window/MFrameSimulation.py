import turtle
from tkinter import *
import random
from math import cos, sin, pi
from Window import *
import numpy as np
import math

class FrameSimulation(FrameBase):
    """
    A frame to allow the user to apply a simulation of the pen's trajectory using a specified CSV.
    """
    def __init__(self, master: Tk, main_window, parent_window, **args):
        self.master_window = parent_window
        super().__init__(master, main_window)
        self.pack_propagate(0)
        self.configure(highlightthickness=0)
        
        self.color_index = 0 
        self.colors = parent_window.frames["filters_frame"].colours
        
        self.forward = 3  # default speed of line
        self.stop = False
        
        self.canvas = Canvas(self)
        self.canvas.config(bd=0, highlightthickness=0)
        self.canvas.pack(fill=BOTH, expand=TRUE)
        self.pack(fill=BOTH, expand=TRUE)
        self.pen = turtle.RawTurtle(self.canvas)  # create pen in canvas 
        self.penSpawn()  # spawns in top left corner

        self.min_angle = -180
        self.max_angle = 180

        self.current_angle = 0

        self.trajectory = self.master_window.get_trajectory()
        if(self.trajectory == "Square"):
            self.initialise_square()

        self.penLoop([])



        
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
    
    def penLoop(self, mapped_rotations):
        """
        The operations performed by the pen on every loop.
        """
        operations_list = [
            self.move_left,
            self.move_right,
            self.move_forward,
            self.move_backward,
            self.circle_left,
            self.circle_right,
            self.rotate_left,
            self.rotate_right
        ]

        # instead of random movements, base them on mapped_rotations.
        # each rotation -pi/4 < rotation < pi/4
        # 3 rotations - need to convert to degrees

        """alpha_degrees = np.rad2deg(mapped_rotations[0])
        beta_degrees = np.rad2deg(mapped_rotations[1])
        theta_degrees = np.rad2deg(mapped_rotations[2])"""

        # use these angles in the drawing

        alpha = random.uniform(-45, 45) / 2
        beta = random.uniform(-45, 45)
        theta = random.uniform(-45, 45)

        steps = int((abs(theta) / theta) * 10)

        self.move_forward(abs(alpha))
        self.pen.setheading(beta + self.current_angle)

        # PERFORM TRAJECTORY BOUNDARY CHECK
        if(self.trajectory == "Square"):
            if(self.perform_square() == True):
                print("switch")
                self.switch_side()

    def initialise_square(self):

        self.BOUNDARY = 150
        self.ending_x = 0
        self.ending_y = 0

        self.current_angle = 90

        self.switch_side()


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

        self.min_angle = self.current_angle - 90
        self.max_angle = self.current_angle + 90
        
    

        
        self.pen.setheading(self.current_angle)



    def perform_square(self):

        if(self.changing_x):

            if(self.has_coordinate_reached_goal(self.pen.xcor(), self.ending_x, self.increasing)):
                return True
        else:
            if(self.has_coordinate_reached_goal(self.pen.ycor(), self.ending_y, self.increasing)):
                return True


        if(self.pen.xcor() < self.small_x_boundary or self.pen.xcor() > self.large_x_boundary or self.pen.ycor() < self.small_y_boundary or self.pen.ycor() > self.large_y_boundary):
            self.pen.setheading(180 - self.pen.heading()) # turn pen around

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
    
    def move_backward(self, amount):
        """Move the pen backward by the specified amount."""
        self.pen.backward(amount)
    
    def move_left(self, amount):
        """Turn the pen left and move forward by the specified amount."""
        self.pen.left(90)
        self.pen.forward(amount)
    
    def move_right(self, amount):
        """Turn the pen right and move forward by the specified amount."""
        self.pen.right(90)
        self.pen.forward(amount)
    
    def circle_left(self, amount):
        """Make a small left circle movement with the pen."""
        self.pen.circle(25, amount)
    
    def circle_right(self, amount):
        """Make a small right circle movement with the pen."""
        self.pen.circle(25, -amount)
    
    def rotate_left(self, amount):
        """Make a large left circle movement with the pen."""
        self.pen.circle(250, amount)
    
    def rotate_right(self, amount):
        """Make a large right circle movement with the pen."""
        self.pen.circle(250, -amount)
