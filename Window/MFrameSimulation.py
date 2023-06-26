import turtle
from tkinter import *
import random
from math import cos, sin, pi
from Window import *

class FrameSimulation(FrameBase):
    """
    A frame to allow the user to apply a simulation of the pen's trajectory using a specified CSV.
    """
    def __init__(self, master: Tk, main_window, parent_window, **args):
        super().__init__(master, main_window)
        self.pack_propagate(0)
        self.configure(highlightthickness=1, highlightbackground="black")
        
        self.color_index = 0 
        self.colors = parent_window.frames["filters_frame"].colours
        
        self.forward = 3  # default speed of line
        self.stop = False
        
        self.canvas = Canvas(self)
        self.canvas.pack(fill=BOTH, expand=TRUE)
        
        self.pack(fill=BOTH, expand=TRUE)
        self.pen = turtle.RawTurtle(self.canvas)  # create pen in canvas 
        self.penSpawn()  # spawns in top left corner
        
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
        
        amount = random.uniform(0, 360 / 4)  # get a random amount to do the operation
        operation_index = random.randint(0, len(operations_list) - 1)  # get a random operation
        operation = operations_list[operation_index]
        operation(amount)  # perform the operation
    
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
