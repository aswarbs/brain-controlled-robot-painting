import turtle
from Window import *
from tkinter import *

class FrameSimulation(FrameBase):
    """
    A frame to allow the user to apply a simulation of the pen's trajectory using a specified CSV.
    """

    def __init__(self, master:Tk, main_window):
        super().__init__(master, main_window)
        self.pack_propagate(0)

        

        self.configure(highlightthickness=1, highlightbackground="black")

        self.color_index = 0 # set initial colour to red
        self.colors = [ # will change upon blinking
            "red",
            "orange",
            "yellow",
            "green",
            "light blue",
            "blue",
            "purple"
        ]
        
        self.DELTA_MIN = 1
        self.DELTA_MAX = 4 # range = 4
        self.THETA_MIN = 4
        self.THETA_MAX = 8 # range = 4
        self.ALPHA_MIN = 8
        self.ALPHA_MAX = 12 # range = 4
        self.BETA_MIN = 12
        self.BETA_MAX = 30 # range = 18

        self.BETA_RANGE = self.BETA_MAX - self.BETA_MIN

        self.forward = 3 # default speed of line
        self.stop = False
        
        self.canvas = Canvas(self)

        self.canvas.pack(fill=BOTH, expand=TRUE)

        self.pack(fill=BOTH, expand=TRUE)
        self.pen = turtle.RawTurtle(self.canvas) # create pen in canvas 
        self.penSpawn() # spawns in top left corner
    

    def penStop(self):
        """
        Stops the simulation on press of the pause button.
        """
        self.stop = True # when stop button is pressed


    def penSpawn(self):
        """
        Spawns the pen to the screen in a specified position.
        """
        self.loops=0
        self.pen.clear()
        self.pen.speed(0)
        self.pen.width(2)
        self.pen.color('red')
        self.pen.penup()
        self.pen.goto(10,0) # initially the pen will start in top left
        self.pen.pendown()
        self.pen.setheading(0)
        self.current = 0
        self.dir = "DOWN"


    def oscilate(self, avg): # causes the pen to oscillate based on the delta level
        """
        Controls the current oscillation of the pen.
        """
        # delta : 1 - 4 : oscillation

        if self.dir == "UP":
            self.current = self.current + 1
            self.pen.left(3)

        elif self.dir == "DOWN":
            self.current = self.current - 1
            self.pen.right(3)

    def thickness(self, avg): # causes the pen thickness to change based on beta level
        """
        Controls the current thickness of the pen.
        """
        # beta : 12 - 30 : thickness

        percentage = (avg - self.BETA_MIN) * (100 / (self.BETA_MAX - avg))
        # convert the beta value to a percentage / 100

        if avg > self.BETA_MAX + (self.BETA_RANGE / 2): # pen thickess increases
            self.pen.width(((percentage - 60)//5) + 1)
            self.dot_timer = 0
            self.pen.pendown()

        else:
            self.pen.width(2) # line is normal
            self.dot_timer = 0
            self.pen.pendown()


    def penBlink(self): # pen changes colour on blink
        """
        Controls the current colour of the pen.
        """
        self.color_index =  self.color_index + 1 # goes to next colour in rainbow
        self.pen.color(self.colors[self.color_index % len(self.colors)])

    def penLoop(self, avg, blink):
        """
        The operations performed by the pen on every loop.
        """
        # default values

        self.pen.forward(self.forward)


        if(avg <= self.BETA_MAX and avg > self.BETA_MIN):
            self.thickness(avg)
            return False
        
        elif(avg <= self.DELTA_MAX and avg > self.DELTA_MIN):
            self.oscilate(avg)
            return False

        elif(avg <= self.THETA_MAX and avg > self.THETA_MIN):
            self.pen.right((avg - self.THETA_MIN) * (90 / (self.THETA_MAX - avg))) 
            # converts the value into a degree from 0 to 90
            return False
        
        elif(avg <= self.ALPHA_MAX and avg > self.ALPHA_MIN):
            self.pen.left((avg - self.ALPHA_MIN) * (90 / (self.ALPHA_MAX - avg))) 
            # converts the value into a degree from 0 to 90
            return False

        elif((avg > 70 or avg< -70) and blink == False):
            self.penBlink()
            return True
        
        return False

            
        

            


