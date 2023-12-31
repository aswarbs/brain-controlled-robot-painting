from Window import *
from tkinter import *
import csv

class FrameWaitingScreen(FrameBase):

    def __init__(self, master: Tk, main_window, parent_frame, **args):
        self.parent_frame = parent_frame
        self.main_window = main_window
        super().__init__(master, main_window)
        self.configure(highlightthickness=1, highlightbackground="black")
        self.pack_propagate(0)

        self.display_screen()

    def display_screen(self):

        Label(self, text="Visual Demonstration", **self.title_label_style).pack(anchor=N, pady=10)

        Label(self,text="Ensure you have chosen a CSV.", **self.secondary_label_style).pack()
        Label(self,text="You can also choose colour schemes and trajectories for the pen to draw!", **self.secondary_label_style).pack()

        buttons_frame = Frame(self)
        buttons_frame.config(bg="white")

        Button(buttons_frame,text="Back", command=lambda:self.main_window.change_frame("main menu"), **self.button_style).pack(side=LEFT, expand=TRUE, pady=10)


        self.start_button = Button(buttons_frame,text="Start", **self.button_style)
        self.start_button.config(bg="#D3D3D3")
        self.start_button.pack(side=LEFT, expand=TRUE, pady=10)


        buttons_frame.pack(side=BOTTOM, fill=X)

    


    

        



