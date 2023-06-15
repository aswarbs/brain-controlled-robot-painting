from Window import *
from tkinter import *

class FrameSettings(FrameBase):
    """
    A frame to allow the user to configure settings.
    """
    def __init__(self, master:Tk, main_window):
        super().__init__(master, main_window)

        self.widgets["start"] = Button(self, text = "back", command = lambda:main_window.change_frame("main menu"), **self.button_style)
        self.widgets["start"].pack()