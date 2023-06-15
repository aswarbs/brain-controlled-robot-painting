from Window import *
from tkinter import *

class FrameAccountMenu(FrameBase):
    """
    Class to display the Account Menu.
    Contains functionality to edit the account.
    """

    def __init__(self, master:Tk, main_window):
        super().__init__(master, main_window)

        self.widgets["start"] = Button(self, text = "back", command = lambda:main_window.change_frame("main menu"), **self.button_style)
        self.widgets["start"].pack()