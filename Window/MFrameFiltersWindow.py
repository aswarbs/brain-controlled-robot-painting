from Window import *
from tkinter import *

class FrameFiltersWindow(FrameBase):
    """
    A frame to allow the user to apply filters to the current data.
    """

    def __init__(self, master:Tk, main_window):
        super().__init__(master, main_window)
        self.configure(highlightthickness=1, highlightbackground="black")

        filters_label = Label(self, text='Filters Frame', **self.tertiary_label_style)
        filters_label.pack()