from Window import *
from tkinter import *
from tkinter import colorchooser

class FrameFiltersWindow(FrameBase):
    """
    A frame to allow the user to apply filters to the current data.
    """

    def __init__(self, master:Tk, main_window):
        super().__init__(master, main_window)
        self.configure(highlightthickness=1, highlightbackground="black")
        self.pack_propagate(0)

        self.buttons = []
        self.colours = ["red", "yellow", "green", "blue", "purple"]

        self.display_colours()

    def display_colours(self):

        colours_frame = Frame(self)
        colours_frame.config(bg="white")

        Label(colours_frame, text="Colour Scheme:", **self.secondary_label_style).pack(side=LEFT,padx=10)

        # Create five square buttons
        for i in range(5):
            button = Button(colours_frame, width=1, height=1, bg=self.colours[i], command=lambda idx=i: self.change_color(idx), highlightthickness=1, highlightbackground="black")
            button.pack(side='left', padx=5)
            self.buttons.append(button)

        colours_frame.pack(side=TOP, expand=TRUE)


    def change_color(self,index):
        color = colorchooser.askcolor(title="Select Color")[1]
        if color:
            self.buttons[index].configure(bg=color)
            self.colours[index] = color

        print(self.colours)