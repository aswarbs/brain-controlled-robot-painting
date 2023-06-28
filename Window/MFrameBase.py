from Window import *
from tkinter import *

class FrameBase(Frame):
    """
    A class which all frames extend.
    Contains an initializer and styling definitions
    to maintain a constant style throughout the app.
    """

    # Constructor
    def __init__(self: "FrameBase", master: Tk, parent_window) -> None:

        # Initialise global styles for tkinter objects
        self.init_styles()

        # Calls the Tk Frame Constructor
        super().__init__(master)

        # All widgets should be stored in the self.widgets dictionary
        self.widgets = dict()

        # All frames should be stored in the self.frames dictionary
        self.frames = dict()

        # Set the background of every frame to white
        self.config(bg="white")

        # The window which holds the frames (MHostWindow)
        self.parent_window = parent_window
        


        


    def init_styles(self: "FrameBase") -> None:
        """
        Initialise the styles used throughout the program.
        """

        # Define Button Styles
        self.button_style = {
            "font": ("Bahnschrift Light", 12), # Font and Size
            "bg": "#42c4ee", # Background Colour, light blue
            "fg": "black", # Foreground Colour
            "width": 22, # Button Width
            "height": 2, # Button Height
            "bd": 1, # Border Width
            "highlightthickness": 0, # Thickness of highlight when button is pressed
            "relief": "ridge", # Button animation when pressed
        }

        self.small_button_style = {
            "font": ("Bahnschrift Light", 10), # Font and Size
            "bg": "#EEEEEE", # Background Colour, light grey
            "fg": "black", # Foreground Colour
            "width": 15, # Button Width
            "height": 1, # Button Height
            "bd": 1, # Border Width
            "highlightthickness": 0, # Thickness of highlight when button is pressed
            "relief": "ridge", # Button animation when pressed
        }



        # Define Label Styles
        self.title_label_style = {
            "font":("Bahnschrift", 26, "bold"), # Font, Size and Effects
            "fg":"#3C79BA", # Foreground Colour, dark blue
            "bg":"white", # Background Colour: white
            
        }

        self.secondary_label_style = {
            "font":("Bahnschrift Light", 14), # Font and Size
            "fg":"#424242", # Foreground Colour, grey
            "bg":"white", # Background Colour
        }

        self.tertiary_label_style = {
            "font": ("Bahnschrift Light", 9), # Font and Size
            "fg":"#343434", # Foreground Colour, grey
            "bg":"white", # Background Colour
            
        }


        # Define Image Styles

        self.image_style = {
            "bg":"white", # Background Colour
            "bd":0 # No Image Border
        }



        # Define Scrollbar Styles
        self.scrollbar_style = {
            "background": "#EAEAEA",  # Background color
            "troughcolor": "#D0D0D0",  # Scrollbar trough color
            
        }

        self.dropdown_style = {
            "bg":'white', # background colour
            "fg":'black', # foreground colour
            "activebackground":'gray', # active bg colour
            "activeforeground":'white', # active fg colour
            "relief":'solid' # button style
        }



    def forget_frame(self: "FrameBase") -> None:
        """
        Forget all subframes in a frame, then forget the frame itself.
        """
        for x in self.frames.values():
            x.forget()
        self.forget()

    
   


        
