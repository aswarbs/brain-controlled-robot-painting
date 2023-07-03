
from Window import *
from tkinter import *
import os

class FrameMainMenu(FrameBase):
    """ 
    A class containing all functionality for the Main Menu of the program.
    Draws all elements of the menu to the screen and describes their functionality.
    """

    def __init__(self, master:Tk, main_window): 

        # Initialise the parent class
        super().__init__(master, main_window)

        for x in range(0, 7):
            self.rowconfigure(x, weight=1)

        for x in range(0,5):
            self.columnconfigure(x,weight=1)

        self.rowconfigure(5,weight=0)


        # Create account logo
        print(os.getcwd())
        self.account_logo = PhotoImage(file='./Assets/account_logo.png')
        # Create account button
        self.widgets["account_button"] = Button(self, image=self.account_logo, **self.image_style, command = lambda:main_window.change_frame("account menu")).grid(row=0, column=0,pady=10, padx=10, sticky=NW)

        # Create settings logo
        self.settings_logo = PhotoImage(file='./Assets/settings.png')
        # Create settings button
        self.widgets["settings_button"] = Button(self, image=self.settings_logo, **self.image_style, command = lambda:main_window.change_frame("settings")).grid(row=0, column=4,pady=10, padx=10, sticky = NE)

        # Create project logo
        self.logo = PhotoImage(file='./Assets/logo.png')
        # Add logo to label
        self.widgets["logo"] = Label(self, image=self.logo, **self.image_style).grid(row=1, column=1, rowspan = 2)

        # Create BCRP title
        self.widgets["title"] = Label(self, justify=LEFT, text="Brain Controlled Robot Painting", **self.title_label_style)
        # Format title on screen
        self.widgets["title"].grid(row = 1, column= 2, columnspan=2, sticky = S, pady=10)

        # Create a description label
        self.widgets["description_label"] = Label(self, justify=LEFT,text="Brain Controlled Robot Painting is a project which allows \nthe user to record their brain activity and generate a \nunique painting from the data.", **self.secondary_label_style)
        # Format description on screen
        self.widgets["description_label"].grid(row=2, column=2, columnspan=2, sticky = N, pady=10)

        # Create image for virtual program button
        self.virtual_image = PhotoImage(file='./Assets/virtual_logo.png')
        # Add image to label
        self.widgets["virtual_image"] = Label(self, image=self.virtual_image, **self.image_style).grid(row=4, column=1,sticky=S)

        # Create image for physical program button
        self.physical_image = PhotoImage(file='./Assets/physical_logo.png')
        # Add image to label
        self.widgets["physical_image"] = Label(self, image=self.physical_image, **self.image_style).grid(row=4, column=2,sticky=S)

        # Create image for exit program button
        self.exit_image = PhotoImage(file='./Assets/exit.png')
        # Add image to label
        self.widgets["exit_image"] = Label(self, image=self.exit_image, **self.image_style).grid(row=4, column=3, sticky=S)

        # Create virtual menu button
        self.widgets["virtual_menu"] = Button(self, text="Virtual", command=lambda:main_window.change_frame("program menu", type="virtual"), **self.button_style)
        # Format button on screen
        self.widgets["virtual_menu"].grid(row=5, column=1, pady=(20,10), sticky = S)

        # Create physical menu button
        self.widgets["physical_menu"] = Button(self, text="Physical", command= lambda:main_window.change_frame("program menu", type="physical"), **self.button_style)
        # Format button on screen
        self.widgets["physical_menu"].grid(row=5, column=2, pady=(20,10), sticky =S)

        # Create exit button
        self.widgets["exit"] = Button(self, text="Exit", command=self.quit, **self.button_style)
        # Format button on screen
        self.widgets["exit"].grid(row=5, column=3, pady=(20,10), sticky = S)

        # Create description for virtual menu
        self.widgets["virtual_description"] = Label(self, text="Use your brain activity to illustrate a\ntrajectory of the pen to the screen.", **self.tertiary_label_style)
        # Format label on screen
        self.widgets["virtual_description"].grid(row=6, column=1, sticky=N)

        # Create description for physical menu
        self.widgets["physical_description"] = Label(self, text="Deploys the UR3e arm to create a unique\nphysical image of your brain activity.", **self.tertiary_label_style)
        # Format label on screen
        self.widgets["physical_description"].grid(row=6, column=2, sticky=N)

        # Create description for exit button
        self.widgets["exit_description"] = Label(self, text="Exits the program, all CSV data will\nbe saved and available next time.", **self.tertiary_label_style)
        # Format label on screen
        self.widgets["exit_description"].grid(row=6, column=3, sticky=N)

        self.pack(fill="both", expand=True)




