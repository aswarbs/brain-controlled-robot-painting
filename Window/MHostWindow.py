from Window import *
from tkinter import *

class HostWindow:
    """
    The window which all frames are displayed on.
    """

    # Constructor
    def __init__(self):

        self.init_frame_dictionary()

        # Initiating the Tk Instance
        self.master = Tk()
        self.master.attributes("-fullscreen", True)
        self.master.configure(background="white")  
        self.master.title("Brain Controlled Robot Painting")
        self.frame = FrameMainMenu(self.master, self)
        self.frame.pack()


    def init_frame_dictionary(self):
        """
        Initialise a dictionary converting classes to Strings to avoid circular imports.
        """
        self.names_to_frames = {
        "main menu":FrameMainMenu,
        "program menu": FrameProgramMenu,
        "settings": FrameSettings,
        "account menu": FrameAccountMenu,
        "csv list": FrameCSVList,
        "csv display": FrameCSVDisplay,
        "connect to sensor": FrameConnectToSensor,
        }

    def change_frame(self, new_frame_name, **args):
        """
        Switches the currently hosted frame.
        """
        self.frame.forget_frame()
        frame = self.names_to_frames.get(new_frame_name)
        self.frame = frame(self.master, self, **args)
        self.frame.pack()





        