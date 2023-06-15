from Window import *
from tkinter import *

class FrameProgramMenu(FrameBase):
    """
    A frame which displays the components of the virtual menu to the screen.
    """

    def __init__(self, master:Tk, main_window, **args):

        # Retrieve whether the user is running the virtual or physical program.
        self.type = args["type"]
        
        self.master = master
        self.main_window = main_window

        # Initialise the current frame.
        super().__init__(master, main_window)

        # Disable the frame from resizing.
        self.grid_propagate(0)



        # Initialise the CSV frame as a list of possible CSVs.
        self.frames["csv_frame"] = FrameCSVList(master, main_window, self)    

        # Initialise the filters frame.
        self.frames["filters_frame"] = FrameFiltersWindow(master, main_window)

        # Create a bottom frame for the subframes
        self.frames["bottom_frame"] = Frame(self)
        self.frames["bottom_frame"].pack(fill="both", expand=True)
        
        # Create a left frame for csv_frame and filters_frame
        self.frames["left_frame"] = Frame(self.frames["bottom_frame"])
        self.frames["left_frame"].pack(side="left", fill="both", expand=True)

        # Configure grid layout for left_frame
        self.frames["left_frame"].grid_columnconfigure(0, weight=1)
        self.frames["left_frame"].grid_rowconfigure(0, weight=1)


        # Place csv_frame and filters_frame inside the left frame using grid
        self.frames["csv_frame"].grid(in_=self.frames["left_frame"], row=0, column=0, sticky="nsew")
        self.frames["filters_frame"].grid(in_=self.frames["left_frame"], row=1, column=0,pady=120)

        # Create a right frame for visual_frame
        self.frames["right_frame"] = Frame(self.frames["bottom_frame"])
        self.frames["right_frame"].pack(side="left", fill="both", expand=True)

        # If the user is running the virtual frame,
        if(self.type == "virtual"):
            # Set the visual frame to the simulation.
            self.frames["visual_frame"] = FrameSimulation(master, main_window)

        # If the user is running the physical program,
        else:
            # Set the visual frame to the physical program menu.
            self.frames["visual_frame"] = FramePhysicalMenu(master, main_window)
    

        # Place visual_frame inside the right frame
        self.frames["visual_frame"].pack(in_=self.frames["right_frame"], fill="both", expand=True)

        self.pack(fill="both", expand=True)

    def change_csv_frame(self, frame_name, **args):
        """
        Change the currently hosted CSV frmame.
        """

        # Retrieve the frame to be changed.
        frame = self.main_window.names_to_frames.get(frame_name)

        # Change the current frame to the new frame.
        self.frames["csv_frame"] = frame(self.master, self.main_window, self, **args)
        self.frames["csv_frame"].grid(in_=self.frames["left_frame"], row=0, column=0, sticky="nsew")
        
        



