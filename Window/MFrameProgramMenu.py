from tkinter import *
from Window import *

class FrameProgramMenu(FrameBase):
    """
    A frame which displays the components of the virtual menu to the screen.
    """

    def __init__(self, master, main_window, **args):
        self.master = master
        self.main_window = main_window

        super().__init__(master, main_window)

        # Disable the frame from resizing.
        self.grid_propagate(0)

        # Create a bottom frame for the subframes.
        self.frames["bottom_frame"] = Frame(self)

        # Create a right frame for visual_frame.
        self.frames["right_frame"] = Frame(self.frames["bottom_frame"])

        # If the user is running the virtual frame,
        if args["type"] == "virtual":
            # Set the visual frame to the simulation.
            self.frames["visual_frame"] = FrameWaitingScreen(self.frames["right_frame"], self.main_window, self, **args)

        # If the user is running the physical program,
        else:
            # Set the visual frame to the physical program menu.
            self.frames["visual_frame"] = FramePhysicalMenu(self.frames["right_frame"], self.main_window)

        # Initialize the CSV frame as a list of possible CSVs.
        self.frames["csv_frame"] = FrameCSVList(master, main_window, self)

        # Initialize the filters frame.
        self.frames["filters_frame"] = FrameFiltersWindow(master, main_window)


        self.frames["bottom_frame"].pack(fill="both", expand=True)

        # Create a left frame for csv_frame and filters_frame.
        self.frames["left_frame"] = Frame(self.frames["bottom_frame"])
        self.frames["left_frame"].pack(side="left", fill="both", expand=True)

        # Configure grid layout for left_frame.
        self.frames["left_frame"].grid_columnconfigure(0, weight=1)
        self.frames["left_frame"].grid_rowconfigure(0, weight=1)

        # Place csv_frame and filters_frame inside the left frame using grid.
        self.frames["csv_frame"].grid(in_ = self.frames["left_frame"], row=0, column=0, sticky="nsew")
        self.frames["filters_frame"].grid(in_ = self.frames["left_frame"],row=1, column=0, sticky="nsew")

        # Configure row and column weights for left_frame to distribute space evenly
        self.frames["left_frame"].rowconfigure(0, weight=1)
        self.frames["left_frame"].rowconfigure(1, weight=1)
        self.frames["left_frame"].columnconfigure(0, weight=1)

        
        self.frames["right_frame"].pack(side="left", fill="both", expand=True)


        # Place visual_frame inside the right frame.
        self.frames["visual_frame"].pack(fill="both", expand=True)

        self.pack(fill="both", expand=True)

    def signal_done(self):
        print("RESET BUFFER")
        self.frames["csv_frame"].reset_buffer()

    def change_csv_frame(self, frame_name, **args):
        """
        Change the currently hosted CSV frame.
        """

        # Retrieve the frame to be changed.
        frame = self.main_window.names_to_frames.get(frame_name)

        # Change the current frame to the new frame.
        self.frames["csv_frame"] = frame(self.master, self.main_window, self, **args)
        self.frames["csv_frame"].grid(in_=self.frames["left_frame"], row=0, column=0, sticky="nsew")

    def change_simulation_frame(self, frame_name, **args):
        """
        Change the currently hosted simulation frame.
        """

        self.frames["visual_frame"].destroy()


        # Retrieve the frame to be changed.
        frame = self.main_window.names_to_frames.get(frame_name)

        # Change the current frame to the new frame.
        self.frames["visual_frame"] = frame(self.frames["right_frame"], self.main_window, self, **args)
        self.frames["visual_frame"].pack(in_=self.frames["right_frame"],fill="both", expand=True)

    def get_trajectory(self):
        return self.frames["filters_frame"].selected_item

        
        



