
import shutil
from tkinter import *
from tkinter import simpledialog
from Window import *
import csv
import os

class FrameProgramMenu(FrameBase):
    """
    A frame which displays the components of the virtual menu to the screen.
    """

    def __init__(self, master, main_window, **args):
        self.master = master
        self.main_window = main_window
        self.type = args['type']

        super().__init__(master, main_window)

        # Disable the frame from resizing.
        self.grid_propagate(0)

        # Create a bottom frame for the subframes.
        self.frames["bottom_frame"] = Frame(self)

        # Create a right frame for visual_frame.
        self.frames["right_frame"] = Frame(self.frames["bottom_frame"])

        # If the user is running the virtual frame,
        if self.type == "virtual":
            # Set the visual frame to the simulation.
            self.frames["visual_frame"] = FrameWaitingScreen(self.frames["right_frame"], self.main_window, self, **args)

        # If the user is running the physical program,
        else:
            # Set the visual frame to the physical program menu.
            self.frames["visual_frame"] = FramePhysicalMenu(self.frames["right_frame"], self.main_window, self)

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

    def allow_progression(self):
        """Allow the start button to start the simulation once a CSV has been selected."""

        if(self.type == "virtual"):
            self.frames["visual_frame"].start_button.config(bg="#42c4ee")
            self.frames["visual_frame"].start_button.config(command=lambda:self.start_simulation())
        else:
            self.frames["visual_frame"].connection_button.config(bg="#42c4ee")
            self.frames["visual_frame"].connection_button.config(command= lambda:self.frames["visual_frame"].attempt_connection())
            

    def start_simulation(self):
        self.change_csv_frame("csv display")
        self.change_simulation_frame("simulation")


    def store_csv(self, file_path):
        """
        Copy the CSV into the muse_data csv.
        """

        # Clear the new file
        with open("muse_data.csv", 'w', newline=''):
            pass
        
        # Copy data from the old file to the new file, writing only columns 2 to 5 and converting to floats
        with open(file_path, 'r') as old_file, open("muse_data.csv", 'a', newline='') as new_file:
            csv_reader = csv.reader(old_file)
            csv_writer = csv.writer(new_file)

            # Skip the first line
            next(csv_reader)
        
            
            for row in csv_reader:
                # Select columns 2 to 5 (indexes 1 to 4) from the row and convert to floats
                selected_columns = [float(value) for value in row[1:5]]
                
                csv_writer.writerow(selected_columns)

    def ask_file_dialog(self):
        user_input = simpledialog.askstring("Input", "Enter file name:", parent=self, initialvalue="")
        
        if user_input is not None:  # User clicked OK
            print("User input:", user_input)

        cwd = os.getcwd()

        # save the csv
        src_dir = cwd
        src_file = 'muse_data.csv'
        dest_dir = cwd + '/CSVs'
        # Build the destination file path
        dest_file = f"{user_input}.csv"

        # Build the source and destination file paths
        src_path = os.path.join(src_dir, src_file)
        dest_path = os.path.join(dest_dir, dest_file)

        # Copy the file and rename it
        shutil.copy(src_path, dest_path)


    def signal_done(self):
        self.frames["csv_frame"].reset_buffer()

    def handle_faces(self):
        self.frames["csv_frame"].handle_faces()
        
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

        
        



