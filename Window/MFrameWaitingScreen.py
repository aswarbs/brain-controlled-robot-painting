from Window import *
from tkinter import *
import csv

class FrameWaitingScreen(FrameBase):

    def __init__(self, master: Tk, main_window, parent_frame, **args):
        self.parent_frame = parent_frame
        self.main_window = main_window
        super().__init__(master, main_window)
        self.configure(highlightthickness=1, highlightbackground="black")
        self.pack_propagate(0)

        self.display_screen()

    def display_screen(self):

        Label(self, text="Visual Demonstration", **self.title_label_style).pack(anchor=N, pady=10)

        Label(self,text="Ensure you have chosen a CSV.", **self.secondary_label_style).pack()
        Label(self,text="You can also choose colour schemes and trajectories for the pen to draw!", **self.secondary_label_style).pack()

        buttons_frame = Frame(self)
        buttons_frame.config(bg="white")

        Button(buttons_frame,text="Back", command=lambda:self.main_window.change_frame("main menu"), **self.button_style).pack(side=LEFT, expand=TRUE, pady=10)


        self.start_button = Button(buttons_frame,text="Start", **self.button_style)
        self.start_button.config(bg="#D3D3D3")
        self.start_button.pack(side=LEFT, expand=TRUE, pady=10)


        buttons_frame.pack(side=BOTTOM, fill=X)

    def start_simulation(self):
        self.parent_frame.change_simulation_frame("simulation")
        self.parent_frame.change_csv_frame("csv display")


    def allow_progression(self):
        """Allow the start button to start the simulation once a CSV has been selected."""
        self.start_button.config(bg="#42c4ee")
        self.start_button.config(command=lambda:self.start_simulation())

        



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
                selected_columns = [float(value) for value in row[0:5]]
                
                csv_writer.writerow(selected_columns)
