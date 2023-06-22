from Window import *
from tkinter import *

from threading import Thread
import time
import csv
import os
import shutil
from datetime import datetime
import numpy as np
from scipy import signal

"""
TODO:
- This frame communicates with the simulation frame. This should be separated, so that this frame also works with the physical program.
"""


class FrameCSVDisplay(FrameBase):
    """
    Display a graph of the chosen CSV to the screen.
    """


    def __init__(self, master:Tk, main_window, parent_frame):
        
        # Initialise the global variable which tracks the instance of Program Menu this frame is contained in.
        self.frame = parent_frame
        
        # Retrieve the instance of the simulation frame where the brain data will be displayed.
        self.simulation_frame = parent_frame.frames["visual_frame"]

        # Initialise parameters relating to the drawing.
        self.play_animation = None
        self.blink = False
        self.back=False


        # Initialise the current frame instance.
        super().__init__(master, main_window)

        # Disable resizing of the current frame.
        self.pack_propagate(0)

        # Create an instance of the CSVHandler class.
        self.csv_handler = CSVHandler()

        # Create the buttons to be displayed on the frame.
        self.create_buttons()

        # Create the text to be displayed on the frame.
        self.create_text()

        # Create the canvas which will hold the graph of brain data.
        self.create_canvas()

        # Create the scrollbar allowing the user to navigate through brain data.
        self.create_scrollbar()

        # Initialise the array holding the CSV Data.
        self.data = []

        # Create a thread to draw the graph of brain data to the screen.
        self.graph_thread = Thread(target=self.handle_graph)
        self.graph_thread.start()

    def handle_graph(self):
        print("hello")
        with open('muse_data.csv', 'r') as file:    
            csv_reader = csv.reader(file)

            try:
                next(csv_reader)  # Skip header row
            except StopIteration:
                self.after(50, self.handle_graph)

            buffer = []
            

            while self.back==False:
                try:
                    while(len(buffer) < 256 and self.back==False):
                        current_row = next(csv_reader)

                        # Convert the elements to float
                        current_row = [float(value) for value in current_row[1:]]

                        buffer.append(current_row)

                        # Calculate the average of the current row of data.
                        avg = sum(current_row) / 4

                        # Increment the position of the slider and the current row to be accessed.
                        self.plus_csv(1)

                        self.plot_graph(current_row)
                        time.sleep(0.01)

                    # calculate psd from buffer

                    # Draw the next row of data. Retrieve whether the user is currently blinking.

                    thread = Thread(target=self.simulation_frame.penLoop)
                    thread.start()

                    self.calculatePSD(buffer)

                    buffer = []
                    

                except StopIteration:
                    # If there is no next row, sleep or perform other desired actions
                    time.sleep(0.1)  # Sleep for 0.1 second

                except KeyboardInterrupt:
                    print('Closing!')


    def calculatePSD(self, buffer):
                # Convert buffer to a numpy array
                buffer_array = np.array(buffer)

                # Transpose the array to have channels in columns
                buffer_array = np.transpose(buffer_array)

                # Calculate PSD for each channel
                freqs, psd = signal.welch(buffer_array, fs=256)

                # Calculate alpha, beta, and theta bands
                alpha_band = np.mean(psd[:, (freqs >= 8) & (freqs <= 12)], axis=1)
                beta_band = np.mean(psd[:, (freqs >= 12) & (freqs <= 30)], axis=1)
                theta_band = np.mean(psd[:, (freqs >= 4) & (freqs <= 8)], axis=1)

                # Print the calculated values
                print("Alpha:", np.mean(alpha_band))
                print("Beta:", np.mean(beta_band))
                print("Theta:", np.mean(theta_band))


    def create_text(self):
        """
        Draws labels depicting the fields of the CSV on the left side of the screen.
        """
        # Create the frame which will hold the labels.
        labels_frame = Frame(self, bg="white")
        labels_frame.pack(side="left", fill="y")

        # Create the TP9 label.
        label_1 = Label(labels_frame, text="TP9",**self.secondary_label_style)
        label_1.pack(anchor="w", pady=50, padx=(10,0))

        # Create the AF7 label.
        label_2 = Label(labels_frame, text="AF7", **self.secondary_label_style)
        label_2.pack(anchor="w", pady=50, padx=(10,0))

        # Create the AF8 label.
        label_3 = Label(labels_frame, text="AF8", **self.secondary_label_style)
        label_3.pack(anchor="w", pady=50, padx=(10,0))

        # Create the TP10 label.
        label_4 = Label(labels_frame, text="TP10", **self.secondary_label_style)
        label_4.pack(anchor="w", pady=50, padx=(10,0))

    def create_canvas(self):
        """
        Creates the canvas widget which will hold the plotted graph.
        """


        # Create a canvas widget and add it to the frame
        self.canvas = Canvas(self,bg="white")
        self.canvas.pack(fill="both", expand=True)

        # Draw the initial vertical line
        x = 50  # x-coordinate of the vertical line
        self.vertical_line = self.canvas.create_line(x, -50, x, 500, fill="black")

    def create_scrollbar(self):
        """
        Creates the scrollbar used to view different parts of the CSV.
        """
        # Create a scrollbar widget and add it to the frame
        self.scrollbar = Scrollbar(self, orient="horizontal", command=self.canvas.xview, **self.scrollbar_style)
        self.scrollbar.pack(fill="x")

        self.canvas.config(xscrollcommand=self.scrollbar.set, highlightthickness=0)


    
    def create_buttons(self):
        """
        Creates the buttons used to change the position of the CSV.
        """

        # Create a subframe to hold the buttons.
        self.buttons_frame = Frame(self)


        # Create a back button which allows the user to navigate the the list of CSVs.
        self.back_button = Button(self.buttons_frame, text="Back", **self.button_style, command=lambda:self.go_back())
        self.back_button.pack(side="left", fill="both", expand=True)

        self.buttons_frame.pack(side="bottom", fill="x")

    def go_back(self):
        # reset simulation
        self.back=True
        self.simulation_frame.penSpawn()
        self.graph_thread.join()

        cwd = os.getcwd()

        # save the csv
        src_dir = cwd
        src_file = 'muse_data.csv'
        dest_dir = cwd + '/CSVs'
        # Get the current date and time
        current_datetime = datetime.now()
        date_time_string = current_datetime.strftime("%Y-%m-%d_%H-%M-%S")

        # Build the destination file path
        dest_file = f"{date_time_string}.csv"

        # Build the source and destination file paths
        src_path = os.path.join(src_dir, src_file)
        dest_path = os.path.join(dest_dir, dest_file)

        # Copy the file and rename it
        shutil.copy(src_path, dest_path)

        
        self.frame.change_csv_frame("csv list")


    


    def plus_csv(self, amount):
        """
        Increments the current position of the CSV by a specified amount.
        """

        # Update the position of the vertical line
        x = self.canvas.coords(self.vertical_line)[0] + amount
        self.canvas.coords(self.vertical_line, x, -50, x, 500)


    

    def play_csv(self):
        """
        Plays the current CSV, incrementing the slider and starting the associated simulation.
        """

        self.paused=False



    def pause_csv(self):
        """
        Pauses the CSV, pausing the slider and the associated simulation.
        """

        self.paused=True
        self.simulation_frame.penStop()






    def plot_graph(self, new_data):
        """
        Plots the graph of the CSV onto the canvas.
        """

        # Add the new row of data to the list of already drawn data.
        self.data.append(new_data)

        # Determine the dimensions of the canvas
        num_rows = len(self.data[0])

        # Iterate over the new rows and create lines on the canvas
        colors = ["red", "blue", "green", "orange"]

        # Initialise the y offset to 0.
        y_offset = 0

        # If more than 1 row of data has been recorded,
        if(len(self.data) > 1):

            # For each channel of the recorded brain data,
            for row_index in range(0, num_rows):
                    
                    # Increment the y offset by 100, displaying each row as a separate line.
                    y_offset += 100

                    # Set the starting x position of the line to the current end position of the previous line.
                    x1 = 50 + len(self.data)

                    # Set the starting y position of the line to the ending y position of the previous line.
                    y1 = self.scale_value(float(self.data[-2][row_index]), 0.4)

                    # Set the ending x position of the line to the starting position + 1.
                    x2 = x1 + 1

                    # Set the ending y position of the line to the scaled value retrieved from the current channel of data.
                    y2 = self.scale_value(float(self.data[-1][row_index]), 0.4)

                    # Plot the current line on the graph. Represent each channel with a distinct colour.
                    self.canvas.create_line(x1, y1 + y_offset, x2, y2 + y_offset, fill=colors[row_index])

        # Update the scroll region of the scrollbar to include the newly plotted line.
        self.update_scroll_region()

    def scale_value(self, value, scaling_factor):
        """
        Scale the given value to fall between -100 and 100.
        """

        value *= scaling_factor

        # If the value exceeds 100, set the value to 100.
        if value > 100:
            return 100
        
        # If the value exceeds -100, set the value to -100.
        if value < -100:
            return -100
        
        # If the value is between -100 and 100, return the unchanged value.
        return value


    def update_scroll_region(self):
        """
        Update the scroll region of the scrollbar to include newly plotted lines.
        """
        
        # Set a new x value for the scrollbar to reach.
        max_x = 50 + len(self.data)

        # Set a new y value for the scrollbar to include.
        max_y = (len(self.data[0]) * 90) + 50

        # Update the scrollbar to include the newly calculated region.
        self.canvas.config(scrollregion=(0, 0, max_x, max_y))

