from Window import *
from tkinter import *

from threading import Thread
import time
import csv
import os
import shutil
from datetime import datetime
import mne
import numpy as np
from mne.time_frequency import psd_array_welch
import matplotlib as plt
from matplotlib.animation import FuncAnimation

# TODO: DETECT MOTION ARTIFACTS.. IF MIN < 0 OR MAX > 40? IN EPOCH ?

class FrameCSVDisplay(FrameBase):
    """
    Display a graph of the chosen CSV to the screen.
    """


    def __init__(self, master:Tk, main_window, parent_frame):

        self.SAMPLING_RATE = 256
        self.BUFFER_SIZE = 2 * self.SAMPLING_RATE
        
        # Initialise the global variable which tracks the instance of Program Menu this frame is contained in.
        self.frame = parent_frame


        
        # Retrieve the instance of the simulation frame where the brain data will be displayed.
        self.simulation_frame = parent_frame.frames["visual_frame"]


        # Initialise parameters relating to the drawing.
        self.play_animation = None
        self.blink = False
        self.back=False
        self.destroy = False
        



        


        # Initialise the current frame instance.
        super().__init__(master, main_window)

        # Disable resizing of the current frame.
        self.pack_propagate(0)

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

        self.psds = []

        # Create a thread to draw the graph of brain data to the screen.
        self.graph_thread = Thread(target=self.handle_graph)
        self.graph_thread.start()



    def handle_graph(self):
        self.buffer = []
        has_started_drawing = False

        with open('muse_data.csv', 'r') as file:
            csv_reader = csv.reader(file)

            while (self.destroy == False and self.back == False):

                try:
                    current_row = next(csv_reader)

                    # Convert the elements to float
                    current_row = [float(value) for value in current_row[1:]]



                    self.buffer.append(current_row)

                    # Increment the position of the slider and the current row to be accessed.
                    self.plus_csv(1)

                    self.plot_graph(current_row)


                    # This check allows the pen to begin drawing for the first time.
                    # Otherwise, reset_buffer is called in MFrameProgramMenu, once the pen
                    # has finished the current line.
                    if(len(self.buffer) > self.BUFFER_SIZE and has_started_drawing == False):
                        has_started_drawing = True
                        self.reset_buffer()

                    time.sleep(0.01)

                except:
                    self.plot_psd()
                    time.sleep(0.01)

    def plot_psd(self):
        # Transpose the data to separate x, y1, y2, and y3 values
        data_transposed = list(zip(*self.psds))

        # Extract the x values and three sets of y values
        x = data_transposed[0]
        y1, y2, y3 = data_transposed[1:]

        # Create a figure and axis
        plt.figure()

        # Plot the lines
        plt.plot(x, y1, label='Line 1')
        plt.plot(x, y2, label='Line 2')
        plt.plot(x, y3, label='Line 3')

        # Add labels and title
        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')
        plt.title('Three Lines from the Array')

        # Add a legend
        plt.legend()

        # Show the plot
        plt.show()

    def reset_buffer(self):
        """ once the pen has stopped drawing, send the next command and reset the buffer """

        # resize the buffer 
        self.buffer = self.buffer[-self.BUFFER_SIZE:] 

        map = self.calculate_psd(self.buffer)

        self.write_line_to_csv(map)


    def write_line_to_csv(self, line):

        with open("mapped_rotations.csv", 'a', newline='') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(line)

    def calculate_psd(self, data):


        # Recorded frequency ranges to be used in the drawing
        # 'Other' covers everything not covered in the previous three bands. Regarded as noise.
        freq_ranges = {'alpha': (8, 12), 'beta': (13, 30), 'other': (1,40)}

        # Convert data to numpy array

        data_array = np.array(data)
        data_array = data_array[:, 1:-1]
        

        # Transpose data array to be in shape (num_channels, buffer_len) = (4,512)
        # Collect data between frequencies 1 and 40 to eliminate some noise
        psd, freqs = psd_array_welch(data_array.T, sfreq=self.SAMPLING_RATE, fmin=1, fmax=40, n_fft=256)

        # Extract PSD values for each channel and frequency range
        freq_band_psd = {}
        for freq_band, (fmin, fmax) in freq_ranges.items():
            freq_mask = (freqs >= fmin) & (freqs <= fmax)

            # Selects only the PSD values that correspond to the frequency range defined by fmin and fmax
            band_rows = psd[:,freq_mask]


            # Calculate the mean psd value for each channel
            mean_psd_values_per_channel = np.mean(band_rows, axis=1)

            print(mean_psd_values_per_channel)

            # Calculate the mean of the psd from each channel, resulting in a single value.
            mean_psd_value = np.mean(mean_psd_values_per_channel)

            freq_band_psd[freq_band] = mean_psd_value

        print(freq_band_psd)

        map = self.calculate_psd_percentage(freq_band_psd)


        return map




    def calculate_psd_percentage(self, freq_band_psd):
        """ Map the frequency bands to floats between 0 and 1. """
        
        mapped_values = {}

        # Sum the values of each frequency band
        summed_values = sum(freq_band_psd.values())

        # For the power spectrum data, exclude the noise in the calculation.
        mapped_values['alpha'] = freq_band_psd['alpha'] / (summed_values - freq_band_psd['other'])

        # For the power spectrum data, exclude the noise in the calculation.
        mapped_values['beta'] = freq_band_psd['beta'] / (summed_values - freq_band_psd['other'])

        # To calculate the percentage of noise in the buffer, divide the noise by the total power.
        mapped_values['other'] = freq_band_psd['other'] / summed_values

        self.psds.append(mapped_values.values())

        print(mapped_values)

        # Return the mapped values as a list
        return [mapped_values[freq_band] for freq_band in ['alpha', 'beta', 'other']]





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

        self.play_image = PhotoImage(file='./Assets/play.png')
        Button(self.buttons_frame, image=self.play_image, command=lambda:self.play_csv(), **self.image_style).pack(side=LEFT)

        self.pause_image = PhotoImage(file='./Assets/pause.png')
        Button(self.buttons_frame, image=self.pause_image, command=lambda:self.pause_csv(), **self.image_style).pack(side=LEFT)


        # Create a back button which allows the user to navigate the the list of CSVs.
        self.back_button = Button(self.buttons_frame, text="Back", **self.button_style, command=lambda:self.go_back())
        self.back_button.pack(side="left", fill="both", expand=True)
        

        

        self.buttons_frame.pack(side="bottom", fill="x")

    def play_csv(self):
        self.back=False
        pass

    def pause_csv(self):
        self.back = True
        pass

    def go_back(self):
        # reset simulation
        self.destroy=True

        
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

        self.frame.change_simulation_frame("waiting screen")
        self.frame.change_csv_frame("csv list")
        
        


    


    def plus_csv(self, amount):
        """
        Increments the current position of the CSV by a specified amount.
        """

        # Update the position of the vertical line
        x = self.canvas.coords(self.vertical_line)[0] + amount
        self.canvas.coords(self.vertical_line, x, -50, x, 500)



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

