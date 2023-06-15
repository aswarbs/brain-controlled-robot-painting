from Window import *
from tkinter import *
from pylsl import StreamInlet, resolve_byprop
from threading import Thread
import time

"""
TODO:
- This frame communicates with the simulation frame. This should be separated, so that this frame also works with the physical program.
"""


class FrameCSVDisplay(FrameBase):
    """
    WRITE PDOC
    """


    def __init__(self, master:Tk, main_window, parent_frame, **args):
        
        # Retrieve the kwarg containing the path to be displayed to the screen.
        self.path = args["path"]

        # Initialise the global variable which tracks the instance of Program Menu this frame is contained in.
        self.frame = parent_frame
        
        # Retrieve the instance of the simulation frame where the brain data will be displayed.
        self.simulation_frame = parent_frame.frames["visual_frame"]

        # Initialize the pen on the screen which draws the simulation.
        self.simulation_frame.penSpawn()

        # Initialise parameters relating to the drawing.
        self.play_animation = None
        self.pause_animation = True
        self.blink = False


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

        # Initialise the currently accessed index of the CSV Data.
        self.loops = 1

        # Create a thread to draw the graph of brain data to the screen.
        graph_thread = Thread(target=self.handle_graph)
        graph_thread.start()


    def handle_graph(self):
        """
        Decide whether the data should be accessed in live time or from a prerecorded file.
        """

        # If the data is being recorded in live time,
        if(self.path == "muse_data.csv"):

            # Indicate that the drawing has started.
            self.play_csv()

            # Retrieve data from the sensor.
            self.connect_to_sensor()

        # If the data has been prerecorded,
        else:

            # Read the data from the specified path.
            data = self.csv_handler.read_csv(self.path)

            # For each row of data,
            for x in data:

                # Plot the current row of data.
                self.plot_graph(x)

                # Sleep, allowing other threads to run.
                time.sleep(0.01)

    

    def connect_to_sensor(self):
        """
        Retrieve data in live time from the Muse sensor.
        """

        # retrieve the currently connected LSL streams.
        streams = resolve_byprop('type', 'EEG', timeout=2)
        if len(streams) == 0:
            raise RuntimeError('Can\'t find EEG stream.')

        # Set active EEG stream to inlet and apply time correction
        self.inlet = StreamInlet(streams[0], max_chunklen=12)

        # Length of epochs used to compute the FFT (in seconds)
        EPOCH_LENGTH = 1

        # Amount of overlap between two consecutive epochs (in seconds)
        OVERLAP_LENGTH = 0.8

        # Amount to 'shift' the start of each next consecutive epoch
        SHIFT_LENGTH = EPOCH_LENGTH - OVERLAP_LENGTH

        # Get the stream info and description
        info = self.inlet.info()

        # Get sampling frequency, for muse2 sampling frequency = 256
        fs = int(info.nominal_srate())

        self.max_samples = int(SHIFT_LENGTH * fs)

        # Create a thread to get the current row of CSV data.
        thread = Thread(target=self.get_row)
        thread.start()


    def get_row(self):
        """
        Get the current row of live CSV data and plot it on the graph.
        """

        # The thread constantly loops and plots the next row of live data.
        while(True):

            # Retrieve the next row of data.
            row = self.csv_handler.read_live_data(self.inlet, self.max_samples)

            # Plot the row on the graph.
            self.plot_graph(row)


        

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


        # Retrieve the images to be displayed on the buttons.
        self.play_image = PhotoImage(file="./Assets/play.png")
        self.pause_image = PhotoImage(file="./Assets/pause.png")
        self.rewind_image = PhotoImage(file="./Assets/rewind.png")

        # Create a play button which allows the user to play the CSV from the current position.
        self.play_button = Button(self.buttons_frame, image=self.play_image, command = lambda:self.play_csv())
        self.play_button.pack(side="left", fill="both")

        # Create a pause button which allows the user to pause the CSV.
        self.pause_button = Button(self.buttons_frame, image=self.pause_image, command = lambda:self.pause_csv())
        self.pause_button.pack(side="left", fill="both")

        # Create a rewind button which sets the current position of the CSV to the start.
        self.rewind_button = Button(self.buttons_frame, image=self.rewind_image, command = lambda:self.rewind_csv())
        self.rewind_button.pack(side="left", fill="both")


        # Create a back button which allows the user to navigate the the list of CSVs.
        self.back_button = Button(self.buttons_frame, text="Back", **self.button_style, command=lambda:self.frame.change_csv_frame("csv list"))
        self.back_button.pack(side="left", fill="both", expand=True)

        self.buttons_frame.pack(side="bottom", fill="x")

    


    def plus_csv(self, amount):
        """
        Increments the current position of the CSV by a specified amount.
        """

        # Update the position of the vertical line
        x = self.canvas.coords(self.vertical_line)[0] + amount
        self.canvas.coords(self.vertical_line, x, -50, x, 500)

        # Increment the current row of the CSV to be drawn.
        self.loops += amount




    def rewind_csv(self):
        """
        Rewinds the position of the CSV to the start.
        """

        # Update the position of the vertical line
        x = 50
        self.canvas.coords(self.vertical_line, x, -50, x, 500)

        self.loops=1

        # reset simulation
        self.simulation_frame.penSpawn()

    

    def play_csv(self):
        """
        Plays the current CSV, incrementing the slider and starting the associated simulation.
        """
        if self.pause_animation:
            self.pause_animation = False
            self.animate_csv()



    def pause_csv(self):
        """
        Pauses the CSV, pausing the slider and the associated simulation.
        """
        self.pause_animation = True
        self.simulation_frame.penStop()

    def animate_csv(self):
        """
        Visually increments the slider on the screen.
        """

        # If the current row to be accessed does not exist yet,
        if(self.loops >= len(self.data)):
           
           # Wait for the row to be accessed.
           self.after(10,self.animate_csv)
           return

        # Calculate the average of the current row of data.
        avg = sum(self.data[self.loops]) / 4

        # Draw the next row of data. Retrieve whether the user is currently blinking.
        self.blink = self.simulation_frame.penLoop(avg, self.blink)
        
        # Increment the position of the slider and the current row to be accessed.
        self.plus_csv(1)

        # Repeat the animation if not paused.
        if not self.pause_animation:
            self.after(10,self.animate_csv)



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
                    y1 = self.scale_value(float(self.data[-2][row_index]) * 0.4)

                    # Set the ending x position of the line to the starting position + 1.
                    x2 = x1 + 1

                    # Set the ending y position of the line to the scaled value retrieved from the current channel of data.
                    y2 = self.scale_value(float(self.data[-1][row_index]) * 0.4)

                    # Plot the current line on the graph. Represent each channel with a distinct colour.
                    self.canvas.create_line(x1, y1 + y_offset, x2, y2 + y_offset, fill=colors[row_index])

        # Update the scroll region of the scrollbar to include the newly plotted line.
        self.update_scroll_region()

    def scale_value(value):
        """
        Scale the given value to fall between -100 and 100.
        """

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


    def change_frame(self,frame):
        """
        Change the currently hosted frame. Delete all active threads. Reset the pen on the simulation.
        """

        # Reset the position of the pen and delete the line currently drawn.
        self.simulation_frame.penSpawn()

        # Get a list of active threads
        active_threads = enumerate()

        # End all active threads.
        for thread in active_threads:
            if(thread.name != "MainThread"):
                thread.join()

        # Change frame to the specified frame.
        self.visual_window.change_csv_frame(frame)
