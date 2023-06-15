from Window import *
from tkinter import *

from pylsl import StreamInlet, resolve_byprop

from threading import Thread
import time


class FrameCSVDisplay(FrameBase):
    """
    A frame which displays the current CSV to the screen.
    """
    def __init__(self, master:Tk, main_window, parent_frame, **args):
        
        
        self.path = args["path"]
        self.frame = parent_frame
        
        self.simulation_frame = parent_frame.frames["visual_frame"]
        self.simulation_frame.penSpawn()

        self.play_animation = None
        self.pause_animation = True
        self.blink = False

        super().__init__(master, main_window)

        self.pack_propagate(0)

        self.csv_handler = CSVHandler()

        self.create_buttons()

        self.create_text()

        self.create_canvas()

        self.create_scrollbar()

        
        self.data = []
        self.loops = 1

        graph_thread = Thread(target=self.handle_graph)
        graph_thread.start()


    def handle_graph(self):
        if(self.path == "muse_data.csv"):
            self.play_csv()
            self.connect_to_sensor()

        else:
            data = self.csv_handler.read_csv(self.path)
            for x in data:
                self.plot_graph(x)
                time.sleep(0.01)

    def plot_graph_wrapper(self, x):
        self.plot_graph(x)

    def connect_to_sensor(self):

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


        thread = Thread(target=self.get_row)
        thread.start()


    def get_row(self):
        while(True):
            row = self.csv_handler.read_live_data(self.inlet, self.max_samples)
            self.plot_graph(row)


        

    def create_text(self):
        """
        Draws labels depicting the fields of the CSV on the left side of the screen.
        """
        # Create the labels frame
        labels_frame = Frame(self, bg="white")
        labels_frame.pack(side="left", fill="y")

        # Create the labels
        label_1 = Label(labels_frame, text="TP9",**self.secondary_label_style)
        label_1.pack(anchor="w", pady=50, padx=(10,0))

        label_2 = Label(labels_frame, text="AF7", **self.secondary_label_style)
        label_2.pack(anchor="w", pady=50, padx=(10,0))

        label_3 = Label(labels_frame, text="AF8", **self.secondary_label_style)
        label_3.pack(anchor="w", pady=50, padx=(10,0))

        label_4 = Label(labels_frame, text="TP10", **self.secondary_label_style)
        label_4.pack(anchor="w", pady=50, padx=(10,0))
        pass

    def create_canvas(self):
        """
        Creates the canvas widget which will hold the CSV.
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

        self.buttons_frame = Frame(self)


        # Create buttons
        self.play_image = PhotoImage(file="./Assets/play.png")
        self.pause_image = PhotoImage(file="./Assets/pause.png")
        self.rewind_image = PhotoImage(file="./Assets/rewind.png")


        self.back_button = Button(self.buttons_frame, text="Back", **self.button_style, command=lambda:self.frame.change_csv_frame("csv list"))
        self.play_button = Button(self.buttons_frame, image=self.play_image, command = lambda:self.play_csv())
        self.pause_button = Button(self.buttons_frame, image=self.pause_image, command = lambda:self.pause_csv())
        self.rewind_button = Button(self.buttons_frame, image=self.rewind_image, command = lambda:self.rewind_csv())

        # Pack buttons to the right
        self.play_button.pack(side="left", fill="both")
        self.pause_button.pack(side="left", fill="both")
        self.rewind_button.pack(side="left", fill="both")
        self.back_button.pack(side="left", fill="both", expand=True)

        self.buttons_frame.pack(side="bottom", fill="x")

    


    def plus_csv(self, amount):
        """
        Increments the current position of the CSV by a specified amount.
        """

        # Update the position of the vertical line
        x = self.canvas.coords(self.vertical_line)[0] + amount
        self.canvas.coords(self.vertical_line, x, -50, x, 500)
        self.loops += amount




    def rewind_csv(self):
        """
        Rewinds the position of the CSV by a specified amount.
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

        if(self.loops >= len(self.data)):
           self.after(10,self.animate_csv)
           return

        avg = sum(self.data[self.loops]) / 4

        self.blink = self.simulation_frame.penLoop(avg, self.blink)
        
        self.plus_csv(1)

        # Repeat the animation if not paused
        if not self.pause_animation:
            self.after(10,self.animate_csv)



    def plot_graph(self, new_data):
        """
        Plots the graph of the CSV onto the canvas.
        """

        # make this a thread

        self.data.append(new_data)

        # Determine the dimensions of the canvas
        num_rows = len(self.data[0])

        # Iterate over the new rows and create lines on the canvas
        colors = ["red", "blue", "green", "orange"]
        y_offset = 0
        if(len(self.data) > 1):
            for row_index in range(0, num_rows):
                    y_offset += 100

                    x1 = 50 + len(self.data)
                    y1 = (float(self.data[-2][row_index]) * 0.4)  # Scale the y-value
                    if(y1 > 100):
                        y1 = 100
                    if(y1 < -100):
                        y1 = -100
                    x2 = 50 + len(self.data) + 1
                    y2 = (float(self.data[-1][row_index]) * 0.4)  # Scale the y-value
                    if(y2 > 100):
                        y2 = 100
                    if(y2 < -100):
                        y2 = -100
                    self.canvas.create_line(x1, y1 + y_offset, x2, y2 + y_offset, fill=colors[row_index])

                    

        self.update_scroll_region()

    def update_scroll_region(self):
        
        max_x = 50 + len(self.data)
        max_y = (len(self.data[0]) * 90) + 50
        self.canvas.config(scrollregion=(0, 0, max_x, max_y))

    def change_frame(self, frame):

        self.simulation_frame.penSpawn()

        # Get a list of active threads
        active_threads = enumerate()

        # Print the active threads
        for thread in active_threads:
            if(thread.name != "MainThread"):
                thread.join()

        # change frame
        self.visual_window.change_csv_frame(frame)
