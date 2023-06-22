from Window import *
from tkinter import *
from muselsl import list_muses
from threading import *
import subprocess
from pylsl import StreamInlet, resolve_byprop
import csv
import numpy as np
import time

class FrameConnectToSensor(FrameBase):
    """
    Allows the user to list available Muse sensors using the muselsl list command.

    Allows the user to connect to a sensor using the Mac Address.
    """

    def __init__(self, master:Tk, main_window, visual_window):

        # Initialise the Frame
        self.visual_window = visual_window
        super().__init__(master, main_window)
        self.pack_propagate(0)

        # Allow the frame to fill empty space
        self.columnconfigure(0,weight=1)
        self.rowconfigure(0, weight=1)

        # Initialise the Buttons and Labels arrays
        self.buttons = []
        self.labels = []

        # Create the frame to connect to the Muse sensor
        self.create_frame()


    def create_frame(self):
        """
        Create the frame which allows the user to connect to the Muse sensor.
        """

        # Create the canvas which will hold the widgets on the screen
        self.canvas = Canvas(self,bg="white",borderwidth=0,highlightthickness=0)
        self.canvas.pack(fill=BOTH, expand=True)

        # Create a frame inside the canvas to hold the buttons
        self.frame = Frame(self.canvas,bg="white")

        # Create a scrollbar and associate it with the canvas
        scrollbar = Scrollbar(self.canvas, orient=VERTICAL, command=self.canvas.yview, **self.scrollbar_style)
        scrollbar.pack(side=RIGHT, fill=Y)

        # Configure the canvas to scroll vertically
        self.canvas.configure(yscrollcommand=scrollbar.set)
        self.canvas.create_window((0, 0), window=self.frame, anchor=NW)

        # Display the options on the bottom of the screen
        self.display_options()

        # Refresh the available muse sensors
        self.refresh()


    def write_live_data(self):
        """
        Read live data from the Brain Sensor, row by row.
        Return the current row of brain data from the sensor.
        """

        # Clear existing data in the muse_data.csv file
        with open("muse_data.csv", mode="w", newline=""):
            pass

        # Connecting to an EEG Stream
        print('Looking for an EEG stream...\n\n\n')
        streams = resolve_byprop('type', 'EEG', timeout=2)
        if len(streams) == 0:
            raise RuntimeError('Can\'t find EEG stream.')
                
        # Set active EEG stream to inlet and apply time correction
        print('Start acquiring data')
        inlet = StreamInlet(streams[0], max_chunklen=12)

        # Get the stream info and description
        info = inlet.info()

        # Get sampling frequency, for muse2 sampling frequency = 256
        fs = 256

        # writing to the txt file 
        with open("muse_data.csv", mode = 'w', newline = '') as file:
            writer = csv.writer(file)
            
                # Length of epochs used to compute the FFT (in seconds)
        EPOCH_LENGTH = 1

        # Amount of overlap between two consecutive epochs (in seconds)
        OVERLAP_LENGTH = 0.8

        # Amount to 'shift' the start of each next consecutive epoch
        SHIFT_LENGTH = EPOCH_LENGTH - OVERLAP_LENGTH


        # Getting Data
        try:
            # The following loop acquires data
            while True:
                        
                # Acquire data
                # Obtain EEG data from the LSL stream
                eeg_data, timestamp = inlet.pull_chunk(timeout = 1, max_samples = int(SHIFT_LENGTH * fs))

                # Getting the data for each channel
                ch_data_tp9 = np.array(eeg_data)[:, 0]
                ch_data_af7 = np.array(eeg_data)[:, 1]
                ch_data_af8 = np.array(eeg_data)[:, 2]
                ch_data_tp10 = np.array(eeg_data)[:, 3]


                row_data = [np.mean(ch_data_tp9), np.mean(ch_data_af7), np.mean(ch_data_af8), np.mean(ch_data_tp10)]

                with open("muse_data.csv", mode = "a", newline = '') as file:
                    # Write to file in a new line
                    writer = csv.writer(file)

                    # write the data to the file
                    writer.writerow(row_data)

        except KeyboardInterrupt:
            print('Closing!')

        except IndexError:
            self.after(10,self.write_live_data)











    def display_detected_sensors(self, devices, mac_addresses):
        """
        Display the sensors found with the list_muses command to the screen.
        """

        # Destroy the "detecting sensors.." label.
        self.label.destroy()


        # Allow the frame to resize.
        self.frame.rowconfigure(0,weight=0)
        self.frame.columnconfigure(0,weight=1)
        self.frame.columnconfigure(1,weight=1)
        self.frame.rowconfigure(1,weight=1)

        # Create a label to display device name.
        device_label = Label(self.frame, text="Device Name", **self.secondary_label_style, pady=5)
        device_label.grid(row=0,column=0)
        self.labels.append(device_label)

        # Create a label to display mac address.
        mac_address_label = Label(self.frame, text="MAC Address", **self.secondary_label_style, pady=5)
        mac_address_label.grid(row=0,column=1)
        self.labels.append(mac_address_label)



        # For each device found,
        for x in range(0,len(devices)):
            
            # Create a function to connect to the chosen sensor.
            change_function = self.create_change_function(mac_addresses[x], x)

            # Create a button displaying the name of the sensor.
            button = Button(self.frame, text=devices[x], command=change_function, **self.button_style)
            button.grid(row=x+1, column=0, padx=10, pady=5)
            self.buttons.append(button)

            # Create a label displaying the Mac Address of the sensor.
            label = Label(self.frame, text=mac_addresses[x], **self.secondary_label_style)
            label.grid(row=x+1, column=1, padx=10, pady=5)
            self.labels.append(label)


    def create_change_function(self, address, x):
        """
        Create the function to execute when a sensor is selected.
        """
        return lambda:self.connect_thread(address, x)



    def connect_thread(self, address, button_index):
        """
        Creates a thread to connect to the specified Muse Sensor in the background.
        """

        # Retrieve the button selected from the list of sensors.
        self.chosen_button = self.buttons[button_index]

        self.address = address
        self.connected = None

        # Create a new thread to connect to the Muse sensor.
        self.connection_thread = Thread(target=self.stream_muse)
        self.connection_thread.start()


    def stream_muse(self):
        """
        Attempts to connect to the Muse sensor with the specified Mac Address.
        """

        # Set the colour of the chosen button to orange to signify it is attempting a connection.
        self.chosen_button.config(bg="orange")

        # Execute the command to attempt to connect to the Muse.
        command = subprocess.Popen(
            ["muselsl", "stream", "--address", self.address],
            stdout=subprocess.PIPE,
        )

        # While the process is running,
        while(command.poll() is None): 

            # Retrieve the currently connected Bluetooth device.
            stdoutdata = subprocess.getoutput("hcitool con")

            # If the device is connected to the specified Mac Address via Bluetooth,
            if self.address in stdoutdata.split():

                # The connection was successful, change the colour of the button to green.
                self.chosen_button.config(bg="green")

                # Change the colour of the stream button to blue, allowing the user to continue.
                self.stream_button.config(bg="#42c4ee")
                self.stream_button.config(command=lambda:self.change_frame("csv display"))

        # The connection was unsuccessful, the command returned an error code.
        self.chosen_button.config(bg="red")


    def display_loading(self, label_text):
        """
        Display the loading animation while the program is searching for available Muse sensors.
        """

        # Destroy each button on the screen.
        for button in self.buttons:
            button.destroy()

        # Destroy each label on the screen.
        for label in self.labels:
            label.destroy()

        # Create the animations that will be displayed to the screen.
        self.animations = [label_text, label_text+".", label_text+"..", label_text+"..."]
        self.animations_index = -1

        # Allow the frame to resize to fit the window.
        self.frame.columnconfigure(0,weight=1)
        self.frame.rowconfigure(0,weight=1)

        # Create a label displaying the animated text.
        self.label = Label(self.frame, text=label_text+"...", **self.secondary_label_style)
        self.label.grid(row=0, column=0, pady=10, sticky=NSEW)

        self.frame.pack(side=TOP,fill=BOTH)

        # Call the function to animate the chosen text.
        self.animate_text()

    def refresh(self):
        """
        Search for available Muse sensors using the muselsl list command.
        """

        # Create a thread to search for available muse sensors.
        self.refresh_thread = Thread(target=self.refresh_sensors_in_subthread)
        self.refresh_thread.start()
    
        # While the thread is running, display the loading animation.
        self.display_loading("Detecting Sensors")



    def animate_text(self):
        """
        Animate a chosen list of text, iterating through an array of text and displaying it to the screen.
        """

        # If the program is still searching for sensors,
        if self.label.winfo_exists():

            # Iterate to the next frame of the animation.
            self.animations_index += 1

            # Display the next frame of the animation to the screen.
            self.label.config(text=self.animations[self.animations_index % len(self.animations)])
            self.after(500,self.animate_text)


    def refresh_sensors_in_subthread(self):
        """
        Update the list of devices to be displayed to the screen.
        """

        # Call the function to list the available Muse sensors.
        muses = list_muses()

        # Initialise the lists of devices and Mac addresses.
        devices = []
        mac_addresses = []

        # For each device found,
        for x in muses:
            # Append the name of the device to the devices list.
            devices.append(x['name'])

            # Append the mac address of the device to the mac addresses list.
            mac_addresses.append(x['address'])

        # Call display_detected_sensors in the main thread once sensors are refreshed
        self.after(0, self.display_detected_sensors, devices, mac_addresses)


    
    def display_options(self):
        """
        Create a subframe which displays the options.
        """


         # Create the options frame
        options = Frame(self, bg="white")
        options.pack(side="bottom", fill="x")

        # Create a back button which redirects the user to the list of available CSVs.
        Button(options, text="Back", command=lambda:self.change_frame("csv list"), **self.button_style).pack(side="left", expand="True",pady=5)

        # Create a refresh button which refreshes the currently available Muse sensors.
        Button(options, text="Refresh", command=self.refresh, **self.button_style).pack(side="left", expand="True",pady=5)


        # Create a stream button which allows the user to stream live data once connected to a sensor.
        self.stream_button = Button(options, text="Stream Data", **self.button_style)
        self.stream_button.pack(side="left", expand="True",pady=5)

        # Change the colour of the stream button to light grey to signify it is initially not selectable.
        self.stream_button.config(bg="#d3d3d3")



    def change_frame(self, frame):
        """
        Destroy all threads currently running and redirect the user to the Display CSV frame.
        """

        self.writing_thread = Thread(target=self.write_live_data)
        self.writing_thread.start()

        # destroy threads (except the streaming thread)
        self.refresh_thread.join()

        # change frame to display the live data.
        self.visual_window.change_csv_frame(frame)
