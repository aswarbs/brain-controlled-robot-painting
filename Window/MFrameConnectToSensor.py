from Window import *
from tkinter import *
from muselsl import list_muses
from threading import *
import subprocess
import sys

class FrameConnectToSensor(FrameBase):
    """
    Class to connect to the Muse Sensor.
    """

    def __init__(self, master:Tk, main_window, visual_window, **args):
        self.visual_window = visual_window

        super().__init__(master, main_window)
        self.pack_propagate(0)
        self.master = master
        # set the 1st column to weight 1 to allow the canvas to fill the space
        self.columnconfigure(0,weight=1)
        self.rowconfigure(0, weight=1)

        self.buttons = []
        self.labels = []

        self.create_frame()


    def create_frame(self):

        self.canvas = Canvas(self,bg="white",borderwidth=0,highlightthickness=0)
        self.canvas.pack(fill=BOTH, expand=True)

        # Create a frame inside the canvas to hold the buttons
        self.frame = Frame(self.canvas,bg="white")

        # Create a scrollbar and associate it with the canvas
        scrollbar = Scrollbar(self.canvas, orient=VERTICAL, command=self.canvas.yview, **self.scrollbar_style)
        scrollbar.pack(side=RIGHT, fill=Y)

        self.canvas.configure(yscrollcommand=scrollbar.set)

        self.canvas.create_window((0, 0), window=self.frame, anchor=NW)

        self.display_options()

        self.refresh()


    def display_detected_sensors(self, devices, mac_addresses):

        self.label.destroy()

        self.frame.rowconfigure(0,weight=0)

        self.frame.columnconfigure(0,weight=1)
        self.frame.columnconfigure(1,weight=1)
        self.frame.rowconfigure(1,weight=1)

        device_label = Label(self.frame, text="Device Name", **self.secondary_label_style, pady=5)
        device_label.grid(row=0,column=0)
        self.labels.append(device_label)

        mac_address_label = Label(self.frame, text="MAC Address", **self.secondary_label_style, pady=5)
        mac_address_label.grid(row=0,column=1)
        self.labels.append(mac_address_label)



        for x in range(0,len(devices)):
            
            change_function = self.create_change_function(mac_addresses[x], x)

            button = Button(self.frame, text=devices[x], command=change_function, **self.button_style)
            button.grid(row=x+1, column=0, padx=10, pady=5)

            self.buttons.append(button)

            label = Label(self.frame, text=mac_addresses[x], **self.secondary_label_style)
            label.grid(row=x+1, column=1, padx=10, pady=5)

            self.labels.append(label)

    def create_change_function(self, address, x):
        return lambda:self.connect_thread(address, x)


    def connect_thread(self, address, button_index):
        """
        Creates a thread to connect to the specified Muse Sensor in the background.
        """
        self.chosen_button = self.buttons[button_index]

        self.address = address

        self.connected = None

        self.connection_thread = Thread(target=self.stream_muse)
        self.connection_thread.start()

    def stream_muse(self):
        self.chosen_button.config(bg="orange")

        command = subprocess.Popen(
            ["muselsl", "stream", "--address", self.address],
            stdout=subprocess.PIPE,
        )

        while(command.poll() is None): # while the process is running
            stdoutdata = subprocess.getoutput("hcitool con")
            if self.address in stdoutdata.split():
                self.chosen_button.config(bg="green")
                self.stream_button.config(bg="#42c4ee")
                self.stream_button.config(command=lambda:self.change_frame("csv display"))
        self.chosen_button.config(bg="red")


    def display_loading(self, label_text):

        for button in self.buttons:
            button.destroy()

        for label in self.labels:
            label.destroy()

        self.animations = [label_text, label_text+".", label_text+"..", label_text+"..."]
        self.animations_index = -1

        self.frame.columnconfigure(0,weight=1)
        self.frame.rowconfigure(0,weight=1)


        self.label = Label(self.frame, text=label_text+"...", **self.secondary_label_style)
        self.label.grid(row=0, column=0, pady=10, sticky=NSEW)

        self.frame.pack(side=TOP,fill=BOTH)

        self.animate_text()

    def refresh(self):

        self.refresh_thread = Thread(target=self.refresh_sensors_in_subthread)
        self.refresh_thread.start()
    
        self.display_loading("Detecting Sensors")



    def animate_text(self):
        if self.label.winfo_exists():
            self.animations_index += 1
            self.label.config(text=self.animations[self.animations_index % len(self.animations)])
            self.after(500,self.animate_text)


    def refresh_sensors_in_subthread(self):
        muses = list_muses()

        devices = []
        mac_addresses = []

        for x in muses:
            devices.append(x['name'])
            mac_addresses.append(x['address'])

        # Call display_detected_sensors in the main thread once sensors are refreshed
        self.master.after(0, self.display_detected_sensors, devices, mac_addresses)


        
    def display_options(self):
        """
        Create a subframe which displays the Options.
        """
         # Create the options frame
        options = Frame(self, bg="white")
        options.pack(side="bottom", fill="x")

        Button(options, text="Back", command=lambda:self.change_frame("csv list"), **self.button_style).pack(side="left", expand="True",pady=5)
        Button(options, text="Refresh", command=self.refresh, **self.button_style).pack(side="left", expand="True",pady=5)

        self.stream_button = Button(options, text="Stream Data", **self.button_style)
        self.stream_button.pack(side="left", expand="True",pady=5)
        self.stream_button.config(bg="#d3d3d3")



    def change_frame(self, frame):

        # destroy threads (except the streaming thread)
        self.refresh_thread.join()

        # change frame
        self.visual_window.change_csv_frame(frame, path="muse_data.csv")
