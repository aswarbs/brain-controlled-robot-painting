from Window import *
from tkinter import *
from threading import Thread
import subprocess
import os
import pty
import netifaces
import time

class FramePhysicalMenu(FrameBase):
    """
    A frame to allow the user to use the UR3e arm to draw to a canvas.
    """
        
    def __init__(self, master:Tk, main_window):
        super().__init__(master, main_window)
        self.main_window = main_window
        self.connected = False
        self.original_directory = os.getcwd()

        self.change_frame_content(self.create_program_screen)
        self.pack_propagate(0)



    def create_program_screen(self):
        # return a frame that is the program screen, set the current frame to this frame
        
        frame = Frame(self)

        title_frame = Frame(frame)
        title_frame.config(bg="white")

        # create help button
        Button(title_frame, text="First Time Connecting?", command= lambda:self.change_frame_content(self.create_help_frame),**self.button_style).pack(side=LEFT, pady=5, padx=5)
        

        title_frame.pack(side=TOP, fill=X)


        options_frame = Frame(frame)
        options_frame.config(bg="white")
        
        # create back button
        Button(options_frame, text="Back", command= lambda:self.change_frame_content(self.main_window.change_frame("main menu")),**self.button_style).pack(side=LEFT, expand=TRUE, pady=5)

        self.connection_button = Button(options_frame, text="Attempt Connection", command= lambda:self.attempt_connection(),**self.button_style)
        self.connection_button.pack(side=LEFT, expand=TRUE, pady=5)

        self.launch_button = Button(options_frame, text="Launch Program",**self.button_style)

        if(self.connected == False):
            self.launch_button.config(bg="#D3D3D3") # cannot launch yet
        else:
            self.launch_button.config(bg="#42c4ee")
            self.launch_button.config(command=lambda:self.run_physical_program())
            self.connection_button.config(bg="green")

        self.launch_button.pack(side=LEFT, expand=TRUE, pady=5)

        options_frame.pack(side=BOTTOM, fill=X)

        Label(frame, text="Connecting to the UR3e Arm", **self.title_label_style).pack(side=TOP, pady=35, padx=10)

        Label(frame,text="If this is your first time using the program, click the help button to set up a workspace.", **self.secondary_label_style).pack()
        Label(frame,text="An Ethernet connection is highly recommended.", **self.secondary_label_style).pack()
        Label(frame,text="Otherwise, ensure you are connected to the same WiFi as the UR3e arm.", **self.secondary_label_style).pack()

        return frame
    
    




    def create_help_frame(self):

        frame = Frame(self)
        # return a frame that is the help screen, set the current frame to this frame
        Label(frame, text="help stuff", **self.secondary_label_style).pack(side=TOP)

        # create back button
        Button(frame, text="Back", command= lambda:self.change_frame_content(self.create_program_screen),**self.button_style).pack(side=BOTTOM)

        return frame
    

    def get_network_ip(self):
        # TODO: MAKE IT SO THAT IT HAS TO START WITH 10
        interfaces = netifaces.interfaces()
        for interface in interfaces:
            addresses = netifaces.ifaddresses(interface)
            if netifaces.AF_INET in addresses:
                for ip_info in addresses[netifaces.AF_INET]:
                    ip_address = ip_info['addr']
                    if not ip_address.startswith('127.'): # filter out localhost
                        return ip_address
        return None

    
    def create_launch_frame(self):

        frame = Frame(self)

        Label(frame, text="Ensure you are connected to the same WiFi as the Robot.", **self.secondary_label_style).pack(pady=10)

        self.external_control_image = PhotoImage(file='./Assets/external_control.png')
        Label(frame, image = self.external_control_image, **self.image_style).pack()

        
        # get IP and display it in label

        ip_address = self.get_network_ip()

        Label(frame, text = "Your IP Address: " + ip_address, **self.secondary_label_style).pack(pady=10)

        Label(frame, text="On the UR3e tablet, navigate to the page shown in the image.", **self.secondary_label_style).pack()

        Label(frame, text="Under Host IP, enter your IP address.", **self.secondary_label_style).pack()

        self.continue_button = Button(frame, text= "Continue", **self.button_style)
        self.continue_button.pack(side= BOTTOM, pady=10)

        # change so the colour is not blu
        # e
        self.continue_button.config(bg="#D3D3D3")

        # check if robot connected to control interface
        


        return frame
    
    def change_frame_content(self, new_content):
        # Destroy all widgets on the current frame
        for widget in self.winfo_children():
            widget.destroy()

        # Create and pack the new content on the frame
        self = new_content()
        self.config(bg="white")
        self.pack(side=LEFT, expand=TRUE,fill=BOTH)


    def attempt_connection(self):

        # connect button is turned orange
        # create a thread that runs launch file
        # on another page:
        # once the launch file has printed robot connected, prompt user to enter ip into tablet
        # confirm button, goes back to main screen and connect button is turned green

        self.connection_button.config(bg="orange")

        # screen to say turn on the robot

        self.connection_thread = Thread(target=self.run_connection_command)
        self.connection_thread.start()



    def run_connection_command(self):
        ros_command = "source devel/setup.bash && roslaunch cms_ur_launch tsoutsi.launch"
        os.chdir("catkin_ws")

        master, slave = pty.openpty()

        process = subprocess.Popen(
            ["bash", "-c", ros_command],
            stdout=slave,
            stderr=slave,
            universal_newlines=True
        )

        while True:
            try:
                output = os.read(master, 1024).decode()
                if output:
                    if "Robot mode is now RUNNING" in output:
                        # SUCCESS!
                        os.chdir(self.original_directory)

                        # open a page that tells the user to put their ip in

                        self.change_frame_content(self.create_launch_frame)

                    if("Robot connected to reverse interface" in output and self.connected==False):
                        self.continue_button.config(bg="#42c4ee")
                        self.continue_button.config(command=lambda:self.change_frame_content(self.create_program_screen))
                        self.connected = True
            



                        #break
            except OSError:
                break

        process.wait()

        if process.returncode != 0:
            # FAIL!
            self.connection_button.config(bg="red")
            os.chdir(self.original_directory)

    def wait_for_moveit_completion(self):
        while not self.moveit_succeeded:
            time.sleep(0.1)
        # Moveit completed, trigger the launch_program method
        self.launch_program()

    def run_physical_program(self):
        self.launch_button.config(bg="orange")
        self.moveit_succeeded = False

        moveit_thread = Thread(target=self.launch_moveit)
        moveit_thread.start()

        # Start a separate thread to wait for moveit completion
        wait_thread = Thread(target=self.wait_for_moveit_completion)
        wait_thread.start()




    def launch_moveit(self):
        moveit_command = "source devel/setup.bash && roslaunch ur3e_moveit_config moveit_planning_execution.launch"

        os.chdir("catkin_ws")

        master, slave = pty.openpty()

        process = subprocess.Popen(
            ["bash", "-c", moveit_command],
            stdout=slave,
            stderr=slave,
            universal_newlines=True
        )

        while True:
            try:
                output = os.read(master, 1024).decode()
                if output:
                    if "You can start planning now!" in output:
                        # SUCCESS!
                        self.moveit_succeeded = True
                        print("moveit succeeded")

                        break
            except OSError:
                break

        process.wait()
        # get moveit successful message

        
        if process.returncode != 0:
            # FAIL!
            self.launch_button.config(bg="red")
            os.chdir(self.original_directory)


    def launch_program(self):


        program_command = "source devel/setup.bash && rosrun ur3e_moveit_config plan_moves.py"
        # get program successful message


        master, slave = pty.openpty()

        process = subprocess.Popen(
            ["bash", "-c", program_command],
            stdout=slave,
            stderr=slave,
            universal_newlines=True
        )

        while True:
            try:
                output = os.read(master, 1024).decode()
                if output:
                    print(output)
                    if "Robot mode is now RUNNING" in output:
                        # SUCCESS!
                        self.connection_button.config(bg="green")
                        os.chdir(self.original_directory)

                        break
            except OSError:
                break

        process.wait()

        if process.returncode != 0:
            # FAIL!
            self.launch_button.config(bg="red")
            os.chdir(self.original_directory)



        pass
