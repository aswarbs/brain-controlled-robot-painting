from Window import *
from tkinter import *
from threading import Thread
import subprocess
import os
import pty
import netifaces
import time

"""
TODO:

- WAIT UNTIL THE USER HAS SELECTED A CSV (MAKE A SELECT CSV FRAME, REPLACE.)
- PASS THE CSV INTO THIS CLASS
- PASS THE CSV TO PLAN_MOVES

- CREATE THE HELP SCREEN
"""

class FramePhysicalMenu(FrameBase):
    """
    A frame holding functionality to connect to the UR3e arm and run the program.
    """
        
    def __init__(self, master:Tk, main_window):

        # Initialise the frame.
        super().__init__(master, main_window)

        # Initialise a global variable representing the Window the frames are held on.
        self.main_window = main_window

        # Initialise a boolean describing if the user is connected to the UR3e arm.
        self.connected = False

        # Initialise a variable describing the current directory.
        self.original_directory = os.getcwd()


        # Change the content on the frame to the Program Screen.
        self.change_frame_content(self.create_program_screen)

        # Disable the frame from resizing.
        self.pack_propagate(0)

    def penLoop(self):
        """Passes the epoch to the plan_moves class"""
        pass

    def penStop(self):
        """Pauses the physical program"""
        pass


    def attempt_connection(self):
        """
        Create a thread which attempts to connect to the UR3e arm.
        """

        # connect button is turned orange
        # create a thread that runs launch file
        # on another page:
        # once the launch file has printed robot connected, prompt user to enter ip into tablet
        # confirm button, goes back to main screen and connect button is turned green

        self.connection_button.config(bg="orange")

        self.connection_thread = Thread(target=self.run_connection_command)
        self.connection_thread.start()




    def create_program_screen(self):
        """
        Return a frame containing the content for the Main Program Screen.
        """
        
        # Create a frame for the content to be placed on.
        frame = Frame(self)

        # Create a subframe to hold the title.
        title_frame = Frame(frame)
        title_frame.config(bg="white")

        # create a Help button which navigates to the Help screen.
        Button(title_frame, text="First Time Connecting?", command= lambda:self.change_frame_content(self.create_help_frame),**self.button_style).pack(side=LEFT, pady=5, padx=5)
        title_frame.pack(side=TOP, fill=X)

        # Create a subframe to hold the options.
        options_frame = Frame(frame)
        options_frame.config(bg="white")
        
        # Create a back button which navigates to the Main Menu.
        Button(options_frame, text="Back", command= lambda:self.go_back(),**self.button_style).pack(side=LEFT, expand=TRUE, pady=5)

        # Create a Connection button which attempts initial connection to the UR3e arm.
        self.connection_button = Button(options_frame, text="Attempt Connection", command= lambda:self.attempt_connection(),**self.button_style)
        self.connection_button.pack(side=LEFT, expand=TRUE, pady=5)

        # Create a launch button which initially has no function.
        self.launch_button = Button(options_frame, text="Launch Program",**self.button_style)

        # If the user is not connected to the UR3e arm,
        if(self.connected == False):
            # Set the colour of the launch button to Grey, disabling the user from launching the program.
            self.launch_button.config(bg="#D3D3D3")

        # If the user is connected to the UR3e arm,
        else:
            # Set the colour of the launch button to Blue.
            self.launch_button.config(bg="#42c4ee")

            # Assign the button a command to attempt to run the program.
            self.launch_button.config(command=lambda:self.run_physical_program())

            # Set the colour of the connection button to green: the user has connected.
            self.connection_button.config(bg="green")

        self.launch_button.pack(side=LEFT, expand=TRUE, pady=5)
        options_frame.pack(side=BOTTOM, fill=X)

        # Create a Title label for the frame.
        Label(frame, text="Connecting to the UR3e Arm", **self.title_label_style).pack(side=TOP, pady=35, padx=10)

        # Create Description labels for the frame.
        Label(frame,text="If this is your first time using the program, click the help button to set up a workspace.", **self.secondary_label_style).pack()
        Label(frame,text="An Ethernet connection is highly recommended.", **self.secondary_label_style).pack()
        Label(frame,text="Otherwise, ensure you are connected to the same WiFi as the UR3e arm.", **self.secondary_label_style).pack()

        return frame
    
    def go_back(self):
        os.chdir(self.original_directory)
        self.change_frame_content(self.main_window.change_frame("main menu"))
    
    


    def create_help_frame(self):
        """
        Create a help frame which prompts the user to create a catkin workspace.
        """


        """
        TODO: 

        THIS WILL BE FOR INITIAL CONNECTION

        RUN ALL COMMANDS TO CREATE CATKIN_WS AND STUFF

        
        """

        # Create a frame to hold the content of the Help screen.
        frame = Frame(self)

        # TEMP
        Label(frame, text="help stuff", **self.secondary_label_style).pack(side=TOP)

        # Create a back button which navigates the user to the Program Screen.
        Button(frame, text="Back", command= lambda:self.change_frame_content(self.create_program_screen),**self.button_style).pack(side=BOTTOM)

        return frame
    

    def get_network_ip(self):
        """
        Display the Network IP address for the user to enter into the UR3e tablet.
        """

        # Retrieve a list of possible IP addresses.
        interfaces = netifaces.interfaces()

        # For each IP address,
        for interface in interfaces:

            # Retrieve the IP address.
            addresses = netifaces.ifaddresses(interface)

            # If the address is an IP address.
            if netifaces.AF_INET in addresses:

                # Retrieve the information about the address.
                for ip_info in addresses[netifaces.AF_INET]:
                    ip_address = ip_info['addr']

                    # Filter out the localhost IP address.
                    if not ip_address.startswith('127.'): 
                        return ip_address
                    
        # Else, only localhost was found.
        return None

    
    def create_launch_frame(self):
        """
        Create the frame which holds the information to connect to the UR3E arm.
        """

        # Create a frame to display the connection information.
        frame = Frame(self)

        # Create a label prompting the user to connect to the same WIFI as the robot.
        Label(frame, text="Ensure you are connected to the same WiFi as the Robot.", **self.secondary_label_style).pack(pady=10)

        # Display the image of the External Control information.
        self.external_control_image = PhotoImage(file='./Assets/external_control.png')
        Label(frame, image = self.external_control_image, **self.image_style).pack()

        # Retrieve the IP Address.
        ip_address = self.get_network_ip()

        # Display the IP Address.
        Label(frame, text = "Your IP Address: " + ip_address, **self.secondary_label_style).pack(pady=10)

        # Display information labels to help the user.
        Label(frame, text="On the UR3e tablet, navigate to the page shown in the image.", **self.secondary_label_style).pack()
        Label(frame, text="Under Host IP, enter your IP address.", **self.secondary_label_style).pack()

        # ADD A BACK BUTTON

        # Create a button allowing the user to continue once successfully connected.
        self.continue_button = Button(frame, text= "Continue", **self.button_style)
        self.continue_button.pack(side= BOTTOM, pady=10)

        # Change the colour of the continue button to grey as the user has not yet connected.
        self.continue_button.config(bg="#D3D3D3")

        
        return frame
    
    def change_frame_content(self, new_content):
        """
        Change the currently hosted frame.
        """

        # Destroy all widgets on the current frame
        for widget in self.winfo_children():
            widget.destroy()

        # Create and pack the new content on the frame
        self = new_content()
        self.config(bg="white")
        self.pack(side=LEFT, expand=TRUE,fill=BOTH)



    def run_connection_command(self):
        """
        Attempt to connect to the UR3E arm using the roslaunch cms_ur_launch tsoutsi.launch command.
        """

        
        ros_command = "source devel/setup.bash && roslaunch cms_ur_launch tsoutsi.launch"

        # Change directory to the catkin workspace.
        
        os.chdir("../../..")
        print(os.getcwd())

        master, slave = pty.openpty()

        # Run the connection command.
        process = subprocess.Popen(
            ["bash", "-c", ros_command],
            stdout=slave,
            stderr=slave,
            universal_newlines=True
        )

        # The command runs indefinitely to establish the connection.
        while True:
            try:
                # Read the current line outputted by the process.
                output = os.read(master, 1024).decode()

                # If a line was read,
                if output:

                    # If the line contains the acceptance string,
                    if "Robot mode is now RUNNING" in output:

                        # SUCCESS!
                        # Revert the directory back to the original directory.
                        os.chdir(self.original_directory)

                        # Change the frame to a frame prompting the user to enter their IP.
                        self.change_frame_content(self.create_launch_frame)

                    # If the user successfully connected to the robot,
                    if("Robot connected to reverse interface" in output and self.connected==False):

                        print("yes")

                        
                        # Allow the user to continue.
                        self.continue_button.config(bg="#42c4ee")
                        self.continue_button.config(command=lambda:self.change_frame_content(self.create_program_screen))

                        # Record that the user has connected to the robot.
                        self.connected = True

                        os.chdir(self.original_directory)
                        break

            


                    # If the process has ended, connection has failed.
                    if "Connection to reverse interface dropped" in output:
                        # FAIL!
                        print("yes")
                        self.connected = False
                        self.connection_button.config(bg="red")

                        os.chdir(self.original_directory)
            except OSError:
                break



    
    def wait_for_moveit_completion(self):
        """
        A thread to wait for a command to succeed without hanging the program.
        """

        # While the acceptance flag has not been set,
        while not self.moveit_succeeded:
            # Wait for the thread to be set.
            time.sleep(0.1)

        # Moveit completed, trigger the launch_program method
        self.launch_program()

    def run_physical_program(self):
        """
        Attempt to run the program on the UR3E arm.
        """

        # Change the colour of the launch button to orange to show that the program is attempting to run.
        self.launch_button.config(bg="orange")

        # Set the moveit success flag to false.
        self.moveit_succeeded = False

        # Create a thread that attempts to launch moveit.
        moveit_thread = Thread(target=self.launch_moveit)
        moveit_thread.start()

        # Start a separate thread to wait for moveit completion
        wait_thread = Thread(target=self.wait_for_moveit_completion)
        wait_thread.start()




    def launch_moveit(self):
        """
        A thread responsible for launching the moveit command.
        """

        moveit_command = "source devel/setup.bash && roslaunch ur3e_moveit_config moveit_planning_execution.launch"

        # Change directory to the catkin workspace in order to run ros commands.

       
        
        os.chdir("../../..")

        master, slave = pty.openpty()

        # Run the command which attempts to launch moveit.
        process = subprocess.Popen(
            ["bash", "-c", moveit_command],
            stdout=slave,
            stderr=slave,
            universal_newlines=True
        )

        # The command runs indefinitely to keep a connection to the robot.
        while True:
            try:
                # Read the current output from the command.
                output = os.read(master, 1024).decode()

                # If the output is the success output,
                if "You can start planning now!" in output:

                    # SUCCESS!
                    self.moveit_succeeded = True
                    os.chdir(self.original_directory)
                    break

                # If the process has ended, it has failed.
                if process.returncode == 1:
                    # FAIL!
                    self.launch_button.config(bg="red")
                    self.moveit_succeeded = False
                    os.chdir(self.original_directory)

            except OSError:
                break



    
    def launch_program(self):
        """
        Attempt to run the plan_moves file which controls movement of the robot.
        """


        program_command = "source devel/setup.bash && rosrun ur3e_moveit_config plan_moves.py"

        os.chdir("../../..")


        master, slave = pty.openpty()

        # Execute the command to run the plan_moves file.
        process = subprocess.Popen(
            ["bash", "-c", program_command],
            stdout=slave,
            stderr=slave,
            universal_newlines=True
        )
        # While the command is running,
        while True:
            try:
                # Read the current output.
                output = os.read(master, 1024).decode()

                # If there is output,
                if output:
                    print(output)


                # Check the return code.
                if "Done" in output:
                    # Process ended successfully.
                    print("Success")
                    self.launch_button.config(bg="#42c4ee")
                    os.chdir(self.original_directory)
                    break


                if "ABORTED: CONTROL_FAILED" in output:
                    # Process failed.
                    print("Failure")
                    self.launch_button.config(bg="red")
                    os.chdir(self.original_directory)
                    break

            except OSError:
                break

