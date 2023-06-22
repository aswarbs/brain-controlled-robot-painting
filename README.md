

<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://projects.cs.nott.ac.uk/psyas17/brain-controlled-robot-painting">
    <img src="Assets/logo.png" alt="Logo" width="80" height="80">
  </a>

<h3 align="center">Brain Controlled Robot Painting</h3>

  <p align="center">
    Brain Controlled Robot Painting is a robotic installation designed to create paintings based on
    the participant's brain activity data.
    <br />
    <a href="https://projects.cs.nott.ac.uk/psyas17/brain-controlled-robot-painting/-/wikis/home"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    ·
    <a href="https://projects.cs.nott.ac.uk/psyas17/brain-controlled-robot-painting/-/issues">Report Bug</a>
    ·
    <a href="https://projects.cs.nott.ac.uk/psyas17/brain-controlled-robot-painting/-/issues">Request Feature</a>
  </p>
</div>




<!-- ABOUT THE PROJECT -->
## About The Project

[![Product Name Screen Shot][product-screenshot]](https://example.com)

  Brain Controlled Robot Painting is a robotic installation designed to create paintings based on
  the participant's brain activity data. The project uses the EEG Muse 2 band to collect the
  participant's brain activity data, which is then sent to a robotic arm (UR3e - Universal Robot Arm)
  that creates a painting based on the data. The installation was designed by an ambitious group of
  2nd year Computer Science students at the University of Nottingham.

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- GETTING STARTED -->
## Getting Started

To get a local copy up and running follow these simple example steps.

### Prerequisites

- [Muse 2 Sensor](https://choosemuse.com/products/muse-s-gen-2-subscription/): This is the brain sensor we use to record the brainwaves for both the physical and digital versions of the program.
(Note: other brain sensors may be suitable, if there is an option to record the power frequency data in the form of a CSV.)

- [Universal Robot UR3e Arm](https://www.universal-robots.com/products/ur3-robot/?utm_source=Google&utm_medium=cpc&utm_cja=Demo&utm_leadsource=Paid%20Search&utm_campaign=HQ_UK_Always-On2021&utm_content=textad&utm_term=ur3e&gclid=CjwKCAjw0ZiiBhBKEiwA4PT9zzCeKHdzOQXcTTBz48I0TD7OVmmo0pPlBwlILHntiE7iao-VbE0PnhoCXwoQAvD_BwE): This is the arm we use to draw the paintings in the physical version of the program. 

- [Gripper attachment](https://projects.cs.nott.ac.uk/comp2002/2022-2023/team9_project/-/blob/main/Prototypes/CMS%20UR3e%20Pen%20Mount.stl) : We 3D printed a custom attachment to hold the pen, which is located in our repository under “Prototypes/CMS UR3e Pen Mount.stl”, along with rubber bands to secure the pen in place. If you don’t have a 3D printer, you can use one of Universal Robots’ advertised grippers, or create an alternative device to hold the pen.


- Pen: We used a Sharpie, but you are free to use any pen you like. 
(Note: different pens will need to be configured on the arm to draw correctly.)

- A2 Paper: This is what we used as the surface for the painting. 
(Note: you are free to use an alternative canvas, but it should be configured to allow the pen to draw.)

- [An Ethernet cable](https://www.screwfix.com/c/electrical-lighting/cable/cat8960001?cableproducttype=ethernet) (Optional): We experienced connection issues when connecting to the robot wirelessly, and this improved when using an Ethernet cable.

- [A USB-C to USB converter](https://www.amazon.co.uk/AmazonBasics-Type-C-Gen1-Female-Adapter/dp/B01GGKYYT0/ref=asc_df_B01GGKYYT0/?tag=googshopuk-21&linkCode=df0&hvadid=205231791255&hvpos=&hvnetw=g&hvrand=11493399249999556160&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=1006500&hvtargid=pla-490647422152&psc=1&th=1&psc=1) (Optional): Some Mac machines have no USB port. If you are dual booting on a Mac, you may need a converter to boot the Ubuntu image.


### Installation

## 1.0.	General Overview

The project “Brain Controlled Robot Painting” is a multi-disciplinary project sits between the fields of Computer Science such as BCI, HCI and creative arts and installation. This project creates an interface between users’ brain activity, measured in real time with a BCI (brain sensor), and a robotic arm designed to create a painting based on the users’ brain activation. 

Our implementation of this includes 2 virtual Python programs and an implementation for the robotic arm. 

### 1.1.	Virtual Programs

The virtual programs are both Python programs using most of the standard Python libraries. 

#### 1.1.1.	Abstract Program
This program runs in real time along with the brain data, drawing lines and abstract art simultaneously as the brain data is being recorded. It can be run for as long as the user wishes and the program can be stopped, and the generated drawing can be exported by the user.
#### 1.1.2.	Faces Program
This program requires the brain data to be recorded prior to the running of the program. The data can be recorded for any amount of time, as per the wishes of the user (advisable: 2-4 mins). After brain data is recorded and this program is run, it generates a face that represents the emotion of the user at the time of recording the brain data.

### 1.2.	The Robotic implementation
The robotic implementation involves the creation of a catkin_ws, which is simply a workspace in ROS (Robotic Operating System) that allows users to build and modify packages. This workspace includes Universal Robot git repositories, and the code that generates the drawing is predominantly in Python.

### 1.3.	Uses
These programs can be used for a range of things such as, generating art, better understanding emotions of people who may not be able to express as well, representing these emotions in a more visual and artistic manner, and simply to see how you are feeling!


## 2.0.	Requirements

### 2.1.	Hardware Requirements

This project requires access to the following hardware:

- Muse 2 Sensor: This is the brain sensor we use to record the brainwaves for both the physical and digital versions of the program.
(Note: other brain sensors may be suitable, if there is an option to record the power frequency data in the form of a CSV.)

- Universal Robot UR3e Arm: This is the arm we use to draw the paintings in the physical version of the program. 

- Gripper attachment: We 3D printed a custom attachment to hold the pen, which is located in our repository under “Prototypes/CMS UR3e Pen Mount.stl”, along with rubber bands to secure the pen in place. If you don’t have a 3D printer, you can use one of Universal Robots’ advertised grippers, or create an alternative device to hold the pen.


- Pen: We used a Sharpie, but you are free to use any pen you like. 
(Note: different pens will need to be configured on the arm to draw correctly.)

- A2 Paper: This is what we used as the surface for the painting. 
(Note: you are free to use an alternative canvas, but it should be configured to allow the pen to draw.)

- An Ethernet cable (Optional): We experienced connection issues when connecting to the robot wirelessly, and this improved when using an Ethernet cable.

- A USB-C to USB converter (Optional): Some Mac machines have no USB port. If you are dual booting on a Mac, you may need a converter to boot the Ubuntu image.

## 2.2.	Software Requirements

### 2.2.1.	Installing Ubuntu

If you already have an Ubuntu distribution installed (preferably 20.04), please skip this step.


We would highly recommend dual booting Linux for this project rather than using a Virtual Machine. This is because the Muse sensor requires a Bluetooth connection and the UR3e requires an Ethernet connection, and using a Virtual Machine for this caused problems, because the platforms we tried did not recognise Bluetooth or Ethernet. Note: dual booting Linux is not currently possible on the M1 Macs. However, you can run Linux from a bootable Linux USB.

#### 2.2.1.1.	Prerequisites:

- Ubuntu 20.04 is the preferred version to install, as this is the official version supported by ROS Noetic. Other distributions may be suitable, but this manual will assume you are using the supported version.
- To install Ubuntu 20.04, you must have a USB drive with at least 4GB of space. When Ubuntu is installed on this drive all other data will be deleted, so ensure that the drive is empty.
- It may be worth making a backup of your existing data in case anything goes wrong.


#### 2.2.1.2.	Creating Partitions:

If you are dual booting Ubuntu alongside Windows, first perform these steps: 
- You must first allocate Ubuntu some space on your disk. In the Windows menu, search for “disk partitions” and click “Create and format hard disk partitions”. 
- In the Disk Management tool, right-click on the drive which you want to partition and select “shrink volume”. 
- The recommended amount of space to allocate Ubuntu is 30-40GB, but you may install on 20GB if you have limited space on your device.

If you are dual booting Ubuntu alongside Mac (not M1), first perform these steps:
- Open the Apple menu and go to System Settings -> General -> Storage.
- You must first allocate Ubuntu some space on your disk. To dual boot Linux on a Mac, you need two extra partitions: one for Linux and a second for swap space. 
- The swap partition must be as big as the amount of RAM your Mac has. To check this, go to Apple -> About this Mac -> Memory.
- Open “Disk Utility” from the Utilities folder in your applications. In the top-left corner, select View -> Show All Devices.
- Select the highest-level drive for your Mac hard disk, then click Partition.
- Use the plus (+) button to create a new partition, then click Add Partition.
- Name it UBUNTU and set the format to MS-DOS (FAT32). Allocate it as much space as you would like to use for Linux. The recommended amount of space to allocate Ubuntu is 30-40GB, but you may install on 20GB if you have limited space on your device.
- Click Apply to create the partition.
- Repeat the steps above to create another partition. Name this partition SWAP and set the format to MS-DOS (FAT). Allocate as much memory as the amount of RAM in your Mac, which you checked previously.


Once you have allocated space for Ubuntu, you must install a disk image from the Ubuntu website onto the USB. We recommend the 20.04 Image for this project.

#### 2.2.1.3.	Creating a bootable image:

If you are dual booting on Windows, follow these steps:
- Download software which creates bootable USB drives. For this tutorial, we will use Rufus.
- Plug in the USB drive to your computer.
- Double click the downloaded Rufus.exe folder which should be in Downloads.
- Rufus may automatically find the Ubuntu ISO. If not, browse to the ISO you downloaded on the USB by clicking on the Select button.
- Ensure that the Target System is correct (check if your system uses BIOS or UEFI), and the Partition Scheme is correct (check if your system uses MBR or GPT). 
- Ensure the correct drive is selected, since the next step erases it!
- Select Start to create the bootable image.
- You may be asked how to write the image, if so, select “Write in ISO Image Mode”.

If you are dual booting on Mac, follow these steps:
- Download software which creates bootable USB drives. For this tutorial, we will use Etcher.
- Plug in the USB drive to your computer. 
- Open Etcher and click Select Image.
- Navigate to the Ubuntu disk image you downloaded and click Open.
- Insert your USB flash drive and Etcher should automatically select it. If it doesn’t click, Select Target or Change to select the flash drive yourself.
- Ensure the correct drive is selected, since the next step erases it!
- Click Flash and enter your administrator password to erase the USB flash drive and create an Ubuntu USB installer.
- When it’s finished, macOS prompts you to Eject the flash drive.

#### 2.2.1.4.	Booting Ubuntu from USB:

If you are dual booting on Windows, follow these steps:
- Ensure the USB is plugged in to the device.
- This step will differ on most devices: you must boot into the new Linux image. You may need to restart the device and press a function key when the system manufacturer screen is shown (try F2, F10, F12), or access the UEFI boot settings from within Windows: In the Windows menu, search for UEFI and then click on “Change advanced startup options”. Then, go to “Advanced Start-up” and click “Restart Now”. If neither of these work for you, you may need to find specific instructions to access boot settings for your device. On the next screen, click “Use a device”, then select the USB containing the Ubuntu image.
- Once the system has restarted, you should see a screen containing an option to “Install Ubuntu”. Navigate to this option using the arrow keys and select it using Enter.

If you are dual booting on Mac, follow these steps:
- Restart your max while holding Option and insert the USB into your device.
- As the boot loader appears, use the arrow keys to select the Boot EFI option and hit Enter.
- Double click the Install Ubuntu item on the desktop.

Now, you can follow the Ubuntu installer guidance to install Ubuntu on the partition.

### 2.2.2.	Installing ROS Noetic

ROS Noetic is the software we used to control the UR3e arm. 

If ROS Noetic is already installed, skip these steps. Otherwise, follow these instructions to install ROS Noetic on your Ubuntu device:


1.	Curl should be installed on the machine. It can be installed with this command:
<div align="center">

      sudo apt install curl
</div>

2.	First, set up your device to accept software from packages.ros.org: 
<div align="center">

      sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
</div>

3.	Next, set up your keys:
<div align="center">

      curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
</div>

4.	Make sure your Debian package index is up to date:
<div align="center">

      sudo apt update
</div>

5.	Install the full desktop version of ROS:
<div align="center">

      sudo apt install ros-noetic-desktop-full
</div>


### 2.2.3.	Installing Python

Python is used to run the ROS controller, and we use various Python libraries to run various components of the program.

If Python is already installed, skip this step. Otherwise, follow these instructions to install Python on your Ubuntu device:

1.	Update the package index:
<div align="center">

      sudo apt update
</div>



2.	Check if Python is already installed:
<div align="center">

      python3 -V
</div>



3.	If Python is not installed, install the prerequisites:
<div align="center">

      sudo apt install software-properties-common
</div>



4.	Add the deadsnakes PPA to your system’s sources list:
<div align="center">

      sudo add-apt-repository ppa:deadsnakes/ppa
</div>



5.	Install Python 3.9:
<div align="center">

      sudo apt install python3.9
</div>

### 2.2.4.	Installing Necessary Libraries

#### 2.2.4.1.	PIP
PIP is Python’s package manager. Many of these libraries can be installed using pip, and pip can be installed on Ubuntu using this command: 
<div align="center">

      sudo apt install python3-pip
</div>




2.2.4.2.	 Muse:
MuseLSL is the Python library we use to connect the Muse sensor to the device and record its data. MuseLSL can be installed using PIP:
<div align="center">

      pip install muselsl
</div>



2.2.4.3.	 Relevant libraries: 

Turtle is the Python library we use to draw the pen path to the screen. Turtle should be installed as part of the Python library, but if this is not the case, you can run this command:
<div align="center">

      pip install PythonTurtle
</div>


 
Pandas is the library we use to handle the CSV containing the brain data. Pandas can be installed with PIP:
<div align="center">

      pip install pandas
</div>


Tkinter is the library we use to create the GUI in the prototype. Tkinter can be installed with PIP:
<div align="center">

      pip install tkinter
      
</div>



PIL is an optional library which allows you to export created images to your files. PIL can be installed with PIP:

<div align="center">

      pip install pillow
</div>

tkmacosx is a module provides some modified widgets of Tkinter which fixes many issues with widgets not working properly on macOS platform and can be installed with PIP:
<div align="center">

      pip install tkmacosx
</div>

mne an open-source Python package for exploring, visualizing, and analyzing human neurophysiological data such as MEG, EEG, sEEG, ECoG, and more. This needs to be a specific version of mne and can be installed using PIP:
<div align="center">

      pip install mne==0.24.1
</div>			
		

Now, you have everything installed to run the project.


## 3.0.	Setting up and Running the Program

### 3.1.	Installing the Required Drivers:

1.	Source the ROS workspace:

<div align="center">

      source /opt/ros/noetic/setup.bash
</div>



2.	Make a directory for the catkin workspace where the project will be stored and enter it.
<div align="center">

      mkdir -p ${HOME}/catkin_ws/src && cd ${HOME}/catkin_ws
</div>


 
3.	Install the Universal Robots drivers to allow ROS to communicate with the UR3e:
<div align="center">

      git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver
</div>




4.	Install additional Universal Robots libraries:
<div align="center">

      git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git src/universal_robot
</div>




## 3.2.	Creating a Catkin Workspace:

1.	Update your package index:
<div align="center">

      sudo apt update -qq
</div>




2.	Update your ROS dependencies:
<div align="center">

      rosdep update
</div>



3.	Install newly required ROS dependencies:
<div align="center">

      rosdep install --from-paths src --ignore-src -y
</div>



4.	Create your catkin workspace:
<div align="center">

      catkin_make
</div>

## 3.3.	Calibrating the Robot:

Note: These steps will work wirelessly, but to eliminate connectivity issues, the arm should be connected to the device using Ethernet.

1.	Call the setup file from your catkin workspace:
<div align="center">

      source ${HOME}/catkin_ws/devel/setup.bash
</div>



2.	Now, power on the robot. 



<div style="text-align:center">
    <img src="URCAP_images/Picture1.png" alt="Power on robot" width="500" height="300" />
</div>
<div style="text-align:center">
    <img src="URCAP_images/Picture2.jpg" alt="Power on robot" width="500" height="300" />
</div>


3.	Launch the calibration file to calibrate the robot:
Note: your robot may be running on a different IP.
<div align="center">

      roslaunch ur_calibration calibration_correction.launch robot_ip:=10.0.1.1 target_filename:="${HOME}/tsoutsi_calibration.yaml"
</div>


When the terminal outputs “[calibration_correction-2] process has finished clearly”, press ctrl-c to exit.


## 3.4.	Creating a Launch File:

1.	Enter the src folder of your catkin workspace.
<div align="center">

      cd ${HOME}/catkin_ws/src
</div>


2.	Create a package for your launch file.
<div align="center">

      catkin_create_pkg cms_ur_launch ur_robot_driver -D "Package containing calibrations and launch files for our UR robots."
</div>





3.	Create a location for your launch file.
<div align="center">

      mkdir -p cms_ur_launch/etc
      mkdir -p cms_ur_launch/launch

</div>




4.	Move the launch file you created into the newly created folder.
<div align="center">

      mv ${HOME}/tsoutsi_calibration.yaml cms_ur_launch/etc
</div>



5.	Copy the drivers into the launch file.
<div align="center">

      roscp ur_robot_driver ur3e_bringup.launch cms_ur_launch/launch/tsoutsi.launch
</div>




6.	Edit the launch file to contain the following values:
<div align="center">

      <arg name="robot_ip" default="10.0.1.1"/>
      <arg name="kinematics_config" default="$(find cms_ur_launch)/etc/tsoutsi_calibration.yaml"/>

</div>



## 3.5.	Adding relevant files
In order to run the program successfully, you need to copy-paste some files into the catkin_ws.
1.	In the catkin_ws navigate to: catkin_ws/src/universal_robot/ur3e_moveit_config
2.	From git you can download plan_moves.py
3.	Copy paste this file into the ur3e_moveit_config
4.	From git u can also download ur3e.srdf
5.	In the catkin_ws navigate to: catkin_ws/src/universal_robot/ur3e_moveit_config/config and replace the contents of the existing ur3e.srdf or simply replace the file in there with the downloaded version.

## 3.6.	Connecting to the Robot:

Prerequisites
-	(Optional): the arm should be connected to the device using Ethernet. This will eliminate connectivity issues.
-	If you are not using an Ethernet connection, the robot and device should be on the same WiFi network.

### 3.6.1.	Finding your IP Address

If you already know your IP address, skip this step.

1.	Open the Activities overview and start typing “Settings”.
2.	Click on “Settings”.
3.	If you are using a wired (ethernet) connection, click on the “Network” panel.
4.	If you are using a wireless connection, click on “WiFi”.
5.	Click the cog button next to the active connection for the IP address.
6.	The IP address is in the form xxx.xxx.xxx.xxx.

### 3.6.2.	Changing the External Control IP Address

1.	Change the external control IP to your IP address on the robot tablet. Go to the Installation tab at the top pf the tablet, and then navigate to URCaps and then External Control.
<div style="text-align:center">
    <img src="URCAP_images/Picture3.png" alt="IP" width="500" height="300" />
</div>

 

3.6.3.	Launching the Robot

1.	Run the launch file on your device.
<div align="center">

      roslaunch cms_ur_launch tsoutsi.launch
</div>




2.	Start the tsoutsi-pc-control program on the robot tablet, by navigating to Run > Load Program> tsoutsi-pc-control.
<div style="text-align:center">
    <img src="URCAP_images/Picture4.png" alt="IP" width="500" height="300" />
</div>



You should see “Robot connected to reverse interface. Ready to receive control commands.” in your terminal. 


3.	In another terminal, cd to the “Executables” directory in our repository. Run the “execArm” executable.
<div align="center">

      ./execArm
</div>




Now, the connection is established, and you are ready to read the brain data.


## 3.7.	Using the Muse Sensor:
Now that the device has connected to the robot, we can record brain data using the sensor.

Prerequisites
-	The MuseLSL library must be installed. Instructions to do this are in the “Installing Necessary Python Libraries” section.

### 3.7.1.	Connecting the Sensor to the Device

1.	First, power on the sensor by holding the power button on the device. 

2.	Next, navigate to the “Executables” folder of our directory using cd. Run the execMuse executable:
<div align="center">

      ./execMuse
</div>



3.	A graph should appear on the screen if the sensor has been successfully connected to your device.




### 3.7.2.	Ensuring the Graph is Correct


A correct graph will have values from the TP9, AF7, AF8 and TP10 columns consistently ranging between 0 and 50. An incorrect graph will have differing values, with lines ranging from extremely high values to extremely low values.

You can tell when the sensor is worn correctly when the colour of the TP9, AF7, AF8 and AF10 columns on the graph is consistently green. 
Note: actions such as blinking and clenching the jaw may cause values to become extremely high or low. This is normal and will not affect the recording of the brain data.

Below is a diagram of the positions of the sensors on the head. This can be used to troubleshoot issues with the graph drawing incorrectly, by adjusting the incorrectly positioned sensor.

 
<div style="text-align:center">
    <img src="URCAP_images/Picture5.png" alt="IP" width="300" height="300" />
</div>

## 3.8.	Using the Brain Data to Control the Arm

### 3.8.1.	Recording Brain Data

 		These instructions are from the “codeinstruct.md” file in our repository:

-	You should be connected to the sensor first.
-	Navigate to the “Codes” folder in our repository.

1.	Run the Python file which will record 60 seconds of data from the sensor:
<div align="center">

      python muse.py
</div>



2.	The output file will be in the current directory, named “muse_data.csv”.

### 3.8.2.	Translating the Data into Pen Movements

Now that the corresponding brain data has been recorded, you are ready to run the main program.
(Note: The robot should already be set up, as described in the “Connecting to the Robot” section.)

Prerequisites
-	The pen should be attached to the end of the arm.
-	The canvas should be placed on the tray of the arm.

1.	In a new terminal, source the ROS environment:
<div align="center">

      source /opt/ros/noetic/setup.bash
</div>





2.	In the same terminal, run the command to start the program:
<div align="center">

      rosrun ur3e_moveit_config plan_moves.py
</div>



If everything was correctly executed, the arm should be drawing on the page!


## 3.9.	Installing the Digital program

Alongside the physical program, we have developed a digital prototype which draws the trajectory of the pen to the screen. This is intended to help you run the program without requiring constant access to the robotic arm. 

Prerequisites:
-	Python should be installed, along with all libraries detailed in the “Installing Necessary Python Libraries” section.
-	Our repository should be installed, detailed in the “Downloading the Brain Controlled Robot Painting” section.


### 3.9.1.	Virtual Programs

### 3.9.2.	Abstract program
1.	In the terminal, navigate to where the python files are located and run:
<div align="center">

      python3 abstract.py
</div>


2.	If running on mac, terminal will often ask for permission when the “Export button” is clicked, so ensure to give appropriate permissions.

### 3.9.3.	Faces program
1.	Ensure that the muse data is prerecorded and located in the same directory as the Python program.
2.	In the terminal, navigate to the directory where the python file is located, and type:
<div align="center">

      python3 faces.py
</div>


The digital prototype is located in our repository, in “Prototypes/Preliminary Prototypes”. There are different types of prototype:




## 4.0.	Troubleshooting

Here are a list of common issues we have faced with the program, and steps to fix them.

### 4.1.	Connecting to the Muse Sensor

If the Muse sensor will not connect to the device, try these steps:
-	Ensure the sensor is powered on.
-	Ensure the Muse sensor is not connected to another device: If the Muse is connected to a device, the lights located at the end of the sensor will be solid. If the Muse is not connected to a device, the lights will be flashing on and off. 


### 4.2.	Connecting to the Arm

If the UR3e will not connect to the device, try these steps:
-	Try pinging the robot, using the terminal command “ping” followed by the IP address of the robot, e.g. ping 10.0.1.1. If this fails, the robot and device are not connected the same network. Solve this issue by ensuring that the device is connected to the same WiFi network as the robot.
-	If the robot keeps disconnecting, try using an Ethernet cable.










<!-- USAGE EXAMPLES -->
## Usage

This project is intended to entertain young people and introduce them to Computer Science. It is a fun, interactive demonstration of how robotics and brain technology can be combined, and also an exploration in the role technology can play in creating unique art.

[Here is our introduction video!](https://www.youtube.com/watch?v=VzouSenkfsg)

_For more examples, please refer to the [Documentation](https://example.com)_


<p align="right">(<a href="#readme-top">back to top</a>)</p>


