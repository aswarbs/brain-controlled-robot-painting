

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

#### Hardware Requirements

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


#### Installing the Required Drivers:

1.	Source the ROS workspace:

<div align="center">

      source /opt/ros/noetic/setup.bash
</div>



2.	Make a directory for the catkin workspace where the project will be stored and enter it.
<div align="center">

      mkdir -p $(path)/brain-controlled-robot-painting/catkin_ws/src && cd $(path)/brain-controlled-robot-painting/catkin_ws
</div>


 
3.	Install the Universal Robots drivers to allow ROS to communicate with the UR3e:
<div align="center">

      git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver
</div>




4.	Install additional Universal Robots libraries:
<div align="center">

      git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git src/universal_robot
</div>




#### Creating a Catkin Workspace:

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

#### Calibrating the Robot:

Note: These steps will work wirelessly, but to eliminate connectivity issues, the arm should be connected to the device using Ethernet.

1.	Call the setup file from your catkin workspace:
<div align="center">

      source $(path)/brain-controlled-robot-painting/catkin_ws/devel/setup.bash
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


#### Creating a Launch File:

1.	Enter the src folder of your catkin workspace.
<div align="center">

      cd $(path)/brain-controlled-robot-painting/catkin_ws/src
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



#### Adding relevant files
In order to run the program successfully, you need to paste some files into the catkin_ws.
1.	Navigate to: brain-controlled-robot-painting/catkin_ws/src/universal_robot/ur3e_moveit_config
2.	From git you can download plan_moves.py
3.	Copy paste this file into the ur3e_moveit_config
4.	From git you can also download ur3e.srdf
5.	In the catkin_ws navigate to: brain-controlled-robot-painting/catkin_ws/src/universal_robot/ur3e_moveit_config/config and replace the contents of the existing ur3e.srdf.

#### Connecting to the Robot:

Prerequisites
-	(Optional): the arm should be connected to the device using Ethernet. This will eliminate connectivity issues.
-	If you are not using an Ethernet connection, the robot and device should be on the same WiFi network.

##### Finding your IP Address

If you already know your IP address, skip this step.

1.	Open the Activities overview and start typing “Settings”.
2.	Click on “Settings”.
3.	If you are using a wired (ethernet) connection, click on the “Network” panel.
4.	If you are using a wireless connection, click on “WiFi”.
5.	Click the cog button next to the active connection for the IP address.
6.	The IP address is in the form xxx.xxx.xxx.xxx.

#### Changing the External Control IP Address

1.	Change the external control IP to your IP address on the robot tablet. Go to the Installation tab at the top pf the tablet, and then navigate to URCaps and then External Control.
<div style="text-align:center">
    <img src="URCAP_images/Picture3.png" alt="IP" width="500" height="300" />
</div>

 

#### Launching the Robot

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

*Note*: If using the GUI, you can navigate to the Physical Program Menu and follow the steps there, rather than using terminal.


#### Troubleshooting

Here are a list of common issues we have faced with the program, and steps to fix them.

#### Connecting to the Muse Sensor

If the Muse sensor will not connect to the device, try these steps:
-	Ensure the sensor is powered on.
-	Ensure the Muse sensor is not connected to another device: If the Muse is connected to a device, the lights located at the end of the sensor will be solid. If the Muse is not connected to a device, the lights will be flashing on and off. 


#### Connecting to the Arm

If the UR3e will not connect to the device, try these steps:
-	Try pinging the robot, using the terminal command “ping” followed by the IP address of the robot, e.g. ping 10.0.1.1. If this fails, the robot and device are not connected the same network. Solve this issue by ensuring that the device is connected to the same WiFi network as the robot.
-	If the robot keeps disconnecting, try using an Ethernet cable.


<!-- USAGE EXAMPLES -->
## Usage

This project is intended to entertain young people and introduce them to Computer Science. It is a fun, interactive demonstration of how robotics and brain technology can be combined, and also an exploration in the role technology can play in creating unique art.

[Here is our introduction video!](https://www.youtube.com/watch?v=VzouSenkfsg)

_For more examples, please refer to the [Documentation](https://example.com)_


<p align="right">(<a href="#readme-top">back to top</a>)</p>


