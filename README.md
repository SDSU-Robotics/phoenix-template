# phoenix-template

## Prerequisites

 1. Install necessary tools and utilities
     - `sudo apt install git`
	 - `sudo apt install net-tools`
	 - `sudo apt install can-utils`
 2. Install ROS Melodic
     - http://wiki.ros.org/melodic/Installation/Ubuntu
     - Perform full desktop installation
 3. Install necessary ROS packages
     - `sudo apt install ros-melodic-joy`
 
## Setup
 1. If you do not have a catkin workspace set up, create one:
     - `mkdir -p ~/catkin_ws/src`
     - `cd ~/catkin_ws/src`
     - `echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc`
     - `source ~/.bashrc`
 2. Clone repo into user directory `git clone https://github.com/SDSU-Robotics/phoenix-template.git`.
 3. Navigate into repo `cd phoenix-template`.

## Building
 1. `cd ~/catkin_ws`
 2. `catkin_make`

 ## Running
 1. Connect USB CAN interface (if not already connected)
 2. Initialize CAN interface
	 - `~/catkin_ws/src/phoenix-template/scripts/canableStart.sh`
	 - If there is an error, try running it again.
	 - If it still doesn't work, check the device name with `ls /dev` and ensure that the device name in the `canableStart.sh` script matches.
 3. Connect game controller (if not already connected)
 4. Call launch file
	- `roslaunch phoenix-template MotorTest.launch`
