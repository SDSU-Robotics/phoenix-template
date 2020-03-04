# phoenix-template

## Prerequisites
 1. Install necessary tools and utilities
     - `sudo apt install git`
	 - `sudo apt install net-tools`
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
 2. Clone repo into src directory. Make sure you are in catkin_ws/src.
    - `git clone https://github.com/SDSU-Robotics/phoenix-template.git`.
 3. Build repository
    - `cd ~/catkin_ws; catkin_make`
 4. Source executables
    - `source ~/.bashrc`

 ## Enable Hot Swapping
Configure Interface
- `sudo gedit /etc/network/interfaces` (if that doesnt work, try `nano` in plasce of `gedit`)
- Add these lines below the text already in the file and save (Each point is one line)

    `allow-hotplug can0`

    `iface can0 can static`

    `bitrate 1000000`

    `txqueuelen 1000`

    `up /sbin/ip link set $IFACE down`

    `up /sbin/ip link set $IFACE up type can`
- A warning about not being able to save metadata will show up, that is fine.

Connect the CAN interface.  The light will turn green if it is configured properly

## Running
 1. Connect CAN interface
 2. Connect game controller
 3. Call launch file
	- `roslaunch phoenix-template TalonTest.launch`
    - Motor controller lights will turn to flashing orange
    - Replace `TalonTest.launch` with the name of whatever launch file you wan to run.
