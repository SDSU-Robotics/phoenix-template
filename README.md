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
 4. `cd ~/catkin_ws`
 5. `catkin_make`
  - Only needs to be done once for setup.  Call whenever you edit source code.

 ## Enable Hot Swapping
 1. Connect USB CAN interface (if not already connected)
 2. Set up permissions
     - `git clone https://github.com/CrossTheRoadElec/Phoenix-Linux-SocketCAN-Example.git`
     - `cd ./Phoenix-Linux-SocketCAN-Example/.`
     - `chmod +x build.sh`
     - `chmod +x clean.sh`
     - `chmod +x canableStart.sh`

 3. Edit interfaces
     - `cd /etc/network/.`
     - `sudo gedit interfaces` (if that doesnt work, try `sudo nano interfaces`)
     - Add these lines below the text already in the file and save (Each point is one line)

       - `allow-hotplug can0`
       - `iface can0 can static`
       - `bitrate 1000000`
       - `txqueuelen 1000`
       - `up /sbin/ip link set $IFACE down`
       - `up /sbin/ip link set $IFACE up type can`
     - Disconect and reconnect the CAN interface.  The light will turn green if it is connected properly

## Running
 1. Connect game controller (if not already connected)
 2. Call launch file
	- `roslaunch phoenix-template MotorTest.launch`
    - Motor controller lights will turn to flashing orange
