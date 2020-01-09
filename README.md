# Trotbot

## About 
<p align="center">
  <img src="https://github.com/ERC-BPGC/Trotbot/blob/master/Trotbot.jpg" alt="drawing" width="400"/>
</p>

**Trotbot** is a robot designed to serve as a delivery robot in an indoor environment. The deliverables will be placed inside the container. It will be able to autonomously navigate around obstacles and reach waypoints set in and around its environment. Identification tags will be used by the Bot to locate the rooms and localize itself.


## Prerequisites:
- [ROS Kinetic](http://wiki.ros.org/kinetic/Installation) on Ububtu 16.04
- [Python catkin-tools](https://catkin-tools.readthedocs.io/en/latest/installing.html) 
- [librealsense](https://github.com/IntelRealSense/librealsense)
- [rplidar_ros](https://github.com/Slamtec/rplidar_ros)

To install the python dependencies:
```bash
pip2 install -r requirements.txt
sudo apt-get install ros-kinetic-turtrlebot3-*
```


To use this repo do:

```bash
cd trotbot_ws/src
git clone https://github.com/ERC-BPGC/Trotbot.git 
cd ..
catkin build
source devel/setup.bash

# run scripts from the navigation package
```

**Note**: The packages have been tested with ROS Kinetic on Ubuntu 16.04.

-----------------------

## Contributors:
- Atharv Sonawane 
- Ojit Mehta
- Rishikesh Vanarse
- Harshal Deshpande
- Mihir Dharmadhikari
- Mohit Gupta
- Atman Kar
