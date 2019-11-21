# Trotbot

![trotbot](Trotbot.jpg "trotbot")

## About 
**Trotbot** is a robot designed to serve as a delivery robot in an indoor environment. The deliverables will be placed inside the container. It will be able to autonomously navigate around obstacles and reach waypoints set in and around its environment. Identification tags will be used by the Bot to locate the rooms and localize itself.

## Prerequisites:
- [ROS Kinetic](http://wiki.ros.org/kinetic/Installation) on Ububtu 16.04
- [Python catkin-tools](https://catkin-tools.readthedocs.io/en/latest/installing.html) 
- shapely 
- descartes
- numpy

To install the dependencies:
```
pip2 install --user $LIBRARY_NAME
```


To use this repo do:

```
cd trotbot_ws/src
git clone https://github.com/ERC-BPGC/Trotbot.git 
cd ..
catkin build
source devel/setup.bash

# run scripts from the navigation package
```

-----------------------

## Contributors:
- Atharv Sonawane 
- Ojit Mehta
- Rishikesh Vanarse
- Harshal Deshpande
- Mihir Dharmadhikari
- Mohit Gupta
- Atman Kar