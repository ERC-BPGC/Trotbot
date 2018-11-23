Arduino interface to MCS-12085/12086 Optical Mouse Sensor

Connect SDIO/SCLK to pins defined in mcs-12085.cpp and then call
mcs12085_dx() and mcs12085_dy() to read distance mouse has moved since
last call.


#Procedure to run obstacle_expansion:
1.Create a catkin package named obstacle_expander 
(dependencies:message_runtime
rospy
jsk_recognition_msgs
sensor_msgs
geometry_msgs
)

2.copy the three message files to msg folder and source files to scripts folder


3.edit the CMakelists
