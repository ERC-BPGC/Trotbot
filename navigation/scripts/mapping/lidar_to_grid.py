#! /usr/bin/env python2

import math, rospy
import numpy as np
import matplotlib.pyplot as plt
from math import cos ,sin ,radians ,pi
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
import tf.transformations as tft

# To be loaded from a YAML file
MIN_ANG = 0 #Minimum angle of 2D Lidar
MAX_ANG = 2*pi #Maximum Angle of 2D Lidar
MAX_HEIGHT = 15.0 #Range of Y-axis for mapping:(-MAX_HEIGHT/2 , MAX_HEIGHT/2)
MAX_WIDTH = 15.0 #Range of X-axis for mapping:(-MAX_WIDTH/2 , MAX_WIDTH/2)
XY_RESO = 0.2 #Grid Resolution of Probability map
SCAN_LEN = 360 # Length of your LaserScan Array
BOT_LENGTH = 0.5 # Max Length of your bot

#Globally declared based on YAML 
Y_RES = int(round(MAX_HEIGHT / XY_RESO)) # Y resolution
X_RES = int(round(MAX_WIDTH / XY_RESO)) # X resolution
RES = X_RES * Y_RES # Pixel Resolution of Probability map
INC = max(1 , int(BOT_LENGTH /(2 * XY_RESO))) #Obstacle Expansion purpose 

class Mapper(object):
    
    def __init__(self):
        self.ang = np.arange(MIN_ANG , MAX_ANG , (MAX_ANG - MIN_ANG)/SCAN_LEN)
    
    def __call__(self):
        self.scan_sb = rospy.Subscriber("/scan" , LaserScan , self.scan_cb)
        self.grid_map_pub = rospy.Publisher("/gridmap" , OccupancyGrid , queue_size = 10)
        self.grid_pub_msg = OccupancyGrid()
        self.present_pos = Pose() 
        self.present_pos.position.x -= (MAX_HEIGHT/2)
        self.present_pos.position.y -= (MAX_WIDTH/2)
        print "Initialized"
        rospy.spin()

    def scan_cb(self , data):
        """
        Reading Lidar Beams from /scan topic
        Args: 
            data from LaserScan Subscriber

        Return:
            None 
        """        
        prob_map = Mapper.main(self , data.ranges)
        # print(prob_map.flatten()*100)
        self.grid_pub_msg.header.frame_id = "base_scan"
        self.grid_pub_msg.info.resolution = XY_RESO
        self.grid_pub_msg.info.width = X_RES
        self.grid_pub_msg.info.height = Y_RES
        self.grid_pub_msg.info.origin = self.present_pos
        self.grid_pub_msg.data = list((prob_map.flatten()*100))
        self.grid_map_pub.publish(self.grid_pub_msg) 


    
    @staticmethod
    def main(self, scan):
        """
        Generating the probability map
        Args: 
            List of ranges from scan

        Return:
            Probability map 
        """   
        # (ox , oy) are list of obstacles
        scan = np.array(scan)
        scan = np.where(scan == np.inf , 27.0 , scan)

        ox = np.cos(self.ang) * scan
        oy = np.sin(self.ang) * scan
        
        #Initialize and empty gridmap
        pmap = np.zeros((Y_RES , X_RES))

        #centx,centy centre of gridmap
        centx = int(round(X_RES/2) - 1)
        centy = int(round(Y_RES/2) - 1)
        #0.8 around obstacle. 1 actual location of obstacle
        
        #Map area around the object with prob 0.8
        for (x,y) in zip(ox , oy):
            try:
                ix = centx + int(round(x/XY_RESO))
                iy = centy + int(round(y/XY_RESO))
                pmap[iy-INC:iy+INC+1 , ix-INC:ix+INC+1] = 0.8
            except:
                pass
        #Map actual location of 
        for (x,y) in zip(ox , oy):
            try:
                ix = centx + int(round(x/XY_RESO))
                iy = centy + int(round(y/XY_RESO))
                pmap[iy , ix] = 1
            except:
                pass
        return pmap

    def animation(self, prob_map):
        """
        matplotlib figure of probability map
        Args: 
            probability map

        Return:
            None 
        """   
        #centx,centy centre of gridmap
        centx = int(round(X_RES/2) - 1)
        centy = int(round(Y_RES/2) - 1)
        
        prob_map[centx , centy] = 0.5

        plt.imshow(prob_map , cmap="seismic")
        plt.clim(-0.4, 1.4)
        plt.gca().set_xticks(np.arange(-.5, X_RES, 1), minor=True)
        plt.gca().set_yticks(np.arange(-.5, Y_RES, 1), minor=True)
        plt.grid(True, which="minor", color="k", linewidth=0.6, alpha=0.5)
        plt.colorbar()
        plt.show()
        



if __name__ == "__main__":
    rospy.init_node("mapper" , anonymous=True)
    o = Mapper()
    o()

    