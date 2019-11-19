#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from navigation.msg import PointArray , PolyArray
import jsk_recognition_msgs.msg import PolygonArray

class main(object):
    def __init__(self):
        self.scan_sub = rospy.Subscriber("/scan" , LaserScan , self.scan_cb)
        self.TESTPUB = rospy.Publisher("/test" , LaserScan , queue_size=10)
        self.rate = rospy.Rate(1)
        self.inf = float("inf")
        self.scan_n = LaserScan()
        self.scan_n.header.frame_id="base_scan"

    def scan_cb(self , scan_data):
        self.scan_n = scan_data

        np_scan = np.array(list(scan_data.ranges))
        a = np.where(np_scan == self.inf , np.nan , np_scan)
        i_ = np.arange(360)*np.pi/180
        a_x = a*np.cos(i_)
        a_y = a*np.sin(i_)

        a_x = np.resize(a_x,(360,1))
        a_y = np.resize(a_y,(360,1))
        a_xy = np.append(a_x , a_y , axis=1)
        # print(self.scan_n)
        a_pxy=np.append(a_xy[1:] , a_xy[0:1] , axis=0)
        a_nxy=np.append(a_xy[-1:-2 :-1] , a_xy[:-1] , axis=0)

        assert(a_pxy[-1].all()==a_nxy[1].all())

        # inf_i_ = np.where(a==self.inf , True , False)

        slp = a_xy - a_pxy
        # print("TEST A_XY")
        # print(a_xy)
        # print("*"*20)
        # print("TEST A_PXY")
        # print(a_pxy)
        # print("*"*20)
        # print("TEST SLOPE")
        # print(slp)

        slope_p = slp[:,0]/slp[:,1]

        # slope_n = np.append(slope_p[1:],slope_p[0])

        _ang = np.arctan(slope_p)
        ang_p = np.where(_ang < 0 , np.pi + _ang , _ang)
        ang_n = np.append(ang_p[1:] , ang_p[0])


        ext_ang = np.array(list(map(self.d_ang , ang_p , ang_n)))

        ext_x_n = (a_xy[:,0]+2*np.cos(ext_ang)).reshape((360,1))
        ext_y_n = (a_xy[:,1]+2*np.cos(ext_ang)).reshape((360,1))
        ext_n_cart = np.append(ext_x_n , ext_y_n , axis=1)

        ext_n_polar = np.sqrt(ext_n_cart[:,0]**2+ext_n_cart[:,1]**2)
        ext_n_polar[np.isnan(ext_n_polar)]=100

        self.scan_n.ranges = list(ext_n_polar)
        self.TESTPUB.publish(self.scan_n)

        self.rate.sleep()

    def d_ang(self, A , B):
        if (A>np.pi/2 and B>np.pi/2):
            return (A+B)/2 - np.pi/2
        elif (A<np.pi/2 and B<np.pi/2):
            return (A+B)/2 + np.pi/2
        else:
            return (A+B)/2

if __name__=="__main__":
    o=main()
    rospy.spin()