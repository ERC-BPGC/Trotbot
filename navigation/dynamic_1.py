#!/usr/bin/env python

"""

"""

from shapely.geometry import Point
from shapely.geometry import Polygon
from descartes import PolygonPatch


import matplotlib.pyplot as plt
import numpy as np
import time
import random

class Node():
    """
    RRT Node
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


'''LIST OF OBSTACLES'''

obstacles = []

obstacles.append(Polygon([(1,1), (3,3), (1,3)]))
obstacles.append(Polygon([(5,4), (4,4), (4,5), (5,6)]))

#---------------------------------
'''PATH'''
RRTpath = []
pointPath = []


def genRandPath(num_pts):
    '''Generates random path (of RRT format) of length num_pts'''
    # print("Random path:")
    mn=0
    mx=10
    for i in range(num_pts):
        a = Node(random.randint(mn, mx)/10.0, random.randint(mn, mx)/10.0)
        mx+=10
        mn+=10
        RRTpath.append(a)
        if i >0:
            RRTpath[i].parent = RRTpath[i-1]
        # print( RRTpath[i].x,  RRTpath[i].y )
        pointPath.append(Point( RRTpath[i].x,  RRTpath[i].y))
    # print("")

genRandPath(8)
# for p in pointPath:
#     # print p



#----------------------------------
'''LIST OF CENTROIDS'''

t = time.time()

centroids = []

for obs in obstacles:
    centroids.append(obs.centroid)

plt.axes()
# print("")
# print( "Centroids: ")
# for p in centroids:
    # print p

safety_radii =[]
for o in obstacles:
    safety_radius = 0
    vertices = list(o.exterior.coords)
    for v in vertices:
        safety_radius = max(Point(v).distance(o.centroid), safety_radius)

    safety_radii.append(safety_radius)
    #print safety_radius

# print ("Safety radii: ", safety_radii)
# print("")

#-----------------------------------
''' COLLISION CHECK '''
def dynamic_collisionCheck():
    for o in obstacles:
        for p in pointPath:
            if o.contains(p):
                #pass
                print("Coll")
            else:
                #pass
                print("Out")
        # print("")

dynamic_collisionCheck()
#-----------------------------------
'''TIME TAKEN'''
print("END STATS: ")
print("No. of Obstacles: ", len(obstacles))
print ("Time: " + str(time.time()-t))
print("")
#----------------------------------
'''GRAPH'''

fig = plt.figure(1, figsize=(5,5), dpi=90)
ax = fig.add_subplot(111)
for obs in obstacles:
    patch = PolygonPatch(obs)
    ax.add_patch(patch)
plt.axis([-1, 8, -1, 8])
for pn in RRTpath:
    plt.plot(pn.x, pn.y, "ok",300)
plt.show()
