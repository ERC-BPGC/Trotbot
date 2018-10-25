#!/usr/bin/env python

"""
RRT with pre-fed lists of polygons and circles. 
"""

import matplotlib.pyplot as plt
import random
import math
import copy

from shapely.geometry import Polygon 
from shapely.geometry import Point
from descartes import PolygonPatch


show_animation = True


class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacleList, obstacleList2,
                 randArea, expandDis=1.0, goalSampleRate=10, maxIter=500,mnl=0.1):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Ramdom Samping Area [min,max]

        """
        self.start = Node(start[0], start[1])
        self.end = Node(goal[0], goal[1])
        self.minrand = randArea[0]
        self.maxrand = randArea[1]
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.obstacleList = obstacleList
        self.obstacleList2 = obstacleList2
        self.mnl=mnl

    def Planning(self, animation=True):
        """
        Pathplanning

        animation: flag for animation on or off
        """

        self.nodeList = [self.start]
        while True:
            # Random Sampling
            if random.randint(0, 100) > self.goalSampleRate:
                rnd = [random.uniform(self.minrand, self.maxrand), random.uniform(
                    self.minrand, self.maxrand)]
            else:
                rnd = [self.end.x, self.end.y]

            # Find nearest node
            nind = self.GetNearestListIndex(self.nodeList, rnd)
            # print(nind)

            # expand tree
            nearestNode = self.nodeList[nind]
            theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)    # returns angle made with x-axis by a vector by from origin to (x,y)

            newNode = copy.deepcopy(nearestNode)
            newNode.x += self.expandDis * math.cos(theta)
            newNode.y += self.expandDis * math.sin(theta)
            newNode.parent = nind

            if not self.__CollisionCheck(newNode, self.obstacleList, self.obstacleList2):
                continue

            self.nodeList.append(newNode)
            print("nNodelist:", len(self.nodeList))

            # check goal
            dx = newNode.x - self.end.x
            dy = newNode.y - self.end.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= self.expandDis:
                print("Goal!!")
                break

            if animation:
                self.DrawGraph(rnd)
       #straight line check
            st_count=0
            
            if d>self.expandDis:
            	st_nodelist=self.nodeList[:]
            	st_theta=math.atan2(self.end.y-newNode.y,self.end.x-newNode.x)
            	st_distance=d
            	iterations=int(st_distance//self.mnl)
            	st_start=len(st_nodelist)
            	
            	for i in range(1,iterations):
            		st_x = newNode.x + (self.mnl)*i*math.cos(st_theta)
            		st_y = newNode.y + (self.mnl)*i*math.sin(st_theta)
            		
            		st_newnode=Node(st_x,st_y)
            		st_newnode.parent=st_start+i-2
            		if not self.__CollisionCheck(st_newnode, self.obstacleList, self.obstacleList2):
            			break
            		st_nodelist.append(st_newnode)
            		
            		st_dx = st_newnode.x - self.end.x
            		st_dy = st_newnode.y - self.end.y
            		st_d = math.sqrt(st_dx * st_dx + st_dy * st_dy)
            		if st_d <= self.expandDis:
            			st_count+=1
            			self.nodeList=st_nodelist
            			print("Goal!!")
            			break
            if st_count!=0:
            	break
        
        
        path = [[self.end.x, self.end.y]]
        lastIndex = len(self.nodeList) - 1
        while self.nodeList[lastIndex].parent is not None:
            node = self.nodeList[lastIndex]
            path.append([node.x, node.y])
            lastIndex = node.parent
        path.append([self.start.x, self.start.y])

        return path

    def DrawGraph(self, rnd=None):
        """
        Draw Graph
        """
        plt.clf()
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^k")
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.x, self.nodeList[node.parent].x], [
                         node.y, self.nodeList[node.parent].y], "-g")

        for (ox, oy, size) in self.obstacleList:
            plt.plot(ox, oy, "ok", ms=30 * size)
        for i in self.obstacleList2:
            poly=Polygon(i)
            fig = plt.figure(1, figsize=(5,5), dpi=90)
            ax = fig.add_subplot(111)
            poly_patch = PolygonPatch(poly)
            ax.add_patch(poly_patch)
        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.001)

    def GetNearestListIndex(self, nodeList, rnd):
        dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1])
                 ** 2 for node in nodeList]
        minind = dlist.index(min(dlist))
        return minind

    def __CollisionCheck(self, node, obstacleList, obstacleList2):

        for (ox, oy, size) in obstacleList:
            dx = ox - node.x
            dy = oy - node.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= size:
                return False  # collision
        for i in obstacleList2:
            poly=Polygon(i)
            nodepoint = Point(node.x, node.y)
            if nodepoint.within(poly):
                return False

        return True  # safe


class Node():
    """
    RRT Node
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


def main():
    print("start simple RRT path planning")

    # ====Search Path with RRT====
    obstacleList = [
        (5, 5, 1),
        (3, 6, 2),
        (3, 8, 2),
        (3, 10, 2),
        (7, 5, 2),
        (9, 5, 2)
    ]  # [x,y,size]
    obstacleList2 = [
        ((4,4),(6,4),(6,-2),(4,-2))
    ]

    # Set Initial parameters
    rrt = RRT(start=[0, 0], goal=[5, 10],
              randArea=[-2, 15], obstacleList=obstacleList, obstacleList2=obstacleList2)
    path = rrt.Planning(animation=show_animation)

    # Draw final path
    if show_animation:
        rrt.DrawGraph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.show()


if __name__ == '__main__':
    main()
