
#!/usr/bin/env python

"""
RRT with pre-fed lists of polygons and circles.
"""

import matplotlib.pyplot as plt
import random
import math
import copy
import time

from shapely.geometry import Polygon
from shapely.geometry import Point,LineString
from descartes import PolygonPatch


#show_animation = False


class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacleList, obstacleList2,
                 randArea, expandDis=1.0, goalSampleRate=15, maxIter=500,mnl=0.01):
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

    def st_linecheck(self,newNode,d):
        line=LineString([(self.end.x,self.end.y),(newNode.x,newNode.y)])
        ob=[Polygon(list(x)) for x in self.obstacleList2[:]]
        for obst in ob:
            if line.intersects(obst):
                return 0
        return 1


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

            if not self.CollisionCheck(node=newNode,obstacleList= self.obstacleList,obstacleList2= self.obstacleList2):
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
            if d > self.expandDis:
                st_count=self.st_linecheck(newNode,d)
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
            ax=plt.gca()
            ax.add_artist(plt.Circle((ox,oy), size,color="b"))
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
        plt.pause(0.000000000000000000000000001)

    def GetNearestListIndex(self, nodeList, rnd):
        dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1])
                 ** 2 for node in nodeList]
        minind = dlist.index(min(dlist))
        return minind

    @staticmethod
    def CollisionCheck(node, obstacleList, obstacleList2):

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

    def __str__(self):
        return ("["+str(self.x)+','+str(self.y)+"]")

def final_path(f_path_i,ol1,ol2):
    f_path_o=[]
    f_path_o.append(f_path_i[0])
    f_path_i_len=len(f_path_i)
    if f_path_i_len<5:
        return f_path_i
    # temp=1
    current_index=0
    ob=[Polygon(list(x)) for x in ol2]
    # for p in ob:
        # print (list((p.exterior.coords)))

    while current_index < f_path_i_len-2:

        # print("in while")
        for temp in range(current_index+1,f_path_i_len-1):

            flag=0
            # print(current_index)

            line_check=LineString([(f_path_i[temp][0],f_path_i[temp][1]),(f_path_i[current_index][0],f_path_i[current_index][1])])
            # print(line_check.length)
            for obst in ob:
                if line_check.intersects((obst)):
                    # print("obstacle")
                    if temp==current_index+1:
                        temp2=temp
                    flag=1
                    break
            if flag==0:
                temp2=temp

        f_path_o.append([f_path_i[temp2][0],f_path_i[temp2][1]])
        current_index=temp2
    f_path_o.append(f_path_i[-1])    #     break# f_path_o.append(f_path_i[-1])
    return f_path_o


def do_RRT(obstacleList2, show_animation, start_point_coors , end_point_coors):
    print("start simple RRT path planning")
#	obstacleList=[]

    # ====Search Path with RRT====
#    obstacleList = [
#        (5, 5, 0),
#    ]  # [x,y,size]
    obstacleList=[]
    obstacleList2=[]
    '''
    obstacleList2 = [
         ((1.7071067811865475, 0.29289321881345254), (2.7071067811865475, 1.2928932188134525), (3.7071067811865475, 2.2928932188134525), (2.2928932188134525, 3.7071067811865475), (1.2928932188134525, 2.7071067811865475), (0.2928932188134524, 1.7071067811865475))
    ]
    '''




    # Set Initial parameters
    rrt = RRT(start= start_point_coors, goal= end_point_coors,
              randArea=[-5, 5], obstacleList= obstacleList, obstacleList2=obstacleList2)
    path = rrt.Planning(animation=show_animation)
    path_in=list(reversed(path[:]))
    final_path_r=final_path(path_in,obstacleList,obstacleList2)
    final_path_r = [tuple(coors) for coors in final_path_r]
    print("this is final path ->")
    print(final_path_r)
    return final_path_r
# Draw final path
    if show_animation:
        rrt.DrawGraph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.plot([x for (x, y) in final_path_r], [y for (x, y) in final_path_r], '-m')
        plt.grid(True)
        for i in (path_in):
            plt.plot(i[0], i[1], marker='x', markersize=3, color="blue")
        for i in (final_path_r):
            plt.plot(i[0], i[1], marker='o', markersize=7, color="yellow")
        plt.show()


if __name__ == '__main__':
    start = time.time()
    do_RRT(show_animation = True , start_point_coors = [0 , 0] , end_point_coors = [5 , 10] , obstacleList2 = [((1,1), (3,3), (1,3)) , (((5,4), (4,4), (4,5), (5,6)))])
    print(time.time() - start)
