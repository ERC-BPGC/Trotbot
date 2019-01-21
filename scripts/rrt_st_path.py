
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

    def st_linecheck(self,newNode,d):
        st_count=0
        st_nodelist=self.nodeList[:]
        st_theta=math.atan2(self.end.y-newNode.y,self.end.x-newNode.x)
        st_distance=d
        iterations=int(st_distance/self.mnl)
        st_start=len(st_nodelist)
        st_cos=math.cos(st_theta)
        st_sin=math.sin(st_theta)
        for i in range(1,iterations):
            st_x = newNode.x + (self.mnl)*i*st_cos
            st_y = newNode.y + (self.mnl)*i*st_sin

            st_newnode=Node(st_x,st_y)
            if not self.CollisionCheck(st_newnode, self.obstacleList, self.obstacleList2):
                st_count=-1
                break
        st_count+=1
        return st_count



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
    min_in=0.1
    temp=0
    current_index=0
    while current_index < (len(f_path_i)):
        if current_index==f_path_i_len-1 :
            break
        cu_y=f_path_i[current_index][1]
        cu_x=f_path_i[current_index][0]
        for check_index in range(current_index+1,f_path_i_len):
            ch_x=f_path_i[check_index][0]
            ch_y=f_path_i[check_index][1]
            alpha=math.atan2(ch_y-cu_y,ch_x-cu_x)
            sin=math.sin(alpha)
            cos=math.cos(alpha)
            f_dist=math.sqrt((ch_y-cu_y)**2+(ch_x-cu_x)**2)
            f_iter=int(f_dist/min_in)
            col_check=0
            for k in range(1,f_iter+1):
                f_x=cu_x+min_in*k*cos
                f_y=cu_y+min_in*k*sin
                f_node=Node(f_x,f_y)
                if not RRT.CollisionCheck(f_node,ol1,ol2):
                    col_check+=1
                    break
            if col_check==0:
                temp=check_index
        f_path_o.append(f_path_i[temp])
        current_index=temp
        temp=temp+1
    return f_path_o


def main():
    print("start simple RRT path planning")

    # ====Search Path with RRT====
    obstacleList = [
        (5, 5, 1),
        (3, 6, 2),
        (2,8,0.5),
        # (8,2,2),
        (0,4,1),
        # (7,2,0.5),
        (6,3,2),
        (8,6,1),
        # (6,8,2)
    ]  # [x,y,size]
    obstacleList2 = [
        # ((1.7071067811865475, 0.29289321881345254), (2.7071067811865475, 1.2928932188134525), (3.7071067811865475, 2.2928932188134525), (2.2928932188134525, 3.7071067811865475), (1.2928932188134525, 2.7071067811865475), (0.2928932188134524, 1.7071067811865475))
    ]

    # Set Initial parameters
    rrt = RRT(start=[0, 0], goal=[5, 10],
              randArea=[-2, 15], obstacleList=obstacleList, obstacleList2=obstacleList2)
    path = rrt.Planning(animation=show_animation)
    path_in=list(reversed(path[:]))
    final_path_r=final_path(path_in,obstacleList,obstacleList2)
    print("this is final path ->")
    print(final_path_r)
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
	main()
