#! /usr/bin/env python

""" A* Path Planning Algorithm for Lidar LaserScan
    Author: Ojit Mehta (@ojitmehta123) """
import time
import sys
import os
import numpy as np
from collections import deque
import  matplotlib
import matplotlib.pyplot as plt
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../mapping')))
try:
    from lidar_to_grid import Mapper
except:
    raise ImportError("Mapper from lidar_to_grid cannot be imported")

show_animation = False

class Node:
    """
    Nodes for AStar
    """
    def __init__(self, x, y, cost, par):
        self.x = x  # index of grid
        self.y = y  # index of grid
        self.cost = cost # Cost to reaach that index
        self.par = par # Parent 

    def __str__(self):
        """ For printing purpose """
        try:
            return str(self.x) + "," + str(self.y) + "," + str(self.cost) + ",[" + str(self.par.x) + "," + str(self.par.y) + "]"
        except:
            return str(self.x) + "," + str(self.y) + "," + str(self.cost) + " , None"
    
    @property
    def get_pos(self):
        """ Getter of the class """
        return np.array([self.x , self.y])

    def __eq__(self , c):
        """ While Equating this node to c  """
        return (c.x , c.y) == (self.x,self.y)

class AStar(object):
    """ AStar Algorithm """
    def __init__(self):
        pass
    def __call__(self , prob_map , start , end , max_iter = 15000 , show_animation = False):
        self.start = start # Start Position
        self.end = end # End Position
        self.prob_map = prob_map # Obstacle Probability map
        self.x_obst = [] # xcordinates of obstacle to be used only for graphing purpose
        self.y_obst = [] # ycordinates of obstacle to be used only for graphing purpose
        #Shape of prob_map
        self.x_max , self.y_max = np.shape(self.prob_map)
        self.max_iter = max_iter # Max iterations to run before path not found error
        # Setting X , Y Obstacles
        for i in range( np.shape(self.prob_map)[0]):
            for j in range( np.shape(self.prob_map)[1] ):
                if self.prob_map[i,j]>0:
                    self.x_obst.append(j)
                    self.y_obst.append(i) 
        

        #Initializing all the visited(none) and unvisited nodes (Start so far)
        self.visited_nodes = []
        self.unvisited_nodes = [Node(start[0] , start[1] , 0 , Node(None , None , 0 , None))]

    def find_path(self):
        """ Finds Path from self.start to self.end taking obstacles tin account from probability map"""
        # For Now the goal should not be in obstacle
        if self.prob_map[self.end[0] , self.end[0]] > 0 :
            raise ValueError("Goal lies inside Obstacle")
        
        check_node = Node(self.start[0] , self.start[1] , 0 , None)
        #Keep Iterating till You find end or iteration runs out
        iter = 0
        while Node(self.end[0] , self.end[1] , 0 , 0) not in self.visited_nodes and iter < self.max_iter:
            neighbours = self.get_neihbour_nodes(check_node) #Check neighbouring nodes of check_node 
            check_node_ind = np.argmin([p.cost + np.sqrt(np.sum((p.get_pos - np.array(self.end))**2))  for p in self.unvisited_nodes ]) #Get the least cost node and make that check_node
            check_node = self.unvisited_nodes[check_node_ind] 
            
            iter+=1
        
        if iter != self.max_iter:
            print("Path found in " + str(iter) +" iterations")
        
        #Get the final path and return
        
        path =  self.get_final_path()
        return path

    def get_final_path(self):
        """ Gets final path from visited nodes """
        try:
            # End node if exists to start node
            last_node = self.visited_nodes[self.visited_nodes.index(Node( self.end[0] , self.end[1] , 0 , 0))]
            final_path = [(last_node.y , last_node.x)] #Because of inverted grid map formed from LaserScan we are appending the y first then x 

            #Keep iterating till the parent is not None because start parent was None
            while last_node.par is not None:
                final_path.append((last_node.y , last_node.x))
                last_node = last_node.par
            
            # print(final_path)
        
            return final_path

        except:
            return -1

    def get_neihbour_nodes(self , p_node):
        """ |n  n   n|
            |n  p   n|
            |n  n   n|
            
            p-->parent
            n--> neighbour of p
            
            Add all n's to unvisited nodes if not in obstacle
            Remove p from unvisited and add to visited nodes
            
            Args: p_node : Parent Node
            Return: None
            """
        for i in range(-1 , 2):
            for j in range(-1 , 2):
                #Not Considering Neg vals
                if p_node.x+i<0 or p_node.y+j<0:
                    continue
   
                if not self.prob_map[p_node.x + i , p_node.y + j]:
                    #Taking Eucledian dist to be the cost
                    new_cost = p_node.cost + np.sqrt(i**2 + j**2)
                    new_parent = p_node
                    new_node = Node(p_node.x + i , p_node.y + j , new_cost , new_parent)

                    #If the newly created node is alrerady in unvisited nodes then check its parent before adding this new_node 
                    if new_node in self.unvisited_nodes:
                        if self.unvisited_nodes[self.unvisited_nodes.index(new_node)].cost < new_node.cost:
                            continue
                        else:
                            self.unvisited_nodes[self.unvisited_nodes.index(new_node)] = new_node

                    else:
                        self.unvisited_nodes.append(new_node)

        #Update Visited and Unvisited Nodes
        self.visited_nodes.append(p_node)
        self.unvisited_nodes.remove(p_node)

    def draw_final_graph(self , final_path):
        """ For Graphing Purpose """
        plt.plot(self.x_obst , self.y_obst , ".k")
        plt.plot([p[0] for p in final_path] , [p[1] for p in final_path] , ".r")
        plt.grid(True)
        plt.show()
        
if __name__ == "__main__":
    inf = 7
    scan=[inf, inf, inf, inf, inf, inf, 2.122950792312622, 1.8218177556991577, 1.59413743019104, 1.423231601715088, 1.323502540588379, 1.3081393241882324, 1.3145588636398315, 1.3490028381347656, 1.335472822189331, 1.319705605506897, 1.3400593996047974, 1.3491653203964233, 1.3466880321502686, 1.3696467876434326, 1.3745601177215576, 1.3833107948303223, 1.3923722505569458, 1.4062138795852661, 1.3897963762283325, 1.4243078231811523, 1.4298373460769653, 1.4416676759719849, 1.450016736984253, 1.471937656402588, 1.5024218559265137, 1.4850928783416748, 1.5270593166351318, 1.5272008180618286, 1.5350757837295532, 1.5613079071044922, 1.58877432346344, 1.6090070009231567, 1.6587506532669067, 1.6668319702148438, 1.6823045015335083, 1.711564540863037, 1.7509064674377441, 1.7584381103515625, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, 2.650038242340088, 2.6445717811584473, 2.631784677505493, 2.6164636611938477, 2.6125850677490234, 2.5924696922302246, 2.5889229774475098, 2.5727877616882324, 2.5825552940368652, 2.5913491249084473, 2.5679054260253906, 2.575549840927124, 2.5961365699768066, 2.578033447265625, 2.563906669616699, 2.576000690460205, 2.586289644241333, 2.5772287845611572, 2.5811655521392822, 2.5825459957122803, 2.588222026824951, 2.5943994522094727, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, 2.9890737533569336, 2.9071550369262695, 2.8188626766204834, 2.7494678497314453, 2.67634916305542, 2.6266491413116455, 2.5653467178344727, 2.5333642959594727, 2.4661786556243896, 2.4012131690979004, 2.3687829971313477, 2.323002338409424, 2.2887566089630127, 2.2677664756774902, 2.296781539916992, 2.3500185012817383, 2.360821008682251, 2.453289031982422, 2.4872214794158936, 2.548398017883301, 2.6022679805755615, 2.6568949222564697, 2.7322375774383545, 2.7981326580047607, 2.8849222660064697, 2.9328646659851074, 3.0372416973114014, inf, inf, inf, inf, inf, 3.4685451984405518, 3.4392693042755127, 3.423564910888672, 3.389524221420288, 3.3799402713775635, 3.355445146560669, 3.335869073867798, 3.295499086380005, 3.285273551940918, 3.2675883769989014, 3.2415120601654053, 3.2439682483673096, 3.21799898147583, 3.207578659057617, 3.2076492309570312, 3.3214356899261475, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, 1.7193019390106201, 1.679825782775879, 1.650679349899292, 1.6179615259170532, 1.5983997583389282, 1.5532128810882568, 1.5364797115325928, 1.5386948585510254, 1.5013322830200195, 1.4845850467681885, 1.4534012079238892, 1.444892168045044, 1.4262295961380005, 1.4062203168869019, 1.4057549238204956, 1.3824982643127441, 1.367220163345337, 1.3536320924758911, 1.3183528184890747, 1.3393183946609497, 1.312820553779602, 1.3034974336624146, 1.2918084859848022, 1.2986373901367188, 1.2754086256027222, 1.271276831626892, 1.2715413570404053, 1.2496140003204346, 1.220840334892273, 1.259627103805542, 1.2267930507659912, 1.2130810022354126, 1.2276153564453125, 1.2212096452713013, 1.281704306602478, 1.3779208660125732, 1.5410783290863037, 1.7324031591415405, 1.9674873352050781, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, 2.966913938522339, 2.879478931427002, 2.812376022338867, 2.7323076725006104, 2.667478322982788, 2.613341808319092, 2.5557971000671387, 2.4918932914733887, 2.439162492752075, 2.3888156414031982, 2.3447256088256836, 2.304054021835327, 2.2600510120391846, 2.2221076488494873, 2.216076374053955, 2.2552261352539062, 2.31876802444458, 2.357189178466797, 2.396010398864746, 2.4466495513916016, 2.4985275268554688, 2.564828395843506, 2.634127140045166, 2.671659231185913, 2.7563955783843994, 2.829598903656006, 2.9014203548431396, 2.9869801998138428, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf]
    o =AStar()

    #First Get the grid map
    d = Mapper()
    prob_map = Mapper.main(d,scan)
    o(prob_map , [36,36], [10,50])
    final_path = o.find_path()
    o.draw_final_graph(final_path)
    

