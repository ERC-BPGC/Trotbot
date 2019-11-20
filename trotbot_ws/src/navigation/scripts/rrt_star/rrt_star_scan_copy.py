#! /usr/bin/env python
"""
Path planning Code of RRT* with
author: Ojit Mehta(@ojitmehta123)
"""
import numpy as np
import sys
import os
# from context import RRT, utils
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../rrt_for_scan')))

from utils_scan import scan_obstacle_checker, make_obstacles_scan, check_intersection_scan
from utils_scan import adjustable_random_sampler as sampler
from descartes import PolygonPatch
from shapely.geometry import Polygon, Point, LineString
import random
import math
import os
import sys

import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../rrt_for_scan/")


try:
    from rrt import RRT
except ImportError:
    raise

show_animation = True


def propagate_cost_to_leaves(node_list, parent_node):

        for node in node_list:
            if node.parent == parent_node:
                node.cost = calculate_cost(parent_node, (node.x, node.y))
                propagate_cost_to_leaves(node_list, node)


def calculate_cost(from_node, to_node):
    """ To calculate Cost from from_node --> to_node
        Arguments: 	from_node --> Node from node_list
                    to_node --> tuple
        Return:
                    Value of Cost from start to to_node
    """
    return from_node.cost + math.sqrt((from_node.x - to_node[0])**2 + (from_node.y - to_node[1])**2)


def find_near_nodes(node_list, new_node, circle_dist):
    """ To Find the nearest nodes at max circle_dist from new_node in node_list from new_node  """
    nnode = len(node_list) + 1
    r = circle_dist * math.sqrt((math.log(nnode) / nnode))
    dist_list = [(node.x - new_node[0]) ** 2 +
                    (node.y - new_node[1]) ** 2 for node in node_list]
    near_inds = [dist_list.index(i) for i in dist_list if i <= r ** 2]
    return near_inds


class Node(object):
    """
    Coordinate representation in node form.
    x,y --> Coordinates
    Parent node is the node connected to the present node
    """

    def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None
            self.cost = 0.0

    def __str__(self):
        return ("("+str(self.x)+','+str(self.y)+")")




class RRTStar(object):
    """
    RRT star algorithm
    """

    def __init__(self, sample_area,
                    expand_dis=1.0,
                    path_resolution=1.0,
                    goal_sample_rate=0.1,
                    max_iter=200,
                    connect_circle_dist=10.0
                    ):
        """
        start: Start Point. in our case remains(0 , 0) unless specified
        goal: Next goal to be reached
        scan = LaserScan polar distances to Obstacles [r1,r2,r3...] initially assuming every scan occurs at 1 rad interval
        randomArea:Random Sampling Area
        """

        self.sample_area = sample_area
        self.sampler = sampler
        self.expand_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate
        self.circle = connect_circle_dist
        self.max_iter = max_iter

    def __call__(self, goal_point, scan, start_point=[0, 0], animation=True):
        """Plans path from start to goal avoiding obstacles.

        Args:
            start_point: tuple with start point coordinates.
            end_point: tuple with end point coordinates.
            scan: list of obstacles which themselves are list of points
            animation: flag for showing planning visualization (default False)

        Returns:
            A list of points representing the path determined from
            start to goal while avoiding obstacles.
            An list containing just the start point means path could not be planned.
        """
        search_until_max_iter = True

        # Make line obstacles and scan in x,y from scan
        line_obstacles, pts = make_obstacles_scan(scan)

        # Setting Start and End
        self.start = Node(start_point[0], start_point[1])
        self.goal = Node(goal_point[0], goal_point[1])

        # Initialize node with Starting Position
        self.node_list = [self.start]

        # Loop for maximum iterations to get the best possible path
        for iter in range(self.max_iter):
            #########################################
            # print("NODE_LIST-->")
            # for printer_i in self.node_list:
            #     print(printer_i)
            #########################################
            # Sample a Random point in the sample area
            rnd_point = sampler(self.sample_area, (self.goal.x , self.goal.y), self.goal_sample_rate)

            ########################################
            # print("RANDOM POINT-->")
            # print(rnd_point)
            ########################################

            # Find nearest node to the sampled point
            distance_list = [(node.x - rnd_point[0])**2 + (node.y -
                              rnd_point[1])**2 for node in self.node_list]
            nearest_node = self.node_list[distance_list.index(min(distance_list))]
            ########################################
            # print("NEAREST_NODE-->")
            # print(nearest_node.x , nearest_node.y , nearest_node.cost)
            ########################################
            # Creating a new Point in the Direction of sampled point
            theta = math.atan2(rnd_point[1] - nearest_node.y,
                               rnd_point[0] - nearest_node.x)
            new_point = nearest_node.x + self.expand_dis*math.cos(theta), \
                        nearest_node.y + self.expand_dis*math.sin(theta)
            
            #########################################
            # print("NEW_POINT-->")
            # print(new_point[0],new_point[1])
            #########################################
            # Check obstacle collision
            new_point = scan_obstacle_checker(scan, new_point)

            if math.isnan(new_point[0]):
                ########################################
                # print("ISNAN-->")
                # print(new_point)
                ########################################
                continue

            nearest_indexes = find_near_nodes(self.node_list, new_point, self.circle)

            # Getting the parent node from nearest indices

            costs = []  # List of Total costs from the start to new_node when attached to parent node in node_list
            temp_points = []

            for index in nearest_indexes:
                near_node = self.node_list[index]
                temp_theta = math.atan2(
                    new_point[1] - near_node.y, new_point[0] - near_node.x)
                # Temp node in the direction of new_point from near_nodes
                temp_point = near_node.x + math.cos(temp_theta), near_node.y + math.sin(temp_theta)
                
                #########################################
                # print("TEMP POINT-->")
                # print(temp_point)
                #########################################
                try:
                    if temp_point and scan_obstacle_checker(scan, temp_point):
                        temp_points.append(temp_point)
                        costs.append(calculate_cost(near_node, temp_point))
            
                    else:
                        costs.append(float("inf"))
                except:
                    costs.append(float("inf"))

            # Get the minimum costs from costs
            #########################################
            # print("COSTS-->")
            # print(costs)
            #########################################
            try:    
                min_cost = min(costs)
            except:
                continue
            # Calculating the minimum cost and selecting the node for which it occurs as parent child

            if min_cost == float("inf"):
                print("min_cost is inf")
                continue

            # Setting the new node as the one with min cost
            min_ind = nearest_indexes[costs.index(min_cost)]
            new_node = Node(temp_points[costs.index(min_cost)][0], temp_points[costs.index(min_cost)][1])
            new_node.parent = self.node_list[min_ind]
            new_node.cost = min_cost

            #########################################
            # print("NEW_NODE-->")
            # print(new_node.x , new_node.y , new_node.cost)
            #########################################

            if new_node:
                    
                self.node_list.append(new_node)
                
                for ind in nearest_indexes:
                    node_check = self.node_list[ind]
                    theta = math.atan2(node_check.y - new_node.y, node_check.x - new_node.x)

                    edge_node = Node(new_node.x + math.cos(theta),
                                     new_node.y + math.sin(theta))
                    edge_node.cost = calculate_cost(new_node, (node_check.x, node_check.y))

                    if not edge_node:
                        #########################################
                        print("NOT EDGE?-->")
                        print(edge_node.x , edge_node.y , edge_node.cost)
                        #########################################
                        continue
                    
                    point_list = [(a,b) for a,b in zip(node_check.path_x,node_check.path_y)]
                    point_list.append((edge_node.x,edge_node.y))
                    try:
                        no_coll = scan_obstacle_checker(scan, (edge_node.x, edge_node.y)) and not check_intersection_scan(point_list, line_obstacles)
                    except:
                        no_coll =False
                        print("Colliding")
                    cost_improv = edge_node.cost < node_check.cost

                    if no_coll and cost_improv:
                        node_check = edge_node
                        node_check.parent = new_node
                        propagate_cost_to_leaves(self.node_list, new_node)
                
                present_node = new_node
                px =[]
                py=[]
                while present_node.parent != None:
                    px.append(present_node.x)
                    py.append(present_node.y)
                    present_node = present_node.parent
                px.append(self.start.x)
                py.append(self.start.y)
                new_node.path_x = px[:]
                new_node.path_y = py[:]
            if animation and iter % 5 == 0:
                self.draw_graph(scan, new_node)

            if (not search_until_max_iter) and new_node:  # check reaching the goal
                print("ENDING")
                last_index = self.search_best_goal_node()
                if last_index:
                    path = [[self.goal.x, self.goal.y]]
                    node = self.node_list[last_index]
                    while node.parent is not None:
                        path.append([node.x, node.y])
                        node = node.parent
                    path.append([node.x, node.y])
                    return path
            # print(iter)

        print(iter)
        print("reached max iteration")

        last_index = self.search_best_goal_node(scan)
        if last_index:
            path = [[self.goal.x, self.goal.y]]
            node = self.node_list[last_index]
            while node.parent is not None:
                path.append([node.x, node.y])
                node = node.parent
            path.append([node.x, node.y])
            return path
        return None

    def draw_graph(self, scan, rnd=None):
        plt.clf()
        pt_ang = np.arange(0,2*np.pi,np.pi/180)
        pt_scan = np.array(scan)
        pts = []
        pt_x = np.multiply(pt_scan,np.cos(pt_ang))
        pt_y = np.multiply(pt_scan,np.sin(pt_ang))

        for a,b in zip(pt_x,pt_y):
		    pts.append((a,b))

        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        # for (ox, oy, size) in self.obstacle_list:
        #     self.plot_circle(ox, oy, size)
        plt.plot([x for (x, _) in pts], [y for (_, y) in pts],'r.')
        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.goal.x, self.goal.y, "xr")
        plt.axis("equal")
        plt.axis((-5,5,-5,5))
        plt.grid(True)
        plt.pause(0.01)

    def search_best_goal_node(self,scan):
        dist_to_goal_list = [math.sqrt(
            (n.x - self.goal.x)**2 + (n.y - self.goal.y)**2) for n in self.node_list]
        goal_inds = [dist_to_goal_list.index(
            i) for i in dist_to_goal_list if i <= self.expand_dis]

        safe_goal_inds = []
        for goal_ind in goal_inds:
            theta = math.atan2(
                self.node_list[goal_ind].y - self.goal.y, self.node_list[goal_ind].x - self.goal.x)

            t_node = Node(self.node_list[goal_ind].x + math.cos(theta) , self.node_list[goal_ind].y + math.sin(theta))
            if scan_obstacle_checker(scan , (t_node.x , t_node.y)):
                safe_goal_inds.append(goal_ind)

        if not safe_goal_inds:
            return None

        min_cost = min([self.node_list[i].cost for i in safe_goal_inds])
        for i in safe_goal_inds:
            if self.node_list[i].cost == min_cost:
                return i

        return None


def main():
    print("Start " + __file__)
    inf  = 100
    # ====Search Path with RRT====
    scan_list =[inf, inf, inf, inf, inf, inf, inf, 3.056861639022827, 3.0252268314361572, 3.0203325748443604, 2.0132949352264404, 1.9735280275344849, 1.953012466430664, 1.9486260414123535, 1.9373271465301514, 1.9421530961990356, 1.9569212198257446, 2.002772331237793, inf, inf, inf, 0.9855800271034241, 0.9726546406745911, 0.9509224891662598, 0.9256065487861633, 0.9257093071937561, 0.922886073589325, 0.9183664917945862, 0.906120240688324, 0.8961981534957886, 0.9056581854820251, 0.9420517683029175, 0.9265859723091125, 0.9183934330940247, 0.9498110413551331, 0.9667358994483948, 1.0045623779296875, 2.442192792892456, 2.4326982498168945, 2.4553067684173584, 2.4645442962646484, 2.506408214569092, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, 1.7533923387527466, 1.71872878074646, 1.70149827003479, 1.6858848333358765, 1.6909033060073853, 1.7164965867996216, 1.7079846858978271, 1.7376644611358643, 1.785688877105713, 3.3595521450042725, 3.3191449642181396, 3.29377818107605, 3.2705166339874268, 3.2538671493530273, 3.2152647972106934, 3.2159855365753174, 3.19524884223938, 3.0854270458221436, 2.8649425506591797, 2.6712183952331543, 2.554349184036255, 2.556751251220703, 2.5452141761779785, 2.5436341762542725, 2.5213427543640137, 2.5272300243377686, 2.5183942317962646, 2.509161949157715, 2.5124869346618652, 2.4297120571136475, 2.3386430740356445, 2.2512683868408203, 2.1818249225616455, 2.103046417236328, 2.0564870834350586, 1.9878475666046143, 1.9268088340759277, 1.885642170906067, 1.8311305046081543, 1.7829922437667847, 1.7410887479782104, 1.7179292440414429, 1.65846848487854, 1.6332777738571167, 1.5900510549545288, 1.5677014589309692, 1.5201916694641113, 1.4969840049743652, 1.4841266870498657, 1.4385578632354736, 1.4428545236587524, 1.4075186252593994, 1.3866387605667114, 1.3508318662643433, 1.3370169401168823, 1.3260891437530518, 1.3067781925201416, 1.2939659357070923, 1.2805120944976807, 1.2707873582839966, 1.2281290292739868, 1.2153609991073608, 1.2014520168304443, 1.2100038528442383, 1.1812770366668701, 1.1618354320526123, 1.1519147157669067, 1.1456130743026733, 1.1571439504623413, 1.1326026916503906, 1.1386666297912598, 1.0997953414916992, 1.091562271118164, 1.1005403995513916, 1.1073905229568481, 1.075814962387085, 1.0699113607406616, 1.078955054283142, 1.0810444355010986, 1.0660544633865356, 1.0604965686798096, 1.0380046367645264, 1.0327768325805664, 1.078073501586914, 1.0491496324539185, 1.0314854383468628, 1.0256805419921875, 1.0399309396743774, 1.039351224899292, 1.0458898544311523, 1.0328317880630493, 1.0365073680877686, 1.0345937013626099, 1.0336904525756836, 1.045121431350708, 1.019679069519043, 0.9782207012176514, 0.9893175959587097, 0.9512673616409302, 0.9016335606575012, 0.8861187696456909, 0.8714970350265503, 0.8524404168128967, 0.8410521745681763, 0.8306225538253784, 0.806861162185669, 0.7750387787818909, 0.772260308265686, 0.7613915205001831, 0.7231266498565674, 0.7228431701660156, 0.7109572291374207, 0.7123162746429443, 0.6959463357925415, 0.6842755079269409, 0.6778551936149597, 0.672926127910614, 0.6455686688423157, 0.6492228507995605, 0.650477945804596, 0.6205228567123413, 0.63560950756073, 0.6331012845039368, 0.6159425973892212, 0.6135613918304443, 0.5959789752960205, 0.610405445098877, 0.5870363712310791, 0.5804761052131653, 0.6007770299911499, 0.5676383972167969, 0.5660935640335083, 0.5781680345535278, 0.5915207862854004, 0.5583205819129944, 0.5651285648345947, 0.5652458071708679, 0.5596476197242737, 0.545867919921875, 0.5507373809814453, 0.5318761467933655, 0.542111337184906, 0.540361225605011, 0.5328619480133057, 0.5332945585250854, 0.5352373123168945, 0.5453101992607117, 0.5408035516738892, 0.5488573908805847, 0.5210607647895813, 0.5293474793434143, 0.5348880290985107, 0.5456039905548096, 0.5199136734008789, 0.5178253650665283, 0.5266040563583374, 0.5523934364318848, 0.5342961549758911, 0.5384075045585632, 0.5334569215774536, 0.5281636714935303, 0.5111650228500366, 0.5428661704063416, 0.5401403903961182, 0.5570170879364014, 0.5265321731567383, 0.5520188212394714, 0.5388387441635132, 0.5361006855964661, 0.5432685613632202, 0.5588036775588989, 0.5637043118476868, 0.5593273043632507, 0.5618724226951599, 0.5681909918785095, 0.580647349357605, 0.5878729224205017, 0.567261815071106, 0.5693681240081787, 0.5898579955101013, 0.6078497171401978, 0.6051531434059143, 0.6091262102127075, 0.5822492837905884, 0.6195617318153381, 0.6141515374183655, 0.617192804813385, 0.6386907696723938, 0.6485453248023987, 0.6483967900276184, 0.6659299731254578, 0.6548660397529602, 0.6864899396896362, 0.6838104128837585, 0.6932967901229858, 0.7090590596199036, 0.7026281356811523, 0.7346917986869812, 0.7633113861083984, 0.7599661350250244, 0.7832435369491577, 0.779472291469574, 0.8087064623832703, 0.8164687752723694, 0.8538014888763428, 0.8430877923965454, 0.8757237195968628, 0.8956906199455261, 0.9206846356391907, 0.9354161024093628, 0.9733413457870483, 0.9885174036026001, 1.0486935377120972, 1.049687385559082, 1.1021144390106201, 1.1006282567977905, 1.1586135625839233, 1.2052730321884155, 1.2459439039230347, 1.302675485610962, 1.3653827905654907, 1.4164248704910278, 1.4472814798355103, 1.535380244255066, 1.532191514968872, 1.5412589311599731, 1.5219082832336426, 1.5472310781478882, 1.5644363164901733, 1.55789053440094, 1.5711780786514282, 1.5786079168319702, 1.5898356437683105, 1.600523829460144, 1.6128002405166626, 1.6159331798553467, 1.626638412475586, 1.8782384395599365, 2.235314130783081, 2.2563958168029785, 2.2954134941101074, 2.307398557662964, 2.3106799125671387, 2.351066827774048, 2.391575813293457, 2.3959853649139404, 2.430370569229126, 2.449401378631592, 2.478189468383789, 2.516754150390625, 2.5472171306610107, 2.59478759765625, 2.6314077377319336, 2.6275601387023926, 2.682375907897949, 2.7329845428466797, 2.7940165996551514, 2.8262627124786377, 2.891613245010376, 2.943937301635742, 2.9711005687713623, 3.0209319591522217, 3.1053292751312256, 1.0324296951293945, 1.0091277360916138, 1.0003530979156494, 0.9934810996055603, 0.9784740209579468, 0.9725538492202759, 0.9597940444946289, 0.9535285830497742, 0.9423716068267822, 0.9491086602210999, 0.966484010219574, 0.9733442068099976, 0.9970974326133728, 1.0060924291610718, 1.0158592462539673, 1.0913983583450317, inf, inf, inf, inf, 2.0314512252807617, 2.0178802013397217, 1.9728460311889648, 1.961500883102417, 1.9714338779449463, 1.9641704559326172, 2.0064427852630615, 2.0385403633117676, 3.066349506378174, 3.039686918258667, 3.0638043880462646, 3.07513165473938, inf, inf, inf, inf, inf, inf, inf, inf, inf]
    # Set Initial parameters
    rrt_star = RRTStar(sample_area=[-2, 15])

    path = rrt_star(goal_point = [0,4], scan = scan_list)

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")

        # Draw final path
        if show_animation:
            rrt_star.draw_graph(scan_list)
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
            plt.grid(True)
            plt.pause(0.01)  # Need for Mac
            plt.show()


if __name__ == '__main__':
    main()
            









