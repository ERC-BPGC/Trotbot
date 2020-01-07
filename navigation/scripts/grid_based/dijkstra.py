#! /usr/bin/env python

""" Dijkstr Path Planning Algorithm for Lidar LaserScan
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
    Nodes for Dijkstra
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
        return (self.x , self.y)

    def __eq__(self , c):
        """ While Equating this node to c  """
        return (c.x , c.y) == (self.x,self.y)

class Dijkstra(object):
    """ Dijkstra Algorithm """
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
            check_node_ind = np.argmin([p.cost for p in self.unvisited_nodes ]) #Get the least cost node and make that check_node
            check_node = self.unvisited_nodes[check_node_ind] 
            
            iter+=1
            print(iter)
        
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
            
            print(final_path)
        
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
    scan =[inf, inf, inf, inf, inf, inf, inf, 3.056861639022827, 3.0252268314361572, 3.0203325748443604, 2.0132949352264404, 1.9735280275344849, 1.953012466430664, 1.9486260414123535, 1.9373271465301514, 1.9421530961990356, 1.9569212198257446, 2.002772331237793, inf, inf, inf, 0.9855800271034241, 0.9726546406745911, 0.9509224891662598, 0.9256065487861633, 0.9257093071937561, 0.922886073589325, 0.9183664917945862, 0.906120240688324, 0.8961981534957886, 0.9056581854820251, 0.9420517683029175, 0.9265859723091125, 0.9183934330940247, 0.9498110413551331, 0.9667358994483948, 1.0045623779296875, 2.442192792892456, 2.4326982498168945, 2.4553067684173584, 2.4645442962646484, 2.506408214569092, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, 1.7533923387527466, 1.71872878074646, 1.70149827003479, 1.6858848333358765, 1.6909033060073853, 1.7164965867996216, 1.7079846858978271, 1.7376644611358643, 1.785688877105713, 3.3595521450042725, 3.3191449642181396, 3.29377818107605, 3.2705166339874268, 3.2538671493530273, 3.2152647972106934, 3.2159855365753174, 3.19524884223938, 3.0854270458221436, 2.8649425506591797, 2.6712183952331543, 2.554349184036255, 2.556751251220703, 2.5452141761779785, 2.5436341762542725, 2.5213427543640137, 2.5272300243377686, 2.5183942317962646, 2.509161949157715, 2.5124869346618652, 2.4297120571136475, 2.3386430740356445, 2.2512683868408203, 2.1818249225616455, 2.103046417236328, 2.0564870834350586, 1.9878475666046143, 1.9268088340759277, 1.885642170906067, 1.8311305046081543, 1.7829922437667847, 1.7410887479782104, 1.7179292440414429, 1.65846848487854, 1.6332777738571167, 1.5900510549545288, 1.5677014589309692, 1.5201916694641113, 1.4969840049743652, 1.4841266870498657, 1.4385578632354736, 1.4428545236587524, 1.4075186252593994, 1.3866387605667114, 1.3508318662643433, 1.3370169401168823, 1.3260891437530518, 1.3067781925201416, 1.2939659357070923, 1.2805120944976807, 1.2707873582839966, 1.2281290292739868, 1.2153609991073608, 1.2014520168304443, 1.2100038528442383, 1.1812770366668701, 1.1618354320526123, 1.1519147157669067, 1.1456130743026733, 1.1571439504623413, 1.1326026916503906, 1.1386666297912598, 1.0997953414916992, 1.091562271118164, 1.1005403995513916, 1.1073905229568481, 1.075814962387085, 1.0699113607406616, 1.078955054283142, 1.0810444355010986, 1.0660544633865356, 1.0604965686798096, 1.0380046367645264, 1.0327768325805664, 1.078073501586914, 1.0491496324539185, 1.0314854383468628, 1.0256805419921875, 1.0399309396743774, 1.039351224899292, 1.0458898544311523, 1.0328317880630493, 1.0365073680877686, 1.0345937013626099, 1.0336904525756836, 1.045121431350708, 1.019679069519043, 0.9782207012176514, 0.9893175959587097, 0.9512673616409302, 0.9016335606575012, 0.8861187696456909, 0.8714970350265503, 0.8524404168128967, 0.8410521745681763, 0.8306225538253784, 0.806861162185669, 0.7750387787818909, 0.772260308265686, 0.7613915205001831, 0.7231266498565674, 0.7228431701660156, 0.7109572291374207, 0.7123162746429443, 0.6959463357925415, 0.6842755079269409, 0.6778551936149597, 0.672926127910614, 0.6455686688423157, 0.6492228507995605, 0.650477945804596, 0.6205228567123413, 0.63560950756073, 0.6331012845039368, 0.6159425973892212, 0.6135613918304443, 0.5959789752960205, 0.610405445098877, 0.5870363712310791, 0.5804761052131653, 0.6007770299911499, 0.5676383972167969, 0.5660935640335083, 0.5781680345535278, 0.5915207862854004, 0.5583205819129944, 0.5651285648345947, 0.5652458071708679, 0.5596476197242737, 0.545867919921875, 0.5507373809814453, 0.5318761467933655, 0.542111337184906, 0.540361225605011, 0.5328619480133057, 0.5332945585250854, 0.5352373123168945, 0.5453101992607117, 0.5408035516738892, 0.5488573908805847, 0.5210607647895813, 0.5293474793434143, 0.5348880290985107, 0.5456039905548096, 0.5199136734008789, 0.5178253650665283, 0.5266040563583374, 0.5523934364318848, 0.5342961549758911, 0.5384075045585632, 0.5334569215774536, 0.5281636714935303, 0.5111650228500366, 0.5428661704063416, 0.5401403903961182, 0.5570170879364014, 0.5265321731567383, 0.5520188212394714, 0.5388387441635132, 0.5361006855964661, 0.5432685613632202, 0.5588036775588989, 0.5637043118476868, 0.5593273043632507, 0.5618724226951599, 0.5681909918785095, 0.580647349357605, 0.5878729224205017, 0.567261815071106, 0.5693681240081787, 0.5898579955101013, 0.6078497171401978, 0.6051531434059143, 0.6091262102127075, 0.5822492837905884, 0.6195617318153381, 0.6141515374183655, 0.617192804813385, 0.6386907696723938, 0.6485453248023987, 0.6483967900276184, 0.6659299731254578, 0.6548660397529602, 0.6864899396896362, 0.6838104128837585, 0.6932967901229858, 0.7090590596199036, 0.7026281356811523, 0.7346917986869812, 0.7633113861083984, 0.7599661350250244, 0.7832435369491577, 0.779472291469574, 0.8087064623832703, 0.8164687752723694, 0.8538014888763428, 0.8430877923965454, 0.8757237195968628, 0.8956906199455261, 0.9206846356391907, 0.9354161024093628, 0.9733413457870483, 0.9885174036026001, 1.0486935377120972, 1.049687385559082, 1.1021144390106201, 1.1006282567977905, 1.1586135625839233, 1.2052730321884155, 1.2459439039230347, 1.302675485610962, 1.3653827905654907, 1.4164248704910278, 1.4472814798355103, 1.535380244255066, 1.532191514968872, 1.5412589311599731, 1.5219082832336426, 1.5472310781478882, 1.5644363164901733, 1.55789053440094, 1.5711780786514282, 1.5786079168319702, 1.5898356437683105, 1.600523829460144, 1.6128002405166626, 1.6159331798553467, 1.626638412475586, 1.8782384395599365, 2.235314130783081, 2.2563958168029785, 2.2954134941101074, 2.307398557662964, 2.3106799125671387, 2.351066827774048, 2.391575813293457, 2.3959853649139404, 2.430370569229126, 2.449401378631592, 2.478189468383789, 2.516754150390625, 2.5472171306610107, 2.59478759765625, 2.6314077377319336, 2.6275601387023926, 2.682375907897949, 2.7329845428466797, 2.7940165996551514, 2.8262627124786377, 2.891613245010376, 2.943937301635742, 2.9711005687713623, 3.0209319591522217, 3.1053292751312256, 1.0324296951293945, 1.0091277360916138, 1.0003530979156494, 0.9934810996055603, 0.9784740209579468, 0.9725538492202759, 0.9597940444946289, 0.9535285830497742, 0.9423716068267822, 0.9491086602210999, 0.966484010219574, 0.9733442068099976, 0.9970974326133728, 1.0060924291610718, 1.0158592462539673, 1.0913983583450317, inf, inf, inf, inf, 2.0314512252807617, 2.0178802013397217, 1.9728460311889648, 1.961500883102417, 1.9714338779449463, 1.9641704559326172, 2.0064427852630615, 2.0385403633117676, 3.066349506378174, 3.039686918258667, 3.0638043880462646, 3.07513165473938, inf, inf, inf, inf, inf, inf, inf, inf, inf]
    o =Dijkstra()

    #First Get the grid map
    d = Mapper()
    prob_map = Mapper.main(d,scan)
    o(prob_map , [36,36], [50,30])
    final_path = o.find_path()
    o.draw_final_graph(final_path)
    

