#! /usr/bin/env

from context import RRT, utils
from RRT import RRT as rrt

from utils import adjustable_random_sampler as sampler
from utils import los_optimizer as path_optimizer

import matplotlib.pyplot as plt

obstacle_list = [[[7,5.10000003],[6,4.9000007],[2.1,9],[2.9,5]],[[3,3],[3,5],[5,5],[5,3]]] 
# obstacle_list = [[[-1,5],[1,4],[2,7],[0,8]]]
my_tree = rrt(sample_area=(-5,5),sampler=sampler,expand_dis=0.1) # object init
path = my_tree((0,0),(10,10),obstacle_list,animation=True) # object called

# for nodes in path:
    # print(nodes)

# new_path = path_optimizer(path,obstacle_list)
# print(new_path)
# temp_vis = my_tree.visualize_tree(path,obstacle_list)

plt.show()