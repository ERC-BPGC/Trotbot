#! /usr/bin/env python

import time

from context import RRT, utils
from RRT import RRT

import utils
from utils import adjustable_random_sampler as sampler
from utils import los_optimizer as path_optimizer

import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    # List of obtacles as a list of lists of points
    general_obstacle_list = [
        [ (8, 5), (7, 8), (2, 9), (3, 5) ],
        [ (3, 3), (3, 5), (5, 5), (5, 3) ], 
    ]

    obstacle_list = general_obstacle_list

    # Instatiate rrt planner object
    my_tree = RRT(sample_area=(-5, 15), sampler=sampler, expand_dis=0.1)


    # Plan path while timing
    print('\n ' + '-'*30 +  "\n> Starting operation ...\n " + '-'*30 + '\n')
    start_time = time.time()

    path, node_list = my_tree((1, 1), (10, 10), obstacle_list)
    print("Path planned.")

    print('\n ' + '-'*30 + "\n> Time taken: {:.4} seconds.\n ".format(time.time() - start_time) + '-'*30 + '\n')

    # Visualize tree
    RRT.visualize_tree(node_list, obstacle_list)


    # Testing los path optimizer
    print('\n ' + '-'*30 +  "\n> Starting operation ...\n " + '-'*30 + '\n')
    start_time = time.time()

    optimized_path = path_optimizer(path, obstacle_list)
    print("Path optimized.")

    print('\n ' + '-'*30 + "\n> Time taken: {:.4} seconds.\n ".format(time.time() - start_time) + '-'*30 + '\n')

    # Visualize path
    utils.visualize_path(optimized_path, obstacle_list)
