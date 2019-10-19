#! /usr/bin/env python2.7

import random
import math
from shapely.geometry import Polygon, Point, LineString
from descartes import PolygonPatch
import matplotlib.pyplot as plt


def check_intersection(points_list, obstacle_list):
    """Check whether line passes through any obstacle.
    
    Args:
        points_list: list of points in the line.
        obstacle_list: list of obstacles as polygons.

    Returns:
        boolean specifying whether or not the line intersects
        and of the obstacles. 
    """

    direct_line = LineString(point_list)
    for obstacle in obstacle_list:
        if direct_line.intersects(Polygon(obstacle)):
            return True
    return False



def adjustable_random_sampler(sample_area, goal, goal_sample_rate):
    """Randomly sample point in area while sampling goal point 
        at a specified rate.

        Args:
            sample_area: area to sample point in.
            goal: tuple containing goal point coordinates.
            goal_sample_rate: number between 0 and 1 specifying how often 
                                to sample the goal point.
    
        Return:
            Randomly selected point as a tuple.

    """

    if random.random() > goal_sample_rate:
        return (random.uniform(sample_area[0], sample_area[1]), 
                random.uniform(sample_area[0], sample_area1))
    else:
        return goal



def los_optimizer(path, obstacle_list):
    """Line of Sight Path Optimizer.

        For each point in the path, it checks if there is a direct
        connection to procceeding points which does not pass through
        any obstacles. By joining such points, number of uneccessary
        points in the path are reduced.

        Args:
            path: list of tuples containing coordinates for a point in path..
            obstacle_list: list of obstacles.

        Returns:
            Optimized path as a list of tuples containing coordinates.
            If path is found to be intersecting with any obstacle and
            there is no lookahead optimization which avoids this, then
            only the path uptill the intersection is returned.
    """

    # Init optimized path with the start as first point in path.
    optimized_path = [path[0]]

    # Loop through all points in path, checking for LOS shortening
    current_index = 0
    while current_index < len(path) - 1:

        # Keep track of whether index has been updated or not
        index_updated = False

        # Loop from last point in path to the current one, checking if 
        # any direct connection exists.
        for lookahead_index in range(len(path) - 1 , current_index, -1):
            if not check_intersection([path[current_index], path[lookahead_index]], obstacle_list):
                # If direct connection exists then add this lookahead point to optimized
                # path directly and skip to it for next iteration of while loop
                optimized_path.append(path[lookahead_index])
                current_index = lookahead_index
                index_updated = True
                break

        # If index hasnt been updated means that there was no LOS shortening
        # and the edge between current and next point passes through an obstacle.  
        if not index_updated:    
        # In this case we return the path so far  
            return optimized_path
    
    return optimized_path