
import matplotlib.pyplot as plt
import numpy as np
import random
from sortedcontainers import SortedList
import time
import matplotlib.pyplot as plt
from utilities import LinkDubins,generate_random_node,steer_towards_point_dubins,find_d_neighborhood




def RRTDubins(start, goal, obstacles, region_x_local, region_y_local, 
                               num_iterations, step_size, seed=695):
    random.seed(seed)
    links = [LinkDubins(start)]
    for _ in range(num_iterations):
        # Generate a random point
        point = generate_random_node(region_x_local,region_y_local,goal)
        # Get the closest link
        closest_link = min(links, key=lambda l: l.get_dubins_distance(point))
        # print(closest_link.point)
        # Steer towards it
        new_link, new_point = steer_towards_point_dubins(closest_link, point, step_size)
        # If there is no dubins distance, no new point
        if new_point == None:
            continue
        # If it collides, return
        if new_link.does_path_collide(obstacles):
            continue

        # Add to chain if it does not collide
        links.append(new_link)
        

    return links


def RRTstarDubins(start, goal, obstacles, region_x, region_y, 
                               num_iterations, step_size, seed=695):
    random.seed(seed)
    links = [LinkDubins(start)]

    for _ in range(num_iterations):
        # Generate a random point
        # Generate a random point
        point = generate_random_node(region_x,region_y,goal)
        
        # Get the closest link
        closest_link = min(links, key=lambda l: l.get_dubins_distance(point))
        # print(closest_link.point)
        # Steer towards it
        new_link, new_point = steer_towards_point_dubins(closest_link, point, step_size)
        
        # If there is no dubins distance, no new point
        if new_point == None:
            continue
        
        # If it collides, return
        if new_link.does_path_collide(obstacles):
            continue
         
        # find neighbor from the new point and if the neighbor can be reached quicker, update it
        
        neighbors = find_d_neighborhood(links,new_point,step_size)
        for neighbor in neighbors:
            distance,_,x_path,y_path,yaw_path = new_link.get_dubins_distance(neighbor.point,return_all = True)
            dubins_link_to_neighbor = LinkDubins(neighbor.point,x_path,y_path,yaw_path,length = distance,upstream = new_link)
            if not (dubins_link_to_neighbor.does_path_collide(obstacles)):
                if (neighbor.path_length > new_link.path_length + distance):
                    neighbor.upstream = new_link
                    neighbor.path_x = x_path
                    neighbor.path_y = y_path
                    neighbor.path_yaw = yaw_path
                    neighbor.local_path_length = distance
        
        links.append(new_link)
    return links