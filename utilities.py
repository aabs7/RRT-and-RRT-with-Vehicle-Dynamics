
import random, math
from dubins_path_planning import dubins_path_planning
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Point

class LinkDubins(object):
    def __init__(self, point, path_x = [], path_y = [],path_yaw = [], length = None, upstream=None):
        self.point = point
        self.upstream = upstream
        self.path_x = path_x
        self.path_y = path_y
        self.path_yaw = path_yaw
        if upstream is not None:
            self.local_path_length = length
        else:
            self.local_path_length = 0
    
        
    @property
    def path_length(self):
        if self.upstream is not None:
            return self.upstream.path_length + self.local_path_length
        else:
            return 0
    
    def get_dubins_distance(self, point, return_all = False):
        curvature = 1.0
        path_x, path_y, path_yaw, _, lengths = dubins_path_planning(self.point[0],
                                                                self.point[1],
                                                                self.point[2],
                                                                point[0],
                                                                point[1],
                                                                point[2],
                                                                curvature)
        if not return_all:
            return sum(lengths)
        else:
            return (sum(lengths),lengths,path_x,path_y,path_yaw)
    
    def get_euclidean_distance(self, point):
        return np.linalg.norm(np.array(self.point[:2]) - np.array(point[:2]))
    
    def does_path_collide(self,obstacles):
        for x,y in zip(self.path_x,self.path_y):
            for obstacle in obstacles:
                if obstacle.contains(Point(x,y)):
                    return True
        else:
            return False

    def __hash__(self):
        return hash(id(self))
    
    def plot(self, fmt='b', alpha=1):
        if self.upstream is not None:
            plt.plot(self.path_x,self.path_y, fmt, alpha=alpha)
    
def generate_random_node(region_x_local,region_y_local,goal):
    # We increase the probability of sampling a point == goal
    # if random.randint(0,100) > 5:
    #     x = random.uniform(region_x_local[0],region_x_local[1])
    #     y = random.uniform(region_y_local[0],region_y_local[1])
    #     phi = random.uniform(-math.pi,math.pi)
    # else:
    #     x = goal[0]
    #     y = goal[1]
    #     phi = goal[2]

    x = random.uniform(region_x_local[0],region_x_local[1])
    y = random.uniform(region_y_local[0],region_y_local[1])
    phi = random.uniform(-math.pi,math.pi)
    return [x,y,phi]


def steer_towards_point_dubins(latest_link,point,step_size):
    curvature = 1.0
    x_a,y_a,phi_a = latest_link.point[0],latest_link.point[1],latest_link.point[2]
    x_b,y_b,phi_b = point[0],point[1],point[2]
    path_x, path_y, path_yaw, mode, lengths = dubins_path_planning(x_a,
                                                                y_a,
                                                                phi_a,
                                                                x_b,
                                                                y_b,
                                                                phi_b,
                                                                curvature)
    if len(path_x) <= 1: 
        return None,None # cannot find a dubins path
    
    if sum(lengths) > step_size:
        path_length = 0
        for i in range(len(path_x)):
            index = i
            if i == len(path_x) - 1:
                break
            x = np.array((path_x[i],path_y[i]))
            y = np.array((path_x[i+1],path_y[i+1]))
            path_length += np.linalg.norm(x-y)
            if path_length >= step_size:
                break

        new_path_x = path_x[:i+1]
        new_path_y = path_y[:i+1]
        new_path_yaw = path_yaw[:i+1]

        new_x = new_path_x[-1]
        new_y = new_path_y[-1]
        # change yaw to the yaw of current line
        new_phi = math.atan2(new_path_y[-1] - new_path_y[-2],new_path_x[-1]-new_path_x[-2])
        #     print(f"new_x:{new_x} new_y:{new_y}, x:{y[0]},y{y[1]}")
    
    else:
        new_x = path_x[-1]
        new_y = path_y[-1]
        new_phi = path_yaw[-1]
        new_path_x = path_x.copy()
        new_path_y = path_y.copy()
        new_path_yaw = path_yaw.copy()
        path_length = sum(lengths)
        
    
    assert len(new_path_x) == len(new_path_y)
    assert len(new_path_yaw) == len(new_path_x)

    new_point = [new_x,new_y,new_phi]
    
    new_link = LinkDubins(new_point,new_path_x,new_path_y,new_path_yaw,path_length,upstream=latest_link)
    return new_link,new_point



def find_dubins_neighborhood(links,new_point,radius):
    neighbors = []
    neighbors_dubins_distances = []
    neighbors_path_x = []
    neighbors_path_y = []
    for link in links:
        d = link.get_euclidean_distance(new_point)
        if d <= radius:
            distance,_,path_x,path_y = link.get_dubins_distance(new_point,return_all = True)
            neighbors.append(link)
            neighbors_dubins_distances.append(distance)
            neighbors_path_x.append(path_x)
            neighbors_path_y.append(path_y)
            
    return neighbors,neighbors_dubins_distances,neighbors_path_x,neighbors_path_y

def find_d_neighborhood(links,new_point,radius):
    neighbors = []
    for link in links:
        if link.get_euclidean_distance(new_point) <= radius:
            neighbors.append(link)
    return neighbors