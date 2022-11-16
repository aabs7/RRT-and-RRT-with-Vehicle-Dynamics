import matplotlib.pyplot as plt
import numpy as np
import math
from utilities import LinkDubins
import os


# Vehicle parameters
LENGTH = 1  # [m]
WIDTH = 0.4  # [m]
BACKTOWHEEL = 0.2  # [m]
WHEEL_LEN = 0.1  # [m]
WHEEL_WIDTH = 0.05  # [m]
TREAD = 0.2  # [m]
WB = 0.6  # [m]

SAVEDIR = './images/'

def plot_car(x, y, yaw, steer=0.0, cabcolor="-r", truckcolor="-k"):  # pragma: no cover

    outline = np.array([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
                        [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

    fr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                         [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD]])

    rr_wheel = np.copy(fr_wheel)

    fl_wheel = np.copy(fr_wheel)
    fl_wheel[1, :] *= -1
    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1

    Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                     [-math.sin(yaw), math.cos(yaw)]])
    Rot2 = np.array([[math.cos(steer), math.sin(steer)],
                     [-math.sin(steer), math.cos(steer)]])

    fr_wheel = (fr_wheel.T.dot(Rot2)).T
    fl_wheel = (fl_wheel.T.dot(Rot2)).T
    fr_wheel[0, :] += WB
    fl_wheel[0, :] += WB

    fr_wheel = (fr_wheel.T.dot(Rot1)).T
    fl_wheel = (fl_wheel.T.dot(Rot1)).T

    outline = (outline.T.dot(Rot1)).T
    rr_wheel = (rr_wheel.T.dot(Rot1)).T
    rl_wheel = (rl_wheel.T.dot(Rot1)).T

    outline[0, :] += x
    outline[1, :] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    plt.plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fr_wheel[0, :]).flatten(),
             np.array(fr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rr_wheel[0, :]).flatten(),
             np.array(rr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fl_wheel[0, :]).flatten(),
             np.array(fl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rl_wheel[0, :]).flatten(),
             np.array(rl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(x, y, "*")

def animate(links,env,start,goal,path_x,path_y,path_yaw):
    image_file = os.path.join(SAVEDIR+'car')
    plt.figure(figsize=(8, 8), dpi=150)
    for i in range(len(path_x)):
        plt.cla()
        plot_env(start, goal, env)
        for link in links:
            link.plot(fmt='y', alpha=0.5)   
        plt.plot(path_x,path_y,"k:")
        plot_car(path_x[i],path_y[i],path_yaw[i])
        img = image_file + str(i) + '.png'
        plt.savefig(img)
        plt.pause(0.05)

def plot_env(start, goal, obstacles):
    for poly in obstacles:
        plt.plot(*poly.exterior.xy, 'k')
    
    plt.plot(start[0], start[1], 'b.')
    plt.plot(goal[0], goal[1], 'g*')

    plt.gca().axis('equal')


def plot_environment_and_links_dubins(links = None,env = None,start = None, goal = None,do_plot_goal_path = True):
    plt.figure(figsize=(8, 8), dpi=150)
    plot_env(start, goal, env)
    for link in links:
        link.plot(fmt='y', alpha=0.5)
        plt.scatter(link.point[0],link.point[1],color = 'k',s = 0.5)

    min_distance = 1000
    min_link = None
    for link in links:
        distance,lengths,path_x,path_y,path_yaw = link.get_dubins_distance(goal,return_all = True)
        if distance <= min_distance:
            min_distance = distance
            min_lengths = lengths
            min_path_x = path_x
            min_path_y = path_y
            min_path_yaw = path_yaw
            goal_path = LinkDubins(goal,min_path_x,min_path_y,min_path_yaw,length = distance, upstream=link)
            if goal_path.does_path_collide(env):
                continue
            min_link = link
    closest_link_to_goal = min_link
    if closest_link_to_goal is None:
        print("Can't reach goal, change configurations of start and end!!!")
        return 0
    goal_path_x = []
    goal_path_y = []
    goal_path_yaw = []
    plt.title(f"Cost of path: {goal_path.path_length}")
    if do_plot_goal_path:
        while goal_path is not None:
            # goal_path.plot(fmt='b:')
            pathx = list(goal_path.path_x)
            pathy = list(goal_path.path_y)
            pathyaw = list(goal_path.path_yaw)
            pathx.extend(goal_path_x)
            pathy.extend(goal_path_y)
            pathyaw.extend(goal_path_yaw)
            goal_path_x = pathx.copy()
            goal_path_y = pathy.copy()
            goal_path_yaw = pathyaw.copy()

            assert len(goal_path_x) == len(goal_path_y)
            assert len(goal_path_yaw) == len(goal_path_x)

            goal_path = goal_path.upstream
    
    plt.plot(goal_path_x,goal_path_y,"k:")
    animate(links,env,start,goal,goal_path_x,goal_path_y,goal_path_yaw)
    plt.show()