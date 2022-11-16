import matplotlib.pyplot as plt
import numpy as np
from sortedcontainers import SortedList
from plotting import plot_environment_and_links_dubins
from environments import get_assignment2_environment, get_forest_environment,get_maze_environment
from rrt import RRTDubins, RRTstarDubins



if __name__ == '__main__':
    start_yaw = np.deg2rad(45.0)
    end_yaw = np.deg2rad(90.0)

    start_multi = [-4, -4, start_yaw]
    goal_multi = [6, 6, end_yaw]
    step_size=2

    env_multi,inflated_env_multi,region_x2,region_y2 = get_assignment2_environment()
    env_maze,inflated_env_maze,region_x_new,region_y_new = get_maze_environment()
    env_forest,inflated_env_forest,region_x_forest,region_y_forest = get_forest_environment()








    links_multi_rrt = RRTDubins(start_multi, goal_multi, inflated_env_multi, region_x2, region_y2,500, step_size)
    plot_environment_and_links_dubins(links_multi_rrt,env = env_multi,start = start_multi,goal = goal_multi)
    

    # links_multi_rrtstar = RRTstarDubins(start_multi, goal_multi, inflated_env_multi, region_x2, region_y2, 500, step_size)
    # plot_environment_and_links_dubins(links_multi_rrtstar, env = env_multi,start = start_multi,goal = goal_multi)



########## Forest

    start_forest = [-4, -4, np.deg2rad(45.0)]
    goal_forest = [6, 6, np.deg2rad(0.0)]

    #22.54
    # links_forest_rrt = RRTDubins(start_forest, goal_forest, inflated_env_forest, region_x_forest, region_y_forest,1500, step_size)
    # plot_environment_and_links_dubins(links_forest_rrt,env = env_forest,start = start_forest,goal = goal_forest)
    
    # links_forest_rrtstar = RRTstarDubins(start_forest, goal_forest, inflated_env_forest, region_x_forest, region_y_forest,1500, step_size)
    # plot_environment_and_links_dubins(links_forest_rrtstar,env = env_forest,start = start_forest,goal = goal_forest)

################# MAZE #############################
    start_maze = [-4, -4, start_yaw]
    goal_maze = [ 5, 4, np.deg2rad(90.0)]

    # links_maze_rrt = RRTDubins(start_maze, goal_maze, inflated_env_maze, region_x_new,region_y_new, 500, step_size)
    # plot_environment_and_links_dubins(links_maze_rrt, env = env_maze,start = start_maze,goal = goal_maze)


    # links_maze_rrtstar = RRTstarDubins(start_maze, goal_maze, inflated_env_maze, region_x_new,region_y_new, 500, step_size)
    # plot_environment_and_links_dubins(links_maze_rrtstar, env = env_maze,start = start_maze,goal = goal_maze)




