from shapely.geometry import Point
from shapely.geometry import LineString
from shapely.geometry import box
from plotting import plot_env
from plotting import LENGTH as vehicle_length
import matplotlib.pyplot as plt

def get_assignment2_environment():
    env_multi = [
        Point(0, -0.5).buffer(1.5, cap_style=3),
        Point(3, 0).buffer(2.0, cap_style=3),
        Point(2, 5).buffer(2.5, cap_style=3),
    ]
    inflated_env_multi = [
        Point(0, -0.5).buffer(1.5 + vehicle_length / 2, cap_style=3),
        Point(3, 0).buffer(2.0 + vehicle_length / 2, cap_style=3),
        Point(2, 5).buffer(2.5 + vehicle_length / 2, cap_style=3),
    ]
    region_x = [-5, 10]
    region_y = [-5, 10]
    return env_multi,inflated_env_multi,region_x,region_y

def get_maze_environment():
    MAZE_X = [-5,7]
    MAZE_Y = [-5,7]
    BOUNDARY = 0.5
    env_new = [
        box(minx=-3, miny=-5, maxx=-2, maxy=2),
        box(minx= 0, miny =1, maxx = 1, maxy = 7),
        box(minx= 3, miny =-5, maxx = 4, maxy = 2),
        # add the boundary
        box(minx = MAZE_X[0], miny = MAZE_Y[0], maxx = MAZE_X[1], maxy = MAZE_Y[0]+BOUNDARY),
        box(minx = MAZE_X[0], miny = MAZE_Y[0], maxx = MAZE_X[0]+BOUNDARY, maxy = MAZE_Y[1]),
        box(minx = MAZE_X[0], miny = MAZE_Y[1]-BOUNDARY, maxx = MAZE_X[1], maxy = MAZE_Y[1]),
        box(minx = MAZE_X[1]-BOUNDARY, miny = MAZE_Y[0], maxx = MAZE_X[1], maxy = MAZE_Y[1]),
    ]
    BOUNDARY_INFLATED = BOUNDARY + vehicle_length / 2
    inflated_env_new = [
        box(minx=-3- vehicle_length / 2, miny=-5- vehicle_length / 2, maxx=-2+ vehicle_length / 2, maxy=2+ vehicle_length / 2),
        box(minx= 0- vehicle_length / 2, miny =1- vehicle_length / 2, maxx = 1+ vehicle_length / 2, maxy = 7+ vehicle_length / 2),
        box(minx= 3- vehicle_length / 2, miny =-5- vehicle_length / 2, maxx = 4+ vehicle_length / 2, maxy = 2+ vehicle_length / 2),
        # add the boundary
        box(minx = MAZE_X[0], miny = MAZE_Y[0], maxx = MAZE_X[1], maxy = MAZE_Y[0]+BOUNDARY_INFLATED),
        box(minx = MAZE_X[0], miny = MAZE_Y[0], maxx = MAZE_X[0]+BOUNDARY_INFLATED, maxy = MAZE_Y[1]),
        box(minx = MAZE_X[0], miny = MAZE_Y[1]-BOUNDARY_INFLATED, maxx = MAZE_X[1], maxy = MAZE_Y[1]),
        box(minx = MAZE_X[1]-BOUNDARY_INFLATED, miny = MAZE_Y[0], maxx = MAZE_X[1], maxy = MAZE_Y[1]),
    ]

    region_x = [-4, 6]
    region_y = [-4, 6]
    return env_new,inflated_env_new,region_x,region_y


def get_forest_environment():
    env_forest = [
        Point(-2, -2).buffer(1),
        Point(-2, 2).buffer(1),
        Point(-2, 6).buffer(1),
        Point(2,2).buffer(1),
        Point(2,6).buffer(1),
        Point(2,-2).buffer(1),
        Point(6,4).buffer(1),
        Point(6,0).buffer(1)
    ]
    inflated_env_forest = [
        Point(-2, -2).buffer(1+ vehicle_length / 2),
        Point(-2, 2).buffer(1+ vehicle_length / 2),
        Point(-2, 6).buffer(1+ vehicle_length / 2),
        Point(2,2).buffer(1+ vehicle_length / 2),
        Point(2,6).buffer(1+ vehicle_length / 2),
        Point(2,-2).buffer(1+ vehicle_length / 2),
        Point(6,4).buffer(1+ vehicle_length / 2),
        Point(6,0).buffer(1+ vehicle_length / 2)
    ]
    region_x = [-5, 10]
    region_y = [-5, 10]
    return env_forest,inflated_env_forest,region_x,region_y

if __name__ == '__main__':
    start = [-4,-4,1.2]
    end = [6,6,2.3]
    env_maze,inflated_env_maze,_,_ = get_maze_environment()

    plt.figure(figsize=(8, 8), dpi=150)
    for poly in env_maze:
        plt.plot(*poly.exterior.xy, 'k')
    
    plt.plot(start[0], start[1], 'b.')
    plt.plot(end[0], end[1], 'g*')
    plt.title("Maze environment")
    plt.gca().axis('equal')


    plt.figure(figsize=(8, 8), dpi=150)
    for poly in inflated_env_maze:
        plt.plot(*poly.exterior.xy, 'k')
    
    plt.plot(start[0], start[1], 'b.')
    plt.plot(end[0], end[1], 'g*')
    plt.title("Inflated maze environment")
    plt.gca().axis('equal')

    plt.show()
