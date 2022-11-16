import pytest
from utilities import LinkDubins
from shapely.geometry import Point
from dubins_path_planning import dubins_path_planning
import numpy as np

curvature = 1.0
env_multi = [
    Point(0, -0.5).buffer(1.5, cap_style=3),
    Point(3, 0).buffer(2.0, cap_style=3),
    Point(2, 5).buffer(2.5, cap_style=3),
]
start_yaw = np.deg2rad(45.0)
end_yaw = np.deg2rad(90.0)
start = [-4, -4, start_yaw]
goal = [6, 6, end_yaw]
step_size=2.0
region_x = [-5, 10]
region_y = [-5, 10]
start_link = LinkDubins(start)
path_x, path_y, path_yaw, mode, lengths = dubins_path_planning(start[0],
                                                                start[1],
                                                                start_yaw,
                                                                goal[0],
                                                                goal[1],
                                                                end_yaw,
                                                                curvature)
goal_link = LinkDubins(goal,path_x,path_y,length = sum(lengths),upstream=start_link)

def test_link_collision():
    assert(goal_link.does_path_collide(env_multi) == True)

def test_upstream_local_path_length():
    assert(goal_link.upstream.local_path_length == 0)

def test_start_link_path_length():
    assert start_link.local_path_length == 0

def test_path_length():
    assert(goal_link.path_length == goal_link.local_path_length)