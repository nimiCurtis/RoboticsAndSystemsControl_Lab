import copy
import math
import random
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot
import numpy as np

#from AStar import RRTStar, show_animation
from rrt_star import  RRTStar , show_animation


def planner(Pc, Pg, O, B=[0, 0.5], expand_dis = .1, path_resolution = 0.01 ,robot_radius = 0.07, show_animation=True):
        
        
    """

    Args:
        Pc: start point (x_s, y_s) --> list: len=2 OR np.array(): shape=(2,)
        Pg: end point (x_g, y_g) --> list: len=2 OR np.array(): shape=(2,)
        O: [(x_obs_1, y_obs_2, radius_obs_1), ..., (x_obs_N, y_obs_N, radius_obs_N)
        B: this is the area where you plan the path in. both x and y should be in between these boundaries.
        delta: Path resolution.
        **args: add additional arguments as you please such as whether to plot a path or not.

    Returns:
        path: [[x_1, y_1], [x_2, y_2], ..., [x_M, y_M]] --> List of lists of size 2 (x, y).
                Important: Make sure that your output 'path' is in the right order (from start to goal)
    """
    rrt_star = RRTStar(
        start = Pc,
        goal = Pg,
        obstacle_list = O,
        rand_area = B,
        expand_dis = expand_dis,
        path_resolution = path_resolution,
        robot_radius = robot_radius)

    path = rrt_star.planning(animation = True) #

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")

        # Draw final path
        if show_animation:
            rrt_star.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], 'r--')
            plt.grid(True)
    # print(path)
    plt.pause(0.01)
    return path[::-1]

#-----------------------------------------------------------------#

def steering_angle(A_r, p_i):
    """

    Args:
        A r - Pose of its own robot relate to the base frame
        p i - next point in path relate to the base frame

    Returns:
        p_i_robot: 2d vector of next point in path with respect to robot frame: (x, y)
        phi: Steering angle to next point [degrees].
    """


    p_i = np.concatenate((p_i, [0,1]), axis=0) ## ??? dimensions
    p_i_robot = np.matmul(np.linalg.inv(A_r), p_i)[0:2]
    phi = np.rad2deg(np.arctan2(p_i_robot[0], p_i_robot[1]))
    return p_i_robot , phi

def main():
    print("Start " + __file__)

    # ====Search Path with RRT====
    obstacle_list = [

    ]  # [x,y,size(radius)]

    # Set Initial parameters
    rrt_star = RRTStar(
        start=[0, 0],
        goal=[0.9, 0.6],
        rand_area=[-0.5, 0.5],
        obstacle_list=obstacle_list,
        expand_dis=0.5,
        robot_radius=0.001)
    path = rrt_star.planning(animation=show_animation)

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")

        # Draw final path
        if show_animation:
            rrt_star.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], 'r--')
            plt.grid(True)
    plt.show()


if __name__ == '__main__':
     main()