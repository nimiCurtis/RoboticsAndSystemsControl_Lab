import random
from re import A
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot
import numpy as np
from sympy import false, true

#from AStar import RRTStar, show_animation
from rrt_star import  RRTStar , show_animation

class Robot:
    def __init__(self, A_r, A_o , disk):
        self.Rr = 0.075
        
        self.right_corner = [1.55 , 0.05]
        self.left_corner = [1.55 , 0.75]
        self.A_r = A_r
        self.p_r = self.A_r[:2 , 3]
        self.A_o = A_o
        self.p_e = self.A_o[:2, 3]
        self.res = 30


        self.disk = disk
        self.reach_disk = 1.1*(self.Rr + self.disk.Rd)

    def set_pose(self, pose):
        self.A_r = pose
        self.p_r = self.A_r[:2 , 3]

    def get_pose(self):
        return self.A_r

    def get_position(self):
        return self.p_r


    def with_disk(self, disk_relative_to_robot ):
        distance_to_disk = np.linalg.norm(disk_relative_to_robot[:2])
        if( distance_to_disk <= self.reach_disk) :
            return True
        else:
            return False

    def steering_angle(self, p_i):
        """
        Args:
            A r - Pose of its own robot relate to the base frame
            p i - next point in path relate to the base frame

        Returns:
            p_i_robot: 2d vector of next point in path with respect to robot frame: (x, y)
            phi: Steering angle to next point [degrees].
        """


        p_i = np.concatenate((p_i, [0,1]), axis=0) 

        p_relative_to_robot = np.matmul(np.linalg.inv(self.A_r), p_i)[0:2]
        phi = np.rad2deg(np.arctan2(p_relative_to_robot[0], p_relative_to_robot[1]))
        return p_relative_to_robot , phi

    def disk_pos(self):
        return self.steering_angle(self.disk.get_position())

    def enemy_pos(self):
        return self.steering_angle(self.p_e)

    
    def corner_path(self):
        p_e_relative_to_robot, phi_e = self.enemy_pos(self.p_e)
        left_corner_relative_to_robot, phi_left_relative_to_robot= self.steering_angle(self.left_corner)
        right_corner_relative_to_robot, phi_right_relative_to_robot= self.steering_angle(self.right_corner)
        d_left=np.cross(left_corner_relative_to_robot, p_e_relative_to_robot)/np.linalg.norm(left_corner_relative_to_robot)
        d_right=np.cross(right_corner_relative_to_robot, p_e_relative_to_robot)/np.linalg.norm(right_corner_relative_to_robot)
        
        if d_left <= d_right:
            path = np.linspace(self.p_r, self.right_corner, self.res)
        else:
            path = np.linspace(self.p_r, self.left_corner, self.res)
        return path ## may be path[::-1]
    
    def find_attacking_position(self):
        p_d = self.disk.get_position()
        return [p_d[0] - self.disk.Rd  , p_d[1]]

    def planner_path(self, O, B=[0,0.2 ], expand_dis = .1, path_resolution = 0.001, show_animation=True):
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
            start = self.get_position(),
            goal = self.find_attacking_position(),
            obstacle_list = O,
            rand_area = B,
            expand_dis = expand_dis,
            path_resolution = path_resolution,
            robot_radius = self.Rr)

        path = rrt_star.planning(animation = True)

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


    def reach_disk(self):
        pass 


class Disk:
    def __init__(self, A_d):
        self.Rd = 0.035
        self.A_d = A_d
        self.p_d = self.A_d[:2,3]
        self.last_p_d = self.p_d

    def set_pose(self , pose):
        self.A_d = pose
        self.p_d = self.A_d[:2 , 3]

    def get_position(self):
        return self.p_d

    def is_moving(self):
        if(np.linalg.norm([self.last_p_d,self.p_d]) < 0.05) :
            return true
        else:
            return false



