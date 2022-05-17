# Import required packages
import numpy as np


def planner(Pc, Pg, O, B=[-0.05, 0.65], delta=0.02, **args):
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

    pass

def steering_angle(A_robot_cam, A_base_cam, p_i_base):
    """

    Args:
        A_robot_cam: Homogeneous matrix from car to camera frame
        A_base_cam: Homogeneous matrix from origin to camera frame
        p_i_base: 2d vector of next point in the path with respect to base (origin) frame: (x, y)

    Returns:
        p_i_car: 2d vector of next point in path with respect to car frame: (x, y)
        alpha: Steering angle to next point [degrees].
    """
    
    A_pi_base = np.concatenate(  (p_i_base , [1]) , axis=0)
    p_i_car = np.matmul( np.matmul(np.linalg.inv(A_robot_cam), A_base_cam),A_pi_base) 
    alpha = np.rad2deg(np.arctan2(p_i_car[0],p_i_car[1]))
    return p_i_car , alpha

def check_steering_angle():
    A_cam_robot = np.array([[-0.1614 ,-0.6982 , 0.6975, 0.412] ,
                                        [0.9769,-0.0127,0.2133 , 0.218 ] ,
                                        [ -0.1400 ,0.7158 , 0.6841 , 0.797 ] ,
                                        [ 0.,0. , 0. , 1. ] ] )
    A_cam_base = np.array([[ -0.7537, 0.6208 , 0.2157 , 0.112 ] ,
                                        [ -0.1910 , -0.5292 , 0.8246 , 0.801 ] ,
                                        [ 0.6261 , 0.5783 , 0.5230 , 0.797 ] ,
                                        [ 0. , 0. , 0. , 1. ]] )
    pi_base = np.array( [ 0.5 , 1.1 , 0. ] ) 
    

    print(steering_angle(A_cam_robot,A_cam_base,pi_base))


if __name__ == '__main__':
    check_steering_angle()


    
