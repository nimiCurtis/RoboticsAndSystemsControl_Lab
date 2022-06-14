"""
All right reserved to  Itamar Mishani and Osher Azulay
imishani@gmail.com (or imishani@andrew.cmu.edu), osherazulay@mail.tau.ac.il
"""

import matplotlib.pyplot as plt
import numpy as np
import time
import math
from scipy.spatial.transform import Rotation as R
import sys, os

from sympy import true

sys.path.insert(0, r'../common/Aruco_Tracker-master')
sys.path.insert(0, r'../Lab5')

from aruco_module import aruco_track
from Lab5_student import planner, steering_angle
from car_control import Controller
import cv2


def save(saver):
    logdir_prefix = 'lab-05'

    data_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../Lab5/data')

    if not (os.path.exists(data_path)):
        os.makedirs(data_path)

    logdir = logdir_prefix + '_' + time.strftime("%d-%m-%Y_%H-%M-%S")
    logdir = os.path.join(data_path, logdir)
    if not (os.path.exists(logdir)):
        os.makedirs(logdir)

    print("\n\n\nLOGGING TO: ", logdir, "\n\n\n")

    import pickle
    with open(logdir + '/data' + '.pkl', 'wb') as h:
        pickle.dump(saver, h)

def calc_motor_command(angle):
    '''

    Args:
        angle: steering angle from car pose to target pose

    Returns:
        left and right motor command to align with the steering angle
    '''
    x = angle / 180.
    if x <= 0:
        right = 1.
        left = -2 * x + 1
    else:
        left = 1.
        right = 2 * x + 1
    left = math.copysign(1, left) - left * 0.5
    right = math.copysign(1, right) - right * 0.5
    return -left, -right

def field_status():
    '''
    w.r.t camera frame
    Returns:

    '''
    t_curr, R_curr, ids = tracker.track()
    try:
        t_curr, R_curr, ids = t_curr.squeeze(), R_curr.squeeze(), ids.squeeze()
        trans, rot, homo = {}, {}, {}
        for i in range(len(ids)):
            trans[ids[i]] = t_curr[i, :]
            rot[ids[i]] = R.from_rotvec(R_curr[i, :]).as_matrix()
            homo[ids[i]] = np.vstack((np.hstack((rot[ids[i]], trans[ids[i]].reshape(-1, 1))), np.array([[0, 0, 0, 1]])))
        # Note that everything is with respect to the camera frame!
        p_r = np.linalg.inv(homo[axes_user_ID]) @ homo[car_ID]
        p_o = np.linalg.inv(homo[axes_user_ID]) @ homo[opponent_ID]
        p_d = np.linalg.inv(homo[axes_user_ID]) @ homo[disc_ID]
        return p_r, p_o, p_d

    except:
        print('Error! Cannot detect frames')
        cntrlr.motor_command(1., 1.)

def with_disk(disk_relative_to_robot , R = 0.025):
    distance_to_disk = np.linalg.norm(disk_relative_to_robot[:2])
    if( distance_to_disk <= R) :
        return True
    else:
        return False

def is_disk_moving(p_d, p_d_new, thresh=0.005):
    if np.linalg.norm(p_d - p_d_new)>=thresh:
        return True
    else:
        return False

def turn_to_disk():
    p_r, p_o, p_d = field_status()
    next , phi = steering_angle(p_r,p_d[:2,3])
    left, right = calc_motor_command(phi)
    cntrlr.motor_command(-left, -right) 

def reach_disk(p_r,p_d, path):
    print("finding the new position of the disk")
    tolerance = 0.01
    i = 1
    for next_goal in path:
        next_ = [10e2, 10e2]
        while np.linalg.norm(next_[:2]) > tolerance:
            print(f'Attempting to reach point: {i} of {len(path)}')
            t_curr, R_curr, ids = tracker.track() ## ??
            try:
                next_ , phi = steering_angle(p_r,next_goal) 
                #executed_path.append(list(curr))
                print(f' Phi: {round(phi)}, error: {next_[:2]}\n Distance: {np.linalg.norm(next_[:2])}')
            except:
                continue
            if np.linalg.norm(next_) <= tolerance:
                cntrlr.motor_command(1, 1)
                continue
            
            p_r_new, p_o_new, p_d_new = field_status()
            if is_disk_moving(p_d[:2,3],p_d_new[:2,3]):
                print("Disk is moving >> get new path plan!")
                break

            left, right = calc_motor_command(phi)
            cntrlr.motor_command(-left, -right)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        i += 1
        cntrlr.motor_command(1, 1)
    print("Reached disk!! ")

def keep_straight():
        print("Keep straight!!")
        cntrlr.motor_command(0.5, 0.5)

# r include robot radius and disk radius
def find_goal(p_d, r = 0.015):
    print("Finding the new position of the disk")
    return [p_d[0] - r , p_d[1] , p_d[2]] # x - r , 


if __name__ == "__main__":

    tracker = aruco_track()

    axes_user_ID = int(input('Enter axes user ID:   '))
    car_ID = int(input('Enter car ID:   '))
    opponent_ID = int(input('Enter opponent ID:   '))
    disc_ID = int(input('Enter disc ID:   '))

    cntrlr = Controller(car_ID)  # input car ID
    cntrlr.connect()
    time.sleep(1)
    cntrlr.motor_command(1., 1.)  # Don't move!

    executed_path = []

##########################################################
    while cntrlr.Connected: #and cntrlr.communicate:
        p_r, p_o, p_d = field_status()
        obs = []  ######################change
        path_ = planner((p_r)[:2, 3].tolist(), find_goal((p_d)[:2, 3].tolist()), obs, show_animation=True)
        disk_relative_to_robot , phi_to_disk = steering_angle(p_r , p_d[:2,3])
        if not(with_disk(disk_relative_to_robot)):
            reach_disk()  

        turn_to_disk()

        while(with_disk(disk_relative_to_robot)):
            keep_straight()

    # while cntrlr.Connected: #and cntrlr.communicate:
    #    if cv2.waitKey(1) & 0xFF == ord('q'):
    #         cntrlr.motor_command(1, 1)
    #         break
    print("Reached goal!! ")
    cntrlr.motor_command(1., 1.)
    tracker.cap.release()
    cv2.destroyAllWindows()
    if input('Save data? (y,n)   ') == 'y':
        save(executed_path)
        print('Saved data as list of size 2 where first element is planned path and second is executed path!')
    sys.exit(0)