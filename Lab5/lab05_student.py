import numpy as np
from sympy import jacobi
import transformations
from scipy.spatial.transform import Rotation as R
import modern_robotics as mr
from numpy.linalg import inv

# need to use the libraries above


def calculate_error(t_curr, R_curr, target_feature_t, target_feature_R, translation_only=False):
    '''
    Calculate error based on the input pose and the target pose
    Input:  (object in current camera frame)
            t_input, 1x3 vector
            R_input, 3x3 matrix
    Output: Error, [t_err, R_err], 6x1
    '''

    t_del = t_curr - target_feature_t # delta of position
    R_multi = np.matmul(R_curr,np.transpose(target_feature_R))
    r = R.from_matrix(R_multi)
    theta_u = r.as_rotvec()
    


    # see paragraph above Eq.13
    # of Chaumette, Francois, and Seth Hutchinson. "Visual servo control. I. Basic approaches."
    # https://hal.inria.fr/inria-00350283/document



    if translation_only:
        error = np.hstack((t_del, np.zeros(3)))
    else:
        error = np.hstack((t_del, theta_u ))

    return error

def feature_jacobian(t_curr, R_curr, target_feature_R):
    '''
    form interaction matrix / feature jacobian base on current camera pose
    Input:  (object in current camera frame)
            t_input, 1x3 vector
            R_input, 3x3 matrix
    Output: Interaction Matrix (feature Jacobian), 6x6
    '''


    R_multi = R_curr*np.transpose(target_feature_R)
    r = R.from_matrix(R_multi)
    theta_u = r.as_rotvec()
    u , theta =  mr.AxisAng3(theta_u)


    p_curr = t_curr # current position vector p
    S = mr.VecToso3(p_curr) # skew-symmetric representation of a vector p
    J_theta_u = np.identity(3) - (theta/2)*S+(1-(np.sinc(theta)/(np.sinc(theta/2)**2)))*(S**2)
    lines_1to3 = np.concatenate([-np.identity(3), S], axis=1)
    lines_4to6 = np.concatenate([np.zeros((3,3)),J_theta_u], axis= 1)
    L_out = np.concatenate([lines_1to3, lines_4to6], axis=0) # The Jacobian J of the gripper 6*6
    return L_out

def control(L, error, _lambda):
    '''
    calculate the twist of camera frame required to reach target pose
    Input:  (object in current camera frame)
            t_input, 1x3 vector <<<<< wrong
            R_input, 3x3 matrix <<<<< wrong
    Output: Twist in camera frame
            [nu_c, omg_c], 1x6
    '''
    t = -_lambda*np.matmul(inv(L),error)

    return np.transpose(t) 

def check():
    t_curr = np.array([0, 0, 1])
    R_curr = np.array([[0, 0, 1],
                    [0.2 , 0.4 , 0.5],
                    [0.7 ,0 ,1]])
    target_feature_t = np.array([1, 0.2, 0.5])
    target_feature_R = np.array([[0, 1, 1],
                    [0.1 , 1 , 0],
                    [0.2 ,0 ,0]])
    print(calculate_error(t_curr, R_curr, target_feature_t, target_feature_R))
    print("")
    print(feature_jacobian(t_curr, R_curr, target_feature_R))
    print("")
    print(control(feature_jacobian(t_curr, R_curr, target_feature_R) ,calculate_error(t_curr, R_curr, target_feature_t, target_feature_R),2 ))

if __name__ == '__main__':
    check()