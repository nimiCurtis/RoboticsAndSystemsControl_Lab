from re import A
import numpy as np
from sympy import *
from scipy.spatial.transform import Rotation as R

alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha1:7')
a1, a2, a3, a4, a5, a6 = symbols('a1:7')
d1, d2, d3, d4, d5, d6 = symbols('d1:7')
q1, q2, q3, q4, q5, q6 = symbols('q1:7')


def set_dh_table():
    """

    Returns: dictionary

    Important: all length arguments in [m]
               all angles argument in [radians]

    """

    dh_subs_dict = { alpha1: pi/2, a1: 0    , d1:0.2433 , q1: q1 ,
                     alpha2: pi  , a2: 0.280, d2: 0.030 , q2: q2 + pi/2 ,
                     alpha3: pi/2, a3: 0    , d3: 0.020 , q3: q3 + pi/2,
                     alpha4: pi/2, a4: 0    , d4: 0.245 , q4: q4 + pi/2,
                     alpha5: pi/2, a5: 0    , d5: 0.057 , q5: q5 + pi,
                     alpha6: 0   , a6: 0    , d6: 0.235 , q6: q6 + pi/2}

    return dh_subs_dict

    pass


def dh(alpha, a, d, theta):
    """
    Args:
        alpha: torsion angle
        a: distance
        d: translation
        theta: rotation angle

    Returns: Homogeneous DH matrix

    Important note: use sympy cos/sin arguments instead of math/numpy versions.
    i.e: cos(theta) \ sin(theta)

    """

    
    return Matrix([[cos(theta), -cos(alpha)*sin(theta) ,sin(alpha)*sin(theta) ,a*cos(theta) ],
                   [sin(theta),cos(alpha)*cos(theta) ,-sin(alpha)*cos(theta) ,a*sin(theta) ],
                   [0 ,sin(alpha) ,cos(alpha) ,d ],
                   [0 ,0 ,0 ,1 ]])

    pass

def FK(theta_list):

    """
    Args:
        theta_list: joint angle vector ---> list [1,6]
    Returns:
        End effector homogeneous matrix --> Matrix((4, 4))

    Hints:
        - we have added a sympy implementation with missing parts, u dont have to use the same method.
        - chain 'Tes' to T_06 at the end.
    """
    dh_dic = set_dh_table()

    T_01 = dh(dh_dic[alpha1], dh_dic[a1], dh_dic[d1], dh_dic[q1])
    T_12 = dh(dh_dic[alpha2], dh_dic[a2], dh_dic[d2], dh_dic[q2])
    T_23 = dh(dh_dic[alpha3], dh_dic[a3], dh_dic[d3], dh_dic[q3])
    T_34 = dh(dh_dic[alpha4], dh_dic[a4], dh_dic[d4], dh_dic[q4])
    T_45 = dh(dh_dic[alpha5], dh_dic[a5], dh_dic[d5], dh_dic[q5])
    T_56 = dh(dh_dic[alpha6], dh_dic[a6], dh_dic[d6], dh_dic[q6])

    Tes = Matrix([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])  # mandatory
    T = T_01 * T_12 * T_23 * T_34 * T_45 * T_56 * Tes

    ''' fill angles to dict for sympy calculations'''
    q = [q1,q2,q3,q4,q5,q6]
    theta_dict = {}
    for i in range(len(theta_list)):
        theta_dict[q[i]] = theta_list[i]

    ''' 
    homogeneous transformation matrix from base_link to end_effector [type: numeric matrix] 
    because we are using sympy, we have to use evalf.
    '''
    T_0G_eval = T.evalf(subs=theta_dict, chop=True, maxn=4)
    return T_0G_eval

    pass

def xyz_euler(A):
    """
    Extract translation and orientation in euler angles

    Args:
        A: Homogeneous transformation --> np.array((4, 4))

    Returns: x, y, z, thetax, thetay, thetaz --> np.array((1, 6))

    Important note: use numpy arrays

    """
    x = A[0,3]
    y = A[1,3]
    z = A[2,3]
    r = R.from_matrix(A[0:3,0:3])
    r_euler=r.as_euler('xyz' , degrees = True)
    pos_euler_vec = np.array([x,y,z,r_euler[0],r_euler[1],r_euler[2]])
    print(pos_euler_vec)
    return pos_euler_vec
    pass

def angles_to_follow():
    """

    Returns: Dictionary of the desired angels

    """
    angles = { 't1': [0, -pi/7,pi/7 ,0 ,0 ,pi/7 ],
               't2': [np.deg2rad(300),np.deg2rad(300) ,0 ,np.deg2rad(300) ,np.deg2rad(300) ,0 ],
               't3': [0,np.deg2rad(340) ,np.deg2rad(75) ,0 ,np.deg2rad(300) ,0 ],
               't4': [pi/10,pi/10 ,pi/10 ,pi/10 ,pi/10 ,pi/10 ],
               't5': [0 ,0 ,0 ,0 ,0 ,0 ]}  # [radians]


    return angles

    pass


''' answer for prerequst exercise'''

angles  =  angles_to_follow()
t = ['t1','t2','t3','t4','t5']
for angles_vec in t:
    T_ans =np.array(FK(angles[angles_vec]))
    xyz_euler(T_ans)
