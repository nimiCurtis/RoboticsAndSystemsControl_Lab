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

    dh_subs_dict = {alpha1: pi / 2, a1: 0, d1: 0.2433, q1: q1,
                    alpha2: pi, a2: 0.280, d2: 0.030, q2: q2 + pi / 2,
                    alpha3: pi / 2, a3: 0, d3: 0.020, q3: q3 + pi / 2,
                    alpha4: pi / 2, a4: 0, d4: 0.245, q4: q4 + pi / 2,
                    alpha5: pi / 2, a5: 0, d5: 0.057, q5: q5 + pi,
                    alpha6: 0, a6: 0, d6: 0.235, q6: q6 + pi / 2}

    return dh_subs_dict




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

    return Matrix([[cos(theta), -cos(alpha) * sin(theta), sin(alpha) * sin(theta), a * cos(theta)],
                   [sin(theta), cos(alpha) * cos(theta), -sin(alpha) * cos(theta), a * sin(theta)],
                   [0, sin(alpha), cos(alpha), d],
                   [0, 0, 0, 1]])



def transform_matrices():
    dh_dic = set_dh_table()

    T_01 = dh(dh_dic[alpha1], dh_dic[a1], dh_dic[d1], dh_dic[q1])
    T_12 = dh(dh_dic[alpha2], dh_dic[a2], dh_dic[d2], dh_dic[q2])
    T_23 = dh(dh_dic[alpha3], dh_dic[a3], dh_dic[d3], dh_dic[q3])
    T_34 = dh(dh_dic[alpha4], dh_dic[a4], dh_dic[d4], dh_dic[q4])
    T_45 = dh(dh_dic[alpha5], dh_dic[a5], dh_dic[d5], dh_dic[q5])
    T_56 = dh(dh_dic[alpha6], dh_dic[a6], dh_dic[d6], dh_dic[q6])

    Tes = Matrix([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])  # mandatory
    T = T_01 * T_12 * T_23 * T_34 * T_45 * T_56 * Tes
    return T

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
    q = [q1, q2, q3, q4, q5, q6]
    theta_dict = {}
    for i in range(len(theta_list)):
        theta_dict[q[i]] = theta_list[i]

    ''' 
    homogeneous transformation matrix from base_link to end_effector [type: numeric matrix] 
    because we are using sympy, we have to use evalf.
    '''
    T_0G_eval = T.evalf(subs=theta_dict, chop=True, maxn=4)
    return T_0G_eval




def xyz_euler(A):
    """
    Extract translation and orientation in euler angles
    Args:
        A: Homogeneous transformation --> np.array((4, 4))
    Returns: x, y, z, thetax, thetay, thetaz --> np.array((1, 6))
    Important note: use numpy arrays
    """
    x = A[0, 3]
    y = A[1, 3]
    z = A[2, 3]
    r = R.from_matrix(A[0:3, 0:3])
    r_euler = r.as_euler('xyz', degrees=True)
    pos_euler_vec = np.array([x, y, z, r_euler[0], r_euler[1], r_euler[2]])
    print(pos_euler_vec)
    return pos_euler_vec
    pass


def angles_to_follow():
    """
    Returns: Dictionary of the desired angels
    """
    angles = {'t1': [np.deg2rad(319.848), np.deg2rad(293.937), np.deg2rad(341.041), np.deg2rad(33.246), np.deg2rad(306.064), np.deg2rad(10.556)],
              't2': [np.deg2rad(5.337), np.deg2rad(339.071), np.deg2rad(113.483), np.deg2rad(326.721), np.deg2rad(102.230), np.deg2rad(314.408)],
              't3': [np.deg2rad(110.799), np.deg2rad(293.646), np.deg2rad(018.081), np.deg2rad(274.031), np.deg2rad(003.567), np.deg2rad(14.832)],
              't4': [np.deg2rad(154.117), np.deg2rad(329.840), np.deg2rad(103.438), np.deg2rad(219.285), np.deg2rad(047.641), np.deg2rad(44.554)],
              't5': [np.deg2rad(205.872), np.deg2rad(314.754), np.deg2rad(113.501), np.deg2rad(261.057), np.deg2rad(086.366), np.deg2rad(031.827)],
              't6': [np.deg2rad(381.434), np.deg2rad(326.417), np.deg2rad(005.659), np.deg2rad(260.705), np.deg2rad(017.092), np.deg2rad(84.800)],
              't7': [np.deg2rad(323.943), np.deg2rad(000.967), np.deg2rad(354.330), np.deg2rad(231.825), np.deg2rad(353.371), np.deg2rad(084.333)],
              't8': [np.deg2rad(011.785), np.deg2rad(041.338), np.deg2rad(005.399), np.deg2rad(224.618), np.deg2rad(051.709), np.deg2rad(93.408)],
              't9': [np.deg2rad(240.502), np.deg2rad(046.017), np.deg2rad(354.122), np.deg2rad(263.023), np.deg2rad(46.490), np.deg2rad(73.568)],
              't10': [np.deg2rad(237.685), np.deg2rad(003.525), np.deg2rad(063.533), np.deg2rad(309.917), np.deg2rad(290.173), np.deg2rad(276.608)]}

    return angles

    pass


# ''' answer for prerequest exercise'''
#
# angles = angles_to_follow()
# t = ['t1', 't2', 't3', 't4', 't5']
# for angles_vec in t:
#     T_ans = np.array(FK(angles[angles_vec]))
#     xyz_euler(T_ans)
