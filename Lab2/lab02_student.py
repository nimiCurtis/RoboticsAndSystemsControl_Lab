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
    #print(T[0:3,3])

    ''' fill angles to dict for sympy calculations'''
    q = [q1, q2, q3, q4, q5, q6]
    theta_dict = {}
    for i in range(len(theta_list)):
        theta_dict[q[i]] = theta_list[i]
    #print(theta_dict)
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
    angles = {'t1': [np.deg2rad(332.134), np.deg2rad(324.773), np.deg2rad(46.888), np.deg2rad(359.105), np.deg2rad(272.026), np.deg2rad(13.36)],
              't2': [np.deg2rad(280.6), np.deg2rad(277.2), np.deg2rad(296.959), np.deg2rad(3.946), np.deg2rad(255.951), np.deg2rad(345.532)],
              't3': [np.deg2rad(269.757), np.deg2rad(279.518), np.deg2rad(83.362), np.deg2rad(39.263), np.deg2rad(268.774), np.deg2rad(318.686)],
              't4': [np.deg2rad(129.176), np.deg2rad(294.538), np.deg2rad(53.908), np.deg2rad(354.559), np.deg2rad(84.608), np.deg2rad(26.319)],
              't5': [np.deg2rad(346.403), np.deg2rad(284.409), np.deg2rad(103.395), np.deg2rad(16.35), np.deg2rad(270.916), np.deg2rad(271.911)],
              
              't6': [np.deg2rad(148.47), np.deg2rad(95.23), np.deg2rad(72.32), np.deg2rad(-20.18), np.deg2rad(86.52), np.deg2rad(100)],
              't7': [np.deg2rad(121.06), np.deg2rad(108.44), np.deg2rad(33.4), np.deg2rad(60.87), np.deg2rad(92.37), np.deg2rad(100)],
              't8': [np.deg2rad(-65.79), np.deg2rad(-38.99), np.deg2rad(109.77), np.deg2rad(132.68), np.deg2rad(-9.04), np.deg2rad(100)],
              't9': [np.deg2rad(80.01), np.deg2rad(-49.3), np.deg2rad(90.83), np.deg2rad(-176.38), np.deg2rad(62.4), np.deg2rad(100)],
              't10': [np.deg2rad(-176.39), np.deg2rad(142.94), np.deg2rad(122.91), np.deg2rad(-66.59), np.deg2rad(127.66), np.deg2rad(100)],
              't11': [np.deg2rad(168.89), np.deg2rad(83.2), np.deg2rad(-121.61), np.deg2rad(12.82), np.deg2rad(104.92), np.deg2rad(100)],

              't12': [np.deg2rad(-3.57), np.deg2rad(-89.55), np.deg2rad(-100.33), np.deg2rad(-66.04), np.deg2rad(-111.77), np.deg2rad(10)],
              't13': [np.deg2rad(-69.17), np.deg2rad(-1.58), np.deg2rad(112.58), np.deg2rad(47), np.deg2rad(-48.02), np.deg2rad(10)],
              't14': [np.deg2rad(132.63), np.deg2rad(104.62), np.deg2rad(37.88), np.deg2rad(59.11), np.deg2rad(-115.05), np.deg2rad(10)],
              't15': [np.deg2rad(-76.98), np.deg2rad(29.93), np.deg2rad(-103.45), np.deg2rad(17.4), np.deg2rad(-2.48), np.deg2rad(10)],
              't16': [np.deg2rad(28.53), np.deg2rad(-91.73), np.deg2rad(126.71), np.deg2rad(-105.42), np.deg2rad(135.09), np.deg2rad(10)],
              't17': [np.deg2rad(19.44), np.deg2rad(-29.42), np.deg2rad(118.01), np.deg2rad(40.54), np.deg2rad(-3.38), np.deg2rad(10)]

              }

    return angles



''' answer for prerequest exercise'''
angles = angles_to_follow()
t = ['t12','t13','t14','t15','t16','t17']
for angles_vec in t:
    T_ans = np.array(FK(angles[angles_vec]))
    print(T_ans)
    print("")

    
