from audioop import cross
import numpy as np
from sympy import *
from scipy.spatial.transform import Rotation as R

alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha1:7')
a1, a2, a3, a4, a5, a6 = symbols('a1:7')
d1, d2, d3, d4, d5, d6 = symbols('d1:7')
q1, q2, q3, q4, q5, q6 = symbols('q1:7')

'''
Hint1: you should use your functions from the previous lab
Hint2: using sympy is easier for debugging, but not mandatory
'''
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

def T_matrices():
    """
    Args:
        None
    Returns:
        Transformation matrices from joint to world axis system --> list of Matrixes((4, 4))
    
    """
    
    dh_dic = set_dh_table()
    Tes = Matrix([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])  # mandatory

    T_01 = dh(dh_dic[alpha1], dh_dic[a1], dh_dic[d1], dh_dic[q1])
    ###
    T_12 = dh(dh_dic[alpha2], dh_dic[a2], dh_dic[d2], dh_dic[q2])
    T_02 = T_01*T_12*Tes
    ###
    T_23 = dh(dh_dic[alpha3], dh_dic[a3], dh_dic[d3], dh_dic[q3])
    T_03 = T_01*T_12*T_23*Tes
    ###
    T_34 = dh(dh_dic[alpha4], dh_dic[a4], dh_dic[d4], dh_dic[q4])
    T_04 = T_01*T_12*T_23*T_34*Tes
    ###
    T_45 = dh(dh_dic[alpha5], dh_dic[a5], dh_dic[d5], dh_dic[q5])
    T_05 = T_01*T_12*T_23*T_34*T_45*Tes
    ###
    T_56 = dh(dh_dic[alpha6], dh_dic[a6], dh_dic[d6], dh_dic[q6])
    T_e = T_01*T_12*T_23*T_34*T_45*T_56*Tes # end effector transformation matrix
    return [T_01,T_02,T_03,T_04,T_05,T_e]



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
    

    ''' fill angles to dict for sympy calculations'''
    T = T_matrices()[-1] # get the end effector transformation matrix
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
    angles = {'t1': [np.deg2rad(300), np.deg2rad(300),0, np.deg2rad(300), np.deg2rad(300), 0]}

    return angles




def Jacobian(Q):
    '''

    Args:
        Q: joint configuration list [1,6]

    Returns:
        Full Jacobian matrix [6,6]
    '''
    q = [q1, q2, q3, q4, q5, q6] 
    theta_dict = {}
    for i in range(len(Q)):
        theta_dict[q[i]] = Q[i]
    
    T = T_matrices() # get the list of transformation matrixes 
    Te = T[-1] # get the end effector transformation matrix

    ## compute linear jacobian in method 1 --> by sympy jacobian function (differentail method)
    JL1 = Te[0:3,3].jacobian(q)
    
    ## compute linear jacobian in method 2 --> by geometric method ~ JLi = bi X ri_e
    ## compute ri_e
    d0_e = Te[:3,-1] # d0_e = r0_e 
    ##    
    d1_e = T[0][:3,-1]
    r1_e = d0_e - d1_e
    ##
    d2_e = T[1][:3,-1]
    r2_e = d0_e - d2_e
    ##
    d3_e = T[2][:3,-1]
    r3_e = d0_e - d3_e
    ##
    d4_e = T[3][:3,-1]
    r4_e = d0_e - d4_e
    ##
    d5_e = T[4][:3,-1]
    r5_e = d0_e - d5_e
    ##
    ## compute bi
    z_axis = Matrix([[0, 0 ,1]]).transpose()
    b0 =   z_axis    
    b1 =   T[0][:3,:3]*z_axis
    b2 =   T[1][:3,:3]*z_axis
    b3 =   T[2][:3,:3]*z_axis
    b4 =   T[3][:3,:3]*z_axis
    b5 =   T[4][:3,:3]*z_axis      
    
    # finally compute JL in method 2
    JL2 = Matrix([[b0.cross(d0_e) , b1.cross(r1_e) , b2.cross(r2_e) , b3.cross(r3_e) , b4.cross(r4_e) , b5.cross(r5_e)  ]])
    
    # compute JA
    JA = Matrix([[b0,b1,b2,b3,b4,b5]])
    
    # compute J -- choose JL1 or JL2
    J = Matrix([[JL1],[JA]])
    #J = Matrix([[JL2],[JA]])

    # compute J for the given thetas
    J_eval = J.evalf(subs=theta_dict, chop=True, maxn=4)

    return J_eval

def LinearJacobian(Q):
    '''
    Args:
        Q: joint configuration list [1,6]

    Returns:
        Linear part of the Jacobian matrix [3,6]
    '''
    J = Jacobian(Q) # get the full jacobian
    JL = J[0:3,:] # take just the linear jacobian
    return JL



def IK_NR_position(guess, target):
    '''

    Args:
        guess: initial angle guess list [1,6] {q1-6}
        target: task configuration tcp position [1,3] {x,y,z}

    Returns:
        Q* - joint configuration angles [1, 6]
    '''
    pass











