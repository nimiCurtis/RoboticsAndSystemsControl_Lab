import numpy as np


#######################
######## Lab 4 ########
#######################

def traj_gen_config(q1, q2, t, Tf):
    """
    path plan configuration space

    Args:
        q1: Start configuration (angles) [degrees] --> np.array((6,))
        q2: Goal configuration (angles) [degrees] --> np.array((6,))
        t: Time instance --> float
        Tf: Total movement time --> float

    Returns: angles positions, angles velocities, angles accelerations
                --> q, dq, ddq --> 3 object of np.array((6,))

    """
    ## equation of angle pos is : q(t) = a0 + a1t +a2t^2 +a3t^2 +a4t^3 +a5t^4 
    ## need to define start&end vel and accel q'(0),q'(Tf) & q''(0),q''(Tf)
    ## according to start&end pos,vel,accel we can calc a0 - a5
    ## and then we can find q'(t) , q''(t) of a given t
    
    # define w1 = start vel & w2 = end vel [deg/s]
    w1 = np.array([0 , 0 , 0 , 0 , 0 , 0 ]) 
    w2 = np.array([0 , 0 , 0 , 0 , 0 , 0 ])

    # define alpha1 = start accel & alpha2 = end accel [deg/(s^2)]
    alpha1 = np.array([0 , 0 , 0 , 0 , 0 , 0 ])
    alpha2 = np.array([0 , 0 , 0 , 0 , 0 , 0 ])

    # based on the algebric equation X = Aa
    # set results 'X' vector  --> np.array((6,6)) 
    X  = np.array([q1,w1,alpha1,q2,w2,alpha2])

    # set coefficients 'A' matrix --> np.array((6,6))
    A = np.array([[1 , 0 , 0 , 0 , 0 , 0],
                 [0 , 1 , 0 , 0 , 0 , 0],
                 [0 , 0 , 2 , 0 , 0 , 0],
                 [1 , Tf , Tf**2 ,    Tf**3  ,    Tf**4  ,    Tf**5  ],
                 [0 , 1  , 2*Tf , 3*(Tf**2) , 4*(Tf**3) , 5*(Tf**4) ],
                 [ 0 , 0 , 2    ,  6*Tf    , 12*(Tf**2), 20*(Tf**3)]]
    )


    # calc coefficients 'a' vector by a = inv(A)*X --> np.array((6,6))
    a = np.matmul(np.linalg.inv(A),X)

    # calc q,dq,ddq by the quintic polynomial equation and the derivatives of q(t) >> writen in the first comment 
    q   = np.dot( np.array([1,t,t**2,t**3,t**4,t**5]),a ) # equals to a[0]*1 + a[1]*t + a[2]*t^2 + ...  --> np.array((6,)) [deg]
    dq  = np.dot( np.array([0,1,2*t,3*(t**2),4*(t**3),5*(t**4)]), a) # --> np.array((6,)) [deg/s]
    ddq = np.dot( np.array([0,0,2,6*t,12*(t**2),20*(t**3)]) ,a ) # --> np.array((6,)) [deg/(s^2)]

    return [q , dq , ddq]

def traj_gen_task(x_s, x_g, t, Tf):
    """
    path plan in Task space

    Args:
        x_s: Start end-effector position and orientation UNITS:[m, degrees] --> np.array((6,))
        x_g: Goal end-effector position and orientation UNITS:[m, degrees] --> np.array((6,))
        t: Time instance --> float
        Tf: Total movement time --> float

    Returns: End-effector position, velocity, acceleration
                --> x, dx, ddx --> 3 object of np.array((6,))

    """

    pass

def generate_x_goals_list():
    """

    Returns: Desired end-effector goals along the planned path --> np.array((4, 6))

    Notes:  1. Position units [m]
            2. Orientation units [degrees]

    """
    pass

def generate_q_goals_list():
    """

    Returns: Desired configuration (angle) goals along the planned path --> --> np.array((4, 6))

    Notes: Orientation units [degrees]

    """

    pass

## function made by nimi for checking the first function
def check_traj_gen_config():
    q1 = np.array([100,100,100,100,100,100])  # [degrees] --> np.array((6,))
    q2 = np.array([10,10,10,10,10,10]) # [degrees] --> np.array((6,))
    t = 0.2  # [sec]
    Tf = 1 # [sec]
    [q,dq,ddq] = traj_gen_config(q1,q2,t,Tf)

    print("\n q = ")
    print(q)

    print("\n dq = ")
    print(dq)

    print("\n ddq = ")
    print(ddq)


if __name__ == "__main__" :
    check_traj_gen_config()

