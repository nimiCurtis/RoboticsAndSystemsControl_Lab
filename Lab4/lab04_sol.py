import numpy as np

def traj_gen_config(q1, q2, t, Tf):
    ''' path plan configuration space '''
    qm = q1 + (q2 - q1)/2
    a0 = q1
    a1 = np.zeros((6,))
    a4 = (qm - 0.5 * (q1 + q2)) / ((Tf / 2) ** 4)
    a3 = (2 * (q1 - q2) / (Tf ** 3)) - 2 * Tf * a4
    a2 = -1.5 * a3 * Tf - 2 * a4 * Tf ** 2

    q = a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3 + a4 * t ** 4
    dq = a1 + 2 * a2 * t + 3 * a3 * t ** 2 + 4 * a4 * t ** 3
    ddq = 2 * a2 + 6 * a3 * t + 12 * a4 * t ** 2

    return q, dq, ddq

def traj_gen_task(x_s, x_g, t, Tf):

    """
    path plan in Task space
    x_s = Start point cartesian
    x_g = goal point cartesian
    """
    # x_s = np.array(list(x_s))   #start point
    # x_g = np.array(list(x_g))   #goal point
    a0 = 0. # np.zeros((3,))
    a1 = 0. # np.zeros((3,))
    a2 = 3 / Tf ** 2
    a3 = -2 / Tf ** 3
    x = x_s + (a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3) * (x_g - x_s)
    dx = (a1 + 2 * a2 * t + 3 * a3 * t ** 2) * (x_g - x_s)
    ddx = (2 * a2 + 6 * a3 * t) * (x_g - x_s)

    return x, dx, ddx

def generate_x_goals_list():
    """

    Returns: list of

    """
    # Osher
    x_ = np.array([[0.44, 0.187, 0.419, 96, 1, 150],
                   [0.369, -0.015, 0.21, 178, 3, 177],
                   [0.372, 0.014, 0.01, 178, 4.5, 175],
                   [0.44, 0.187, 0.419, 96, 1, 150]])
    ## 1st x_goals_list
    # x_ = np.array([[0.473,0.327,0.596,95.02,1.246,149.476],
    #                [0.339,-0.057,0.178,-173.919,6.867,177.152],
    #                [0.333,-0.058,0.003,176.147,9.303,176.937],
    #                [0.42,0.195,0.162,176.279,9.24,176.867]])
    ## 2nd x_goals_list
    # x_ = np.array([[0.289,-0.24,0.255,176.183,9.151,176.812],
    #                [0.288,-0.025,0.164,176.305,9.11,176.85],
    #                [0.288,-0.025,-0.003,176.313,9.202,176.883],
    #                [0.401,-0.025,0.266,176.297,9.12,176.85]])
    return x_


def generate_q_goals_list():
    """

    Returns: list of

    """
    # Osher
    jointPoses = np.array([[0.1, 343, 75, 354, 300, 0.1],
                           [7.5, 337, 80, 271, 287, 10],
                           [7.5, 313, 97, 272, 329, 10],
                           [0.1, 343, 75, 354, 300, 0.1]])
    ## 1st q_goals_list
    # jointPoses = np.array([[28.09,318.093,358.474,306.245,299.005,66.133],
    #                        [6.524,342.753,98.882,261.634,301.641,13.788],
    #                        [358.946,318.651,109.525,280.363,340.004,352.23],
    #                        [34.567,319.744,65.515,267.042,295.333,38.442]])
    ## 2nd q_goals_list
    # jointPoses = np.array([[325.468,337.702,69.597,278.086,277.667,327.673],
    #                        [6.767,353.127,113.455,272.71,309.901,8.058],
    #                        [6.482,321.739,122.224,282.166,349.944,357.498],
    #                        [4.6,336.055,65.962,272.478,279.43,7.254]])

    return jointPoses