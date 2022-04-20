from turtle import color
import numpy as np
import pickle
import glob
import matplotlib.pyplot as plt
from lab04_sol import traj_gen_config , traj_gen_task
from lab04_student_final import traj_gen_config_5order
from lab02_student import FK




## set your own path to data dir ##
#data_path = 'f:/DATA/robotics_lab/RoboticsAndSystemsControl_Lab/Lab4/data/data'
data_path = 'C:/Users/15coh/Desktop/RoboticsAndSystemsControl_Lab/Lab4/data/data'
## data organized as array of arrays of arrays ~ data[number of trial]['0' for angles array / '1' for xyz][index of joint / index of coordinate]
data = []

for file in glob.glob(data_path + '/*/*.pkl'):
    with open(file, 'rb') as h:
        data.append(pickle.load(h))

## set time constants
Tf = 3.  # taken from lab_exe code 

## define theoretical arrays and trials arrays with informative names


## --------------------------------------------------------------------- ##
## trial 1 ##
## task space goals
x_1 = np.array([[0.473,0.327,0.596,95.02,1.246,149.476],
                    [0.339,-0.057,0.178,-173.919,6.867,177.152],
                    [0.333,-0.058,0.003,176.147,9.303,176.937],
                    [0.42,0.195,0.162,176.279,9.24,176.867]
                    ])
## conf space goals
jointPoses1 = np.array([[28.09,318.093,358.474,306.245,299.005,66.133],
                            [6.524,342.753,98.882,261.634,301.641,13.788],
                            [358.946,318.651,109.525,280.363,340.004,352.23],
                            [34.567,319.744,65.515,267.042,295.333,38.442],
                            [28.09,318.093,358.474,306.245,299.005,66.133]])

## set trial array
trial1_conf = data[0][1]
#print(trial1_conf)
trial1_task = data[1][1][9:]
## find theoretical path
theo1_conf = [] ## doesnt work
theo1_task = []

## set N to be the number of samples
N1 = len(trial1_task)  # taken from lab_exe code
t1= np.linspace(0,Tf,int(N1/3))


## use the functions from the pre-lab
for i in range(3):
    for dt in t1:
        #theo1_conf.append(list(traj_gen_config(jointPoses1[i] , jointPoses1[i+1] , dt , Tf )[0]))
        theo1_conf.append(list(FK(traj_gen_config(np.deg2rad(jointPoses1[i]) , np.deg2rad(jointPoses1[i+1]) , dt , Tf )[0])[0:3,3])) # takes just the q array from the return of traj_gen_config function
        theo1_task.append(list(traj_gen_task(x_1[i] , x_1[i+1] , dt , Tf )[0])) # takes just the x array from the return of traj_gen_task function

## convert to np.array because of plotting method  
theo1_task = np.array(theo1_task)
theo1_conf = np.array(theo1_conf)
#print(theo1_conf)

## --------------------------------------------------------------------- ##

## trial 2 ##
## task space goals
x_2 = np.array([[0.289,-0.24,0.255,176.183,9.151,176.812],
                    [0.288,-0.025,0.164,176.305,9.11,176.85],
                    [0.288,-0.025,-0.003,176.313,9.202,176.883],
                    [0.401,-0.025,0.266,176.297,9.12,176.85]
                    ])
## conf space goals
jointPoses2 = np.array([[325.468,337.702,69.597,278.086,277.667,327.673],
                            [6.767,353.127,113.455,272.71,309.901,8.058],
                            [6.482,321.739,122.224,282.166,349.944,357.498],
                            [4.6,336.055,65.962,272.478,279.43,7.254],
                            [325.468,337.702,69.597,278.086,277.667,327.673]])

## set trial array
trial2_conf = data[2][1]
trial2_task = data[3][1][6:]
## find theoretical path
theo2_conf = [] ## doesnt working
theo2_task = []

## set N to be the number of samples
N2 = len(trial2_task)  # taken from lab_exe code
t2= np.linspace(0,Tf,int(N2/3))

## use the functions from the pre-lab
for i in range(3):
    for dt in t2:
        theo2_conf.append(list(FK(traj_gen_config(np.deg2rad(jointPoses2[i]) , np.deg2rad(jointPoses2[i+1]) , dt , Tf )[0])[0:3,3])) # takes just the q array from the return of traj_gen_config function
        theo2_task.append(list(traj_gen_task(x_2[i] , x_2[i+1] , dt , Tf )[0])) # takes just the x array from the return of traj_gen_task function

## convert to np.array because of plotting method  
theo2_task = np.array(theo2_task)
theo2_conf = np.array(theo2_conf)

## --------------------------------------------------------------------- ##

## trial 3 ##
## task space goals
x_3 = np.array([[0.44, 0.187, 0.419, 96, 1, 150],
                   [0.369, -0.015, 0.21, 178, 3, 177],
                   [0.372, 0.014, 0.01, 178, 4.5, 175],
                   [0.44, 0.187, 0.419, 96, 1, 150]])
## conf space goals
jointPoses3 = np.array([[0.1, 343, 75, 354, 300, 0.1],
                           [7.5, 337, 80, 271, 287, 10],
                           [7.5, 313, 97, 272, 329, 10],
                           [0.1, 343, 75, 354, 300, 0.1]])

## find theoretical path
theo3_conf3 = [] ## doesnt working
theo3_conf5 = [] ## doesnt working
theo3_task = []
## set trial array1
trial3_conf5 = data[4][1]
trial3_conf3 = data[5][1]
trial3_task = data[6][1][1:]

## set N to be the number of samples
N3 = len(trial3_task)  # taken from lab_exe code
t3= np.linspace(0,Tf,int(N3/3))

## use the functions from the pre-lab
for i in range(3):
    for dt in t3:
        theo3_conf3.append(list(FK(traj_gen_config(np.deg2rad(jointPoses3[i]) , np.deg2rad(jointPoses3[i+1]) , dt , Tf )[0])[0:3,3])) # takes just the q array from the return of traj_gen_config function
        theo3_conf5.append(list(FK(traj_gen_config_5order(np.deg2rad(jointPoses3[i]) , np.deg2rad(jointPoses3[i+1]) , dt , Tf )[0])[0:3,3])) # takes just the q array from the return of traj_gen_config function
        theo3_task.append(list(traj_gen_task(x_3[i] , x_3[i+1] , dt , Tf )[0])) # takes just the x array from the return of traj_gen_task function

## convert to np.array because of plotting method  
theo3_task = np.array(theo3_task)
theo3_conf3 = np.array(theo3_conf3)
theo3_conf5 = np.array(theo3_conf5)

## auxilary function for plotting
def plot(theo_path, trial_path,title , x_goals):
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.plot(theo_path[:,0],theo_path[:,1], theo_path[:,2] , color = 'red' , linestyle='dashed')
    ax.plot(trial_path[:,0],trial_path[:,1], trial_path[:,2] , color = 'blue')
    ax.scatter(x_goals[0,0],x_goals[0,1],x_goals[0,2] , color = 'purple' , s = 30 )
    ax.scatter(x_goals[1:,0],x_goals[1:,1],x_goals[1:,2] , color = 'black' , s = 30 )
    ax.set_title(title)
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.legend(['Theoretical path' , 'Experiment path' , '1st POI','POI'])
    plt.show()

## plotting ##
#plot( theo1_task , trial1_task , 'Trial 1 - Theoretical vs Experiment - in Task Space' , x_1)
#plot( theo2_task , trial2_task , 'Trial 2 - Theoretical vs Experiment - in Task Space' , x_2)
#plot( theo3_task , trial3_task , 'Trial 3 - Theoretical vs Experiment - in Task Space' , x_3)
#plot( theo1_conf , trial1_conf , 'Trial 1 - Theoretical vs Experiment - in Conf Space' , x_1)
#plot( theo2_conf , trial2_conf , 'Trial 2 - Theoretical vs Experiment - in Conf Space' , x_2)
#plot( theo3_conf3 , trial3_conf3 , 'Trial 3 - Theoretical vs Experiment - in Conf Space' , x_3)

## slicing trail vectors to be in the same length of the theo vectors.

trial1_conf_error_calc = trial1_conf[11:]
trial2_conf_error_calc = trial2_conf[35:]
trial3_conf3_error_calc = trial3_conf3[40:91] # 123(51)
theo1_task_error = theo1_task[0:len(theo1_task),0:3]
theo2_task_error = theo2_task[0:len(theo2_task),0:3]
theo3_task_error = theo3_task[0:len(theo3_task),0:3]

## Calc the average error

traj_task1_error = np.mean(np.absolute(np.subtract(theo1_task_error,trial1_task)))
traj_task2_error = np.mean(np.absolute(np.subtract(theo2_task_error,trial2_task)))
traj_task3_error = np.mean(np.absolute(np.subtract(theo3_task_error,trial3_task)))
traj_conf1_error = np.mean(np.absolute(np.subtract(theo1_conf,trial1_conf_error_calc)))
traj_conf2_error = np.mean(np.absolute(np.subtract(theo2_conf,trial2_conf_error_calc)))
traj_conf3_error = np.mean(np.absolute(np.subtract(theo3_conf3,trial3_conf3_error_calc)))

print('traj_task1_error is')
print(traj_task1_error)
print('traj_task2_error is')
print(traj_task2_error)
print('traj_task3_error is')
print(traj_task3_error)
print('traj_conf1_error is')
print(traj_conf1_error)
print('traj_conf2_error is')
print(traj_conf2_error)
print('traj_conf3_error is')
print(traj_conf3_error)
