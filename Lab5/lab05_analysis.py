import numpy as np
import matplotlib.pyplot as plt
import pickle
import glob
import math
import pandas as pd



#data_path = set your own path to data dir
section1_data_path = 'F:/DATA/robotics_lab/RoboticsAndSystemsControl_Lab/Lab5/data_lab5/section1_data'
section1_data = []
for file in glob.glob(section1_data_path + '/*/*.pkl'):
    with open(file, 'rb') as h:
        section1_data.append(pickle.load(h))

section2_data_path = 'F:/DATA/robotics_lab/RoboticsAndSystemsControl_Lab/Lab5/data_lab5/section2_data'
section2_data = []
for file in glob.glob(section2_data_path + '/*/*.pkl'):
    with open(file, 'rb') as h:
        section2_data.append(pickle.load(h))


# calculating rms
def rms(x):
    return np.sqrt(np.mean(x**2))

def plot_data(data,trials,title):
    plt.figure(figsize=(10, 5))
    for trial in range(len(trials)):
        data_dict = {'RMSE':[],'Iteration':[]}
        n = 0
        for i in range(len(data[trial][1])):
            rmse = rms(data[trial][1][i])
            data_dict['RMSE'].append(rmse)
            data_dict['Iteration'].append(n)
            n = n+1
        data_df = pd.DataFrame.from_dict(data_dict)
        plt.plot(data_df['Iteration'], data_df['RMSE'])
    plt.title(title)
    plt.xlabel('Iterations Number')
    plt.ylabel('RMSE')
    plt.grid()
    plt.legend(trials,loc=1)
    plt.show()

trials1 = ['$\lambda$ = 0.5','$\lambda$ = 0.4','$\lambda$ = 0.2','$\lambda$ = 0.6','$\lambda$ = 0.8']
trials2 = ['Path A','Path B','Path C','Path D','Path E']
plot_data(section1_data,trials1,'Static Aruco pose control for different $\lambda$')
plot_data(section2_data,trials2,'Dynamic Aruco pose control for $\lambda$ = 0.6')
