import matplotlib.pyplot as plt
import numpy as np

def plot(locs = {}, tasks = {}):

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    
    if len(locs) != 0:
        loc_x, loc_y, loc_z = [], [], []
        
        for key, val in locs.items():
            loc_x.append(val[0])
            loc_y.append(val[1])
            loc_z.append(val[2])
            
        ax.scatter(loc_x, loc_y, loc_z, cmap = None)
            
    if len(tasks) != 0:
        task_x, task_y, task_z = [], [], []

        for key, val in tasks.items():
            task_x.append(val[0])
            task_y.append(val[1])
            task_z.append(val[2])
        ax.scatter(task_x, task_y, task_z, cmap = None)
            
    #print('plotting')
    plt.show()
    #print('done plotting')
