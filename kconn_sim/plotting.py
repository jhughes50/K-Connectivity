import matplotlib.pyplot as plt
import numpy as np

def plot3D(locs, tasks, i, conns, swarm):

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    loc_x,loc_y,loc_z = [],[],[]
    
    for loc in locs:
        loc_x.append(loc[0])
        loc_y.append(loc[1])
        loc_z.append(loc[2])

    tsk_x,tsk_y,tsk_z = [],[],[]

    for t in tasks.values():
        tsk_x.append(t[0])
        tsk_y.append(t[1])
        tsk_z.append(t[2])

    for c in conns:
        con_x, con_y, con_z = [], [], []
        con_x.append( swarm[c[0]].location[0] )
        con_y.append( swarm[c[0]].location[1] )
        con_z.append( swarm[c[0]].location[2] )
        con_x.append( swarm[c[1]].location[0] )
        con_y.append( swarm[c[1]].location[1] )
        con_z.append( swarm[c[1]].location[2] )
        ax.plot(con_x,con_y,con_z,'bo',linestyle="--")
    
    #ax.scatter(loc_x,loc_y,loc_z)
    ax.scatter(tsk_x,tsk_y,tsk_z,'ro')
    plt.savefig('swarm3D_t_'+str(i)+'.png')

    plt.clf()
    #plt.show()

def plot2D(locs, tasks, i, conns, swarm):
    loc_x,loc_y,loc_z = [],[],[]
    
    for loc in locs:
        loc_x.append(loc[0])
        loc_y.append(loc[1])


    tsk_x,tsk_y,tsk_z = [],[],[]

    for t in tasks.values():
        tsk_x.append(t[0])
        tsk_y.append(t[1])

    con_x, con_y = [], []

    for c in conns:
        con_x, con_y = [], []
        con_x.append( swarm[c[0]].location[0] )
        con_y.append( swarm[c[0]].location[1] )
        con_x.append( swarm[c[1]].location[0] )
        con_y.append( swarm[c[1]].location[1] )
        plt.plot(con_x,con_y,'bo',linestyle="--")
        
    #plt.scatter(loc_x,loc_y)
    plt.scatter(tsk_x,tsk_y)
    #plt.show()
    plt.savefig('swarm2D_t_'+str(i)+'.png')
    plt.clf()
