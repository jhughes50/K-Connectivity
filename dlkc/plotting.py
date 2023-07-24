import matplotlib.pyplot as plt
import numpy as np

def plot3D(locs, tasks, i, conns, swarm):

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    loc_x,loc_y,loc_z = [],[],[]
    
    for ag in swarm:
        if ag.agent_type == 0:
            loc_x.append(ag.location[0])
            loc_y.append(ag.location[1])
            loc_z.append(0)
        else:
            loc_x.append(ag.location[0])
            loc_y.append(ag.location[1])
            loc_z.append(1)

    tsk_x,tsk_y,tsk_z = [],[],[]

    for t in tasks:
        tsk_x.append(t[0])
        tsk_y.append(t[1])
        tsk_z.append(0)

    for c in conns:
        con_x, con_y, con_z = [], [], []
        con_x.append( swarm[c[0]].location[0] )
        con_y.append( swarm[c[0]].location[1] )
        con_x.append( swarm[c[1]].location[0] )
        con_y.append( swarm[c[1]].location[1] )
        if swarm[c[0]].agent_type == 0:
            con_z.append(0)
        else:
            con_z.append(1)
        if swarm[c[1]].agent_type == 0:
            con_z.append(0)
        else:
            con_z.append(1)
        ax.plot(con_x, con_y, con_z, 'ko',linestyle="--")

    ax.plot(tsk_x,tsk_y,tsk_z,'ks',markerfacecolor='none',ms=15,markeredgecolor='black',zorder=4)
    ax.plot(loc_x, loc_y, loc_z, 'bo', linestyle='None')
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_zticks([])
    plt.savefig('swarm3D_t_'+str(i)+'.png')

    plt.clf()
        

        
def plot2D(locs, tasks, i, conns, swarm):
    loc_x,loc_y,loc_z = [],[],[]
    
    for loc in locs:
        loc_x.append(loc[0])
        loc_y.append(loc[1])


    tsk_x,tsk_y,tsk_z = [],[],[]

    for t in tasks:
        tsk_x.append(t[0])
        tsk_y.append(t[1])

    con_x, con_y = [], []

    for c in conns:
        con_x, con_y = [], []
        con_x.append( swarm[c[0]].location[0] )
        con_y.append( swarm[c[0]].location[1] )
        con_x.append( swarm[c[1]].location[0] )
        con_y.append( swarm[c[1]].location[1] )
        plt.plot(con_x,con_y,'ko',linestyle="--",zorder=1)

    t2_x, t2_y = [], []
    t1_x, t1_y = [], []
    for ag in swarm:
        if ag.agent_type == 1:
            t2_x.append(ag.location[0])
            t2_y.append(ag.location[1])
        else:
            t1_x.append(ag.location[0])
            t1_y.append(ag.location[1])
        
    #plt.scatter(loc_x,loc_y)
    plt.plot(tsk_x,tsk_y,'ks',markerfacecolor='none',ms=35,markeredgecolor='black',zorder=4)
    plt.scatter(t1_x,t1_y, c='b',zorder=2)
    plt.scatter(t2_x,t2_y, c='r',zorder=3)
    plt.xlim([0,200])
    plt.ylim([-25,325])
    #plt.show()
    plt.savefig('images1/swarm2D_t_'+str(i)+'.png')
    plt.clf()
