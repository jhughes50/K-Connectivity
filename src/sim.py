#!/usr/bin/env python3

import math
import rclpy
import random
import numpy as np
from scipy.optimize import minimize, rosen, rosen_der, LinearConstraint
from rclpy.node import Node
from kconn_sim.barrier_certificates import SafetyCertificate, ConnectionCertificate
from kconn_sim.agent import Agent
from kconn_sim.cluster import Cluster
from kconn_sim.network import K0Network, Network
from kconn_sim.plotting import *
from kconn_sim.checks import *
from time import sleep
from sklearn.cluster import KMeans
from scipy.spatial.distance import cdist

class Simulation(Node):

    def __init__(self):

        super().__init__('MultiAgentSim')

        self.type0 = 25
        self.type1 = 7
        self.num_agents = self.type0 + self.type1
        self.r_conn = 50
        self.r_safety = 5
        self.dim = 2
        
        assert self.num_agents == (self.type0 + self.type1)

        self.k0 = 2
        self.k1 = 3

        self.alpha = 1

        self.control_magnitude = 1 #move at 1 m/s
        
        self.env = np.array([[10,200],[175,10],[75,300],[25,20]])

        self.n_clust = len(self.env)
        
        self.locations = np.array((1,1)) 
        self.swarm_control = None
        self.conn_vec = None
        
        self.swarm = list()
        self.clusters = list()

        self.__init_swarm__()
        self.cycle()
        
        
    def __init_swarm__(self):
        # add layer1 agents to swarm
        for i in range(self.type0):
            env_i = random.randint(0,len(self.env)-1)
            self.swarm.append( Agent(i,
                                     1.0*np.random.randint(0,5,size=2),
                                     self.env[env_i],
                                     env_i,
                                     0) )
        # add layer 2 agents to swarm 
        for i in range(self.type1):
            env_i = random.randint(0,len(self.env)-1)
            self.swarm.append( Agent(i+self.type0,
                                     1.0*np.random.randint(0,5,size=2),
                                     self.env[env_i],
                                     env_i,
                                     1) )
        # initialize the clusters 
        for i in range(self.n_clust):
            self.clusters.append( Cluster(i) ) 
            
        self.get_logger().info('Swarm Initialized')


    def locs_to_numpy(self):
        locs = list()
        for ag in self.swarm:
            locs.append(ag.location)

        self.locations =  np.array(locs)

        
    def cluster(self):

        locs = list()

        for ag in self.swarm:
            locs.append(ag.location)
        X = np.array( locs[0:self.type0] )

        km = KMeans(n_clusters = self.n_clust, random_state = 0).fit(X)

        return km.labels_, km.cluster_centers_

    
    def matching(self, centroids):
        # TODO: make this resistant if type0/type1 is not int -- DONE

        W = cdist(self.locations[self.type0:],centroids, 'euclidean')
        
        for iter, ag in enumerate(self.swarm[self.type0:]):
            ag.cluster = ag.task_index # np.argmin(W[iter])
            W[:,ag.cluster] = np.inf
        
        for cluster in self.clusters:
            cluster.add_members( self.swarm[self.type0:] )

            
    def calc_control_vector(self):
        control_vector = np.zeros((1,3))
        for ag in self.swarm:
            theta = math.atan2(ag.location[1],ag.location[0]) - math.atan2(self.env[ag.task][1], self.env[ag.task][0])
            control_vectorq[0] = self.magnitude * math.cos(theta)
            control_vector[1] = self.magnitude * math.sin(theta)

            ag.set_desired_control(control_vector)

            
    def set_swarm_control(self):
        for ag in self.swarm:
            u = ag.desired_control
            if ag._id == 0:
                self.swarm_control = u
            else:
                self.swarm_control = np.concatenate((self.swarm_control,u), axis=None)

                
    def conn_matrix(self):
        l = list()
        for i in range(self.num_agents-1):
            for j in range(i+1, self.num_agents):
                if j in self.swarm[i].connections:
                    l.append(1)
                else:
                    l.append(0)
        self.conn_vec = np.array(l)

        
    def connection_constraint(self, u):
        # u is a flattened n-by-2 array, thus 2n-by-1
        res = list()
        u = np.reshape(u,(self.num_agents, self.dim))

        for i in range(self.num_agents-1):
            for j in range(i+1,self.num_agents):
                d = math.dist(self.locations[i] + u[i],
                          self.locations[j] + u[j])
                res.append(self.r_conn - d)
        return np.array(res)*self.conn_vec

    
    def safety_constraint(self, u):
        res = list()
        u = np.reshape(u,(self.num_agents,self.dim))
        for i in range(self.num_agents-1):
            for j in range(i+1, self.num_agents):
                d = math.dist(self.locations[i]+u[i], self.locations[j]+u[j])
                res.append(d - self.r_safety)
        return np.array(res)
    
                
    def optimize(self, Bs, Bc):

        u0 = 0.0 * np.ones(self.dim*self.num_agents)
        
        func = lambda u : np.sum( np.linalg.norm(u - self.swarm_control)**2 )
        
        cons = ({'type': 'ineq', 'fun': self.connection_constraint},
                {'type': 'ineq', 'fun': self.safety_constraint})
        
        res = minimize(func, u0, method='SLSQP', constraints=cons)
        return np.reshape(res.x,(self.num_agents,self.dim))

    
    def cycle(self):

        Bs = SafetyCertificate(self.r_safety, self.swarm, self.dim, 0.1)
        Bc = ConnectionCertificate(self.r_conn, self.swarm, self.dim, 0.1)

        cycle_iter = 0
        
        while rclpy.ok():

            labels, centroids = self.cluster()

            for iter, ag in enumerate( self.swarm[:self.type0] ):
                ag.cluster = ag.task_index #labels[iter]

            for cluster in self.clusters:
                cluster.set_members( self.swarm,self.type0 )
                cluster.set_centroid( centroids[cluster.cluster_id] )
                
            self.locs_to_numpy()

            self.matching(centroids)
            
            k0net = K0Network(self.clusters, self.k0)

            net = Network(self.clusters, self.swarm, k0net, self.k0, self.k1, self.type0, self.type1)
            net.connect()
            
            for ag in self.swarm:
                ag.set_connections(net.connections)
                ag.set_desired_control(float(self.control_magnitude),self.env)
            
            self.set_swarm_control()
            self.conn_matrix()

            u_star = self.optimize(Bs,Bc)
        
            for ag in self.swarm:
                ag.update_location(u_star[ag._id])
                #check_destination(ag, self.env)
                check_vector(ag)
                print(ag)

            if cycle_iter == 1000:
                break
            else:
                cycle_iter += 1
                if cycle_iter%10 == 0:
                    plot3D(self.locations,self.env, cycle_iter,net.connections, self.swarm)
                    plot2D(self.locations,self.env, cycle_iter,net.connections, self.swarm)
                sleep(0.1)
        for cl in self.clusters:
            print("task %s: "%(str(cl.cluster_id)), len(cl))
        print("Num Connections: ",len(net.connections))
            
if __name__ == "__main__":
    rclpy.init()

    Simulation()
