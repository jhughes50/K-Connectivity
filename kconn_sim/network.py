import numpy as np
import itertools

class Network():

    def __init__(self, clusters, swarm, k0net, k0, k1):

        self.clusters = clusters
        self.swarm = swarm
        self.k0net = k0net
        
        self.k0 = k0
        self.k1 = k1

        self.connections = k0net.connections
        
    def connect(self):
        n = len(self.swarm)
        k_iter = 1
        
        while k_iter < (self.k1 - self.k0):
            for ag1 in self.swarm:
                for ag2 in self.swarm:
                    if (ag1.agent_type == 1) and (ag2.agent_type == 1) and (ag1._id != ag2._id):
                        if abs(ag1._id-ag2._id) % n == k_iter:
                            print('connecting %s to %s' %(ag1._id,ag2._id))
                            self.connections.append((ag1._id, ag2._id))
            k_iter += 1

class K0Network():

    def __init__(self, clusters, k0):

        self.clusters = clusters

        self.connections = list()
        self.nodes = list()

        self.k = k0
        
        for cl in self.clusters:
            self.nodes.append( cl.members )

        self.nodes = list(itertools.chain(*self.nodes))

        self.n_nodes = len(self.nodes)

        self.construct(self.nodes)
        
    def construct(self, nodes):
        iter = 1
        while iter < self.k:
            for i in range(self.n_nodes - 1): 
                for j in range(i,self.n_nodes): 
                    if abs( i - j ) % self.n_nodes == iter: 
                        self.connections.append((nodes[i],nodes[j]))
            iter += 1


