import numpy as np
import itertools

class Network():

    def __init__(self, clusters, swarm, k0net, k0, k1, type_sep, num_type1):

        self.clusters = clusters
        self.swarm = swarm
        self.k0net = k0net
        
        self.k0 = k0
        self.k1 = k1

        self.type_sep = type_sep
        self.n_type1 = num_type1
        
        self.connections = k0net.connections
        
    def connect(self):
        n = len(self.swarm)
        k_iter = 1
        
        while k_iter <= (self.k1 - self.k0):
            for ag1 in self.swarm[self.type_sep:]:
                for ag2 in self.swarm[self.type_sep:]:
                    if ag1._id != ag2._id:
                        if ( abs(ag1._id - ag2._id) ) % self.n_type1 == k_iter and ag1._id > ag2._id:
                            self.connections.append((ag1._id, ag2._id))
                            break
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
        
        while iter <= self.k:
            for i in range(self.n_nodes - 1): 
                for j in range(i+1,self.n_nodes): 
                    if abs( i - j ) % self.n_nodes == iter: 
                        self.connections.append((nodes[i],nodes[j]))
            
            iter += 1
        # connect the last node to the first one
        self.connections.append((nodes[0],nodes[self.n_nodes-1]))

