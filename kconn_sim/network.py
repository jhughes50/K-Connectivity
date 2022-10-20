import numpy as np
import itertools

class ClusterNetwork():
    """ depricated """
    def __init__(self, cluster, k0, r_conn):

        self.cluster = cluster
        self.n_nodes = len(cluster)
        self.k = k0
        self.connections = list()

    def construct(self, nodes):
        iter = 1
        while iter <= ((self.k+1)*self.n_nodes)/2:
            for i in range(self.n_nodes - 1):
                for j in range(i,self.n_nodes):
                    if abs( nodes[i]-nodes[j] ) % self.n_nodes == iter:
                        self.connections.append((nodes[i],nodes[j]))
            iter += 1 

class K1Network():

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
        while iter <= int(((self.k+1)*self.n_nodes)/2):
            for i in range(self.n_nodes - 1): 
                for j in range(i,self.n_nodes): 
                    if abs( nodes[i]-nodes[j] ) % self.n_nodes == iter: 
                        self.connections.append((nodes[i],nodes[j]))
            iter += 1


