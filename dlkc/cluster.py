import numpy as np


class Cluster:

    def __init__(self, iden):

        self.cluster_id = iden
        self.members = list()
        self.centroid = None

        self.index_type0 = list()
        self.index_type1 = list()

        self.iter = 0
        
    def __str__(self):
        return "Members: %s\nCentroid: %s" %(self.members, self.centroid)

    def __len__(self):
        """ return the number of nodes in the cluster """
        return len(self.members)

    @property
    def members(self):
        return self.members
    
    def setMembers(self, swarm):
        self.members = list()
        self.iter = 0
        
        for ag in swarm:
            if ag.cluster == self.cluster_id:
                self.members.append( ag._id )
                self.index_type0.append(self.iter)
                self.iter += 1

    def addMembers(self, swarm):
        """ Add layer2 members to the cluster """
        # insert the L2 member in the middle of the cluster
        insert_ind = int(len(swarm) / 2)
        for ag in swarm:
            if ag.cluster == self.cluster_id:
                self.members.insert( insert_ind, ag._id )
                self.index_type1.insert( insert_ind, self.iter)
                self.iter += 1

    def calcCentroid(self, system):
        x, y = list(), list()
        location_dict = dict()

        for member in members:
            loc = system[member].location
            x.append(loc[0])
            y.append(loc[1])
            location_dict[member] = loc
        mid = (sum(x)/len(x),sum(y)/len(y))
        mid = np.array(mid).reshape((1,2))
        locs = np.array(location_dict.values())
        d = cdist(mid, locs, 'euclidean')
        a = np.argmin(d)
        label = list(location_dict.keys())[a]
        self.centroid = list(location_dict.values())[a]

        return self.centroid, label
            
    def setCentroid(self, center):
        self.centroid = center














