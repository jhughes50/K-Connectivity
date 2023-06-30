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
    
    def set_members(self, swarm, sep):
        self.members = list()
        self.iter = 0
        
        for ag in swarm[:sep]:
            if ag.cluster == self.cluster_id:
                self.members.append( ag._id )
                self.index_type0.append(self.iter)
                self.iter += 1

    def add_members(self, swarm):
        """ Add layer2 members to the cluster """
        # insert the L2 member in the middle of the cluster
        insert_ind = int(len(swarm) / 2)
        for ag in swarm:
            if ag.cluster == self.cluster_id:
                self.members.insert( insert_ind, ag._id )
                self.index_type1.insert( insert_ind, self.iter)
                self.iter += 1

    def set_centroid(self, center):
        self.centroid = center














