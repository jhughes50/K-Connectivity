import numpy as np
import math


class BarrierCertificate:
    """ 
    parent class to both the safety and connectivity barrier certificates
    """
    def __init__(self, locs, n, dim):
        self.locs_ = locs
        self.n_ = n
        self.dim_ = dim

        
    def getResidual(self, u):
        res = list()
        u = np.reshape(u,(self.n_,self.dim_))
    
        for i in range(self.n_-1):
            for j in range(i+1, self.n_):
                d = math.dist(self.locs_[i]+u[i], self.locs_[j]+u[j])
                res.append(d)
                
        return np.array(res)

    @property
    def locs(self):
        return self.locs_
    
    @locs.setter
    def locs(self, l):
        self.locs_ = l

    @property
    def num_agents(self):
        return self.n_

    @num_agents.setter
    def num_agents(self, n):
        self.n_ = n

        
class ConnectionCertificate(BarrierCertificate):

    def __init__(self, locs, n, dim, radius, vec):

        super().__init__(locs, n, dim)
        self.radius_ = radius
        self.dim_ = dim
        self.conn_vec_ = vec
        
        
    def connectionConstraint(self, u):
        res = self.getResidual(u)
        res = self.radius_ - res
        return res*self.conn_vec_
                                 
    
class SafetyCertificate(BarrierCertificate):

    def __init__(self, locs, n, dim, radius):

        super().__init__(locs, n, dim)
        self.radius_ = radius
        self.dim_ = dim


    def safetyConstraint(self, u):
        res = self.getResidual(u)
        return res - self.radius_
        
        
