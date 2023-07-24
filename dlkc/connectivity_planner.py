"""
Author: Jason Hughes
Date: June 2023
About: Multi-Layer K-Connectivity planner
"""
import numpy as np
from scipy.optimize import minimize, rosen, rosen_der, LinearConstraint

class ConnectivityPlanner:

    def __init__(self, n, dim):

        self.num_agents_ = n
        
        self.dimension_ = dim

        self.locations_ = np.array((0,0))
        self.connection_vector_ = np.array((1))
        

    @property
    def num_agents(self):
        return self.num_agents_

    @num_agents.setter
    def num_agents(self, n):
        self.num_agents_ = n

    @property
    def dimension(self):
        return self.dimension_

    @dimension.setter
    def dimension(self, d):
        self.dimension_ = d

    @property
    def locations(self):
        return self.locations_

    @locations.setter
    def locations(self, l):
        assert type(l) == type(np.array([1,1]))
        assert l.shape == (self.dimension_, self.num_agents_)
        self.locations_ = l

    @property
    def connection_vector(self):
        return self.connectivity_vector_

    @connection_vector.setter
    def connection_vector(self, v):
        assert len(v) == self.num_agents_
        self.connectivity_vector_ = v
        

    def optimize(self, Bs, Bc, control):

        u0 = 0.0 * np.ones(self.dimension_*self.num_agents)
        
        func = lambda u : np.sum( np.linalg.norm(u - control)**2 )
        
        cons = ({'type': 'ineq', 'fun': Bc.connectionConstraint},
                {'type': 'ineq', 'fun': Bs.safetyConstraint})
        
        res = minimize(func, u0, method='SLSQP', constraints=cons)
        return np.reshape(res.x,(self.num_agents_,self.dimension_))
    
