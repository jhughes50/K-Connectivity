"""
Author: Jason Hughes
Date: June 2023
About: Multi-Layer K-Connectivity planner
"""

class ConnectivityPlanner:

    def __init__(self):

        self.num_agents_ = 0
        self.type0_ = 0
        self.type1_ = 0

        self.connectivity_radius_ = 0
        self.safety_radius_ = 0

        self.magnitude_ = 1
        
        self.dimension_ = 2

        self.locations_ = np.array((0,0))
        self.connectivity_vector = np.array((1))
        

    @property
    def num_agents(self):
        return self.num_agents_

    @num_agents.setter
    def num_agents(self, n):
        self.num_agents_ = n

    @property
    def type0(self):
        return self.type0_

    @type0.setter
    def type0(self, n):
        self.type0_ = n

    @property
    def type1(self):
        return self.type1_


    @type1.setter
    def type1(self, n):
        self.type1_ = n

    @property
    def connectivity_radius(self):
        return self.connectivity_radius_

    @connectivity_radius.setter
    def connectivity_radius(self, r):
        self.connectivity_radius_ = r

    @property
    def safety_radius(self):
        return self.safety_radius_

    @safety_radius.setter
    def safety_radius(self, r):
        self.safety_radius_ = r

    @property
    def magnitude(self):
        return self.magnitude_

    @magnitude.setter
    def magnitude(self, m):
        self.magnitude_ = m

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
        assert type(l) == type(np.array(1,1))
        assert l.shape == (self.num_agents_, self.dimension_)
        self.locations_ = l

    @property
    def connectivity_vector(self):
        return self.connectivity_vector_

    @connectivity_vector.setter
    def connectivity_vector(self, v):
        assert len(v) == self.num_agents_
        self.connectivity_vector_ = v
        

    def optimize(self, Bs, Bc, control):

        u0 = 0.0 * np.ones(self.dim*self.num_agents)
        
        func = lambda u : np.sum( np.linalg.norm(u - control)**2 )
        
        cons = ({'type': 'ineq', 'fun': Bc.connectivityConstraint},
                {'type': 'ineq', 'fun': Bs.safetyConstraint})
        
        res = minimize(func, u0, method='SLSQP', constraints=cons)
        return np.reshape(res.x,(self.num_agents,self.dim))
    
