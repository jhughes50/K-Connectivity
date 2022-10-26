import numpy as np

class BarrierCertificates:

    def __init__(self):
        pass

    def dist(self, x, y):
        return np.linalg.norm(x - y)**2
    
       
        
class ConnectionCertificate(BarrierCertificates):

    def __init__(self, r, s, g=1):

        super().__init__()
        self.radius = r
        self.swarm = s
        self.n = len(s)
        self.A = np.zeros((self.n,self.n,3))
        self.gamma = g

    def construct_A(self, connections):
        for conn in connections:
            i, j = conn[0], conn[1]
            self.A[i][i] = -2 * (self.swarm[i].location -
                                 self.swarm[j].location) * self.swarm[i].desired_control 
            self.A[i][j] = 2 * (self.swarm[i].location -
                                self.swarm[j].location) * self.swarm[i].desired_control

    def get_b(self, i, j):
        return self.gamma * (self.radius**2 -
                             self.dist(self.swarm[i].location, self.swarm[j].location))


    
class SafetyCertificate(BarrierCertificates):

    def __init__(self, r, s, g=1):

        super().__init__()
        self.radius = r
        self.swarm = s
        self.n = len(s)
        self.A = np.zeros((self.n,self.n,3))
        self.gamma = g
        
    def construct_A(self):
    
        for i in range(self.n-1):
            for j in range(i+1,self.n):
                self.A[i][i] = -2 * (self.swarm[i].location -
                                     self.swarm[j].location) * self.swarm[i].desired_control
                self.A[i][j] = 2 * (self.swarm[i].location -
                                    self.swarm[j].location) * self.swarm[j].desired_control
        self.A = np.reshape(self.A.flatten(),(self.n,3*self.n))
        print(self.A.shape)
                
    def get_b(self, i, j):
        return self.gamma * (self.dist(self.swarm[i].location, self.swarm[j].location) - self.radius**2)

    def s_ij(self, loc_i, loc_j):
        """ dep """
        return self.radius**2 - np.linalg.norm(loc_i-loc_j)**2
