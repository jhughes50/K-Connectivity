import numpy as np

class BarrierCertificates:

    def __init__(self):
        pass

    def dist(self, x, y):
        return np.linalg.norm(x - y)**2

    def construct_A(self, swarm, n):
        A = np.zeros((n,n))
        for i in range(n-1):
            for j in range(i,n):
                A[i][i] = -2 * (swarm[i].location - swarm[j].location) * swarm[i].desired_control
                A[i][j] = 2 * (swarm[i].location - swarm[j].location) * swarm[j].desired_control
        return A
        
class ConnectionCertificate(BarrierCertificates):

    def __init__(self, r, s, g=1):

        super().__init__()
        self.radius = r
        self.swarm = s
        self.n = len(s)
        self.A = np.zeros((self.n,self.n))
        self.gamma = g

    def update_A(self):
        self.A = self.construct_A(self.swarm, self.n)

    def get_b(self, i, j):
        return self.gamm * self.dist(self.swarm[i].location, self.swarm[j].location) + self.radius**2)

class SafetyCertificate(BarrierCertificates):

    def __init__(self, r, s, g=1):

        super().__init__()
        self.radius = r
        self.swarm = s
        self.n = len(s)
        self.A = np.zeros((self.n,self.n))
        self.gamma = g
        
    def update_A(self):
        self.A = self.construct_A(self.swarm, self.n)
            
    def get_b(self, i, j):
        return self.gamma * (self.dist(self.swarm[i].location, self.swarm[j].location) - self.radius**2)
