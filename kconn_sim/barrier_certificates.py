import numpy as np

class BarrierCertificates:

    def __init__(self):
        pass

    def dist(self, x, y):
        return np.linalg.norm(x - y)**2
    
       
        
class ConnectionCertificate(BarrierCertificates):

    def __init__(self, r, s, dim, g=1):

        super().__init__()
        self.radius = r
        self.swarm = s
        self.n = len(s)
        self.A = np.zeros((self.n,self.n,3))
        self.b = None
        self.dim = dim
        self.gamma = g
        

    def construct_A(self):

        A = list()
        for ag in self.swarm[:-1]:
            for j in range(ag._id+1, self.n):
                A_ij = np.zeros((self.n,self.dim))
                if j in ag.connections:
                    #print(2 * (self.swarm[ag._id].location - self.swarm[j].location), ' row %s'%j)
                    A_ij[ag._id] = -2 * (self.swarm[ag._id].location -
                                         self.swarm[j].location) 
                    A_ij[j] = 2 * (self.swarm[ag._id].location -
                                      self.swarm[j].location) 
                A.append(A_ij.flatten())
        self.A = np.array(A)

            
    def construct_b(self):
        b_ij = list()
        for ag in self.swarm:
            for j in range(ag._id+1, self.n):
                if j in ag.connections:
                    b_ij.append( self.get_b(ag._id,j) )
                else:
                    b_ij.append( 0. )
        self.b = np.array(b_ij)
        
    def get_b(self, i, j):
        return self.gamma * (self.radius**2 -
                             self.dist(self.swarm[i].location, self.swarm[j].location))


    
class SafetyCertificate(BarrierCertificates):

    def __init__(self, r, s, dim, g=1):

        super().__init__()
        self.radius = r
        self.swarm = s
        self.n = len(s)
        self.A = np.zeros((self.n,self.n,3))
        self.gamma = g
        self.dim = dim
        self.b = None
        
    def construct_A(self):
        A = list()
        for i in range(self.n-1):
            for j in range(i+1,self.n):
                A_ij = np.zeros((self.n,self.dim))
                A_ij[i] = -2 * (self.swarm[i].location -
                                     self.swarm[j].location) * self.swarm[i].desired_control
                A_ij[j] = 2 * (self.swarm[i].location -
                                    self.swarm[j].location) * self.swarm[j].desired_control
                #print(A_ij.flatten())
                A.append(A_ij.flatten())

        self.A = np.array(A)
                
    def get_b(self, i, j):
        return self.gamma * (self.dist(self.swarm[i].location, self.swarm[j].location) - self.radius**2)

    def construct_b(self):
        b_ij = list()
        for i in range(self.n-1):
            for j in range(i+1, self.n):
                b_ij.append(self.get_b(i,j))
        self.b = np.array(b_ij)
        
        
        
