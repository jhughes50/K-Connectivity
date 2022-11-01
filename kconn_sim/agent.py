import numpy as np
import math

class Agent:

    def __init__(self, iden, location, task, agent_type):

        self._id = iden
        self.location = location
        self.task = task
        self.agent_type = agent_type
        self.cluster = None
        self.connections = list()
        self.desired_control = None 
        
    def __str__(self):
        output =( "AGENT: %s\n"%self._id+
                  "location: %s\n"%self.location+
                  "task: %s\n"%self.task+
                  "type: %s\n"%self.agent_type+
                  "cluster: %s\n"%self.cluster+
                  "connections: %s\n"%self.connections+
                  "control: %s\n"%self.desired_control+
                  "\n"
                 )
        return output
    
    def set_connections(self, conns):
        """ add the nodes this agent is connected to to a list """
        self.connections.clear()
        for conn in conns:
            if conn[0] == self._id:
                self.connections.append( conn[1] )
            elif conn[1] == self._id:
                self.connections.append( conn[0] )

    def set_desired_control(self, magnitude: float):
        control_vector = np.zeros((1,2))[0]
        v = self.task - self.location
        v = v/np.linalg.norm(v)

        control_vector[0] = magnitude * v[0]
        control_vector[1] = magnitude * v[1]
        
        self.desired_control = control_vector

    def update_location(self, u):
        #norm_u = u/np.sum(u)
        #print('u: ',u)
        #print('norm: ',norm_u)
        self.location = self.location + u
        #print(self.location)
