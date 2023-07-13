import numpy as np
import math

class Agent:

    def __init__(self, iden, location, task_pose, t_index, agent_type):

        self.sys_id_ = iden
        self.location_ = location
        self.task_pose_ = task_pose
        self.task_index_ = t_index
        self.agent_type_ = agent_type
        self.cluster_ = None
        self.connections_ = list()
        self.desired_control_ = None 
        
    def __str__(self):
        output =( "AGENT: %s\n"%self.sys_id_+
                  "location: %s\n"%self.location_+
                  "task: %s\n"%self.task_index_+
                  "type: %s\n"%self.agent_type_+
                  "cluster: %s\n"%self.cluster_+
                  "connections: %s\n"%self.connections_+
                  "control: %s\n"%self.desired_control_+
                  "\n"
                 )
        return output

    @property
    def sys_id(self):
        return self.sys_id_

    @sys_id.setter
    def sys_id(self, s):
        self.sys_id_ = s
    
    @property
    def location(self):
        return self.location_

    @location.setter
    def location(self, l):
        self.location_ = l

    @property
    def task_pose(self):
        return self.task_pose_

    @task_pose.setter
    def task_pose(self, p):
        self.task_pose_ = p
        
    @property
    def task_index(self):
        return self.task_index_

    @task_index.setter
    def task_index(self, t):
        self.task_index_ = t

    @property
    def agent_type(self):
        return self.agent_type_

    @agent_type.setter
    def agent_type(self, a):
        self.agent_type_ = a

    @property
    def cluster(self):
        return self.cluster_

    @cluster.setter
    def cluster(self, c):
        self.cluster_ = c

    @property
    def connections(self):
        return self.connections_

    @connections.setter
    def connections(self, c):
        self.connections_ = c

    @property
    def desired_control(self):
        return self.desired_control_

    @desired_control.setter
    def desired_control(self, d):
        self.desired_control_ = d
        
    def setConnections(self, conns):
        """ add the nodes this agent is connected to to a list """
        self.connections_.clear()
        for conn in conns:
            if conn[0] == self.sys_id_:
                self.connections_.append( conn[1] )
            elif conn[1] == self.sys_id_:
                self.connections_.append( conn[0] )

    def calcDesiredControl(self, magnitude: float):
        control_vector = np.zeros((1,2))[0]
        v = self.task_pose_ - self.location_
        v = v/np.linalg.norm(v)

        control_vector[0] = magnitude * v[0]
        control_vector[1] = magnitude * v[1]
        
        self.desired_control = control_vector

    def updateLocation(self, u):
        # FOR SIM ONLY
        self.location = self.location + u
        
