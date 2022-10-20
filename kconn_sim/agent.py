import numpy as np

class Agent:

    def __init__(self, iden, location, task, agent_type):

        self._id = iden
        self.location = location
        self.task = task
        self.agent_type = agent_type
        self.cluster = None
        self.connections = list()

    def __str__(self):
        output =( "AGENT: %s\n"%self._id+
                  "location: %s\n"%self.location+
                  "task: %s\n"%self.task+
                  "type: %s\n"%self.agent_type+
                  "cluster: %s\n"%self.cluster+
                  "connections: %s\n"%self.connections+
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
                
        
