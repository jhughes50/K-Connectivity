#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from kconn_sim.barrier_certificates import BarrierCertificates
from kconn_sim.multi_agent_swarm import MultiAgentSwarm
from kconn_sim.plotting import *
from time import sleep
from sklearn.cluster import KMeans


class Simulation(Node):

    def __init__(self):

        super().__init__('MultiAgentSimNode')
        
        self.num_agents = 20
        self.type0 = 15
        self.type1 = 5

        self.k0 = 2
        self.k1 = 3

        self.env = {0:np.array((-100,100,0)),
                    1: np.array((100,100,0)),
                    2: np.array((-100,-100,0))} 
        
        self.swarm = MultiAgentSwarm(self.num_agents)

        self.initialize_swarm()
        self.cycle()

        
    def initialize_swarm(self):
        self.get_logger().info('Initializing Swarm State')
        
        for i in range(self.num_agents):
            self.swarm.loctracker[i] = np.random.randint(-5,5,size=2)
        for i in range(self.type0):
            self.swarm.loctracker[i] = np.append(self.swarm.loctracker[i], np.array((0)))
        for i in range(self.type0,self.type0+self.type1):
            self.swarm.loctracker[i] = np.append(self.swarm.loctracker[i], np.array((1)))

        plot(locs = self.swarm.loctracker, tasks = self.env)
        self.get_logger().info('plotted')

    def cluster(self):
        X = np.array( list(self.swarm.loctracker.values())[0:self.type0] )

        km = KMeans(n_clusters = int(self.type0/self.type1), random_state = 0).fit(X)
    
        return km.labels_
        

    def cycle(self):

        rate = self.create_rate(1)

        while rclpy.ok():
            self.get_logger().info('Cycling')
            labels = self.cluster()

            sleep(1)
        

            
if __name__ == "__main__":
    rclpy.init()

    sim = Simulation()
