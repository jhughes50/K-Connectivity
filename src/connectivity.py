#!/usr/bin/env python3

import math
import rclpy
import random
import numpy as np
from scipy.optimize import minimize, rosen, rosen_der, LinearConstraint
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from dlkc.barrier_certificates import SafetyCertificate, ConnectionCertificate
from dlkc.agent import Agent
from dlkc.cluster import Cluster
from dlkc.network import K0Network, Network
from dlkc.plotting import *
from dlkc.checks import *
from casa_msgs.msg import CasaAgent, CasaAgentArray
from geometry_msgs.msg import Twist
from time import sleep
from sklearn.cluster import KMeans
from scipy.spatial.distance import cdist

class AgentConnectivity(Node):

    def __init__(self):
        super().__init__('AgentConnectivity')

        # set QoS standard
        qos = QoSProfile(reliability = QoSReliabilityPolicy.BEST_EFFORT,
                         durability =  QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history = QoSHistoryPolicy.KEEP_LAST,
                         depth = 10 )

        # declare and get parmeters
        self.declare_parameter("sys_id", rclpy.Parameter.Type.INTEGER)
        self.declare_parameter("level", rclpy.Parameter.Type.INTEGER)
        self.declare_parameter("alpha", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("magnitude", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("safety_radius", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("connection_radius", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("dimension", rclpy.Parameter.Type.INTEGER)
        
        self.sys_id_ = self.get_parameter("sys_id").value
        self.level_ = self.get_parameter("level").value
        self.alpha_ = self.get_parameter("alpha").value
        self.magnitude_ = self.get_parameter("magnitude").value
        self.safety_radius_ = self.get_parameter("safety_radius").value
        self.conn_radius_ = self.get_parameter("connection_radius").value
        self.dim_ = self.get_parameter("dimension").value
        
        self.control_vector_ = np.array((0,0))
        self.connection_vector_ = np.array((0,0))

        self.control_vec_pub_ = self.create_publisher(Twist,
                                                      "kconn/control",
                                                      qos)

        self.system_dict_ = dict()
        self.system_locations_ = list()

        self.system_type_0_ = dict()
        self.system_type_1_ = dict()

        self.clusters_ = list()

        self.num_agents_ = 0
        
        topic_namespace = "casa"+str(self.sys_id_)

        self.create_subscription(CasaAgentArray, topic_namespace+"/internal/system_array",
                                 self.agentArrayCallback,
                                 qos)
        
        self.timer_ = self.create_timer(1.0, self.cycleCallback)

        # these cant be called until agent array is populated
        while self.num_agents_ == 0:
            pass
        
        self.connection_certificate_ = ConnectionCertificate(self.system_locations_,
                                                               self.num_agents_,
                                                               self.dim_,
                                                               self.conn_radius_,
                                                               self.connection_vector_
                                                               ) 
        self.safety_certificate_ = SafetyCertificate(self.system_locations_,
                                                     self.num_agents_,
                                                     self.dim_,
                                                     self.safety_radius_)
        
        
    def agentArrayCallback(self, msg):
        #array msg callback
        for ag in msg.agents:
            location = np.array((ag.local_pose.x, ag.local_pose.y))
            if ag.connectivity_level == 0:
                if ag.sys_id not in list(self.system_dict_.keys()):
                    self.system_type_0_[ag.sys_id] = Agent(ag.sys_id,
                                                           ag.location,
                                                           ag.assigned_task,
                                                           ag.connectivity_level)
                else:
                    self.system_type_0_[ag.sys_id].location = location
            else:
                if ag.sys_id not in list(self.system_dict_.keys()):
                    self.system_type_1_[ag.sys_id] = Agent(ag.sys_id,
                                                           ag.location,
                                                           ag.assigned_task,
                                                           ag.connectivity_level)
                else:
                    self.system_type_1_[ag.sys_id].location = location
                    
        self.system_dict_ = self.system_type_0_ | self.system_type_1_            
        self.num_agents_ = len(self.system_dict_)
                                                               

    def updateSystemLocations(self):
        # function to update the location of all the agents in the swarm
        self.system_locations_.clear()
        for val in self.system_dict_.values():
            self.system_locations_.append(val.location)
            

    def clusterByTask(self):
        for ag in self.system_type_0_.values():
            ag.cluster = ag.task_index
            
            
    def cycleCallback(self):
        self.updateSystemLocations()
        # also need to update the locations at the barrier certificates 

        for ag in self.system_type_0_.values():
            ag.cluster = a
        
        self.clusterByTask()
        
        # TODO
        # 1. cluster by task
        # 2. initiate k0 network
        # 3. initiate the whole network
        # 4. connect the network
        # 5. save the connections
        # 6. set connections in a vector
        # 7. optimize
        # 8. interpret the output of the optimization
