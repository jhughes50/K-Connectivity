#!/usr/bin/env python3

import math
import rclpy
import random
import numpy as np
from scipy.optimize import minimize, rosen, rosen_der, LinearConstraint
from rclpy.node import Node
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
        super().init('AgentConnectivity')

        # set QoS standard
        qos = QoSProfile(reliability = QoSReliabilityPolicy.BEST_EFFORT,
                         durability =  QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history = QoSHistoryPolicy.KEEP_LAST,
                         depth = 10 )

        # declare and get parmeters
        self.declare_parameter("sys_id", rclpy.Parameter.Type.INTEGER)
        self.declare_parameter("level", rclpy.Parameter.Type.INTEGER)
        self.declare_parameter("alpha", rclpy.Parameter.Type.INTEGER)
        self.declare_parameter("magnititude", rclpy.Parameter.Type.INTEGER)
        self.declare_parameter("safety_radius", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("connection_radius", rclpy.Parameter.Type.DOUBLE)
        
        self.sys_id_ = self.get_parameter("sys_id").value
        self.level_ = self.get_parameter("level").value
        self.alpha_ = self.get_parameter("alpha").value
        self.magnitude_ = self.get_parameter("magnitude").value
        self.safety_radius_ = self.get_parameter("safety_radius").value
        self.conn_radius_ = self.get_parameter("connection_radius").value
        
        self.control_vector_ = np.array((0,0))

        self.control_vec_pub_ = self.create_publsiher(Twist,
                                                      "kconn/control",
                                                      qos)

        self.system_dict_ = dict()
        self.system_list_ = list()
        
        topic_namespace = "casa"+str(self.sys_id_)

        self.create_subscription(CasaAgentArray, topic_namespace+"/internal/system_array",
                                 self.agentArrayCallback,
                                 qos)
        
        self.timer_ = self.create_timer(1.0, self.cycleCallback)
        
        self.safety_barrier_ = SafetyCertificate(self.safety_radius_, list(self.system_dict_.values()), self.dim, 0.1)
        self.conn_barrier_ = ConnectionCertificate(self.conn_radius), list(self.system_dict_.values()), self.dim, 0.1)

        
    def agentArrayCallback(self, msg):
        #array msg callback
        for ag in msg.agents:
            location = np.array((ag.local_pose.x, ag.local_pose.y))
            if ag.sys_id not in list(self.system_dict_.keys()):
                self.system_dict_[ag.sys_id] = Agent(ag.sys_id,
                                                     ag.location,
                                                     ag.assigned_task,
                                                     ag.connectivity_level)
            else:
                self.system_dict_[ag.sys_id].location = location 
    
    
    def cycleCallback(self):
        # update planner constraints
        # call the planner
        pass
