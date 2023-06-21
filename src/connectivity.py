#!/usr/bin/env python3

import math
import rclpy
import random
import numpy as np
from scipy.optimize import minimize, rosen, rosen_der, LinearConstraint
from rclpy.node import Node
from kconn_sim.barrier_certificates import SafetyCertificate, ConnectionCertificate
from kconn_sim.agent import Agent
from kconn_sim.cluster import Cluster
from kconn_sim.network import K0Network, Network
from kconn_sim.plotting import *
from kconn_sim.checks import *
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
        
        self.sys_id_ = self.get_parameter("sys_id").value
        self.level_ = self.get_parameter("level").value
        self.alpha_ = self.get_parameter("alpha").value
        self.magnitude_ = self.get_parameter("magnitude").value

        self.control_vector_ = np.array((0,0))

        self.control_vec_pub_ = self.create_publsiher(Twist,
                                                      "kconn/control",
                                                      qos)
        
        self.timer_ = self.create_timer(1.0, self.cycleCallback)


    def cycleCallback(self):
        # update planner constraints
        # call the planner
