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
from dlkc.connectivity_planner import ConnectivityPlanner
from dlkc.cluster import Cluster
from dlkc.network import K0Network, Network
from dlkc.plotting import *
from dlkc.checks import *
from casa_msgs.msg import CasaAgent, CasaAgentArray
from geometry_msgs.msg import TwistStamped, PoseStamped
from time import sleep
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
        self.declare_parameter("level_0", rclpy.Parameter.Type.INTEGER)
        self.declare_parameter("level_1", rclpy.Parameter.Type.INTEGER)
        self.declare_parameter("task", rclpy.Parameter.Type.INTEGER)
        
        self.sys_id_ = self.get_parameter("sys_id").value
        self.my_level_ = self.get_parameter("level").value
        self.alpha_ = self.get_parameter("alpha").value
        self.magnitude_ = self.get_parameter("magnitude").value
        self.safety_radius_ = self.get_parameter("safety_radius").value
        self.conn_radius_ = self.get_parameter("connection_radius").value
        self.dim_ = self.get_parameter("dimension").value
        self.level_0_connectivity_ = self.get_parameter("level_0").value
        self.level_1_connectivity_ = self.get_parameter("level_1").value
        self.my_task_ = self.get_parameter("task").value
        
        self.tasks_ = {0:[100,100], 1:[200,200]}
        
        self.control_vector_ = np.array((0,0))
        self.connection_vector_ = np.array((0,0))
        self.desired_control_ = np.array((0,0))
        
        self.control_vec_pub_ = self.create_publisher(TwistStamped,
                                                      "casa"+str(self.sys_id_)+"/internal/goto_vel",
                                                      qos)

        self.received_ = False
        self.received_pose_ = False
        
        self.my_pose_ = np.array([])
        self.my_index_ = 0
        
        self.system_dict_ = dict()
        self.system_locations_ = list()
        
        self.system_type_0_ = dict()
        self.system_type_1_ = dict()
        self.system_type_0_locations_ = list()
        self.system_type_1_locations_ = list()

        self.clusters_ = list()
        self.centroids_ = dict()
        
        self.num_agents_ = 0
        self.num_clusters_ = len(self.tasks_)
        
        topic_namespace = "casa"+str(self.sys_id_)

        self.create_subscription(CasaAgentArray, topic_namespace+"/internal/system_array",
                                 self.agentArrayCallback,
                                 qos)
        self.create_subscription(PoseStamped, topic_namespace+"/internal/local_position",
                                 self.localCallback, qos)
                                 
        self.timer_ = self.create_timer(1.0, self.cycleCallback)
        
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
        self.planner_ = ConnectivityPlanner(self.num_agents_,
                                            self.dim_)
        
        for i in range(self.num_clusters_):
            self.clusters_.append(Cluster(i))

        if self.my_level_ == 0:
            self.system_type_0_[self.sys_id_] = Agent(self.sys_id_,
                                                      self.my_pose_,
                                                      self.tasks_[self.my_task_],
                                                      self.my_task_,
                                                      self.my_level_)
        elif self.my_level_ == 1:
            self.system_type_1_[self.sys_id_] = Agent(self.sys_id_,
                                                      self.my_pose_,
                                                      self.tasks_[self.my_task_],
                                                      self.my_task_,
                                                      self.my_level_)
        else:
            self.get_logger().error("Input level was incorrect, must be 0 or 1")


    def localCallback(self, msg):
        self.received_pose_ = True
        self.my_pose_ = np.array([msg.pose.position.x, msg.pose.position.y])

        if self.my_level_ == 0:
            self.system_type_0_[self.sys_id_].location = self.my_pose_
        else:
            self.system_type_1_[self.sys_id_].location = self.my_pose_
        
    def agentArrayCallback(self, msg):
        #array msg callback
        self.received_ = True
        for ag in msg.agents:
            location = np.array((ag.local_pose.x, ag.local_pose.y))
            if ag.connectivity_level == 0:
                if ag.sys_id not in list(self.system_dict_.keys()):
                    self.system_type_0_[ag.sys_id] = Agent(ag.sys_id,
                                                           location,
                                                           self.tasks_[ag.assigned_task],
                                                           ag.assigned_task,
                                                           ag.connectivity_level)
                else:
                    self.system_type_0_[ag.sys_id].location = location
            else:
                if ag.sys_id not in list(self.system_dict_.keys()):
                    self.system_type_1_[ag.sys_id] = Agent(ag.sys_id,
                                                           location,
                                                           self.tasks_[ag.assigned_task],
                                                           ag.assigned_task,
                                                           ag.connectivity_level)
                else:
                    self.system_type_1_[ag.sys_id].location = location
                    
        self.system_dict_ = self.system_type_0_ | self.system_type_1_
        self.system_dict_ = self.sortDict(self.system_dict_)
        self.num_agents_ = len(self.system_dict_)
                                                               

    def updateSystemLocations(self):
        # function to update the location of all the agents in the swarm
        self.system_locations_ = list()
        self.system_type_0_locations_ = list()
        self.system_type_1_locations_ = list()
        
        for val in self.system_type_0_.values():
            self.system_type_0_locations_.append(val.location)
        for val in self.system_type_1_.values():
            self.system_type_1_locations_.append(val.location)

        total_locations = self.system_type_0_locations_ + self.system_type_1_locations_

        self.system_type_0_locations_ = np.array(self.system_type_0_locations_)
        self.system_type_1_locations_ = np.array(self.system_type_1_locations_)

        self.system_locations_ = np.array(total_locations)

        
    def clusterByTask(self):
        for ag in self.system_dict_.values():
            ag.cluster = ag.task_index

            
    def sortDict(self, in_dict: dict):
        return dict(sorted(in_dict.items()))
        
            
    def calcConnectionVector(self):
        l = list()
        keys = list(self.system_dict_.keys())
        count = 0
        for i in keys:
            keys_mod = keys[count+1:]
            for j in keys_mod:
                if j in self.system_dict_[i].connections:
                    l.append(1)
                else:
                    l.append(0)
            count += 1
        return np.array(l)


    def publishControl(self, control):
        msg = TwistStamped()
        msg.twist.linear.x = control[0]
        msg.twist.linear.y = control[1]
        msg.twist.linear.z = -5.0
        self.control_vec_pub_.publish(msg)

    
    def cycleCallback(self):

        self.get_logger().info("num_agents: %s" %self.num_agents_)
        if self.received_ and self.received_pose_:

            self.system_type_0_ = self.sortDict(self.system_type_0_)
            self.system_type_1_ = self.sortDict(self.system_type_1_)

            self.updateSystemLocations()

            self.my_index_ = list(self.system_dict_.keys()).index(self.sys_id_)
            self.get_logger().info("my index: %s" %self.my_index_)
            
            self.clusterByTask()
            for cluster in self.clusters_:
                cluster.setMembers(self.system_dict_.values())
                self.get_logger().info("cluster: %s"%(cluster))

            level_0_network = K0Network(self.clusters_, self.level_0_connectivity_) 

            full_network = Network(self.clusters_,
                                   self.system_dict_.values(),
                                   level_0_network,
                                   self.level_0_connectivity_,
                                   self.level_1_connectivity_,
                                   len(self.system_type_0_),
                                   len(self.system_type_1_)
                                   )
            control = list()
            for ag in self.system_dict_.values():
                ag.setConnections(full_network.connections)
                ag.calcDesiredControl(self.magnitude_)
                control.append(ag.desired_control)
            self.desired_control_ = np.array(control)
            
            self.connection_vector_ = self.calcConnectionVector()
            self.get_logger().info("locations: %s " %(self.system_locations_))

            self.planner_.locations = self.system_locations_
            self.planner_.connection_vector = self.connection_vector_
            self.planner_.num_agents = self.num_agents_
            
            self.connection_certificate_.locs = self.system_locations_.flatten()
            self.safety_certificate_.locs = self.system_locations_.flatten()

            self.connection_certificate_.num_agents = self.num_agents_
            self.safety_certificate_.num_agents = self.num_agents_
            
            u_star = self.planner_.optimize(self.safety_certificate_, self.connection_certificate_, self.desired_control_.flatten())

            control = u_star[self.my_index_,:]
            self.publishControl(control)
            self.get_logger().info("control vector: %s" %(control))

            
            
        # TODO
        # 1. cluster by task -- DONE
        # 2. initiate k0 network -- DONE
        # 3. initiate the whole network -- DONE
        # 4. connect the network -- DONE
        # 5. save the connections -- DONE
        # 6. set connections in a vector -- DONE
        # 7. optimize -- DONE
        # 8. interpret the output of the optimization
