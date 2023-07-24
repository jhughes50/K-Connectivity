"""
Author: Jason Hughes
Date: July 2023
About: launch file for sim
"""

import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace 
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    ld = LaunchDescription()

    path = os.path.join(
        get_package_share_directory('dlkc'),
        'config',
        'connectivity.yaml'
        )

    with open(path,'r') as f:
        config = yaml.safe_load(f)

    for arg in sys.argv:
        if arg.startswith("num_agents:="):
            num_agents = int(arg.split(':=')[1])
        
    params = config[list(config.keys())[0]]['ros__parameters']     
   
    for i in range(1,num_agents+1):     
        params["sys_id"] = i
        params["level"] = 0
        
        node = Node(package = 'dlkc',
                    name = 'dlkc_node',
                    executable = 'connectivity_node.py',
                    parameters = [params],
                    output = 'screen'
                    )

        ld.add_action(node)

    return ld

        
