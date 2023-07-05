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

    #path = "../config/connectivity.yaml"
    #with open(path,'r') as f:
    #    config = yaml.safe_load(f)

    path = os.path.join(
        get_package_share_directory('dlkc'),
        'config',
        'connectivity.yaml'
        )

    with open(path,'r') as f:
        config = yaml.safe_load(f)
    
    params = config[list(config.keys())[0]]['ros__parameters']     
    
    node = Node(package = 'dlkc',
                name = 'dlkc_node',
                executable = 'connectivity_node.py',
                parameters = [params])

    ld.add_action(node)

    return ld

        
