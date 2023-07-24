#!/usr/bin/env python3
"""
Author: Jason Hughes
Date: June 2023
About: Node for the dual-layered k-connectivity planner
"""

import rclpy
from connectivity import AgentConnectivity

def main(args=None):
    rclpy.init(args=args)

    conn = AgentConnectivity()

    rclpy.spin(conn)

    conn.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
