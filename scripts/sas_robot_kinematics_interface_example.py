# Copyright (c) 2020-2022
# Murilo Marques Marinho at the University of Tokyo.
# This software can be used for Research Purposes only.
# For commercial purposes, contact the author.
# Author: Murilo M. Marinho, email: murilomarinho@ieee.org
import time
from dqrobotics import *  # Despite what PyCharm might say, this is very much necessary or DQs will not be recognized

from sas_common import rclcpp_init, rclcpp_Node, rclcpp_spin_some
from sas_robot_kinematics import RobotKinematicsClient, RobotKinematicsServer

try:
    # Initialize rclcpp
    rclcpp_init()
    # Get a rclcpp_Node
    node = rclcpp_Node("my_node_name")

    # Initialize the RobotKinematicsServer
    rkp = RobotKinematicsServer(node, 'my_test_kinematics')

    # Initialize the RobotKinematicsClient
    rki = RobotKinematicsClient(node, 'my_test_kinematics')

    # Wait for RobotKinematicsClient to be enabled
    while not rki.is_enabled():
        time.sleep(0.1)
        # Send info from RobotKinematicsServer to the RobotKinematicsClient
        # RobotKinematicsClient will be enabled when those values are received
        rkp.send_pose(DQ([1]))
        rkp.send_reference_frame(DQ([1]))
        rclcpp_spin_some(node)

    # Read the values sent by the RobotKinematicsServer
    print(rki.get_pose())
    print(rki.get_reference_frame())

except KeyboardInterrupt:
    print("Interrupted by user")
