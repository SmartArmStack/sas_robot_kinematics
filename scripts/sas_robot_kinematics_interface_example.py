#!/usr/bin/python3
"""
# Copyright (c) 2020-2026 Murilo Marques Marinho
#
#    This file is part of sas_robot_kinematics.
#
#    sas_robot_kinematics is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    sas_robot_kinematics is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with sas_robot_kinematics.  If not, see <https://www.gnu.org/licenses/>.
#
# #######################################################################################
#
#   Author: Murilo M. Marinho, email: murilomarinho@ieee.org
#
# #######################################################################################
"""
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
