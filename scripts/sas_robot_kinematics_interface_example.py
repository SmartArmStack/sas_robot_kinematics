# Copyright (c) 2020-2022
# Murilo Marques Marinho at the University of Tokyo.
# This software can be used for Research Purposes only.
# For commercial purposes, contact the author.
# Author: Murilo M. Marinho, email: murilomarinho@ieee.org
import time
from dqrobotics import *  # Despite what PyCharm might say, this is very much necessary or DQs will not be recognized
import rospy
from sas_robot_kinematics import RobotKinematicsInterface, RobotKinematicsProvider

rospy.init_node('my_node_name', disable_signals=True)
try:
    # Initialize the RobotKinematicsProvider
    rkp = RobotKinematicsProvider('my_test_kinematics')

    # Initialize the RobotKinematicsInterface
    rki = RobotKinematicsInterface('my_test_kinematics')

    # Wait for RobotKinematicsInterface to be enabled
    while not rki.is_enabled():
        time.sleep(0.1)
        # Send info from RobotKinematicsProvider to the RobotKinematicsInterface
        # RobotKinematicsInterface will be enabled when those values are received
        rkp.send_pose(DQ([1]))
        rkp.send_reference_frame(DQ([1]))

    # Read the values sent by the RobotKinematicsProvider
    print(rki.get_pose())
    print(rki.get_reference_frame())

except KeyboardInterrupt:
    print("Interrupted by user")
