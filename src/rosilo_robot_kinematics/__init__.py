"""
# Copyright (c) 2012-2021 Murilo Marques Marinho
#
#    This file is part of rosilo_robot_kinematics.
#
#    rosilo_robot_kinematics is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    rosilo_robot_kinematics is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with rosilo_robot_kinematics.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Murilo M. Marinho, email: murilo@nml.t.u-tokyo.ac.jp
#
# ################################################################
"""

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import rosilo_conversions as rc


class RobotKinematicsInterface:
    def __init__(self, node_prefix):
        self.enabled_ = False
        self.pose_ = None

        self.publisher_desired_pose_ = rospy.Publisher(node_prefix + "set/desired_pose",
                                                       PoseStamped,
                                                       queue_size=1)
        self.publisher_desired_gripper_state_ = rospy.Publisher(node_prefix + "set/desired_gripper_state",
                                                                Float64,
                                                                queue_size=1)
        self.publisher_desired_interpolator_speed_ = rospy.Publisher(node_prefix + "set/desired_interpolator_speed",
                                                                     Float64,
                                                                     queue_size=1)

        self.subscriber_pose_ = rospy.Subscriber(node_prefix + "get/pose",
                                                 PoseStamped,
                                                 self._callback_pose)

    def is_enabled(self):
        if self.pose_ is not None:
            return True
        else:
            return False

    def get_pose(self):
        if self.pose_ is not None:
            return self.pose_
        else:
            raise Exception(rospy.get_name() + "::Trying to get_pose but value is None.")

    def send_desired_pose(self, desired_pose):
        self.publisher_desired_pose_.publish(rc.dq_to_geometry_msgs_pose_stamped(desired_pose))

    def send_desired_interpolator_speed(self, desired_interpolator_speed):
        self.publisher_desired_interpolator_speed_.publish(rc.double_to_std_msgs_float64(desired_interpolator_speed))

    def _callback_pose(self, msg):
        self.pose_ = rc.geometry_msgs_pose_stamped_to_dq(msg)
