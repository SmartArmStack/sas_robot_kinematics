"""
# Copyright (c) 2012-2021 Murilo Marques Marinho
#
#    This file is part of rosilo_robot_driver.
#
#    rosilo_robot_driver is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    rosilo_robot_driver is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with rosilo_robot_driver.  If not, see <https://www.gnu.org/licenses/>.
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


class RobotKinematicsProvider:

    def __init__(self, topic_prefix):
        self.desired_interpolator_speed_ = None
        self.desired_pose_ = None
        self.enabled_ = False
        self.topic_prefix = topic_prefix
        self.desired_pose_ = 0

        print(rospy.get_name() + "::Initializing RobotKinematicsProvider with prefix " + topic_prefix)
        self.publisher_pose_ = rospy.Publisher(topic_prefix + "get/pose", PoseStamped, queue_size=1)

        self.subscriber_desired_pose_ = rospy.Subscriber(topic_prefix + "set/desired_pose",
                                                         PoseStamped,
                                                         self._callback_desired_pose)
        self.subscriber_desired_interpolator_speed_ = rospy.Subscriber(topic_prefix + "set/desired_interpolator_speed",
                                                                       Float64,
                                                                       self._callback_desired_interpolator_speed)

    def _callback_desired_pose(self, msg):
        self.desired_pose_ = rc.geometry_msgs_pose_to_dq(msg.pose)
        if not self.enabled_:
            self.enabled_ = True
            print(rospy.get_name() + "::RobotKinematicsProvider enabled.")

    def _callback_desired_interpolator_speed(self, msg):
        self.desired_interpolator_speed_ = rc.std_msgs_float64_to_double(msg)

    def is_enabled(self):
        return self.enabled_

    def get_desired_pose(self):
        if self.enabled_:
            return self.desired_pose_
        else:
            raise Exception(
                rospy.get_name() + "::Trying to get_desired_pose of an unitialized RobotKinematicsProvider.")

    def get_desired_interpolator_speed(self):
        return self.desired_interpolator_speed_

    def send_pose(self, pose):
        self.publisher_pose_.publish(rc.dq_to_geometry_msgs_pose_stamped(pose))
