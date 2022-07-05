#pragma once
/*
# Copyright (c) 2020-2022 Murilo Marques Marinho
#
#    This file is part of sas_robot_kinematics.
#
#    sas_robot_kinematics is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    sas_robot_driver is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with sas_robot_kinematics.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Murilo M. Marinho, email: murilo@nml.t.u-tokyo.ac.jp
#
# ################################################################*/

#include <atomic>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <dqrobotics/DQ.h>
using namespace DQ_robotics;

namespace sas
{
class RobotKinematicsInterface
{
private:
    std::atomic_bool enabled_;
    const std::string topic_prefix_;

    ros::Subscriber subscriber_pose_;
    DQ pose_;
    ros::Subscriber subscriber_reference_frame_;
    DQ reference_frame_;

    ros::Publisher publisher_desired_pose_;
    ros::Publisher publisher_desired_interpolator_speed_;

    void _callback_pose(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void _callback_reference_frame(const geometry_msgs::PoseStamped& msg);
public:
    RobotKinematicsInterface()=delete;
    RobotKinematicsInterface(const RobotKinematicsInterface&)=delete;

#ifdef IS_SAS_PYTHON_BUILD
    RobotKinematicsInterface(const std::string& topic_prefix);
#endif
    RobotKinematicsInterface(ros::NodeHandle& node_handle, const std::string& topic_prefix);
    RobotKinematicsInterface(ros::NodeHandle& node_handle_publisher, ros::NodeHandle& node_handle_subscriber, const std::string& topic_prefix);

    bool is_enabled() const;
    DQ get_pose() const;
    DQ get_reference_frame() const;
    void send_desired_pose(const DQ& desired_pose) const;
    void send_desired_interpolator_speed(const double& interpolator_speed) const;
};
}
