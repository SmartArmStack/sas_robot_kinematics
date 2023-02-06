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
#   Author: Murilo M. Marinho, email: murilomarinho@ieee.org
#
# ################################################################*/

#include <atomic>

#include <dqrobotics/DQ.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>

using namespace DQ_robotics;

namespace sas
{

class RobotKinematicsProvider
{
protected:
    std::atomic_bool enabled_;
    const std::string topic_prefix_;

    ros::Publisher publisher_pose_;
    ros::Publisher publisher_reference_frame_;

    ros::Subscriber subscriber_desired_pose_;
    DQ desired_pose_;
    ros::Subscriber subscriber_desired_interpolator_speed_;
    double desired_interpolator_speed_;

    void _callback_desired_pose(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void _callback_desired_interpolator_speed(const std_msgs::Float64::ConstPtr& msg);
public:
    RobotKinematicsProvider()=delete;
    RobotKinematicsProvider(const RobotKinematicsProvider&)=delete;

#ifdef IS_SAS_PYTHON_BUILD
    RobotKinematicsProvider(const std::string& topic_prefix);
#endif
    RobotKinematicsProvider(ros::NodeHandle& node_handle, const std::string& topic_prefix);
    RobotKinematicsProvider(ros::NodeHandle& node_handle_publisher, ros::NodeHandle& node_handle_subscriber, const std::string& topic_prefix);

    DQ get_desired_pose() const;
    double get_desired_interpolator_speed() const;

    bool is_enabled() const;

    void send_pose(const DQ& pose) const;
    void send_reference_frame(const DQ& reference_frame) const;

};

}
