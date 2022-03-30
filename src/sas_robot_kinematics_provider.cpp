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
#include <sas_robot_kinematics/sas_robot_kinematics_provider.h>
#include <sas_conversions/sas_conversions.h>

namespace sas
{

void RobotKinematicsProvider::_callback_desired_pose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    desired_pose_ = geometry_msgs_pose_to_dq(msg->pose);
}

void RobotKinematicsProvider::_callback_desired_interpolator_speed(const std_msgs::Float64::ConstPtr &msg)
{
    desired_interpolator_speed_ = std_msgs_float64_to_double(*msg);
}

RobotKinematicsProvider::RobotKinematicsProvider(ros::NodeHandle &node_handle, const std::string &topic_prefix):
    RobotKinematicsProvider(node_handle, node_handle, topic_prefix)
{
    //Delegated to RobotKinematicsProvider::RobotKinematicsProvider(ros::NodeHandle &nodehandle_publisher, ros::NodeHandle &nodehandle_subscriber, const std::string &topic_prefix)
}

RobotKinematicsProvider::RobotKinematicsProvider(ros::NodeHandle &nodehandle_publisher, ros::NodeHandle &nodehandle_subscriber, const std::string &topic_prefix):
    enabled_(false),
    topic_prefix_(topic_prefix),
    desired_pose_(0)
{
    ROS_INFO_STREAM(ros::this_node::getName() + "::Initializing RobotKinematicsProvider with prefix " + topic_prefix);
    publisher_pose_ = nodehandle_publisher.advertise<geometry_msgs::PoseStamped>(topic_prefix + "get/pose", 1);
    publisher_reference_frame_ = nodehandle_publisher.advertise<geometry_msgs::PoseStamped>(topic_prefix + "get/reference_frame", 1);

    subscriber_desired_pose_ = nodehandle_subscriber.subscribe(topic_prefix + "set/desired_pose", 1, &RobotKinematicsProvider::_callback_desired_pose, this);
    subscriber_desired_interpolator_speed_ = nodehandle_subscriber.subscribe(topic_prefix + "set/desired_interpolator_speed", 1, &RobotKinematicsProvider::_callback_desired_interpolator_speed, this);
}

bool RobotKinematicsProvider::is_enabled() const
{
    return is_unit(desired_pose_);
}

DQ RobotKinematicsProvider::get_desired_pose() const
{
    if(is_enabled())
    {
        return desired_pose_;
    }
    else
    {
        throw std::runtime_error(ros::this_node::getName() + "::Trying to get_desired_pose of an unitialized RobotKinematicsProvider.");
    }
}

double RobotKinematicsProvider::get_desired_interpolator_speed() const
{
    return desired_interpolator_speed_;
}

void RobotKinematicsProvider::send_pose(const DQ &pose) const
{
    publisher_pose_.publish(dq_to_geometry_msgs_pose_stamped(pose));
}

void RobotKinematicsProvider::send_reference_frame(const DQ &reference_frame) const
{
    publisher_reference_frame_.publish(sas::dq_to_geometry_msgs_pose_stamped(reference_frame));
}

}


