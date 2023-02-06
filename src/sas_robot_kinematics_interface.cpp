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
#include <sas_robot_kinematics/sas_robot_kinematics_interface.h>
#include <sas_conversions/sas_conversions.h>
#include <sas_common/sas_common.h>
namespace sas
{

void RobotKinematicsInterface::_callback_pose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pose_ = geometry_msgs_pose_to_dq(msg->pose);
    if(! enabled_)
    {
        enabled_ = true;
        ROS_INFO_STREAM(ros::this_node::getName() + "::RobotKinematicsInterface enabled.");
    }
}

void RobotKinematicsInterface::_callback_reference_frame(const geometry_msgs::PoseStamped &msg)
{
    reference_frame_ = geometry_msgs_pose_stamped_to_dq(msg);
}

#ifdef IS_SAS_PYTHON_BUILD
RobotKinematicsInterface::RobotKinematicsInterface(const std::string &topic_prefix):
    RobotKinematicsInterface(sas::common::get_static_node_handle(), topic_prefix)
{
    //Delegated to RobotKinematicsInterface::RobotKinematicsInterface(ros::NodeHandle &node_handle_publisher, ros::NodeHandle &node_handle_subscriber, const std::string &topic_prefix)
}
#endif

RobotKinematicsInterface::RobotKinematicsInterface(ros::NodeHandle &node_handle, const std::string &topic_prefix):
    RobotKinematicsInterface(node_handle, node_handle, topic_prefix)
{
    //Delegated to RobotKinematicsInterface::RobotKinematicsInterface(ros::NodeHandle &node_handle_publisher, ros::NodeHandle &node_handle_subscriber, const std::string &topic_prefix)
}

RobotKinematicsInterface::RobotKinematicsInterface(ros::NodeHandle &node_handle_publisher, ros::NodeHandle &node_handle_subscriber, const std::string &topic_prefix):
    enabled_(false),
    topic_prefix_(topic_prefix),
    pose_(0),
    reference_frame_(0)
{
    ROS_INFO_STREAM(ros::this_node::getName() + "::Initializing RobotKinematicsInterface with prefix " + topic_prefix);
    subscriber_pose_ = node_handle_subscriber.subscribe(topic_prefix + "/get/pose", 1, &RobotKinematicsInterface::_callback_pose, this);
    subscriber_reference_frame_ = node_handle_subscriber.subscribe(topic_prefix + "/get/reference_frame", 1, &RobotKinematicsInterface::_callback_reference_frame, this);
    publisher_desired_pose_ = node_handle_publisher.advertise<geometry_msgs::PoseStamped>(topic_prefix + "/set/desired_pose", 1);
    publisher_desired_interpolator_speed_ = node_handle_publisher.advertise<std_msgs::Float64>(topic_prefix + "/set/desired_interpolator_speed", 1);
}

bool RobotKinematicsInterface::is_enabled() const
{
    return( is_unit(pose_) &&
            is_unit(reference_frame_));
}

DQ RobotKinematicsInterface::get_pose() const
{
    if(is_enabled())
    {
        return pose_;
    }
    else
    {
        throw std::runtime_error(ros::this_node::getName() + "::Trying to get_desired_pose of an unitialized RobotKinematicsInterface");
    }
}

DQ RobotKinematicsInterface::get_reference_frame() const
{
    if(is_enabled())
    {
        return reference_frame_;
    }
    else
        throw std::runtime_error(ros::this_node::getName() + "::RobotKinematicsInterface::get_reference_frame()::trying to get reference frame but uninitialized.");
}

void RobotKinematicsInterface::send_desired_pose(const DQ &desired_pose) const
{
    publisher_desired_pose_.publish(dq_to_geometry_msgs_pose_stamped(desired_pose));
}

void RobotKinematicsInterface::send_desired_interpolator_speed(const double &interpolator_speed) const
{
    publisher_desired_interpolator_speed_.publish(double_to_std_msgs_float64(interpolator_speed));
}

}


