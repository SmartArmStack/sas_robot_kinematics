#pragma once
/*
# Copyright (c) 2020-2023 Murilo Marques Marinho
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

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sas_core/sas_object.hpp>
#include <sas_msgs/msg/float64.hpp>

using namespace rclcpp;
using namespace DQ_robotics;

namespace sas
{
class RobotKinematicsInterface: private sas::Object
{
private:
    std::shared_ptr<Node> node_;

    std::atomic_bool enabled_;
    const std::string topic_prefix_;

    Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_pose_;
    DQ pose_;
    Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_reference_frame_;
    DQ reference_frame_;

    Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_desired_pose_;
    Publisher<sas_msgs::msg::Float64>::SharedPtr publisher_desired_interpolator_speed_;

    void _callback_pose(const geometry_msgs::msg::PoseStamped& msg);
    void _callback_reference_frame(const geometry_msgs::msg::PoseStamped& msg);
public:
    RobotKinematicsInterface()=delete;
    RobotKinematicsInterface(const RobotKinematicsInterface&)=delete;

//#ifdef IS_SAS_PYTHON_BUILD
//    RobotKinematicsInterface(const std::string& topic_prefix);
//#endif
    RobotKinematicsInterface(const std::shared_ptr<Node> &node, const std::string& topic_prefix);

    bool is_enabled() const;
    DQ get_pose() const;
    DQ get_reference_frame() const;
    void send_desired_pose(const DQ& desired_pose) const;
    void send_desired_interpolator_speed(const double& interpolator_speed) const;
};
}
