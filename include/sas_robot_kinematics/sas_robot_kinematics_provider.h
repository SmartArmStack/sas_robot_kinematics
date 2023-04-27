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

class RobotKinematicsProvider: private sas::Object
{
protected:
    std::shared_ptr<Node> node_;

    std::atomic_bool enabled_;
    const std::string topic_prefix_;

    Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_pose_;
    Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_reference_frame_;

    Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_desired_pose_;
    DQ desired_pose_;
    Subscription<sas_msgs::msg::Float64>::SharedPtr subscriber_desired_interpolator_speed_;
    double desired_interpolator_speed_;

    void _callback_desired_pose(const geometry_msgs::msg::PoseStamped& msg);
    void _callback_desired_interpolator_speed(const sas_msgs::msg::Float64& msg);
public:
    RobotKinematicsProvider()=delete;
    RobotKinematicsProvider(const RobotKinematicsProvider&)=delete;

#ifdef IS_SAS_PYTHON_BUILD
    RobotKinematicsProvider(const std::string& topic_prefix);
#endif
    RobotKinematicsProvider(const std::shared_ptr<Node>& node, const std::string& topic_prefix);

    DQ get_desired_pose() const;
    double get_desired_interpolator_speed() const;

    bool is_enabled() const;

    void send_pose(const DQ& pose) const;
    void send_reference_frame(const DQ& reference_frame) const;

};

}
