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
/**
 * @class RobotKinematicsServer
 * @brief Server for robot kinematics topics.
 *
 * Publishes the robot's current pose and reference frame, and subscribes to
 * desired poses and desired interpolator speeds. The server stores the most
 * recently received desired values and exposes accessors for them.
 */
class RobotKinematicsServer: private sas::Object
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
    RobotKinematicsServer()=delete;
    RobotKinematicsServer(const RobotKinematicsServer&)=delete;

#ifdef IS_SAS_PYTHON_BUILD
    RobotKinematicsServer(const std::string& topic_prefix);
#endif

    /**
     * @brief Construct a RobotKinematicsServer.
     *
     * @param node Shared pointer to the ROS2 node used for creating
     *             publishers/subscribers.
     * @param topic_prefix Topic name prefix used for subscriptions and
     *                     publications.
     */
    RobotKinematicsServer(const std::shared_ptr<Node>& node, const std::string& topic_prefix);

    /**
     * @brief Get the most recently received desired pose.
     * @return Desired pose as a dual quaternion (DQ).
     */
    DQ get_desired_pose() const;

    /**
     * @brief Get the most recently received desired interpolator speed.
     * @return Desired interpolator speed as a double.
     */
    double get_desired_interpolator_speed() const;

    /**
     * @brief Check whether the server is enabled (publishers/subscribers active).
     * @return true if enabled, false otherwise.
     */
    bool is_enabled() const;

    /**
     * @brief Publish the current robot pose.
     * @param pose Pose to publish (as DQ).
     */
    void send_pose(const DQ& pose) const;

    /**
     * @brief Publish the robot's reference frame pose.
     * @param reference_frame Reference frame to publish (as DQ).
     */
    void send_reference_frame(const DQ& reference_frame) const;

};

}
