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
 * @class RobotKinematicsClient
 * @brief Client for robot kinematics.
 *
 * Subscribes to the robot's current pose and reference frame, and provides
 * methods to publish desired poses and desired interpolator speeds. The
 * client tracks the most recently received pose and reference frame and
 * exposes simple accessors to retrieve them.
 */
class RobotKinematicsClient: private sas::Object
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
    RobotKinematicsClient()=delete;
    RobotKinematicsClient(const RobotKinematicsClient&)=delete;

//#ifdef IS_SAS_PYTHON_BUILD
//    RobotKinematicsInterface(const std::string& topic_prefix);
//#endif
    /**
     * @brief Construct a RobotKinematicsClient.
     *
     * @param node Shared pointer to the ROS2 node used for creating
     *             publishers/subscribers.
     * @param topic_prefix Topic name prefix used for subscriptions and
     *                     publications.
     */
    RobotKinematicsClient(const std::shared_ptr<Node> &node, const std::string& topic_prefix);

    /**
     * @brief Check whether the client is enabled (subscribers/publishers active).
     * @return true if enabled, false otherwise.
     */
    bool is_enabled() const;

    /**
     * @brief Get the last received robot pose.
     * @return Current pose as a dual quaternion (DQ).
     */
    DQ get_pose() const;

    /**
     * @brief Get the last received reference frame pose.
     * @return Reference frame as a dual quaternion (DQ).
     */
    DQ get_reference_frame() const;

    /**
     * @brief Publish a desired pose for the robot.
     * @param desired_pose Desired pose to publish (as DQ).
     */
    void send_desired_pose(const DQ& desired_pose) const;

    /**
     * @brief Publish a desired interpolator speed value.
     * @param interpolator_speed Desired interpolator speed (units depend on
     *                           the consumer of this topic).
     */
    void send_desired_interpolator_speed(const double& interpolator_speed) const;
};
}
