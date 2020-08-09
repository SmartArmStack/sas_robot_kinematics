#pragma once

#include <atomic>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <dqrobotics/DQ.h>
using namespace DQ_robotics;

namespace rosilo
{
class RobotKinematicsInterface
{
private:
    std::atomic_bool enabled_;
    const std::string topic_prefix_;

    ros::Subscriber subscriber_pose_;
    DQ pose_;

    ros::Publisher publisher_desired_pose_;

    void _callback_pose(const geometry_msgs::PoseStamped::ConstPtr& msg);
public:
    RobotKinematicsInterface(const RobotKinematicsInterface&)=delete;
    explicit RobotKinematicsInterface(ros::NodeHandle& node_handle, const std::string& topic_prefix);
    explicit RobotKinematicsInterface(ros::NodeHandle& node_handle_publisher, ros::NodeHandle& node_handle_subscriber, const std::string& topic_prefix);

    bool is_enabled() const;
    DQ get_pose() const;
    void send_desired_pose(const DQ& desired_pose) const;
};
}
