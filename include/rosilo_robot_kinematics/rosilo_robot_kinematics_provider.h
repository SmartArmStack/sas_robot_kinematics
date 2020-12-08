#pragma once

#include <atomic>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>

#include <dqrobotics/DQ.h>
using namespace DQ_robotics;

namespace rosilo
{
class RobotKinematicsProvider
{
protected:
    ros::CallbackQueue publisher_callback_queue_;
    ros::CallbackQueue subscriber_callback_queue_;

    std::atomic_bool enabled_;
    const std::string topic_prefix_;

    ros::Subscriber subscriber_desired_pose_;
    DQ desired_pose_;

    ros::Publisher publisher_pose_;

    void _callback_desired_pose(const geometry_msgs::PoseStamped::ConstPtr& msg);

public:
    RobotKinematicsProvider(const RobotKinematicsProvider&)=delete;
    explicit RobotKinematicsProvider(ros::NodeHandle& node_handle, const std::string& topic_prefix);
    explicit RobotKinematicsProvider(ros::NodeHandle& node_handle_publisher, ros::NodeHandle& node_handle_subscriber, const std::string& topic_prefix);

    bool is_enabled() const;
    DQ get_desired_pose() const;
    void send_pose(const DQ& pose) const;
};
}
