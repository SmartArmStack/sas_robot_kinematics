#include <rosilo_robot_kinematics/rosilo_robot_kinematics_interface.h>
#include <rosilo_conversions/rosilo_conversions.h>
namespace rosilo
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


RobotKinematicsInterface::RobotKinematicsInterface(ros::NodeHandle &node_handle, const std::string &topic_prefix):
    RobotKinematicsInterface(node_handle, node_handle, topic_prefix)
{
    //Delegated to RobotKinematicsInterface::RobotKinematicsInterface(ros::NodeHandle &node_handle_publisher, ros::NodeHandle &node_handle_subscriber, const std::string &topic_prefix)
}

RobotKinematicsInterface::RobotKinematicsInterface(ros::NodeHandle &node_handle_publisher, ros::NodeHandle &node_handle_subscriber, const std::string &topic_prefix):
    enabled_(false),
    topic_prefix_(topic_prefix),
    pose_(0)
{
    ROS_INFO_STREAM(ros::this_node::getName() + "::Initializing RobotKinematicsInterface with prefix " + topic_prefix);
    subscriber_pose_ = node_handle_subscriber.subscribe(topic_prefix + "get/pose", 1, &RobotKinematicsInterface::_callback_pose, this);
    publisher_desired_pose_ = node_handle_publisher.advertise<geometry_msgs::PoseStamped>(topic_prefix + "set/desired_pose", 1);
    publisher_desired_gripper_state_ = node_handle_publisher.advertise<std_msgs::Float64>(topic_prefix + "set/desired_gripper_state", 1);
    publisher_desired_interpolator_speed_ = node_handle_publisher.advertise<std_msgs::Float64>(topic_prefix + "set/desired_interpolator_speed", 1);
}

bool RobotKinematicsInterface::is_enabled() const
{
    return enabled_;
}

DQ RobotKinematicsInterface::get_pose() const
{
    if(enabled_)
    {
        return pose_;
    }
    else
    {
        throw std::runtime_error(ros::this_node::getName() + "::Trying to get_desired_pose of an unitialized RobotKinematicsInterface");
    }
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


