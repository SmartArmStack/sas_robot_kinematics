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

void RobotKinematicsInterface::_callback_gripper_state(const std_msgs::Float64::ConstPtr &msg)
{
    gripper_state_ = rosilo::std_msgs_float64_to_double(*msg);
}

RobotKinematicsInterface::RobotKinematicsInterface(ros::NodeHandle &node_handle, const std::string &topic_prefix):
    enabled_(false),
    topic_prefix_(topic_prefix),
    pose_(0),
    gripper_state_(0)
{
    ROS_INFO_STREAM(ros::this_node::getName() + "::Initializing RobotKinematicsInterface with prefix " + topic_prefix);
    subscriber_pose_ = node_handle.subscribe(topic_prefix + "get/pose", 1, &RobotKinematicsInterface::_callback_pose, this);
    subscriber_gripper_state_ = node_handle.subscribe(topic_prefix + "get/gripper_state", 1, &RobotKinematicsInterface::_callback_gripper_state, this);
    publisher_desired_pose_ = node_handle.advertise<geometry_msgs::PoseStamped>(topic_prefix + "set/desired_pose", 1);
    publisher_desired_gripper_state_ = node_handle.advertise<std_msgs::Float64>(topic_prefix + "set/desired_gripper_state", 1);
}

RobotKinematicsInterface::RobotKinematicsInterface(ros::NodeHandle &node_handle_publisher, ros::NodeHandle &node_handle_subscriber, const std::string &topic_prefix):
    enabled_(false),
    topic_prefix_(topic_prefix),
    pose_(0),
    gripper_state_(0)
{
    ROS_INFO_STREAM(ros::this_node::getName() + "::Initializing RobotKinematicsInterface with prefix " + topic_prefix);
    subscriber_pose_ = node_handle_subscriber.subscribe(topic_prefix + "get/pose", 1, &RobotKinematicsInterface::_callback_pose, this);
    subscriber_gripper_state_ = node_handle_subscriber.subscribe(topic_prefix + "get/gripper_state", 1, &RobotKinematicsInterface::_callback_gripper_state, this);
    publisher_desired_pose_ = node_handle_publisher.advertise<geometry_msgs::PoseStamped>(topic_prefix + "set/desired_pose", 1);
    publisher_desired_gripper_state_ = node_handle_publisher.advertise<std_msgs::Float64>(topic_prefix + "set/desired_gripper_state", 1);
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

double RobotKinematicsInterface::get_gripper_state() const
{
    if(!enabled_)
        throw std::runtime_error("Trying to RobotKinematicsInterface::get_gripper_state() when not enabled.");
    return gripper_state_;
}

void RobotKinematicsInterface::send_desired_gripper_state(const double &gripper_state) const
{
    /*if(gripper_state > 1 || gripper_state < 0)
    {
        throw std::runtime_error("Trying to RobotKinematicsInterface::send_desired_gripper_state() with gripper_state > 1 or gripper_state <0");
    }*/
    publisher_desired_gripper_state_.publish(rosilo::double_to_std_msgs_float64(gripper_state));
}

}


