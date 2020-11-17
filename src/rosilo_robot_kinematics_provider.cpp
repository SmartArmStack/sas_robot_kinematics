#include <rosilo_robot_kinematics/rosilo_robot_kinematics_provider.h>
#include <rosilo_conversions/rosilo_conversions.h>
namespace rosilo
{

void RobotKinematicsProvider::_callback_desired_pose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    desired_pose_ = geometry_msgs_pose_to_dq(msg->pose);
    if(! enabled_)
    {
        enabled_ = true;
        ROS_INFO_STREAM(ros::this_node::getName() + "::RobotKinematicsProvider enabled.");
    }
}

void RobotKinematicsProvider::_callback_desired_gripper_state(const std_msgs::Float64::ConstPtr &msg)
{
    desired_gripper_state_ = rosilo::std_msgs_float64_to_double(*msg);
}

RobotKinematicsProvider::RobotKinematicsProvider(ros::NodeHandle &node_handle, const std::string &topic_prefix):
    enabled_(false),
    topic_prefix_(topic_prefix),
    desired_pose_(0),
    desired_gripper_state_(0)
{
    ROS_INFO_STREAM(ros::this_node::getName() + "::Initializing RobotKinematicsProvider with prefix " + topic_prefix);
    subscriber_desired_pose_ = node_handle.subscribe(topic_prefix + "/set/desired_pose", 1, &RobotKinematicsProvider::_callback_desired_pose, this);
    subscriber_desired_gripper_state_  = node_handle.subscribe(topic_prefix + "/set/desired_gripper_state", 1, &RobotKinematicsProvider::_callback_desired_gripper_state, this);
    publisher_pose_ = node_handle.advertise<geometry_msgs::PoseStamped>(topic_prefix + "/get/pose", 1);
    publisher_gripper_state_ = node_handle.advertise<std_msgs::Float64>(topic_prefix + "/get/gripper_state", 1);
}

RobotKinematicsProvider::RobotKinematicsProvider(ros::NodeHandle &nodehandle_publisher, ros::NodeHandle &nodehandle_subscriber, const std::string &topic_prefix):
    enabled_(false),
    topic_prefix_(topic_prefix),
    desired_pose_(0),
    desired_gripper_state_(0)
{
    ROS_INFO_STREAM(ros::this_node::getName() + "::Initializing RobotKinematicsProvider with prefix " + topic_prefix);
    //The callback queue needs to be defined before the nodehandle is used (with subscribe or advertise), otherwise it doesn't work
    nodehandle_subscriber.setCallbackQueue(&subscriber_callback_queue_);
    nodehandle_publisher.setCallbackQueue(&publisher_callback_queue_);

    subscriber_desired_pose_ = nodehandle_subscriber.subscribe(topic_prefix + "/set/desired_pose", 1, &RobotKinematicsProvider::_callback_desired_pose, this);
    subscriber_desired_gripper_state_  = nodehandle_subscriber.subscribe(topic_prefix + "/set/desired_gripper_state", 1, &RobotKinematicsProvider::_callback_desired_gripper_state, this);
    publisher_pose_ = nodehandle_publisher.advertise<geometry_msgs::PoseStamped>(topic_prefix + "/get/pose", 1);
    publisher_gripper_state_ = nodehandle_publisher.advertise<std_msgs::Float64>(topic_prefix + "/get/gripper_state", 1);
}

bool RobotKinematicsProvider::is_enabled() const
{
    return enabled_;
}

DQ RobotKinematicsProvider::get_desired_pose() const
{
    if(enabled_)
    {
        return desired_pose_;
    }
    else
    {
        throw std::runtime_error(ros::this_node::getName() + "::Trying to get_desired_pose of an unitialized RobotKinematicsProvider.");
    }
}

void RobotKinematicsProvider::send_pose(const DQ &pose) const
{
    publisher_pose_.publish(dq_to_geometry_msgs_pose_stamped(pose));
}

void RobotKinematicsProvider::send_gripper_state(const double& gripper_state) const
{
    publisher_gripper_state_.publish(rosilo::double_to_std_msgs_float64(gripper_state));
}

double RobotKinematicsProvider::get_desired_gripper_state() const
{
    return desired_gripper_state_;
}

}


