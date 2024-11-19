#ifndef XBOT_ROS_BRIDGE_HPP
#define XBOT_ROS_BRIDGE_HPP

/**
 * Implements FollowJointTrajectoryAction interface.

 * Subscribes to /xbotcore/joint_state to get current robot state.

 * Receives trajectory from `repair_motion_controller` as a FollowJointTrajectoryActionGoal. (/robot_trajectory_controller/follow_joint_trajectory/goal)

 * Loops through the trajectory and publishes each point to /xbotcore/command.

 * Publishes trajectory feedback to
    - `/robot_trajectory_controller/follow_joint_trajectory/feedback`. Publishes trajectory result to
    - `/robot_trajectory_controller/follow_joint_trajectory/result`.
 */


#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <xbot_msgs/JointCommand.h>
#include <xbot_msgs/JointState.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>


class JointTrajectoryExecutor
{
public:
    JointTrajectoryExecutor(
        ros::NodeHandle nh,
        std::string robot_controller_name,
        double goal_execution_timeout,
        double joint_angle_tolerance,
        std::shared_ptr<xbot_msgs::JointState> current_joint_state_ptr
    );

    virtual ~JointTrajectoryExecutor();

    ros::NodeHandle nh_;

    // action server
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> follow_joint_trajectory_as_;

    // Subscribers
    ros::Subscriber joint_state_sub_;

    // Publishers
    ros::Publisher xbot_joint_command_pub_;

    // callbacks
    void jointStateCB(const xbot_msgs::JointState::ConstPtr& msg);

    // action server callbacks
    void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal);

    // helper functions
    void publishJointCommads(std::vector<std::string> joint_names, std::vector<double> joint_positions);

private:
    xbot_msgs::JointState current_joint_state_;
    double joint_angle_tolerance_;
    double goal_execution_timeout_;
    std::shared_ptr<xbot_msgs::JointState> current_joint_state_ptr_;
    control_msgs::FollowJointTrajectoryResult result_;

};





class XbotRosBridge
{

public:
    double goal_execution_timeout_ = 5.0;
    double joint_angle_tolerance_ = 0.02;

    XbotRosBridge(ros::NodeHandle nh);

    virtual ~XbotRosBridge();

    ros::NodeHandle nh_;

    // repair controller name
    std::string robot_controller_name = "/robot_trajectory_controller";

    // trajectory executor
    std::shared_ptr<JointTrajectoryExecutor> joint_trajectory_executor_;

    // subscribers
    ros::Subscriber xbot_joint_state_sub_;

    // subscriber callback
    void xbotJointStateCB(const xbot_msgs::JointState::ConstPtr& msg);

    // publishers
    ros::Publisher ros_joint_state_pub_;

    // current joint state as a shared pointer
    xbot_msgs::JointState current_joint_state_;
    std::shared_ptr<xbot_msgs::JointState> current_joint_state_ptr_;

};


#endif // XBOT_ROS_BRIDGE_HPP