#include "repair_motion_controller/xbot_ros_bridge.hpp"


JointTrajectoryExecutor::JointTrajectoryExecutor(
ros::NodeHandle nh,
std::string robot_controller_name,
double goal_execution_timeout,
double joint_angle_tolerance,
std::shared_ptr<xbot_msgs::JointState> current_joint_state_ptr):
    nh_(nh), 
    follow_joint_trajectory_as_(nh, robot_controller_name + "/follow_joint_trajectory", boost::bind(&JointTrajectoryExecutor::executeCB, this, _1), false)

{
    // set goal execution timeout
    goal_execution_timeout_ = goal_execution_timeout;

    // set joint angle tolerance
    joint_angle_tolerance_ = joint_angle_tolerance;

    // subscribers
    joint_state_sub_ = nh_.subscribe("/xbotcore/joint_states", 1, &JointTrajectoryExecutor::jointStateCB, this);

    // advertise publishers
    xbot_joint_command_pub_ = nh_.advertise<xbot_msgs::JointCommand>("/xbotcore/command", 1);

    // start action server
    follow_joint_trajectory_as_.start();

    current_joint_state_ptr_ = current_joint_state_ptr;

    ROS_INFO("Joint Trajectory Executor for %s started!", robot_controller_name.c_str());
    
}

JointTrajectoryExecutor::~JointTrajectoryExecutor(){}


void JointTrajectoryExecutor::jointStateCB(const xbot_msgs::JointState::ConstPtr& msg){
    current_joint_state_ = *msg;
}

void JointTrajectoryExecutor::executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal){

    ROS_INFO("Received a goal trajectory!");

    // check if xbot2-core is available
    if (xbot_joint_command_pub_.getNumSubscribers() == 0){
        ROS_WARN("xbot2-core is not discovered. Make sure to run it!");
        ROS_WARN("Aborting trajectory execution.");
        result_.error_code = -6;
        follow_joint_trajectory_as_.setAborted(result_, "Failed to discover xbot2-core");
        return;
    }

    // get joint names from goal
    std::vector<std::string> joint_names = goal->trajectory.joint_names;

    // get trajectory points from goal
    std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory_points = goal->trajectory.points;

    // length of trajectory
    int amount_of_trajectory_points = (int)trajectory_points.size();
    ROS_INFO("Trajectory length: %d", amount_of_trajectory_points);

    ROS_INFO("Executing trajectory...");

    std::vector<double> joint_positions;

    ros::Time start_time = ros::Time::now();

    // iterate over trajectory points
    for (int i = 0; i < trajectory_points.size(); i++)
    {
        // get joint position fro traj point
        joint_positions = trajectory_points[i].positions;

        // publish joint cmd
        publishJointCommads(joint_names, joint_positions);
        
        

        // ros::Duration(0.2).sleep(); 
        // ROS_INFO("sending next point...");

        // Calculate sleep time based on time_from_start
        ros::Duration time_from_start = trajectory_points[i].time_from_start;

        // Wait until the next time_from_start
        ros::Time current_time = ros::Time::now();
        ros::Duration sleep_time = start_time + time_from_start - current_time;

        
        
        if (sleep_time.toSec() > 0)
        {
            sleep_time.sleep();
        }
        else
        {
            ROS_WARN("Missed time point! Continuing to the next trajectory point.");
        }

        // Gradually decelerate the trajectory
        // if (i > 0)
        // {
        //     ros::Duration previous_time_from_start = trajectory_points[i - 1].time_from_start;
        //     ros::Duration current_time_from_start = trajectory_points[i].time_from_start;
        //     ros::Duration time_diff = current_time_from_start - previous_time_from_start;

        //     if (time_diff.toSec() > 0)
        //     {
        //     double deceleration_factor = (static_cast<double>(i) / amount_of_trajectory_points) * 10;
        //     ros::Duration adjusted_sleep_time = time_diff * deceleration_factor;
        //     adjusted_sleep_time.sleep();
        //     ROS_WARN("sleep %f", adjusted_sleep_time.toSec());
        //     }
        // }
    }

    ROS_INFO("Trajectory execution is completed!");
    
    // mark the goal as succeeded
    result_.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    follow_joint_trajectory_as_.setSucceeded(result_);
}

void JointTrajectoryExecutor::publishJointCommads(std::vector<std::string> joint_names, std::vector<double> joint_positions){
    // create joint command msg
    xbot_msgs::JointCommand joint_cmd;

    // set time stamp
    joint_cmd.header.stamp = ros::Time::now();

    // set joint names
    joint_cmd.name = joint_names;

    // set positions
    joint_cmd.position = std::vector<float>(joint_positions.begin(), joint_positions.end());

    // set control mode uint8_t
    joint_cmd.ctrl_mode = std::vector<uint8_t>(joint_positions.size(), 1);

    // publish
    xbot_joint_command_pub_.publish(joint_cmd);
}


XbotRosBridge::XbotRosBridge(ros::NodeHandle nh): nh_(nh)
{
    // subscribe to xbot joint states
    xbot_joint_state_sub_ = nh_.subscribe("/xbotcore/joint_states", 1, &XbotRosBridge::xbotJointStateCB, this);

    // publisher for ros joint states
    ros_joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);

    // create the pointer to the current joint state
    current_joint_state_ptr_ = std::make_shared<xbot_msgs::JointState>();

    joint_trajectory_executor_ = std::make_shared<JointTrajectoryExecutor>(nh_, robot_controller_name, goal_execution_timeout_, joint_angle_tolerance_, current_joint_state_ptr_);

    ROS_INFO("XBot ROS Bridge started!");
}

XbotRosBridge::~XbotRosBridge(){}


void XbotRosBridge::xbotJointStateCB(const xbot_msgs::JointState::ConstPtr& msg){
    // create joint state msg
    sensor_msgs::JointState joint_state;

    // set the joint state pointer
    current_joint_state_ptr_->motor_position = msg->motor_position;

    // set timestamp
    joint_state.header.stamp = ros::Time::now();

    // set joint names
    joint_state.name = msg->name;

    // set joint poistions
    joint_state.position = std::vector<double>(msg->motor_position.begin(), msg->motor_position.end());

    // set joint velocities
    joint_state.velocity = std::vector<double>(msg->motor_velocity.begin(), msg->motor_velocity.end());

    // publish joint state
    ros_joint_state_pub_.publish(joint_state);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "xbot_ros_bridge");
    ros::NodeHandle nh;
    ROS_INFO("Starting XBot ROS Bridge...");
    XbotRosBridge xbot_ros_bridge(nh);

    ros::Rate rate(300);

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}