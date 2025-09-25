#!/usr/bin/env python3

import rospy
import rospkg
import actionlib
from repair_motion_controller.msg import RepairMoveToAction, RepairMoveToFeedback, RepairMoveToResult, RepairMoveToGoal
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from xbot_msgs.msg import JointCommand
from klampt.math import so3, se3

from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal


import time
import os

from threading import Thread

from repair_klampt_motion_planner import RepairMotionPlanner, home_joint_config


JOINT_NAMES = [
    'j_torso_base', 'j_torso_1', 
    'j_arm_1_1', 'j_arm_1_2', 'j_arm_1_3', 'j_arm_1_4', 'j_arm_1_5', 'j_arm_1_6', 'j_arm_1_7',
    'j_arm_2_1', 'j_arm_2_2', 'j_arm_2_3', 'j_arm_2_4', 'j_arm_2_5', 'j_arm_2_6', 'j_arm_2_7' 
    ]



class RepairMotionControlServer:
    def __init__(self, name):
        self._action_name = name
        
        self._current_joint_config_dict = {} 
        self._joint_names = JOINT_NAMES
        
        self.gazing = False

        # parameters
        enable_vis_param = rospy.search_param('enable_vis')
        self._is_vis_enable = rospy.get_param(enable_vis_param, True)
        self._robot = rospy.get_param(rospy.search_param('robot'), 'real')

        rospy.logwarn(f"{self._is_vis_enable}   |    {self._robot}")
        self._planner =  RepairMotionPlanner(show_vis=self._is_vis_enable)

        ## init action server of the node
        self._as = actionlib.SimpleActionServer(
                        self._action_name, 
                        RepairMoveToAction, 
                        execute_cb=self.action_executor, 
                        auto_start=False
                    )
        self._feedback = RepairMoveToFeedback()
        self._result = RepairMoveToResult()

        ## init FollowJointTrajectoryAction clients for controlling robot joints 
        self._fjt_client = actionlib.SimpleActionClient(
            '/robot_trajectory_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
            )

        ## Subscribers for updating robot's joint states
        self.robot_states_sub = rospy.Subscriber('/joint_states', JointState, self.update_current_joint_config)

        # xbot joint command publisher in set initial joint configuration
        self._xbot_cmd_pub = rospy.Publisher('/xbotcore/command', JointCommand, queue_size=10)

        # Publisher for real-robots left and right arm End-effector poses
        self.leftEE_pub = rospy.Publisher('/left_arm/end_effector', Pose, queue_size=10)
        self.rightEE_pub = rospy.Publisher('/right_arm/end_effector', Pose, queue_size=10)

        # timer for Klapmt vis actions
        self._timer = rospy.Timer(rospy.Duration(0.1), self.handle_vis_actions)


        # Start the action server
        self._as.start()
        rospy.loginfo(f"{self._action_name} is started")

        # set init robot config only when running with dummy. DO NOT DO IT WITH REAL ROBOT
        if self._robot == "dummy":
            self.set_xbot_init_joint_config()


    def set_xbot_init_joint_config(self):
        rospy.loginfo("Waiting for '/xbotcore' node...")
        while self._xbot_cmd_pub.get_num_connections() == 0:
            if rospy.is_shutdown():
                return
            rospy.sleep(0.1) # wait for 100ms before checking again
        
        rospy.loginfo("'/xbotcore' node is detected! Publishing initial joint configuration to '/xbotcore'...")
        msg = JointCommand()
        msg.name = [key for key, _ in home_joint_config.items()]
        msg.position = [val for _, val in home_joint_config.items()]
        msg.ctrl_mode = [1] * len(msg.position)
        self._xbot_cmd_pub.publish(msg)
        rospy.loginfo("Initial joint configuration is published to /xbotcore. Shutting down the 'xbotcore/command' publisher...")
        self._xbot_cmd_pub.unregister()
        rospy.loginfo("'xbotcore/command' publisher is destroyed.")


    def update_current_joint_config(self, joint_states:JointState):
        # get robot current joint configuration from robot_state publisher
        # set motion_planners real_robot configuration to received robot state
        self._planner.update_real_robot_joint_states(joint_states.position)
        
        # publish updated end-effector poses
        leftEE, rightEE = self._planner.get_realRobot_EEs()
        self.leftEE_pub.publish(leftEE)
        self.rightEE_pub.publish(rightEE)
        
        
    def action_executor(self, goal: RepairMoveToGoal):
        # Validate the selected arm for planning:
        rospy.loginfo("TESTINGS")
        if goal.arm == 0:
            status = "Goal is received for Left Arm."
            self.set_status_feedback(status)
            rospy.loginfo(status)
            # make right arm pose None
            goal.target_pose_right = None
        elif goal.arm == 1:
            status = "Goal is received  for Right Arm."
            self.set_status_feedback(status)
            rospy.loginfo(status)
            # make left arm pose None
            goal.target_pose_left = None
        elif goal.arm == 2:
            status = "Goal is received for Both Arms."
            self.set_status_feedback(status)
            rospy.loginfo(status)
        elif goal.arm == 88:
            status = "Moving Sand in Klampt"
            self.set_status_feedback(status)
            rospy.loginfo(status)
            # Call the move_to_home() method
            self.move_sand_x(goal.target_pose_left)
            status = "Removed Sand from klampt"
            self.set_status_feedback(status)
            rospy.logwarn(status)
            self._result.success = True
            self._as.set_succeeded(self._result)
            return  # Exit early since we are handling this special case
        elif goal.arm == 99:
            status = "Goal is received for Home position."
            self.set_status_feedback(status)
            rospy.loginfo(status)
            # Call the move_to_home() method
            self.move_to_home()
            return  # Exit early since we are handling this special case
        else:
            status = "Invalid Goal is received: goal.arm must be 0, 1, or 2"
            self.set_status_feedback(status)
            rospy.logerr(status)
            # Terminate goal execution
            self.terminate_planning()
            return
        
        

        print("\n################### START PLANNING ###################\n")

        # calulate the path to the goal from current robot config
        start_config = self._planner.planner_robot.getConfig()
        # path, traj = self.get_path_to_cartesian_goal(start_config, goal)

        try: 
            self.set_status_feedback("Start planning...")
            plan = self._planner.get_plan_to_cartesian_goal(goal.target_pose_left, goal.target_pose_right, gazing=self.gazing)
            self.set_status_feedback("Planning is successfully completed.")
            print("stats: ")
            print(plan.getStats())

        except (RuntimeError, ValueError) as e:
            rospy.logerr(f"{e}")
            self.set_status_feedback(f"{e}")
            self.terminate_planning()
            self._planner.planner_robot.setConfig(start_config)
            return
        
        # get the path from the plan
        path = plan.getPath()


        # get the statistics of the plan
        stats = plan.getStats()

        # get the joint trajectory
        traj_msg = self._planner.get_ros_joint_trajectory_from_plan(plan, goal.target_time, joint_update_rate=200)

        
        if traj_msg is None:
            status = "Faield to create the JointTrajectory for the plan."
            rospy.logwarn(status)
            self.set_status_feedback(f"{status}")
            self.terminate_planning()
            self._planner.planner_robot.setConfig(start_config)
            return

        status = "JointTrajectory for the plan is created!"
        self.set_status_feedback(f"{status}")
        rospy.loginfo(status)


        print("\n################### END PLANNING ###################\n")


        print("\n################### START PLAN EXECUTION ###################\n")

        # execute the motion in vis
        if self._is_vis_enable:
            def vis_traj_animate():
                interpolate_path = self._planner.get_interpolate_path(path, num_waypoints=100)
                self._planner.execute_planned_path_in_vis(interpolate_path)
            Thread(target=lambda: vis_traj_animate()).start()
            # vis_traj_animate()
        
        input("Press Enter to continue after visualization...")


        # move robot to goal
        status = "Moving robot to the goal config..."
        self.set_status_feedback(status)
        rospy.loginfo(status)

        if self.move_robot_to_goal(traj_msg):
            status = "Robot is moved to the goal config successfully."
            self.set_status_feedback(status)
            rospy.loginfo(status)
            rospy.loginfo(f"Plan stats: {stats}")
            self._result.success = True
            
            self._result.best_path_length = float(stats['bestPathLength'])
            self._result.num_waypoints = int(len(traj_msg.points))
            self._result.num_milestones = int(stats['numMilestones'])
            self._result.duration.data = traj_msg.points[-1].time_from_start
            
            self._as.set_succeeded(self._result)

        else:
            status = "Robot failed to reach the goal."
            self.set_status_feedback(status)
            rospy.logwarn(status)
            self._result.success = False
            self._as.set_succeeded(self._result)

        print("\n################### END PLAN EXECUTION ###################\n\n")


    def move_robot_to_goal(self, joint_trajectory):
        """ 
        Controls the robot with '/robot_trajectory_controller/follow_joint_trajectory' action client.
        """

        # warn if xbot_ros_bridge is not avaiable
        timeout = 5.0
        status = "Waiting for FollowJointTrajectory Action Server..."
        robot_controller_available = self._fjt_client.wait_for_server(rospy.Duration(timeout))

        # Log the FollowJointTrajectory servers availability
        if not robot_controller_available: 
            status = "Failed to find FollowJointTrajectory Action Server. Make sure to run the '/xbot_ros_bridge' node and 'xbotcore' with the correct robot config.yaml!"
            rospy.logwarn(status)
            self.set_status_feedback(status)
            self.terminate_motion_execution()
            return

        # Create a goal msg
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = joint_trajectory

        # Create a new trajectory with only the 0 index joint changing
        # first_trajectory = JointTrajectory()
        # first_trajectory.joint_names = joint_trajectory.joint_names
        # first_point = joint_trajectory.points[0]

        # for point in joint_trajectory.points:
        #     new_point = JointTrajectoryPoint()
        #     new_point.positions = list(first_point.positions)
        #     new_point.positions[0] = point.positions[0]  # Change only the 0 index joint
        #     # new_point.velocities = list(first_point.velocities) if first_point.velocities else []
        #     # new_point.accelerations = list(first_point.accelerations) if first_point.accelerations else []
        #     # new_point.effort = list(first_point.effort) if first_point.effort else []
        #     new_point.time_from_start = point.time_from_start
        #     first_trajectory.points.append(new_point)

        # Create a new trajectory with only the 0 index joint fixed at the last point's 0 index joint
        # second_trajectory = JointTrajectory()
        # second_trajectory.joint_names = joint_trajectory.joint_names
        # last_point = joint_trajectory.points[-1]

        # for point in joint_trajectory.points:
        #     new_point = JointTrajectoryPoint()
        #     new_point.positions = list(point.positions)
        #     new_point.positions[0] = last_point.positions[0]  # Fix the 0 index joint to the last point's 0 index joint
        #     # new_point.velocities = list(point.velocities) if point.velocities else []
        #     # new_point.accelerations = list(point.accelerations) if point.accelerations else []
        #     # new_point.effort = list(point.effort) if point.effort else []
        #     new_point.time_from_start = point.time_from_start
        #     second_trajectory.points.append(new_point)

     
        # # send the goal to the action servers for first trajectory
        # status = "Sending waypoints to the trajectory controllers..."
        # goal.trajectory = first_trajectory
        # self._fjt_client.send_goal(goal, feedback_cb=self.fjt_feedback_cb)

        # # wait for the result
        # self._fjt_client.wait_for_result()
        # result = self._fjt_client.get_result()
        
        # if result.error_code != 0:
            # send the goal to the action servers for second trajectory
        status = "Sending waypoints to the trajectory controllers..."
        # goal.trajectory = second_trajectory
        self._fjt_client.send_goal(goal, feedback_cb=self.fjt_feedback_cb)

        # wait for the result
        self._fjt_client.wait_for_result()
        result = self._fjt_client.get_result()

        return True if result.error_code == 0 else False
    
    
    def terminate_planning(self):
        status = "Terminating planning.."
        self.set_status_feedback(status)
        rospy.logwarn(status)
        self._result.success = False
        self._as.set_succeeded(self._result)
        status = "Planning is terminated."
        self.set_status_feedback(status)
        rospy.logwarn(status)

    def terminate_motion_execution(self):
        status = "Terminating motion execution.."
        self.set_status_feedback(status)
        rospy.logwarn(status)
        self._result.success = False
        self._as.set_succeeded(self._result)
        status = "Motion execution is terminated."
        self.set_status_feedback(status)
        rospy.logwarn(status)
    
    def set_status_feedback(self, status:str):
        self._feedback.status = status
        self._as.publish_feedback(self._feedback)


    def fjt_feedback_cb(self, feedback):
        rospy.logdebug(f"Feedback recieved: {feedback}")


    def validate_motion_to_target(self, pos_tol=0.01, ori_tol=0.01):
        current_left_tcp = self._planner.getEndEffectorPose_leftArm(self._planner.real_robot)
        current_right_tcp = self._planner.getEndEffectorPose_rightArm(self._planner.real_robot)
        target_left_tcp = self._planner.getEndEffectorPose_leftArm(self._planner.planner_robot)
        target_right_tcp = self._planner.getEndEffectorPose_rightArm(self._planner.planner_robot)

        pos_x_err_L = target_left_tcp.position.x - current_left_tcp.position.x
        pos_y_err_L = target_left_tcp.position.y - current_left_tcp.position.y
        pos_z_err_L = target_left_tcp.position.z - current_left_tcp.position.z
        ori_x_err_L = target_left_tcp.orientation.x - current_left_tcp.orientation.x
        ori_y_err_L = target_left_tcp.orientation.y - current_left_tcp.orientation.y
        ori_z_err_L = target_left_tcp.orientation.z - current_left_tcp.orientation.z
        ori_w_err_L = target_left_tcp.orientation.w - current_left_tcp.orientation.w
        pos_x_err_R = target_right_tcp.position.x - current_right_tcp.position.x
        pos_y_err_R = target_right_tcp.position.y - current_right_tcp.position.y
        pos_z_err_R = target_right_tcp.position.z - current_right_tcp.position.z
        ori_x_err_R = target_right_tcp.orientation.x - current_right_tcp.orientation.x
        ori_y_err_R = target_right_tcp.orientation.y - current_right_tcp.orientation.y
        ori_z_err_R = target_right_tcp.orientation.z - current_right_tcp.orientation.z
        ori_w_err_R = target_right_tcp.orientation.w - current_right_tcp.orientation.w

        pos_err_L = (pos_x_err_L, pos_y_err_L, pos_z_err_L)
        ori_err_L = (ori_x_err_L, ori_y_err_L, ori_z_err_L, ori_w_err_L)
        pos_err_R = (pos_x_err_R, pos_y_err_R, pos_z_err_R)
        ori_err_R = (ori_x_err_R, ori_y_err_R, ori_z_err_R, ori_w_err_R)

        # check tolerance for left arm
        if abs(pos_err_L[0]) > pos_tol or abs(pos_err_L[1]) > pos_tol or abs(pos_err_L[2]) > pos_tol or \
            abs(ori_err_L[0]) > ori_tol or abs(ori_err_L[1]) > ori_tol or abs(ori_err_L[2]) > ori_tol or abs(ori_err_L[3]) > ori_tol:
            return False, (pos_err_L, ori_err_L, pos_err_R, ori_err_R)
        
        # check tolerance for right arm
        if abs(pos_err_R[0]) > pos_tol or abs(pos_err_R[1]) > pos_tol or abs(pos_err_R[2]) > pos_tol or \
            abs(ori_err_R[0]) > ori_tol or abs(ori_err_R[1]) > ori_tol or abs(ori_err_R[2]) > ori_tol or abs(ori_err_R[3]) > ori_tol:
            return False, (pos_err_L, ori_err_L, pos_err_R, ori_err_R)
        return True, (pos_err_L, ori_err_L, pos_err_R, ori_err_R)


    
    def handle_vis_actions(self, event):
        if self._planner.moveToHome:
            self._planner.moveToHome = False
            self.move_to_home()
        
        if self._planner.resetGhost:
            self._planner.resetGhost = False
            self._planner.reset_ghost()
        
        if self._planner.moveToGhost:
            self._planner.moveToGhost = False
            self.move_to_ghost()

    def move_sand_x(self, pose):
        """Moves the terrain 'sand' by +dx meters along the x-axis."""
        self._planner.move_sand_x(pose)


    def move_to_home(self):

        print("\n################### START PLANNING ###################\n")

        start_config = self._planner.real_robot.getConfig()
        self._planner.planner_robot.setConfig(start_config)
        try:
            self.set_status_feedback("Start planning to Home...")
            goal_joint_config = [val for _, val in home_joint_config.items()]


            plan = self._planner.get_plan_to_joint_goal(target_joint_config=goal_joint_config)

            self.set_status_feedback("Planning is successfully completed.")
            print("stats: ")
            print(plan.getStats())

        except (RuntimeError, ValueError) as e:
            rospy.logerr(f"{e}")
            self.set_status_feedback(f"{e}")
            self.terminate_planning()
            self._planner.planner_robot.setConfig(start_config)
            return

        path = plan.getPath()
        stats = plan.getStats()

        traj_msg = self._planner.get_ros_joint_trajectory_from_plan(plan, 5.0, joint_update_rate=100)

        if traj_msg is None:
            status = "Faield to create the JointTrajectory for the plan."
            rospy.logwarn(status)
            self.set_status_feedback(f"{status}")
            self.terminate_planning()
            self._planner.planner_robot.setConfig(start_config)
            return
        
        status = "JointTrajectory for the plan is created!"
        self.set_status_feedback(f"{status}")
        rospy.loginfo(status)

        print("\n################### END PLANNING ###################\n")


        print("\n################### START PLAN EXECUTION ###################\n")

        # execute the motion in vis
        if self._is_vis_enable:
            def vis_traj_animate():
                interpolate_path = self._planner.get_interpolate_path(path, num_waypoints=100)
                self._planner.execute_planned_path_in_vis(interpolate_path)
            Thread(target=lambda: vis_traj_animate()).start()

        # move robot to goal
        status = "Moving robot to the goal config..."
        self.set_status_feedback(status)
        rospy.loginfo(status)

        if self.move_robot_to_goal(traj_msg):
            rospy.loginfo(f"Plan stats: {stats}")
            self._result.success = True

            self._result.best_path_length = float(stats['bestPathLength'])
            self._result.num_waypoints = int(len(traj_msg.points))
            self._result.num_milestones = int(stats['numMilestones'])
            self._result.duration.data = traj_msg.points[-1].time_from_start

            self._as.set_succeeded(self._result)

        else:
            status = "Robot failed to reach the goal."
            self.set_status_feedback(status)
            rospy.logwarn(status)
            self._result.success = False
            self._as.set_succeeded(self._result)
        
        print("\n################### END PLAN EXECUTION ###################\n\n")



    def move_to_ghost(self):
        goal_joint_config = self._planner.get_ghost_config()
        start_config = self._planner.real_robot.getConfig()
        print("\n################### START PLANNING ###################\n")

        try:
            self.set_status_feedback("Start planning to Home...")
            


            plan = self._planner.get_plan_to_joint_goal(target_robot_config=goal_joint_config)

            self.set_status_feedback("Planning is successfully completed.")
            print("stats: ")
            print(plan.getStats())

        except (RuntimeError, ValueError) as e:
            rospy.logerr(f"{e}")
            self.set_status_feedback(f"{e}")
            self.terminate_planning()
            self._planner.planner_robot.setConfig(start_config)
            return

        path = plan.getPath()
        stats = plan.getStats()

        traj_msg = self._planner.get_ros_joint_trajectory_from_plan(plan, 5.0, joint_update_rate=100)

        if traj_msg is None:
            status = "Faield to create the JointTrajectory for the plan."
            rospy.logwarn(status)
            self.set_status_feedback(f"{status}")
            self.terminate_planning()
            self._planner.planner_robot.setConfig(start_config)
            return
        
        status = "JointTrajectory for the plan is created!"
        self.set_status_feedback(f"{status}")
        rospy.loginfo(status)

        print("\n################### END PLANNING ###################\n")


        print("\n################### START PLAN EXECUTION ###################\n")

        # execute the motion in vis
        if self._is_vis_enable:
            def vis_traj_animate():
                interpolate_path = self._planner.get_interpolate_path(path, num_waypoints=100)
                self._planner.execute_planned_path_in_vis(interpolate_path)
            Thread(target=lambda: vis_traj_animate()).start()

        # move robot to goal
        status = "Moving robot to the goal config..."
        self.set_status_feedback(status)
        rospy.loginfo(status)

        if self.move_robot_to_goal(traj_msg):
            rospy.loginfo(f"Plan stats: {stats}")
            self._result.success = True

            self._result.best_path_length = float(stats['bestPathLength'])
            self._result.num_waypoints = int(len(traj_msg.points))
            self._result.num_milestones = int(stats['numMilestones'])
            self._result.duration.data = traj_msg.points[-1].time_from_start

            self._as.set_succeeded(self._result)


        else:
            status = "Robot failed to reach the goal."
            self.set_status_feedback(status)
            rospy.logwarn(status)
            self._result.success = False
            self._as.set_succeeded(self._result)
        
        print("\n################### END PLAN EXECUTION ###################\n\n")















# ========================================================================================================


if __name__ == '__main__':
    rospy.init_node('repair_motion_controller', anonymous=False)
    server = RepairMotionControlServer(rospy.get_name())
    rospy.spin()

