import time
import numpy as np
from klampt import *
from klampt import vis
from IPython.display import clear_output
from klampt.vis.ipython import Playback
from klampt.model import ik
from klampt.plan.robotplanning import plan_to_config

import random
import klampt
from klampt.plan.cspaceutils import *
from klampt.plan.cspace import *
from klampt.plan.robotcspace import RobotCSpace
from klampt.model.collide import WorldCollider
from klampt.model.trajectory import *
from klampt.model import collide, ik, create
from klampt.math import so3, se3
import copy
# from repair_klampt_motion_planner import rotation_matrix_to_euler, euler_to_rotation_matrix
import os
import yaml

import rospy
import rospkg
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from threading import Thread
import tf2_ros
import geometry_msgs

vis.init("GLUT")


rospack = rospkg.RosPack()
pkg_path = rospack.get_path('repair_motion_controller')
home_joint_config_path = f"{pkg_path}/config/joint_config/home_joint_config.yaml"
with open(home_joint_config_path, 'r') as file:
    home_joint_config = yaml.safe_load(file)['home_joint_config']



# ===================== CONFIGURATIONS START ============================================================================

# WORLD PATH
WORLD_PATH = (os.path.join(os.path.dirname(__file__), "../robot_description/klampt_world.xml"))

# INITIAL ROBOT CONFIGURATION
INITIAL_JOINT_POSITIONS = [ val for _, val in home_joint_config.items()]

# END EFFECTORS
LEFT_ARM_EE_LINK = "left_hand_v1_wide_grasp_link"
LEFT_ARM_CAMERA_LINK = "arm_1_camera_mount"
RIGHT_ARM_EE_LINK = "right_hand_v1_2_research_grasp_link"
RIGHT_ARM_CAMERA_LINK = "arm_2_camera_mount"

# VISUALIZE
VIS_UPDATE_RATE = 100  # Hz

# PLANNER
NUM_WAY_POINTS = 1000
PLANNING_TIME_LIMIT = 5.0
NUM_IK_TRIES = 100
MAX_PLANNER_ITERS = 500
MAX_PLANNER_TIME = 10.0
EDGE_CHECK_RESOLUTION = 0.01
IS_PLANNER_OPTIMIZING = True
SIMPLIFY_TYPE = "ConvexHull"

PLANNER_SETTINGS_SBL = {  # SBL planner.
    "type": "sbl",
    "perturbationRadius": 0.25,
    "bidirectional": True,
    "shortcut": 1,
    "restart": 1,
    "restartTermCond": "{foundSolution:1,maxIters:1000}",
}

PLANNER_SETTINGS_RRT = {  # RRT planner.
    "type": "rrt",
    "perturbationRadius": 0.25,
    "bidirectional": True,
    "shortcut": True,
    "restart": True,
    "restartTermCond": "{foundSolution:1,maxIters:1000}",
}

# CAMERA
CAMERA_DIST = 4  # Distance from the target
CAMERA_ROT = [0.0, 0.0, 1.57, 1]  # Orientation as a quaternion (w, x, y, z)
CAMERA_TGT = [0.0, 0.0, 1.5]  # Target position (where the camera is looking)
CAMERA_POS = [3, 2, 1]  # Camera position

# ===================== CONFIGURATIONS END ============================================================================

def get_link_index_by_name(robot, link_name):
    for i in range(robot.numLinks()):
        if robot.link(i).getName() == link_name:
            return i
    raise ValueError(f"Link '{link_name}' not found")

class RepairMotionPlanner:
    def __init__(self, show_vis=True):
        # flags
        self.show_vis = show_vis

        self.moveToHome = False
        self.moveToGhost = False
        self.resetGhost = False

        # load robot for planning
        self.world = WorldModel()
        self.world.loadFile(WORLD_PATH)
        self.planner_robot = self.world.robot(0) 
        self.sand_plane = self.add_sand_to_env()

        # load real_robot for visualizing real robot states
        self.__world_for_real_robot = WorldModel()
        self.__world_for_real_robot.loadFile(WORLD_PATH)
        self.real_robot = self.__world_for_real_robot.robot(0)

        
        # robot drivers
        self._robot_drivers = [self.planner_robot.driver(i) for i in range(self.planner_robot.numDrivers())]
        self._sliding_guide_driver = self._robot_drivers[0]
        self._torso_driver = self._robot_drivers[1]
        self._left_arm_drivers = self._robot_drivers[2:9]
        self._right_arm_drivers = self._robot_drivers[9:]

        # End Effectors
        self.endEffector_leftArm = self.planner_robot.link(LEFT_ARM_EE_LINK)
        self.endEffector_rightArm = self.planner_robot.link(RIGHT_ARM_EE_LINK)

        # 'configure robot's joints to initial configuration
        self.set_initial_joint_positions(self.planner_robot)
        self.set_initial_joint_positions(self.real_robot)

        # Retrieve link indices by name
        link_1_7_index = get_link_index_by_name(self.planner_robot, "arm_1_7")
        link_1_5_index = get_link_index_by_name(self.planner_robot, "arm_1_5")
        link_1_flange_index = get_link_index_by_name(self.planner_robot, "arm_1_angle_flange")
        # Disable self-collisions between specified link pairs
        self.planner_robot.enableSelfCollision(link_1_7_index, link_1_5_index, False)
        self.planner_robot.enableSelfCollision(link_1_flange_index, link_1_5_index, False)
        # initialize collisions
        self.ignore_sand = False
        self.__collider = WorldCollider(self.world)
        self.__cspace = RobotCSpace(self.planner_robot, self.__collider)
        self.__cspace.eps = 1e-2
        self.__init_robot_collision()

        self.__set_robot_color()
        if self.show_vis:
            self.__visualize()
        #self.print_robot_info()
        self.__loginfo("RepairMotionPlanner is initialized")

    def print_robot_info(self):
        print("Robot info:")
        print("  Name:", self.planner_robot.getName())
        print("  Links:", self.planner_robot.numLinks())
        print("  Link names:")
        for i in range(self.planner_robot.numLinks()):
            print(f"    {i}: {self.planner_robot.link(i).getName()}")
        print("  Joints:", self.planner_robot.numDrivers())
        print("  Joint names:")
        for i in range(self.planner_robot.numDrivers()):
            print("    ", self.planner_robot.driver(i).getName())


    def __loginfo(self, info):
        print(f"[INFO] [{rospy.Time.now().secs}.{rospy.Time.now().nsecs}]: {info}")

    def __logwarn(self, info):
        print(f"\033[93m[WARN] [{rospy.Time.now().secs}.{rospy.Time.now().nsecs}]: {info}\033[0m")

    def __logerr(self, info):
        print(f"\033[91m[ERROR] [{rospy.Time.now().secs}.{rospy.Time.now().nsecs}]: {info}\033[0m")
      
    
    def on_moveToHomePose(self):
                print("moveToHomePose clicked!")
                self.moveToHome = True
    
    def on_moveToGhostPose(self):
                print("moveToGhostPose clicked!")
                self.moveToGhost = True

    def on_resetGhostPose(self):
                print("moveToGhostPose clicked!")
                self.resetGhost = True

    def update_robot_cspace_and_colliders(self):
        """
        Updates the robot's configuration space and collision settings.
        """
        self.__collider = WorldCollider(self.world)
        self.__cspace = RobotCSpace(self.planner_robot, self.__collider)
        self.__cspace.eps = 1e-2
        self.__init_robot_collision()


    def __visualize(self):
        if self.show_vis:
            # Configure Camera View
            viewport = vis.getViewport()  # Obtain the current viewport

            # Modify the camera parameters
            # viewport.camera.dist = CAMERA_DIST
            # viewport.camera.rot = CAMERA_ROT
            # viewport.camera.tgt = CAMERA_TGT
            # viewport.camera.pos = CAMERA_POS
            # vis.setViewport(viewport)  # Apply the modified viewport
            
            print("real_robot:", self.real_robot)
            print("ghost config:", self.world.robot(0).getConfig())
            
            print("World contents:")
            print("  Robots:", self.world.numRobots())
            print("  Rigid objects:", self.world.numRigidObjects())
            print("  Terrains:", self.world.numTerrains())

            vis.kill()     # Force close any existing window
            vis.clear()    # Clear all previous items
            
            # got some errors here with viewport etc. this config here seemed to work but might have issues
            robot = self.world.robot(0)
            print("Now setting cool 1")
            robot.setConfig(self.real_robot.getConfig())
            
            vis.add("world", self.world)
            vis.add("real_robot", robot, color = [0.5,0.5,0.5,1]) # use to show the real robot's state
            vis.add("ghost", self.world.robot(0).getConfig(), color = (0,1,0,0.5))
            vis.edit("ghost")

            vis.addAction(self.on_moveToHomePose, "MoveToHomePose")
            vis.addAction(self.on_moveToGhostPose, "MoveToGhostPose")
            vis.addAction(self.on_resetGhostPose, "ResetGhostPose")

            vis.show()
            #

            self.update_vis_thread = Thread(target=self.__update_vis_task)
            self.update_vis_thread.start()

    def __update_vis_task(self):
        while vis.shown():
            vis.lock()
            vis.setItemConfig("ghost", self.planner_robot.getConfig())
            # self.planner_robot.setConfig(self.planner_robot.getConfig()) # use to show the planned target configuration
            #self.real_robot.setConfig(self.planner_robot.getConfig()) #TODO ?! wth
            vis.unlock()
            # vis.addText("RB1", "%.2f" % (time.time(),), position=(0, 20))
            time.sleep(1 / VIS_UPDATE_RATE)

    def __set_robot_color(self, color=[1, 1, 0, 0.5]):
        for i in range(self.planner_robot.numLinks()): 
            link = self.planner_robot.link(i) 
            appearance = link.appearance() 
            appearance.setColor(color[0], color[1], color[2], color[3])

    def reset_ghost(self):
        self.planner_robot.setConfig(self.real_robot.getConfig())
        self.__loginfo("Ghost pose has been reset to the current robot pose.")

    def get_ghost_config(self):
        return vis.getItemConfig("ghost")



    def execute_planned_path_in_vis(self, path):
        """  
        When vis is enabled, this will show the planned motion in vis for given the interpolate path.
        """
        for milestone in path:
            vis.setItemConfig("ghost", milestone)
            # self.planner_robot.setConfig(milestone)
            time.sleep(1 / len(path))  # Wait for visualization update

    def update_real_robot_joint_states(self, joint_configs:List):
        """  
        Given joint joint_configs of the robot as a list, 
        this function update the real robot configuration in the vis.

        Parameters:
            joint_configs (list(float)): A list containing joint configurations of all 16 joints in the order of
             [j_torso_base, j_torso_1, j_arm_1_1, j_arm_1_2, j_arm_1_3, j_arm_1_4, j_arm_1_5, j_arm_1_6, j_arm_1_7, j_arm_2_1, j_arm_2_2, j_arm_2_3, j_arm_2_4, j_arm_2_5, j_arm_2_6, j_arm_2_7]

        """
        # validate number of elements in joint_states
        if len(joint_configs) != 16:
            raise ValueError("length of joint_states should be 16.")

        for index, val in enumerate(joint_configs) :
            self.real_robot.driver(index).setValue(val)


    def get_robot_drivers(self):
        """
        Returns a list of `RobotModelDrivers` of the robot.

        Returns:
           list[RobotModelDriver]: A list containing `RobotModelDriver`s of the robot.
        """
        return[self.planner_robot.driver(i) for i in range(self.robot.numDrivers())]
    
    def get_robot_drivers_link_indices(self) -> List[int]:
        """  
        Retrieves the indices of all links driven by the robot's drivers.

        Returns:
           list[int]: A list containing the indices of all links controlled by the robot's drivers.
        """
        driver_indices = []
        for drvId in range(self.planner_robot.numDrivers()):
            linkName = self.planner_robot.driver(drvId).getName()
            linkIndex = self.planner_robot.link(linkName).index
            driver_indices.append(linkIndex)
        return driver_indices
    
    def get_robot_drivers_velocity_limits(self):
        """  
        Retrieves the velocity limits of the joints controlled by the robot's drivers. 
        
        Returns:
            list[(float, float)]: A list of velocity limits of the joints.
        """
        vel_limits = []
        for i in range(self.planner_robot.numDrivers()):
            vel_limits.append(self.planner_robot.driver(i).getVelocityLimits())
        return vel_limits

    def getJointValues(self, robot:RobotModel):
        """  
        Retrieves the current values of the joints controlled by the robot's drivers. 
        
        Returns:
            list[Float]: A list of the current values of the joints controlled by the robot's drivers.
        """
        drivers = self.get_robot_drivers()
        return [drv.getValue() for drv in drivers]

    def getJointLimits(self):
        robot_drivers = self.get_robot_drivers()
        return [drv.getLimits() for drv in robot_drivers]

    def setJointValues(self, robot:RobotModel, vals):
        for i in range(robot.numDrivers()):
            robot.driver(i).setValue(vals[i])
        robot.setConfig(self.planner_robot.getConfig())

    def getEndEffectorPose_leftArm(self, robot:RobotModel) -> Pose:
        """ 
        Returns the current End Effector Pose (`position`, `orientation`) of the Left Arm.

        Parameters:
            robot (RobotModel): The robot model from which the end effector pose is to be retrieved.
                - Options: `self.planner_robot` or `self.real_robot`
    
        Returns:
            Pose: A `Pose` object containing the current position and orientation of the left arm's end effector.
        """
        R, t = robot.link(LEFT_ARM_EE_LINK).getTransform()
        quat = so3.quaternion(R)
        # swap z and w
        pose = Pose()
        pose.orientation.w = quat[0]
        pose.orientation.x = quat[1]
        pose.orientation.y = quat[2]
        pose.orientation.z = quat[3]
        pose.position.x = t[0]
        pose.position.y = t[1]
        pose.position.z = t[2]
        return pose 
    
    def getEndEffectorPose_rightArm(self, robot:RobotModel) -> Pose:
        """ 
        Returns the current End Effector Pose (`position`, `orientation`) of the Right Arm.

        Parameters:
            robot (RobotModel): The robot model from which the end effector pose is to be retrieved.
                - Options: `self.planner_robot` or `self.real_robot`
    
        Returns:
            Pose: A `Pose` object containing the current position and orientation of the right arm's end effector.
        """
        R, t = robot.link(RIGHT_ARM_EE_LINK).getTransform()
        quat = so3.quaternion(R)
        # swap z and w
        pose = Pose()
        pose.orientation.w = quat[0]
        pose.orientation.x = quat[1]
        pose.orientation.y = quat[2]
        pose.orientation.z = quat[3]  
        pose.position.x = t[0]
        pose.position.y = t[1]
        pose.position.z = t[2]
        return pose
    
    def get_realRobot_EEs(self):
        """ Returns left and right arm End-effector poses of the real robot. """
        leftEE = self.getEndEffectorPose_leftArm(self.real_robot)
        rightEE = self.getEndEffectorPose_rightArm(self.real_robot)
        return leftEE, rightEE
    
    # def print_robot_info(self):
    #     """ 
    #     Prints the links and joint information of the robot.
    #     """
    #     self.__loginfo("Robot info:")
    #     self.__loginfo("\t  Name:", self.planner_robot.getName())
    #     self.__loginfo("\t  Links:", self.planner_robot.numLinks())
    #     self.__loginfo("\t  Link names:")
    #     for i in range(self.planner_robot.numLinks()):
    #         self.__loginfo(f"\t    {i}: {self.planner_robot.link(i).getName()}")
    #     self.__loginfo("\t  Joints:", self.planner_robot.numDrivers())
    #     self.__loginfo("\t  Joint names:")
    #     for i in range(self.planner_robot.numDrivers()):
    #         self.__loginfo("\t    ", self.planner_robot.driver(i).getName(), self.planner_robot.driver(i).getLimits())
    
    def filterJointConfigFromFullConfig(self, full_config):
        """
        Filters the joint configuration values from a full configuration array.

        This function extracts the values of the joints controlled by the robot's drivers
        from the full configuration array. It uses the indices of the joints to retrieve
        their corresponding values from the full configuration.
        
        Parameters:
            full_config (list): The full configuration array containing values for all joints of the robot.
        
        Returns:
            list: A list of the values for the joints controlled by the robot's drivers.
        """
        driver_values = []
        joint_ids = self.get_robot_drivers_link_indices()
        for joint_id in joint_ids:
            driver_values.append(full_config[joint_id])
        return driver_values
    
    def filter_path(self, path):
        """
        Filters the configuration values of the robot's joints (drivers) from a given path.

        Parameters:
            path (list): A list of configuration arrays, where each array represents the state of all joints at a specific time step.
        
        Returns:
            list: A list of filtered configuration arrays, containing only the values for the joints controlled by the robot's drivers.
        """
        filtered_path = []
        joint_ids = self.get_robot_drivers_link_indices()
        for i in range(len(path)):
            filtered_config = [ path[i][joint_id] for joint_id in joint_ids ]
            filtered_path.append(filtered_config)
        return filtered_path
    
    def set_initial_joint_positions(self, robot:RobotModel):
        """  
        Sets the initial configuration to a given `RobotModel`.

        Parameters:
            robot (RobotModel): `self.planner_robot` or `self.real_robot`.
        """
        self.setJointValues(robot, INITIAL_JOINT_POSITIONS)

    def __init_robot_collision(self):
        """
        Initializes the robot's collision detection and handling.

        This function sets up collision tests and ignores specific collisions for the robot's drivers. 
        It identifies collisions between different parts of the robot and adds them to a set of collisions 
        to be ignored. It also explicitly ignores collisions between certain predefined links of the robot. 
        Any detected collisions are logged as warnings.

        Steps:
            1. Iterate through collision tests and add colliding objects to a set to be ignored.
            2. Ignore collisions for the collected set.
            3. Disable self-collision for specific robot links.
            4. Log warnings for any detected collisions.
            5. Print an info message if self-collisions are detected.

        Note:
            This function assumes the existence of methods `collisionTests()`, `ignoreCollision()`, 
            and `enableSelfCollision()` provided by the `self.__collider` and `self.planner_robot` objects.
        """
        ignore_coll_sets = set()
        for i, j in self.__collider.collisionTests():
            if i[1].collides(j[1]):
                ignore_coll_sets.add((i[0], j[0]))

        for ignore_set in ignore_coll_sets:
            self.__collider.ignoreCollision(ignore_set)
            if (type(ignore_set[0])) == klampt.robotsim.RobotModelLink and \
            (type(ignore_set[1])) == klampt.robotsim.RobotModelLink:
                self.planner_robot.enableSelfCollision(ignore_set[0].getIndex(), ignore_set[1].getIndex(), False)
        
        # ignore collision between left_inner_finger_pad and right_inner_finger_pad

        if self.ignore_sand:
            terrain_index = self.world.index(self.world.terrain("Sand"))

            for i in range(self.world.numTerrains() + self.world.numRigidObjects() + self.world.numRobots()):
                if i != terrain_index:
                    self.__collider.ignoreCollision(i, terrain_index)

        
        self.__collider.ignoreCollision(
            (
                self.planner_robot.link("torso_1"),
                self.planner_robot.link("sliding_guide_link"),
            )
        )
        self.planner_robot.enableSelfCollision(
            self.planner_robot.link("torso_1").getIndex(),
            self.planner_robot.link("sliding_guide_link").getIndex(),
            False,
        )

        for i, j in self.__collider.collisionTests():
            if i[1].collides(j[1]):
                rospy.logwarn(" - Object", i[0].getName(), "collides with", j[0].getName())

        if self.planner_robot.selfCollides():
            self.__logwarn("Robot self collision.")
    

    # Helper method to log planner statistics
    def _log_planner_stats(self, plan, start_time, num_iters):
        self.__loginfo(f"Planning time: {time.time() - start_time}s over {num_iters} iterations")
        V, E = plan.getRoadmap()
        self.__loginfo(f"{len(V)} feasible milestones sampled, {len(E)} edges connected")
        self.__loginfo(f"Planner stats: {plan.getStats()}")

    # Helper method to handle a failed plan
    def _handle_failed_plan(self, verbose, plan):
        self.__loginfo("[INFO] [RepairMotionPlanner]: Failed to plan a feasible path")
        if verbose >= 1:
            self.__loginfo(f"Planner stats: {plan.getStats()}")
        if verbose >= 2:
            V, _ = plan.getRoadmap()
            self.__loginfo("Some sampled configurations:")
            self.__loginfo(f"\t {V[:min(10, len(V))]}")
        

    def get_ik_objective(self, link, goal_pos, goal_rot) -> IKObjective:
        """
        Create an `IKObjective` for a specified link with given goal position and rotation.

        Parameters:
            link: The link of the robot for which the IK objective is to be created.
            goal_pos: A list or tuple representing the target position (x, y, z) for the link.
            goal_rot: A list or tuple representing the target rotation matrix (3x3) for the link.

        Returns:
            ik.Objective: The `IKObjective` with the specified goal position and rotation.
        """
        return ik.objective(link, ref=None, R=goal_rot, t=goal_pos)

    # def find_ik_fast(
    #         self, 
    #         objectives: List[IKObjective], 
    #         num_tries: int = 5,
    #         moveable_subset=[],
    #         oneshot=True,
    #         solve_nearby=True
    #     ) ->Tuple[List[float], Pose, Pose, str]:
    #     """  
    #     Attempts to solve the inverse kinematics (IK) problem for the given objectives and start configuration.
    #     Returns the goal configuration, TCP poses for the left and right arms, and information about success.

    #     Parameters:
    #         objectives (List[object]): List of IK objectives for the solver.
    #         num_tries (int, optional): Number of tries with increasing tolerance (default is 5).

    #     Returns:
    #         Tuple ([List[float], Pose, Pose, str]):
    #         A tuple containing:
    #         - goal_config (List[float]): The resulting configuration if IK succeeds.
    #         - tcp_left_pose (Pose): The TCP pose of the left arm as a `geometry_msgs/Pose` object.
    #         - tcp_right_pose (Pose): The TCP pose of the right arm as a `geometry_msgs/Pose` object.
    #         - info (str): Information about the IK process, indicating success or failure.

    #     Raises:
    #         RuntimeError: If IK fails after the specified number of tries.
    #     """
    #     solver = IKSolver(self.planner_robot)
        
    #     # can be used to plot all links, care spam
    #     # for i in range(self.planner_robot.numLinks()):
    #     #     link = self.planner_robot.link(i)
    #     #     print(f"[{i}] Link name: {link.getName()}")
        
    #     if(moveable_subset != []):
    #         #print(f"Adding subset {moveable_subset}")
    #         solver.setActiveDofs(moveable_subset)
        
    #     for obj in objectives:
    #         solver.add(obj)
    #     # try to solve IK multiple times to get a good solution
    #     # vary the tolerance each time
    #     info = ""
    #     res = False
    #     for i in range(num_tries):
    #         solver.setTolerance(1e-4 * 10 * i)
    #         res = solver.solve()
    #         if res:
    #             info = f"IK succeeded after {i + 1} tries with tolerance {solver.getTolerance()}"
    #             break

    #     if not res:
    #         error_msg = f"IK failed after {num_tries} attempts. Target pose(s) might be invalid. \nResidual: {solver.getResidual()}"
    #         if(oneshot):
    #             raise RuntimeError(error_msg)
    #         else:
    #             return None, None, None, "False"

    #     goal_config = self.planner_robot.getConfig()
    #     # get tcp poses
    #     tcp_left_pose = self.getEndEffectorPose_leftArm(self.planner_robot)
    #     tcp_right_pose = self.getEndEffectorPose_rightArm(self.planner_robot)
        
    #     return goal_config, tcp_left_pose, tcp_right_pose, info


    def find_ik_fast(
            self, 
            objectives: List[IKObjective], 
            num_tries: int = 5,
            moveable_subset=[],
            oneshot=True,
            solve_nearby=True
        ) -> Tuple[List[float], Pose, Pose, str]:
        """  
        Attempts to solve IK quickly with some robustness:
        - multiple randomized seeds
        - adaptive tolerance schedule
        - feasibility filtering
        - chooses solution closest to start configuration
        """

        solver = IKSolver(self.planner_robot)
        solver.setMaxIters(2000)

        if moveable_subset:
            solver.setActiveDofs(moveable_subset)

        for obj in objectives:
            solver.add(obj)

        init_config = self.planner_robot.getConfig()
        best_conf, best_dist = None, float("inf")
        info = ""

        # tolerance schedule, from strict to looser
        tolerances = [1e-4, 5e-4, 1e-3, 5e-3, 1e-2]
        factor = 1.2
        maxDeviation=0.3
        for i in range(num_tries):
            # use either init config or a perturbed one
            # if i == 0:
            #     self.planner_robot.setConfig(init_config)
            # else:
            #     qrand = [qi + random.uniform(-0.05, 0.05) for qi in init_config]
            #     self.planner_robot.setConfig(qrand)

            # tol = tolerances[min(i, len(tolerances)-1)]
            solver.setTolerance(1e-4)
            dofs = solver.getActiveDofs()
            q = self.planner_robot.getConfig()
            qmin,qmax = self.planner_robot.getJointLimits()
            for d in dofs:
                qmin[d] = max(qmin[d],q[d]-maxDeviation)
                qmax[d] = min(qmax[d],q[d]+maxDeviation)
            solver.setJointLimits(qmin,qmax)
            solver.setBiasConfig(q)

            # try solve() first, fall back to solve_nearby if requested
            success = solver.solve()
            # if not success and solve_nearby:
            #     success = ik.solve_nearby(
            #         objectives,
            #         maxDeviation=0.15,
            #         iters=200,
            #         tol=1e-3,
            #         feasibilityCheck=self.is_feasible,
            #         numRestarts=2,
            #         activeDofs=moveable_subset if moveable_subset else None,
            #     )
            maxDeviation*= factor
            if success and self.is_feasible():
                qsol = self.planner_robot.getConfig()
                dist = vectorops.distance(qsol, init_config)
                if dist < best_dist:
                    best_conf, best_dist = qsol, dist
                    info = f"IK succeeded on try {i+1} with tolerance 1e-3"

        if best_conf is None:
            error_msg = f"IK failed after {num_tries} attempts. Residual: {solver.getResidual()}"
            if oneshot:
                raise RuntimeError(error_msg)
            else:
                return None, None, None, "False"

        # set robot to the best found config
        self.planner_robot.setConfig(best_conf)

        # get tcp poses
        tcp_left_pose = self.getEndEffectorPose_leftArm(self.planner_robot)
        tcp_right_pose = self.getEndEffectorPose_rightArm(self.planner_robot)

        return best_conf, tcp_left_pose, tcp_right_pose, info

    def is_feasible(self):
        # return True only if *no* self‐collisions are present
        geoms = [g for _,g in self.__collider.geomList]
        # pairs to test are given by the mask
        pairs = [(i,j) for i in range(len(self.__collider.geomList))
                        for j in self.__collider.mask[i] if i<j]
        return not any(collide.self_collision_iter(geoms, pairs))

    def find_ik(
            self, 
            objectives: List[IKObjective], 
            num_tries: int = 5,
            moveable_subset=[],
            oneshot=True,
            solve_nearby=True
        ) ->Tuple[List[float], Pose, Pose, str]:
        """  
        Attempts to solve the inverse kinematics (IK) problem for the given objectives and start configuration.
        Returns the goal configuration, TCP poses for the left and right arms, and information about success.

        Parameters:
            objectives (List[object]): List of IK objectives for the solver.
            num_tries (int, optional): Number of tries with increasing tolerance (default is 5).

        Returns:
            Tuple ([List[float], Pose, Pose, str]):
            A tuple containing:
            - goal_config (List[float]): The resulting configuration if IK succeeds.
            - tcp_left_pose (Pose): The TCP pose of the left arm as a `geometry_msgs/Pose` object.
            - tcp_right_pose (Pose): The TCP pose of the right arm as a `geometry_msgs/Pose` object.
            - info (str): Information about the IK process, indicating success or failure.

        Raises:
            RuntimeError: If IK fails after the specified number of tries.
        """
        goal_config, tcp_left_pose, tcp_right_pose, info = self.find_ik_fast(objectives, num_tries, moveable_subset, oneshot, solve_nearby)
        if goal_config is None:
            return goal_config, tcp_left_pose, tcp_right_pose, info
        return goal_config, tcp_left_pose, tcp_right_pose, info
        active_dofs = moveable_subset if(moveable_subset != []) else None
        solutions = []
        success = False
        deviation = 0.3
        ik_max_iter = 0
        init_config = self.planner_robot.getConfig()
        for n in range(num_tries):
            while not success and ik_max_iter <= 50:
                ik_max_iter +=1  
                self.planner_robot.setConfig(init_config)
                if solve_nearby:
                    success = ik.solve_nearby(
                        objectives,
                        maxDeviation=deviation,
                        iters=2000,
                        tol=1e-3,
                        activeDofs = active_dofs,
                        feasibilityCheck=self.is_feasible,
                        numRestarts=5,      # nearby: stick to current seed and small radius
                    )
                    deviation *= 1.2
                else:
                    success = ik.solve_global(objectives, 
                                        iters=500,           # increase per‑trial iterations
                                        tol=1e-2,             # tighter convergence
                                        numRestarts=1,       # more random restarts
                                        feasibilityCheck=self.is_feasible,
                                        activeDofs = active_dofs,
                                        startRandom=False      # randomize even the first guess
                                        )
                
                if success:
                    solutions.append(self.planner_robot.getConfig())
            print("FOUND ik solution at iteration: ", ik_max_iter)

        final_solutions = []
        if solutions != []:
            for conf in solutions:
                if not self.is_robot_config_feasible(conf):
                    final_solutions.append(conf)
        else:
            print("No Valid IK found")
            return None, None, None, "False"
        
        goal_config = min(solutions, key=lambda q: vectorops.distance(q, init_config))
        self.planner_robot.setConfig(goal_config)

        # get tcp poses
        tcp_left_pose = self.getEndEffectorPose_leftArm(self.planner_robot)
        tcp_right_pose = self.getEndEffectorPose_rightArm(self.planner_robot)
        #self.planner_robot.setConfig(init_config)

        return goal_config, tcp_left_pose, tcp_right_pose, "True"
       


    def get_robot_goal_config_old(self, left_arm_target_pose=None, right_arm_target_pose=None, gazing=True):
        """
        Failsafe in case gazing or avoidance does not work as assumed.
        Sets the target pose for the left and/or right arm.

        Parameters:
            target_pose_left (geometry_msgs/Pose, optional): The target pose for the left arm. Default is None.
            target_pose_right (geometry_msgs/Pose, optional): The target pose for the right arm. Default is None.

        Returns:
            Tuple ([List[float], Pose, Pose, str]):
            A tuple containing:
            - goal_config (List[float]): The resulting configuration if IK succeeds.
            - tcp_left_pose (Pose): The TCP pose of the left arm as a `geometry_msgs/Pose` object.
            - tcp_right_pose (Pose): The TCP pose of the right arm as a `geometry_msgs/Pose` object.
            - info (str): Information about the IK process, indicating success or failure.


        Raises:
            ValueError: If neither `target_pose_left` nor `target_pose_right` is provided.
            RuntimeError: If IK fails after the specified number of tries.

        """
        if left_arm_target_pose is None and right_arm_target_pose is None:
            raise ValueError("At least one of `target_pose_left` or `target_pose_right` must be provided")

        ik_objs = []

        # remove vis goal labels if exists
        if self.show_vis:
            try:
                vis.remove("LeftArm IK goal")
            except:
                pass

            try:
                vis.remove("RightArm IK goal")
            except:
                pass

        # Get Ik Objective for Left Arm
        if left_arm_target_pose is not None:
            # Select the end effector link.     
            leftEE_link = self.endEffector_leftArm
            # Get current positions and orientation.
            currentTransform_leftEE = leftEE_link.getTransform()
            currentPose_leftEE, currentOrientation_leftEE = currentTransform_leftEE[1], so3.quaternion(currentTransform_leftEE[0])
            # Determine goal orientation.
            # If goal orientation is provided use that, otherwise use current orientation as goal orientation.
            left_goal_rot = (
                so3.from_quaternion((left_arm_target_pose.orientation.w, left_arm_target_pose.orientation.x, left_arm_target_pose.orientation.y, left_arm_target_pose.orientation.z))
                if left_arm_target_pose.orientation else currentOrientation_leftEE)
            # get ik objective
            position = [left_arm_target_pose.position.x, left_arm_target_pose.position.y, left_arm_target_pose.position.z]
            ik_objective_left = self.get_ik_objective(leftEE_link, position, left_goal_rot)
            ik_objs.append(ik_objective_left)
            if self.show_vis:
                vis.add("LeftArm IK goal",ik_objective_left)
      
        # Get Ik Objective for Right Arm
        if right_arm_target_pose is not None:
            # Select the end effector link.
            rightEE_link = self.endEffector_rightArm
            # Get current positions and orientation.
            currentTransform_rightEE = rightEE_link.getTransform()
            currentPose_rightEE, currentOrientation_rightEE = currentTransform_rightEE[1], so3.quaternion(currentTransform_rightEE[0])
            # Determine goal orientation.
            # If goal orientation is provided use that, otherwise use current orientation as goal orientation.
            right_goal_rot = (
                so3.from_quaternion((right_arm_target_pose.orientation.w, right_arm_target_pose.orientation.x, right_arm_target_pose.orientation.y, right_arm_target_pose.orientation.z))
                if right_arm_target_pose.orientation else currentOrientation_rightEE)
            # get ik objective
            position = [right_arm_target_pose.position.x, right_arm_target_pose.position.y, right_arm_target_pose.position.z]
            ik_objective_right = self.get_ik_objective(rightEE_link, position, right_goal_rot)
            ik_objs.append(ik_objective_right)
            if self.show_vis:
                vis.add("RightArm IK goal",ik_objective_right) 

        # try to get the goal configuration solving IK. 
        try:
            goal_config, left_tcp, right_tcp, info = self.find_ik(ik_objs, num_tries = NUM_IK_TRIES)
        except RuntimeError as e:
            raise RuntimeError(e)
        
        torso_link = self.planner_robot.link("torso_1")
        R, t = torso_link.getTransform()
        print("TORSO TRANSFORM: ", t)
        
        return goal_config, left_tcp, right_tcp, info
    
    
    def get_robot_goal_config(self, left_arm_target_pose=None, right_arm_target_pose=None, gazing=True): # new test method
        """
        Sets the target pose for the left and/or right arm.

        Parameters:
            target_pose_left (geometry_msgs/Pose, optional): The target pose for the left arm. Default is None.
            target_pose_right (geometry_msgs/Pose, optional): The target pose for the right arm. Default is None.

        Returns:
            Tuple ([List[float], Pose, Pose, str]):
            A tuple containing:
            - goal_config (List[float]): The resulting configuration if IK succeeds.
            - tcp_left_pose (Pose): The TCP pose of the left arm as a `geometry_msgs/Pose` object.
            - tcp_right_pose (Pose): The TCP pose of the right arm as a `geometry_msgs/Pose` object.
            - info (str): Information about the IK process, indicating success or failure.


        Raises:
            ValueError: If neither `target_pose_left` nor `target_pose_right` is provided.
            RuntimeError: If IK fails after the specified number of tries.

        """
        if left_arm_target_pose is None and right_arm_target_pose is None:
            raise ValueError("At least one of `target_pose_left` or `target_pose_right` must be provided")

        ik_objs = []

        # remove vis goal labels if exists
        if self.show_vis:
            try:
                vis.remove("LeftArm IK goal")
            except:
                pass

            try:
                vis.remove("RightArm IK goal")
            except:
                pass

        # Get Ik Objective for Left Arm
        if left_arm_target_pose is not None:
            # Select the end effector link.     
            leftEE_link = self.endEffector_leftArm
            # Get current positions and orientation.
            currentTransform_leftEE = leftEE_link.getTransform()
            currentPose_leftEE, currentOrientation_leftEE = currentTransform_leftEE[1], so3.quaternion(currentTransform_leftEE[0])
            # Determine goal orientation.
            # If goal orientation is provided use that, otherwise use current orientation as goal orientation.
            left_goal_rot = (
                so3.from_quaternion((left_arm_target_pose.orientation.w, left_arm_target_pose.orientation.x, left_arm_target_pose.orientation.y, left_arm_target_pose.orientation.z))
                if left_arm_target_pose.orientation else currentOrientation_leftEE)
            # get ik objective
            position = [left_arm_target_pose.position.x, left_arm_target_pose.position.y, left_arm_target_pose.position.z]
            ik_objective_left = self.get_ik_objective(leftEE_link, position, left_goal_rot)
            ik_objs.append(ik_objective_left)
            if self.show_vis:
                vis.add("LeftArm IK goal",ik_objective_left)
      
        # Get Ik Objective for Right Arm
        if right_arm_target_pose is not None:
            # Select the end effector link.
            rightEE_link = self.endEffector_rightArm
            # Get current positions and orientation.
            currentTransform_rightEE = rightEE_link.getTransform()
            currentPose_rightEE, currentOrientation_rightEE = currentTransform_rightEE[1], so3.quaternion(currentTransform_rightEE[0])
            # Determine goal orientation.
            # If goal orientation is provided use that, otherwise use current orientation as goal orientation.
            right_goal_rot = (
                so3.from_quaternion((right_arm_target_pose.orientation.w, right_arm_target_pose.orientation.x, right_arm_target_pose.orientation.y, right_arm_target_pose.orientation.z))
                if right_arm_target_pose.orientation else currentOrientation_rightEE)
            # get ik objective
            position = [right_arm_target_pose.position.x, right_arm_target_pose.position.y, right_arm_target_pose.position.z]
            ik_objective_right = self.get_ik_objective(rightEE_link, position, right_goal_rot)
            ik_objs.append(ik_objective_right)
            if self.show_vis:
                vis.add("RightArm IK goal",ik_objective_right) 

        # try to get the goal configuration solving IK. 
        try:
            goal_config, left_tcp, right_tcp, info = self.find_ik(ik_objs, num_tries = NUM_IK_TRIES)
        except RuntimeError as e:
            raise RuntimeError(e)
        
        
        # Hirachical move other arm away
        if(left_arm_target_pose is None or right_arm_target_pose is None):
            print("Validating distance of goal config or replanning..")
            unused_arm = "left" if left_arm_target_pose is None else "right"
            
            if(unused_arm=="left"):
                print("Unused arm is left")
                EE_link = self.endEffector_leftArm
                avoid_link = self.endEffector_rightArm
                moveable_link_names = ["arm_1_1", "arm_1_2", "arm_1_3", "arm_1_4", "arm_1_5", "arm_1_6", "arm_1_7"]
                moveable_subset = [self.planner_robot.link(name).index for name in moveable_link_names]
                
                gaze_link = self.planner_robot.link(LEFT_ARM_CAMERA_LINK)
                
            elif(unused_arm=="right"):
                print("Unused arm is right")
                EE_link = self.endEffector_rightArm
                avoid_link = self.endEffector_leftArm
                moveable_link_names = ["arm_2_1", "arm_2_2", "arm_2_3", "arm_2_4", "arm_2_5", "arm_2_6", "arm_2_7"]
                moveable_subset = [self.planner_robot.link(name).index for name in moveable_link_names]
                
                gaze_link = self.planner_robot.link(RIGHT_ARM_CAMERA_LINK)
                
            
            else: raise ValueError("unused_arm must be 'left' or 'right'")
            
            currentTransform_EE = EE_link.getTransform()
            currentPose_EE, currentOrientation_EE = currentTransform_EE[1], so3.quaternion(currentTransform_EE[0])
            
            avoid_Transform_EE = avoid_link.getTransform()
            avoid_Pose_EE, avoid_Orientation_EE = avoid_Transform_EE[1], so3.quaternion(avoid_Transform_EE[0])
            
            gaze_Transform_EE = gaze_link.getTransform()
            gaze_Pose_EE, gaze_Orientation_EE = gaze_Transform_EE[1], so3.quaternion(gaze_Transform_EE[0]) # orient this towards other hand!
            
            
            
            # Compute difference vector
            difference = vectorops.sub(position, currentPose_EE)
            
            # Euclidean (L2) distance
            distance = vectorops.norm(difference)
            distance_initial = distance
            
            print(distance)
            # Save to just change parts of joints
            initial_config = self.planner_robot.getConfig()
            print(f"INITIAL CONFIG: {initial_config}")
            
            if(distance <= 0.3):
                print("Klampt needs to avoid oh no!")
                
                while (distance <= 0.3):
                    self.planner_robot.setConfig(initial_config)
                    # Sample goal poses and check for ik solutions
                    sampled_goal = self.sample_away_from_xy(position=position)
                    
                    if(gazing==True):
                        dir_vec = vectorops.sub(avoid_Pose_EE, sampled_goal)
                        dir_vec = vectorops.unit(dir_vec)
                        goal_rot = so3.align([0,0,1], dir_vec)
                        ik_objective_avoid = self.get_ik_objective(gaze_link, sampled_goal, goal_rot)
                        
                    else:
                        goal_rot = so3.from_quaternion(avoid_Orientation_EE)
                        ik_objective_avoid = self.get_ik_objective(EE_link, sampled_goal, goal_rot)
                        
                    
                    
                    if self.show_vis:
                        try:
                            vis.remove("IK goal move away")
                        except:
                            pass
                        vis.add("IK goal move away",ik_objective_avoid)
                        
                    try:
                        goal_config_new, left_tcp_new, right_tcp_new, info = self.find_ik([ik_objective_avoid], num_tries = NUM_IK_TRIES, moveable_subset=moveable_subset, oneshot=False)
                        if(info == "False"):
                            continue
                        elif(not self.is_robot_config_feasible(goal_config_new)):
                            continue
                        else:
                            goal_config, left_tcp, right_tcp = goal_config_new, left_tcp_new, right_tcp_new
                    except RuntimeError as e:
                        raise RuntimeError(e)
                    
                    
                    
                    left_tcp_pos = [left_tcp.position.x, left_tcp.position.y, left_tcp.position.z]
                    right_tcp_pos = [right_tcp.position.x, right_tcp.position.y, right_tcp.position.z]
                    print(left_tcp_pos)
                    print(right_tcp_pos)
                    difference = vectorops.sub(left_tcp_pos, right_tcp_pos)
                    distance = vectorops.norm(difference)
                    
                    print(f"New Distance {distance}")
                    print(f"initial distance: {distance_initial}")
                    
                # for i in moveable_subset:
                #     goal_config_main[i] = goal_config[i]
                
            elif(gazing==True):
                acceptable_offset = 0.03
                retries = 5
                for gaze_idx in range(retries):
                    sampled_goal = currentPose_EE + np.random.uniform(-acceptable_offset, acceptable_offset, size=3)
                    print("SAMPLED GOAL: ", sampled_goal)
                    print("CURRENT POST: ", currentPose_EE)
                    dir_vec = vectorops.sub(avoid_Pose_EE, currentPose_EE)
                    dir_vec = vectorops.unit(dir_vec)
                    goal_rot = so3.align([0,0,1], dir_vec)
                    R_z90 = so3.rotation([0, 0, 1], -math.radians(90))
                    goal_rot = so3.mul(goal_rot, R_z90)
                    ik_objective_gaze = self.get_ik_objective(gaze_link, sampled_goal, goal_rot)
                    
                    ik_objectives_copy = ik_objs.copy()
                    ik_objectives_copy.append(ik_objective_gaze)
                    
                    if self.show_vis:
                        try:
                            vis.remove("IK goal move away")
                        except:
                            pass
                        vis.add("IK goal move away",ik_objective_gaze)
                        
                    try:
                        #goal_config_new, left_tcp_new, right_tcp_new, info = self.find_ik(ik_objs, num_tries = NUM_IK_TRIES, moveable_subset=moveable_subset, oneshot=False)
                        goal_config_new, left_tcp_new, right_tcp_new, info = self.find_ik(ik_objectives_copy, num_tries = NUM_IK_TRIES, oneshot=False)
                        if(info == "False"):
                            continue
                        elif(not self.is_robot_config_feasible(goal_config_new)):
                            continue
                        else:
                            print("SETTING NEW GOAL CONFIG :)")
                            goal_config, left_tcp, right_tcp = goal_config_new, left_tcp_new, right_tcp_new
                            break
                    except RuntimeError as e:
                        raise RuntimeError(e)
                    
            
        
        return goal_config, left_tcp, right_tcp, info

    def sample_away_from_xy(self, position, min_xy_distance=0.15, xy_range=0.3, z_range=(0.05, 0.3)):
        """
        Sample 3D poses that are at least `min_xy_distance` away in x and y from the given position.

        Args:
            position: [x, y, z] list, the point to avoid.
            min_xy_distance: minimum required separation in x and y axes.
            num_samples: how many samples to generate.
            xy_range: max offset range in x and y directions (symmetric around the origin).
            z_range: tuple (min_z_offset, max_z_offset) to vary height.

        Returns:
            List of sampled 3D [x, y, z] points.
        """
        px, py, pz = position

        while True:
            dx = np.random.uniform(-xy_range, xy_range)
            dy = np.random.uniform(-xy_range, xy_range)

            # Skip samples too close in x or y
            if abs(dx) < min_xy_distance or abs(dy) < min_xy_distance:
                continue

            dz = np.random.uniform(z_range[0], z_range[1])
            if(pz + dz <= 1.20):
                continue

            sampled = [px + dx, py + dy, pz + dz]
            return sampled


    def is_robot_config_feasible(self, robot_config):
        """ 
        Check if the current robot configuration is feasible 
        
        Parameters:
            robot_config (List[float]): The robot configuration to check, typically a list of joint positions.
        
        Returns:
            `True` or `False`
        """
        return self.__cspace.feasible(robot_config)
            
        
    def get_feasibility_info(self, robot_config):
        """ 
        Returns a dictionary of information from feasibility check. 
        
        Parameters:
            robot_config (List[float]): The robot configuration to check, typically a list of joint positions.
        
        Returns:
            A dictionary of information from feasibility check including;
            - feasible
            - collision
            - self_collision
            - joint_limits
        """
        # Create a dictionary for the feasibility checks
        feasibility_checks = {
            'feasible': self.__cspace.feasible(robot_config),
            'collision': self.__cspace.envCollision(robot_config),
            #'self_collision': self.planner_robot.selfCollides(),
            'self_collision': len(list(self.__collider.robotSelfCollisions(self.planner_robot.index))) > 0,
            'joint_limits': self.__cspace.inJointLimits(robot_config)
        }
        return feasibility_checks
            

    def get_plan_to_goal_config(self, goal_config, planner='sbl') -> Tuple[Union['EmbeddedMotionPlan', 'MotionPlan', None], float, int]:
        """ 
        Given the goal configuration of the robot, returns the path to the goal from current configuration.

        Parameters:
            goal_config (List[float]): The goal configuration of the robot.
            planner (str): type of planner to use. Options are 'sbl' or 'rrt'. Default planner is 'sbl'.

        Returns:
            tuple ((plan, planning_time, num_iters)):
        """
        if planner == 'sbl':
            settings = PLANNER_SETTINGS_SBL
        elif planner == 'rrt':
            settings = PLANNER_SETTINGS_RRT
        else:
            self.__logwarn(f"given planner {planner} is not a valid planner. Using sbl planner...")
            settings = PLANNER_SETTINGS_SBL

        plan = plan_to_config(self.world, self.planner_robot, goal_config, movingSubset="auto", **settings)

        if not plan:
            raise RuntimeError("Failed to generate a motion plan for given goal configuration.")
        
        # Execute the planner iterations
        t0 = time.time()
        num_iters = 0
        for round_num in range(MAX_PLANNER_ITERS // 10):
            plan.planMore(10)
            num_iters += 10
            if not IS_PLANNER_OPTIMIZING:
                path = plan.getPath()
                if path and len(path) > 0:
                    break

            if time.time() - t0 > MAX_PLANNER_TIME:
                break

        planning_time = time.time() - t0
        # self.planner_robot.setConfig(start_config)
        return plan, planning_time, num_iters
    
    def validate_plan_to_goal_config(self, plan:MotionPlan):
        """  
        Validate the motion plan to ensure it has a valid path to the goal configuration.

        Parameters:
            plan (MotionPlan): The motion plan to be validated.

        Returns:
            bool: Returns True if the path is valid (non-empty), otherwise False.
        """
        # Extract the path and validate it
        path = plan.getPath()
        return False if not path or len(path) == 0 else True
        
            
    def get_interpolate_path(self, path, num_waypoints=NUM_WAY_POINTS):
        """  
        Generate an interpolated path between waypoints.

        Parameters:
            path (list): A list of waypoints, where each waypoint is a list or tuple representing a configuration.
            num_waypoints (int, optional): The number of intermediate waypoints to generate between each pair of 
                waypoints in the original path. Defaults to NUM_WAY_POINTS.

        Returns:
            list: A list of interpolated waypoints.
        """
        interpolated_path = []
        total_points = len(path) - 1
        for i in range(total_points):
            start = path[i]
            end = path[i + 1]
            for t in np.linspace(0, 1, num=num_waypoints):
                interp_point = [start[j] * (1 - t) + end[j] * t for j in range(len(start))]
                interpolated_path.append(interp_point)
        return interpolated_path
    

    def get_trajectory_to_goal(self, path):
        """  
        Create and visualize a trajectory to the goal configuration.

        Parameters:
            path (list): A list of waypoints that the trajectory will follow.

        Returns:
            RobotTrajectory: The generated `RobotTrajectory` object.

        Notes:
            - The `show_vis` flag should be set to True to visualize the trajectory.
            - The visualization will add the trajectory to the `vis` module with a discretization interval of 0.5.
        """
        traj = RobotTrajectory(self.planner_robot, range(len(path)), path)
        if self.show_vis:
            vis.add("trajectory", traj.discretize(0.5), color=(0, 0, 1, 1))
        return traj
    

    def get_plan_to_joint_goal(self, target_robot_config=None, target_joint_config=None, planner:str ='sbl'):
        """  
        Plan a path to a joint goal for the robot arms.

        Parameters:
            target_joint_config (List[float]): The target joint configuration for the robot arms.

        Returns:
            MotionPlan: The `MotionPlan` object containing the planned path.
        """
        # set robot config to current config
        if target_joint_config is not None:
            start_config = self.real_robot.getConfig()
            self.setJointValues(self.planner_robot, target_joint_config)
        
        elif target_robot_config is not None:
            start_config = self.real_robot.getConfig()
            self.planner_robot.setConfig(target_robot_config)

        goal_robot_config = self.planner_robot.getConfig()
        # self.planner_robot.setConfig(start_config)


        # check if the goal configuration is feasible
        if not self.is_robot_config_feasible(self.planner_robot.getConfig()):
            error = f"\t- Configuration is not feasible: {self.get_feasibility_info(self.planner_robot.getConfig())}"
            raise RuntimeError(error)

        # if feasible
        feasibility_info = self.get_feasibility_info(self.planner_robot.getConfig())
        self.__loginfo(f"\t- Feasibility info: {feasibility_info}")
        print("---\n")


        # set robot config back to start config
        self.planner_robot.setConfig(start_config)
        
        # get the plan from start config to goal config
        self.__loginfo("Getting plan to the traget...")
        plan, dt, num_iters = self.get_plan_to_goal_config(goal_robot_config, planner=planner)
        info = f"plan info; \n \
                \t- path len: {len(plan.getPath())}\n \
                \t- time to plan: {dt}\n \
                \t- num_iters: {num_iters}  \
                "
        self.__loginfo(info)
        print("---\n")

        # validate plan to goal config
        if not self.validate_plan_to_goal_config(plan):
            raise RuntimeError(f"Failed to plan a feasible path.\nPlanner stats: {plan.getStats()}")


        self.__loginfo(f"Motion planning was successfully completed.\n")

        return plan


    def get_plan_to_cartesian_goal(
            self, 
            left_arm_goal_pose:Pose = None, 
            right_arm_goal_pose:Pose = None, 
            planner:str ='sbl',
            gazing=True
        ) -> MotionPlan:
        """
        Plan a path to a Cartesian goal for the robot arms.

        Parameters:
            left_arm_goal_pose (Pose, optional): The target pose for the left arm. Defaults to None.
            right_arm_goal_pose (Pose, optional): The target pose for the right arm. Defaults to None.
            planner (str, optional): The planner to use for motion planning. Defaults to 'sbl'.
            waypoints (int, optional): The number of waypoints to interpolate along the path. Defaults to NUM_WAY_POINTS.

        Returns:
            plan (MotionPlan): The `MotionPlan`.
        
        Raises:
            RuntimeError: If the goal configuration is not feasible or if a feasible path cannot be planned.
        """

        self.__loginfo("Setting planner_robot config to real_robot config...")
        self.planner_robot.setConfig(self.real_robot.getConfig())
        
        start_config = self.planner_robot.getConfig()
        
        # Get goal config
        self.__loginfo("Getting robot configuration for target pose...")
        goal_config, left_tcp, right_tcp, info = self.get_robot_goal_config(left_arm_target_pose=left_arm_goal_pose, right_arm_target_pose=right_arm_goal_pose, gazing=gazing)

        left_tcp_pos = [left_tcp.position.x, left_tcp.position.y, left_tcp.position.z]
        left_tcp_quad = [left_tcp.orientation.x, left_tcp.orientation.y, left_tcp.orientation.z, left_tcp.orientation.w]

        right_tcp_pos = [right_tcp.position.x, right_tcp.position.y, right_tcp.position.z]
        right_tcp_quad = [right_tcp.orientation.x, right_tcp.orientation.y, right_tcp.orientation.z, right_tcp.orientation.w]

        info = [info, 
                f"- left tcp pos:   {left_tcp_pos}", 
                f"- left tcp quad:  {left_tcp_quad}", 
                f"- right tcp pos:  {right_tcp_pos}", 
                f"- right tcp quad: {right_tcp_quad}"
                ]
        
        for info_ in info:
            self.__loginfo(info_)
        print("---\n")



        self.__loginfo("\nChecking feasibility for the target configuration...")
        # check if the goal configuration is feasible
        self.planner_robot.setConfig(goal_config)
        if not self.is_robot_config_feasible(self.planner_robot.getConfig()):
            error = f"\t- Configuration is not feasible: {self.get_feasibility_info(self.planner_robot.getConfig())}"
            raise RuntimeError(error)

        # if feasible
        feasibility_info = self.get_feasibility_info(self.planner_robot.getConfig())
        self.__loginfo(f"\t- Feasibility info: {feasibility_info}")
        print("---\n")


        # set robot config back to start config
        self.planner_robot.setConfig(start_config)
        
        # get the plan from start config to goal config
        self.__loginfo("Getting plan to the traget...")
        plan, dt, num_iters = self.get_plan_to_goal_config(goal_config, planner=planner)
        info = f"plan info; \n \
                \t- path len: {len(plan.getPath())}\n \
                \t- time to plan: {dt}\n \
                \t- num_iters: {num_iters}  \
                "
        self.__loginfo(info)
        print("---\n")

        # validate plan to goal config
        if not self.validate_plan_to_goal_config(plan):
            raise RuntimeError(f"Failed to plan a feasible path.\nPlanner stats: {plan.getStats()}")


        self.__loginfo(f"Motion planning was successfully completed.\n")

        return plan
    
    def get_ros_joint_trajectory_from_plan(\
        self, 
        plan:MotionPlan=None, 
        target_time:float = 8.0, 
        joint_update_rate:int=100
    ) -> JointTrajectory:
        """  
        Generates a ROS JointTrajectory message from a motion plan.

        Parameters:
            plan (MotionPlan): The motion plan containing the path to be converted into a JointTrajectory.
            target_time (float): The desired time to complete the trajectory, in seconds. If this value is 
                                less than the minimum required time based on velocity limits, it will be adjusted.
            joint_update_rate (int): The rate at which joint positions are updated, in Hz.

        Returns:
            JointTrajectory: A ROS JointTrajectory message containing the waypoints and timing information.
                            Returns None if the plan is invalid or not provided.
        """
        # validate the path
        if plan is None:
            self.__logwarn("Plan is None. Failed to create JointTrajectory!")
            return None
        
        # get the path from the plan (optimal path)
        path = plan.getPath()

        last_config = path[-1]

        # --- Convert Klampt config -> SE3 pose ---
        # Assuming 'plan.robot' exists and is the robot model
        old_config = self.planner_robot.getConfig()

        self.planner_robot.setConfig(last_config)

        # Example: using robot base link (index 0) or specific link
        link_index = 0  # change this to the link of interest
        T = self.planner_robot.link("right_hand_v1_2_research_grasp_link").getTransform()  # (R, t)

        # Convert to ROS Pose
        R, t = T
        q = so3.quaternion(R)
        q = np.roll(q, 1)
        new = list(t) + list(q)
        print("LAST MILESTONE: ", new)
        self.publish_tf_np(new, child_frame="last_milestone_klampt")
        self.planner_robot.setConfig(old_config)
        
        # filter only joint values of the path and make it a numpy array
        path = np.array(self.filter_path(path))

        # get joint velocity limits
        joint_vel_limits = self.get_robot_drivers_velocity_limits()
        joint_vel_upper_limits = np.array([limit[1] for limit in joint_vel_limits ])

        # calculate distances and max times for each milestone transition
        milestone_distance = np.abs(np.diff(path, axis=0))
        max_time_needed_for_milestone = np.max(milestone_distance / joint_vel_upper_limits, axis=1) 
        # add 2.0s of threshold to max_time_needed_for_milestone to slow down if speed is super fast!
        max_time_needed_for_milestone += 10.0

        # Total time needed for full transition
        total_time_needed = np.sum(max_time_needed_for_milestone)   # add a threshold of 2.0 seconds 

        # Adjust target time if need
        if target_time < total_time_needed :
            warn = f"target_time {target_time:.4f}s is not sufficient to reach the goal. Minimum required time is {total_time_needed:.4f}s."
            self.__logwarn(warn)
            info = f"Setting target_time to {total_time_needed:.4f}s"
            self.__loginfo(info)
            target_time = total_time_needed

        # compute target times for each milestone
        target_time_for_milestone = np.round(target_time * max_time_needed_for_milestone / total_time_needed, 2)

        # create waypoints
        waypoints = []
        for i, (start, end, time_needed) in enumerate(zip(path[:-1], path[1:], target_time_for_milestone)):
            num_waypoints = int(time_needed * joint_update_rate)
            # interpolate waypoints
            segment_waypoints = np.linspace(start, end, num=num_waypoints, endpoint=True)
            waypoints.append(segment_waypoints)

        print("LAST WAYPOINT: ", np.array(waypoints)[-1, -1])

        # Flatten the list of waypoints
        waypoints = np.round(np.vstack(waypoints), 8)



        # generate time stamps
        time_from_start = np.round(np.linspace(0, target_time, len(waypoints)), 4)


         # create a JointTrajectory instance and asign values
        traj_msg = JointTrajectory()
        joint_names = [key for key, _ in home_joint_config.items()]
        traj_msg.joint_names = joint_names

        # Add waypoints and time_from_start to the JointTrajectory message
        for i, (waypoint, time) in enumerate(zip(waypoints, time_from_start)):
            point = JointTrajectoryPoint()
            point.positions = waypoint.tolist()  # Convert NumPy array to a list
            point.time_from_start = rospy.Duration(time)  # Convert time to ROS Duration
            traj_msg.points.append(point)


        return traj_msg
    

    def add_sand_to_env(self, margin=0.01):
        """
        obb: Open3D OrientedBoundingBox, e.g. pcl_base_link.get_minimal_oriented_bounding_box()
        """
        # Dimensions and pose from Open3D OBB
        w, h, d = 1.0, 2.5, 0.005
        Rw = np.eye(3)   # 3x3 rotation (world)
        tw = np.array([0, 0, 1.08])

        # Origin-centered box primitive (local axes)
        geom = create.primitives.box(w, h, d, type="GeometricPrimitive")
        try: geom.setCollisionMargin(margin)
        except Exception: pass

        # Add as a rigid object and place at OBB pose
        obj = self.world.makeRigidObject("Sand")
        obj.geometry().set(geom)
        obj.setTransform(so3.from_matrix(Rw.tolist()), tw.tolist())
        obj.appearance().setColor(1, 0, 0, 0.35)  # translucent red

        # Optional visualization
        try: vis.add("Sand", obj)
        except Exception: pass

        return obj



    def move_sand_x(self, pose):
        """Moves the terrain 'shelf' by +dx meters along the x-axis."""
        vis.lock()
        R = se3.identity()[0]  # 3x3 rotation matrix
        dx = pose.position.x
        if dx == 0:
            t = [0, 0, 1.08] 
        else:
            t = [dx, 0, 0] 
        self.sand_plane.setTransform(R, t)
        vis.unlock()
        return True   
    

    def publish_tf_np(self, pose, par_frame="world", child_frame="goal_frame"):
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = geometry_msgs.msg.TransformStamped()

        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = par_frame
        static_transformStamped.child_frame_id = child_frame

        static_transformStamped.transform.translation.x = float(pose[0])
        static_transformStamped.transform.translation.y = float(pose[1])
        static_transformStamped.transform.translation.z = float(pose[2])

        static_transformStamped.transform.rotation.x = float(pose[3])
        static_transformStamped.transform.rotation.y = float(pose[4])
        static_transformStamped.transform.rotation.z = float(pose[5])
        static_transformStamped.transform.rotation.w = float(pose[6])

        broadcaster.sendTransform(static_transformStamped)
        rospy.sleep(1)

        return True

        


    

