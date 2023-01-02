#!/usr/bin/env python3
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from math import pi, tau, dist, fabs, cos

from rjje_arm.srv import MoveArm, MoveArmRequest, MoveArmResponse
from rjje_arm.srv import MoveHand, MoveHandRequest, MoveHandResponse
from moveit_commander.conversions import pose_to_list

# Hand actions
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal

"""
Notes:
    1. the ros ik solver is optimized for 6 dof robots. Generate a solver from 
    TracIK?
"""

def all_close(goal, actual, tolerance, position_only = True):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        if position_only:
            return d <= tolerance 
        else:
            # phi = angle between orientations
            cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
            return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        NODE_NAME = "rjje_arm_move_group"
        rospy.init_node(NODE_NAME, anonymous=True)
        ## Can see the kinematic model and the robot's current joint states
        self.robot = moveit_commander.RobotCommander()
        ## Remote interface for getting, setting, and updating the planning scene
        self.scene = moveit_commander.PlanningSceneInterface()
        ## Planning group (group of joints) to plan and execute motions
        group_name = "rjje_arm"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Display trajectories in Rviz:
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
        )

        # We can get the name of the reference frame for this robot:
        self.planning_frame = self.move_group.get_planning_frame()
        print(f"============ Planning frame: {self.planning_frame}" )
        # We can also print the name of the end-effector link for this group:
        self.eef_link = self.move_group.get_end_effector_link()
        print(f"============ End effector link: {self.eef_link}" )
        # We can get a list of all the groups in the robot:
        self.group_names = self.robot.get_group_names()
        print("============ Available Planning Groups:", self.robot.get_group_names())
        # Sometimes for debugging it is useful to print the entire state of the robot
        print("============ Printing robot state")
        print(self.robot.get_current_state())

        self.move_group.set_max_velocity_scaling_factor(0.2)
        self.move_group.set_max_acceleration_scaling_factor(1.0)

        ARM_SERVICE = rospy.get_param("/SERVICES/ARM_SERVICE")
        self.arm_server = rospy.Service(ARM_SERVICE, MoveArm, self.arm_service) 

        HAND_SERVICE = rospy.get_param("/SERVICES/HAND_SERVICE")
        self.hand_server = rospy.Service(HAND_SERVICE, MoveHand, self.hand_service) 
        GRIPPER_ACTION_SERVER_NAME = rospy.get_param("ACTIONS/GRIPPER_ACTION_SERVER_NAME")
        self.hand_action_client = actionlib.SimpleActionClient(GRIPPER_ACTION_SERVER_NAME, GripperCommandAction)
        rospy.loginfo(f"{NODE_NAME} has been initialized")

    def arm_service(self, move_arm_req: MoveArmRequest):
        """
        Service callback function that 
        1. takes in a goal pose of the end-efffector link
        2. plan a trajectory that goes there 
        3. executes the trajectory
        4. Report the execution status back

        Args:
            move_arm_req (MoveArmRequest): arm move request

        Returns:
            Arm-move service response (MoveArmResponse): 
                current pose of the end effector, and the success status.
        """
        # For testing
        # pose_goal.orientation = geometry_msgs.msg.Quaternion(0.7665437588561735,  -0.010203678134488959, 0.008562231779498048, 0.64205392211101)
        # pose_goal.position = geometry_msgs.msg.Point(0.012, -0.102, 0.304)

        # Use approxIK
        self.move_group.set_joint_value_target(move_arm_req.target_pose, 
                                               self.eef_link, True)
        
        # TODO: this is a make shift way for the move_group planning
        print("Current pose", self.move_group.get_current_pose().pose)
        planning_success, traj, planning_time, planning_error = self.move_group.plan()
        self.move_group.execute(traj, wait=True)

        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        self.move_group.clear_pose_targets()
        current_pose = self.move_group.get_current_pose().pose
        return MoveArmResponse(current_pose = current_pose,
                               success = all_close(move_arm_req.target_pose, current_pose, 0.05))

    def hand_service(self, move_hand_req: MoveHandRequest) -> MoveHandResponse:
        """
        Service to open / close hand, by calling action server GRIPPER_ACTION_SERVER_NAME. 
        We have this funciton to be consistent with the arm chain of commands: 
            arm service -> arm action server -> esp32 

        Args:
            move_hand_req (MoveHandRequest): service request to open / close hand

        Returns:
            result (MoveHandResponse)
        """
        self.hand_action_client.wait_for_server() 
        goal = GripperCommandGoal()
        if move_hand_req.open:
            goal.command.position = 0.61
        else: 
            goal.command.position = 0.0
        self.hand_action_client.send_goal(goal)
        self.hand_action_client.wait_for_result()
        return MoveHandResponse(success = (self.hand_action_client.get_state() == actionlib.GoalStatus.SUCCEEDED))
        

if __name__ == "__main__":
    tutorial = MoveGroupPythonInterfaceTutorial()
    rospy.spin()


