#!/usr/bin/env python3
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from math import pi, tau, dist, fabs, cos

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

"""
Notes:
    1. the ros ik solver is optimized for 6 dof robots. Generate a solver from 
    TracIK?
"""

def all_close(goal, actual, tolerance):
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
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("rjje_arm_move_group", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).
        ## This interface can be used to plan and execute motions:
        group_name = "rjje_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
        )

        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print(f"============ Planning frame: {planning_frame}" )
        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print(f"============ End effector link: {eef_link}" )
        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())
        # Sometimes for debugging it is useful to print the entire state of the robot
        print("============ Printing robot state")
        print(robot.get_current_state())

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_pose_goal(self):
        move_group = self.move_group
        # move_group.set_position_target([-0.041000, -0.09000, 0.2050000])
        # # Is this the eef pose? 
        move_group.set_goal_tolerance(0.1000000)
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation = geometry_msgs.msg.Quaternion(0.7665437588561735,  -0.010203678134488959, 0.008562231779498048, 0.64205392211101)
        pose_goal.position = geometry_msgs.msg.Point(0.012, -0.102, 0.304)

        # Use approxIK
        move_group.set_joint_value_target(pose_goal, "link_6", True)
        # move_group.set_pose_target(pose_goal)

        success = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        # TODO
        print(f"Current pose: {self.move_group.get_current_pose()}")
        return all_close(pose_goal, current_pose, 0.01)

if __name__ == "__main__":
    tutorial = MoveGroupPythonInterfaceTutorial()
    # input("============ Press `Enter` to execute a movement using a pose goal ...")
    tutorial.go_to_pose_goal()


