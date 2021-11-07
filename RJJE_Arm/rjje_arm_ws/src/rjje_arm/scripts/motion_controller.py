"""
Motion controller that can control a robot arm and a gripper independently, in a soft real-time fashion, ~2ms 
Internally, the arm and gripper shares the same message to the uC. And their status is updated in a lock-free manner.
On arduino, all motors go from (0, 180) following the right-hand convention, with the neutral position = 90. 
"""
import rospy
import numpy as np
import math
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryFeedback
from control_msgs.msg import FollowJointTrajectoryResult
from control_msgs.msg import GripperCommandAction
from control_msgs.msg import GripperCommandFeedback
from control_msgs.msg import GripperCommandResult
from sensor_msgs.msg import JointState
import actionlib
from threading import Lock
from rjje_arm.msg import MotionControl
from rjje_arm.msg import JointFeedback
import threading

ARM_ACTION_SERVER = "rjje_arm"
GRIPPER_ACTION_SERVER = "rjje_gripper"

def send_reponse(action_server: actionlib.SimpleActionServer, action_server_name: str, status):
    """
    Send response back to ros action server. Currently only success is sent
    :param action_server: action server for the arm or for the gripper
    :param action_server_name: name of the action server
    :param status of trajectory completion: PREEMPTED or SUCCESSFUL
    """
    msg = f'{action_server_name} Trajectory completion status: {status}'
    rospy.loginfo(msg)
    res = FollowJointTrajectoryResult()
    if status == "SUCCESSFUL": 
        action_server.set_succeeded(result=res, text=msg)
    elif status == "PREEMPTED":
        action_server.set_preempted()


class MotionController: 
    def __init__(self): 
        rospy.init_node("motion_controller", anonymous=True)     #anonymous=true ensures unique node name by adding random numbers
        # one digit after decimal point
        self.commanded_angles = [90, 90, 90, 90, 90, 90]
        self.execution_time = 1.0 #seconds, one digit after decimal point
        # in CPython, bool write is atomic.
        self.arm_move_event = threading.Event()
        # self.gripper_to_move = True
        self.ANGULAR_THRESHOLD = 0.5

        self.action_server_arm = actionlib.SimpleActionServer('/rjje_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction, execute_cb=self.process_arm_action, auto_start=False)    #auto_start is false, so we can start at a later time
        self.action_server_arm.start()
        rospy.loginfo("Arm action server started")

        self.action_server_gripper = actionlib.SimpleActionServer('/rjje_gripper_controller/gripper_command', GripperCommandAction, execute_cb=self.process_gripper_action, auto_start=False)    #auto_start is false, so we can start at a later time
        self.action_server_gripper.start()
        rospy.loginfo("Gripper action server started")

        self.__move_joints_lock = Lock()
        self.__joint_state_lock = Lock()
        self.joint_state_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
        self.joint_state_msg = JointState()
        self.joint_state_msg.position = self.__convert_to_joint_msg_angles(self.commanded_angles)
        # TODO: our URDF model's gripper isn't accurate, so disabling visualizing the gripper for now
        self.joint_state_msg.position[5] = 0.0

        #TODO: to change to soft-coded way
        self.joint_state_msg.name = ["link1_bracket_1", "link_2_bracket_2_1", "bracket_2_2_link_3", "bracket_3_2_link_4", "link_5_link_6", "link_6_left_gripper"]
        # index (our internal representation -> joint moveit's trajectory )
        self.traj_point_joint_name_lookup = []
        rospy.loginfo("joint state publisher set up")

        self.motion_control_pub = rospy.Publisher("/rjje_arm/motion_control", MotionControl, queue_size=10)
        self.motion_control_msg = MotionControl()
        self.joint_feedback_sub = rospy.Subscriber("/rjje_arm/joint_feedback", JointFeedback, self.joint_feedback_cb)
        rospy.loginfo("Interface for arduino topics is setup")

#############################################################################################
    def update_action_servers(self): 
        with self.__joint_state_lock: 
            #now the arm action server must be waiting 
            if np.allclose(np.array(self.joint_state_msg.position[:5]), np.array(self.__convert_to_joint_msg_angles(self.commanded_angles[:5])), atol=self.ANGULAR_THRESHOLD):
                self.arm_move_event.set()
        # TODO Potential Bug: our URDF model's gripper isn't accurate, so disabling visualizing the gripper for now
        # if self.gripper_to_move: 
        #     if abs(self.joint_state_msg.position[5] - self.commanded_angles[5]) < self.ANGULAR_THRESHOLD: 
        #         self.gripper_to_move = False
#############################################################################################
    def publish_joint_state_msg(self): 
        with self.__joint_state_lock:
            self.joint_state_msg.header.stamp = rospy.Time.now()
            self.joint_state_pub.publish(self.joint_state_msg)

    def joint_feedback_cb(self, msg): 
        with self.__joint_state_lock:
            self.joint_state_msg.position = self.__convert_to_joint_msg_angles(msg.joint_angles)
            # TODO: our URDF model's gripper isn't accurate, so disable the gripper for now
            self.joint_state_msg.position[5] = 0.0

    def process_gripper_action(self, goal): 
        self.__process_action(GRIPPER_ACTION_SERVER, goal)

    def process_arm_action(self, goal): 
        self.__process_action(ARM_ACTION_SERVER, goal)

    def __process_action(self, action_server_name: str, goal):
        """
        Actual callback for Moveit! trajectory following action request. Note that each action server runs on a separate thread
        from the main one.
        :param action_server: arm/gripper action server
        :param action_server_name: name of the action server
        :param goal: the goal trajectory from ros action client
        :return: (list_of_exection_times, list_of_commanded_angles)
        """
        # Workflow:
        # Start from point 1, since the first point is the current starting point
        # Check for pre-emption
        # Figure out the duration and joint positions of each trajectory segment
        # Realize each segment and time it
        # Wait till finish
        # Send response
        if action_server_name == ARM_ACTION_SERVER:
            action_server = self.action_server_arm
        elif action_server_name == GRIPPER_ACTION_SERVER:
            action_server = self.action_server_gripper

        traj = goal.trajectory
        num_points = len(traj.points)
        commanded_angles_ls = []
        time_ls = []
        self.__build_traj_joint_lookup(traj)
        for i in range(1, num_points):
            if action_server.is_preempt_requested():
                rospy.loginfo(f"Trajectory Action Preempted on {action_server_name}" )
                self.action_server_arm.set_preempted()
                send_reponse(action_server, action_server_name, "PREEMPTED")
                break
            else: 
                #TODO: if we add gripper service in, we need to change the commanded angle a bit
                self.__get_commanded_angles_from_traj(traj.points[i])

                #TODO: if move gripper before finishing moving arm, will cause problem
                self.execution_time = (traj.points[i].time_from_start - traj.points[i-1].time_from_start).to_sec()
                with self.__move_joints_lock: 
                    rospy.loginfo(f"{action_server_name} is moving joints: {self.commanded_angles}, will accomplish in {self.execution_time}") 
                    self.__move_joints() 
                    # wait for motion to finish 
                self.arm_move_event.wait(10)    #10s as timeout
                rospy.loginfo(f"update action server: {self.joint_state_msg.position} | vs {self.__convert_to_joint_msg_angles(self.commanded_angles)}")
        send_reponse(action_server, action_server_name, "SUCCESSFUL")

    def __convert_to_joint_msg_angles(self, commanded_angles: list): 
        return [3.1415 * ((angle-90.0)/180.0) for angle in commanded_angles]

    def __convert_to_rjje_command_angles(self, ros_commanded_angles: list): 
        return [180 * (angle/3.1415) + 90 for angle in ros_commanded_angles]

    def __move_joints(self): 
        self.motion_control_msg.task_id = (self.motion_control_msg.task_id + 1)%10
        self.motion_control_msg.c0 = self.commanded_angles[0]
        self.motion_control_msg.c1 = self.commanded_angles[1]
        self.motion_control_msg.c2 = self.commanded_angles[2]
        self.motion_control_msg.c3 = self.commanded_angles[3]
        self.motion_control_msg.c4 = self.commanded_angles[4]
        self.motion_control_msg.c5 = self.commanded_angles[5]
        self.motion_control_msg.execution_time = self.execution_time
        self.motion_control_pub.publish(self.motion_control_msg)
        rospy.loginfo(f"Sent new joint msg to arduino: {self.commanded_angles}")
    
    def __build_traj_joint_lookup(self, traj): 
        """
        Build a look up table for index (our internal representation) -> index(joint moveit's trajectory)
        """
        if not self.traj_point_joint_name_lookup:
            self.traj_point_joint_name_lookup = [traj.joint_names.index(joint) for joint in self.joint_state_msg.name]

    def __get_commanded_angles_from_traj(self, traj_point): 
        """
        Update self.commanded_angles[:5] from trajectory. So we're not updating the gripper angle here
        """
        for i in range(len(self.commanded_angles) - 1): 
            index = self.traj_point_joint_name_lookup[i]
            self.commanded_angles[i] = traj_point.positions[index]
        self.commanded_angles[:5] = self.__convert_to_rjje_command_angles(self.commanded_angles[:5])


if __name__ == '__main__': 
    mc = MotionController()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        mc.publish_joint_state_msg()
        mc.update_action_servers()
        r.sleep()


