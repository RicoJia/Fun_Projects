"""
Motion controller that can control a robot arm and a gripper independently, in a soft real-time fashion, ~2ms 
Internally, the arm and gripper shares the same message to the uC. And their status is updated in a 
lock-free manner.
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

def send_reponse(action_server: actionlib.SimpleActionServer, action_server_name: str):
    """
    Send response back to ros action server. Currently only success is sent
    """
    msg = f'{action_server_name} Trajectory completed'
    rospy.loginfo(msg)
    res = FollowJointTrajectoryResult()
    action_server.set_succeeded(result=res, text=msg)

class MotionController: 
    def __init__(self): 
        rospy.init_node("motion_controller", anonymous=True)     #anonymous=true ensures unique node name by adding random numbers
        # one digit after decimal point
        self.commanded_angles = [90, 90, 90, 90, 90, 90]
        self.execution_time = 1.0 #seconds, one digit after decimal point
        # in CPython, bool write is atomic.
        self.arm_move_event = threading.Event()
        # self.gripper_to_move = True
        self.ANGULAR_THRESHOLD = 1.0

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
        # TODO: our URDF model's claw isn't accurate, so disabling visualizing the claw for now
        self.joint_state_msg.position[5] = 0.0

        #TODO: to change to soft-coded way
        self.joint_state_msg.name = ["link1_bracket_1", "link_2_bracket_2_1", "bracket_2_2_link_3", "bracket_3_2_link_4", "link_5_link_6", "link_6_left_gripper"]
        rospy.loginfo("joint state publisher set up")

        self.motion_control_pub = rospy.Publisher("/rjje_arm/motion_control", MotionControl, queue_size=10)
        self.motion_control_msg = MotionControl()
        self.joint_feedback_sub = rospy.Subscriber("/rjje_arm/joint_feedback", JointFeedback, self.joint_feedback_cb)
        rospy.loginfo("Interface for arduino topics is setup")

#############################################################################################
    def update_action_servers(self): 
        # TODO
        with self.__joint_state_lock: 
            print(f"update action server: {self.joint_state_msg.position} | {self.__convert_to_joint_msg_angles(self.commanded_angles)}")
            #now the arm action server must be waiting 
            if np.allclose(np.array(self.joint_state_msg.position[:5]), np.array(self.__convert_to_joint_msg_angles(self.commanded_angles[:5])), atol=self.ANGULAR_THRESHOLD):
                self.arm_move_event.set()
        # TODO Potential Bug: our URDF model's claw isn't accurate, so disabling visualizing the claw for now
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
            # TODO: our URDF model's claw isn't accurate, so disabling visualizing the claw for now
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

        success = True
        traj = goal.trajectory
        num_points = len(traj.points)
        commanded_angles_ls = []
        time_ls = []
        for i in range(1, num_points):
            if action_server.is_preempt_requested():
                rospy.loginfo(f"Trajectory Action Preempted on {action_server_name}" )
                self.action_server_arm.set_preempted()
                success = False
                break
            else: 
                if action_server_name == ARM_ACTION_SERVER: 
                    self.commanded_angles[:5] = self.__convert_to_rjje_command_angles(traj.points[i].positions) 
                elif action_server_name == GRIPPER_ACTION_SERVER: 
                    self.commanded_angles[5] = self.__convert_to_rjje_command_angles(traj.points[i].positions) 

                #TODO: if move claw before finishing moving arm, will cause problem
                self.execution_time = (traj.points[i].time_from_start - traj.points[i-1].time_from_start).to_sec()
                with self.__move_joints_lock: 
                    print(f"{action_server_name} is moving joints: {self.commanded_angles}, will accomplish in {self.execution_time}") 
                    self.__move_joints() 
                    # wait for motion to finish 
                self.arm_move_event.wait(10)    #10s as timeout
                send_reponse(action_server, action_server_name)

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
        print(f"Sent new joint msg to arduino: {self.commanded_angles}")


if __name__ == '__main__': 
    mc = MotionController()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        mc.publish_joint_state_msg()
        mc.update_action_servers()
        r.sleep()


