"""
Motion controller that can control a robot arm and a gripper independently, in a soft real-time fashion, ~2ms 
Internally, the arm and gripper shares the same message to the uC. And their status is updated in a lock-free manner.
On arduino, all motors go from (0, 180) following the right-hand convention, with the neutral position = 90. 
1. Modes of Operations: 
    1. Teaching mode: /rjje_arm/motion_control is [x,x,x,x,x, 361]. To turn it off, [x,x,x,x,x, 362]. In teaching mode, joint values are recorded.

2. IO:
    - Inputs:
        1. esp/joint_states: MQTT topic from robot
    - Outputs:
        1. /joint_states: ROS topic of robot joint states
3. For testing with mosquito:
    - mosquitto_sub -d -t "esp/arm"
"""
import rospy
import numpy as np
import time
import math
import threading
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryFeedback
from control_msgs.msg import FollowJointTrajectoryResult
from control_msgs.msg import GripperCommandAction
from control_msgs.msg import GripperCommandFeedback
from control_msgs.msg import GripperCommandResult
from sensor_msgs.msg import JointState
import actionlib

import sys 
import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0, currentdir) 
from utils.rico_mqtt import MqttClient, MqttSubscriberCbs

ARM_ACTION_SERVER = "rjje_arm"
GRIPPER_ACTION_SERVER = "rjje_gripper"
MODE_SWITCH_SERVICE = "rjje_mode_switch"

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
    elif status == "ABORTED":
        action_server.setAborted() 

class Constants:
    NODE_NAME = "motion_controller"
    BROKER_IP = "127.0.0.1"
    BROKER_PORT = 1883

class MotionController:
    def __init__(self):
        self.joint_state_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
        self.mqtt_client = MqttClient(
            Constants.BROKER_IP,
            Constants.BROKER_PORT,
            {
                "esp/joint_states": self.joint_state_cb
            },
            Constants.NODE_NAME
        )
        self.joint_state_msg = JointState()
        self.joint_state_msg.header.frame_id = ''
        self.joint_state_msg.header.stamp = rospy.get_rostime()
        # self.joint_state_msg.header.seq = some_num
        
        # index (our internal representation -> joint moveit's trajectory )
        self.traj_point_joint_name_lookup = []
        rospy.loginfo("joint state lookup has been set up")

        # one digit after decimal point
        self.commanded_angles = [90, 90, 90, 90, 90, 90]
        self.execution_time = 1.0 #seconds, one digit after decimal point

        self.action_server_arm = actionlib.SimpleActionServer('/rjje_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction, execute_cb=self.process_arm_action, auto_start=False)    #auto_start is false, so we can start at a later time
        self.action_server_arm.start()
        rospy.loginfo("Arm action server started")

        self.action_server_gripper = actionlib.SimpleActionServer('/rjje_gripper_controller/gripper_command', GripperCommandAction, execute_cb=self.process_gripper_action, auto_start=False)    #auto_start is false, so we can start at a later time
        self.action_server_gripper.start()
        rospy.loginfo("Gripper action server started")

    def process_gripper_action(self, goal): 
        self.__process_action(GRIPPER_ACTION_SERVER, goal)

    def process_arm_action(self, goal): 
        self.__process_action(ARM_ACTION_SERVER, goal)

    def __process_action(self, action_server_name: str, goal):
        """
        Actual callback for Moveit! trajectory following action request. 
        Note that each action server runs on ros topics (effectively single-threaded)
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

        # we need joint_state being published
        if len(self.joint_state_msg.name) == 0:
            rospy.logwarn(f"failed to execute action since joint msg has not been received")
            send_reponse(action_server,action_server_name, "ABORTED")
            return 
        traj = goal.trajectory
        num_points = len(traj.points)

        # TODO: potential bug - Assumption: MQTT topics for joint angle exeuction follows the same order as joint_states
        # trajectory joint_names might be in a different order than joint_states
        # So we need to publish joints in the same order
        
        self.calculate_arm_joint_state_indices(traj)
        self.publish_mqtt_string(traj)
        print(self.arm_joint_state_indices)
        # parse joint_state_indices
        # send them out 
        # time it
        total_time = traj.points[-1].time_from_start.to_sec()
        rospy.loginfo("Executing plan")
        time.sleep(total_time)
        send_reponse(action_server, action_server_name, "SUCCESSFUL")
        rospy.loginfo("Successful action execution")
 
    def publish_mqtt_string(self,traj):
        mqtt_string = ""
        last_time = 0
        for pt in traj.points[1:]:
            values = list(map(lambda i: pt.positions[i], self.arm_joint_state_indices))
            current_time = pt.time_from_start.to_sec() 
            interval = current_time-last_time
            value_str = ";".join(str(v) for v in values) + ";" + str(interval) + "|"
            mqtt_string += value_str
            last_time = current_time
        #TODO
        print("mqtt:"+mqtt_string)
        self.mqtt_client.publish("esp/arm", mqtt_string)

            
    def calculate_arm_joint_state_indices(self, traj):
        '''
        return indices of traj.points in /joint_states 
        '''
        if not hasattr(self, "arm_joint_state_indices"):
            self.arm_joint_state_indices = []
            for name in traj.joint_names:
                self.arm_joint_state_indices.append(
                    self.joint_state_msg.name.index(name)
                )
                
    def joint_state_cb(self, userdata, msg):
        joint_names, position = msg.payload.decode("utf-8").split("|")
        if len(self.joint_state_msg.name) == 0:
            self.joint_state_msg.name = joint_names.split(";")
        self.joint_state_msg.position = MqttSubscriberCbs.get_array_from_string(position.split(";"))
        self.joint_state_msg.header.stamp = rospy.Time.now()
        self.joint_state_pub.publish(self.joint_state_msg)

if __name__ == '__main__': 
    rospy.init_node(Constants.NODE_NAME)
    Constants.BROKER_IP = rospy.get_param("~broker_ip")
    rospy.loginfo("================================================================" )
    rospy.loginfo("Started: " + Constants.NODE_NAME)
    rospy.loginfo("Broker IP: " + Constants.BROKER_IP)
    mc = MotionController()
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        # mc.update_action_servers()

        r.sleep()


