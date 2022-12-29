#!/usr/local/python3
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import time

import sys 
import os
import inspect
import re
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0, currentdir) 
from utils.rico_mqtt import MqttClient, MqttSubscriberCbs

def sort_ls_based_on_indices(ls: list, indices: list):
    zipped_sorted_ls = sorted(zip(ls, indices), key=lambda zipped: zipped[1])
    return [z[0] for z in zipped_sorted_ls]
class Params:
    ARM_JOINTS_NUM = 5
    HAND_JOINTS_NUM = 2
    TIME_NUM=1
    EXECUTION_PUBLISH_FREQ = 100

class GazeboMotionController:
    """
    Control flows:
    1. motion controller -> MQTT Topic esp/arm, esp/hand -> this node -> /joint_1_controller/command 
        -> Gazebo Ros Controller
    2. Gazebo -> this node -> esp/joint_states -> motion_controller
    """
    def __init__(self) -> None:
        """
        1. This is currently an ugly implementation of MQTT communication. Protobuf serialization 
        should be implemented, ideally. 
        """
        # 1. Read joint names from rosparam, which is originally from gazebo ROS control
        # like /joint_6_controller/joint
        ros_params = rospy.get_param_names()
        # Here we have names such as /joint_3_controller repeating multiple times
        joint_controller_names = set(re.findall("/joint_._controller", "|".join(ros_params)))
        self.real_joint_names = [rospy.get_param(p+"/joint") for p in joint_controller_names]
        self.real_joint_names.sort()
        # ASSUMPTION: hand joints have higher joint index than arm joints

        # 2. Create arm and hand publishers
        joint_topics = [p+"/command" for p in joint_controller_names]
        publishers = tuple(((topic, rospy.Publisher(topic, Float64, queue_size=10) ) for topic in joint_topics))
        self.arm_publishers = publishers[: -Params.HAND_JOINTS_NUM]
        self.hand_publishers = publishers[-Params.HAND_JOINTS_NUM :]
        print("Joint control topics: ", joint_topics)
        
        self.mqtt_client = MqttClient("127.0.0.1", 1883,
                        {
                            "esp/arm": self.arm_joints_cb,
                            "esp/hand": self.hand_joint_cb
                        }, 
                        "gazebo_motion_controller")
    
        self.rate = rospy.Rate(Params.EXECUTION_PUBLISH_FREQ)
       
        self.gazebo_joint_state_sub = rospy.Subscriber("/joint_states", JointState, self.gazebo_joint_state_cb) 
        self.gazebo_js_indices = []

    def arm_joints_cb(self, userdata, msg):
        """
        Each plan has 5 joint angles 3 digit precision + execution time. Ex: 12.3;32.2;23.0;45.4;66.2;0.02;|... , where 0.02 is the execution time*/
        """
        self.__execute_plans(userdata, msg, self.arm_publishers)

    def hand_joint_cb(self, userdata, msg):
        """
        Payload = 0, claw will be closed, 1 will be open
        """
        self.__execute_plans(userdata, msg, self.hand_publishers, apply_mimic_joint=True)
        
    def gazebo_joint_state_cb(self, msg):
        '''
        Published message looks like: joint_1_name; ... | joint_1_value; ... |
        ASSUMPTION: joint_states published from Gazebo is in the same order as plan
        ''' 
        gazebo_joint_names_indices = [self.real_joint_names.index(js) for js in msg.name]
        gazebo_joint_names = sort_ls_based_on_indices(msg.name, gazebo_joint_names_indices)
        gazebo_joint_names = ";".join(gazebo_joint_names)

        print(gazebo_joint_names, "|", self.real_joint_names, "|| indices:", gazebo_joint_names_indices)

        positions = sort_ls_based_on_indices(msg.position, gazebo_joint_names_indices)
        positions = ";".join([str(p) for p in positions])
        self.mqtt_client.publish("esp/joint_states", gazebo_joint_names + "|" + positions)
    
    def __execute_plans(self, userdata, msg, publishers: list, apply_mimic_joint: bool = False):
        """
        Parse plans that look like: val_a1; val_a2; ... time_1;| val_b1, ... time_2;| ... 
        Then execute each plan according to its time
        note that a copy of all these data is executed in the MQTT callback
        Args:
            userdata (_type_): Custom User data 
            msg (_type_): Actual MQTT Message
            publishers (list): Length of each plan
            apply_mimic_joint (bool): Hardcoded patch for simulate mimic joint issues

        Returns:
        """
        plans = []
        plan_length = len(publishers) + 1
        commands = str(msg.payload.decode("utf-8")).split("|")
        for command in commands:
            commanded_angles_and_time = MqttSubscriberCbs.get_array_from_string(command.split(";"))
            if len(commanded_angles_and_time) == plan_length: 
                plans.append(commanded_angles_and_time)
            elif apply_mimic_joint and len(commanded_angles_and_time) != 0:
                new_angle = commanded_angles_and_time[0] * -1
                new_commanded_angles_and_time = [commanded_angles_and_time[0], new_angle, commanded_angles_and_time[1]]
                plans.append(new_commanded_angles_and_time) 

        for plan in plans:
            execution_time = plan[-1]
            for i in range(plan_length - 1):
                topic, pub = publishers[i]
                pub.publish(Float64(plan[i]))
            time.sleep(execution_time) 
    
if __name__ == '__main__':
    rospy.init_node("gazebo_motion_controller")
    rospy.loginfo("Initializing motion controller")    
    gazebo_motion_controller = GazeboMotionController()
    rospy.spin()


