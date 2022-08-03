#!/usr/local/python3
# MQTT Topic esp/arm -> this node -> /joint_1_controller/command ...
import rospy
import rosgraph
import rostopic
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import time
from functools import partial
import numpy as np

import sys 
import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0, currentdir) 
from utils.rico_mqtt import MqttClient, MqttSubscriberCbs


class Params:
    ARM_JOINTS_NUM = 5
    HAND_JOINTS_NUM = 2
    TIME_NUM=1
    EXECUTION_PUBLISH_FREQ = 100
    
class GazeboMotionController:
    def __init__(self) -> None:
        master = rosgraph.Master('/rostopic')
        pubs, subs = rostopic.get_topic_list(master=master)
        sub_topics = [topic for topic, *_ in subs]
        # filter returns a filter object, cannot be unpacked directly
        joint_topics = list (filter(lambda topic: "controller/command" in topic, sub_topics))
        publishers = tuple(((topic, rospy.Publisher(topic, Float64, queue_size=10) ) for topic in joint_topics))
        self.arm_publishers = publishers[: -Params.HAND_JOINTS_NUM]
        self.hand_publishers = publishers[-Params.HAND_JOINTS_NUM :]
            
        print("Recording joints: ", joint_topics)
        
        self.mqtt_client = MqttClient("127.0.0.1", 1883,
                        {
                            "esp/arm": self.arm_joints_cb,
                            "esp/hand": self.hand_joint_cb
                        }, 
                        "gazebo_motion_controller")
    
        self.rate = rospy.Rate(Params.EXECUTION_PUBLISH_FREQ)
       
        self.gazebo_joint_state_sub = rospy.Subscriber("/rjje_arm_gazebo/joint_states", JointState, self.gazebo_joint_state_cb) 
        self.arm_joint_states = []
        self.hand_joint_states = []

    def gazebo_joint_state_cb(self, msg):
        '''
        Published message looks like: joint_1_name; ... | joint_1_value; ... |
        ASSUMPTION: joint_states published from Gazebo is in the same order as plan
        ''' 
        names = ";".join(msg.name)
        positions = ";".join([str(p) for p in msg.position])
        self.mqtt_client.publish("esp/joint_states", names + "|" + positions)
        self.arm_joint_states = list(msg.position[: -Params.HAND_JOINTS_NUM])
        self.hand_joint_states = list(msg.position[-Params.HAND_JOINTS_NUM :])
    
    def __execute_plans(self, userdata, msg, publishers: list, joint_states: list, apply_mimic_joint: bool = False):
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
        # TODO: Bug: not length of publishers
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

            #TODO
            print(f'{command}, {len(commanded_angles_and_time)}, {plan_length}')

        for plan in plans:
            execution_time = plan[-1]
            num_intervals = int(np.ceil(execution_time * Params.EXECUTION_PUBLISH_FREQ))
            delta_intervals = (np.array(plan[:-1]) - np.array(joint_states))/(num_intervals)
            angles_to_pub = joint_states
            for interval in range(num_intervals):
                for i in range(plan_length - 1):
                    topic, pub = publishers[i]
                    angles_to_pub[i] += + delta_intervals[i]
                    pub.publish(Float64(angles_to_pub[i]))
                self.rate.sleep()
            
    def arm_joints_cb(self, userdata, msg):
        """
        Each plan has 5 joint angles 3 digit precision + execution time. Ex: 12.3;32.2;23.0;45.4;66.2;0.02;|... , where 0.02 is the execution time*/
        """
        self.__execute_plans(userdata, msg, self.arm_publishers, self.arm_joint_states)

    def hand_joint_cb(self, userdata, msg):
        """
        Payload = 0, claw will be closed, 1 will be open
        """
        self.__execute_plans(userdata, msg, self.hand_publishers, self.hand_joint_states, apply_mimic_joint=True)
    
if __name__ == '__main__':
    rospy.init_node("gazebo_motion_controller")
    rospy.loginfo("Initializing motion controller")    
    gazebo_motion_controller = GazeboMotionController()
    rospy.spin()


