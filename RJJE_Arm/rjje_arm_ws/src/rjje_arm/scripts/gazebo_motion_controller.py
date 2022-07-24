#!/usr/local/python3
# MQTT Topic esp/plan -> this node -> /joint_1_controller/command ...
import rospy
import rosgraph
import rostopic
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import sys 
import os
import time
import inspect
from functools import partial
import numpy as np

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0, currentdir) 
from utils.rico_mqtt import MqttClient, MqttSubscriberCbs


class Params:
    JOINT_NUM = 5
    TIME_NUM=1
    EXECUTION_PUBLISH_FREQ = 100
    
class GazeboMotionController:
    def __init__(self) -> None:
        master = rosgraph.Master('/rostopic')
        pubs, subs = rostopic.get_topic_list(master=master)
        sub_topics = [topic for topic, *_ in subs]
        # filter returns a filter object, cannot be unpacked directly
        joint_topics = list (filter(lambda topic: "command" in topic, sub_topics))
        self.publishers = tuple(((topic, rospy.Publisher(topic, Float64, queue_size=10) ) for topic in joint_topics))
            
        print("Recording joints: ", joint_topics)
        
        self.mqtt_client = MqttClient("127.0.0.1", 1883,
                        {
                            "esp/plan": self.plan_cb,
                            "esp/hand": self.hand_cb
                        }, 
                        "gazebo_motion_controller")
    
        self.rate = rospy.Rate(Params.EXECUTION_PUBLISH_FREQ)
       
        self.gazebo_joint_state_sub = rospy.Subscriber("/rjje_arm_gazebo/joint_states", JointState, self.gazebo_joint_state_cb) 
        self.joint_states = []

    def gazebo_joint_state_cb(self, msg):
        '''
        Published message looks like: joint_1_name; ... | joint_1_value; ... |
        ASSUMPTION: joint_states published from Gazebo is in the same order as plan
        ''' 
        names = ";".join(msg.name)
        positions = ";".join([str(p) for p in msg.position])
        self.mqtt_client.publish("esp/joint_states", names + "|" + positions)
        self.joint_states = msg.position
            
    def plan_cb(self, userdata, msg):
        """
        Each plan has 5 joint angles 3 digit precision + execution time. Ex: 12.3;32.2;23.0;45.4;66.2;0.02;|... , where 0.02 is the execution time*/
        """
        plans = []
        commands = str(msg.payload.decode("utf-8")).split("|")
        for command in commands:
            commanded_angles = MqttSubscriberCbs.get_array_from_string(command.split(";"))
            if len(commanded_angles) == Params.JOINT_NUM + Params.TIME_NUM:
                plans.append(commanded_angles)
        #TODO - to finish smooth execution
        print(plans)
        for plan in plans:
            execution_time = plan[Params.JOINT_NUM]
            num_intervals = int(np.ceil(execution_time * Params.EXECUTION_PUBLISH_FREQ))
            delta_intervals = (np.array(plan[:Params.JOINT_NUM]) - np.array(self.joint_states[:Params.JOINT_NUM]))/num_intervals
            for interval in range(num_intervals):
                for i in range(Params.JOINT_NUM):
                    topic, pub = self.publishers[i]
                    angle_to_pub = plan[i] - (num_intervals - interval + 1) * delta_intervals[i]
                    # print(plan[i])
                    pub.publish(Float64(angle_to_pub))
                self.rate.sleep()

    def hand_cb(self, userdata, msg):
        """
        Payload = 0, claw will be closed, 1 will be open
        """
        pass
    
if __name__ == '__main__':
    rospy.init_node("gazebo_motion_controller")
    rospy.loginfo("Initializing motion controller")    
    gazebo_motion_controller = GazeboMotionController()
    rospy.spin()


