#!/usr/local/python3
# MQTT Topic esp/plan -> this node -> /joint_1_controller/command ...
import rospy
import rosgraph
import rostopic
from std_msgs.msg import Float64
import sys 
import os
import time
import inspect
from functools import partial

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
            
        print(joint_topics, self.publishers)
        
        self.mqtt_client = MqttClient("127.0.0.1", 1883,
                        {
                            "esp/plan": self.plan_cb,
                            "esp/hand": self.hand_cb
                        }, 
                        "gazebo_motion_controller")
    
        self.rate = rospy.Rate(Params.EXECUTION_PUBLISH_FREQ)
        
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
            for i in range(Params.JOINT_NUM):
                topic, pub = self.publishers[i]
                pub.publish(Float64(plan[i]))
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


