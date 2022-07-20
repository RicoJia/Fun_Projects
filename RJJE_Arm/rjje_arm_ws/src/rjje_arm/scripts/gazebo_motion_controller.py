#!/usr/local/python3
# MQTT Topic esp/plan -> this node -> /joint_1_controller/command ...
import rospy
import rosgraph
import rostopic

if __name__ == '__main__':
    rospy.init_node("gazebo_motion_controller")
    rospy.loginfo("Initializing motion controller")    
    master = rosgraph.Master('/rostopic')
    pubs, subs = rostopic.get_topic_list(master=master)
    sub_topics = [topic for topic, *_ in subs]
    # filter returns a filter object, cannot be unpacked directly
    joint_topics = list (filter(lambda topic: "command" in topic, sub_topics))
    print(joint_topics)


