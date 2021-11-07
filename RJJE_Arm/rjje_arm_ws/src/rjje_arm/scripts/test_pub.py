import rospy
import numpy as np
from rjje_arm.msg import MotionControl


rospy.init_node("test_pub", anonymous=True)     #anonymous=true ensures unique node name by adding random numbers
pub = rospy.Publisher("rjje_arm/motion_control", MotionControl, queue_size=10)
r = rospy.Rate(1)
msg = MotionControl()
while not rospy.is_shutdown():
    # msg.commanded_angles = [100, 120, 130, 40, 50]
    msg.task_id = 9
    msg.c0 = 100
    msg.c1 = 10
    msg.c2 = 10
    msg.c3 = 90
    msg.c4 = 10
    msg.c5 = 60
    msg.execution_time = 1
    pub.publish(msg)
    r.sleep()



