import rospy
from std_msgs.msg import String
from rjje_arm.msg import MotionControl
from sensor_msgs.msg import JointState

class MotionController: 
    def __init__(self): 
        self.pub = rospy.Publisher("motion_control_msg", MotionControl, queue_size=10)
        rospy.init_node("motion_controller", anonymous=True)     #anonymous=true ensures unique node name by adding random numbers
        self.motion_control_msg = MotionControl()
        self.joint_state_sub = rospy.Subscriber("/joint_states", JointState, self.sub_joint_state_cb)

    def pub_motion_control_msg(self): 
        self.pub.publish(self.motion_control_msg)
    
    def sub_joint_state_cb(self, msg): 
        self.motion_control_msg.commanded_angles = []
        for angle in (msg.position):
            self.motion_control_msg.commanded_angles.append(round(angle * 180.0/3.1415926 + 90))

if __name__ == '__main__': 
    mc = MotionController()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown(): 
        mc.pub_motion_control_msg()
        rate.sleep() #rospy.sleep vs rate.sleep() - 你尽量应该用rate。sleep， 因为这个比较准确！


