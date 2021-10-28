import rospy
from std_msgs.msg import String
from rjje_arm.srv import MotionControl
from sensor_msgs.msg import JointState

class MotionController: 
    def __init__(self): 
        rospy.init_node("motion_controller", anonymous=True)     #anonymous=true ensures unique node name by adding random numbers
        print("waiting for service")
        rospy.wait_for_service('/move_joints_service')
        self.move_joints = rospy.ServiceProxy('/move_joints_service', MotionControl)

    def call_move_joints_srv(self): 
        commanded_angles = [90.0, 90.0, 90.0, 60.0, 90.0, 120.0]
        execution_time = 2    #2s
        try: 
            self.move_joints(commanded_angles, execution_time)
            print("Done")
        except rospy.ServiceException as exc: 
            print("Service cannot proceed:", exc)


if __name__ == '__main__': 
    mc = MotionController()
    mc.call_move_joints_srv()


