import rospy
import serial
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

def decimal_to_list(num: float) -> list: 
    """
    Round a float to one digit after decimal point, and convert a float to list. E.g., 2.48->[2,5]
    """
    num_one_decimal = round(num, 1)
    ret = []
    ret.append(math.floor(num_one_decimal))
    #extra round to prevent small numertical error in subtraction
    ret.append(round(10*(num_one_decimal - ret[0])))
    return ret

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
        self.commanded_angles = [170, 90, 180, 0, 90, 130]
        self.execution_time = 1.0 #seconds, one digit after decimal point
        self.action_server_arm = actionlib.SimpleActionServer('rjje_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction, execute_cb=self.process_arm_action, auto_start=False)    #auto_start is false, so we can start at a later time
        self.action_server_arm.start()
        rospy.loginfo("Arm action server started")
        self.action_server_gripper = actionlib.SimpleActionServer('rjje_gripper_controller/gripper_command', GripperCommandAction, execute_cb=self.process_gripper_action, auto_start=False)    #auto_start is false, so we can start at a later time
        self.action_server_gripper.start()
        rospy.loginfo("Gripper action server started")
        self.move_joints_lock = Lock()
        self.change_current_angles_lock = Lock()
        self.joint_state_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
        self.joint_state_msg = JointState()
        self.joint_state_msg.position = [90, 90, 90, 90, 90, 120]

        # #TODO
        # # Serial Ports
        self.port = rospy.get_param("~port")
        self.ser = serial.Serial(self.port, 9600, timeout=20)

    def __process_action(self, action_server: actionlib.SimpleActionServer, action_server_name: str, goal):
        """
        Actual callback for Moveit! trajectory following action request. Note that each action server runs on a separate thread
        from the main one.
        :param action_server: arm/gripper action server
        :param action_server_name: name of the action server
        :param goal: the goal trajectory from ros action client
        :return: (list_of_exection_times, list_of_commanded_angles)
        """
        # Workflow:
        #figure out joint names and their positions
        #start from point 1, since the first point is the current starting point
        #check for pre-emption
        #figure out the duration and joint positions of each trajectory segment
        #realize each segment and time it
        #check if the action has been preempted
        success = True
        traj = goal.trajectory
        num_points = len(traj.points)
        if len(self.joint_state_msg.name) == 0: 
            self.joint_state_msg.name = traj.joint_names
        commanded_angles_ls = []
        time_ls = []
        for i in range(1, num_points):
            if action_server.is_preempt_requested():
                rospy.loginfo(f"Trajectory Action Preempted on {action_server_name}" )
                self.action_server_arm.set_preempted()
                success = False
                break
            time_ls.append((traj.points[i].time_from_start - traj.points[i-1].time_from_start).to_sec())
            commanded_angles_ls.append(self.__convert_to_rjje_command_angles(traj.points[i].positions))
        return time_ls, commanded_angles_ls

    def __convert_to_rjje_command_angles(self, ros_commanded_angles: list): 
        return [180 * (angle/3.1415) + 90 for angle in ros_commanded_angles]

    def process_gripper_action(self, goal): 
        time_ls, commanded_angles_ls = self.__process_action(self.action_server_gripper, "rjje_gripper", goal)
        #TODO
        print(time_ls, commanded_angles_ls)
        send_reponse(self.action_server_arm, "rjje_gripper")

    def process_arm_action(self, goal): 
        time_ls, commanded_angles_ls = self.__process_action(self.action_server_arm,  "rjje_arm", goal)
        for time, commanded_angles in zip(time_ls, commanded_angles_ls): 
           self.commanded_angles[:5] = list(commanded_angles)
           self.execution_time = time 
           with self.move_joints_lock: 
               self.move_joints()
        send_reponse(self.action_server_arm, "rjje_arm")

    def move_joints(self): 
        # minimalist "service" provided by arduino: 
        # Python sends joint angles, execution time, and gets a response back
        def send_int_in_list(ls: list): 
            raw_bytes = b''
            for integer in ls: 
                raw_bytes += (integer).to_bytes(1,"little") 
            self.ser.write(raw_bytes)

        commanded_angles_in_list = []
        for angle in self.commanded_angles: 
            commanded_angles_in_list.extend(decimal_to_list(angle))

        send_int_in_list(commanded_angles_in_list + decimal_to_list(self.execution_time))

        # Response from Arduino: success = 1, fail = 0
        angles = []
        for i in range(6): 
            int_part = ord(self.ser.read(1))
            two_decimal_part = ord(self.ser.read(1))
            angles.append(int_part + two_decimal_part * 0.01)

        with self.change_current_angles_lock: 
            self.joint_state_msg.position= angles

        #TODO
        print(self.joint_state_msg)
    def publish_joint_angles(self): 
        with self.change_current_angles_lock: 
            self.joint_state_pub.publish(self.joint_state_msg)


if __name__ == '__main__': 
    mc = MotionController()
    mc.publish_joint_angles()
    rospy.spinOnce()


