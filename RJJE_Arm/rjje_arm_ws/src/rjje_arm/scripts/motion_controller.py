"""
Motion controller that can control a robot arm and a gripper independently, in a soft real-time fashion, ~2ms 
Internally, the arm and gripper shares the same message to the uC. And their status is updated in a 
lock-free manner.
"""
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
        self.commanded_angles = [90, 90, 90, 90, 90, 120]
        self.execution_time = 1.0 #seconds, one digit after decimal point
        # in CPython, bool write is atomic.
        self.arm_to_move = False
        self.gripper_to_move = True
        self.ANGULAR_THRESHOLD = 0.05

        self.action_server_arm = actionlib.SimpleActionServer('rjje_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction, execute_cb=self.process_arm_action, auto_start=False)    #auto_start is false, so we can start at a later time
        self.action_server_arm.start()
        rospy.loginfo("Arm action server started")

        self.action_server_gripper = actionlib.SimpleActionServer('rjje_gripper_controller/gripper_command', GripperCommandAction, execute_cb=self.process_gripper_action, auto_start=False)    #auto_start is false, so we can start at a later time
        self.action_server_gripper.start()
        rospy.loginfo("Gripper action server started")

        self.move_joints_lock = Lock()
        self.joint_state_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
        self.joint_state_msg = JointState()
        self.joint_state_msg.position = self.commanded_angles
        #TODO: to change to soft-coded way
        self.joint_state_msg.name = ["link1_bracket_1", "link_2_bracket_2_1", "bracket_2_2_link_3", "bracket_3_2_link_4", "link_5_link_6", "link_6_left_gripper", "link_6_right_gripper"]
        rospy.loginfo("joint state publisher set up")

        # Serial Ports
        self.port = rospy.get_param("~port")
        self.ser = serial.Serial(self.port, 9600, timeout=20)
        self.reset_arduino()
        rospy.loginfo("Serial port open, now listening to uC")

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

    def __convert_to_joing_msg_angles(self, commanded_angles: list): 
        return [3.1415 * ((angle-90.0)/180.0) for angle in commanded_angles]

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
           print("to acquire")
           with self.move_joints_lock: 
               #TODO
               print("moving joints")
               self.move_joints()
        # wait for motion to finish
        self.arm_to_move = True
        #TODO
        print("waiting to move")
        while (self.arm_to_move): 
            pass
        send_reponse(self.action_server_arm, "rjje_arm")

    def reset_arduino(self): 
        self.__send_message_header("RESET")

    def __send_message_header(self, header): 
        """
        message contract: 1 byte HEADER| 12 bytes commanded_angles | 2 byte EXECUTION_TIME
        header: b'11111111' = RESET, b'0' = OKAY
        """
        if header == "RESET": 
            #TODO
            print("sent reset")
            byte = (0xff).to_bytes(1, "little")
        elif header == "OKAY": 
            byte = (0x00).to_bytes(1, "little")
        self.ser.write(byte)

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
        self.__send_message_header("OKAY")
        send_int_in_list(commanded_angles_in_list + decimal_to_list(self.execution_time))
        #TODO
        print(f"moved joints: {self.commanded_angles}")


    def update_servers_and_publish_joint_angles(self): 
        # read angles from arduino (two_decimal places)
        for i in range(6): 
            int_part = ord(self.ser.read(1))
            two_decimal_part = ord(self.ser.read(1))
            self.joint_state_msg.position[i] = int_part + two_decimal_part * 0.01
            #TODO
            print("int: ", int_part)
        # self.__convert_to_joing_msg_angles(self.joint_state_msg.position)
        self.joint_state_pub.publish(self.joint_state_msg)

        if self.arm_to_move: 
            #now the arm action server must be waiting 
            if abs(self.joint_state_msg.position[:5] - self.commanded_angles[:5]) < self.ANGULAR_THRESHOLD: 
                self.arm_to_move = False
        if self.gripper_to_move: 
            if abs(self.joint_state_msg.position[5] - self.commanded_angles[5]) < self.ANGULAR_THRESHOLD: 
                self.gripper_to_move = False

if __name__ == '__main__': 
    mc = MotionController()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        mc.update_servers_and_publish_joint_angles()
        r.sleep()


