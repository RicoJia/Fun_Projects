import rospy
import serial
import math
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryFeedback
from control_msgs.msg import FollowJointTrajectoryResult
from control_msgs.msg import GripperCommandAction
from control_msgs.msg import GripperCommandFeedback
from control_msgs.msg import GripperCommandResult
import actionlib

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


class MotionController: 
    def __init__(self): 
        rospy.init_node("motion_controller", anonymous=True)     #anonymous=true ensures unique node name by adding random numbers
        # one digit after decimal point
        self.commanded_angles = [170, 90, 180, 0, 90, 130]
        self.execution_time = 1.0 #seconds, one digit after decimal point
        self.action_server_arm = actionlib.SimpleActionServer('rjje_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction, execute_cb=self.follow_trajectory, auto_start=False)    #auto_start is false, so we can start at a later time
        self.action_server_arm.start()
        rospy.loginfo("Arm action server started")
        self.action_server_gripper = actionlib.SimpleActionServer('rjje_gripper_controller/gripper_command', GripperCommandAction, execute_cb=self.follow_trajectory, auto_start=False)    #auto_start is false, so we can start at a later time
        self.action_server_gripper.start()

        # #TODO
        # # Serial Ports
        # self.port = rospy.get_param("~port")
        # self.ser = serial.Serial(self.port, 9600, timeout=20)

    def gripper_command(self,goal): 
        success = True
        traj = goal.trajectory
        num_points = len(traj.points)
        traj_joint_names = traj.joint_names
        #TODO
        print(traj_joint_names)
        for i in range(1, num_points):
            if self.action_server_arm.is_preempt_requested():
                rospy.loginfo("Trajectory Action Preempted on rjje gripper" )
                self.action_server_arm.set_preempted()
                success = False
                break
            duration = (traj.points[i].time_from_start - traj.points[i-1].time_from_start ).to_sec()
        #TODO 
        if success: 
            msg = 'gripper Trajectory completed'
            rospy.loginfo(msg)
            res = FollowJointTrajectoryResult()
            self.action_server_gripper.set_succeeded(result=res, text=msg)

    def follow_trajectory(self, goal): 
        """
        Call back function for Moveit! trajectory following action request. Note that this is a separate thread
        from the main one.
        :param goal: the goal trajectory of actuated joints, i.e, phi angles.
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
        traj_joint_names = traj.joint_names
        #TODO
        print(traj.points)
        for i in range(1, num_points):
            if self.action_server_arm.is_preempt_requested():
                rospy.loginfo("Trajectory Action Preempted on rjje arm" )
                self.action_server_arm.set_preempted()
                success = False
                break
            duration = (traj.points[i].time_from_start - traj.points[i-1].time_from_start ).to_sec()
        #TODO 
        if success: 
            msg = 'Trajectory completed'
            rospy.loginfo(msg)
            res = FollowJointTrajectoryResult()
            self.action_server_arm.set_succeeded(result=res, text=msg)

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
        res = self.ser.read(1)
        print("res:",res)


if __name__ == '__main__': 
    mc = MotionController()
    #TODO
    # mc.move_joints()
    rospy.spin()


