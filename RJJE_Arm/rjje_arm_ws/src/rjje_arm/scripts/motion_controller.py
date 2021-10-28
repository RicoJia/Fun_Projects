import rospy
import serial
import math

def decimal_to_list(num: float) -> list: 
    """
    Round a float to one digit after decimal point, and convert a float to list. E.g., 2.48->[2,5]
    """
    num_one_decimal = round(num, 1)
    ret = []
    ret.append(math.floor(num_one_decimal))
    ret.append(int(10*(num_one_decimal - ret[0])))
    return ret


class MotionController: 
    def __init__(self): 
        rospy.init_node("motion_controller", anonymous=True)     #anonymous=true ensures unique node name by adding random numbers
        self.port = rospy.get_param("~port")
        self.ser = serial.Serial(self.port, 9600, timeout=1)
        self.commanded_angles = [10, 20, 30, 60, 50, 120]
        self.execution_time = 2.2 #s, one digit after decimal point

    def move_joints(self): 
        # minimalist "service" provided by arduino: 
        # Python sends joint angles, execution time, and gets a response back
        def send_int_in_list(ls: list): 
            raw_bytes = b''
            for integer in ls: 
                raw_bytes += (integer).to_bytes(1,"little") 
            self.ser.write(raw_bytes)

        send_int_in_list(self.commanded_angles + decimal_to_list(self.execution_time))

        # Response from Arduino: success = 1, fail = 0
        # res = self.ser.read(1)
        # print("res:",res)


if __name__ == '__main__': 
    mc = MotionController()
    mc.move_joints()


