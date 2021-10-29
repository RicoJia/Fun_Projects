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
    #extra round to prevent small numertical error in subtraction
    ret.append(round(10*(num_one_decimal - ret[0])))
    return ret


class MotionController: 
    def __init__(self): 
        rospy.init_node("motion_controller", anonymous=True)     #anonymous=true ensures unique node name by adding random numbers
        self.port = rospy.get_param("~port")
        self.ser = serial.Serial(self.port, 9600, timeout=20)
        # one digit after decimal point
        self.commanded_angles = [170, 90, 180, 0, 90, 130]
        self.execution_time = 1.0 #seconds, one digit after decimal point

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
    mc.move_joints()


