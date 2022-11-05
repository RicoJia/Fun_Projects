# RJJE Arm
Introduction
------
1. Objectives
    1. 3D Description and Simulation of an RJJE Arm
    2. Rviz & Programmatic for Moveit! Motion planning 
    3. WIFI-based motor control of the physical RJJE Arm 
2. The [System diagram can be accessed using draw.io](https://drive.google.com/file/d/1ujubSrS_AvXeORWJ76qhUnCQ4BP0E4v_/view?usp=sharing) 

## Usage
### Firmware Setup
------
1. Open Arduino IDE, install these libraries using ```library manager```
    - ```Adafruit_PWMServoDriver```
    - ```EspMQTTClient```
2. On Arduino IDE, setup ESP32: [see here](https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/)
3. To Control the claw, do ```rosservice call /rjje_arm/claw_control OPEN_VAL```, where ```OPEN_VAL``` = 0 for closing, 1 for opening
### Software Setup
4. To visualize the robot only, you can see the robot with 
    - In Gazebo, ```./build_and_visualize.sh gazebo``` 
    - In Rviz: ```./build_and_visualize.sh rviz```
    - **Launch the full project**: ```    ./build_and_visualize.sh```

## Firmware Design
### Motors
1. Simple Motor Testing: [Working Adafruit_PCA9685 video, including schematics](https://www.youtube.com/watch?v=y8X9X10Tn1k)
    - Electrical: 
        - Jack plug for external power 
        - Vcc is positive for signal, V+ is the positive supply. 
        - switching directions will cause a lot of noise on the supply. 
        - [Optional]may need a cap, like 470 uF for many motors.  
    - Some notes: 
        1. The working frequency is around 60hz, not 50hz
        2. Some brands of PCA9685 requires V+ to be connected to 5v.
    - on ESP32: [Nice article on ESP 32 - PCA9865](https://dronebotworkshop.com/esp32-servo/)  

        <p align="center">
        <img src="https://user-images.githubusercontent.com/106101331/175851195-075cdfe5-a3cd-4bd5-86f6-e74e4d874305.png" height="400" width="width"/>
        <figcaption align="center">Pinouts of ESP32</figcaption>
        </p>

        - Arduino Nano -> ESP32 pinouts:
            ```
            Analog 4 (GPIO32) -> SDA (GPIO 21)
            Analog 5 (GPIO33) -> SCL (GPIO 22)
            ```
        - ESP 32 connections -> PCA9685
            ``` 
            VIN -> Vcc (5v)
            GND -> GND
            D21 -> SDA
            D22 -> SCL
            ```
    - The rudder on top of a servo motor has holes that need to be rethreaded. One can use a "tap" for rethreading.
        
2. Servo Calibration.
    - [How to calibrate servos](https://create.arduino.cc/projecthub/jeremy-lindsay/calibrating-my-servos-fa27ce)
    - the servo's orientation needs to be adjusted to 90 degrees. The orientation of a servo (from 0 to 180 degrees) is shown below
        <p align="center">
        <img src="https://user-images.githubusercontent.com/39393023/135568554-f84da7c6-10e5-4773-9298-33f507092285.JPEG" height="400" width="width"/>
        <figcaption align="center">Note: the rudder is on the right hand side of the servo</figcaption>
        </p>

    - Some servos work well in the range ```[10, 170]``` degrees. [Source](https://www.intorobotics.com/how-to-control-servo-motors-with-arduino-no-noise-no-vibration/)

    - Servos take PWM signals as inputs. The output torque rating goes higher if the input voltage is higher.

3. Servo Motors Trouble Shooting
    1. Use the [simple servo script](https://docs.arduino.cc/learn/electronics/servo-motors) to test if the servo microcontroller are good
    2. **Use I2C tool if you have trouble connecting to PCA9865**. I2C Tool can scan I2C devices. Or SMBus devices (protocal based on I2C, a two wire communication). [Link](https://www.arduino.cc/reference/en/libraries/i2cdetect/)
    3. Get spare motors and microcontrollers. If servos are still not working, try different Servo control script on the internet as well 
    4. **If you have a servo that vibrates, here are some possible reasons: ** [Source](https://electronicguidebook.com/reasons-why-a-servo-motor-vibrates/)
        - Power is instable, or not adequate. So get a larger power supply, or power cable
        - Working near the range limits
        - Motors are cheap so that parts don't work properly. **In my case, some motors did work better than others. I think that's because the good ones have better correspondence to the internal PID control loop**
    5. Analog Servos' feedback can have around 1 degree of error. Which can introduce jittering in replay
    6. If motors start to have large oscillations, and power LEDs blink, then make sure no other appliances are connected to the power source

### WIFI
1. WIFI Connection
    1. RJJE arm operates using an ESP32 microcontroller, which comes with a WIFI module. For communication, MQTT protocal is used. 
        - In a speed test, 1 request(30-byte)-response(30-byte) takes ~0.01s to finish. The router's upload and & download speeds are around ~130 & ~150 mbps. Therefore, we are using WIFI for real time robot arm control.
### Development Notes 
1. MQTT Speed test Result
    1. Test setup: a wifi network with upload / Download speed at 150Mb/s. We test the average total time of a "handshake", i.e., publish a message bidirectionally. 
    2. Results:
        - Need to get pub/sub on host machine (~0.07s per message exchange (30 bytes), but might take longer)
2. For servo controller, I recommend 

RJJE Arm Modelling
------
This file is the design documentation of rjje_arm. 

### 3D CAD Modelling
1. STL file example - https://create.arduino.cc/projecthub/danny-van-den-heuvel/6dof-robotic-arm-50eab6
    - Illustration of the robot at neutral position
        ![Screenshot from 2022-09-03 17-13-32](https://user-images.githubusercontent.com/106101331/188289152-804a9ba5-a919-4928-9aa1-a9497506350b.png)

2. Modelling 
    - First, obtain the STL files of major parts created by Rico Jia. Note that these STL files were created using Onshape, and some not-important details are omitted. 
    - When assembling STL models into a full 3D model of the robot, special attention should be paid to: 
        1. In general, ROS follows multiple ways to express rrotation with angles. [See here](https://www.ros.org/reps/rep-0103.html). In URDF, it's **Z-Y-X** Euler angle
    - **One discovery we found was that our STL file does not specify dimensions.** I used onshape to generate it, and set dimensions to centimeters. So in Rviz and Gazebo, they are assumed to be meters. One thing we have to do is to scale the model down to centimenters in ```xacro``` files. 

### Gazebo Simulation 
1.  The Gazebo simulation is a behavior level abstraction of the physical RJJE arm. It is used as a tool to validate the correctness of the software. 
    1.  Below behaviours are simulated:
        1.  RJJE physical simulation ```URDF -> SDF```
        2.  MQTT command interface of the RJJE arm controller
            1.  ROS control is used to control the robot  
    2.  Below behaviours are not simulated: 
        1.  Problems in MQTT Orchestration
        2. Failure in RJJE Arm hardware failure  
2. Third party [Gazebo Joint State publisher](https://github.com/yossioo/gazebo_ros_joints_publisher) plugin is being used to publish ```/rjje_arm_gazebo/joint_states``` directly from Gazebo, which gets relayed into ```Gazebo_Motion_Controller```
   1. This is installed as a git submodule
3. Assumptions 
   1. Gazebo does not support mimic joints. Therefore for the hand, we simulate mimic joints by copying the joint command for the driving motor. 
4. Performance
   1. In Gazebo, there might be a delay coming from communication & control execution (~10%), for both the arm and the hand. In the future, this could be improved. 
   
RJJE Arm Motion Planning
------
This file is the design documentation of rjje_arm. Currently, we are able to do motion planning using the default ```moveit_setup_assistant``` 

### Motion Planning 
<p align="center">
<img src="https://user-images.githubusercontent.com/106101331/180908988-571c55b2-4fdf-4a24-affa-a226fe67df6d.png" height="400" width="width"/>
</p>

1. Requirements
    - Smooth motion execution (D)
        - debug messaging system
    - Teaching mode
    - Task contructor
2. According to this [post](https://answers.ros.org/question/313637/openclose-end-effector-with-moveit-rviz/), gripper in Moveit is controlled differently. So there's no ball-and-arrow at eef. Therefore we control the gripper separately

3. ```motion_controller``` 
   1. Inputs: 
      1. MQTT topics
         1. ```esp/joint_states```
      2. ROS actions
         1. ```/rjje_arm_controller/follow_joint_trajectory```
         2. ```/rjje_gripper_controller/gripper_command```
    1. Outputs: 
       1. ROS Topics
          1. ```/joint_states```
       2. MQTT Topics
          1. ```esp/arm```
          2. ```esp/hand```
4. ROS Moveit! action servers setup
   1. ```controller.yaml```
      - ```fake_controller.yaml``` for fake execution (no gazebo required)
      - ```rjje_```
   2. ```/joint_states``` must be published. Otherwise actions will not be even published to their topics.
      1. Even tho the action servers will take care of the action execution result

