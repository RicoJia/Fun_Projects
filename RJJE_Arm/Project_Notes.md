# RJJE Arm
## Introduction
1. Objectives
    1. Pick and Place using moveit & camera
    2. Arm mounted on mobile platform

## Setup
### Hardware Setup
1. Motor Testing: [Adafruit_PCA9685](https://learn.adafruit.com/16-channel-pwm-servo-driver?view=all)
    - Electrical: 
        - Jack plug for external power 
        - Vcc is positive for signal, V+ is the positive supply. 
    - switching directions will cause a lot of noise on the supply. 
    - may need a cap, like 470 uF for many motors.  
    - channel-board-pinout mapping
    - servo_min-servo_max mapping. 
        - [How to calibrate servos](https://create.arduino.cc/projecthub/jeremy-lindsay/calibrating-my-servos-fa27ce)

2. Servo Motor Control: There are (at least) two ways to do this: Raspberry Pi, or Arduino (nano, uno, etc.)
    1. [Raspberry Pi I2C config](https://learn.adafruit.com/adafruits-raspberry-pi-lesson-4-gpio-setup/configuring-i2c). 
    2. [Arduino setup](https://wiki.keyestudio.com/Ks0173_keyestudio_Nano_ch340)
        - Power: 1. VIN 7v-12v 2. Mini-B USB
        - In Arduino IDE, select NANO as the board
        - Bootloader: choose ATMega328P (old bootloader)
        - Select USB port

    3. I2C Tool can scan I2C devices. Or SMBus devices (protocal based on I2C, a two wire communication). 

3. WIFI Connection
    1. RJJE arm operates using an ESP32 microcontroller, which comes with a WIFI module. For communication, MQTT protocal is used. 
        - In a speed test, 1 request(30-byte)-response(30-byte) takes ~0.01s to finish. The router's upload and & download speeds are around ~130 & ~150 mbps. Therefore, we are using WIFI for real time robot arm control.

### Software Setup
1. Docker Container Setup 
    1. Pull Docker image: ```docker pull ricojia/rjje_arm``` 
    2. Build container ```./dockint build ricojia/rjje_arm $(pwd)```
    3. Start container ```./dockint run ricojia/rjje_arm $(pwd)/rjje_arm_ws bash```
        - this is an ephemeral container so it doesn't need to be removed 

2. Install adafruit library 
    - Open Arduino IDE, select ```rjje_arm/arduino_files/servo_control/``` as sketch folder (under file->preferences)
    - follow [this link](https://learn.adafruit.com/16-channel-pwm-servo-driver?view=all#install-adafruit-pca9685-library-1825143-2)

3. Install rosserial_arduino for arduino work space. [Follow this link](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)

4. Build project
    ```bash
    catkin_make
    source devel/setup.bash
    roscd rjje_arm/
    cd scripts
    ./build_and_launch.sh
    ```

## Design Notes
### Foundation
1. Servo Motors
    - The rudder on top of a servo motor has holes that need to be rethreaded. One can use a "tap" for rethreading. 
    - Before installing a servo, the servo's orientation needs to be adjusted to 90 degrees. The orientation of a servo (from 0 to 180 degrees) is shown below
        <p align="center">
        <img src="https://user-images.githubusercontent.com/39393023/135568554-f84da7c6-10e5-4773-9298-33f507092285.JPEG" height="400" width="width"/>
        <figcaption align="center">Note: the rudder is on the right hand side of the servo</figcaption>
        </p>
    
    - Some servos work well in the range ```[10, 170]``` degrees. [Source](https://www.intorobotics.com/how-to-control-servo-motors-with-arduino-no-noise-no-vibration/)

    - Servos take PWM signals as inputs. The output torque rating goes higher if the input voltage is higher. 

    - **If you have a servo that vibrates, here are some possible reasons: ** [Source](https://electronicguidebook.com/reasons-why-a-servo-motor-vibrates/)
        - Power is instable, or not adequate. So get a larger power supply, or power cable
        - Working near the range limits
        - Motors are cheap so that parts don't work properly. **In my case, some motors did work better than others. I think that's because the good ones have better correspondence to the internal PID control loop**

    - Analog Servos' feedback can have around 1 degree of error. Which can introduce jittering in replay

2. 3D CAD Modelling
    1. STL file example - https://create.arduino.cc/projecthub/danny-van-den-heuvel/6dof-robotic-arm-50eab6
    2. Modelling 
        - First, obtain the STL files of major parts created by Rico Jia. Note that these STL files were created using Onshape, and some not-important details are omitted. 
        - When assembling STL models into a full 3D model of the robot, special attention should be paid to: 
            1. In general, ROS follows multiple ways to express rrotation with angles. [See here](https://www.ros.org/reps/rep-0103.html). In URDF, it's **Z-Y-X** Euler angle

3. Software Setup (D)
    - Build Docker and Tools  
    - STL and Collada files
        - STL from 3D printing, Collada has physics as well.
        - seems like STL can -> DAE files. This can be done on oneshape
        - [How to add mesh to URDF](http://wiki.ros.org/urdf/Tutorials/Building%20a%20Visual%20Robot%20Model%20with%20URDF%20from%20Scratch) 
    - Put the robot together 
        - Stepper Motors
        - Mechanical
    - Moveit Pipeline (D)

4. Arm motion planning (simple case: move arm, then claw)
    - Smooth motion execution (D)
        - debug messaging system 
    - Teaching mode
    - Task contructor

### 3D Object Detection
1. Visualize point cloud
    1. Examples 
        - [Real Sense Tracker](https://github.com/spkywood/realsense-tracker)
        - [Robomaster board detection](https://blog.csdn.net/weixin_39298885/article/details/120207053)
    2. We need image and depth map aligned. Since we have images coming from 2 cameras, we need to "align them" by finding reference points. 
        - /camera/aligned_depth_to_color: aligned info
        - /camera/color - RGB info
        - /camera/depth - non-aligned depth information
    3. Also, we can utilize realsense2_camera package to output "ordered" point clouds to us. Therefore have ```ordered_pc:=true``` 
        - [doc](http://docs.ros.org/en/api/sensor_msgs/html/point__cloud2_8py_source.html#l00060)

2.  1. YOLO v5 + pick and place. 
        - YOLO v5 docker, being able to run 
        - get images -> yolo, test

## TODO List 
1. DL Questions
    1. Structure of YOLO 
    2. Other smaller alternative? This is what we want
    3. Training visualization: tensorboard?
        - how to label them?
2. VoiceBox
3. Use pyrealsense instead of launching the node itself
4. protobuf - nanopb messaging system b/w arduino & laptop
5. STM 32. 
    1. Run ros on it. with an added topic for teaching mode
    2. Add
6. Zero gravity?

## Log
1. going back to ros:
    - publisher joint_msg 
    - subscriber has task id to distinguish task.
    - CPU: too high? event check
    - moveit visualization error? conversion?
    - NIT
        - separate out the 5 joints (update self.commanded_angle[:5], from start to end)
        - To transition to a succeeded state, the goal must be in a preempting or active state, it is currently in state: 3 (setting response too many times?)


========================================================================
## Roadmap 
========================================================================
0. Esp32 - wifi control for robot
    - lap -> Esp32 -> what laptop has sent. Need 60% Flash Memory
    - Speed test 
        - Need to get pub/sub on host machine
    - ROS test on ESP32 
1. claw 
    - adjust cad
    - (open & close)
2. Teaching mode
    - How to do zero-gravity?
    - Turn on teaching mode: 360 360 ... Turn off (ros service)
    - sleep, record joint angles, wakeup
    - arduino: calibrate angles. If not accurate, the visualization will be off.
        - the calculated angle and the raw value mapping
        - real_angle -> commanded angles (offset, etc.)
        - repeat on every motor
    - arduino: sleep/wakeup upon command
        1. test service 
            - wrong check sum of msg. Double in python? nope. Anything to do with Arduino CB? yep. Switch to working?
        2. Add the function & get real angle (looks pretty good!)
    - replay
        1. setup 6 motors, wires (D)
        2. read from all 6 of them (D)
        3. add replay logic
            - replay the right commanded angles
            - move_joints
3. Calibration Tools: 2 aruco markers
    - Big marker under the robot (step 1)
    - optional step: side aruco on robot
    - Need cardbox to make the calibration plates
4. Cup pick & place

