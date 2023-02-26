# The Junior Project
## Introduction 
Junior is a mobile collaborative robot that can:
    - Deliver Coffee, which will be the default mode without the arm 
        - LIDAR, base
    - Pick Sock, which can identify socks, and go pick it up
        - LIDAR, Base
        - RGBD camera 
        - Robot arm
        - [Potential Candidate](https://www.robotshop.com/products/yahboom-rosmaster-x3-plus-ros-robot-6-dof-ai-vision-robotic-arm-python-programming-jetson-nano-4gb-xavier-nx-tx2-nx-raspberrypi-4b?gclid=CjwKCAiAnZCdBhBmEiwA8nDQxYBtDZUPTl7GYgzN1houEoAWVr78h4YMmYX54OvLlLO6VoDGnoxSYhoCrB0QAvD_BwE) 
    - Voice control
    - Follow its owner 
    - Less Relevant:
        - 3D visual mapping
Its evolution is more in line with this illustration: 
    ![](https://pictshare.net/sefveq.jpg)

## Components
1. Version 1 (Coffee bot prototype)
    - [Yahboom base](https://www.robotshop.com/products/yahboom-g1-ai-vision-smart-tank-robot-kit-python-with-5g-wifi-video-camera-raspberry-pi-4bwithout-raspberry-pi-board?srsltid=AeTunco6-d3frhPy6rbZGyZCumjOjl0uphopEYHqqDsi3fvVAVT7h0wYJXM) 
    - [LIDAR]
    - [RGBD Camera]
    - Arm (Only as a recognize-pick-n-place prototype)

2. Version 2 
    - Base
    - LIDAR
    - RGBD Camera 
    - Robot Arm

Juniors Main components include: 
    - [RJJE Arm](./junior_ws/src/rjje_arm/docs/rjje_arm_main.md)
    - Depth Camera
    - Voice Interacter
    - Coffee Bot Mobile Base
    - Arduino Files that Drive all these


## Docker Container Setup 
1. Pull Docker image: ```docker pull ricojia/rjje_arm``` 
2. Build container ```./dockint build ricojia/rjje_arm $(pwd)```
3. Start container ```./dockint run ricojia/rjje_arm $(pwd)/rjje_arm_ws bash```
    - this is an ephemeral container so it doesn't need to be removed 

4. Build project
    ```bash
    catkin_make
    source devel/setup.bash
    roscd rjje_arm/
    cd scripts
    ```
========================================================================
## Roadmap 
========================================================================
1. Moveit tool Overhaul 
    - Make a custom moveit interface (3 days, D)
    - Code clean up (1 day)

2. Change motors (2 days)
    - add one state on ESP32:
        1. Read angular values
        2. Move robot to neutral.

4. Coffee Bot Recommission
    - Bring the robot back to original state (same hardware)
    - Change wheels, test
        - Change code for the 4x4

5. Mount arms
    - Try with battery pack

6. Voice Interacter
7. Hand-Eye Calibration, with manual moveit commands (I)

## TODO List 
1. Potential bugs and improvements
    1. motion_controller.py: Assumption: MQTT topics for joint angle exeuction follows the same order as joint_states (low priority)
    2. Grasping: deep learning based?
    3. Trajectory execution: can apply a smoothing function to the trajectory waypoints

2. DL Questions
    1. Structure of YOLO 
    2. Other smaller alternative? This is what we want
    3. Training visualization: tensorboard?
        - how to label them?

3. VoiceBox

4. Use pyrealsense instead of launching the node itself

6. STM 32. 
    1. Run ros on it. with an added topic for teaching mode
    2. Add

7. Teaching mode [Optional]
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

## Tmp - [BOM](https://docs.google.com/spreadsheets/d/1E54WXbF1ZFw96C1kWT-nc_feYRRSePSamz8Ig8BraOA/edit?usp=sharing)
1. V2 We are still going to finish up the prototype, with all software setup. 
    - 3D printed platform 
    - Standoff 
    - [Motors with Encoders](https://www.robotshop.com/products/25d-12v-encoder-gear-motor-with-mounting-bracket-65mm-wheel-smart-robot-diy)

2. V3 Options:
    - Make the robot by yourself!
        - aluminum CNC 
        - 3D printed parts
        - This guy has [good stuff](https://www.youtube.com/watch?v=a1cSpcCnxMk)
        - [From hoverboard](https://www.youtube.com/watch?v=RZAt1Hm5knc): 
    - Arm: 
        - Elephant Robotics
            - myPalletizer

## Done
1. Gazebo & Motion Planning (D)
    - claw adjust cad (D)
    - Motion Planning with fake joint controller (D)
    - Set up Gazebo (D)
    - Motion Controller (D)
2. Esp32 - wifi control for robot (D)
    - ESP32 -> motor control.
    - lap -> Esp32 -> what laptop has sent. Need 60% Flash Memory 
3. Calibration Tools: 2 aruco markers
    - RGBD cam detection (D), RGBD camera - Detect Aruco Marker Pose
4. Fixes 
    - gazebo test (1day)
    - mqtt_test during boot up (1day)
