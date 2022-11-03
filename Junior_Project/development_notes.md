# The Junior Project
## Introduction 
Junior is a mobile collaborative robot that can:
    - Follow its owner 
    - Deliver Coffee 
    - Pick up simple objects (visual servo / GraspNet)
    - Learn simple indoor tasks, such as open its drawers. 
Its evolution is more in line with this illustration: 
    ![](https://pictshare.net/sefveq.jpg)

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
1. Gazebo & Motion Planning (D)
    - claw adjust cad (D)
    - Motion Planning with fake joint controller (D)
    - Set up Gazebo (D)
    - Motion Controller (D)
2. Esp32 - wifi control for robot (D)
    - ESP32 -> motor control.
    - lap -> Esp32 -> what laptop has sent. Need 60% Flash Memory 
3. Calibration Tools: 2 aruco markers
    - Big marker under the robot (step 1)
    - RGBD cam detection (D), RGBD camera - interaction with Arm
    - optional step: side aruco on robot
    - Need cardbox to make the calibration plates
4. Coffee Bot Recommission
5. Voice Interacter

## TODO List 
1. Potential bugs
   1. motion_controller.py: Assumption: MQTT topics for joint angle exeuction follows the same order as joint_states 

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
