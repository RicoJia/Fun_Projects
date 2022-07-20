# RJJE Arm
## Introduction
1. Objectives
    1. Pick and Place using moveit & camera
    2. Arm mounted on mobile platform
2. [System Flowchart](https://drive.google.com/file/d/1ujubSrS_AvXeORWJ76qhUnCQ4BP0E4v_/view?usp=sharing)

## Setup
1. Hardware setup, [see doc](./rjje_arm_ws/src/arduino_files/README.md)

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

5. To visualize the robot only, you can see the robot with 
   - In Gazebo, ```./build_and_visualize.sh gazebo``` 
   - In Rviz: ```./build_and_visualize.sh```

## Design Notes
### Robot
2. Modelling, [see doc](rjje_arm_ws/src/rjje_arm/docs/rjje_arm_modelling.md)
    - 3D Modelling 
    - Gazebo Environment

3. RJJE Arm Motion Planning [see doc](rjje_arm_ws/src/rjje_arm/docs/rjje_arm_motion_planning.md)
    - Arm motion planning (simple case: move arm, then claw)

4. Vision [see doc](rjje_arm_ws/src/rjje_arm/docs/rjje_arm_vision.md)
    - 3D Object Detection

========================================================================
## Roadmap 
========================================================================
1. Esp32 - wifi control for robot
    - lap -> Esp32 -> what laptop has sent. Need 60% Flash Memory (On hold, PCA9685 stopped working)
    - ```motion_controller``` node, with joint states, ros services
2. Gazebo & Motion Planning
    - claw adjust cad
    - Motion Planning with fake joint controller
    - Set up Gazebo 
3. Calibration Tools: 2 aruco markers
    - Big marker under the robot (step 1)
    - RGBD cam detection (D), RGBD camera - interaction with Arm
    - optional step: side aruco on robot
    - Need cardbox to make the calibration plates

4. Cup pick & place
5. Javascript 手撕做个小网站
6. Teaching mode [Optional]
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

