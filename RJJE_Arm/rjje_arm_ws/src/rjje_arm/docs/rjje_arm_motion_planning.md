# RJJE Arm Motion Planning
This file is the design documentation of rjje_arm. Currently, we are able to do motion planning using the default ```moveit_setup_assistant``` 

## Motion Planning 
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

## Test Plan 
