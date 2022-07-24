# RJJE Arm Motion Planning
This file is the design documentation of rjje_arm. Currently, we are able to do motion planning using the default ```moveit_setup_assistant``` 
    - 

## Arm motion planning 
1. Requirements
    - Smooth motion execution (D)
        - debug messaging system
    - Teaching mode
    - Task contructor
2. According to this [post](https://answers.ros.org/question/313637/openclose-end-effector-with-moveit-rviz/), gripper in Moveit is controlled differently. So there's no ball-and-arrow at eef. Therefore we control the gripper separately

## Test Plan 
