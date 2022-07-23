# RJJE Arm Modelling
This file is the design documentation of rjje_arm. 

## 3D CAD Modelling
1. STL file example - https://create.arduino.cc/projecthub/danny-van-den-heuvel/6dof-robotic-arm-50eab6
2. Modelling 
    - First, obtain the STL files of major parts created by Rico Jia. Note that these STL files were created using Onshape, and some not-important details are omitted. 
    - When assembling STL models into a full 3D model of the robot, special attention should be paid to: 
        1. In general, ROS follows multiple ways to express rrotation with angles. [See here](https://www.ros.org/reps/rep-0103.html). In URDF, it's **Z-Y-X** Euler angle
    - **One discovery we found was that our STL file does not specify dimensions.** I used onshape to generate it, and set dimensions to centimeters. So in Rviz and Gazebo, they are assumed to be meters. One thing we have to do is to scale the model down to centimenters in ```xacro``` files. 

## URDF

## Gazebo Simulation 
1.  The Gazebo simulation is a behavior level abstraction of the physical RJJE arm. It is used as a tool to validate the correctness of the software. 
    1.  Below behaviours are simulated:
        1.  RJJE physical simulation ```URDF -> SDF```
        2.  MQTT command interface of the RJJE arm controller
            1.  ROS control is used to control the robot  
    2.  Below behaviours are not simulated: 
        1.  Problems in MQTT Orchestration
        2. Failure in RJJE Arm hardware failure  

