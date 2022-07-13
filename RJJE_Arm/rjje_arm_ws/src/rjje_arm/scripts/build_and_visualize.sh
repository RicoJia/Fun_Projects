# This file can be used to launch rviz or gazebo models
# assume we have sourced the workspace
cd ../urdf
xacro rjje_arm.xacro > rjje_arm.xacro.urdf
# roslaunch rjje_arm rjje_arm_visualization.launch

if [ ${1} == "gazebo" ]; then
    roslaunch rjje_arm gazebo.launch
else
    roslaunch rjje_arm rjje_arm_visualization.launch
fi
