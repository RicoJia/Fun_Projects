# This file can be used to install, and launch rviz or gazebo models
# To visualize the robot only, you can see the robot with 
#    - In Gazebo, ./build_and_visualize.sh gazebo
#    - In Rviz: ./build_and_visualize.sh rviz
#    - Launch the moveit pipeline: ./build_and_visualize.sh

echo "Checking for dependencies..."
install_pip_package_on_demand(){
    # {1} should be the module name in Python. ${2} should be pip package name
    python3 -c "import ${1}"
    if [ "${?}" == 0 ]; then
        echo "${1} has been installed"
    else 
        pip install ${2}
    fi
}
install_pip_package_on_demand "paho" "paho-mqtt"

echo "Converted Xacro"
cd ../urdf
xacro rjje_arm.xacro > rjje_arm.xacro.urdf

if [ ${1} == "gazebo" ]; then
    roslaunch rjje_arm gazebo.launch
elif [ ${1} == "rviz" ]; then
    roslaunch rjje_arm rjje_arm_visualization.launch
else
    roslaunch rjje_arm demo.launch
fi