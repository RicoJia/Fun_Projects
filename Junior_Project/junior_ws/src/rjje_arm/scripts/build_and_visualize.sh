#!/bin/bash
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

# Test that sends and receives a message through a pub and sub
test_mqtt(){
    echo "Mqtt test, broker IP: ${BROKER_IP}"
    # Here BROKER_IP should hav been set
    msg="test message"
    (sleep 1 && mosquitto_pub -t "test_topic" -m "$msg" -h ${BROKER_IP}) &
    # Here we have to read the mosquitto sub in the fg, 
    # because we can't retrieve the variable set in the bg 
    a=$(mosquitto_sub -d -t "test_topic" -W 2 -h ${BROKER_IP})
    wait
    if [[ $a == *"$msg"* ]]; then echo "MQTT test passed"; return 0; 
    else echo "MQTT test failed"; return 1; fi
}

if [[ ${1} == "gazebo_only" ]]; then
    roslaunch rjje_arm gazebo.launch
elif [[ ${1} == "rviz_only" ]]; then
    roslaunch rjje_arm rjje_arm_visualization.launch
elif [[ ${1} == "gazebo_control" ]]; then
    export BROKER_IP="127.0.0.1"
    test_status=test_mqtt
    if $test_status == 0; then roslaunch rjje_arm demo.launch run_in_gazebo:=true; 
    else exit 1; fi

elif [[ ${1} == "rviz_control" ]]; then
    roslaunch rjje_arm demo.launch fake_execution:=true
elif [[ ${1} == "real_hardware_control" ]]; then
    export BROKER_IP="100.66.47.29"
    test_mqtt
    roslaunch rjje_arm demo.launch 
else
    echo "Invalid input args, please check this file for valid args of operations"
fi
