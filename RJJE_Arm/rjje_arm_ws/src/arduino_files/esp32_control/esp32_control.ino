/*
  Author: Rico Ruotong Jia, 2022
  BSD license, all text above must be included in any redistribution
  Introduction:
    1. ESP32 Receive 5 servo angle values and 1 gripper command through wifi.
    2. Display mode of operation

  Modes of operations:
      1. Teaching mode: /rjje_arm/motion_control is [270, 270,270, 270,270, 270]. To turn it off, [360, 360,360, 360,360, 360]
      2. Regular Mode: listens to /rjje_arm/motion_control, and execute joint angles within execution_time. The upper stream python node can replay recorded angles, too. Note, only angles within (0, 180) will be executed
  Notes: 
      1. MG996R has a different rotation orientation than FB5116M. So the default flip_rotation_ flag is different. Their pwm range is also slightly different.
*/

