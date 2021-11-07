/*
  
  Receive 6 servo angle values from ESP 8266 through wifi at a time, 
  then issue control of 6 Servo Motors
  Author: Rico Ruotong Jia, 2021
  BSD license, all text above must be included in any redistribution
*/
#include <ros.h>  // must before message declaration
#include <rjje_arm/ArmControl.h>
#include <rjje_arm/GripperControl.h>
#include <rjje_arm/JointFeedback.h>
#include "servo_control.h"

static const int DELAY = 1000/UPDATE_FREQUENCY;

// Declarations

static Motor motors[6]; 
static float commanded_angles[6] = {90, 90, 90, 90, 90, 120};
static float current_angles[6] = {90, 90, 90, 90, 90, 120};
static float step_angles[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
static byte arm_task_id = 0;
/* static byte gripper_task_id = 0; */
static bool operating = false;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40); 
ros::NodeHandle nh;
rjje_arm::JointFeedback joint_msg;
ros::Publisher joint_msg_pub("/robot_joint", &joint_msg); 

void sub_cb(const rjje_arm::ArmControl& msg){
    if (msg.task_id != arm_task_id){
        arm_task_id = msg.task_id;
        commanded_angles[0] = msg.c0;  
        commanded_angles[1] = msg.c1;  
        commanded_angles[2] = msg.c2;  
        commanded_angles[3] = msg.c3;  
        commanded_angles[4] = msg.c4;  
        commanded_angles[5] = msg.c5;  
        for (unsigned char i = 0; i < 5; ++i) {
            step_angles[i] = (commanded_angles[i] - current_angles[i])/msg.execution_time/UPDATE_FREQUENCY;
        }
        operating = true; 
    }
}

/* void gripper_sub_cb(const rjje_arm::GripperControl& msg){ */
/*     if (msg.task_id != gripper_task_id){ */
/*         arm_task_id = msg.task_id; */
/*         commanded_angles[5] = msg.commanded_angle;  */
/*         step_angles[5] = (commanded_angles[5] - current_angles[5])/msg.execution_time/UPDATE_FREQUENCY; */
/*         operating = true; */
/*     } */
/* } */
ros::Subscriber<rjje_arm::ArmControl> arm_sub("/rjje_arm/arm_control", sub_cb);
/* ros::Subscriber<rjje_arm::GripperControl> gripper_sub("/rjje_arm/gripper_control", gripper_sub_cb); */

void setup(){  
  Serial.begin(9600); //comment out ros node stuff if using this
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(20); 

  /* Angle Limits: 
      motor 0: [5, 170]
      motor 1: [5, 170] 
      motor 2: [35, 170] 
      motor 3: [10, 140] 
      motor 4: [0, 180] 
      motor 5[0, 130]
    */
  motors[1].offset_ = 5; 
  motors[2].flip_rotation_ = true; 
  motors[2].offset_ = 5; 
  motors[3].flip_rotation_ = true; 
  motors[3].offset_ = 10; 
  motors[5].is_claw_ = true;
  motors[5].offset_ = -10;

  nh.initNode(); 
  nh.advertise(joint_msg_pub);
  nh.subscribe(arm_sub);
  /* nh.subscribe(gripper_sub); */
  joint_msg.commanded_angles_length = 6; 
  //TODO
  joint_msg.commanded_angles = &current_angles[0];

  back_to_neutral(motors, pwm, commanded_angles); 
  delay(2500); 
}

void update_current_angles(){
    // TODO swap for analog step motors
    for (unsigned char i = 0; i < 6; ++i) {
        current_angles[i] += step_angles[i];
        //TODO
        /* char result[32]; // Buffer big enough for 7-character float */
        /* dtostrf(current_angles[i], 6,2, result); */
        /* nh.loginfo(result); */
    }
}

bool can_stop(){
    char stop_count = 0; 
    for (unsigned char i = 0; i < 6; ++i) {
        if (abs(commanded_angles[i] - current_angles[i]) < ANGULAR_THRESHOLD){
          ++stop_count;
        }
    }
    return stop_count == 6; 
}

void update_step_angles(){
    for(unsigned char i =0; i < 6; ++i){
        if(abs(step_angles[i]) > abs(commanded_angles[i] - current_angles[i])){
          step_angles[i] = commanded_angles[i] - current_angles[i]; 
        }
    }
}

void loop(){ 
    unsigned long start = millis();
    if(operating)
    {
        // prevent overshoot
        update_step_angles(); 
        for (char channel_id = 0; channel_id < 6; ++channel_id){
            float angle_to_execute = current_angles[channel_id] + step_angles[channel_id]; 
            //motors[channel_id].set_angle(angle_to_execute, channel_id, pwm);
        }
        update_current_angles(); 
        if(operating && can_stop()){
            operating = false;
        }
    }


    //TODO
    joint_msg_pub.publish(&joint_msg); 
    /* nh.loginfo("hz"); */
    nh.spinOnce();
    /* unsigned long now = millis(); */
    /* if (start + DELAY > now){ */
    /*     delay(start + DELAY - now); */
    /* } */
}
