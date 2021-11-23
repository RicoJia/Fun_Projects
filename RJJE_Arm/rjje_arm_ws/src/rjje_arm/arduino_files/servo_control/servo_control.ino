/*
  
  Receive 6 servo angle values from ESP 8266 through wifi at a time, 
  then issue control of 6 Servo Motors
  Author: Rico Ruotong Jia, 2021
  BSD license, all text above must be included in any redistribution
*/

// Modes of operations: 
// 1. Teaching mode: /rjje_arm/motion_control is [270, 270,270, 270,270, 270]. To turn it off, [360, 360,360, 360,360, 360]

#include <ros.h>  // must before message declaration
#include <rjje_arm/MotionControl.h>
#include <rjje_arm/JointFeedback.h>
#include "servo_control.h"

static const int DELAY = 1000/UPDATE_FREQUENCY;

// Declarations

static Motor motors[6]; 
static float commanded_angles[6] = {90, 90, 90, 90, 90, 90};
static float current_angles[6] = {90, 90, 90, 90, 90, 90};
static float step_angles[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
static byte arm_task_id = 0;
static bool operating = false;
static bool teaching_mode = false;
static const char servo_inputs[6] = {A0, A1, A2, A3, A6, A7};

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PWM_BOARD_ADDR); 
ros::NodeHandle nh;
rjje_arm::JointFeedback joint_msg;
ros::Publisher joint_msg_pub("/rjje_arm/joint_feedback", &joint_msg); 

bool switch_to_teaching_mode(const rjje_arm::MotionControl& msg){
    return (msg.c5 == TEACHING_MODE_VAL); 
}

bool switch_to_regular_mode(const rjje_arm::MotionControl& msg){
    return (msg.c5 == REGULAR_MODE_VAL); 
}

void sub_cb(const rjje_arm::MotionControl& msg){
    if (msg.task_id != arm_task_id){
        arm_task_id = msg.task_id;
        if (!teaching_mode){
            if (switch_to_teaching_mode(msg)){
                teaching_mode = true; 
                operating = true;
                return; 
            }
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
        else{
            if (switch_to_regular_mode(msg)){
                teaching_mode = false;
                operating = false;
            }
        }
    }
}

ros::Subscriber<rjje_arm::MotionControl> arm_sub("/rjje_arm/motion_control", sub_cb);

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
  motors[2].flip_rotation_ = !motors[2].flip_rotation_; 
  motors[2].offset_ = 5; 
  motors[3].flip_rotation_ = !motors[3].flip_rotation_; 
  motors[3].offset_ = 10; 
  motors[5].is_claw_ = true;
  motors[5].offset_ = -10;

  nh.initNode(); 
  nh.advertise(joint_msg_pub);
  nh.subscribe(arm_sub);
  joint_msg.joint_angles_length = 6; 
  joint_msg.joint_angles = &current_angles[0];

  back_to_neutral(motors, pwm, commanded_angles); 
  for (char i = 0; i < 6; ++i) {
      pinMode(servo_inputs[i], INPUT_PULLUP); 
  }

  delay(2500); 
}

void update_current_angles(bool read_from_servo_inputs){
    
    for (unsigned char i = 0; i < 6; ++i) {
        if (read_from_servo_inputs){
            int raw_value = analogRead(servo_inputs[i]);
            // TODO calibrate
            /* current_angles =  */
        }
        else{
            current_angles[i] += step_angles[i];
        }
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
        if (!pwm_board_connected()){
          nh.loginfo("pwm board not connected!"); 
        }
        else{
          if (teaching_mode){
              //TODO
              update_current_angles(true);
          }
          else{
              // prevent overshoot
              update_step_angles(); 
              for (char channel_id = 0; channel_id < 6; ++channel_id){
                  float angle_to_execute = current_angles[channel_id] + step_angles[channel_id]; 
                  motors[channel_id].set_angle(angle_to_execute, channel_id, pwm);
              }
              update_current_angles(false); 
              if(operating && can_stop()){
                  operating = false;
              }

          }
        }
    }


    joint_msg_pub.publish(&joint_msg); 
    nh.spinOnce();
    unsigned long now = millis();
    if (start + DELAY > now){
        delay(start + DELAY - now);
    }
}
