/*
  
  Receive 6 servo angle values from ESP 8266 through wifi at a time, 
  then issue control of 6 Servo Motors
  Author: Rico Ruotong Jia, 2021
  BSD license, all text above must be included in any redistribution
*/
#include <ros.h>  // must before message declaration
#include <rjje_arm/MotionControl.h>
#include "servo_control.h"

static const int DELAY = 1000/UPDATE_FREQUENCY;
static double step_angle = 0.6;   //deg

// Declarations
static Motor motors[6]; 
static double commanded_angles[6] = {90, 90, 90, 90, 90, 120};
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40); 
ros::NodeHandle nh; 

void sub_cb(const rjje_arm::MotionControl& motion_control_msg){
  for (unsigned channel_id = 0; channel_id < 6; ++channel_id){
    commanded_angles[channel_id] = motion_control_msg.commanded_angles[channel_id]; 
  }
}

ros::Subscriber<rjje_arm::MotionControl> sub("/motion_control_msg", sub_cb); 


void setup(){  
  /* Serial.begin(19200); //comment out ros node stuff if using this */
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
  motors[0].min_angle_ = 0;
  motors[0].max_angle_ = 180;
  motors[1].min_angle_ = 0;
  motors[1].max_angle_ = 180;
  motors[2].min_angle_ = 0;
  motors[3].max_angle_ = 180;
  motors[3].min_angle_ = 0;
  motors[3].max_angle_ = 180;
  motors[4].min_angle_ = 0;
  motors[4].max_angle_ = 180;
  motors[5].min_angle_ = 0;
  motors[5].max_angle_ = 90;   // single finger can rotate 90 degrees max. So in total two fingers can be 180 degrees apart

  motors[1].offset_ = 5; 
  motors[2].flip_rotation_ = true; 
  motors[2].offset_ = 5; 
  motors[3].flip_rotation_ = true; 
  motors[3].offset_ = 10; 
  motors[5].is_claw_ = true;

  nh.initNode(); 
  nh.subscribe(sub); 
}

void loop(){ 
    unsigned long start = millis();
    /* back_to_neutral(motors, pwm); */
    /* test_arm(commanded_angles, motors, step_angle, pwm); */

    for (unsigned int channel_id = 0; channel_id < 6; ++channel_id){
        motors[channel_id].set_angle(commanded_angles[channel_id], channel_id, pwm);
    }
    nh.spinOnce();

    delay(DELAY + start - millis());
}
