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

// Declarations
static Motor motors[6]; 
static byte commanded_angles[6] = {90, 90, 90, 90, 90, 120};
static byte current_angles[6] = {90, 90, 90, 90, 90, 120};
static double step_angles[24] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40); 

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
}

void update_command_angles(){
// Python sends joint angles, execution time, and gets a response back
    if (Serial.available()){
        size_t num_bytes = Serial.readBytes(commanded_angles,6); 
        Serial.println("debug1");

        byte execution_time_buf[2];
        Serial.readBytes(execution_time_buf,2);
        double execution_time = array_to_double(execution_time_buf);
        Serial.println(String(execution_time));
    }
}


void loop(){ 
    update_command_angles();
    unsigned long start = millis();
    /* Serial.println(commanded_angles[3]);  */
    /* for (char channel_id = 0; channel_id < 6; ++channel_id){ */
    /*     commanded_angles[channel_id] += sign(current_angles[channel_id] - commanded_angles[channel_id]) * step_angle; */
    /*     motors[channel_id].set_angle(commanded_angles[channel_id], channel_id, pwm); */
    /* } */
    delay(DELAY + start - millis());
    /* back_to_neutral(motors, pwm); */
    /* test_arm(commanded_angles, motors, step_angle, pwm); */


}
