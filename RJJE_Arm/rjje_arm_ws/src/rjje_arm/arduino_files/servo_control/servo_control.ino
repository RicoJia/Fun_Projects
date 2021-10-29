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
static double commanded_angles[6] = {90, 90, 90, 90, 90, 120};
static double current_angles[6] = {90, 90, 90, 90, 90, 120};
static double step_angles[24] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
static double execution_time;
static bool operating = false;

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

  back_to_neutral(motors, pwm); 
  delay(2500); 
}

/**
* @brief: Read received bytes from serial into a double array. Precision is 1
* @param: dst: pointer to array that stores the output
* @param: num_elements: number of elements in the array
*/
void read_bytes_into_double_array(double* dst, size_t num_elements){
    byte buffer[num_elements*2]; 
    Serial.readBytes(buffer, num_elements*2); 
    for (char channel_id = 0; channel_id < num_elements; ++channel_id){
        dst[channel_id] = buffer[2*channel_id] + 0.1 * buffer[2*channel_id+1];
    }
}

/**
* @brief: Initialize commanded_angles, execution_time, and step_angles
*  Python sends joint angles, execution time, and gets a response back
*/
void initialize(){
    if (Serial.available()){
        read_bytes_into_double_array(commanded_angles, 6); 
        read_bytes_into_double_array(&execution_time, 1); 
        for (int i = 0; i < 6; ++i){
            step_angles[i] = (commanded_angles[i] - current_angles[i])/execution_time/UPDATE_FREQUENCY;
        }
        operating = true;
    }
}

void update_current_angles(){
    // TO swap for analog step motors
    for (unsigned int i = 0; i < 6; ++i) {
        current_angles[i] += step_angles[i];
    }
}

bool can_stop(){
    char stop_count = 0; 
    for (unsigned int i = 0; i < 6; ++i) {
        if (abs(commanded_angles[i] - current_angles[i]) < ANGULAR_THRESHOLD){
          ++stop_count;
        }
    }
    return stop_count == 6; 
}

void send_finish_response(){
    Serial.write(1); 
}

void loop(){ 
    initialize();
    unsigned long start = millis();
    if(operating)
    {

        // might consider update_step_angles();
        for (char channel_id = 0; channel_id < 6; ++channel_id){
            double angle_to_execute = current_angles[channel_id] + step_angles[channel_id]; 
            motors[channel_id].set_angle(angle_to_execute, channel_id, pwm);
        }
        update_current_angles(); 
        if (operating && can_stop()){
            send_finish_response();
            operating = false;

            for (unsigned int i = 0; i < 6; ++i) {
                Serial.println(current_angles[i]); 
            }
        }
    }
    unsigned long time_elapsed = millis() - start;
    delay(DELAY - time_elapsed);
}
