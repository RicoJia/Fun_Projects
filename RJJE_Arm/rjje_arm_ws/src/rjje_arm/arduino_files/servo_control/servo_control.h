/*!
 *  @file servo_control.h
 *  Non-ROS Helper functions for servo_control
 *  Author: Rico Ruotong Jia, 2021
 *  BSD license, all text above must be included in any redistribution
 */
#ifndef __SERVO_CONTROL_H__
#define __SERVO_CONTROL_H__

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVO_FREQ 50
#define SERVO_MIN 120
#define SERVO_MAX 480
#define UPDATE_FREQUENCY 100    //hz
#define ANGULAR_THRESHOLD 4.0   //degrees
#define RESET 0xFF
#define OKAY 0x0F    //this is a crappy design, but we can use this since we're just sending two decimal places

inline int sign(float i){
  if (i == 0) return 0; 
  else if (i < 0) return -1; 
  else return 1;
}

struct Motor{
    // In robotics a common convention is the right-hand rotation, which defines the positive direction of rotation is counter-clockwise of positive z-axis. 
    // Below all angles follow the right-hand convention 

    // In my setup, all motors have the neutral position at 90
    float last_angle_ = 90;
    char offset_=0;   //offset should is added on commanded_angle
    // Therefore we need to "flip" the angle about the neutral position if necessary.
    bool is_claw_ = false;
    bool flip_rotation_ = false;   

    bool set_angle(const float& commanded_angle, const short& channel_id, const Adafruit_PWMServoDriver& pwm){
        float real_angle = get_real_angle(commanded_angle);
        if (real_angle != -1){
          int pulselength = map(real_angle, 0, 180, SERVO_MIN, SERVO_MAX);
          pwm.setPWM(channel_id, 0, pulselength);
          return true;
        }
        else{
          return false;
        }
    }

  private: 
    /**
    * @brief: For claw, commanded_angle is the commanded angle between two fingers.
    * @param: commanded_angle: angle commanded by the motion controller
    * @return: angle to be executed on servo after neccesary conversion. 
              -1 if the commanded_angle exceeds angle limits
    */
    float get_real_angle (float commanded_angle){
       if (is_claw_){
           float rotation_angle = commanded_angle/2;
           //we're hard coding this because we're running out of memory
           if (0 <= rotation_angle && rotation_angle <= 90){
              return 90-rotation_angle + offset_;    //offset due to mislignment of rudder and finger 
           }
           else return -1; 
       }
       else{
         commanded_angle += offset_; 
         float real_angle = (flip_rotation_) ? 180 - commanded_angle : commanded_angle;
         if (0 <= real_angle && real_angle <= 180){
           real_angle = last_angle_ + 0.5*(real_angle - last_angle_) ; 
           last_angle_ = real_angle;
           return real_angle; 
         }
         else{
           return -1;
         }
       }
    }
}; 

inline void back_to_neutral(Motor* const motors, const Adafruit_PWMServoDriver& pwm, float* commanded_angles){
    commanded_angles[0] = 90; 
    commanded_angles[1] = 90; 
    commanded_angles[2] = 90; 
    commanded_angles[3] = 90; 
    commanded_angles[4] = 90; 
    commanded_angles[5] = 90; 
    for (unsigned int channel_id = 0; channel_id < 6; ++channel_id){
        motors[channel_id].set_angle(commanded_angles[channel_id], channel_id, pwm);
    }
}

void test_arm(float* const commanded_angles, Motor* const motors, const float& step_angle, const Adafruit_PWMServoDriver& pwm){
    static float degrees[6] = {90, 90, 90, 30, 90, 90};
    for (unsigned int channel_id = 0; channel_id < 6; ++channel_id){
        commanded_angles[channel_id] += sign(degrees[channel_id] - commanded_angles[channel_id]) * step_angle;
        motors[channel_id].set_angle(commanded_angles[channel_id], channel_id, pwm);
    }
}

#endif /* end of include guard: __SERVO_CONTROL_H__ */
