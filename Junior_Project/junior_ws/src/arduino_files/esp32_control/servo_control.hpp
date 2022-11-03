#ifndef __SERVO_CONTROL_HPP__
#define __SERVO_CONTROL_HPP__

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


#define PWM_BOARD_ADDR 0x40
#define SERVO_FREQ 60
#define SERVO_MIN 120  // for MG96
#define SERVO_MAX 580    
// #define SERVO_MIN 124   // For Analog Feedback servos
// #define SERVO_MAX 348       
const byte ARM_MOTOR_NUM = 5;
const byte MAX_WAYPOINT_NUM = 64;
#define ANGULAR_THRESHOLD 0.01   //radians
#define UPDATE_FREQUENCY 40   //hz


inline int sign(float i){
  if (i == 0) return 0; 
  else if (i < 0) return -1; 
  else return 1;
}

bool pwm_board_connected(){
    Wire.beginTransmission(PWM_BOARD_ADDR); 
    byte error = Wire.endTransmission(); 
    return (error==0); 
}

double convert_to_rjje_commanded_angles(double angle){
  return 180 * (angle/3.1415) + 90;
}

/**
 * 1. A common convention is the right-hand rotation, which defines the positive direction of rotation is counter-clockwise of positive z-axis. Below, all angles follow the right-hand convention
 * 2. In my setup, all motors have their neutral positions at 90 deg
 * 3. Due to each motor's positioning, we need to "flip" the angle about the neutral position if necessary.
 */
struct Motor{
    float last_angle_ = 90;
    char offset_=0;                 //offset is added on commanded_angle
    bool is_claw_ = false;
    bool flip_rotation_ = false;     //false for MG96

    bool set_angle(const float& commanded_angle, const short& channel_id, Adafruit_PWMServoDriver& pwm){
        float real_angle = get_real_angle(commanded_angle);
        if (real_angle != -1){
          uint16_t pulselength = map(real_angle, 0, 180, SERVO_MIN, SERVO_MAX);
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
           //TODO
           float rotation_angle = commanded_angle/2;
            return rotation_angle + offset_;    //offset due to mislignment of rudder and finger 
       }
       else{
         if (0 <= commanded_angle && commanded_angle <= 180){
           commanded_angle += offset_; 
           float real_angle = (flip_rotation_) ? 180 - commanded_angle : commanded_angle;
           return real_angle; 
         }
         else{
           return -1;
         }
       }
    }
}; 

/**
 * This class does these things:
 *  1. starts a pwm object, set it up
 *  2. Set an angle on a servo
 */
class ServoControl
{
public:
    ServoControl (): pwm(PWM_BOARD_ADDR){
        pwm.begin();
        pwm.setOscillatorFrequency(27000000);
        pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~60 Hz updates
        delay(5000);

        motors[2].flip_rotation_ = true;
        motors[3].flip_rotation_ = true;
        motors[5].is_claw_ = true;

        double initial_arm_angles[ARM_MOTOR_NUM] = {0,0,0,0,0};
        double initial_claw_angle = 0.0;
        execute_arm_angles(initial_arm_angles); 
        execute_claw_angle(initial_claw_angle); 
        delay(5000); 
    }
    ~ServoControl (){}

    void execute_arm_angles(double* arm_execution_angles){
        for (byte i = 0; i < ARM_MOTOR_NUM; i++)
        {
          motors[i].set_angle(convert_to_rjje_commanded_angles(arm_execution_angles[i]), i, pwm);
        }
    }

    void execute_claw_angle(double angle){
        motors[ARM_MOTOR_NUM].set_angle(convert_to_rjje_commanded_angles(angle), ARM_MOTOR_NUM, pwm);
    }

private:
    Adafruit_PWMServoDriver pwm;
    Motor motors[ARM_MOTOR_NUM+1];

};
#if 0
#endif

#endif /* end of include guard: __SERVO_CONTROL_HPP__ */
