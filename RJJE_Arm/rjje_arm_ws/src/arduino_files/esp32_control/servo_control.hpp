#ifndef __SERVO_CONTROL_HPP__
#define __SERVO_CONTROL_HPP__

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// set this to 0 if you're using a regular arduino
#define ESP32 1
#ifdef ESP32
const int ANALOG_INPUT = 34;
// 12-bit ADC
const double INTERCEPT = 203.5682231220682;
const double SLOPE = -0.07711167;
#else 
const int ANALOG_INPUT = A0;
// 10-bit ADC
const double INTERCEPT = -46.3003;
const double SLOPE = 0.458;
#endif

#define PWM_BOARD_ADDR 0x40
#define SERVO_FREQ 50
// #define SERVO_MIN 120  // for MG96
// #define SERVO_MAX 480    
#define SERVO_MIN 124   // For Analog Feedback servos
#define SERVO_MAX 348       
#define UPDATE_FREQUENCY 100    //hz
#define ANGULAR_THRESHOLD 0.5   //degrees
#define TEACHING_MODE_VAL 361.0
#define REGULAR_MODE_VAL 362.0
#define INTERCEPT_ANALOG -46.3003
#define SLOPE_ANALOG 0.4580

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


/**
 * 1. A common convention is the right-hand rotation, which defines the positive direction of rotation is counter-clockwise of positive z-axis. Below, all angles follow the right-hand convention
 * 2. In my setup, all motors have their neutral positions at 90 deg
 * 3. Due to each motor's positioning, we need to "flip" the angle about the neutral position if necessary.
 */
struct Motor{
    float last_angle_ = 90;
    char offset_=0;                 //offset is added on commanded_angle
    bool is_claw_ = false;
    bool flip_rotation_ = true;     //false for MG96

    bool set_angle(const float& commanded_angle, const short& channel_id, Adafruit_PWMServoDriver& pwm){
        float real_angle = get_real_angle(commanded_angle);
        if (real_angle != -1){
          uint16_t pulselength = map(real_angle, 0, 180, SERVO_MIN, SERVO_MAX);
          pwm.setPWM(channel_id, 0, pulselength);
          //TODO
          Serial.println("commanded: " + String(commanded_angle) + " | real_angle: " + String(real_angle) + " | pwm val: " + String(pulselength));

          return true;
        }
        else{
          return false;
        }
    }
    
    float convert_to_commanded_angle(const int& raw_value){
        float real_angle = SLOPE_ANALOG * raw_value + INTERCEPT_ANALOG; 
        if (is_claw_){
            return (90 - (real_angle - offset_)) * 2; 
        }
        else{
            return (flip_rotation_) ? (180 - real_angle - offset_) : (real_angle - offset_); 
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
           last_angle_ = real_angle;
           return real_angle; 
         }
         else{
           return -1;
         }
       }
    }
}; 

// test procedure: 
//  1. add this to the wifi controlled interface, with set angle, and Motor class
//
/**
 * This class does these things:
 *  1. starts a pwm object, set it up
 *  2. Set an angle on a servo
 *  3. Read from a servo
 *      - Base on the real world constraints of the motors 
 */
class ServoControl
{
public:
    ServoControl (): pwm(PWM_BOARD_ADDR){
        pwm.begin();
        pwm.setOscillatorFrequency(27000000);
        pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
        /* pinMode(A0, INPUT_PULLUP);  */
        pinMode(ANALOG_INPUT, INPUT_PULLUP); 
        delay(5000); 
    }
    ~ServoControl (){}

    void set_angle(const float& angle, byte index){
        motors[index].set_angle(angle, 0, pwm);
    }

private:
    Adafruit_PWMServoDriver pwm;
    Motor motors[6];
};
#if 0
#endif

#endif /* end of include guard: __SERVO_CONTROL_HPP__ */
