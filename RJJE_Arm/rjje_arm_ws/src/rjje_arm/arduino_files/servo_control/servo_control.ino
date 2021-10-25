/*
  Receive 6 servo angle values from ESP 8266 through wifi at a time, 
  then issue control of 6 Servo Motors
  Author: Rico Ruotong Jia, 2021
*/
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ros.h>  // must before message declaration
#include <rjje_arm/MotionControl.h>

#define SERVO_FREQ 50
#define SERVO_MIN 120
#define SERVO_MAX 480
#define SERVO_MAX_ANGULAR_VEL 10  //10 degrees/second

const double MAX_ANGULAR_INCREMENT = SERVO_MAX_ANGULAR_VEL/SERVO_FREQ; 

int sign(double i){
  if (i == 0) return 0; 
  else if (i < 0) return -1; 
  else return 1;
}

struct Motor{
  // In robotics a common convention is the right-hand rotation, which defines the positive direction of rotation is counter-clockwise of positive z-axis. 
  // Below all angles follow the right-hand convention 
  double min_angle_ = 0;  
  double max_angle_ = 180;   
  // In my setup, all motors have the neutral position at 90
  double neutral_angle_=90;   
  double last_angle_ = neutral_angle_;
  double offset_=0;   //offset should is added on commanded_angle
  // Therefore we need to "flip" the angle about the neutral position if necessary.
  bool is_claw_ = false;
  bool flip_rotation_ = false;   

  /**
  * @brief: For claw, commanded_angle is the commanded angle between two fingers.
  * @param: commanded_angle: angle commanded by the motion controller
  * @return: angle to be executed on servo after neccesary conversion. 
            -1 if the commanded_angle exceeds angle limits
  */
  double get_real_angle (double commanded_angle){
     if (is_claw_){
         double rotation_angle = commanded_angle/2;
         if (min_angle_ <= rotation_angle && rotation_angle <= max_angle_){
            return max_angle_-rotation_angle - 10;    //offset is 10, due to mislignment of rudder and finger 
         }
         else return -1; 
     }
     else{
       commanded_angle += offset_; 
       double real_angle = (flip_rotation_) ? 180 - commanded_angle : commanded_angle;
       if (min_angle_ <= real_angle && real_angle <= max_angle_){
         /*
         if (abs(real_angle-last_angle_) > 3.5){
    real_angle = last_angle_ + sign(real_angle - last_angle_) * 3.5; 
         }
         */
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

// Declarations
Motor motors[6]; 
double commanded_angles[6] = {90, 90, 90, 90, 90, 120};
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40); 
ros::NodeHandle nh; 

void sub_cb(const rjje_arm::MotionControl& motion_control_msg){
  for (unsigned channel_id = 0; channel_id < 6; ++channel_id){
    commanded_angles[channel_id] = motion_control_msg.commanded_angles[channel_id]; 
  }
}

ros::Subscriber<rjje_arm::MotionControl> sub("/motion_control_msg", sub_cb); 

void set_angle(const double& real_angle, const short& channel_id){
    int pulselength = map(real_angle, 0, 180, SERVO_MIN, SERVO_MAX);
    pwm.setPWM(channel_id, 0, pulselength);
}

void setup(){  
  pwm.begin();
  //Serial.begin(9600);
  // Tune oscillator frequency until the output PWM is around 50Hz, 
  // maybe using an oscilloscope
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
  /*
  motors[0].min_angle_ = 5;
  motors[0].max_angle_ = 170;
  motors[1].min_angle_ = 5;
  motors[1].max_angle_ = 170;
  motors[2].min_angle_ = 35;
  motors[3].max_angle_ = 170;
  motors[3].min_angle_ = 10;
  motors[3].max_angle_ = 180;
  motors[4].min_angle_ = 0;
  motors[4].max_angle_ = 180; 
  motors[5].min_angle_ = 0;
  motors[5].max_angle_ = 90;   // single finger can rotate 90 degrees max. So in total two fingers can be 180 degrees apart
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
  motors[3].offset_ = 5; 
  motors[5].is_claw_ = true;

  nh.initNode(); 
  nh.subscribe(sub); 
}

void stop_arm(){
    double degrees[5] = {80, 80, 80, 120, 80};
    for (unsigned int channel_id = 0; channel_id < 4; ++channel_id){
      int pulselength = map(degrees[channel_id], 0, 180, SERVO_MIN, SERVO_MAX);
      pwm.setPWM(channel_id, 0, pulselength);
    }
    Serial.println("RJJE Arm Stopped");
}

void loop(){ 

    for (unsigned int channel_id = 0; channel_id < 6; ++channel_id){
        double real_angle = motors[channel_id].get_real_angle(commanded_angles[channel_id]);  
        if (real_angle != -1){
            set_angle(real_angle, channel_id); 
        }
        //Serial.println("channel_id" + String(channel_id) + "real angle: " + String(real_angle));
    }
    //TODO
    // nh.loginfo("Program info");

    nh.spinOnce();
    // fresh frequency: 100hz
    delay(20);
}
