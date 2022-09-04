#include "servo_control.h"

#define ESP32 1
#ifdef ESP32
const int ANALOG_INPUT = 34;
/* const int SCL = 22; */
/* const int SDA = 21; */
// 12-bit ADC
const double INTERCEPT = 203.5682231220682;
const double SLOPE = -0.07711167;
#else 
const int ANALOG_INPUT = A0;
/* const int SCL = A5; */
/* const int SDA = A4; */
// 10-bit ADC
const double INTERCEPT = -46.3003;
const double SLOPE = 0.458;
#endif

static Motor motors[6]; 
static float commanded_angles[6] = {180, 90, 90, 90, 90, 90};

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PWM_BOARD_ADDR); 

void setup()
{
  Serial.begin(115200); //comment out ros node stuff if using this

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  /* pinMode(A0, INPUT_PULLUP);  */
  pinMode(ANALOG_INPUT, INPUT_PULLUP); 
  delay(5000); 
  
  /* Angle Limits: 
      motor 0: [5, 170]
      motor 1: [5, 170] 
      motor 2: [35, 170] 
      motor 3: [10, 140] 
      motor 4: [0, 180] 
      motor 5[0, 130]
    */
  // motors[1].offset_ = 5; 
  // motors[2].flip_rotation_ = !motors[2].flip_rotation_; 
  // motors[2].offset_ = 5; 
  // motors[3].flip_rotation_ = !motors[3].flip_rotation_; 
  // motors[3].offset_ = 10; 
  // motors[5].is_claw_ = true;
  // motors[5].offset_ = -10;
}

void servo_read(){
      /* int val = analogRead(A0); */
      int val = analogRead(ANALOG_INPUT);
      double measured = SLOPE * val + INTERCEPT; 
      Serial.println(" |measured: " + String(measured) + " | raw: " + val); 
      delay(200);  
}

void loop()
{
  
  for (int i = 0; i <=180; i+=90) {
      motors[0].set_angle(i, 0, pwm);
      delay(3000);  
  }

    for (int i = 180; i >= 0; i-=90) {
      motors[0].set_angle(i, 0, pwm);
      delay(3000);
  }
}

