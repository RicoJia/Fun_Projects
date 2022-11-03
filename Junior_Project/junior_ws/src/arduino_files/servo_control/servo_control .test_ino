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
  
  /* motors[0].flip_rotation_ = true; */
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
  
  for (int i = 0; i <=180; i+=3) {
      motors[0].set_angle(i, 0, pwm);
      delay(100);  
      int val = analogRead(ANALOG_INPUT);

      double measured = SLOPE * val + INTERCEPT; 
      Serial.println("sent: " + String(i)+" |"+"original measurement: "+String(val)+"|"+"processed measured: " + String(measured));
      /* servo_read(); */
  }

    for (int i = 180; i >= 0; i-=3) {
      motors[0].set_angle(i, 0, pwm);
      delay(100);
      int val = analogRead(ANALOG_INPUT);
      double measured = SLOPE * val + INTERCEPT; 
      Serial.println("sent: " + String(i)+" |"+"original measurement: "+String(val)+" |"+"processed measured: " + String(measured));

      /* Serial.println("sent: " + String(i)+" |"+"measured: " + String(val)); */

      /* delay(300);   */
      /* servo_read(); */
  }
}

