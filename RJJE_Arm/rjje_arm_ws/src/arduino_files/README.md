# RJJE Arm Hardware
This file is the design documentation of rjje_arm's hardware structure.
    1. ```esp32_control``` is under active development. servo_control will be merged into ```esp32_control``` and get deprecated.
    2. The [System diagram can be accessed using draw.io](https://drive.google.com/file/d/1ujubSrS_AvXeORWJ76qhUnCQ4BP0E4v_/view?usp=sharing) 

## Usage
1. Open Arduino IDE, install these libraries using ```library manager```
    - ```Adafruit_PWMServoDriver```
    - ```EspMQTTClient```
2. To Control the claw, do ```rosservice call /rjje_arm/claw_control OPEN_VAL```, where ```OPEN_VAL``` = 0 for closing, 1 for opening

## Design
### Motors
- TODO: all connections
1. Simple Motor Testing: [Adafruit_PCA9685, including schematics](https://learn.adafruit.com/16-channel-pwm-servo-driver?view=all)
    - Electrical: 
        - Jack plug for external power 
        - Vcc is positive for signal, V+ is the positive supply. 
    - switching directions will cause a lot of noise on the supply. 
    - may need a cap, like 470 uF for many motors.  
    - channel-board-pinout mapping
    - servo_min-servo_max mapping. 
        - [How to calibrate servos](https://create.arduino.cc/projecthub/jeremy-lindsay/calibrating-my-servos-fa27ce)
    - on ESP32: 

        <p align="center">
        <img src="https://user-images.githubusercontent.com/106101331/175851195-075cdfe5-a3cd-4bd5-86f6-e74e4d874305.png" height="400" width="width"/>
        <figcaption align="center">Pinouts of ESP32</figcaption>
        </p>

        ```
        Analog 4 (GPIO32) -> SDA (GPIO 21)
        Analog 5 (GPIO33) -> SCL (GPIO 22)
        ```

2. Servo Motors
    - The rudder on top of a servo motor has holes that need to be rethreaded. One can use a "tap" for rethreading.
    - Before installing a servo, the servo's orientation needs to be adjusted to 90 degrees. The orientation of a servo (from 0 to 180 degrees) is shown below
        <p align="center">
        <img src="https://user-images.githubusercontent.com/39393023/135568554-f84da7c6-10e5-4773-9298-33f507092285.JPEG" height="400" width="width"/>
        <figcaption align="center">Note: the rudder is on the right hand side of the servo</figcaption>
        </p>

    - Some servos work well in the range ```[10, 170]``` degrees. [Source](https://www.intorobotics.com/how-to-control-servo-motors-with-arduino-no-noise-no-vibration/)

    - Servos take PWM signals as inputs. The output torque rating goes higher if the input voltage is higher.

    - **If you have a servo that vibrates, here are some possible reasons: ** [Source](https://electronicguidebook.com/reasons-why-a-servo-motor-vibrates/)
        - Power is instable, or not adequate. So get a larger power supply, or power cable
        - Working near the range limits
        - Motors are cheap so that parts don't work properly. **In my case, some motors did work better than others. I think that's because the good ones have better correspondence to the internal PID control loop**

    - Analog Servos' feedback can have around 1 degree of error. Which can introduce jittering in replay

3. Servo Motor Control: There are (at least) two ways to do this: Raspberry Pi, or Arduino (nano, uno, etc.)
    1. [Raspberry Pi I2C config](https://learn.adafruit.com/adafruits-raspberry-pi-lesson-4-gpio-setup/configuring-i2c). 
    2. [Arduino setup](https://wiki.keyestudio.com/Ks0173_keyestudio_Nano_ch340)
        - Power: 1. VIN 7v-12v 2. Mini-B USB
        - In Arduino IDE, select NANO as the board
        - Bootloader: choose ATMega328P (old bootloader)
        - Select USB port

    3. I2C Tool can scan I2C devices. Or SMBus devices (protocal based on I2C, a two wire communication).

4. WIFI Connection
    1. RJJE arm operates using an ESP32 microcontroller, which comes with a WIFI module. For communication, MQTT protocal is used. 
        - In a speed test, 1 request(30-byte)-response(30-byte) takes ~0.01s to finish. The router's upload and & download speeds are around ~130 & ~150 mbps. Therefore, we are using WIFI for real time robot arm control.
 

### Notes 
1. PCA9685 is not sitting well with ESP32, so this is still on going: 
    - After testing with esp32_control, PCA9685 stopped responing I2C requests in i2c_scanner. New PCA9685 ordered.
2. MQTT Speed test Result
    1. Test setup: a wifi network with upload / Download speed at 150Mb/s. We test the average total time of a "handshake", i.e., publish a message bidirectionally. 
    2. Results:
        - Need to get pub/sub on host machine (~0.07s per message exchange (30 bytes), but might take longer)
