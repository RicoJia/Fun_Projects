# RJJE Arm Hardware
This file is the design documentation of rjje_arm's hardware structure.
    1. ```esp32_control``` is under active development. servo_control will be merged into ```esp32_control``` and get deprecated.
    2. The [System diagram can be accessed using draw.io](https://drive.google.com/file/d/1ujubSrS_AvXeORWJ76qhUnCQ4BP0E4v_/view?usp=sharing) 

## Usage
1. Open Arduino IDE, install these libraries using ```library manager```
    - ```Adafruit_PWMServoDriver```
    - ```EspMQTTClient```
2. On Arduino IDE, setup ESP32: [see here](https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/)
2. To Control the claw, do ```rosservice call /rjje_arm/claw_control OPEN_VAL```, where ```OPEN_VAL``` = 0 for closing, 1 for opening

## Design
### Motors
1. Simple Motor Testing: [Working Adafruit_PCA9685 video, including schematics](https://www.youtube.com/watch?v=y8X9X10Tn1k)
    - Electrical: 
        - Jack plug for external power 
        - Vcc is positive for signal, V+ is the positive supply. 
        - switching directions will cause a lot of noise on the supply. 
        - [Optional]may need a cap, like 470 uF for many motors.  
    - Some notes: 
        1. The working frequency is around 60hz, not 50hz
        2. Some brands of PCA9685 requires V+ to be connected to 5v.
    - on ESP32: [Nice article on ESP 32 - PCA9865](https://dronebotworkshop.com/esp32-servo/)  

        <p align="center">
        <img src="https://user-images.githubusercontent.com/106101331/175851195-075cdfe5-a3cd-4bd5-86f6-e74e4d874305.png" height="400" width="width"/>
        <figcaption align="center">Pinouts of ESP32</figcaption>
        </p>

        - Arduino Nano -> ESP32 pinouts:
            ```
            Analog 4 (GPIO32) -> SDA (GPIO 21)
            Analog 5 (GPIO33) -> SCL (GPIO 22)
            ```
        - ESP 32 connections -> PCA9685
            ``` 
            VIN -> Vcc (5v)
            GND -> GND
            D21 -> SDA
            D22 -> SCL
            ```
    - The rudder on top of a servo motor has holes that need to be rethreaded. One can use a "tap" for rethreading.
        
2. Servo Calibration.
    - [How to calibrate servos](https://create.arduino.cc/projecthub/jeremy-lindsay/calibrating-my-servos-fa27ce)
    - the servo's orientation needs to be adjusted to 90 degrees. The orientation of a servo (from 0 to 180 degrees) is shown below
        <p align="center">
        <img src="https://user-images.githubusercontent.com/39393023/135568554-f84da7c6-10e5-4773-9298-33f507092285.JPEG" height="400" width="width"/>
        <figcaption align="center">Note: the rudder is on the right hand side of the servo</figcaption>
        </p>

    - Some servos work well in the range ```[10, 170]``` degrees. [Source](https://www.intorobotics.com/how-to-control-servo-motors-with-arduino-no-noise-no-vibration/)

    - Servos take PWM signals as inputs. The output torque rating goes higher if the input voltage is higher.

3. Servo Motors Trouble Shooting
    1. Use the [simple servo script](https://docs.arduino.cc/learn/electronics/servo-motors) to test if the servo microcontroller are good
    2. **Use I2C tool if you have trouble connecting to PCA9865**. I2C Tool can scan I2C devices. Or SMBus devices (protocal based on I2C, a two wire communication). [Link](https://www.arduino.cc/reference/en/libraries/i2cdetect/)
    3. Get spare motors and microcontrollers. If servos are still not working, try different Servo control script on the internet as well 
    4. **If you have a servo that vibrates, here are some possible reasons: ** [Source](https://electronicguidebook.com/reasons-why-a-servo-motor-vibrates/)
        - Power is instable, or not adequate. So get a larger power supply, or power cable
        - Working near the range limits
        - Motors are cheap so that parts don't work properly. **In my case, some motors did work better than others. I think that's because the good ones have better correspondence to the internal PID control loop**
    5. Analog Servos' feedback can have around 1 degree of error. Which can introduce jittering in replay

### WIFI
1. WIFI Connection
    1. RJJE arm operates using an ESP32 microcontroller, which comes with a WIFI module. For communication, MQTT protocal is used. 
        - In a speed test, 1 request(30-byte)-response(30-byte) takes ~0.01s to finish. The router's upload and & download speeds are around ~130 & ~150 mbps. Therefore, we are using WIFI for real time robot arm control.
### Development Notes 
1. MQTT Speed test Result
    1. Test setup: a wifi network with upload / Download speed at 150Mb/s. We test the average total time of a "handshake", i.e., publish a message bidirectionally. 
    2. Results:
        - Need to get pub/sub on host machine (~0.07s per message exchange (30 bytes), but might take longer)
