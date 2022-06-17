# RJJE Arm Hardware
This file is the design documentation of rjje_arm's hardware structure. 

## Motors
1. Servo Motors
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
