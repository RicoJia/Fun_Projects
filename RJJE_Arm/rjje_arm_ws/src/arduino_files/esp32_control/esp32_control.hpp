#ifndef __ESP32_CONTROL_HPP__
#define __ESP32_CONTROL_HPP__

#include "servo_control.hpp"

const byte MOTOR_NUM = 6; 
const byte MAX_WAYPOINT_NUM = 64;

/**
* @brief: This class encapsulates facilities for subscribe callbacks, motor control, and state publishing.
*/
class Esp32Control
{
public:
    Esp32Control (){}
    ~Esp32Control (){}

    String get_joint_states(){
        String msg;
        //TODO
        if (waypoints_ != nullptr){
            for (unsigned int i = 0; i < 5; ++i) {
                actual_angles_[i] = waypoints_[current_waypoint][i];
            }
        }
        for (unsigned int i = 0; i < 4; ++i) {
            Serial.println(String(execution_times_[0]) + "exc time");
        }

        actual_angles_[MOTOR_NUM-1] = claw_angle_;

        for (byte i = 0; i < 6; ++i) {
            msg += String(actual_angles_[i]);
            msg += ";";
        }
        return msg;
    }

    /**
    * @brief: callback to update the 6th angle, hand angle.
    *   payload = 0, it will be closed, 1 will be open
    */
    void claw_sub_callback(const String & payload) {
        if (payload == "0") {
            claw_angle_ = 0.0;
        }
        else if (payload == "1"){
            claw_angle_ = 90.0;
        } 
    }

    /**
    * @brief: Callback to update the 5 joint angles
    */
    void plan_sub_callback(const String & payload) {
        reset_waypoints();
        /* payload has plans for waypoints. Each plan has 5 joint angles 3 digit precision + execution time | Ex: 12.3;32.2;23.0;45.4;66.2;0.02;| , where 0.02 is the execution time*/

        int waypoint_start_i = 0;
        for(byte i = 0; i < MAX_WAYPOINT_NUM; ++i){
            int delim_i = payload.indexOf('|', waypoint_start_i);
            if (delim_i == -1) break;
            auto waypoint_payload = payload.substring(waypoint_start_i, delim_i);
            fill_waypoint(waypoint_payload, num_waypoints_);
            waypoint_start_i = delim_i + 1;
            num_waypoints_ += 1;
        }
    }
private:
    double actual_angles_[MOTOR_NUM] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double claw_angle_ = 0.0;

    double waypoints_[MAX_WAYPOINT_NUM][MOTOR_NUM-1];    // joint motors.
    double execution_times_[MAX_WAYPOINT_NUM];                         // time for for each waypoint
    byte num_waypoints_ = 0;
    byte current_waypoint = 0;

    /**
    * @brief: clear waypoints_, num_waypoints_, and execution_times_; then allocate memories for them
    */
    void reset_waypoints(){
        num_waypoints_ = 0;
    }

    void fill_waypoint(const String& waypoint_payload, const byte& waypoint_i){
        int start_i = 0;
        for (byte i = 0; i < 6; ++i) {
            int delim_i = waypoint_payload.indexOf(';', start_i);
            auto data = waypoint_payload.substring(start_i, delim_i).toDouble();
            
            if (i == 5) execution_times_[waypoint_i] = data;
            else waypoints_[waypoint_i][i] = data;

            start_i = delim_i + 1;
        }

    }
};

#endif /* end of include guard: __ESP32_CONTROL_HPP__ */
