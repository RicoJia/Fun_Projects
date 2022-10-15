#ifndef __ESP32_CONTROL_HPP__
#define __ESP32_CONTROL_HPP__

#include "servo_control.hpp"

/**
* @brief: This class encapsulates facilities for subscribe callbacks, motor control, and state publishing.
*/
class Esp32Control
{
public:
    Esp32Control (){}
    ~Esp32Control (){}

    /**
    * @brief: Tool function to return the "actual angles" in a string
    * @return: actual angles
    */
    String get_joint_states(){
        // e.g., msg: '0.00;0.00;0.00;0.00;0.00;0.00|'
        String msg = "link1_bracket_1;link_2_bracket_2_1;bracket_2_2_link_3;bracket_3_2_link_4;link_5_link_6;link_6_left_gripper|";
        for (byte i = 0; i < ARM_MOTOR_NUM; ++i) {
            msg += String(arm_actual_angles_[i]);
            msg += ";";
        }
        msg += String(claw_angle_) += ";";
        return msg;
    }

    /**
    * @brief: callback to update the 6th angle, hand angle.
    *   payload = 0, it will be closed, 1 will be open
    */
    void claw_sub_callback(const String & payload) {
        Serial.println("claw_sub_callback" + payload);
        if (payload == "0") {
            claw_angle_ = 0.0;
            claw_angle_unexecuted_ = true;
        }
        else if (payload == "1"){
            claw_angle_ = 1.0;
            claw_angle_unexecuted_ = true;
        } 
    }

    /**
    * @brief: Callback to update the 5 joint angles
    */
    void plan_sub_callback(const String & payload) {
        reset_waypoints();
        /* 
            payload has plans for waypoints. Each plan has 5 joint angles 3 digit precision + execution time |
            Ex: 1.57;-0.43;1.33;2.41;3.14;0.2;| , where 0.2 is the execution time
        */

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

   
    double get_claw_angle(){
        claw_angle_unexecuted_ = false;
        return claw_angle_;
    }

    /**
    * @brief Get the step angles of the arms if a series of new waypoints have been passed in 
    * @return ptr to angles for execution, which are the "current_angles"
    */
    double* get_arm_current_angles() {
        if (num_waypoints_ == 0) return nullptr;
        auto& commanded_angles = waypoints_[current_waypoint_];
        if (current_waypoint_ == num_waypoints_){
            reset_waypoints();
            return nullptr;
        }
        else {
            // when we are at the start of a waypoint
            if (!arm_step_angles_set_){
                for(unsigned char i =0; i < ARM_MOTOR_NUM; ++i){
                    arm_step_angles_[i] = (commanded_angles[i] - arm_actual_angles_[i])/execution_times_[current_waypoint_]/UPDATE_FREQUENCY;
                }
                arm_step_angles_set_ = true;
            }

            // Logic: update step angle; update current angle; if we can switch, switch
            for(unsigned char i =0; i < ARM_MOTOR_NUM; ++i){
                // when we approach the end of current waypoint
                if(abs(arm_step_angles_[i]) > abs(commanded_angles[i] - arm_actual_angles_[i])){
                    arm_step_angles_[i] = commanded_angles[i] - arm_actual_angles_[i]; 
                }
                arm_actual_angles_[i] += arm_step_angles_[i];
            }
        if (can_switch(commanded_angles)){
            arm_step_angles_set_ = false;
            current_waypoint_ += 1;
        } 
        return arm_actual_angles_;
        }
    }

    /**
     * @brief we publish heart beat every interval_ms  
     * @param interval_ms - interval between heart beats in milliseconds
     * @return true 
     * @return false 
     */
    bool should_publish_heartbeat(int interval_ms){
        if (millis() - last_heartbeat_time_ >= interval_ms){
            last_heartbeat_time_ = millis();
            return true;
        }
        return false;
    }

    bool claw_angle_unexecuted_ = false;
private:
    unsigned long last_heartbeat_time_ = millis();
    double arm_actual_angles_[ARM_MOTOR_NUM] = {0.0, 0.0, 0.0, 0.0, 0.0};
    double claw_angle_ = 0.0;

    double waypoints_[MAX_WAYPOINT_NUM][ARM_MOTOR_NUM];    // joint motors.
    double execution_times_[MAX_WAYPOINT_NUM];                         // time for for each waypoint
    byte num_waypoints_ = 0;
    byte current_waypoint_ = 0;
    bool arm_step_angles_set_ = false;
    double arm_step_angles_[ARM_MOTOR_NUM] = {0.0, 0.0, 0.0, 0.0, 0.0};

    /**
    * @brief: clear waypoints_, num_waypoints_, and execution_times_; then allocate memories for them
    */
    void reset_waypoints(){
        num_waypoints_ = 0;
        current_waypoint_ = 0; 
    }

    bool can_switch(double* commanded_angles){
        char stop_count = 0; 
        for (unsigned char i = 0; i < ARM_MOTOR_NUM; ++i) {
            if (abs(commanded_angles[i] - arm_actual_angles_[i]) < ANGULAR_THRESHOLD){
                ++stop_count;
            }
        }
        return stop_count == ARM_MOTOR_NUM; 
    }

    void fill_waypoint(const String& waypoint_payload, const byte& waypoint_i){
        int start_i = 0;
        for (byte i = 0; i < ARM_MOTOR_NUM + 1; ++i) {
            int delim_i = waypoint_payload.indexOf(';', start_i);
            auto data = waypoint_payload.substring(start_i, delim_i).toDouble();
            
            if (i == 5) execution_times_[waypoint_i] = data;
            else waypoints_[waypoint_i][i] = data;

            start_i = delim_i + 1;
        }

    }
};

#endif /* end of include guard: __ESP32_CONTROL_HPP__ */
