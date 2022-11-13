/*
If you try to run mosquitto_pub -t "esp/joint_states" -h IP
and get "connection refused", Make sure in your machine, run 
"mosquitto" to turn on the broker
Once it connects successfully to a Wifi network and a MQTT broker, it subscribe to a topic and send a message to it.
Ref: https://github.com/plapointe6/EspMQTTClient/issues/82
RJ: The previous example, using PubSubClient simply doesn't work. Pretty certain something is broken in that library.
    - You can do wildcard too: 
        client.subscribe("mytopic/wildcardtest/#", [](const String & topic, const String & payload) {
        Serial.println(topic + ": " + payload);
        });
Arduino IDE:
    - board: DOIT ESP32 DEVKIT V1
    - programmer: 
*/

#include <EspMQTTClient.h>
#include "esp32_control.hpp"
#include "servo_control.hpp"

EspMQTTClient client(
    /* "EngelWiFi", */
    /* "2394Engel", */
    /* "10.0.1.82", */
    "Unit-503",
    "3c1d2684",
    "100.66.47.29", // MQTT Broker server ip
    "MQTTUsername", // Can be omitted if not needed
    "MQTTPassword", // Can be omitted if not needed
    "ESP", // Client name that uniquely identify your device
    1883 // The MQTT port, default to 1883. this line can be omitted
);
Esp32Control esp32_control;
// This function is called once everything is connected (Wifi and MQTT)
void onConnectionEstablished()
{
    client.subscribe("esp/arm", plan_sub_callback);
    client.subscribe("esp/hand", claw_sub_callback);
}

/**
* Wrapper function because of signature EspMQTTClient::subscribe(static_function) 
*/
void claw_sub_callback(const String & payload) {
    esp32_control.claw_sub_callback(payload);
}

/**
* Wrapper function because of signature EspMQTTClient::subscribe(static_function)
*/
void plan_sub_callback(const String & payload) {
    esp32_control.plan_sub_callback(payload);
}

ServoControl* servo_control = nullptr;

void setup()
{
    Serial.begin(115200);
    Serial.println("Initializing WIFI MQTT Client and Servo");
    client.enableDebuggingMessages(); // Enable debugging messages sent to serial output 
    client.enableHTTPWebUpdater(); // Enable the web updater. User and password default to values of MQTTUsername and MQTTPassword. These can be overrited with enableHTTPWebUpdater("user", "password").
    client.enableLastWillMessage("TestClient/lastwill", "I am going offline"); // You can activate the retain flag by setting the third parameter to true 
    // 16kb, approximately 120 waypoints, which should be way more than enough. 
    // Without this we can send up to 256 bytes.
    client.setMaxPacketSize(16384);
    Serial.println("WIFI MQTT Client and Control Initialized");
    servo_control = new ServoControl(); 
    Serial.println("Servo Control Initialized");
    Serial.println("RJJE arm initialized");
}


void loop()
{
    // loop is non-blocking
    client.loop();
    client.enableDebuggingMessages(false);
    unsigned long start_time = millis();
    double* arm_execution_angles = esp32_control.get_arm_current_angles();
    if (arm_execution_angles != nullptr){
        servo_control->execute_arm_angles(arm_execution_angles);
    }
    if (esp32_control.claw_angle_unexecuted_){
        servo_control->execute_claw_angle(esp32_control.get_claw_angle());
    }
    client.publish("esp/joint_states", esp32_control.get_joint_states());
    if (esp32_control.should_publish_heartbeat(1000)){
        client.publish("esp/heartbeat", "heartbeat");
    }
    unsigned long diff = millis() - start_time;
    if (diff < (unsigned long)1000/UPDATE_FREQUENCY) delay((unsigned long)1000/UPDATE_FREQUENCY - diff);
}
