/*
Once it connects successfully to a Wifi network and a MQTT broker, it subscribe to a topic and send a message to it.
Ref: https://github.com/plapointe6/EspMQTTClient/issues/82
RJ: The previous example, using PubSubClient simply doesn't work. Pretty certain something is broken in that library.
    - You can do wildcard too: 
        client.subscribe("mytopic/wildcardtest/#", [](const String & topic, const String & payload) {
        Serial.println(topic + ": " + payload);
        });

*/

#include "EspMQTTClient.h"
#include "esp32_control.hpp"

EspMQTTClient client(
    /* "EngelWiFi", */
    /* "2394Engel", */
    /* "10.0.1.82", */
    "Unit-380",
    "3ecbe779",
    "100.65.212.26", // MQTT Broker server ip
    "MQTTUsername", // Can be omitted if not needed
    "MQTTPassword", // Can be omitted if not needed
    "ESP", // Client name that uniquely identify your device
    1883 // The MQTT port, default to 1883. this line can be omitted
);
Esp32Control esp32_control;

double desired_angles[6];
double actual_angles[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
/* double (*waypoints)[5] = nullptr; */
double **waypoints = nullptr;
byte num_waypoints = 0;

const int FREQ = 10;

// This function is called once everything is connected (Wifi and MQTT)
void onConnectionEstablished()
{
    client.subscribe("esp/plan", plan_sub_callback);
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

void setup()
{
    Serial.begin(115200);
    esp32_control = Esp32Control();
    client.enableDebuggingMessages(); // Enable debugging messages sent to serial output
    client.enableHTTPWebUpdater(); // Enable the web updater. User and password default to values of MQTTUsername and MQTTPassword. These can be overrited with enableHTTPWebUpdater("user", "password").
    client.enableLastWillMessage("TestClient/lastwill", "I am going offline"); // You can activate the retain flag by setting the third parameter to true
    Serial.println("Initialized");
}


void loop()
{
    // loop is non-blocking
    unsigned long start_time = millis();
    client.loop();
    client.enableDebuggingMessages(true);
    client.publish("esp/joint_states", esp32_control.get_joint_states());
    unsigned long diff = millis() - start_time;
    if (diff < (unsigned long)1000/FREQ) delay((unsigned long)1000/FREQ - diff);
}
