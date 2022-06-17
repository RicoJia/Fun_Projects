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

EspMQTTClient client(
    "EngelWiFi",
    "2394Engel",
    "10.0.1.82",
    /* "Unit-380", */
    /* "3ecbe779", */
    /* "100.65.212.26", // MQTT Broker server ip */
    "MQTTUsername", // Can be omitted if not needed
    "MQTTPassword", // Can be omitted if not needed
    "ESP", // Client name that uniquely identify your device
    1883 // The MQTT port, default to 1883. this line can be omitted
);
unsigned long start_time;
double angles[6];

void sub_callback(const String & payload) {
    /* payload looks has 3 digit precision 12.3;32.2;23.0;45.4;66.2;77.1; */
    int start_i = 0;
    for (byte i = 0; i < 6; ++i) {
        int delim_i = payload.indexOf(';', start_i);
        angles[i] = payload.substring(start_i, delim_i).toDouble();
        start_i = delim_i + 1;
    }
}

void setup()
{
    Serial.begin(115200);
    client.enableDebuggingMessages(); // Enable debugging messages sent to serial output
    client.enableHTTPWebUpdater(); // Enable the web updater. User and password default to values of MQTTUsername and MQTTPassword. These can be overrited with enableHTTPWebUpdater("user", "password").
    client.enableLastWillMessage("TestClient/lastwill", "I am going offline"); // You can activate the retain flag by setting the third parameter to true
    start_time = millis();
}

// This function is called once everything is connected (Wifi and MQTT)
void onConnectionEstablished()
{
    client.subscribe("esp/plan", sub_callback);
    client.publish("esp/joint_states", "this is a message");
}

void loop()
{
    // loop is non-blocking
    client.loop();
    client.enableDebuggingMessages(true);
    /* Serial.println("love"); */
}
