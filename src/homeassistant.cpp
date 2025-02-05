#include <PsychicMqttClient.h>
#include <ArduinoJson.h>
#include <esp_wifi.h>
#include <DW1000Ng.hpp>
#include "driver/temp_sensor.h"
#include "Preferences.h"
#include "motor.hpp"

#include "homeassistant.hpp"
#include "secrets.h"
#include "network.hpp"

#ifdef MOTOR_TMC2209
HomeAssistant::HomeAssistant(Preferences *preferences, DW1000 *dw1000, Motor* motor)
#else
HomeAssistant::HomeAssistant(Preferences *preferences, DW1000 *dw1000)
#endif
{
    this->mDw1000 = dw1000;
    this->mPreferences = preferences;
    #ifdef MOTOR_TMC2209
    this->mMotor = motor;
    #endif
    this->mNextScheduledStateSend = 0;

    // legacy esp32 temp sensor
    temp_sensor_config_t temp_sensor = TSENS_CONFIG_DEFAULT();
    temp_sensor.dac_offset = TSENS_DAC_L2; // TSENS_DAC_L2 is default; L4(-40°C ~ 20°C), L2(-10°C ~ 80°C), L1(20°C ~ 100°C), L0(50°C ~ 125°C)
    temp_sensor_set_config(temp_sensor);
    temp_sensor_start();

    // zero out tags
    for (int i = 0; i < 8; i++)
    {
        this->mTagDistances[i].distance = -1;
    }
}
boolean HomeAssistant::connect()
{
    Serial.println("Connecting to MQTT server");
    this->mMqttClient.setServer(MQTT_SERVER);

    this->mMqttClient.onConnect([&](bool sessionPresent)
                                { Serial.println("Connected to MQTT server"); });

    this->mMqttClient.connect();

    while (!this->mMqttClient.connected())
    {
        delay(1000);
        Serial.println("Waiting for MQTT connection...");
    }

    uint8_t macAddr[6];
    esp_wifi_get_mac(WIFI_IF_STA, macAddr);

    this->sendOverallDiscovery();
    this->sendNumericStateDiscovery("antennaDelay", "number", "", "", macAddr, 
    [](JsonDocument &doc) {
        doc["min"] = 10000;
        doc["max"] = 65000;
        doc["step"] = 1;
    });

#ifdef DW1000_ANCHOR
    this->sendAnchorCoordinateDiscovery("x");
    this->sendAnchorCoordinateDiscovery("y");
    this->sendAnchorCoordinateDiscovery("z");
#endif

// tags get sensor for x/y/z to make it easier to set
#ifdef DW1000_TAG
    this->sendNumericStateDiscovery("x", "sensor", "distance", "m", macAddr, [](JsonDocument &doc) {
        doc["min"] = 0,
        doc["max"] = 100,
        doc["step"] = 0.1;
    });
    this->sendNumericStateDiscovery("y", "sensor", "distance", "m", macAddr, [](JsonDocument &doc) {
        doc["min"] = 0,
        doc["max"] = 100,
        doc["step"] = 0.1;
    });
    this->sendNumericStateDiscovery("z", "sensor", "distance", "m", macAddr, [](JsonDocument &doc) {
        doc["min"] = 0,
        doc["max"] = 100,
        doc["step"] = 0.1;
    });
#endif

#ifdef MOTOR_TMC2209
    // The one set by home assistant flow
    this->sendNumericStateDiscovery("angle", "number", "", "°", macAddr, [](JsonDocument &doc) {
        doc["min"] = 0,
        doc["max"] = 360,
        doc["step"] = 0.1;
    });
    // the current angle of the motor
    this->sendNumericStateDiscovery("angle", "sensor", "", "°", macAddr, [](JsonDocument &doc) {
        doc["min"] = 0,
        doc["max"] = 360,
        doc["step"] = 0.1;
    });
    // the offset of the motor set by home assistant - manual
    this->sendNumericStateDiscovery("angleOffset", "number", "", "°", macAddr, [](JsonDocument &doc) {
        doc["min"] = 0,
        doc["max"] = 360,
        doc["step"] = 0.1;
    });
#endif

    this->mMqttClient.onMessage([&](char *topic, char *payload, int retain, int qos, bool dup)
                                { 
        // figure out the axis based on the topic
        String topicStr = String(topic);
        String payloadStr = String(payload);
        debugV("MQTT: received message on topic %s, payload: %s", topicStr.c_str(), payloadStr.c_str());
        // remove /set from the end of topic
        topicStr.remove(topicStr.length() - 4);
        if(topicStr.endsWith("-x")) {
            this->sendNumericState("x", "number", atof(payload));
            debugV("MQTT: Set x to %f", atof(payload));
        } else if(topicStr.endsWith("-y")) {
            this->sendNumericState("y", "number", atof(payload));
            debugV("MQTT: Set y to %f", atof(payload));
        } else if(topicStr.endsWith("-z")) {
            this->sendNumericState("z", "number", atof(payload));
            debugV("MQTT: Set z to %f", atof(payload));
        } else if(topicStr.endsWith("-antennaDelay")) {
            int antennaDelay = atoi(payload);
            this->mPreferences->putInt("antennaDelay", antennaDelay);
            DW1000Ng::setAntennaDelay(antennaDelay);
            this->sendNumericState("antennaDelay", "number", antennaDelay);
            debugV("MQTT: Set antenna delay to %d", antennaDelay);
        } else if(topicStr.endsWith("-angle")) {
            this->sendNumericState("angle", "number", atof(payload));
            debugV("MQTT: Set angle to %f", atof(payload));
            #ifdef MOTOR_TMC2209
            this->mMotor->update(atof(payload));
            #endif
        } else if(topicStr.endsWith("-angleOffset")) {
            this->sendNumericState("angleOffset", "number", atof(payload));
            debugV("MQTT: Set angle offset to %f", atof(payload));
        } else {
            debugV("MQTT: received message on unknown topic %s", topicStr.c_str());
        } });

    return true;
}

void HomeAssistant::handle()
{
    if (!this->mMqttClient.connected())
    {
        this->connect();
    }

    // overall state is only sent every 10 seconds
    if (millis() > this->mNextScheduledStateSend)
    {
        this->sendOverallState();

        this->mNextScheduledStateSend = millis() + 10000;
    }

// device ranging state sent every loop
#ifdef DW1000_ANCHOR
    uint8_t knownTagCount = this->mDw1000->getKnownTagCount();
    for (uint8_t i = 0; i < knownTagCount; i++)
    {
        DW1000::TagDistance *tagDistance = this->mDw1000->getKnownTag(i);

        // if we haven't sent a discovery message for this tag yet, do so
        if (this->mTagDistances[i].distance == -1)
        {
            this->sendTagDiscovery(tagDistance->eui);
            this->mTagDistances[i] = *tagDistance;
        }

        // if distance hasn't changed, don't send
        if (abs(this->mTagDistances[i].distance - tagDistance->distance) > 0.01)
        {
            // make sure we're not sending garbage
            if (tagDistance->distance > 0.1 && tagDistance->distance < 100)
            {
                debugV("MQTT: sending distance to tag %02x%02x%02x%02x%02x%02x%02x%02x: %f", tagDistance->eui[0], tagDistance->eui[1], tagDistance->eui[2], tagDistance->eui[3], tagDistance->eui[4], tagDistance->eui[5], tagDistance->eui[6], tagDistance->eui[7], tagDistance->distance);

                this->sendTagDistanceToAnchorEUI(tagDistance->distance, tagDistance->eui);
                // store tag distance
                this->mTagDistances[i] = *tagDistance;
            }
        }
    }
#endif
}

String HomeAssistant::getDeviceName()
{
    // get mac address for device name
    uint8_t macAddr[6];
    esp_wifi_get_mac(WIFI_IF_STA, macAddr);

#ifdef DW1000_ANCHOR
    return this->getDeviceName(macAddr, false);
#elif defined(DW1000_TAG)
    return this->getDeviceName(macAddr, true);
#endif
}

String HomeAssistant::getDeviceName(byte macAddr[], boolean tag)
{
    char macAddrStr[20];
    sprintf(macAddrStr, "%02x%02x%02x%02x%02x%02x\0", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);

    String deviceName = String("dw1000-");
    if (tag)
    {
        deviceName.concat("tag-");
    }
    else
    {
        deviceName.concat("anchor-");
    }
    deviceName.concat(macAddrStr);
    return deviceName;
}

void HomeAssistant::sendOverallDiscovery()
{

    uint8_t macAddr[6];
    esp_wifi_get_mac(WIFI_IF_STA, macAddr);
    // convert to string
    char macAddrStr[20];
    sprintf(macAddrStr, "%02x%02x%02x%02x%02x%02x\0", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);

    String deviceName = this->getDeviceName();
    String discoveryTopic = "homeassistant/sensor/" + deviceName + "/config";

    JsonDocument doc;
    char buffer[1024];

#ifdef DW1000_ANCHOR
    addCommonDiscovery(doc, macAddr, false);
#elif defined(DW1000_TAG)
    addCommonDiscovery(doc, macAddr, true);
#endif

    doc["state_topic"] = "homeassistant/sensor/" + deviceName + "/state";

    // doc["p"] = "sensor";
    doc["device_class"] = "temperature";
    doc["unit_of_measurement"] = "°C";
    doc["value_template"] = "{{ value_json.temperature }}";
    doc["unique_id"] = String("dwT-") + deviceName;

    size_t n = serializeJson(doc, buffer);

    Serial.println("Sending discovery message");
    Serial.println(discoveryTopic);
    Serial.println(buffer);

    this->mMqttClient.publish(discoveryTopic.c_str(), 2, true, buffer, n);
}

void HomeAssistant::sendTagDiscovery(byte tag_eui[])
{
    uint8_t macAddr[6];
    esp_wifi_get_mac(WIFI_IF_STA, macAddr);
    // convert to string
    char macAddrStr[20];
    sprintf(macAddrStr, "%02x%02x%02x%02x%02x%02x\0", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);

    byte tagMacAddr[6];
    for (int i = 0; i < 6; i++)
    {
        tagMacAddr[i] = tag_eui[5 - i];
    }
    char tagMacAddrStr[20];
    sprintf(tagMacAddrStr, "%02x%02x%02x%02x%02x%02x\0", tagMacAddr[0], tagMacAddr[1], tagMacAddr[2], tagMacAddr[3], tagMacAddr[4], tagMacAddr[5]);

    // first 2 bytes of EUI are dummy - the actual mac address of the esp32 is the last 6 bytes
    String tagDeviceName = this->getDeviceName(tagMacAddr, true);
    String discoveryTopic = "homeassistant/sensor/" + tagDeviceName + "-" + macAddrStr + "/config";

    JsonDocument doc;
    // reverse tag eui to get the mac address

    // addCommonDiscovery(doc, tagMacAddr, true);

    doc["name"] = macAddrStr + String("dist");
    doc["device_class"] = "distance";
    doc["unit_of_measurement"] = "m";
    doc["value_template"] = "{{ value_json.distance }}";
    doc["unique_id"] = "dwD-" + tagDeviceName + '-' + macAddrStr;
    doc["state_topic"] = "homeassistant/sensor/" + tagDeviceName + "-" + macAddrStr + "/state";
    JsonObject dev = doc["dev"].to<JsonObject>();
    JsonArray ids = dev["ids"].to<JsonArray>();
    ids.add(tagMacAddrStr);

    char buffer[1024];
    size_t n = serializeJson(doc, buffer);
    debugV("Sending discovery message %s", buffer);
    this->mMqttClient.publish(discoveryTopic.c_str(), 2, true, buffer, n);
}

void HomeAssistant::addCommonDiscovery(JsonDocument &doc, byte macAddr[], boolean tag)
{
    char macAddrStr[20];
    sprintf(macAddrStr, "%02x%02x%02x%02x%02x%02x\0", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
    String deviceName = this->getDeviceName(macAddr, tag);

    JsonObject dev = doc["dev"].to<JsonObject>();
    JsonArray ids = dev["ids"].to<JsonArray>();
    ids.add(macAddrStr);

    dev["name"] = deviceName;
    if (tag)
    {
        dev["mdl"] = "DW1000 Tag";
    }
    else
    {
        dev["mdl"] = "DW1000 Anchor";
    }
    dev["mf"] = "JackFromScratch";
    dev["sw"] = "0.1.0";
    dev["sn"] = macAddrStr;
    dev["hw"] = "0.1";

    doc["qos"] = 0;
    JsonObject origin = doc["o"].to<JsonObject>();
    origin["name"] = "JackFromScratch";
}

// send the anchor discovery message for x/y/z coordinates to HomeAssistant
// this mainly lets home assistant set coordinates for the anchor, we don't actually use this in the code
void HomeAssistant::sendAnchorCoordinateDiscovery(String axis)
{
    uint8_t macAddr[6];
    esp_wifi_get_mac(WIFI_IF_STA, macAddr);

    sendNumericStateDiscovery(axis, "number", "distance", "m", macAddr, [](JsonDocument &doc) {
        doc["min"] = 0,
        doc["max"] = 100,
        doc["step"] = 0.1;
    });
}

void HomeAssistant::sendNumericStateDiscovery(String name, String deviceType, String deviceClass, String unitOfMeasurement, byte macAddr[],
                                              std::function<void(JsonDocument &)> customCallback = nullptr )
{
    // convert to string
    char macAddrStr[20];
    sprintf(macAddrStr, "%02x%02x%02x%02x%02x%02x\0", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);

    String deviceName = this->getDeviceName();

    String baseTopic = "homeassistant/" + deviceType + "/" + deviceName + "-" + name;

    String discoveryTopic = baseTopic + "/config";

    JsonDocument doc;
    char buffer[1024];

    doc["name"] = name;
    if (deviceClass != "")
    {
        doc["device_class"] = deviceClass;
    }
    if (unitOfMeasurement != "")
    {
        doc["unit_of_measurement"] = unitOfMeasurement;
    }
    doc["value_template"] = "{{ value_json." + name + " }}";
    doc["state_topic"] = baseTopic + "/state";
    doc["command_topic"] = baseTopic + "/set";
    customCallback(doc);
    /*doc["min"] = 0;
    doc["max"] = 100;
    doc["step"] = 0.1;*/
    doc["unique_id"] = String("dw") + name + "-" + deviceName;
    JsonObject dev = doc["dev"].to<JsonObject>();
    JsonArray ids = dev["ids"].to<JsonArray>();
    ids.add(macAddrStr);

    size_t n = serializeJson(doc, buffer);

    this->mMqttClient.publish(discoveryTopic.c_str(), 2, true, buffer, n);
    // make sure we subscribe to the command topic topic
    this->mMqttClient.subscribe((baseTopic + "/set").c_str(), 2);
}

void HomeAssistant::sendNumericState(String axis, String deviceType, float value)
{
    String stateTopic = "homeassistant/" + deviceType + "/" + this->getDeviceName() + "-" + axis + "/state";
    JsonDocument doc;

    doc[axis] = value;

    char buffer[128];

    size_t n = serializeJson(doc, buffer);

    Serial.println("Sending state message");
    Serial.println(stateTopic);
    Serial.println(buffer);

    bool success = this->mMqttClient.publish(stateTopic.c_str(), 2, false, buffer, n);

    // store value in flash
}

void HomeAssistant::sendTagDistanceToAnchorEUI(float distance, byte tag_eui[])
{
    // since adding an attribute to a device isn't enforced that it is sent actually BY the device,
    // we can spoof it and send it from the anchor, since it knows the distance
    // this saves power and time on the tag
    uint8_t macAddr[6];
    esp_wifi_get_mac(WIFI_IF_STA, macAddr);
    // convert to string
    char macAddrStr[20];
    sprintf(macAddrStr, "%02x%02x%02x%02x%02x%02x\0", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);

    byte tagMacAddr[6];
    for (int i = 0; i < 6; i++)
    {
        tagMacAddr[i] = tag_eui[5 - i];
    }
    // char tagMacAddrStr[20];
    // sprintf(tagMacAddrStr, "%02x%02x%02x%02x%02x%02x\0", tagMacAddr[0], tagMacAddr[1], tagMacAddr[2], tagMacAddr[3], tagMacAddr[4], tagMacAddr[5]);

    String tagDeviceName = this->getDeviceName(tagMacAddr, true);

    String stateTopic = "homeassistant/sensor/" + tagDeviceName + "-" + macAddrStr + "/state";

    JsonDocument doc;
    doc["distance"] = distance;
    char buffer[128];
    size_t n = serializeJson(doc, buffer);
    this->mMqttClient.publish(stateTopic.c_str(), 2, false, buffer, n);
}

void HomeAssistant::sendOverallState()
{
    String stateTopic = "homeassistant/sensor/" + this->getDeviceName() + "/state";
    JsonDocument doc;

    float temperature;
    temp_sensor_read_celsius(&temperature);
    debugV("DW1000 temperature: %f", temperature);
    debugV("stateTopic: %s", stateTopic.c_str());

    doc["temperature"] = temperature;

    char buffer[128];

    size_t n = serializeJson(doc, buffer);

    Serial.println("Sending state message");
    Serial.println(stateTopic);
    Serial.println(buffer);

    bool success = this->mMqttClient.publish(stateTopic.c_str(), 2, false, buffer, n);
}
