#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <PsychicMqttClient.h>
#include <Preferences.h>

#ifdef MOTOR_TMC2209
#include "motor.hpp"
#endif
#include "dw1000.hpp"

class HomeAssistant {
    public:
    #ifdef MOTOR_TMC2209
    HomeAssistant(Preferences* preferences, DW1000* dw1000, Motor* motor);
    #else
    HomeAssistant(Preferences* preferences, DW1000* dw1000);
    #endif

    boolean connect();
    
    void handle();

    private:
        PsychicMqttClient mMqttClient;
        DW1000* mDw1000;
        #ifdef MOTOR_TMC2209
        Motor* mMotor;
        #endif
        
        Preferences* mPreferences;
        String getDeviceName();
        String getDeviceName(byte macAddr[], boolean tag);
        /**
         * Sends discovery message for the "overall" device, i.e just registers with entities that all devices have.
         */
        void sendOverallDiscovery();
        /**
         * Sends a discovery of the anchor on the tag entity in HomeAssistant.
         * This is just so the distances to the anchors is displayed in HomeAssistant on the tag as opposed to being on the anchor
         * since the ranging results aren't sent to the tag.
         */
        void sendTagDiscovery(byte tag_eui[]);
        /**
         * Adds common discovery attributes to the JSON document.
         */
        
        void addCommonDiscovery(JsonDocument &doc, byte macAddr[], boolean tag);

        /**
         * Sends a message to home assistant to have x/y/z coordinates displayed for the anchor.
         *  We don't actually use this in the code, this is just so they exist for node-red to use in its calculations.
         */
        void sendAnchorCoordinateDiscovery(String axis);
        
        /**
         * sends a message to home assistant about a numeric state discovery with attributes configurable.
         */
        void sendNumericStateDiscovery(String name, String deviceType, String deviceClass, String unitOfMeasurement, byte macAddr[], std::function<void(JsonDocument &)> customCallback);
        
        void sendNumericState(String name, String deviceType, float value);
        
        /**
         * USED BY ANCHORS ONLY
         * Sends a message to the MQTT server with the distance to the tag.
         * This also attributes the distance to the tag's EUI in HomeAssistant under the same device.
         */
        void sendTagDistanceToAnchorEUI(float distance, byte tag_eui[]);
        void sendOverallState();
        unsigned long mNextScheduledStateSend;
        DW1000::TagDistance mTagDistances[8];
};