#include <Arduino.h>
#include <QMC5883LCompass.h>
#include <Adafruit_LSM6DSL.h>
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgTime.hpp>
#include <DW1000NgConstants.hpp>
#include <DW1000NgRanging.hpp>
#include <DW1000NgRTLS.hpp>

#include "dw1000.hpp"
#include "network.hpp"
#include <WiFi.h>
#include <esp_wifi.h>
#include "homeassistant.hpp"
#include <TMCStepper.h>
#include <Preferences.h>


#ifdef MOTOR_TMC2209
#include "motor.hpp"
#define STEPS_PER_REV 10000 // todo calibrate
#define STEP_PIN 7
#define DIR_PIN 6
Motor* stepper = new Motor(STEP_PIN, DIR_PIN, STEPS_PER_REV);

void interrupt() {
    stepper->motorInterrupt();
} 
#endif

Preferences* preferences;
QMC5883LCompass compass;
Adafruit_LSM6DSL lsm6dsl;
Adafruit_Sensor *lsm_temp, *lsm_accel, *lsm_gyro;
Network network;
DW1000* dw1000;
HomeAssistant* homeAssistant;



boolean compassWorking = false;
boolean accelWorking = false;


#define DWM1000_TAG 0
#define DWM1000_ANCHOR 1
#define DWM1000_MODE DWM1000_ANCHOR

volatile uint32_t blink_rate = 200;

const uint8_t DWM1000_RST = 9; // need to bodge this in hardware - forgot to connect
const uint8_t DWM1000_IRQ = 3; 
const uint8_t DWM1000_CS = 21;




void setup() {
    Serial.begin(921600);
    // set a pin high for testing
    pinMode(14, INPUT); // QMC5883L data ready pin
    Serial.println("Hello World!");

    preferences = new Preferences();
    preferences->begin("dw1000", false);

  #ifdef MOTOR_TMC2209
  
        // register timer interrupt for motor
    hw_timer_t *timer = NULL;
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, interrupt, true);
    timerAlarmWrite(timer, 1000, true);
    timerAlarmEnable(timer);
    
  #endif
    

    network.connect();


    Wire.begin(47,48);

    byte error;

    // compass should live at 0x0D
    // need to test because at least one module is broken
    Wire.beginTransmission(0x0D);
    error = Wire.endTransmission();
    if (error == 0) {
        Serial.println("QMC5883L found at 0x0D");
        compassWorking = true;
        compass.init();
    } else {
        Serial.println("Compass 5883L not found at 0x0D - is it broken?");
    }

    // accel should live at 0x68
    Wire.beginTransmission(0x6B);
    error = Wire.endTransmission();
    if (error == 0) {
        Serial.println("LSM6DSLTR found at 0x6B");
        accelWorking = true;
        
        lsm6dsl.begin_I2C(0x6B, &Wire);
        lsm_temp = lsm6dsl.getTemperatureSensor();
        lsm_accel = lsm6dsl.getAccelerometerSensor();
        lsm_gyro = lsm6dsl.getGyroSensor();
    } else {
        Serial.println("LSM6DSLTR not found at 0x6B - is it broken?");
    }

    Serial.println("Initializing SPI...");
    pinMode(DWM1000_CS, OUTPUT);

    SPI.begin(18, 19, 20);

    Serial.println("SPI initialized");

    uint8_t macAddr[6];
    esp_wifi_get_mac(WIFI_IF_STA, macAddr);
    
    dw1000 = new DW1000(preferences, DWM1000_CS, DWM1000_IRQ, DWM1000_RST, macAddr);

#if defined(DW1000_ANCHOR) || defined(DW1000_TAG)
#ifdef MOTOR_TMC2209
    homeAssistant = new HomeAssistant(preferences, dw1000, stepper);
    #else
    homeAssistant = new HomeAssistant(preferences, dw1000);
    #endif
    
    homeAssistant->connect();
    
#endif
}

void loop() {
  network.handle();
  
// if anchor or tag run actual program
#if defined(DW1000_ANCHOR) || defined(DW1000_TAG)
  dw1000->handle();
  homeAssistant->handle();
#endif

#ifdef MOTOR_TMC2209
  // rotate 1 degree
  //stepper->step();
  // toggle step pin
  digitalWrite(STEP_PIN, !digitalRead(STEP_PIN));
#endif

  
  /*if(compassWorking) {
    compass.read();
    Serial.print("X: ");
    Serial.print(compass.getX());
    Serial.print(" Y: ");
    Serial.print(compass.getY());
    Serial.print(" Z: ");
    Serial.println(compass.getZ());
  }*/

  /*if(accelWorking) {
    sensors_event_t accel_event;
    lsm_accel->getEvent(&accel_event);
    Serial.println();
    Serial.println("Accelerometer information follows:");
    Serial.print("X: ");
    Serial.print(accel_event.acceleration.x);
    Serial.print(" Y: ");
    Serial.print(accel_event.acceleration.y);
    Serial.print(" Z: ");
    Serial.println(accel_event.acceleration.z);
  }*/
}


