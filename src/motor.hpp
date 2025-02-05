#pragma once
#include <Arduino.h>

class Motor {
    public:
    Motor(uint8_t stepPin, uint8_t dirPin, uint32_t stepsPerRev);
    void update(int angle);

    void motorInterrupt();

    private:
    boolean mInitialized;
    uint8_t mStepPin;
    uint8_t mDirPin;
    uint32_t mStepsPerRev;
    volatile uint32_t mCurrentPosition;
    volatile int32_t mStepsRemaining;


};