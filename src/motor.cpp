#include "motor.hpp"
#include <Arduino.h>
#include "network.hpp"

Motor::Motor(uint8_t stepPin, uint8_t dirPin, uint32_t stepsPerRev) {
    mStepPin = stepPin;
    mDirPin = dirPin;
    mStepsPerRev = stepsPerRev;
    mCurrentPosition = 0;
    mStepsRemaining = 0;
    mInitialized = false;
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
}

void Motor::motorInterrupt() {
    boolean currentStepPin = digitalRead(mStepPin);
    boolean currentDirPin = digitalRead(mDirPin);
    if(mStepsRemaining != 0) {
        pinMode(mStepPin, OUTPUT);
        digitalWrite(mStepPin, !currentStepPin);
        if(currentStepPin) {
            // we've completed a step
            int inc = currentDirPin ? -1 : 1;
            if(mCurrentPosition == 0 && inc == -1) {
                mCurrentPosition = mStepsPerRev;
            } else if(mCurrentPosition == mStepsPerRev && inc == 1) {
                mCurrentPosition = 0;
            }
            mCurrentPosition += inc;
            
            mStepsRemaining += currentDirPin ? 1 : -1;
        }
    } else {
        // set to input to prevent glitching
        pinMode(mStepPin, INPUT);
    }
    
}

void Motor::update(int angle) {
    if(mInitialized == false) {
        mInitialized = true;

        // first angle set, use this as the current position
        mCurrentPosition = angle * mStepsPerRev / 360;
        mStepsRemaining = 0;
        return;
    }    

    int targetPosition = angle * mStepsPerRev / 360;
    int distance = targetPosition - (mCurrentPosition + mStepsRemaining);
    int clockwiseDistance = (distance + mStepsPerRev) % mStepsPerRev;
    int counterClockwiseDistance = abs((int)((mStepsPerRev - clockwiseDistance) % mStepsPerRev));

    debugV("motor Clockwise distance: %d, Counter clockwise distance: %d", clockwiseDistance, counterClockwiseDistance);

    if(clockwiseDistance < counterClockwiseDistance) {
        mStepsRemaining += clockwiseDistance;
    } else {
        mStepsRemaining -= counterClockwiseDistance;
    }

    if(mStepsRemaining > 0) {
        digitalWrite(mDirPin, LOW);
    } else {
        digitalWrite(mDirPin, HIGH);
    }

    debugV("Updating motor to angle %d, current position %d, steps remaining: %d", angle, mCurrentPosition, mStepsRemaining);
}