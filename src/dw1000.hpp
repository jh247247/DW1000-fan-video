#pragma once

#include <Arduino.h>
#include <Preferences.h>

#include <DW1000NgRTLS.hpp>

class DW1000
{
public:
    typedef struct
    {
        byte eui[8];
        float distance;
    } TagDistance;

    typedef struct
    {
        byte eui[8];
        uint8_t reliability; // track reliability of ranging over time
    } Anchor;

    DW1000(Preferences *preferences, uint8_t ss, const uint8_t irq, const uint8_t rst, const uint8_t *macAddr);

    void handle();

#ifdef DW1000_ANCHOR
    uint8_t getKnownTagCount() { return mTagDistancesCount; }
    TagDistance *getKnownTag(uint8_t index) { return &mTagDistances[index]; }
#elif defined(DW1000_TAG)
    uint8_t getKnownAnchorsCount() { return mAnchorsCount; }
    Anchor *getKnownAnchor(uint8_t index) { return &mAnchors[index]; }
    float getDistanceToAnchor(byte anchor_eui[]);

#endif

private:
#ifdef DW1000_ANCHOR
    unsigned long mMinBlinkDelay = 5000;  // ms
    unsigned long mMaxBlinkDelay = 25000; // ms
    TagDistance mTagDistances[8];
    uint8_t mTagDistancesCount = 0;

#elif defined(DW1000_TAG)
    unsigned long mMinBlinkDelay = 100; // ms
    unsigned long mMaxBlinkDelay = 500; // ms
    // support at most 8 anchors for now
    Anchor mAnchors[8];
    uint8_t mAnchorsCount = 0;
#endif
    unsigned long mLastBlinkSent = 0;
    unsigned long mNextBlinkScheduled = 0;

    void transmitAnchorAdvertiseBlink();
    void transmitTagTargetedBlink(u16_t anchor_short_address);
    RangeRequestResult tagTargetedRangeRequest(u16_t anchor_short_address);
    void transmitRangeReportToTag(uint16_t range, byte tag_eui[]);
};
