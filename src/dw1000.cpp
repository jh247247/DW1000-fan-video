

#include "dw1000.hpp"

#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgTime.hpp>
#include <DW1000NgConstants.hpp>
#include <DW1000NgRanging.hpp>
#include <DW1000NgRTLS.hpp>
#include <Preferences.h>

#include "network.hpp"

// blink specifier that this blink is from an anchor, not a tag
#define DEVICE_IS_ANCHOR 0x03

// packet specifier that this is a range report to a specific tag
#define RANGE_REPORT 0xA1

DW1000::DW1000(Preferences *preferences, const uint8_t ss, const uint8_t irq, const uint8_t rst, const uint8_t *macAddr)
{

    DW1000Ng::initialize(ss, irq, rst);

    char msg[128];

    // DEBUG chip info and registers pretty printed
    DW1000Ng::getPrintableDeviceIdentifier(msg);
    Serial.print("Device ID: ");
    Serial.println(msg);
    DW1000Ng::getPrintableExtendedUniqueIdentifier(msg);
    Serial.print("Unique ID: ");
    Serial.println(msg);
    DW1000Ng::getPrintableNetworkIdAndShortAddress(msg);
    Serial.print("Network ID & Device Address: ");
    Serial.println(msg);
    DW1000Ng::getPrintableDeviceMode(msg);
    Serial.print("Device mode: ");
    Serial.println(msg);

    device_configuration_t DW1000_TXMODE = {
        false,
        true,
        true,
        true,
        false,
        SFDMode::STANDARD_SFD,
        Channel::CHANNEL_4,
        DataRate::RATE_6800KBPS,
        PulseFrequency::FREQ_64MHZ,
        PreambleLength::LEN_128,
        PreambleCode::CODE_3};

    frame_filtering_configuration_t TAG_FRAME_FILTER_CONFIG = {
        false,
        false,
        true,
        false,
        false,
        false,
        false,
        false};

    DW1000Ng::applyConfiguration(DW1000_TXMODE);
    // DW1000Ng::enableFrameFiltering(TAG_FRAME_FILTER_CONFIG);

    char EUI[24];
    // use mac address as device address
    sprintf(EUI, "BE:EF:%02X:%02X:%02X:%02X:%02X:%02X", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
    Serial.print("EUI: ");
    Serial.println(EUI);
    DW1000Ng::setEUI(EUI);

    DW1000Ng::setDeviceAddress(macAddr[1] << 8 | macAddr[0]);
    DW1000Ng::setNetworkId(10);
    // DW1000Ng::setTXPower(DriverAmplifierValue::db_);
    DW1000Ng::setReceiveFrameWaitTimeoutPeriod(15000); // 15ms - ranging usually takes 10ms

#ifdef DW1000_ANCHOR
    this->mNextBlinkScheduled = millis() + random(mMinBlinkDelay, mMaxBlinkDelay);
#endif

    DW1000Ng::setAntennaDelay(preferences->getInt("antennaDelay", 16436));

    DW1000Ng::enableDebounceClock();
    DW1000Ng::enableLedBlinking();

    DW1000Ng::setGPIOMode(10, LED_MODE);
    DW1000Ng::setGPIOMode(12, LED_MODE);

    Serial.println(F("Committed configuration ..."));

    // DEBUG chip info and registers pretty printed
    DW1000Ng::getPrintableDeviceIdentifier(msg);
    Serial.print("Device ID: ");
    Serial.println(msg);
    DW1000Ng::getPrintableExtendedUniqueIdentifier(msg);
    Serial.print("Unique ID: ");
    Serial.println(msg);
    DW1000Ng::getPrintableNetworkIdAndShortAddress(msg);
    Serial.print("Network ID & Device Address: ");
    Serial.println(msg);
    DW1000Ng::getPrintableDeviceMode(msg);
    Serial.print("Device mode: ");
    Serial.println(msg);
}

// add a custom blink message - this one advertises that the current device is an anchor
void DW1000::transmitAnchorAdvertiseBlink()
{
    byte Blink[] = {BLINK, DW1000NgRTLS::increaseSequenceNumber(), 0, 0, 0, 0, 0, 0, 0, 0, NO_BATTERY_STATUS | NO_EX_ID, DEVICE_IS_ANCHOR};
    DW1000Ng::getEUI(&Blink[2]);
    DW1000Ng::setTransmitData(Blink, sizeof(Blink));
    DW1000Ng::startTransmit();
}

// add a custom blink message, this one is targeted at a specific anchor via short address
void DW1000::transmitTagTargetedBlink(u16_t anchor_short_address)
{
    byte Blink[] = {BLINK, DW1000NgRTLS::increaseSequenceNumber(), 0, 0, 0, 0, 0, 0, 0, 0, NO_BATTERY_STATUS | NO_EX_ID, TAG_LISTENING_NOW, 0, 0};
    // copy short address to blink message
    memcpy(&Blink[12], &anchor_short_address, 2);
    DW1000Ng::getEUI(&Blink[2]);
    DW1000Ng::setTransmitData(Blink, sizeof(Blink));
    DW1000Ng::startTransmit();
}

RangeRequestResult DW1000::tagTargetedRangeRequest(u16_t anchor_short_address)
{
    this->transmitTagTargetedBlink(anchor_short_address);

    DW1000NgRTLS::waitForTransmission();
    if (!DW1000NgRTLS::receiveFrame())
        return {false, 0};

    size_t init_len = DW1000Ng::getReceivedDataLength();
    byte init_recv[init_len];
    DW1000Ng::getReceivedData(init_recv, init_len);

    if (!(init_len > 17 && init_recv[15] == RANGING_INITIATION))
    {
        return {false, 0};
    }

    DW1000Ng::setDeviceAddress(DW1000NgUtils::bytesAsValue(&init_recv[16], 2));
    return {true, DW1000NgUtils::bytesAsValue(&init_recv[13], 2)};
}

// transmit range report to tag with next anchor to
void DW1000::transmitRangeReportToTag(uint16_t range, byte tag_eui[])
{
    byte data[] = {
        DATA, RANGE_REPORT,
        0, 0, 0, 0, 0, 0, 0, 0, // for eui
        0, 0};                  // for range in cm
    DW1000Ng::getEUI(&data[2]);
    DW1000NgUtils::writeValueToBytes(&data[10], range, 2);
    DW1000Ng::setTransmitData(data, sizeof(data));
    DW1000Ng::startTransmit();
}

#ifdef DW1000_ANCHOR

/**
 * Anchor mode handle function
 * This sends out blink messages according to ALOHA protocol - i.e: randomly
 * When a blink message is responded to by a tag, the anchor will respond with a ranging request message
 */
void DW1000::handle()
{
    // is scheduled?
    if (millis() > mNextBlinkScheduled)
    {
        // transmit blink message
        this->transmitAnchorAdvertiseBlink();

        mNextBlinkScheduled = millis() + random(mMinBlinkDelay, mMaxBlinkDelay);
        DW1000Ng::startReceive();
    }

    if (DW1000NgRTLS::receiveFrame())
    {
        size_t len = DW1000Ng::getReceivedDataLength();
        byte data[len];
        DW1000Ng::getReceivedData(data, len);
        if (data[0] == BLINK && len > 11 && data[11] == TAG_LISTENING_NOW && len == 14)
        {

            byte anchor_eui[8];
            DW1000Ng::getEUI(anchor_eui);

            // check if last 2 bytes are the last 2 bytes of our eui, if they aren't, we're not the target so we should exit
            if (memcmp(&data[12], &anchor_eui[0], 2) != 0)
            {
                return;
            }

            byte tag_eui[8];
            memcpy(tag_eui, &data[2], 8);
            DW1000NgRTLS::transmitRangingInitiation(&data[2], &data[2]); // todo figure out address? eui?
            DW1000NgRTLS::waitForTransmission();
            RangeAcceptResult result = DW1000NgRTLS::anchorRangeAccept(NextActivity::ACTIVITY_FINISHED, 0);
            debugV("Received blink message from tag and performed ranging");

            if (result.success)
            {
                debugV("Range accept success");
                debugV("RX power: %f dBm", DW1000Ng::getReceivePower());
                // convert range to 10cm resolution float
                // try and find the tag in the list of known tags
                bool found = false;
                for (uint8_t i = 0; i < mTagDistancesCount; i++)
                {
                    if (memcmp(mTagDistances[i].eui, tag_eui, 8) == 0)
                    {
                        found = true;
                        mTagDistances[i].distance = result.range;
                        break;
                    }
                }
                // todo limit to 8 tags so we don't segfault
                if (found == false)
                {
                    // add tag to list
                    memcpy(mTagDistances[mTagDistancesCount].eui, tag_eui, 8);
                    mTagDistances[mTagDistancesCount].distance = result.range;
                    mTagDistancesCount++;
                }
            }
            else
            {
                debugE("Range accept failed");
                return;
            }
        }
        else
        {
            /*rdebugV("Received unknown message");
            // print out unknown message over debugV
            for (size_t i = 0; i < len; i++)
            {
                Debug.printf("%02X ", data[i]);
            }
            Debug.printf("\n");*/
        }
    }
}

#elif defined(DW1000_TAG)
/**
 * Tag mode handle function
 * This listens for blink messages and responds with a ranging request message
 */
void DW1000::handle()
{
    // is scheduled?
    if (millis() > mNextBlinkScheduled)
    {
        debugV("Known anchors: %d", mAnchorsCount);
        // list known anchors addresses
        for (uint8_t i = 0; i < mAnchorsCount; i++)
        {
            debugV("Anchor %d: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X", i, mAnchors[i].eui[0], mAnchors[i].eui[1], mAnchors[i].eui[2], mAnchors[i].eui[3], mAnchors[i].eui[4], mAnchors[i].eui[5], mAnchors[i].eui[6], mAnchors[i].eui[7]);
            RangeRequestResult requestResult = this->tagTargetedRangeRequest(DW1000NgUtils::bytesAsValue(mAnchors[i].eui, 2));
            if (requestResult.success)
            {
                RangeInfrastructureResult result = DW1000NgRTLS::tagRangeInfrastructure(requestResult.target_anchor, 3000);
                if (result.success)
                {
                    // increase reliability of anchor to max of 100
                    mAnchors[i].reliability = min((mAnchors[i].reliability + 100) / 2, 100);
                    debugV("Tag range infrastructure success, reliability: %d", mAnchors[i].reliability);
                }
                else
                {
                    // decrease reliability of anchor to min of 0
                    mAnchors[i].reliability = max(mAnchors[i].reliability / 2, 0);
                    debugE("Tag range infrastructure failed, reliability: %d", mAnchors[i].reliability);
                }
            }
            else
            {
                debugE("Tag range request failed");
            }
        }

        mNextBlinkScheduled = millis() + random(mMinBlinkDelay, mMaxBlinkDelay);
        debugV("Next ranging session scheduled in %d ms", mNextBlinkScheduled - millis());
    }

    if (DW1000NgRTLS::receiveFrame())
    {
        size_t len = DW1000Ng::getReceivedDataLength();
        byte data[len];
        DW1000Ng::getReceivedData(data, len);
        if (data[0] == BLINK && data[11] == DEVICE_IS_ANCHOR)
        {
            rdebugV("Received anchor blink message");
            // print out blink message over debugV
            for (size_t i = 0; i < len; i++)
            {
                Debug.printf("%02X ", data[i]);
            }
            Debug.printf("\n");

            byte anchor_address[8];
            // extract eui from blink message
            memcpy(anchor_address, &data[2], 8);
            // check if we've seen this anchor before
            bool found = false;
            for (uint8_t i = 0; i < mAnchorsCount; i++)
            {
                if (memcmp(mAnchors[i].eui, anchor_address, 8) == 0)
                {
                    found = true;
                    break;
                }
            }
            if (!found)
            {
                Anchor anchor;
                // add anchor to list
                memcpy(anchor.eui, anchor_address, 8);
                anchor.reliability = 100;
                mAnchors[mAnchorsCount] = anchor;
                mAnchorsCount++;
            }
        }
        else
        {
            /*rdebugV("Received unknown message");
            // print out unknown message over debugV
            for (size_t i = 0; i < len; i++)
            {
                Debug.printf("%02X ", data[i]);
            }
            Debug.printf("\n");*/
        }
    }
}

#else
#warning "Not configured as anchor or tag"
void DW1000::handle()
{
    Serial.println("Not configured as anchor or tag");
    debugE("Not configured as anchor or tag");
    delay(1000);
}
#endif
