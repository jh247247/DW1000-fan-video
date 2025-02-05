# DW1000 fan pointing thingamagjig

This is the system shown off in [My video](https://youtu.be/dcd7I6KgY1Y) where I make an indoor location system to get a fan to point at me.

This uses the DW1000 UWB module to determine distances between radios in a 3d space, at minimum 4 fixed anchors.

This is very MVP software without power or feature optimizations, but it does work.

# Principle of operation

The DW1000 can determine distances between 2 radios using what is called Asymmetric Two Way Ranging. You can read about it here: https://www.qorvo.com/products/d/da008448

Using those distances, you can uniquely determine a point in 3d space that a radio exists in, given that space has at least 4 stationary radios spread out the area.

Since the measurements in this system are imperfect, the Levenberg-Marquandt algorithm is used to minimize the error in the measurements and find the best fitting point the tag could exist in.

Therefore, by finding the tag's position in 3d space you can have fun interations between them!

The application given in the linked video is trivial, but is an actual in-the-wild application of the technology outside of proof of concepts.
Future work that may or may not be added here:

- a CNC gantry to move a camera to a location given by another tag
- a gimbal for a spotlight to point at a subject
- "Hot zones" where emergency stops are enabled when a subject enters a known dangerous area
- ???

# Structure

Everything here can be considered MVP at best or broken at worst.

_hardware_
This is currently the broken version shown in the video, it has to be patched to get everything working. Hopefully I can get around to it.

- The power supply in power.sch is completely wrong - VCC is taken off the wrong side of the inductor
- MOSI is not attached to the DW1000
- RST is not attached to the DW1000 (optional - can have polling instead)
- IRQ is not attached to the DW1000 (optional - can have polling instead)
- The QMC5883L Compass is not working and doesn't respond to I2C commands - unknown as to why

# Installation

1. Have a home assistant instance up and running with an MQTT server and node-red
2. Add your secrets (WIFI SSID, Password, mqtt server, hopes and dreams) to secrets.cpp
3. Plug in your hardware into usb and run the esp32-s3 target (via platform.io) to run a skeleton sketch that prints out info
4. Note down the host name
5. Modify the platformio.ini file to add a target for your specific hardware. Add either `-DDW1000_ANCHOR` or `-DDW1000_TAG` to set that board as an anchor or tag. Examples from my hardware is provided.
6. Program your board ota using the new platform.io target you've added
7. Rinse and repeat for all your boards.
8. Import the `flows.json` file into your node red instance and duplicate the `Tag` subflow nodes. Add your mac address as it appears in the logs as the `TAG_MAC` env var for that subflow
9. Make sure you add the coordinates for the anchors, this is completely up to you. I recommend setting one as (0,0) with some height off the ground to reference off that one
10. The coordinates for your tags should start updating automatically and be available in Home Assistant for your automations
11. ???
12. Profit!
