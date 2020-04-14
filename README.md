# mBotEnhanced
_Forking mBot factory firmware to extend its capabilities_

Makes use of:
* My fork (https://github.com/tiggi78/Makeblock-Libraries) of MakeBlock Libraries (https://github.com/Makeblock-official/Makeblock-Libraries)
* YetAnotherArduinoPcIntLibrary (https://github.com/paulo-raca/YetAnotherArduinoPcIntLibrary/)
* Arduino-Makefile (https://github.com/sudar/Arduino-Makefile)

Starting from Makeblock-Libraries/examples/Firmware_For_mBlock/mbot_factory_firmware.ino

- Code cleanup and
- Ultrasonic now uses PCINT to measure pulse and can be triggered witouth waiting the measure
