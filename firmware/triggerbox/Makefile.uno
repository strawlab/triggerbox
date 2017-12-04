# see http://www.martyndavis.com/?p=335 for required hack on Ubuntu Precise
# see http://mjo.tc/atelier/2009/02/arduino-cli.html
# see https://github.com/sudar/Arduino-Makefile/

ARDUINO_DIR = /usr/share/arduino
ARDUINO_PORT = /dev/trig* /dev/ttyACM*
BOARD_TAG = uno
ARDUINO_LIBS = SPI EEPROM UDEV
include /usr/share/arduino/Arduino.mk
