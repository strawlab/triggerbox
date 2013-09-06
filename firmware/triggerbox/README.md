Triggerbox firmware
===================

This firmware can be compiled and installed with the Arduino IDE or at
the command line.

Arduino IDE compilation
-----------------------

The file `triggerbox.ino` in this folder can be opened directly in the
[Arduino IDE](http://arduino.cc/en/main/software).

Command-line compilation
------------------------

To build this firmware for an Arduino Uno board on Ubuntu linux, do
the following steps.

Install the required software:

    sudo apt-get install arduino-mk

Make the firmware and upload it onto your Arduino device:

    make BOARD_TAG=uno upload # <-- substitute "uno" for the name of your board
