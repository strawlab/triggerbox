# triggerbox

camera triggering with clock synchronization to host PCs

Required hardware:

Arduino Nano A000005 (later models WILL NOT WORK).

## Firmware Build and Install

This firmware can be compiled and installed with the Arduino IDE or at
the command line.

### Option 1: Compiling and Uploading using the Arduino IDE

The file `triggerbox.ino` can be opened directly in the [Arduino
IDE](http://arduino.cc/en/main/software). Set your board to Arduino Nano
("Tools"->"Board"->"Arduino Nano"). Set your port ("Tools"->"Port") correctly,
which depends on your specific computer setup. Then click the "Upload" button.

### Option 2: Command-line Compiling and Uploading using Arduino CLI

To build this firmware for an Arduino Nano on Ubuntu linux, do the following
steps.

Install the required software:

Intall [Arduino CLI](https://arduino.github.io/arduino-cli/latest/), then:

    arduino-cli core update-index
    arduino-cli core install arduino:avr

Compile the firmware

    arduino-cli compile --fqbn arduino:avr:nano

Upload the firmware

    # Note the port `/dev/ttyUSB0` may be different on your computer. In Windows,
    # it will be something like `COM4`.
    arduino-cli upload --port /dev/ttyUSB0 --fqbn arduino:avr:nano
