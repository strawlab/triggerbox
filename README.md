# triggerbox - camera triggering for synchronized multi-camera setups

## Principle of operation

Cameras trigger inputs are connected to pin D9 on an Arduino. (Optionally, a
buffer amplifier can be used. See below for why this may be a good idea.) This
is used to generate a digital pulse which triggers image acquisition on cameras.
Furthermore, a host PC communicates with the Arduino to build a live model of
the clocks on both systems and allows very precise (sub-millisecond)
determination of the timing of trigger pulses from the Arduino. This allows
precise timing information to be taken from the resulting acquired image frames.

## Required hardware:

The minimal required hardware is this:

 - Arduino Nano A000005 (later models WILL NOT WORK).

If using the minimal hardware, you will need to connect pin D9 to the trigger
inputs of your camera directly.

## Optional buffer amplified and circuit board:

For additional robustness for multi-camera setups, particularly those that have
an opto-coupled trigger input which accordingly draws substantially more
current, you will want a buffer amplifier to trigger more cameras. In the
`circuit_board` directory is the schematic of such a circuit.

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
