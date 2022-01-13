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
`hardware_v1` and `hardware_v2` directories are the PCB layouts, schematics, and enclosure of the triggerbox.
- Hardware_v1: BNC outputs for triggering only
- Hardware_v2: 4-pin terminal block ouputs for triggering and powering the cameras via external powersource.

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

## Testing with host PC

Run the standalone demo program on your host PC. 

 * Prerequisite 1: install [rust](https://rustup.rs/).
 * Prerequisite 2 (linux only): make sure you are in the `dialout` group (or
   whichever group owns the device file - `/dev/ttyUSB0` in the example below):
   ```sudo adduser `whoami` dialout```. 

The demo can be run like this:

    cd braid-triggerbox-rs
    # You may need to change the device path from `/dev/ttyUSB0` in this example.
    # On Windows, this will be something like `COM4` instead of `/dev/ttyUSB0`.
    cargo run -- --device /dev/ttyUSB0 --fps 100
    
If successful, the output will look like:

```
Requested 100 fps, using 100 fps
Connecting to trigger device ..
got new time model: None
.. connected.
got new time model: Some(ClockModel { gain: 0.009991053259000182, offset: 1642089273.653188, residuals: 0.00008373855939680652, n_measurements: 5 })
got new time model: Some(ClockModel { gain: 0.009990740683861077, offset: 1642089273.65347, residuals: 0.00008408054048913982, n_measurements: 6 })
got new time model: Some(ClockModel { gain: 0.009990941849537194, offset: 1642089273.6532729, residuals: 0.00008470585419217969, n_measurements: 7 })
got new time model: Some(ClockModel { gain: 0.0099910874851048, offset: 1642089273.6531227, residuals: 0.00008554253827242064, n_measurements: 8 })
got new time model: Some(ClockModel { gain: 0.009991202969104052, offset: 1642089273.652988, residuals: 0.0000863512614728279, n_measurements: 9 })
got new time model: Some(ClockModel { gain: 0.009991223458200693, offset: 1642089273.652963, residuals: 0.00008639545796995662, n_measurements: 10 })
got new time model: Some(ClockModel { gain: 0.009991292201448232, offset: 1642089273.6528616, residuals: 0.00008692377247143668, n_measurements: 11 })
got new time model: Some(ClockModel { gain: 0.009991496510338038, offset: 1642089273.6525416, residuals: 0.00009343214287582668, n_measurements: 12 })
```

## License

Apache 2.0 or MIT at your choice. See `LICENSE-APACHE` and `LICENSE-MIT`.
