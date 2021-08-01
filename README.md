# triggerbox

camera triggering with clock synchronization to host PCs

Required hardware:

Arduino Nano A000005 (later models WILL NOT WORK).

## firmware build and install

This firmware can be compiled and installed with the Arduino IDE or at
the command line.

### Arduino IDE compilation

The file `triggerbox.ino` in the folder `firmware/triggerbox` can be opened
directly in the [Arduino IDE](http://arduino.cc/en/main/software).

### Command-line compilation

To build this firmware for an Arduino Nano on Ubuntu linux, do the following
steps.

Install the required software:

    curl -O http://debs.strawlab.org/xenial/arduino-udev_0.2.3-0ads1_all.deb
    sudo apt install ./arduino-udev_0.2.3-0ads1_all.deb
    sudo apt-get install arduino-mk

Make the firmware and upload it onto your Arduino device. With Arduino Nano:

    # Do this if you are using an Arduino Nano
    cd firmware/triggerbox
    make upload

## Device setup

After building and uploading firmware to the device, its name must be saved
to it with the [arduino-udev](https://github.com/strawlab/arduino-udev)
package.

Unplug and replug the triggerbox, then run this to flash the EEPROM on the
device with its new name (which will be `trig1`):

    arduino-udev-name --set-name trig1 --verbose /dev/ttyUSB0

Now, run the udev rules to make a symlink to the device at `/dev/trig1`:

    udevadm control --reload-rules
    udevadm trigger --attr-match=subsystem=tty
    ls /dev/trig1

## Troubleshooting

If your device is at `/dev/ttyACM0`, the `modemmanager` package may interfere
with its normal operation. Therefore, ensure you do not have modemmanager
installed:

    apt-get remove modemmanager

If the commands above in device setup suceeded but there is no device at
`/dev/trig1`, check the output of:

    arduino-udev-name /dev/ttyUSB0

If the output is `trig1`, this means the `--set-name` command above succeeded.
However, since there is no device at `/dev/trig1`, this means the udev rules
from the `arduino-udev` package are not working as expected. In this case, check
the output of `lsusb` for the VendorID and ProductID of your trigger device and
ensure they are in the udev rules file from the `arduino-udev` package. (The
rules file is typically installed at `/lib/udev/rules.d/99-arduino-udev.rules`.)
