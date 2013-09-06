To build this firmware for an Arduino Uno board on Ubuntu linux, do
the following steps.

Install the required software::

    sudo apt-get install arduino-mk

Make the firmware (`build-cli/triggerbox.hex`)::

    make

Upload the firmware onto your Arduino device::

    make upload
