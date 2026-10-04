# braid-triggerbox

Host-side driver for the camera synchronization triggerbox used by
[Braid](https://strawlab.org/braid/).

The triggerbox generates the hardware trigger pulses for a multi-camera setup.
This crate talks to it over a USB serial port: it sets the trigger rate, starts
and stops the pulses, and continuously measures the triggerbox clock to build a
model relating trigger pulse numbers to host time. See
[synchronization.md](https://github.com/strawlab/triggerbox/blob/main/synchronization.md)
for the theory of operation.

Two kinds of triggerbox are supported, both speaking the same protocol:

- an Arduino Nano running
  [`triggerbox.ino`](https://github.com/strawlab/triggerbox/blob/main/triggerbox.ino),
  and
- a Raspberry Pi Pico running the
  [Pico firmware](https://github.com/strawlab/triggerbox/tree/main/hardware_v3/braid-triggerbox-firmware-pico).

## Example

`examples/standalone-triggerbox-demo.rs` drives a triggerbox and prints the
clock model as it is updated:

```sh
cargo run --example standalone-triggerbox-demo -- --device /dev/ttyACM0 --fps 100
```

Run it with `--help` for all options. A Pico is ready immediately, so
`--sleep 0.5` shortens the default 7 second wait for an Arduino Nano to reset.

## License

Licensed under either of the Apache License, Version 2.0 or the MIT license, at
your option.
