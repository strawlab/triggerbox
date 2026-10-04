# braid-triggerbox-comms

The protocol spoken between the host and the
[Braid](https://strawlab.org/braid/) camera synchronization triggerbox
firmware.

This crate is shared by the host library,
[`braid-triggerbox`](https://crates.io/crates/braid-triggerbox), and the
[Raspberry Pi Pico firmware](https://github.com/strawlab/triggerbox/tree/main/hardware_v3/braid-triggerbox-firmware-pico).
It is `no_std` when the default `std` feature is disabled. Enable the `defmt`
feature for [`defmt`](https://defmt.ferrous-systems.com/) formatting on the
device.

## License

Licensed under either of the Apache License, Version 2.0 or the MIT license, at
your option.
