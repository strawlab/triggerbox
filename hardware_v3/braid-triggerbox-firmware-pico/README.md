# Braid Triggerbox Firmware for Raspberry Pi Pico

Connections:

GP0 - camera trigger
USB - computer running Braid

## Instructions

Based on [Project template for rp2040-hal](https://github.com/rp-rs/rp2040-project-template).

It includes all of the `knurling-rs` tooling as showcased in https://github.com/knurling-rs/app-template (`defmt`, `defmt-rtt`, `panic-probe`, `flip-link`) to make development as easy as possible.

[`picotool`](https://github.com/raspberrypi/picotool) is configured as the
default runner. Boot the Pico into bootloader mode (hold BOOTSEL while plugging
it in) and then run
```sh
cargo run --release
```

This builds the firmware, loads it over USB, verifies it and reboots the Pico
into it. If you have a debug probe, check out [alternative
runners](#alternative-runners).

The Pico W works too. Note that its onboard LED is driven by the wireless chip,
not GPIO25, so the LED blink in this firmware is not visible on a Pico W.

<!-- TABLE OF CONTENTS -->
<details open="open">

  <summary><h2 style="display: inline-block">Table of Contents</h2></summary>
  <ol>
    <li><a href="#markdown-header-requirements">Requirements</a></li>
    <li><a href="#installation-of-development-dependencies">Installation of development dependencies</a></li>
    <li><a href="#running">Running</a></li>
    <li><a href="#alternative-runners">Alternative runners</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#code-of-conduct">Code of conduct</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>

<!-- Requirements -->
<details open="open">
  <summary><h2 style="display: inline-block" id="requirements">Requirements</h2></summary>

- The standard Rust tooling (cargo, rustup) which you can install from https://rustup.rs/

- Toolchain support for the cortex-m0+ processors in the rp2040 (thumbv6m-none-eabi)

- flip-link - this allows you to detect stack-overflows on the first core, which is the only supported target for now.

- picotool (the default runner), version 2.0 or later.

- Optionally, for debugging: probe-rs and a CMSIS-DAP probe. You can use a second Raspberry Pi Pico as a CMSIS-DAP probe debugger.

  - Download this file: https://github.com/majbthrd/DapperMime/releases/download/20210225/raspberry_pi_pico-DapperMime.uf2
  - Boot the Pico in bootloader mode by holding the bootset button while plugging it in
  - Open the drive RPI-RP2 when prompted
  - Copy raspberry_pi_pico-DapperMime.uf2 from Downloads into RPI-RP2
  - Connect the debug pins of your CMSIS-DAP Pico to the target one
      - Connect GP2 on the Probe to SWCLK on the Target
      - Connect GP3 on the Probe to SWDIO on the Target
      - Connect a ground line from the CMSIS-DAP Probe to the Target too

</details>

<!-- Installation of development dependencies -->
<details open="open">
  <summary><h2 style="display: inline-block" id="installation-of-development-dependencies">Installation of development dependencies</h2></summary>

```sh
rustup target install thumbv6m-none-eabi
cargo install flip-link
# The default 'runner'. On macOS:
brew install picotool
# On other platforms, see https://github.com/raspberrypi/picotool
```

</details>


<!-- Running -->
<details open="open">
  <summary><h2 style="display: inline-block" id="running">Running</h2></summary>

For a debug build
```sh
cargo run
```
For a release build
```sh
cargo run --release
```

`defmt` log output is only shown when using the debug probe runner (see
[alternative runners](#alternative-runners)).

If you do not specify a DEFMT_LOG level, it will be set to `debug`.
That means `println!("")`, `info!("")` and `debug!("")` statements will be printed.
If you wish to override this, you can change it in `.cargo/config.toml`
```toml
[env]
DEFMT_LOG = "off"
```
You can also set this inline (on Linux/MacOS)
```sh
DEFMT_LOG=trace cargo run
```

or set the _environment variable_ so that it applies to every `cargo run` call that follows:
#### Linux/MacOS/unix
```sh
export DEFMT_LOG=trace
```

Setting the DEFMT_LOG level for the current session
for bash
```sh
export DEFMT_LOG=trace
```

#### Windows
Windows users can only override DEFMT_LOG through `config.toml`
or by setting the environment variable as a separate step before calling `cargo run`
- cmd
```cmd
set DEFMT_LOG=trace
```
- powershell
```ps1
$Env:DEFMT_LOG = trace
```

```cmd
cargo run
```

</details>
<!-- ALTERNATIVE RUNNERS -->
<details open="open">
  <summary><h2 style="display: inline-block" id="alternative-runners">Alternative runners</h2></summary>

The runner is set in `.cargo/config.toml`. Some alternatives are listed below.

* **Loading with picotool (default)**

  ```toml
  runner = "picotool load -u -v -x -t elf"
  ```

  picotool talks to the RP2040 bootloader over its PICOBOOT USB interface. The
  Pico must be in bootloader mode (hold BOOTSEL while plugging it in).

* **Loading a UF2 over USB mass storage**

  ```console
  $ cargo install elf2uf2-rs --locked
  ```

  ```toml
  runner = "elf2uf2-rs -d"
  ```

  This builds a UF2 file and copies it to the `RPI-RP2` drive that appears in
  bootloader mode. On Linux, you need to mount the drive first. On recent macOS
  versions, writing to this drive can hang indefinitely, which is why picotool
  is the default.

* **Using a debug probe**

  ```console
  $ cargo install probe-rs-tools --locked
  ```

  ```toml
  runner = "probe-rs run --chip RP2040"
  ```

  This needs a CMSIS-DAP probe (see [requirements](#requirements)) but does not
  need the Pico to be in bootloader mode, and it shows `defmt` log output.

</details>

## License

The contents of this repository are dual-licensed under the _MIT OR Apache
2.0_ License. That means you can chose either the MIT licence or the
Apache-2.0 licence when you re-use this code. See `MIT` or `APACHE2.0` for more
information on each specific licence.

Any submissions to this project (e.g. as Pull Requests) must be made available
under these terms.
