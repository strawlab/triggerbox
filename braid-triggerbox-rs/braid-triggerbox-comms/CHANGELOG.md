# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.2.0](https://github.com/strawlab/triggerbox/compare/braid-triggerbox-comms/0.1.0...braid-triggerbox-comms/0.2.0) - 2026-10-04

### Changed
- *(braid-triggerbox-comms)* [**breaking**] replace bbqueue with a plain buffer

### Dependencies
- *(deps)* [**breaking**] update braid-triggerbox-comms to fugit 0.6 and defmt 1

### Fixed
- [**breaking**] never panic in the firmware on data from the host
- *(braid-triggerbox-comms)* parse N= (set device name)

### Other
- clippy fix
- clippy fixes
- remove non-backwards compatible firmware
- update rp-pico 0.2.0 -> 0.6.0
- add test encode/decode tests for all message types
- bugfix: many interrupted messages
- sleep duration is part of API
- cargo clippy fix
- move clock emulation code
- flexible clock freq
- better naming
- cleanup
- cleanup
- cleanup
- add docstrings
