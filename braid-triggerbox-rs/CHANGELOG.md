# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.5.0](https://github.com/strawlab/triggerbox/compare/braid-triggerbox/0.4.2...braid-triggerbox/0.5.0) - 2026-10-04

### Added
- *(braid-triggerbox)* [**breaking**] take TriggerboxOptions in TriggerboxDevice::new

### Changed
- *(braid-triggerbox-comms)* [**breaking**] replace bbqueue with a plain buffer

### Dependencies
- *(deps)* update braid-triggerbox dependencies
- *(deps)* [**breaking**] update braid-triggerbox-comms to fugit 0.6 and defmt 1

### Documentation
- update stale READMEs, comments and demo defaults

### Fixed
- [**breaking**] never panic in the firmware on data from the host
- *(braid-triggerbox)* handle device data without panicking
- *(braid-triggerbox-comms)* parse N= (set device name)
- *(braid-triggerbox)* always send two CRC digits when setting the name
- *(braid-triggerbox)* resynchronize on corrupted data from the device
