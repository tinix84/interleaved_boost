# 3-Phase Interleaved Boost Converter — C2000 Firmware

Embedded firmware for a **3-phase interleaved boost converter** running on the TI TMS320F28035 Piccolo Isolated controlCARD.

## Overview

This project implements a fully functional 3-phase interleaved boost modulator with:

- **3-phase interleaved PWM** with configurable phase delay and dead-time
- **CLA (Control Law Accelerator)** for real-time current control offloading
- **Serial communication** interface for parameter tuning and monitoring
- **Ring buffer** based command processing
- **TI DSP/BIOS HAL** (DSP2803x headers and drivers)

## Hardware

| Component | Details |
|:----------|:--------|
| **MCU** | [TMS320F28035 Isolated controlCARD](http://www.ti.com/tool/tmdscncd28035iso) (TI Piccolo, 60 MHz, CLA) |
| **Topology** | 3-phase interleaved boost (120 degree phase shift) |
| **Control** | Digital current-mode control via CLA |

## Project Structure

```
interleaved_boost/
├── main.c                 # Entry point — system init, ISR setup
├── application.c/h        # Application logic, state machines, serial protocol
├── device.c/h             # Peripheral configuration (PWM, ADC, GPIO)
├── I_Controller.cla/h     # CLA current controller implementation
├── CLAshared.h            # Shared variables between CPU and CLA
├── CLAtasks.cla           # CLA task definitions
├── DCL/                   # Digital Control Library (TI DCL)
├── defines.h              # System constants (PWM freq, dead-time, etc.)
├── globals.c              # Global variable definitions
├── ringbuffer.c/h         # Ring buffer for serial communication
├── commands.h             # Serial command definitions
├── error.h                # Error code definitions
├── hwrcommunication.py    # Python script for host-side serial communication
├── hal/                   # TI DSP2803x HAL (headers, drivers, linker scripts)
└── targetConfigs/         # CCS target configuration files
```

## Key Design Decisions

- **CLA for current loop:** The TMS320F28035's Control Law Accelerator runs the inner current control loop independently from the main CPU, enabling higher control bandwidth
- **Interleaved topology:** 3 phases at 120 degree offset reduce input/output ripple by a factor of 3 at the switching frequency, enabling smaller passive components
- **Serial interface:** Runtime parameter adjustment (duty cycle, period, phase delay, dead-time) via Python script

## Getting Started

### Prerequisites

- [Code Composer Studio (CCS)](https://www.ti.com/tool/CCSTUDIO) v5+
- TI C2000 compiler
- [TMDSCNCD28035ISO controlCARD](http://www.ti.com/tool/tmdscncd28035iso)

### Build and Flash

1. Import the project into Code Composer Studio
2. Set the target configuration to match your debug probe (XDS100v2 or similar)
3. Build the project (`Project > Build All`)
4. Connect the controlCARD and flash (`Run > Debug`)

### Host Communication

Use the Python script to send commands and monitor parameters:

```bash
python hwrcommunication.py
```

## Author

[Riccardo Tinivella](https://github.com/tinix84) — developed at NTB/OST Buchs

## License

See project files for license details.
