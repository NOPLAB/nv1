# Technology Stack

## Languages & Frameworks
- **Rust**: Primary language for all components
- **Embassy**: Async runtime for embedded systems
- **ROS2**: Robot Operating System integration via r2r crate

## Embedded Dependencies
- **embassy-stm32**: HAL and runtime for STM32 microcontrollers
- **embassy-executor**: Async task executor
- **corncobs**: COBS encoding for serial communication
- **postcard**: Serialization format
- **pid**: PID controller implementation
- **defmt**: Logging framework for embedded systems
- **cortex-m**: ARM Cortex-M specific functionality

## Target & Toolchain
- **Target**: thumbv7em-none-eabi (ARM Cortex-M4)
- **Toolchain**: Nightly Rust with rust-src component
- **Runner**: probe-rs for flashing and debugging
- **Build Features**: Uses build-std for core and alloc