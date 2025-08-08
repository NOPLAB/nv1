# NV1 Project Overview

## Project Purpose
NV1 is an embedded robotics project built in Rust that consists of multiple components:

- **nv1-stm32-md**: Motor driver firmware for STM32F446RE microcontroller - controls 4 motors with encoders using PID control
- **nv1-stm32-hub**: Hub firmware (workspace with UI, UI tester, and core hub modules) 
- **nv1-msg**: Shared message protocol library for communication between components
- **nv1-ros**: ROS2 integration modules (communicator, ONNX inference, OpenCV processing)

## Architecture
The system follows a distributed architecture:
- STM32 microcontrollers handle real-time motor control and sensor interfacing
- ROS2 nodes handle higher-level processing and communication
- Shared message protocol ensures consistent communication

## Target Hardware
- STM32F446RE microcontroller (ARM Cortex-M4)
- Uses probe-rs for flashing and debugging
- 4-motor omni-directional drive system with encoders
- Various sensors and communication interfaces