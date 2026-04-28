# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

NV1 is an embedded robotics system written in Rust with multiple components:

- **nv1-stm32**: STM32 firmware workspace (motor driver, hub, hub UI, UI tester, md core)
- **nv1-msg**: Shared message protocol library for inter-component communication
- **nv1-ros**: ROS2 integration modules (communicator, ONNX, OpenCV)

## Essential Commands

### Building and Flashing STM32 Components
```bash
# All STM32 firmware lives in the nv1-stm32 workspace (default target: thumbv7em-none-eabi)
cd nv1-stm32 && cargo run -p nv1-md --release
cd nv1-stm32 && cargo run -p nv1-hub --release

# Host-side UI tester (override the workspace default thumb target)
cd nv1-stm32 && cargo run -p nv1-hub-ui-tester --target x86_64-pc-windows-msvc

# md-core unit tests (host)
cd nv1-stm32 && cargo test -p nv1-md-core --target x86_64-pc-windows-msvc

# Message library
cd nv1-msg && cargo build && cargo test
```

### ROS Components
```bash
cd nv1-ros && cargo build
cargo run -p nv1-ros-communicator
cargo run -p nv1-ros-onnx  
cargo run -p nv1-ros-opencv
```

### Code Quality
```bash
cargo fmt --all        # Format code
cargo clippy --all     # Lint checking
cargo check --all      # Compilation check
```

## Architecture

### Hardware Target
- **MCU**: STM32F446RE (ARM Cortex-M4, thumbv7em-none-eabi)
- **Toolchain**: Nightly Rust with probe-rs for flashing
- **Features**: 4-motor omni drive, encoders, PID control

### Software Stack
- **Embassy**: Async embedded runtime
- **defmt**: Logging (set DEFMT_LOG=trace for debug)
- **postcard/COBS**: Serialization and framing
- **ROS2**: Higher-level processing via r2r

### Project Structure
```
nv1-stm32/             # STM32 firmware workspace
├── nv1-md/            # Motor driver (binary, thumb)
├── nv1-md-core/       # Motor driver pure logic (no_std lib, host-testable)
├── nv1-hub/           # Hub firmware (binary, thumb)
├── nv1-hub-ui/        # Hub UI (lib, no_std + std)
└── nv1-hub-ui-tester/ # UI tester (host-only, sdl2)
nv1-msg/               # Message protocol (shared with nv1-ros)
nv1-ros/               # ROS integration (workspace)
```

## Development Patterns

### Embedded Code Style
- Embassy async/await patterns
- Manual peripheral initialization via PAC
- Global statics prefixed with `G_` (e.g., `G_HUB_MSG`)
- Hardware constants in SCREAMING_SNAKE_CASE
- Modular structure (motor.rs, sensors.rs, communication.rs)

### Error Handling
- `.unwrap()` common for hardware initialization that must succeed
- Proper async error propagation where possible
- Hardware failures often result in system halt by design

### Task Completion
When modifying code, always run:
1. `cargo check` - verify compilation
2. `cargo clippy` - catch common issues  
3. `cargo fmt` - ensure consistent formatting
4. For STM32: test flash with `cargo run --release` if hardware available