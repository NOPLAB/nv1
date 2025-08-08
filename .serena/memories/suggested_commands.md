# Suggested Commands for NV1 Development

## Essential Build Commands
```bash
# Build and flash STM32 motor driver
cd nv1-stm32-md && cargo run --release

# Build and flash STM32 hub
cd nv1-stm32-hub && cargo run -p nv1-hub --release  

# Build message protocol library
cd nv1-msg && cargo build

# Build ROS components
cd nv1-ros && cargo build
```

## Code Quality Commands  
```bash
# Format all code
cargo fmt --all

# Check for common issues
cargo clippy --all

# Run tests (where available)
cargo test --all
```

## Development Commands
```bash
# Check compilation without building
cargo check --all

# Build in debug mode for development
cargo build

# Clean build artifacts
cargo clean
```

## Hardware Debug Commands
```bash
# Set debug log level
export DEFMT_LOG=trace

# Flash with debug info
cargo run --features debug

# Check probe-rs connection
probe-rs list
```

## System Commands (Linux/WSL2)
```bash
# Standard Linux commands available
ls, cd, grep, find, git
```