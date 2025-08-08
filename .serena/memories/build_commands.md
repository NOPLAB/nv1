# Build and Development Commands

## Building Components

### STM32 Motor Driver (nv1-stm32-md)
```bash
cd nv1-stm32-md
cargo build --release
cargo run --release  # Flashes and runs on target
```

### STM32 Hub (nv1-stm32-hub)
```bash
cd nv1-stm32-hub
cargo build --release  # Builds workspace
cargo run -p nv1-hub --release  # Runs main hub firmware
cargo run -p nv1-hub-ui --release  # Runs UI component
cargo run -p nv1-hub-ui-tester --release  # Runs UI tester
```

### Message Protocol (nv1-msg)
```bash
cd nv1-msg
cargo build
cargo test
```

### ROS Components (nv1-ros)
```bash
cd nv1-ros
cargo build
cargo run -p nv1-ros-communicator
cargo run -p nv1-ros-onnx
cargo run -p nv1-ros-opencv
```

## Hardware Interaction
- **Flashing**: Uses probe-rs with STM32F446RETx chip configuration
- **Debugging**: DEFMT_LOG=trace environment variable controls log level
- **Reset**: --connect-under-reset flag used for reliable connection