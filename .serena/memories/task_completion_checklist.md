# Task Completion Checklist

## For Embedded Components (STM32)
1. **Build Check**: Run `cargo check` to verify compilation
2. **Release Build**: Run `cargo build --release` to ensure optimized build works
3. **Flash Test**: If possible, test flash to target hardware with `cargo run --release`
4. **Code Review**: Ensure proper error handling for hardware operations
5. **Memory Safety**: Verify no unsafe code issues, proper heap usage

## For ROS Components  
1. **Build Check**: Run `cargo check` and `cargo build`
2. **Dependencies**: Verify all ROS2 and external dependencies are available
3. **Integration**: Check message protocol compatibility with nv1-msg

## General Practices
1. **Format**: Run `cargo fmt` to ensure consistent formatting
2. **Clippy**: Run `cargo clippy` to catch common issues
3. **Test**: Run `cargo test` if tests exist
4. **Documentation**: Update inline documentation if adding new public APIs

## Hardware-Specific Considerations
- Verify clock configurations match hardware setup
- Check pin assignments against actual PCB layout  
- Ensure PWM frequencies and timer configurations are appropriate
- Validate encoder and sensor interfacing