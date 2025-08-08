# Code Style and Conventions

## General Patterns
- Uses Embassy async/await patterns extensively
- Modular structure with separate files for different functionality (motor.rs, fmt.rs, sensors.rs, etc.)
- Constants defined in SCREAMING_SNAKE_CASE
- Global static variables prefixed with G_ (e.g., G_HUB_MSG)

## Embedded Specific Patterns
- Manual peripheral initialization using PAC (Peripheral Access Crate)
- Direct register manipulation for hardware setup
- Async tasks spawned with spawner.must_spawn()
- Heap initialization at start of main function
- Error handling with .unwrap() common for hardware initialization

## Project Structure
- Each component is a separate Rust package/workspace
- Shared types and messages in nv1-msg crate
- Hardware abstraction in separate modules (motor.rs, sensors.rs)
- Communication tasks isolated in separate async functions

## Naming Conventions
- Snake_case for variables and functions
- PascalCase for types and structs  
- Module names match file names
- Hardware-specific constants grouped by functionality