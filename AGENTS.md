# Repository Guidelines

## Project Structure & Module Organization

This project implements a 5 ms DC motor speed loop on an STM32F103C8T6 with a TB6612 driver and encoder.

- `App/`: PID math, command parsing, UART transport, speed control, VOFA telemetry, OLED UI, and fault reporting.
- `Components/`: TB6612 motor/encoder and SSD1306 OLED drivers.
- `Core/`: CubeMX-generated initialization, interrupt handlers, and application entry point.
- `Drivers/`: STM32 HAL and CMSIS dependencies.
- `tests/`: hardware-independent C tests for PID, commands, and telemetry.
- `Pid.ioc`, `cmake/`, and the root `CMakeLists.txt`: hardware configuration and firmware build setup.
- `docx/`: wiring images, datasheets, and reference documents. See `README.md` for operation and pin assignments.

## Build, Test, and Development Commands

Run commands from this directory. Firmware builds require CMake 3.22+, Ninja, and the Arm GNU toolchain on `PATH`.

```sh
cmake --preset Debug
cmake --build --preset Debug
```

These configure and build firmware under `build/Debug/`. Substitute `Release` for an optimized build. Running firmware requires flashing the board; the local helper documented in the README is `bash ~/stm32/scripts/build-flash.sh "$PWD"` and resides outside this repository.

Compile and run host tests with GCC:

```sh
for module in pid cmd vofa; do
  gcc -std=c11 -Wall -Wextra -I App/$module \
    tests/test_$module.c App/$module/$module.c -lm \
    -o /tmp/test_$module && /tmp/test_$module || break
done
```

## Coding Style & Naming Conventions

Use C11, two-space indentation, and opening braces on separate lines, matching nearby code. Follow module-prefixed APIs such as `PID_Update` and `SpeedLoop_Tick`, uppercase configuration macros, and existing filename conventions. No repository-wide formatter or linter is configured.

Register new application sources and include paths in the root `CMakeLists.txt`. Keep custom edits in generated files inside `USER CODE` blocks; CubeMX can overwrite other edits and `cmake/stm32cubemx/`.

## Testing Guidelines

Tests use standalone C assertions/check helpers without an external framework. Name files `tests/test_<module>.c` and require all checks to pass. No coverage threshold is configured. Validate timing, encoder direction, faults, and motor response on hardware when affected; keep blocking UART/OLED work outside the control interrupt.

## Commit & Pull Request Guidelines

Recent commits use concise Chinese descriptions, often prefixed with the affected area, such as `README：` or `固件：`. Follow that style and keep commits focused. PRs should explain behavior changes, list build/test results, link relevant issues, and include wiring changes or measured response data when applicable.
