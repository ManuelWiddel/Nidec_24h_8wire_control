# Nidec 24h 8-Wire Motor Control

This is the repository for the corresponding blog post https://arobotaweek.wordpress.com/2026/11/24/my-motors-have-arrived-say-hello-to-the-nidec-24h-8-wire-variant/.

## Features

- **Motor PWM control** using STM32 timer PWM output.
- **Quadrature encoder feedback** for measuring motor position and velocity.
- **PID speed control** with configurable gains and output limits.
- **Velocity observer** that estimates velocity from measured position.
- **Timer-driven task scheduling**:
  - 100 Hz control task for the observer, PID controller, and motor output.
  - 10 Hz telemetry task for sending values to the visualizer.
- **UART telemetry** for monitoring the speed setpoint, measured motor velocity, and estimated velocity.

## Control overview

The main control loop is scheduled by TIM5 output-compare interrupts. The interrupt callbacks set scheduler flags; the main loop consumes those flags and runs the corresponding tasks.

At 100 Hz, the application:
1. Reads the encoder count and converts it to a position value using the configured scale factor.
2. Updates the velocity observer.
3. Uses the observer's velocity estimate as the PID feedback value.
4. Applies the PID output to the motor PWM.

At 10 Hz, the application sends three scaled signals through the visualizer interface:

| Signal | Description |
|---|---|
| `setpoint` | Motor speed setpoint |
| `motor velocity` | Encoder velocity measurement |
| `obs. velocity` | Observer velocity estimate |

The current speed setpoint is initialized to `10.0f` and its sign is reversed every 300 executions of the 100 Hz task.

## Main peripherals

| Peripheral | Purpose in the application |
|---|---|
| TIM1 | Motor PWM generation |
| TIM2 | Encoder interface |
| TIM3 | Encoder-related base timer |
| TIM4 | Periodic encoder update and velocity estimate update |
| TIM5 | 100 Hz and 10 Hz task scheduling |
| USART1 | Visualizer telemetry at 115200 baud |
| GPIO | Motor direction control |

## Project modules

The application includes the following project-specific modules:

- `encoder.h` / encoder module — encoder initialization, capture handling, and measurement access.
- `motor.h` / motor module — motor initialization and PWM command.
- `pid.h` / PID module — PID controller configuration and update.
- `visualizer.h` / visualizer module — UART-based signal transmission.

The project also uses the STM32 Hardware Abstraction Layer (HAL) and the board configuration provided by `main.h`.

## Configuration

The principal control parameters are defined in `main.c`:

### Velocity PID

| Parameter | Current value |
|---|---:|
| `Kp` | `30.0f` |
| `Ki` | `300.0f` |
| `Kd` | `0.0f` |
| Sample time | `0.01f` s |
| Output limits | `-2000.0f` to `2000.0f` |

### Position PID

A position PID configuration is present in the source (`Kp = 0.01f`, `Ki = 0.0f`, `Kd = 0.0f`, sample time `0.1f` s). The current main control loop does not use it for motor control.

### Velocity observer

| Parameter | Current value |
|---|---:|
| `alpha` | `0.9f` |
| `beta` | `0.05f` |
| Update interval | `0.01f` s |

These values are source-code defaults and may need adjustment for a different motor, encoder, driver, or mechanical setup.

## Build and flash

This repository's application source is written for an STM32 project using the STM32 HAL. Open the project with the STM32-compatible build environment used to create its project files (commonly STM32CubeIDE), select the matching target configuration, build, and flash the resulting firmware.

The complete project must include the relevant STM32 startup/system files, HAL drivers, generated peripheral configuration, linker configuration, and the project-specific encoder, motor, PID, and visualizer modules. Those files are not described or reproduced by this README.

## Hardware and safety notes

- Connect the motor driver, encoder, and MCU pins according to the board configuration and wiring used by your project.
- Verify PWM polarity, motor direction, encoder scaling, timer clocks, and current limits before running the motor.
- Start with conservative controller gains and a mechanically safe test setup.
- Ensure the motor driver has appropriate current limiting and an accessible emergency stop.
- Do not assume the example parameter values are suitable for other hardware.

## License

The source file contains the STM32-generated STMicroelectronics license notice and refers to a `LICENSE` file in the project root. Check the repository's `LICENSE` file for the terms that apply to the complete project.
