# STM32 FreeRTOS Ball Balancer

## Overview
This project is a real-time, closed-loop control system that balances a rolling ball on a 3-axis Stewart-style platform. The system uses a Raspberry Pi for high-speed computer vision to track the ball, and an STM32F103 microcontroller running FreeRTOS to handle the physics calculations, PID control, and servo actuation.

Unlike standard Arduino projects, this system bypasses heavy hardware abstraction layers (HAL) in favor of bare-metal register manipulation to squeeze maximum performance out of the silicon, allowing complex floating-point inverse kinematics to run at a strict 50Hz deadline.

## Hardware Architecture
* **Microcontroller:** STM32F103 (Cortex-M3)
* **Vision System:** Raspberry Pi + Camera Module
* **Actuators:** 3x Standard Hobby Servos (spaced 120° apart)
* **Communication:** UART Serial Link (115200 baud)

## Software Architecture

### 1. Bare-Metal Clock & Timer Configuration
To handle the heavy floating-point trigonometry without a hardware FPU, the STM32's Reset and Clock Control (RCC) registers are manually configured to bypass the default 8 MHz oscillator. The internal Phase-Locked Loop (PLL) is engaged to turbocharge the system bus to **64 MHz**. 
Hardware Timers (TIM2 and TIM3) are configured via direct register access to generate precise 1 MHz PWM signals, ensuring perfectly granular microsecond control over the servos to prevent jitter.

### 2. FreeRTOS & Thread Safety
The system architecture uses a Real-Time Operating System (FreeRTOS) to separate the asynchronous data intake from the strict mathematical control loop.
* **UART Task:** Driven by hardware interrupts. It captures incoming camera data letter-by-letter into a static buffer to prevent overflow.
* **Control Task:** Runs strictly every 20ms using `vTaskDelayUntil`.
* **The Mailbox Pattern:** To prevent "Data Tearing" (where the control loop reads half-updated coordinates), data is passed between tasks using a FreeRTOS Queue of length 1 configured as an atomic Mailbox. This guarantees the physics engine always reads a perfectly synchronized target frame.

## Control Theory & Physics

### PID Control (Error to Degrees)
The system treats the X and Y axes as independent 1D control problems. The PID controller measures the pixel error from the center of the platform and calculates the required corrective angles:
* **X-Axis Error** drives the **Roll** angle.
* **Y-Axis Error** drives the **Pitch** angle.

The system uses exact frame timestamps to calculate the true `dt` for the Derivative term, preventing massive output spikes caused by noisy camera data or dropped frames. Integral Anti-Windup (clamping) is implemented to prevent the system from aggressively overcompensating if the platform gets physically stuck.

### Inverse Kinematics (Degrees to PWM)
Once the PID controller requests a desired Pitch and Roll, the Inverse Kinematics engine takes over to physically move the platform:
1.  A **3D Rotation Matrix** calculates where the three top connection points must exist in 3D space.
2.  The engine calculates the 3D spatial gap between the fixed motor bases and the target top joints.
3.  Using trigonometric projection onto the 2D plane of the servo horns, the code solves for $\alpha$ (the exact angle the motor must sweep to connect the linkage rod to the target).
4.  The angle is converted to microseconds and fired to the Timer Capture/Compare Registers (CCR).

## UART Communication Protocol
The STM32 expects ASCII-formatted text strings ending in a newline (`\n`). The custom C-string parser automatically handles multiple data formats:

**Positional Data Format:**
* `X,Y\n` (Assumes ball is found)
* `Found,X,Y\n` (1 = Tracking, 0 = Lost target)

**Live PID Tuning Format:**
* `PID,Kp,Ki,Kd\n` (Allows live tuning of the control system without recompiling the STM32 firmware).

## Safety & Edge Cases
* **Dead-Man's Switch:** If the camera feed freezes or the UART wire is disconnected, a timestamp watchdog detects the stale data and safely levels the platform.
* **Kinematic Singularities:** If the PID requests a tilt angle that requires the servo linkage to stretch further than physically possible, the software clamps the trigonometric ratios to prevent `NaN` math exceptions and system crashes.
* **Boundary Clamping:** Incoming UART pixel coordinates are digitally clamped to the maximum width/height of the camera sensor to reject false-positive tracking glitches.
