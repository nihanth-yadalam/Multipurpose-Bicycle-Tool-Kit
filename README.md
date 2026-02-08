**Multipurpose Bicycle Kit** is an embedded safety and ride-tracking system for bicycles. It combines crash detection, GPS location reporting, and ride statistics into a single STM32-based unit with a simple user interface.

## Problem Statement
Cyclists are vulnerable to crashes and often ride in areas where immediate help is not available. A practical system must:
- Detect a real crash (not potholes or sharp turns).
- Confirm the rider is down before sending an alert.
- Give the rider a chance to cancel false alarms.
- Provide accurate, immediate location details.
- Track ride statistics for everyday use.

## Our Solution
We built a microcontroller system around STM32F4 that fuses data from an MPU6050 IMU and a NEO-6M GPS. A multi-stage algorithm filters false positives and confirms a fall before triggering an alert. The GPS runs continuously so the most recent coordinates are ready at the moment of an incident. A keypad, LCD, buzzer, LED, and button provide local control and feedback.

## Key Features
- Crash detection with multi-stage confirmation.
- Grace period with buzzer/LED and user cancel button.
- Continuous GPS tracking with NMEA parsing.
- Crash location snapshot and optional path history via circular buffer.
- Ride tracking: speed, distance, average/max speed, lap mode, and trip logging.
- LCD + keypad interface, LED status, and buzzer alerts.

## System Architecture
- MCU: STM32F4 (direct register programming)
- IMU: MPU6050 (I2C)
- GPS: NEO-6M (UART/NMEA)
- UI: 16x2 LCD via PCF8574 (I2C), 4x4 keypad
- Alerts: LED + buzzer + cancel button

## Core Algorithms

### 1) Crash Alert Algorithm (State Machine)
The crash detection uses both acceleration and angular velocity to reduce false alarms. The logic runs in a timed loop and proceeds through stages:

- **Monitor**: sample IMU at ~100 Hz and compute metrics.
- **Trigger**: detect a simultaneous high-impact and high-rotation event.
- **Confirm**: within a time window, require the bike to stay tilted beyond a threshold for a continuous streak.
- **Grace**: buzzer + blinking LED for a cancel window. If no cancel, send alert.

Thresholds and timing (from the current code):
- Impact threshold: 1.0 g
- Rotation threshold: 250 dps
- Tilt threshold: 70 degrees
- Confirm window: 10 s
- Tilt streak: 5 s
- Grace window: 30 s

These values are tuned for road tests and can be adjusted in the firmware.

### 2) IMU Signal Processing
Raw accelerometer and gyroscope values are converted to physical units, then combined into:
- Resultant acceleration magnitude $A_{res}$
- Resultant angular velocity magnitude $G_{res}$
- Tilt angle from the vertical axis

### 3) GPS Parsing and Location Snapshot
The GPS runs continuously. GPGGA sentences are parsed to extract:
- UTC time
- Latitude / longitude in decimal degrees
- Fix quality and satellites

When a crash is confirmed, the most recent coordinates are used to build an alert (e.g., a Google Maps link).

### 4) Path History (Circular Buffer)
To improve reliability, a short rolling history of recent GPS points can be stored in a circular array. This helps recover last known locations if signal drops at impact and provides direction-of-travel context.

### 5) Ride Tracker
The system computes speed and distance using consecutive GPS points and time deltas. It maintains:
- Real-time speed
- Average and maximum speed
- Total ride time
- Lap segments and trip logging

## Repository Layout
- [for git/main.c](for%20git/main.c): Primary firmware (STM32F4) with crash detection, GPS parsing, LCD/keypad, and alerts.
- [Bicycle/main_function_total _code_C.txt](Bicycle/main_function_total%20_code_C.txt): Consolidated reference version of the main firmware.
- [Bicycle/MPU_6050_Code_c.txt](Bicycle/MPU_6050_Code_c.txt): Standalone MPU6050 test and crash logic reference.
- [Bicycle/Script_R1_ES_Bicycle.txt](Bicycle/Script_R1_ES_Bicycle.txt): Presentation/script notes describing the algorithm and GPS features.

## How It Works (High Level Flow)
1. System initializes I2C, UART, LCD, keypad, IMU, buzzer, LED, and button.
2. GPS runs continuously in the background via UART interrupt.
3. IMU is sampled at fixed intervals; crash state machine evaluates conditions.
4. On confirmed crash, buzzer/LED activate; user can cancel.
5. If not canceled, the alert uses the most recent GPS data.

## Hardware Notes
This firmware uses direct register access for STM32F4 peripherals and assumes:
- I2C on PB8/PB9 for MPU6050 and PCF8574
- UART2 on PA2/PA3 for GPS
- LED on PC13, buzzer on PA9, button on PA1
Adjust pins as required for your board.

## Future Improvements
- Persist trip logs in external memory (SD/EEPROM).
- Bluetooth or GSM auto-alert integration for emergency messaging.
- Add configurable thresholds via keypad menu.

## License
See [LICENSE](LICENSE).
