**Multipurpose Bicycle Kit** is an embedded safety and ride-tracking system for bicycles. It combines crash detection, GPS location reporting, and ride statistics into a single STM32-based unit with a simple user interface.

## Problem Statement
Modern vehicles include automatic safety and security features, but bicycles typically do not. This creates two gaps:
- After a crash, a rider may be unable to call for help.
- A bicycle is easy to steal because most locks are purely mechanical.

We need a low-cost system that brings crash alerts, location tracking, and anti-theft features to bicycles while avoiding false alarms from potholes or sharp turns.

## Our Solution
We built a microcontroller system around STM32F4 that fuses data from an MPU6050 IMU and a NEO-6M GPS. A multi-stage algorithm filters false positives and confirms a fall before triggering an alert. The GPS runs continuously so the most recent coordinates are ready at the moment of an incident. A keypad, LCD, buzzer, LED, and button provide local control, while Bluetooth sends alerts to a phone.

## Key Features
- Anti-theft arming and password unlock via keypad.
- Wrong password attempts trigger buzzer + location alert.
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
- Bluetooth: HC-05 (alert delivery)
- UI: 16x2 LCD via PCF8574 (I2C), 4x4 keypad
- Alerts: LED + buzzer + cancel button

## Core Algorithms

### 1) Smart Crash Detection (6-Step Flow)
The crash detection uses both acceleration and angular velocity to reduce false alarms. The logic runs in a timed loop and proceeds through steps:

1. **Monitor (Always Watching)**: continuously sample IMU and compute metrics.
2. **Trigger (Detecting a Fall)**: require both a hard impact and a fast rotation at the same time.
3. **Confirm (Sure it is a Real Crash)**: verify the bike is tilted beyond the threshold and stays down for a continuous streak.
4. **Grace Period**: buzzer + blinking LED for 30 seconds.
5. **User Action**: rider can cancel the alert during the grace period.
6. **Send Alert**: if not canceled, send a crash alert with location via Bluetooth.

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

When a crash is confirmed, the most recent coordinates are used to build an alert (e.g., a Google Maps link). The same coordinates can be sent on a theft alert after wrong password attempts.

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

## Anti-Theft Workflow
1. **Arm**: power on and press `AAA` on the keypad (LCD shows a welcome message).
2. **Unlock**: enter the correct password to enter Ride Mode.
3. **Wrong Password**: after two wrong attempts, the buzzer sounds and a theft alert is sent with GPS coordinates via Bluetooth.
4. **Stop Alarm**: enter the special code `###` to silence the buzzer if triggered accidentally.
5. **Power Off**: press `DDD` to shut down the system after riding.

## How It Works (High Level Flow)
1. System initializes I2C, UART, LCD, keypad, IMU, buzzer, LED, and button.
2. GPS runs continuously in the background via UART interrupt.
3. IMU is sampled at fixed intervals; crash state machine evaluates conditions.
4. On confirmed crash, buzzer/LED activate; user can cancel.
5. If not canceled, the alert uses the most recent GPS data and sends it over Bluetooth.

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
