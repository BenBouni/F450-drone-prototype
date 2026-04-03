# ESP32 Quadcopter — From-Scratch Flight Controller & Ground Control Station

A fully custom quadcopter system built from scratch, without external flight-controller libraries. Everything — from the Madgwick filter to the PID controllers, motor mixing, and radio protocol — is implemented manually in C++ on ESP32 hardware, with a Python/PyQt6 ground station for real-time telemetry visualisation .

---
# Overview

## README STRUCTURE  :
- [System Architecture](#system-architecture)
- [Repository Structure](#repository-structure)
- [Drone Firmware] :
  - FreeRTOS Task Architecture
  - Inter-task Communication
  - IMU — `class IMU` — `IMU.cpp`
  - PID Controller — `class PID` — `PID.cpp`
  - Motor Mixing — `class mixMotor` / `class moteur` — `motorLOGIC.cpp`
  - Failsafe — `class failsafe` — `Radio.cpp`
  - Radio — `class Emitor_receptor` — `Radio.cpp`
- [GCS Firmware — `test.cpp`](#gcs-firmware--testcpp)
- [Ground Station — `GS_python.py`](#ground-station--gs_pythonpy)
- [Hardware Bill of Materials](#hardware-bill-of-materials)
- [Dependencies](#dependencies)
- [TODO / Known Limitations](#todo--known-limitations)
- [Safety](#safety)

---

## System Architecture

The system is split into two physical units communicating via nRF24L01 radio modules.

<img width="1821" height="1301" alt="controll diagram" src="https://github.com/user-attachments/assets/2307bf07-ccb1-4f19-939a-b15690fba4cd" />

| Unit       | Hardware    | Role |
|------------|-------------|------|
| ESP32 B    | Drone       | IMU reading, attitude estimation, PID control, motor mixing, failsafe |
| ESP32 A    | GCS bridge  | Xbox controller (BLE), radio TX/RX, USB serial to PC |
| PC (Python)| Ground station | Telemetry display, real-time plots, 3D orientation visualisation |

---

## Repository Structure

```
├── MultiTaks_Ver.cpp   # Drone firmware (ESP32 B) — flight controller
├── test.cpp            # GCS firmware (ESP32 A)  — controller bridge
├── GS_python.py        # Ground station UI — PyQt6/OpenGL telemetry display
└── controll_diagram.png # System architecture diagram
```

---

## Drone Firmware 

### FreeRTOS Task Architecture

The drone firmware runs four concurrent FreeRTOS tasks across two ESP32 cores.

| Task | Core | Priority | Period | Responsibility |
|------|------|----------|--------|----------------|
| `Captor` | 1 | 2 | 2 ms | IMU read + Madgwick filter → attitude angles |
| `Controll` | 1 | 3 | 4 ms | PID computation + motor mixing + arming logic |
| `failsafeAction` | 1 | 4 | 4 ms | Signal-loss detection + emergency stop |
| `Radiocore` | 0 | 1 | 10 ms | nRF24L01 TX (telemetry) / RX (orders) |

`loop()` is deleted at startup — all execution happens inside the tasks.

### Inter-task Communication

Shared data is protected by four dedicated mutexes:

| Mutex | Protects |
|-------|----------|
| `xMutexRadio` | Raw received control packet |
| `xMutexControll` | `order` struct (thr, pitch, roll, yaw, armed) |
| `xMutexTele` | `tele` struct (actual roll, pitch, yaw) |
| `xMutexFailsafe` | Last valid signal timestamp |

Atomic types (`std::atomic<float>`, `std::atomic<bool>`) are used for the shared `order` and `tele` structs to allow lockless reads where appropriate.

### IMU — `class IMU`

- **Sensor:** GY-87 (MPU-6050 accelerometer + gyroscope), via I²C
- **Calibration:** 500-sample startup averaging to compute gyro bias and accelerometer offset; `AccelFC` is derived from the measured gravity magnitude rather than the datasheet value
- **Attitude estimation:** Custom Madgwick filter (quaternion-based), implemented from scratch
  - Inputs: calibrated gyro (rad/s) + normalised accelerometer
  - Aberrant acceleration values rejected via reciprocal-norm bounds check
  - Output: quaternions → Euler angles (roll, pitch, yaw in degrees)
  - `BETA` tunable at runtime via serial input
- **Update rate:** 2 ms (`Captor` task)

### PID Controller — `class PID`

Three independent PID instances for roll, pitch, and yaw.

- **Gains (initial):** Kp = 1.0, Ki = 0.0, Kd = 0.0 — *to be tuned in flight*
- **Timestep:** computed from `micros()` delta, clamped to `DT_MAX = 1 ms` to prevent windup on blocking
- **Anti-windup:** integral clamped to ±400
- **NaN/Inf guard:** invalid measurements return 0 without crashing the loop

### Motor Mixing — `class mixMotor` / `class moteur`

Standard X-frame quadcopter mixing:

```
Motor layout (top view):
  AG (front-left) ●   ● AD (front-right)
  DG (rear-left)  ●   ● DD (rear-right)

AG = thr + pitch + roll + yaw
AD = thr + pitch - roll - yaw
DG = thr - pitch + roll - yaw
DD = thr - pitch - roll + yaw
```

- PWM via `ledcWrite`, 50 Hz, 14-bit resolution
- Throttle clamped to [1000, 1600] µs to reserve headroom for corrections
- Arming/disarming propagated to all four motor instances

### Failsafe — `class failsafe`

Two-level signal-loss response:

| Condition | Delay | Action |
|-----------|-------|--------|
| Temporary loss | > 1 000 ms | Set thr = 1300, zero all corrections, disarm |
| Critical loss | > 10 000 ms | Immediate full stop (`stopTout()`) |

Signal time is refreshed on every valid received packet.

### Radio — `class Emitor_receptor`

- **Module:** nRF24L01 (CE = 21, CSN = 22)
- **Mode:** primarily listening; switches to TX every 100 ms to send telemetry
- **Telemetry sent:** battery voltage (placeholder), actual pitch, roll, yaw
- **Orders received:** throttle, pitch, roll, yaw, armed flag

---

## GCS Firmware — `test.cpp`

Runs on a second ESP32 (ESP32 A), acting as a wireless bridge between the Xbox controller and the drone.

### Tasks

| Task | Core | Role |
|------|------|------|
| `loop()` | 1 | Xbox controller read, order mapping, serial telemetry print |
| `Radiocore` | 0 | nRF24L01 TX (orders) / RX (telemetry), shared struct update |

### Controller Mapping (Xbox via BLE)

| Input | Output | Range |
|-------|--------|-------|
| Right trigger − Left trigger | Throttle | [1000, 2000] µs |
| Right stick X | Yaw setpoint | [−30°, +30°] |
| Right stick Y | Pitch setpoint | [−30°, +30°] |
| Left stick X | Roll setpoint | [−30°, +30°] |
| Xbox button (toggle) | Armed flag | true / false |

Telemetry is forwarded to the PC via USB serial at 115 200 baud, prefixed with `GS,`.

---

## Ground Station — `GS_python.py`

Python application built with PyQt6, pyqtgraph, and PyOpenGL.

### Components

| Class | Role |
|-------|------|
| `SerialWorker5` | Background `QThread`; reads serial, parses `GS,` lines, maintains 100-sample rolling buffers |
| `GSMainWindow` | Main window; real-time labels + plots |
| `GLWidget` | `QOpenGLWidget`; renders a 3D drone model rotated by live telemetry angles |

### Serial Protocol (ESP32 A → PC)

```
GS,<pitch>,<roll>,<yaw>,<extra_value> 
```
- `<extra_value>` is a placeholder for future telemetry (e.g. battery voltage)
- Lines are emitted at ~10 Hz, parsed by `SerialWorker5`, and stored in rolling buffers for plotting and visualisation

### Telemetry Computed Per Frame

- Rolling mean and standard deviation for pitch, roll, yaw, and the extra data channel
- Emitted as a single signal to the UI thread to avoid race conditions

### 3D Visualisation

OpenGL fixed-function pipeline; draws:
- Coordinate axes (XYZ)
- Central body (grey box)
- Four arms (red = front, white = rear)
- Four rotor circles (blue)

Rotations applied in Yaw → Pitch → Roll order.

---

## Hardware Bill of Materials

| Component | Details |
|-----------|---------|
| Flight controller MCU | ESP32 (dual-core, 240 MHz) |
| GCS bridge MCU | ESP32 (dual-core, 240 MHz) |
| IMU | GY-87 (MPU-6050 accel/gyro) |
| Radio | 2× nRF24L01 + PA LNA |
| ESCs | 4× standard PWM ESC (30 A) |
| Motors | 4× brushless DC |
| Controller | Xbox controller (Bluetooth) |
| Power | 3S LiPo battery (11.1 V) |

---

## Dependencies

### Drone / GCS Firmware (Arduino/ESP32)
- `RF24` — nRF24L01 driver
- `Wire` — I²C (IMU)
- `Servo` — (included, not yet active)
- `BLEGamepadClient` — Xbox BLE input (GCS only)
- FreeRTOS — included with ESP32 Arduino core

### Ground Station (Python)
 - `pyqt6` _ GUI framework_
 - `pyqtgraph` __ Real-time plotting
 - `pyserial` __ Serial communication
 - `numpy` __ Data processing
 - `PyOpenGL` __ 3D visualisation


---

## Configuration

| Parameter | Location | Default | Notes |
|-----------|----------|---------|-------|
| BETA (Madgwick) | `IMU` class | 0.1 | Tunable via serial at runtime |
| PID Kp/Ki/Kd | `PIDroll/pitch/yaw` | 1.0 / 0.0 / 0.0 | Requires in-flight tuning |
| Failsafe timeouts | `monFailsafe` constructor | 1 000 / 10 000 ms | Adjust to RF environment |
| Radio channel | `Emitor_receptor` | pipes `00001` | Must match on both ends |
| Control loop period | `frequency_controll` | 4 ms | |
| Sensor period | `frequency_captor` | 2 ms | |
| Radio period | `frequency_radio` | 10 ms | |
| Serial port (GS) | `SerialWorker5.__init__` | `COM3` | Change to match OS |

---

## TODO / Known Limitations

- [ ] PID gains not yet tuned — requires flight testing
- [ ] BETA parameter not yet tuned
- [ ] Yaw drifts over time — magnetometer (GY-87 has HMC5883L) not yet integrated
- [ ] Battery voltage in telemetry is hardcoded to 11.1 V — ADC measurement not implemented yet 
- [ ] GCS `RadioAlternateEmitor_Receptor::update()` calls `receivePacket` redundantly (logic bug)
- [ ] Deadzone not applied to stick inputs — may cause drift at low throttle
- [ ] Failsafe only cuts motors — no controlled descent / return-to-home
- [ ] `draw_drone()` defined twice in `GLWidget` (second definition silently overrides first)
- [ ] GPS (position hold, return-to-home) — planned
- [ ] Barometer (altitude hold) — planned
- [ ] Kill switch / low-battery buzzer — planned
- [ ] might add a buzzer for low battery / arming feedback / signal loss warning / calibration status .
- [ ] might design a cutom controller with it's radio and receive the telemetry directly on the pc without the need of the second esp32 as a bridge so the code can be more universal, but for now the xbox controller is more convenient for testing and tuning .
- [ ] might add a camera and implement a first-person view (FPV) system, streaming video back to the ground station.

---

## Safety

- **Always arm on a bench with propellers removed** when tuning PID or BETA parameters.
- The failsafe will disarm and stop all motors after 10 seconds of signal loss.
- Throttle is clamped to a maximum of 1600 µs to preserve correction authority.
- The LED on pin 2 is HIGH during IMU calibration and LOW when the drone is ready.




