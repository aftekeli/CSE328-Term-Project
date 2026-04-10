# ESP32 Motor Fault Detection with On-Device TinyML

**CSE328 — Internet of Things | Term Project**
**Student:** Ahmet Faruk Tekeli — 20220808617

---

## Overview

An ESP32-based system that detects motor faults in real time using a TinyML model trained and deployed via [Edge Impulse](https://edgeimpulse.com). Vibration data from an ADXL345 accelerometer is classified entirely on-device — no cloud round-trip at inference time — into three states: **NORMAL**, **WARNING**, and **FAULT**. A secondary MLX90614 IR thermometer provides an independent overheat threshold check.

## Hardware

| Component | Interface | Role |
|-----------|-----------|------|
| ESP32 | — | Main MCU, TinyML inference |
| ADXL345 | SPI | Vibration — primary ML input |
| MLX90614 | I2C | Temperature — independent threshold |
| OLED 128×64 (SSD1306) | I2C | Live status display |
| 5V Relay | GPIO 26 | Cuts motor power on FAULT |
| RGB LED | GPIO 25/33/32 | Green / Amber / Red state indicator |
| DC Motor | via relay | Simulation target |

See [`hardware/pin_mapping.md`](hardware/pin_mapping.md) for full wiring details.

## Repository Structure

```
firmware/
  motor_fault_detection/
    motor_fault_detection.ino   main sketch
    config.h                    pin definitions & thresholds
    sensor_adxl345.h/.cpp       ADXL345 SPI driver
    sensor_mlx90614.h/.cpp      MLX90614 I2C driver
    ei_inference.h/.cpp         Edge Impulse inference wrapper
    state_machine.h/.cpp        NORMAL / WARNING / FAULT / LOCKED logic
    actuator.h/.cpp             relay + RGB LED control
    display.h/.cpp              OLED SSD1306 display
hardware/
  pin_mapping.md
  components.md
```

## Setup

### 1. Arduino Libraries

Install via **Arduino IDE → Sketch → Include Library → Manage Libraries**:

- `Adafruit SSD1306`
- `Adafruit GFX Library`

### 2. Edge Impulse Model

The TinyML model is not included in this repository — it is trained on your own vibration data and downloaded per-device.

1. Create a project at [edgeimpulse.com](https://edgeimpulse.com)
2. Collect vibration data using the Edge Impulse Data Forwarder (3 classes: `normal`, `warning`, `fault`)
3. Train: Spectral Analysis block → Neural Network → K-Means Anomaly
4. **Deployment → Arduino library → Download ZIP**
5. Arduino IDE → **Sketch → Include Library → Add .ZIP Library**
6. Uncomment the `#include` line in `ei_inference.h` and rebuild

See the [Motion Recognition tutorial](https://docs.edgeimpulse.com/tutorials/end-to-end/motion-recognition) for step-by-step guidance.

### 3. Flash

Open `firmware/motor_fault_detection/motor_fault_detection.ino` in Arduino IDE, select **ESP32 Dev Module**, and upload.

## Fault Detection Logic

![System Architecture](docs/system_architecture.png)

The system uses two independent fault detection paths that feed a single state machine:

| Path | Sensor | Method | Triggers |
|------|--------|--------|---------|
| Vibration | ADXL345 | Edge Impulse TinyML (Spectral + NN + K-Means) | WARNING / FAULT |
| Temperature | MLX90614 | Classic threshold (≥ 75 °C) | FAULT |

**State transitions:**

```
NORMAL ──→ WARNING ──→ FAULT ──→ LOCKED
  │            │           │
Green LED   Amber LED   Red LED (blink 2 Hz)
Motor ON    Motor ON    Relay opens → Motor OFF
```

- **LOCKED** state latches after FAULT — motor cannot restart without explicit reset
- **K-Means anomaly block** catches unseen fault patterns not present in training data
- Two consecutive WARNING inferences required before state upgrade (hysteresis)
