# Pin Mapping

## ADXL345 — SPI

| ADXL345 Pin | ESP32 Pin | Notes |
|-------------|-----------|-------|
| CS          | GPIO 5    | Chip Select |
| SCL / SCLK  | GPIO 18   | SPI Clock |
| SDA / MOSI  | GPIO 23   | Master Out |
| SDO / MISO  | GPIO 19   | Master In |
| VCC         | 3.3V      | |
| GND         | GND       | |

## MLX90614 — I2C

| MLX90614 Pin | ESP32 Pin | Notes |
|--------------|-----------|-------|
| SDA          | GPIO 21   | Shared I2C bus |
| SCL          | GPIO 22   | Shared I2C bus |
| VCC          | 3.3V      | |
| GND          | GND       | |

## OLED 128x64 — I2C

| OLED Pin | ESP32 Pin | Notes |
|----------|-----------|-------|
| SDA      | GPIO 21   | Shared with MLX90614 |
| SCL      | GPIO 22   | Shared with MLX90614 |
| VCC      | 3.3V      | |
| GND      | GND       | |

## Actuators — GPIO

| Component         | ESP32 Pin | Active Level | Notes |
|-------------------|-----------|--------------|-------|
| Relay IN          | GPIO 26   | HIGH = motor OFF | LOW = motor running |
| RGB LED — Red     | GPIO 25   | HIGH = on | PWM capable |
| RGB LED — Green   | GPIO 33   | HIGH = on | PWM capable |
| RGB LED — Blue    | GPIO 32   | HIGH = on | PWM capable |

## Notes

- ADXL345 must be **rigidly mounted** to the motor body (epoxy or strong double-sided tape). Loose mounting will corrupt vibration data and invalidate the TinyML model.
- I2C bus is shared between MLX90614 (address 0x5A) and OLED SSD1306 (address 0x3C) — no conflict.
- Relay module: verify active-high vs active-low before wiring. Logic above assumes active-HIGH (IN=HIGH → coil energised → NC opens → motor cut).
