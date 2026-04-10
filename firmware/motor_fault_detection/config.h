#pragma once

// ─── SPI — ADXL345 ───────────────────────────────────────────────────────────
#define ADXL345_CS_PIN    5
#define SPI_SCK_PIN      18
#define SPI_MISO_PIN     19
#define SPI_MOSI_PIN     23

// ─── I2C — MLX90614 + OLED ───────────────────────────────────────────────────
#define I2C_SDA_PIN      21
#define I2C_SCL_PIN      22
#define MLX90614_ADDR    0x5A
#define OLED_ADDR        0x3C

// ─── Actuators ───────────────────────────────────────────────────────────────
#define RELAY_PIN        26   // LOW = relay energised (motor off)
#define RGB_R_PIN        25
#define RGB_G_PIN        33
#define RGB_B_PIN        32

// ─── Fault Thresholds ────────────────────────────────────────────────────────
#define VIB_WARN_THRESHOLD   0.30f  // g  — RMS above this → WARNING
#define VIB_FAULT_THRESHOLD  0.70f  // g  — RMS above this → FAULT
#define TEMP_FAULT_CELSIUS   75.0f  // °C — motor surface temp limit

// ─── Edge Impulse Inference ──────────────────────────────────────────────────
#define EI_SAMPLE_FREQ_HZ    62.5f
#define EI_WINDOW_MS         1000   // 1-second inference window
#define EI_CONFIDENCE_MIN    0.70f  // minimum confidence to accept a label
#define EI_ANOMALY_THRESHOLD 0.50f  // K-Means anomaly score threshold

// ─── Timing ──────────────────────────────────────────────────────────────────
#define LOOP_INTERVAL_MS     500
#define DISPLAY_REFRESH_MS   500
