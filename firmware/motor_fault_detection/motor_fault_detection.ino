/*
 * ESP32 Motor Fault Detection with On-Device TinyML
 * CSE328 — Internet of Things Term Project
 * Author: Ahmet Faruk Tekeli — 20220808617
 *
 * Required libraries (install via Arduino IDE Library Manager):
 *   - Adafruit SSD1306
 *   - Adafruit GFX
 *   - Adafruit MLX90614  (optional convenience wrapper)
 *   - Edge Impulse Arduino SDK  (add via .ZIP after training)
 *
 * See README.md for full setup and Edge Impulse deployment instructions.
 */

#include "config.h"
#include "sensor_adxl345.h"
#include "sensor_mlx90614.h"
#include "ei_inference.h"
#include "state_machine.h"
#include "actuator.h"
#include "display.h"

// ─── Inference sample buffer ─────────────────────────────────────────────────
// 3 axes × (EI_SAMPLE_FREQ_HZ × EI_WINDOW_MS / 1000) samples
static const uint16_t SAMPLE_COUNT = (uint16_t)(EI_SAMPLE_FREQ_HZ * EI_WINDOW_MS / 1000);
static const uint16_t BUF_LEN      = SAMPLE_COUNT * 3;
static float accel_buf[BUF_LEN];

static unsigned long last_loop_ms = 0;

// ─── setup() ─────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("Motor Fault Detection — boot");

    if (!adxl345_init()) {
        Serial.println("ERROR: ADXL345 init failed");
        while (true) delay(1000);
    }

    if (!mlx90614_init()) {
        Serial.println("WARN: MLX90614 not found — temperature path disabled");
    }

    ei_inference_init();
    actuator_init();
    display_init();

    Serial.println("Init OK. Starting main loop.");
}

// ─── loop() ──────────────────────────────────────────────────────────────────
void loop() {
    if (millis() - last_loop_ms < LOOP_INTERVAL_MS) return;
    last_loop_ms = millis();

    // 1. Collect accelerometer samples for EI inference window
    // TODO: replace with interrupt-driven FIFO read for accurate timing
    for (uint16_t i = 0; i < SAMPLE_COUNT; i++) {
        AccelData a;
        adxl345_read(a);
        accel_buf[i * 3 + 0] = a.x;
        accel_buf[i * 3 + 1] = a.y;
        accel_buf[i * 3 + 2] = a.z;
        delayMicroseconds((uint32_t)(1000000.0f / EI_SAMPLE_FREQ_HZ));
    }

    // 2. Run Edge Impulse inference
    EIResult ei = ei_run(accel_buf, BUF_LEN);

    // 3. Read temperature
    float temp_c    = mlx90614_read_celsius();
    bool  temp_ovht = mlx90614_fault(temp_c);

    // 4. Update state machine
    SystemState state = state_update(ei.label, ei.confidence,
                                     ei.anomaly, temp_c, temp_ovht);

    // 5. Drive actuators (relay + RGB LED)
    actuator_update(state);

    // 6. Update OLED
    display_update(state, ei, temp_c);

    // 7. Serial debug output
    Serial.printf("[%s] EI=%s(%.2f) Anom=%.2f Temp=%.1fC\n",
                  state_name(state), ei.label, ei.confidence,
                  ei.anomaly, temp_c);

    // TODO: optional Wi-Fi telemetry to Edge Impulse / ThingSpeak
}
