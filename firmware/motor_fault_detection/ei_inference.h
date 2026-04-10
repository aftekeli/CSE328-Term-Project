#pragma once
#include <Arduino.h>

// Edge Impulse inference result
struct EIResult {
    char    label[16];    // "normal", "warning", "fault"
    float   confidence;   // 0.0 – 1.0
    float   anomaly;      // K-Means anomaly score
    bool    valid;        // false if inference failed or confidence too low
};

// NOTE: Before using these functions, install the Edge Impulse Arduino library:
//   1. Train your model in Edge Impulse Studio
//   2. Deployment → Arduino library → Download ZIP
//   3. Arduino IDE → Sketch → Include Library → Add .ZIP Library
//
// Then uncomment the #include below and rebuild.
//
// #include <motor_fault_detection_inferencing.h>

bool     ei_inference_init();
EIResult ei_run(const float *accel_buf, uint16_t buf_len);
