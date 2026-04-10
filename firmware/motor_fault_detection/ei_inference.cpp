#include "ei_inference.h"
#include "config.h"

// Uncomment after installing the Edge Impulse Arduino library:
// #include <motor_fault_detection_inferencing.h>

bool ei_inference_init() {
    // TODO: any EI-specific init (e.g. set logging level)
    return true;
}

EIResult ei_run(const float *accel_buf, uint16_t buf_len) {
    EIResult result = {"unknown", 0.0f, 0.0f, false};

    // TODO: fill signal struct and call ei_run_classifier()
    //
    // signal_t signal;
    // numpy::signal_from_buffer(accel_buf, buf_len, &signal);
    //
    // ei_impulse_result_t ei_result = {0};
    // EI_IMPULSE_ERROR err = run_classifier(&signal, &ei_result, false);
    // if (err != EI_IMPULSE_OK) return result;
    //
    // // Find highest-confidence label
    // float best = 0.0f;
    // for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    //     if (ei_result.classification[i].value > best) {
    //         best = ei_result.classification[i].value;
    //         strncpy(result.label, ei_result.classification[i].label, 15);
    //     }
    // }
    // result.confidence = best;
    // result.anomaly    = ei_result.anomaly;
    // result.valid      = (best >= EI_CONFIDENCE_MIN);

    return result;
}
