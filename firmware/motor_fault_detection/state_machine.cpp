#include "state_machine.h"
#include "config.h"
#include <string.h>

static SystemState current_state  = STATE_NORMAL;
static uint8_t     warn_streak    = 0;   // consecutive WARNING readings
static const uint8_t WARN_STREAK_NEEDED = 2;

SystemState state_update(const char *label, float confidence,
                         float anomaly, float temp_c, bool temp_fault) {
    if (current_state == STATE_LOCKED) return STATE_LOCKED;

    // Temperature path takes priority — immediate FAULT
    if (temp_fault) {
        current_state = STATE_FAULT;
        warn_streak = 0;
        return current_state;
    }

    // Anomaly detection: unknown vibration pattern → escalate
    if (anomaly >= EI_ANOMALY_THRESHOLD && current_state == STATE_NORMAL) {
        current_state = STATE_WARNING;
        return current_state;
    }

    // TODO: implement full transition logic using EI label
    // if strcmp(label, "fault") == 0 && confidence >= EI_CONFIDENCE_MIN → FAULT
    // if strcmp(label, "warning") == 0 → increment streak, upgrade after WARN_STREAK_NEEDED
    // if strcmp(label, "normal") == 0  → decrement streak / reset to NORMAL

    if (strcmp(label, "fault") == 0 && confidence >= EI_CONFIDENCE_MIN) {
        current_state = STATE_FAULT;
        warn_streak = 0;
    } else if (strcmp(label, "warning") == 0 && confidence >= EI_CONFIDENCE_MIN) {
        warn_streak++;
        if (warn_streak >= WARN_STREAK_NEEDED) current_state = STATE_WARNING;
    } else if (strcmp(label, "normal") == 0 && confidence >= EI_CONFIDENCE_MIN) {
        if (current_state == STATE_WARNING) warn_streak = 0;
        current_state = STATE_NORMAL;
    }

    // Latch FAULT
    if (current_state == STATE_FAULT) current_state = STATE_LOCKED;

    return current_state;
}

void state_reset() {
    current_state = STATE_NORMAL;
    warn_streak   = 0;
}

SystemState state_get() { return current_state; }

const char *state_name(SystemState s) {
    switch (s) {
        case STATE_NORMAL:  return "NORMAL";
        case STATE_WARNING: return "WARNING";
        case STATE_FAULT:   return "FAULT";
        case STATE_LOCKED:  return "LOCKED";
        default:            return "UNKNOWN";
    }
}
