#pragma once
#include <Arduino.h>

enum SystemState {
    STATE_NORMAL  = 0,
    STATE_WARNING = 1,
    STATE_FAULT   = 2,
    STATE_LOCKED  = 3   // FAULT latched — requires explicit reset
};

// Returns the new state given EI label, confidence, anomaly score, temperature.
// Implements hysteresis: two consecutive WARNING readings required before upgrade.
SystemState state_update(const char *label, float confidence,
                         float anomaly, float temp_c, bool temp_fault);

// Reset from LOCKED → NORMAL (call after manual acknowledgement)
void state_reset();

SystemState state_get();
const char *state_name(SystemState s);
