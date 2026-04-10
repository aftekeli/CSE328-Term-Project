#include "actuator.h"
#include "config.h"
#include <Arduino.h>

// RGB LED state blink tracking
static unsigned long last_blink_ms = 0;
static bool          blink_on      = false;

void actuator_init() {
    pinMode(RELAY_PIN, OUTPUT);
    pinMode(RGB_R_PIN, OUTPUT);
    pinMode(RGB_G_PIN, OUTPUT);
    pinMode(RGB_B_PIN, OUTPUT);

    actuator_motor_enable();
    actuator_update(STATE_NORMAL);
}

static void set_rgb(bool r, bool g, bool b) {
    digitalWrite(RGB_R_PIN, r ? HIGH : LOW);
    digitalWrite(RGB_G_PIN, g ? HIGH : LOW);
    digitalWrite(RGB_B_PIN, b ? HIGH : LOW);
}

void actuator_update(SystemState state) {
    switch (state) {
        case STATE_NORMAL:
            actuator_motor_enable();
            set_rgb(false, true, false);   // green
            break;

        case STATE_WARNING:
            actuator_motor_enable();
            set_rgb(true, true, false);    // amber
            break;

        case STATE_FAULT:
        case STATE_LOCKED:
            actuator_motor_disable();
            // Blink red at 2 Hz
            if (millis() - last_blink_ms >= 250) {
                blink_on = !blink_on;
                set_rgb(blink_on, false, false);
                last_blink_ms = millis();
            }
            break;
    }
}

void actuator_motor_enable() {
    digitalWrite(RELAY_PIN, LOW);    // relay de-energised = motor powered
}

void actuator_motor_disable() {
    digitalWrite(RELAY_PIN, HIGH);   // relay energised = motor cut
}
