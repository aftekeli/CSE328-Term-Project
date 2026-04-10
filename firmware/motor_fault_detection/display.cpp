#include "display.h"
#include "config.h"
#include <Wire.h>

// Requires: Adafruit SSD1306 + Adafruit GFX libraries
// Install via Arduino IDE Library Manager
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64

static Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

static unsigned long last_fault_blink = 0;
static bool          fault_blink_on   = false;

void display_init() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("OLED init failed");
        return;
    }
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(0, 0);
    oled.println("Motor Fault Detect");
    oled.println("Initialising...");
    oled.display();
}

void display_update(SystemState state, const EIResult &ei, float temp_c) {
    if (state == STATE_LOCKED || state == STATE_FAULT) {
        display_fault_screen();
        return;
    }

    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setCursor(0, 0);

    // Line 1: state badge
    // TODO: add triangle / bullet prefix per state
    oled.print(state_name(state));
    oled.print("  Conf:");
    oled.print((int)(ei.confidence * 100));
    oled.println("%");

    // Line 2: anomaly score
    oled.print("Anom: ");
    oled.println(ei.anomaly, 2);

    // Line 3: temperature + bar
    oled.print("Temp: ");
    oled.print(temp_c, 1);
    oled.println("C");

    // Line 4: motor + model status
    // TODO: add motor on/off and model version
    oled.print("Motor: ");
    oled.print((state == STATE_NORMAL || state == STATE_WARNING) ? "ON" : "OFF");

    oled.display();
}

void display_fault_screen() {
    if (millis() - last_fault_blink >= 500) {
        fault_blink_on = !fault_blink_on;
        last_fault_blink = millis();
    }

    oled.clearDisplay();
    if (fault_blink_on) {
        oled.setTextSize(2);
        oled.setCursor(4, 16);
        oled.println("!! MOTOR");
        oled.setCursor(4, 40);
        oled.println("STOPPED !!");
    }
    oled.display();
}
