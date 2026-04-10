#include "sensor_mlx90614.h"
#include "config.h"
#include <Wire.h>

// MLX90614 SMBus command for object temperature
#define MLX90614_CMD_TOBJ1  0x07

bool mlx90614_init() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(100000);

    // Probe device
    Wire.beginTransmission(MLX90614_ADDR);
    return (Wire.endTransmission() == 0);
}

float mlx90614_read_celsius() {
    Wire.beginTransmission(MLX90614_ADDR);
    Wire.write(MLX90614_CMD_TOBJ1);
    Wire.endTransmission(false);

    Wire.requestFrom((uint8_t)MLX90614_ADDR, (uint8_t)3);
    if (Wire.available() < 3) return -999.0f;

    uint8_t lo  = Wire.read();
    uint8_t hi  = Wire.read();
    // uint8_t pec = Wire.read();  // PEC byte — ignored for now

    uint16_t raw = ((uint16_t)hi << 8) | lo;
    float kelvin = raw * 0.02f;
    return kelvin - 273.15f;
}

bool mlx90614_fault(float temp_c) {
    return temp_c >= TEMP_FAULT_CELSIUS;
}
