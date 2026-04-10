#pragma once
#include <Arduino.h>

// ADXL345 register addresses
#define ADXL345_REG_DEVID       0x00
#define ADXL345_REG_POWER_CTL   0x2D
#define ADXL345_REG_DATA_FORMAT 0x31
#define ADXL345_REG_BW_RATE     0x2C
#define ADXL345_REG_DATAX0      0x32

struct AccelData {
    float x, y, z;   // acceleration in g
};

bool    adxl345_init();
bool    adxl345_read(AccelData &out);
float   adxl345_rms(const float *samples, uint16_t count);
