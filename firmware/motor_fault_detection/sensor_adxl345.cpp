#include "sensor_adxl345.h"
#include "config.h"
#include <SPI.h>
#include <math.h>

// ─── SPI helpers ─────────────────────────────────────────────────────────────

static void spi_write_reg(uint8_t reg, uint8_t val) {
    digitalWrite(ADXL345_CS_PIN, LOW);
    SPI.transfer(reg & 0x3F);   // write: bit7=0, bit6=0
    SPI.transfer(val);
    digitalWrite(ADXL345_CS_PIN, HIGH);
}

static uint8_t spi_read_reg(uint8_t reg) {
    digitalWrite(ADXL345_CS_PIN, LOW);
    SPI.transfer(0x80 | reg);   // read: bit7=1
    uint8_t val = SPI.transfer(0x00);
    digitalWrite(ADXL345_CS_PIN, HIGH);
    return val;
}

static void spi_read_burst(uint8_t reg, uint8_t *buf, uint8_t len) {
    digitalWrite(ADXL345_CS_PIN, LOW);
    SPI.transfer(0xC0 | reg);   // read + multi-byte: bit7=1, bit6=1
    for (uint8_t i = 0; i < len; i++) buf[i] = SPI.transfer(0x00);
    digitalWrite(ADXL345_CS_PIN, HIGH);
}

// ─── Public API ──────────────────────────────────────────────────────────────

bool adxl345_init() {
    pinMode(ADXL345_CS_PIN, OUTPUT);
    digitalWrite(ADXL345_CS_PIN, HIGH);

    SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, ADXL345_CS_PIN);
    SPI.setDataMode(SPI_MODE3);
    SPI.setFrequency(5000000);   // 5 MHz

    // Verify device ID
    if (spi_read_reg(ADXL345_REG_DEVID) != 0xE5) return false;

    // TODO: configure bandwidth (BW_RATE), range (DATA_FORMAT), FIFO
    // BW_RATE  0x2C → 0x0D  = 800 Hz output data rate
    // DATA_FORMAT 0x31 → 0x0B = full resolution, ±16g
    spi_write_reg(ADXL345_REG_BW_RATE, 0x0D);
    spi_write_reg(ADXL345_REG_DATA_FORMAT, 0x0B);
    spi_write_reg(ADXL345_REG_POWER_CTL, 0x08);  // measure mode

    return true;
}

bool adxl345_read(AccelData &out) {
    uint8_t buf[6];
    spi_read_burst(ADXL345_REG_DATAX0, buf, 6);

    int16_t raw_x = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t raw_y = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t raw_z = (int16_t)((buf[5] << 8) | buf[4]);

    // Full-resolution scale factor: 3.9 mg/LSB
    const float scale = 0.0039f;
    out.x = raw_x * scale;
    out.y = raw_y * scale;
    out.z = raw_z * scale;

    return true;
}

float adxl345_rms(const float *samples, uint16_t count) {
    // TODO: implement if classical RMS fallback needed alongside EI inference
    float sum = 0.0f;
    for (uint16_t i = 0; i < count; i++) sum += samples[i] * samples[i];
    return sqrtf(sum / count);
}
