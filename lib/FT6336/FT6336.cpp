// Minimal FT6336 capacitive touch driver. Part of Case Feeder Pro. GPL-3.0.
#include "FT6336.h"

static constexpr uint8_t ADDR          = 0x38;
static constexpr uint8_t REG_TD_STATUS = 0x02;   // number of touch points, then P1 XH, XL, YH, YL
static constexpr uint8_t REG_THRESHOLD = 0x80;

FT6336::FT6336(int sda, int scl, int intPin, int rstPin, int width, int height)
    : _sda(sda), _scl(scl), _int(intPin), _rst(rstPin), _width(width), _height(height) {}

void FT6336::begin() {
    Wire.begin(_sda, _scl);
    if (_int >= 0) pinMode(_int, INPUT);
    if (_rst >= 0) {
        pinMode(_rst, OUTPUT);
        digitalWrite(_rst, LOW);
        delay(10);
        digitalWrite(_rst, HIGH);
    }
    delay(200);   // controller start-up time
}

void FT6336::read() {
    isTouched = false;
    Wire.beginTransmission(ADDR);
    Wire.write(REG_TD_STATUS);
    if (Wire.endTransmission() != 0) return;

    // Only trust a complete transfer; a short read counts as no touch.
    uint8_t d[5];
    if (Wire.requestFrom(ADDR, (uint8_t)sizeof(d)) != sizeof(d)) {
        while (Wire.available()) Wire.read();
        return;
    }
    for (uint8_t& b : d) b = Wire.read();

    const uint8_t touches = d[0] & 0x0F;
    if (touches == 0 || touches > 2) return;
    const uint16_t x = ((d[1] & 0x0F) << 8) | d[2];
    const uint16_t y = ((d[3] & 0x0F) << 8) | d[4];
    points[0].x = x;   // lvgl_driver.h clamps to the screen
    points[0].y = y;
    isTouched = true;
}

void FT6336::setThreshold(uint8_t threshold) {
    writeRegister(REG_THRESHOLD, threshold);
}

void FT6336::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}
