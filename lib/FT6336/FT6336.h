// Minimal FT6336 capacitive touch driver (I2C, first touch point only).
// Coordinates are raw panel coordinates (portrait); lvgl_driver.h maps them
// to the screen orientation.
// Part of Case Feeder Pro. Licensed under GPL-3.0.
#ifndef FT6336_H
#define FT6336_H

#include <Arduino.h>
#include <Wire.h>

struct TouchPoint {
    uint16_t x;
    uint16_t y;
};

class FT6336 {
public:
    FT6336(int sda, int scl, int intPin, int rstPin, int width, int height);

    void begin();                          // starts I2C and resets the controller
    void read();                           // updates isTouched / points[0]
    void setThreshold(uint8_t threshold);  // lower = more sensitive

    bool       isTouched = false;
    TouchPoint points[1] = {};

private:
    int _sda, _scl, _int, _rst, _width, _height;
    void writeRegister(uint8_t reg, uint8_t value);
};

#endif
