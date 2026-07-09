/*
 * Compatibility wrapper: keep class name HP206C so existing project code
 * does not need refactor, but internally use BMP390 (Adafruit_BMP3XX).
 */
#ifndef HP206C_H
#define HP206C_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP3XX.h>

class HP206C {
public:
    bool begin();
    bool readAll(float* temp_C, float* pressure_hPa);
    uint8_t getAddress() const { return i2c_address_; }

private:
    bool configureSensor();

    Adafruit_BMP3XX bmp_;
    uint8_t i2c_address_ = 0;
};

#endif
