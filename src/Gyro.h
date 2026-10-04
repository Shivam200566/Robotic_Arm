#ifndef GYRO_H
#define GYRO_H

#include <Wire.h>
#include <Arduino.h>

const float ACC_SCALE = 9.81f / 16384.0f;
const float ROT_SCALE = 1.0f / 131.0f;

const uint8_t PWR_MGMT_1 = 0x6B;
const uint8_t GYRO_CONFIG = 0x1B;
const uint8_t ACCEL_CONFIG = 0x1C;
const uint8_t ACCEL_XOUT_H = 0x3B;

class Gyro {
public:
    uint8_t address;
    float positionA[3];
    float rotationV[3];
    int correctionR[3];
    float temperature;
    int oldTime;
    int gscale, ascale;

    Gyro(int ascale = 0, int gscale = 0, uint8_t address = 0x68) {
        this->address = address;
        this->ascale = ascale;
        this->gscale = gscale;
        correctionR[0] = correctionR[1] = correctionR[2] = 0;
        oldTime = 0;
    }

    void begin() {
        Wire.begin();
        wakeUp();
        setScale();
    }

    void wakeUp() {
        Wire.beginTransmission(address);
        Wire.write(PWR_MGMT_1);
        Wire.write(0);
        Wire.endTransmission(true);
    }

    void setScale() {
        Wire.beginTransmission(address);
        Wire.write(GYRO_CONFIG);
        Wire.write(gscale << 3);
        Wire.endTransmission(true);

        Wire.beginTransmission(address);
        Wire.write(ACCEL_CONFIG);
        Wire.write(ascale << 3);
        Wire.endTransmission(true);
    }

    signed short readShort() const {
        return (Wire.read() << 8) | Wire.read();
    }

    void poll() {
        Wire.beginTransmission(address);
        Wire.write(ACCEL_XOUT_H);
        Wire.endTransmission(false);
        Wire.requestFrom(address, (uint8_t)14, (uint8_t)true);

        positionA[0] = readShort() * ACC_SCALE * (1 << ascale);
        positionA[1] = readShort() * ACC_SCALE * (1 << ascale);
        positionA[2] = readShort() * ACC_SCALE * (1 << ascale);
        temperature = readShort() / 340.0f + 36.53f;

        rotationV[0] = (readShort() - correctionR[0]) * ROT_SCALE * (1 << gscale);
        rotationV[1] = (readShort() - correctionR[1]) * ROT_SCALE * (1 << gscale);
        rotationV[2] = (readShort() - correctionR[2]) * ROT_SCALE * (1 << gscale);
    }
};

#endif