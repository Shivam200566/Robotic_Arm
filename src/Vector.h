#ifndef VECTOR_H
#define VECTOR_H

#include <Arduino.h>

class Vector {
public:
    float data[3];

    Vector(float x = 0.0f, float y = 0.0f, float z = 0.0f) {
        data[0] = x;
        data[1] = y;
        data[2] = z;
    }

    float& operator[](int i) { return data[i]; }
    const float& operator[](int i) const { return data[i]; }

    void normalize() {
        float len = sqrt(data[0] * data[0] + data[1] * data[1] + data[2] * data[2]);
        if (len > 0.00001f) {
            data[0] /= len;
            data[1] /= len;
            data[2] /= len;
        }
    }

    Vector cross(const Vector& v) const {
        return Vector(
            data[1] * v[2] - data[2] * v[1],
            data[2] * v[0] - data[0] * v[2],
            data[0] * v[1] - data[1] * v[0]
        );
    }
};

#endif