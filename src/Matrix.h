#ifndef MATRIX_H
#define MATRIX_H

#include <Arduino.h>
#include "Vector.h"

class Matrix {
public:
    float m[3][3];

    Matrix() {
        memset(m, 0, sizeof(m));
    }

    static Matrix rotation(float angle, float x, float y, float z) {
        Matrix r;
        float c = cos(angle);
        float s = sin(angle);
        float c1 = 1.0f - c;

        r.m[0][0] = c + x * x * c1;
        r.m[0][1] = x * y * c1 - z * s;
        r.m[0][2] = x * z * c1 + y * s;

        r.m[1][0] = y * x * c1 + z * s;
        r.m[1][1] = c + y * y * c1;
        r.m[1][2] = y * z * c1 - x * s;

        r.m[2][0] = z * x * c1 - y * s;
        r.m[2][1] = z * y * c1 + x * s;
        r.m[2][2] = c + z * z * c1;

        return r;
    }

    Matrix operator*(const Matrix& other) const {
        Matrix result;
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                result.m[row][col] =
                    m[row][0] * other.m[0][col] +
                    m[row][1] * other.m[1][col] +
                    m[row][2] * other.m[2][col];
            }
        }
        return result;
    }

    Vector operator*(const Vector& v) const {
        return Vector(
            m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
            m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
            m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2]
        );
    }
};

#endif