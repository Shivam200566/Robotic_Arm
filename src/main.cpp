#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

#include "Vector.h"
#include "Matrix.h"
#include "Gyro.h"

const int servoCount = 6;
const int servoPulseMin = 128;
const int servoPulseMax = 640;
const int servoPulseWidth = servoPulseMax - servoPulseMin;

int analogMin = 0;
const int analogMax = 1023;
const float attenuation = 0.8f;

Vector compass0;

float servoRange[servoCount][2] = {
  {0.2f, 0.9f}, 
  {0.35f, 0.95f}, 
  {0.0f, 1.0f}, 
  {0.25f, 1.0f}, 
  {0.0f, 0.75f}, 
  {0.15f, 0.4f}
};

Vector sensors[4];
float analog = 0;
float angles[servoCount] = {-1};
float servoValue[servoCount] = {0.5f, 0.4f, 0.2f, 0.25f, 0.0f, 0.2f};
int reversedRotation[servoCount] = {0, 1, 0, 1, 0, 0};
float offsetValue[servoCount] = {0.5f, 0.4f, 0.2f, 0.25f, 0.0f, 0.2f};

Gyro accel1(0, 1, 0x68);
Gyro accel2(0, 1, 0x69);
Adafruit_LSM303_Accel_Unified accel3 = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified mag1 = Adafruit_LSM303_Mag_Unified(12345);

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

void readSensors() {
  accel1.poll();
  accel2.poll();  
  sensors_event_t accel3e;
  accel3.getEvent(&accel3e);  
  sensors_event_t mag1e;
  mag1.getEvent(&mag1e);

  sensors[0] = Vector(accel1.positionA[0], accel1.positionA[1], accel1.positionA[2]);
  sensors[0].normalize();

  sensors[1] = Vector(accel2.positionA[0], accel2.positionA[1], accel2.positionA[2]);
  sensors[1].normalize();

  sensors[2] = Vector(accel3e.acceleration.x, accel3e.acceleration.y, accel3e.acceleration.z);
  sensors[2].normalize();
  sensors[3] = Vector(mag1e.magnetic.x, mag1e.magnetic.y, mag1e.magnetic.z);

  analog = float(analogRead(A0) - analogMin) / (analogMax - analogMin);
}

void calibrate() {
  analogMin = analogRead(A0);
  readSensors();

  float a1 = -atan2(-sensors[1][1], -sensors[1][0]);
  Matrix r0 = Matrix::rotation(-a1, 0, 1, 0);
  Matrix r1 = Matrix::rotation(angles[4], 1, 0, 0);
  Vector v = r0 * r1 * sensors[3];
  v[2] = 0;
  v.normalize();
  compass0 = v;
}

void setServoAngle(int servo, float angle) {
  float v = max(servoRange[servo][0], min(servoRange[servo][1], angle / float(M_PI) + offsetValue[servo]));
  if (servoValue[servo] >= 0)
      servoValue[servo] = servoValue[servo] * attenuation + v * (1.0f - attenuation);
  else 
    servoValue[servo] = v;
}

void updateServos() {
  for (int i = 0; i < servoCount; i++) {
    if (servoValue[i] < 0) {
      pwm.setPWM(i, 4096, 0);
    } else {
      pwm.setPWM(i, 0, int(servoPulseMin + (reversedRotation[i] ? 1.0f - servoValue[i] : servoValue[i]) * servoPulseWidth)); 
    }
  }
}

void calculateAngles() {
  angles[1] = -atan2(-sensors[0][2], -sensors[0][1]);
  angles[2] = atan2(-sensors[0][0], -sensors[0][1]);
  float elbowPitch = atan2(-sensors[1][0], -sensors[1][1]);
  angles[3] = elbowPitch - angles[2];
  angles[4] = atan2(sensors[2][1], sensors[2][2]);
  angles[5] = (sqrt(max(analog, 0.0f)) * (servoRange[5][1] - servoRange[5][0])) * M_PI;

  float a1 = -atan2(-sensors[1][1], -sensors[1][0]);
  Matrix r0 = Matrix::rotation(-a1, 0, 1, 0);
  Matrix r1 = Matrix::rotation(angles[4], 1, 0, 0);
  
  Vector v = r0 * r1 * sensors[3];
  v[2] = 0;
  v.normalize();
  
  Vector a = v.cross(compass0);
  angles[0] = -asin(a[2]);

  for (int i = 0; i < servoCount; i++) {
    setServoAngle(i, angles[i]);
  }
}

void setup() {
  Serial.begin(115200);
  accel1.begin();
  accel2.begin();
  accel3.begin();
  mag1.begin();

  pwm.begin();
  pwm.setPWMFreq(50);
  for (int i = 0; i < servoCount; i++) {
    pwm.setPWM(i, 4096, 0);
  }

  calibrate();
}

void loop() {
  static int lastServoUpdate = 0;
  int t = millis();

  if (t - lastServoUpdate > 20) {
    lastServoUpdate = t;
    readSensors();
    calculateAngles();
    updateServos();
  }
  
  yield();
}