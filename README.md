# 3D-Printed Mimic Robotic Arm

An ESP8266-powered 6-DOF robotic arm that mirrors human arm motion in real-time using IMU sensors, magnetometers, and PCA9685 PWM drivers. Developed as part of an undergraduate project at **National Institute of Technology Jamshedpur**.

## Hardware Architecture
* **Microcontroller:** Wemos D1 Mini (ESP8266)
* **Motion Tracking Sensors:** 
  * 2x GY-521 (MPU6050 Accelerometer/Gyroscope @ `0x68`, `0x69`)
  * 1x GY-511 (LSM303DLHC Accelerometer/Magnetometer)
  * 1x ACS715 / Linear Hall Effect Sensor (Gripper Position @ `A0`)
* **Actuators & Controller:**
  * PCA9685 16-Channel 12-Bit PWM Driver (`0x40`)
  * 3x MG996R Servos (High Torque Joints)
  * 3x MG990S Servos (Wrist and Gripper)

## Code Features
* **Matrix Kinematics:** Inverse spatial rotation transformations to isolate true magnetic compass yaw.
* **Signal Attenuation:** Exponential Moving Average (EMA) filtering to smooth motion jitter.
* **Watchdog Protection:** Embedded non-blocking yield loops specifically structured for ESP8266 board stability.

## License
MIT License