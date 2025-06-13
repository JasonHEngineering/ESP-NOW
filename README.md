# ESP-NOW

Example 1
Devices:
![image](https://github.com/user-attachments/assets/16754321-6825-4c28-940d-f1b7e944de9f)
Master ESP32 device (1x ESP32-S3-DevKit-C1 N16R8)
Slave ESP32 device (2x my own custom PCBA with ESP32-C3, with integrated MPU-6050 gyro/accelerometer, and stepper motor driver DRV8825)

Data involved in wireless transfer:
Master send out angular data to slave devices
Slave devices process the data to drive stepper motors
Slave devices send out respective obtained gyro data to master device
Master device print out received gyro data on serial com

Example 2
Devices:
![image](https://github.com/user-attachments/assets/43fb3fd7-2e23-4590-87c1-1c8e5a140d04)

