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
![image](https://github.com/user-attachments/assets/bde1adad-085c-48d7-8af5-618fb9222117)
Master ESP32 device (1x LILYGO T-Display-S3, connected to BNO055)
Slave ESP32 device (2x my own custom PCBA with ESP32-C3, with integrated stepper motor driver DRV8825)

Data involved in wireless transfer:
Master send out IMU data to slave devices, specifically Y and Z Euler angles measured from the BNO055 sensor to respective slave devices
Slave device 1 drive stepper motor according to input Y angle while slave device 2 drive stepper motor according to input Z angle

