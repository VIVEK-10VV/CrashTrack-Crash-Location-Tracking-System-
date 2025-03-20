# CrashTrack-Crash-Location-Tracking-System-
The Vehicle Collision Alert System enhances road safety by detecting accidents using an STM32 microcontroller and an MPU6050 sensor. Upon collision detection, it utilizes GPS to pinpoint the vehicle's location and GSM to send immediate alerts to emergency responders and designated contacts, ensuring rapid assistance
This system uses a microcontroller,gyroscope and accelerometer,gsm module and a gps module.
The microcontroller used here is nucleo-f44re(stm32f44ret6) and mpu6050 is used to combine the need of gyroscope and oscilloscope


so here , we need to open the stm32ide and create a stm32 project.

Then we need to set the ioc file details.

first we need to configure the ioc file.
we need 3 uarts to send,receive and display data from stm32 and we use i2c to communicate with the mpu6050 module(gps and gsm modules require usart communication only).

In this microcontroller usart 2 is the default st-link display,so it is used for serial display

usart2 configuration

![usart2ioc](https://github.com/user-attachments/assets/5d5f8b28-a4eb-4cd7-a23f-09c2862e8fdc)

