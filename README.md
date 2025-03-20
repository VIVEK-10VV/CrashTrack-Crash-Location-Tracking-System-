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

![usart2ioc2](https://github.com/user-attachments/assets/25ba0e05-6bd9-4aa8-8172-48238719122b)

usart1 is used as the communication gateway b/w gsm module and stm32

![usart1ioc](https://github.com/user-attachments/assets/4d511563-44ed-4b4b-b34d-379d805c3fc5)

![usart1ioc2](https://github.com/user-attachments/assets/1dcf80c8-1ab1-48fa-9b28-b693d5877a81)

usart3 is used to communicate between gps module and stm32

![usart3ioc](https://github.com/user-attachments/assets/be537c54-9ffd-47f7-bd05-2077d7b588f9)


![usart3ioc2](https://github.com/user-attachments/assets/acff8af3-d759-4e5b-9fb2-9167bf0e089a)


mpu6050 uses i2c1 to communicate with the stm32.sda,scl pins in the respected stm32 should be used

![i2cioc1](https://github.com/user-attachments/assets/f719c7a4-6e9e-4b47-a582-c1d1e150595d)


![i2cioc2](https://github.com/user-attachments/assets/fa788fa2-7929-431c-96cf-7851d6c8ae5a)


now to the program part . upload the main c file in user defined areas


output

![WhatsApp Image 2025-03-20 at 4 23 29 PM](https://github.com/user-attachments/assets/3f551dd8-099f-44e2-bd35-2884df8a31be)


![WhatsApp Image 2025-03-20 at 4 23 28 PM](https://github.com/user-attachments/assets/aae43a66-6562-4cb2-acd1-e650247d927e)


![WhatsApp Image 2025-03-20 at 4 23 28 PM (1)](https://github.com/user-attachments/assets/4f643c3a-46cf-4851-b349-bfa7b73a3402)
