/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define THRESHOLD_ACC 0.8        // Acceleration Threshold (g)
#define THRESHOLD_GYRO 3        // Gyroscope Threshold (°/s)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
uint8_t Rxdata[750];
char Txdata[750];
char GPS_Payyload[100];
uint8_t Flag=0;
static int Msgindex;
char *ptr;
float time,Latitude,Longitude;
char latDir, lonDir;
int Hours,Min,Sec;

char buffer[350];
uint8_t rec_data[6];
uint8_t Rec_Data[6];
int16_t X_acc;
int16_t Y_acc;
int16_t Z_acc;
int16_t Gyro_X;
int16_t Gyro_Y;
int16_t Gyro_Z;
float Ax,Ay,Az;
float Gx,Gy,Gz;


//uint8_t testBuffer[50]={"uoooooooooooo"};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void get_location (void);
void format_data(float Time, float Lat, char LatDir, float Long, char LonDir);
float convertDMtoDD(float value, char direction);
//void GSM_Init(void);
void GSM_SendATCommand(char *cmd);
void GSM_ReceiveResponse(char *buffer, uint16_t size);
void i2c_ready(void);
void accl_read(void);
void MPU6050_Read_Gyro (void);

//void format_data(float Time,float Lat,float Long);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t StartMSG[] = "the device is getting ready";
uint8_t EndMSG[] = "not connected";
uint8_t StartMSG1[] = "configuring gyroscope";
uint8_t EndMSG1[] = "failed to configure gyroscope";
uint8_t StartMSG2[] = "configuring accelerometer";
uint8_t EndMSG2[] = "failed to configure accelerometer";
uint8_t StartMSG3[] = "exiting power mode";
uint8_t EndMSG3[] = "unable to exit";
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
HAL_UART_Receive_DMA(&huart3, (uint8_t*)Rxdata, 700);
//HAL_UART_Transmit_DMA(&huart1,"hello", 5);
//GSM_Init();
  /* USER CODE END 2 */
HAL_Delay(100);
void i2c_ready(void){
 HAL_UART_Transmit(&huart2, StartMSG, sizeof(StartMSG), 10000);
 HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c1,(0b1101000<<1) + 0, 1,
 100);
 if(ret==HAL_OK){
 HAL_UART_Transmit(&huart2, StartMSG, sizeof(StartMSG), 10000);
 }
 else
 {
 HAL_UART_Transmit(&huart2, EndMSG, sizeof(StartMSG), 10000);
 }
 uint8_t temp = 8;
 HAL_StatusTypeDef set = HAL_I2C_Mem_Write(&hi2c1, (0b1101000<<1) + 0, 27,
 1,&temp,1,100);
 if(set == HAL_OK){
 HAL_UART_Transmit(&huart2, StartMSG1, sizeof(StartMSG1), 10000);
 }
 else{
 HAL_UART_Transmit(&huart2, EndMSG1, sizeof(StartMSG1), 10000);
 }
 uint8_t temp1 = 8;
 HAL_StatusTypeDef set1 = HAL_I2C_Mem_Write(&hi2c1, (0b1101000<<1) +0, 28,1,&temp1,1,100);
 if(set1 == HAL_OK){
 HAL_UART_Transmit(&huart2, StartMSG2, sizeof(StartMSG2), 10000);
 }
 else{
 HAL_UART_Transmit(&huart2, EndMSG2, sizeof(StartMSG2), 10000);
 }
 uint8_t temp2 = 0;
 HAL_StatusTypeDef set2 = HAL_I2C_Mem_Write(&hi2c1,
(0b1101000<<1) + 0, 107,1,&temp2,1,100);
 if(set2 == HAL_OK){
 HAL_UART_Transmit(&huart2, StartMSG3,sizeof(StartMSG3), 10000);
 }
 else{
 HAL_UART_Transmit(&huart2, EndMSG3,sizeof(StartMSG3), 10000);
 }
}
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  i2c_ready();
	  accl_read();
	  MPU6050_Read_Gyro();
	  float avg_acc = (abs(Ax) + abs(Ay) + abs(Az)) /2;
	  	  float avg_gyro = (fabs(Gx) + fabs(Gy) + fabs(Gz)) /2;

	  	  //if (avg_acc > THRESHOLD_ACC || avg_gyro > THRESHOLD_GYRO){
	  		  get_location();
	  		//format_data(float Time, float Lat, char LatDir, float Long, char LonDir);
	  		 //format_data();

	  	  //}
	  //HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	// GSM_Init();




  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA2_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    Flag = 1;
    HAL_UART_Receive_DMA(&huart3, (uint8_t*)Rxdata, 700);
    HAL_UART_Transmit(&huart3, (uint8_t*)"Data Received\n", 14, 100);
}

void get_location(void)
{
	 //char smsMessage[160];

    if (Flag == 1)
    {
        Msgindex = 0;

        strncpy(Txdata, (char*)Rxdata, sizeof(Txdata) - 1);
        Txdata[sizeof(Txdata) - 1] = '\0';

                if (strstr(Txdata, "$GPGGA")) {
                    HAL_UART_Transmit(&huart2, (uint8_t*)"GPGGA Message Found\n", 21, 100);
                } else {
                    HAL_UART_Transmit(&huart2, (uint8_t*)"No GPGGA Message\n", 18, 100);
                }
        HAL_UART_Transmit(&huart2, (uint8_t*)"Raw Data:\n", 10, 100);
               HAL_UART_Transmit(&huart2, (uint8_t*)Txdata, strlen(Txdata), 100);
                HAL_UART_Transmit(&huart2, (uint8_t*)"\n", 1, 100);

        ptr = strstr(Txdata, "GPGGA");
        if (ptr != NULL)
        {
            ptr += 6; // Move pointer past "GPRMC"

            char *endPtr = strchr(ptr, '\n');
                        if (endPtr)
                        {
                            size_t len = endPtr - ptr;
                            if (len > sizeof(GPS_Payyload) - 1) len = sizeof(GPS_Payyload) - 1;
                            strncpy(GPS_Payyload, ptr, len);
                            GPS_Payyload[len] = '\0';
                        }
                        HAL_UART_Transmit(&huart2, (uint8_t*)"GPS_Payyload:\n", 14, 100);
                                   HAL_UART_Transmit(&huart2, (uint8_t*)GPS_Payyload, strlen(GPS_Payyload), 100);
                                   HAL_UART_Transmit(&huart2, (uint8_t*)"\n", 1, 100);



       if (strstr(GPS_Payyload, ",N,") != NULL){
            // Extract Time, Latitude, Longitude
           // if  == 3)
    	   int parsed =  sscanf(GPS_Payyload, "%f,%f,%c,%f,%c", &time, &Latitude, &latDir, &Longitude, &lonDir);

    	   if (parsed ==5 ) {
    	       HAL_UART_Transmit(&huart2, (uint8_t*)"Parsing Success!\n", 17, 100);
    	       format_data(time, Latitude, latDir, Longitude, lonDir);

    	   } else {
    	       HAL_UART_Transmit(&huart2, (uint8_t*)"Parsing Failed!\n", 16, 100);
    	   }


       }
            HAL_Delay(1);
            Flag = 0;
        }

    }
    HAL_UART_Receive_DMA(&huart3, (uint8_t*)Rxdata, 700);
}
float convertDMtoDD(float value, char direction) {
    int degrees = (int)(value / 100);   // Extract degrees
    float minutes = value - (degrees * 100); // Extract minutes
    float decimalDegrees = degrees + (minutes / 60);

    // Apply direction correction
    if (direction == 'S' || direction == 'W') {
        decimalDegrees = -decimalDegrees;
    }

    return decimalDegrees;
}

void GSM_SendATCommand(char *cmd) {
    HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);  // Debug to USART2
    HAL_Delay(500);
}

void GSM_ReceiveResponse(char *buffer, uint16_t size) {
    memset(buffer, 0, size);  // Clear buffer before receiving
    if (HAL_UART_Receive(&huart1, (uint8_t*)buffer, size, 5000) == HAL_OK) {
        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
        // Debug output
    } else {
        HAL_UART_Transmit(&huart2, (uint8_t*)"No Response!\r\n", 14, HAL_MAX_DELAY);
    }
}
void accl_read(void){




HAL_I2C_Mem_Read(&hi2c1, (0b1101000<<1) + 1,0x3B,1, rec_data, 6, 100);
X_acc=(int16_t)(rec_data[0] << 8 | rec_data [1]);
Y_acc=(int16_t)(rec_data[2] << 8 | rec_data [3]);
Z_acc=(int16_t)(rec_data[4] << 8 | rec_data [5]);
        Ax = X_acc/ 16384.0;  // Assuming ±2g sensitivity
        Ay = Y_acc / 16384.0;
        Az = Z_acc / 16384.0;





// Format the data into a string using sprintf
 sprintf(buffer, "X_acc =%.5f\r\n",Ax);
 // Send the string through USART
 HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 1000);
// Use 1000 ms as timeout
 HAL_Delay(1000); // Introduce a delay of 1 second
 sprintf(buffer, "Y_acc = %.5f\r\n",Ay);
 // Send the string through USART
 HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 1000);
// Use 1000 ms as timeout
 HAL_Delay(1000); // Introduce a delay of 1 second
 sprintf(buffer, "Z_acc = %.5f\r\n",Az);
 // Send the string through USART
 HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer),
1000); // Use 1000 ms as timeout
 HAL_Delay(1000); // Introduce a delay of 1 second
}
void MPU6050_Read_Gyro (void)
{


// Read 6 BYTES of data starting from GYRO_XOUT_H register
HAL_I2C_Mem_Read (&hi2c1, (0b1101000<<1) + 1, 0x43, 1, Rec_Data, 6,
1000);
Gyro_X = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
Gyro_Y = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
Gyro_Z = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

        Gx = Gyro_X / 131.0;      // Assuming ±250°/s sensitivity
        Gy = Gyro_Y / 131.0;
        Gz = Gyro_Z / 131.0;

sprintf(buffer, "Gyro_X = %.5f\r\n",Gx);
 // Send the string through
 HAL_UART_Transmit(&huart2,(uint8_t*)buffer, strlen(buffer), 1000); // Use 1000 ms as timeout
 HAL_Delay(1000);
 sprintf(buffer, "Gyro_Y = %.5f\r\n",Gy);
 //Send the string through USART

HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 1000); // Use 1000ms as timeout

HAL_Delay(1000);
 sprintf(buffer, "Gyro_Z =%.5f\r\n",Gz);

 // Send the string through USART

 HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 1000); //Use 1000 ms as timeout

 HAL_Delay(1000);

}

//void GSM_Init(void)
//{
//    uint8_t response[20];
//
//
//
//
//    // Send AT command to check module response
//    HAL_UART_Transmit(&huart1, (uint8_t*)"AT\r\n", 4, 1000);
//    HAL_Delay(2000);
//    HAL_UART_Receive(&huart1, response,strlen((char*)response) , 2000);
//    HAL_UART_Transmit(&huart2, response, strlen((char*)response), 1000);  // Debug output
//
//    if (strstr((char*)response, "OK") != NULL) {
//        HAL_UART_Transmit(&huart2, (uint8_t*)"GSM Ready\n", 10, 1000);
//    } else {
//        HAL_UART_Transmit(&huart2, (uint8_t*)"GSM Init Failed\n", 17, 1000);
//        return;
//    }
//
//    // Set SMS text mode
//    HAL_UART_Transmit(&huart1, (uint8_t*)"AT+CMGF=1\r\n", 11, 1000);
//    HAL_Delay(1000);
//    HAL_UART_Receive(&huart1, response, sizeof(response), 2000);
//    HAL_UART_Transmit(&huart2, response, strlen((char*)response), 1000);
//
//    if (strstr((char*)response, "OK") != NULL) {
//        HAL_UART_Transmit(&huart2, (uint8_t*)"SMS Mode Set\n", 13, 1000);
//    } else {
//        HAL_UART_Transmit(&huart2, (uint8_t*)"SMS Mode Failed\n", 16, 1000);
//    }
//}


void format_data(float Time, float Lat, char LatDir, float Long, char LonDir)
{
    char Data[100];
    char response[100];
    char smsMessage[160];
    Hours = (int)Time / 10000;
    Min = ((int)Time % 10000) / 100;
    Sec = ((int)Time % 100);

    // Convert UTC to IST
    Hours += 5;
    Min += 30;

    if (Min >= 60) {
        Min -= 60;
        Hours += 1;
    }
    if (Hours >= 24) {
        Hours -= 24; // Adjust for next day overflow
    }

    float lat_dd = convertDMtoDD(Latitude, latDir);
    float lon_dd = convertDMtoDD(Longitude, lonDir);


    sprintf(Data, "IST Time-%02d:%02d:%02d Lat-%f, Long-%f", Hours, Min, Sec, lat_dd, lon_dd);
    HAL_UART_Transmit(&huart2, (uint8_t*) Data, strlen(Data), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n\n", 3, HAL_MAX_DELAY);


    sprintf(smsMessage,"Accident detected! Location: https://maps.google.com/?q=%.6f,%.6f",
                convertDMtoDD(Latitude, latDir), convertDMtoDD(Longitude, lonDir));

        // Send AT commands to GSM module via UART
//        HAL_UART_Transmit(&huart3, (uint8_t*)"AT\r\n", 4, 1000); // Check GSM module
//        HAL_Delay(1000);
//
//        HAL_UART_Transmit(&huart3, (uint8_t*)"AT+CMGF=1\r\n", 11, 1000); // Set SMS mode to text
//        HAL_Delay(1000);

//        HAL_UART_Transmit(&huart1, (uint8_t*)"AT+CMGS=\"+916238310278\"\r\n", 24, 1000); // Replace with your phone number
//        HAL_Delay(1000);
//
//        HAL_UART_Transmit(&huart1, (uint8_t*)smsMessage, strlen(smsMessage), 1000); // Send message
//        HAL_Delay(1000);
//
//        uint8_t ctrlZ = 26; // ASCII for Ctrl+Z (End SMS)
//        HAL_UART_Transmit(&huart1, &ctrlZ, 1, 1000);
//        HAL_Delay(1000);
    GSM_SendATCommand("AT+CMGF=1\r\n");
    GSM_ReceiveResponse(response, sizeof(response));
    HAL_UART_Transmit(&huart2, (uint8_t*)response, strlen(response), 1000);
    HAL_Delay(1000);

    // Step 6: Set recipient number
    GSM_SendATCommand("AT+CMGS=\"9562928527\"\r\n");
    HAL_Delay(2000);  // Ensure SIM800A is ready for message

    // Step 7: Send GPS location as an SMS
    GSM_SendATCommand(smsMessage);
    GSM_SendATCommand("\x1A\r\n");  // Ctrl+Z to end SMS
    GSM_ReceiveResponse(response, sizeof(response));
    HAL_UART_Transmit(&huart2, (uint8_t*)response, strlen(response), 1000);

}




/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
