/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdlib.h>
#include <string.h>
#include "robot.pb.h"
#include "pb_encode.h"
#include <math.h>
#include "cJSON.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR (0x68 << 1)
#define AS5600_I2C_ADDR (0x36 << 1)
#define WHEEL_RADIUS  0.03  // 3 cm
#define WHEEL_BASE    0.175  // 15 cm
#define TICKS_PER_REV 4096  // AS5600 resolution
#define ACCEL_SCALE 16384.0 // 16384 LSB/g for ±2g
#define GYRO_SCALE  131.0   // 131 LSB/(°/s) for ±250°/s
#define TEMP_SCALE  340.0
#define TEMP_OFFSET 36.53
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t low_bytes_angle_data = 0;
uint8_t high_bytes_angle_data = 0;
volatile uint16_t raw_angle;

uint8_t low_bytes_angle_data_2 = 0;
uint8_t high_bytes_angle_data_2 = 0;
volatile uint16_t raw_angle_2;

uint8_t sign;
uint8_t receivedSign;
uint8_t messageCursor = 0;
uint8_t frameCursor = 0;
char messageReceived[50];
uint64_t frameReceived[10];
float sensorsValues[5];
char msg[100];

float x_pos = 0.0, y_pos = 0.0, theta = 0.0;
uint16_t prev_angle_L = 0, prev_angle_R = 0;

int16_t Accel[3], Gyro[3], Temp;
float Accel_f[3], Gyro_f[3], Temp_f;

enum Communication{
	EMERGENCY_STOP,
	MOVE_FORWARD,
	MOVE_REVERSE,
	TURN_LEFT,
	TURN_RIGHT,
	SET_MOVMENT_SPEED,
	GET_ROBOT_DATA = 7,
	SET_LED_BRIGHTNESS

};



//typedef struct {
//    int sensorValue1;
//    int sensorValue2;
//    int sensorValue3;
//    int sensorValue4;
//    int sensorValue5;
//    float xPos;
//    float yPos;
//    float theta;
//} RobotData;

void setMovmentSpeed(int motorA,int motorB) {
	uint8_t motorAOutput = motorA;
	uint8_t motorBOutput = motorB;
	if(motorA<=100)HAL_GPIO_WritePin(MOTOR_A_DIRECTION_GPIO_Port,MOTOR_A_DIRECTION_Pin , GPIO_PIN_RESET);
	else
	{
		HAL_GPIO_WritePin(MOTOR_A_DIRECTION_GPIO_Port,MOTOR_A_DIRECTION_Pin , GPIO_PIN_SET);
		motorAOutput = motorAOutput - 100;
	}

	if(motorB<=100)HAL_GPIO_WritePin(MOTOR_B_DIRECTION_GPIO_Port,MOTOR_B_DIRECTION_Pin , GPIO_PIN_RESET);
	else
	{
		HAL_GPIO_WritePin(MOTOR_B_DIRECTION_GPIO_Port,MOTOR_B_DIRECTION_Pin , GPIO_PIN_SET);
		motorBOutput = motorBOutput - 100;
	}

	if(motorA!=0){
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,motorAOutput);
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
	}

	if(motorB!=0){
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,motorBOutput);
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,0);
	}




}
void emergencyStop() {
	HAL_GPIO_WritePin(MOTOR_A_DIRECTION_GPIO_Port,MOTOR_A_DIRECTION_Pin , GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_B_DIRECTION_GPIO_Port,MOTOR_B_DIRECTION_Pin , GPIO_PIN_RESET);


	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,0);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
}

void moveForward() {
	HAL_GPIO_WritePin(MOTOR_A_DIRECTION_GPIO_Port,MOTOR_A_DIRECTION_Pin , GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_B_DIRECTION_GPIO_Port,MOTOR_B_DIRECTION_Pin , GPIO_PIN_RESET);

	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,100);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,100);
}

void moveReverse() {
	HAL_GPIO_WritePin(MOTOR_A_DIRECTION_GPIO_Port,MOTOR_A_DIRECTION_Pin , GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_B_DIRECTION_GPIO_Port,MOTOR_B_DIRECTION_Pin , GPIO_PIN_SET);

	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,100);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,100);
}

void turnRight() {
	HAL_GPIO_WritePin(MOTOR_A_DIRECTION_GPIO_Port,MOTOR_A_DIRECTION_Pin , GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_B_DIRECTION_GPIO_Port,MOTOR_B_DIRECTION_Pin , GPIO_PIN_RESET);

	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,100);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,100);
}
void turnLeft() {
	HAL_GPIO_WritePin(MOTOR_A_DIRECTION_GPIO_Port,MOTOR_A_DIRECTION_Pin , GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_B_DIRECTION_GPIO_Port,MOTOR_B_DIRECTION_Pin , GPIO_PIN_SET);

	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,100);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,100);
}
void getEncoderValue(){
	HAL_I2C_Mem_Read(&hi2c3, AS5600_I2C_ADDR, 0x0D, 1, &low_bytes_angle_data, sizeof(low_bytes_angle_data), HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c3, AS5600_I2C_ADDR, 0x0C, 1, &high_bytes_angle_data, sizeof(high_bytes_angle_data), HAL_MAX_DELAY);
	raw_angle = ((high_bytes_angle_data<< 8) | low_bytes_angle_data) & 0x0FFF;
	HAL_I2C_Mem_Read(&hi2c1, AS5600_I2C_ADDR, 0x0D, 1, &low_bytes_angle_data_2, sizeof(low_bytes_angle_data_2), HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, AS5600_I2C_ADDR, 0x0C, 1, &high_bytes_angle_data_2, sizeof(high_bytes_angle_data_2), HAL_MAX_DELAY);
	raw_angle_2 = ((high_bytes_angle_data_2<< 8) | low_bytes_angle_data_2) & 0x0FFF;
}


void MPU6050_Init() {
    uint8_t check, data;

    // Check if MPU6050 is connected
    HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, 0x75, 1, &check, 1, 1000);
    if (check != 0x68) {
        // MPU6050 not found, handle error
        return;
    }

    // Wake up MPU6050 (Write 0 to Power Management Register)
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, 0x6B, 1, &data, 1, 1000);

    // Set accelerometer configuration (±2g)
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, 0x1C, 1, &data, 1, 1000);

    // Set gyroscope configuration (±250°/s)
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, 0x1B, 1, &data, 1, 1000);
}

void MPU6050_Read_All(int16_t *Accel, int16_t *Gyro, int16_t *Temp) {
    uint8_t data[14];

    // Read 14 bytes from MPU6050 starting at ACCEL_XOUT_H
    HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, 0x3B, 1, data, 14, 1000);

    // Accelerometer values
    Accel[0] = (int16_t)(data[0] << 8 | data[1]);  // X
    Accel[1] = (int16_t)(data[2] << 8 | data[3]);  // Y
    Accel[2] = (int16_t)(data[4] << 8 | data[5]);  // Z

    // Temperature value
    *Temp = (int16_t)(data[6] << 8 | data[7]);

    // Gyroscope values
    Gyro[0] = (int16_t)(data[8] << 8 | data[9]);   // X
    Gyro[1] = (int16_t)(data[10] << 8 | data[11]); // Y
    Gyro[2] = (int16_t)(data[12] << 8 | data[13]); // Z
}

void getSensorValues(){
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	sensorsValues[0] = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	sensorsValues[1] = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	sensorsValues[2] = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	sensorsValues[3] = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	sensorsValues[4] = HAL_ADC_GetValue(&hadc1);
//	sprintf(msg, "%f,%f,%f,%f,%f\r\n",sensorsValues[0], sensorsValues[1], sensorsValues[2], sensorsValues[3], sensorsValues[4]);
//	HAL_UART_Transmit_IT(&huart2, (uint8_t*)msg , strlen(msg));

}
void setLedBrightness(int brightness) {
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,brightness);
}

float computeDeltaAngle(uint16_t new_angle, uint16_t prev_angle) {
    int16_t delta = new_angle - prev_angle;

    // Handle encoder overflow (angle wrapping)
    if (delta > (TICKS_PER_REV / 2)) {
        delta -= TICKS_PER_REV;  // Counterclockwise overflow
    } else if (delta < -(TICKS_PER_REV / 2)) {
        delta += TICKS_PER_REV;  // Clockwise overflow
    }

    // Convert to radians
    return (delta * 2.0f * M_PI) / TICKS_PER_REV;
}

void updateRobotPosition() {
    // Read new encoder values
    getEncoderValue();

    // Compute angle changes
    float delta_angle_L = computeDeltaAngle(raw_angle, prev_angle_L);
    float delta_angle_R = computeDeltaAngle(raw_angle_2, prev_angle_R);

    // Convert to linear displacement
    float d_L = delta_angle_L * WHEEL_RADIUS;
    float d_R = delta_angle_R * WHEEL_RADIUS;
    float d_C = (d_L + d_R) / 2.0;  // Center displacement
    float d_theta = (d_R - d_L) / WHEEL_BASE;  // Change in orientation

    // Update position
    x_pos += d_C * cos(theta);
    y_pos += d_C * sin(theta);
    theta += d_theta;

    // Keep theta in range [-π, π]
    if (theta > M_PI) {
        theta -= 2.0 * M_PI;
    } else if (theta < -M_PI) {
        theta += 2.0 * M_PI;
    }

    // Store previous angles
    prev_angle_L = raw_angle;
    prev_angle_R = raw_angle_2;
}

void sendRobotData() {
	cJSON *jsonObj = cJSON_CreateObject();

	cJSON_AddNumberToObject(jsonObj, "sensorValue1", sensorsValues[0]);
	cJSON_AddNumberToObject(jsonObj, "sensorValue2", sensorsValues[1]);
	cJSON_AddNumberToObject(jsonObj, "sensorValue3", sensorsValues[2]);
	cJSON_AddNumberToObject(jsonObj, "sensorValue4", sensorsValues[3]);
	cJSON_AddNumberToObject(jsonObj, "sensorValue5", sensorsValues[4]);
	cJSON_AddNumberToObject(jsonObj, "xPos", x_pos);
	cJSON_AddNumberToObject(jsonObj, "yPos", y_pos);
	cJSON_AddNumberToObject(jsonObj, "theta", theta);
	cJSON_AddNumberToObject(jsonObj, "rawAngle1", raw_angle);
	cJSON_AddNumberToObject(jsonObj, "rawAngle2", raw_angle_2);
	cJSON_AddNumberToObject(jsonObj, "xGyro", Gyro_f[0]);
	cJSON_AddNumberToObject(jsonObj, "yGyro", Gyro_f[1]);
	cJSON_AddNumberToObject(jsonObj, "zGyro", Gyro_f[2]);

	cJSON_AddNumberToObject(jsonObj, "xAccel", Accel_f[0]);
	cJSON_AddNumberToObject(jsonObj, "yAccel", Accel_f[1]);
	cJSON_AddNumberToObject(jsonObj, "zAccel", Accel_f[2]);

	cJSON_AddNumberToObject(jsonObj, "Temp", Temp_f);



	    // Convert JSON object to string
	char *jsonString = cJSON_PrintUnformatted(jsonObj);

	// Send JSON string via UART (or any other communication interface)
	HAL_UART_Transmit_IT(&huart2, (uint8_t*)jsonString, strlen(jsonString));

	// Free the JSON object after use
	cJSON_Delete(jsonObj);
}

void Convert_To_Units(int16_t *Accel, int16_t *Gyro, int16_t Temp, float *Accel_f, float *Gyro_f, float *Temp_f) {
    for (int i = 0; i < 3; i++) {
        Accel_f[i] = Accel[i] / ACCEL_SCALE; // g
        Gyro_f[i]  = Gyro[i]  / GYRO_SCALE;  // °/s
    }
    *Temp_f = (Temp / TEMP_SCALE) + TEMP_OFFSET; // °C
}

void Encoders_Init() {
    prev_angle_L = raw_angle;
    prev_angle_R = raw_angle_2;
}

//void sendRobotData2() {
//    uint8_t buffer[128];  // Increased buffer size for more data
//    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
//
//    // Fill Protobuf message
//    RobotData pos = {
//        .sensor_value_1 = sensorsValues[0],
//        .sensor_value_2 = sensorsValues[1],
//        .sensor_value_3 = sensorsValues[2],
//        .sensor_value_4 = sensorsValues[3],
//        .sensor_value_5 = sensorsValues[4],
//        .raw_angle_1 = raw_angle,  // Read from AS5600
//        .raw_angle_2 = raw_angle_2,
//        .x_pos = x_pos,  // Computed position
//        .y_pos = y_pos,
//        .theta = theta
//    };
//
//    // Serialize the data
//    if (!pb_encode(&stream, RobotData_fields, &pos)) {
//        printf("Encoding failed!\n");
//        return;
//    }
//
//    // Send serialized data over UART
//    HAL_UART_Transmit_IT(&huart2, buffer, stream.bytes_written);
//}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &receivedSign, 1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  MPU6050_Init();

  Encoders_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  MPU6050_Read_All(Accel, Gyro, &Temp);
	  Convert_To_Units(Accel, Gyro, Temp, Accel_f, Gyro_f, &Temp_f);
	  updateRobotPosition();
	  getSensorValues();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10D19CE4;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x10D19CE4;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 39;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 39;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 39;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 99;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|MOTOR_B_DIRECTION_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MOTOR_A_DIRECTION_GPIO_Port, MOTOR_A_DIRECTION_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin MOTOR_B_DIRECTION_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|MOTOR_B_DIRECTION_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MOTOR_A_DIRECTION_Pin */
  GPIO_InitStruct.Pin = MOTOR_A_DIRECTION_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTOR_A_DIRECTION_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart -> Instance == USART2){
		if(receivedSign == '\n'){
			frameReceived[frameCursor] = atoi(messageReceived);
			if(strcmp(frameReceived[0], EMERGENCY_STOP ) == 0) emergencyStop();
			else if(strcmp(frameReceived[0], MOVE_FORWARD) == 0)moveForward();
			else if(strcmp(frameReceived[0], MOVE_REVERSE) == 0)moveReverse();
			else if(strcmp(frameReceived[0], TURN_LEFT) == 0)turnLeft();
			else if(strcmp(frameReceived[0], TURN_RIGHT) == 0)turnRight();
			else if(strcmp(frameReceived[0], SET_MOVMENT_SPEED) == 0)setMovmentSpeed(frameReceived[1], frameReceived[2]);
			else if(strcmp(frameReceived[0], SET_LED_BRIGHTNESS) == 0)setLedBrightness(frameReceived[1]);
			else if(strcmp(frameReceived[0], GET_ROBOT_DATA) == 0){sendRobotData();}
			memset(messageReceived, 0, 50);
			memset(frameReceived, 0, 10);
			messageCursor = 0;
			frameCursor = 0;
		}
		else if(receivedSign == ' '){
			frameReceived[frameCursor++] = atoi(messageReceived);
			messageCursor = 0;
			memset(messageReceived, 0, 50);
		}
		else messageReceived[messageCursor++] = (char)receivedSign;
	HAL_UART_Receive_IT(&huart2, &receivedSign, 1);
	}
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
