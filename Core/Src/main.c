/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "crc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
    uint8_t packageIndex;
    uint16_t fanSpeed;
    uint16_t targetSpeed;
    uint32_t crcresult;
    uint32_t timestamp;
    uint16_t duty;
    float_t P;
    float_t I;
    float_t D;
}systemData_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define TX_BUFF_SIZE 12 //uart send data bytes
#define TX_BUFF_DEBUG_SIZE 30 // debug uart send data bytes
#define UART_PERIOD 20 //uart send data period ms
#define OLED_PERIOD 50 //spi oled send data period ms
#define PWM_MAX 2879
#define PWM_MIN 0
#define FANSPEED_MAX 5000 //record
#define INTEGRAL_MAX 3548000 //error
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern CRC_HandleTypeDef hcrc;
// now TIM2 frequency is 25kHz, counter period 2880
//uint16_t duty_value = 287;

uint16_t ccr_reg_01 = 0;
uint16_t ccr_reg_02 = 0;
uint16_t dif_val = 0;
uint16_t fanspeed = 0;
uint32_t freq_tim4 = 1e6;// TIM4 frequency is 1MHz
systemData_t dataField = {
    .packageIndex = 0,
    .targetSpeed = 3000,  //target speed
    .timestamp = 0,
    .duty = 287,
    .P = 0,
    .I = 0,
    .D = 0
};
uint32_t primask_bit;
//uint8_t g_tx_buffer[TX_BUFF_SIZE];
uint8_t g_tx_buffer[TX_BUFF_DEBUG_SIZE];
uint32_t last_send_time = 0;
HAL_StatusTypeDef status;
uint16_t fanspeed_stored[5] = {0};
uint16_t speed_count = 0;
uint8_t i = 0;
float_t fanspeed_sum = 0;
float_t fanspeed_filter = 0;
uint16_t fanspeed_index = 0;
float_t kp = 0;
float_t ki = 0;
float_t kd = 0;
float_t error_fanspeed = 0;//0 1 2 present e(k-2) e(k-1) e(k)
float_t delta_pwm = 0;
float_t duty_value_new = 0;
float_t error_fanspeed_sum = 0;
float_t ts = 0.02; // tim3 period, influence PID parameter
uint32_t wait_time = 0;
uint32_t raw_int,big_endian_int;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void DataPackAndSend(void);
void DataPackAndSendDebug(void);
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
    kp = 4.5;// 4.3
    ki = 3.6;//14 or 15
    kd = 15;
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_CRC_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // start TIM2 PWM channel
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  // start TIM3
  HAL_TIM_Base_Start_IT(&htim3);
  // start TIM4, monitor fan speed
  HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_1);
  HAL_GPIO_WritePin(GPIOF,GPIO_PIN_7,GPIO_PIN_RESET);
  last_send_time = HAL_GetTick();
  wait_time = last_send_time;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      if(HAL_GetTick() - last_send_time >= UART_PERIOD)
      {
          dataField.packageIndex++;
          last_send_time = HAL_GetTick();
          // use PF7 to measure function time
          HAL_GPIO_WritePin(GPIOF,GPIO_PIN_7,GPIO_PIN_SET);
          //DataPackAndSend();
          DataPackAndSendDebug();
          HAL_GPIO_WritePin(GPIOF,GPIO_PIN_7,GPIO_PIN_RESET);
      }
      /*
      if(HAL_GetTick() - wait_time >= 8000)
      {
          dataField.targetSpeed = 3000;
      }*/
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// TIM3 interrupt function
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM3)
    {
        // moving average fan speed
        /*
        if(speed_count < 5)
        {
            fanspeed_stored[speed_count] = dataField.fanSpeed;//risk? access the same time?
            speed_count++;
            fanspeed_sum = 0;
            for(i = 0;i < speed_count;i++)
            {
                fanspeed_sum += fanspeed_stored[i];
            }
            fanspeed_filter = fanspeed_sum / speed_count;
        }
        else
        {
            fanspeed_stored[fanspeed_index % 5] = dataField.fanSpeed;
            fanspeed_index++;
            fanspeed_sum = 0;
            for(i = 0;i < 5;i++)
            {
                fanspeed_sum += fanspeed_stored[i];
            }
            fanspeed_filter = fanspeed_sum / 5;
        }*/
        //PID algorithm,modify TIM2 PWM duty
        //error_fanspeed = (float_t)dataField.targetSpeed - fanspeed_filter;
        error_fanspeed = (float_t)dataField.targetSpeed - (float_t)dataField.fanSpeed;
        if(fabsf(error_fanspeed) < 800.0f)
        {
            error_fanspeed_sum += error_fanspeed;
            if(error_fanspeed_sum > INTEGRAL_MAX)
                error_fanspeed_sum = INTEGRAL_MAX;
            if(error_fanspeed_sum < -INTEGRAL_MAX)
                error_fanspeed_sum = -INTEGRAL_MAX;
        }
        else
        {
            error_fanspeed_sum = 0.0f;
        }
        dataField.P = kp * error_fanspeed;
        dataField.I = ki * error_fanspeed_sum * ts;
        duty_value_new = dataField.P;
        //duty_value_new = dataField.P + dataField.I;
        if(duty_value_new < PWM_MIN)
        {
            duty_value_new = PWM_MIN;
        }
        else if(duty_value_new > PWM_MAX)
        {
            duty_value_new = PWM_MAX;
        }
        dataField.duty = (uint16_t)duty_value_new;
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,dataField.duty);// change TIM2 PWM duty
    }
}
// TIM4 interrupt function
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM4)
    {
        ccr_reg_02 = (uint16_t)HAL_TIM_ReadCapturedValue(&htim4,TIM_CHANNEL_1);
        dif_val = ccr_reg_02 - ccr_reg_01;
        if(dif_val < 1000)
            return;
        dataField.fanSpeed = (uint16_t)round(30 * freq_tim4 /dif_val);
        ccr_reg_01 = ccr_reg_02;
    }
}
// data package and send function
void DataPackAndSend(void)
{
    systemData_t localData;
    uint32_t crcdata[2];
    // close interrupt
    primask_bit = __get_PRIMASK();
    __disable_irq();
    localData = dataField;
    //open interrupt
    __set_PRIMASK(primask_bit);
    //data package
    g_tx_buffer[0] = 0x55;//frame header
    g_tx_buffer[1] = 0xAA;//frame header
    g_tx_buffer[2] = 10;//frame length except header
    g_tx_buffer[3] = localData.packageIndex;//
    g_tx_buffer[4] = (localData.fanSpeed >> 8) & 0xFF;
    g_tx_buffer[5] = localData.fanSpeed & 0xFF;
    g_tx_buffer[6] = (localData.targetSpeed >> 8) & 0xFF;
    g_tx_buffer[7] = localData.targetSpeed & 0xFF;
    //calculate crc
    crcdata[0] = (g_tx_buffer[0] << 24) | (g_tx_buffer[1] << 16) | (g_tx_buffer[2] << 8) | g_tx_buffer[3];
    crcdata[1] = (g_tx_buffer[4] << 24) | (g_tx_buffer[5] << 16) | (g_tx_buffer[6] << 8) | g_tx_buffer[7];
    localData.crcresult = HAL_CRC_Calculate(&hcrc,crcdata,2);
    g_tx_buffer[8] = (localData.crcresult >> 24) & 0xFF;
    g_tx_buffer[9] = (localData.crcresult >> 16) & 0xFF;
    g_tx_buffer[10] = (localData.crcresult >> 8) & 0xFF;
    g_tx_buffer[11] = localData.crcresult & 0xFF;
    //uart send g_tx_buffer
    HAL_UART_Transmit_DMA(&huart1,g_tx_buffer,TX_BUFF_SIZE);
}

void DataPackAndSendDebug(void)
{
    systemData_t localData;
    uint32_t crcdata[7] = {0};
    // close interrupt
    primask_bit = __get_PRIMASK();
    __disable_irq();
    localData = dataField;
    //open interrupt
    __set_PRIMASK(primask_bit);
    //data package
    g_tx_buffer[0] = 0x55;//frame header
    g_tx_buffer[1] = 0xAA;//frame header
    g_tx_buffer[2] = TX_BUFF_DEBUG_SIZE - 2;//frame length except header
    g_tx_buffer[3] = localData.packageIndex;//
    g_tx_buffer[4] = (localData.fanSpeed >> 8) & 0xFF;
    g_tx_buffer[5] = localData.fanSpeed & 0xFF;
    g_tx_buffer[6] = (localData.targetSpeed >> 8) & 0xFF;
    g_tx_buffer[7] = localData.targetSpeed & 0xFF;
    g_tx_buffer[8] = (localData.duty >> 8) & 0xFF;
    g_tx_buffer[9] = localData.duty & 0xFF;
    // change to big-endian
    memcpy(&raw_int, &localData.P, sizeof(float));
    big_endian_int = __REV(raw_int);
    memcpy(&g_tx_buffer[10], &big_endian_int, sizeof(float));
    //g_tx_buffer[10] = (localData.P >> 24) & 0xFF;
    //g_tx_buffer[11] = (localData.P >> 16) & 0xFF;
    //g_tx_buffer[12] = (localData.P >> 8) & 0xFF;
    //g_tx_buffer[13] = localData.P & 0xFF;
    memcpy(&raw_int, &localData.I, sizeof(float));
    big_endian_int = __REV(raw_int);
    memcpy(&g_tx_buffer[14], &big_endian_int, sizeof(float));
    //g_tx_buffer[14] = (localData.I >> 24) & 0xFF;
    //g_tx_buffer[15] = (localData.I >> 16) & 0xFF;
    //g_tx_buffer[16] = (localData.I >> 8) & 0xFF;
    //g_tx_buffer[17] = localData.I & 0xFF;
    memcpy(&raw_int, &localData.D, sizeof(float));
    big_endian_int = __REV(raw_int);
    memcpy(&g_tx_buffer[18], &big_endian_int, sizeof(float));
    //g_tx_buffer[18] = (localData.D >> 24) & 0xFF;
    //g_tx_buffer[19] = (localData.D >> 16) & 0xFF;
    //g_tx_buffer[20] = (localData.D >> 8) & 0xFF;
    //g_tx_buffer[21] = localData.D & 0xFF;
    localData.timestamp = HAL_GetTick();
    memcpy(&raw_int, &localData.timestamp, sizeof(float));
    big_endian_int = __REV(raw_int);
    memcpy(&g_tx_buffer[22], &big_endian_int, sizeof(float));
    //g_tx_buffer[22] = (localData.timestamp >> 24 ) & 0xFF;
    //g_tx_buffer[23] = (localData.timestamp >> 16 ) & 0xFF;
    //g_tx_buffer[24] = (localData.timestamp >> 8 ) & 0xFF;
    //g_tx_buffer[25] = localData.timestamp & 0xFF;
    //calculate crc
    crcdata[0] = (g_tx_buffer[0] << 24) | (g_tx_buffer[1] << 16) | (g_tx_buffer[2] << 8) | g_tx_buffer[3];
    crcdata[1] = (g_tx_buffer[4] << 24) | (g_tx_buffer[5] << 16) | (g_tx_buffer[6] << 8) | g_tx_buffer[7];
    crcdata[2] = (g_tx_buffer[8] << 24) | (g_tx_buffer[9] << 16) | (g_tx_buffer[10] << 8) | g_tx_buffer[11];
    crcdata[3] = (g_tx_buffer[12] << 24) | (g_tx_buffer[13] << 16) | (g_tx_buffer[14] << 8) | g_tx_buffer[15];
    crcdata[4] = (g_tx_buffer[16] << 24) | (g_tx_buffer[17] << 16) | (g_tx_buffer[18] << 8) | g_tx_buffer[19];
    crcdata[5] = (g_tx_buffer[20] << 24) | (g_tx_buffer[21] << 16) | (g_tx_buffer[22] << 8) | g_tx_buffer[23];
    crcdata[6] = (g_tx_buffer[24] << 24) | (g_tx_buffer[25] << 16) | 0x0000;
    localData.crcresult = HAL_CRC_Calculate(&hcrc,crcdata,7);
    g_tx_buffer[26] = (localData.crcresult >> 24) & 0xFF;
    g_tx_buffer[27] = (localData.crcresult >> 16) & 0xFF;
    g_tx_buffer[28] = (localData.crcresult >> 8) & 0xFF;
    g_tx_buffer[29] = localData.crcresult & 0xFF;
    //uart send g_tx_buffer
    HAL_UART_Transmit_DMA(&huart1,g_tx_buffer,TX_BUFF_DEBUG_SIZE);
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
#ifdef USE_FULL_ASSERT
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
