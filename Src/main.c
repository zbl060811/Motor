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
#include "can.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <assert.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_C610_M1_ID 0x201
#define CAN_C610_M2_ID 0x202
#define CAN_C610_M3_ID 0x203
#define CAN_C610_M4_ID 0x204
#define CAN_C610_M5_ID 0x205
#define CAN_C610_M6_ID 0x206
#define CAN_C610_M7_ID 0x207
#define CAN_C610_M8_ID 0x208

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

extern CAN_TxHeaderTypeDef g_can1_txheader;
CAN_RxHeaderTypeDef g_can1_rxheader; 
int16_t motor_speed = 0;
uint8_t motor_select = 0;
int16_t current_cont[4];

typedef struct
{
    uint16_t etid;  // Electric tuning ID
    int16_t mahr;   // Mechanical angle height of rotor
    int16_t speed;  // Rotor speed
    int16_t torque; // Actual output torque
} Motor_parameters;

Motor_parameters motor1[20];

void Get_Motor_Information(uint8_t motor_sel ,Motor_parameters *motor, uint8_t *data)     // 分解C610电调发送过来的数据
{
  assert(motor);
  assert(data);
  motor[motor_sel].etid = g_can1_rxheader.StdId;
  motor[motor_sel].mahr = (int16_t)(data[0]) << 8 | data[1];
  motor[motor_sel].speed = (data[2]) << 8 | data[3];
  motor[motor_sel].torque = (int16_t)(data[4]) << 8 | data[5];
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1)     // 判断是否为CAN1
  {
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &g_can1_rxheader, rx_data);        

    // motor_select = g_can1_rxheader.StdId - 0x200;
    // Get_Motor_Information(motor_select, motor1, rx_data);
    // current_cont[0] = (motor_speed - motor1[motor_select].speed) * 10;
    // if(motor_select < 5)
    //     can_send_data(0X200, current_cont, motor_select);
    // else
    //     can_send_data(0X1FF, current_cont, motor_select);

    switch (g_can1_rxheader.StdId)
    {
        case CAN_C610_M1_ID:
          motor_select = 0;
          Get_Motor_Information(motor_select, motor1, rx_data);
          current_cont[0] = (motor_speed - motor1[motor_select].speed) * 10;
          can_send_data(0X200, current_cont, motor_select);
        break;

        case CAN_C610_M2_ID:
          motor_select = 1;
          Get_Motor_Information(motor_select, motor1, rx_data);
          current_cont[0] = (motor_speed - motor1[motor_select].speed) * 10;
          can_send_data(0X200, current_cont, motor_select);
        break;

        case CAN_C610_M3_ID:
          motor_select = 2;
          Get_Motor_Information(motor_select, motor1, rx_data);
          current_cont[0] = (motor_speed - motor1[motor_select].speed) * 10;
          can_send_data(0X200, current_cont, motor_select);
        break;

        case CAN_C610_M4_ID:
          motor_select = 3;
          Get_Motor_Information(motor_select, motor1, rx_data);
          current_cont[0] = (motor_speed - motor1[motor_select].speed) * 10;
          can_send_data(0X200, current_cont, motor_select);
        break;

        case CAN_C610_M5_ID:
          motor_select = 4;
          Get_Motor_Information(motor_select, motor1, rx_data);
          current_cont[0] = (motor_speed - motor1[motor_select].speed) * 10;
          can_send_data(0X1FF, current_cont, motor_select);
        break;

        case CAN_C610_M6_ID:
          motor_select = 5;
          Get_Motor_Information(motor_select, motor1, rx_data);
          current_cont[0] = (motor_speed - motor1[motor_select].speed) * 10;
          can_send_data(0X1FF, current_cont, motor_select);
        break;

        case CAN_C610_M7_ID:
          motor_select = 6;
          Get_Motor_Information(motor_select, motor1, rx_data);
          current_cont[0] = (motor_speed - motor1[motor_select].speed) * 10;
          can_send_data(0X1FF, current_cont, motor_select);
        break;

        case CAN_C610_M8_ID:
          motor_select = 7;
          Get_Motor_Information(motor_select, motor1, rx_data);
          current_cont[0] = (motor_speed - motor1[motor_select].speed) * 10;
          can_send_data(0X1FF, current_cont, motor_select);
        break;
    }
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_2)
	{
		motor_speed += 500;
	}                     
}


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
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  //HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
