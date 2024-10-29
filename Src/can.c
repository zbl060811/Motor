/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */
CAN_TxHeaderTypeDef g_can1_txheader;			// CAN外设发送结构体
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
	CAN_FilterTypeDef sFilterConfig;				              // 实例化一个CAN接收器
	
	sFilterConfig.FilterBank = 0;													/* 选择过滤器0 */
	sFilterConfig.FilterActivation = ENABLE;							/* 开启该过滤器 */
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;			/* 筛选器模式是ID掩码模式 */
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;		/* 筛选器位宽 */
	sFilterConfig.FilterIdHigh = 0x0000;									
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;    /* 过滤器0关联到FIFO0 */
	
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);			  	// 配置过滤器给CAN外设	
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);			// CAN_IT_RX_FIFO0_MSG_PENDING：当FIFO0中有消息的时候进入中断。

	HAL_CAN_Start(&hcan1);
  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void can_send_data(uint32_t id, int16_t *current, uint8_t motor_sel)
{
	uint32_t tx_email = CAN_TX_MAILBOX0;
	uint8_t motor_data1[8];
  uint8_t motor_data2[8];
  	
//	for(uint8_t i = 0; i < 8; i ++)
//	{
//		data[i * 2] = (uint8_t)current[i] >> 8;
//		data[i * 2 + 1] = (uint8_t)current[i];
//	}

  if(motor_sel < 4)
  {
    switch(motor_sel)
    {
      case 0:
        motor_data1[0] = (uint8_t)(current[0] >> 8);
        motor_data1[1] = (uint8_t)current[0];
      break;
      
      case 1:
        motor_data1[2] = (uint8_t)(current[0] >> 8);
        motor_data1[3] = (uint8_t)current[0];
      break;

      case 2:
        motor_data1[4] = (uint8_t)(current[0] >> 8);
        motor_data1[5] = (uint8_t)current[0];
      break;

      case 3:
        motor_data1[6] = (uint8_t)(current[0] >> 8);
        motor_data1[7] = (uint8_t)current[0];
      break;
    }
  }
  else
  {
    switch(motor_sel)
    {
      case 4:
        motor_data2[0] = (uint8_t)(current[0] >> 8);
        motor_data2[1] = (uint8_t)current[0];
      break;
      
      case 5:
        motor_data2[2] = (uint8_t)(current[0] >> 8);
        motor_data2[3] = (uint8_t)current[0];
      break;

      case 6:
        motor_data2[4] = (uint8_t)(current[0] >> 8);
        motor_data2[5] = (uint8_t)current[0];
      break;

      case 7:
        motor_data2[6] = (uint8_t)(current[0] >> 8);
        motor_data2[7] = (uint8_t)current[0];
      break;
    }
  }
 	
	g_can1_txheader.StdId = id;
	g_can1_txheader.DLC = 8;
	g_can1_txheader.IDE = CAN_ID_STD;
	g_can1_txheader.RTR = CAN_RTR_DATA;

  if(motor_sel < 4)
  	HAL_CAN_AddTxMessage(&hcan1, &g_can1_txheader, motor_data1, &tx_email);
  else
    HAL_CAN_AddTxMessage(&hcan1, &g_can1_txheader, motor_data2, &tx_email);

	// while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3); 
}
/* USER CODE END 1 */
