/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	__HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
	free(UART_RX_BUFF);
	UART_RX_BUFF = (uint8_t*)malloc(3);
	//memset(UART_RX_BUFF, 0, sizeof(3));
	HAL_UART_Receive(&huart1, UART_RX_BUFF, 3, 10);
	if(UART_RX_BUFF[0] == SYNCHRO)
	{
		UART_first_byte = UART_RX_BUFF[0];
		UART_length.cstd[1] = UART_RX_BUFF[1];
		UART_length.cstd[0] = UART_RX_BUFF[2];
		if(UART_length.istd >= 6)
		{
			free(UART_RX_BUFF);
			free(UART_TX_BUFF);
			UART_TX_BUFF = (uint8_t*)malloc(UART_length.istd);
			UART_RX_BUFF = (uint8_t*)malloc(UART_length.istd);
			HAL_UART_Receive(&huart1, UART_TX_BUFF, (UART_length.istd - 3), 10);
			UART_RX_BUFF[0] = UART_first_byte;
			UART_RX_BUFF[1] = UART_length.cstd[1];
			UART_RX_BUFF[2] = UART_length.cstd[0];
			memcpy((char*)(UART_RX_BUFF + 3), (char*)UART_TX_BUFF, (UART_length.istd - 3));		
			memcpy((char*)(UART_TX_BUFF), (char*)UART_RX_BUFF, UART_length.istd);		
			if(UART_RX_BUFF[3] == UART_ADDR)
			{	
				if(xor_handler(UART_RX_BUFF) == 0)
				{			
					UART_pack_parser();
				}
				else
				{
					free(UART_TX_BUFF);
					UART_TX_BUFF = (uint8_t*)malloc(9);
					strcpy((char*)UART_TX_BUFF, "Error_xor");
					HAL_UART_Transmit(&huart1, UART_TX_BUFF, 9, 10);					
				}
			}
			else
			{
				free(UART_TX_BUFF);
				UART_TX_BUFF = (uint8_t*)malloc(13);
				strcpy((char*)UART_TX_BUFF, "Error_address");
				HAL_UART_Transmit(&huart1, UART_TX_BUFF, 13, 10);				
			}
		}
		else
		{
			free(UART_TX_BUFF);
			UART_TX_BUFF = (uint8_t*)malloc(9);
			strcpy((char*)UART_TX_BUFF, "Error_len");
			HAL_UART_Transmit(&huart1, UART_TX_BUFF, 9, 10);
		}
	}
	else 
	{
		free(UART_TX_BUFF);
		UART_TX_BUFF = (uint8_t*)malloc(13);
		strcpy((char*)UART_TX_BUFF, "Error_synchro");
		HAL_UART_Transmit(&huart1, UART_TX_BUFF, 13, 10);
	}
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
