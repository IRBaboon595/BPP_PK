/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi4;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart8;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

ADF4360_InitTypeDef 					hADF4360i;
ADF4360_RCounterTypeDef				hADF4360_Ri;
ADF4360_NCounterTypeDef				hADF4360_Ni;
ADF4360_ControlRegTypeDef			hADF4360_CTRLi;

double att_steps[6] = {16, 8, 4, 2, 1, 0.5};

uint8_t 					ser_att = 0;
uint8_t 					*SPI1_RX_BUFF;
uint8_t 					*SPI1_TX_BUFF;
uint8_t 					*SPI4_RX_BUFF;
uint8_t 					*SPI4_TX_BUFF;
uint8_t 					*SPI_BUFF;
uint8_t 					*UART_RX_BUFF;
uint8_t 					*UART_TX_BUFF;
uint8_t						UART_first_byte;
uint8_t 					uart_command;
uint8_t						adc_enable;
uint8_t						tim2_overflow = 0;
uint8_t 					adc_counter = 0;
uint8_t						UART_BIG_TX_BUFF[3000];

long_std_union 		adc_result;
std_union					UART_length;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI4_Init(void);
static void MX_UART4_Init(void);
static void MX_UART7_Init(void);
static void MX_UART8_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI3_Init(void);
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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_SPI4_Init();
  MX_UART4_Init();
  MX_UART7_Init();
  MX_UART8_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
	free(SPI_BUFF);
	SPI_BUFF = (uint8_t *)malloc(3);
	HAL_GPIO_WritePin(ADF4360_1_LE_GPIO_Port, ADF4360_1_LE_Pin, GPIO_PIN_SET);		
	HAL_GPIO_WritePin(ADF4360_2_LE_GPIO_Port, ADF4360_2_LE_Pin, GPIO_PIN_SET);		
	HAL_GPIO_WritePin(ADF4360_1_CE_GPIO_Port, ADF4360_1_CE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ADF4360_2_CE_GPIO_Port, ADF4360_2_CE_Pin, GPIO_PIN_SET);
	/*HAL_GPIO_WritePin(RF_5V_RX_CTRL_GPIO_Port, RF_5V_RX_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RF_5V_TX_CTRL_GPIO_Port, RF_5V_TX_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TX_CH1_1_5V_CTRL_GPIO_Port, TX_CH1_1_5V_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TX_CH2_1_5V_CTRL_GPIO_Port, TX_CH2_1_5V_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RX_CH1_1_5V_1_CTRL_GPIO_Port, RX_CH1_1_5V_1_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RX_CH1_1_5V_2_CTRL_GPIO_Port, RX_CH1_1_5V_2_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RX_CH2_1_5V_1_CTRL_GPIO_Port, RX_CH2_1_5V_1_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RX_CH2_1_5V_2_CTRL_GPIO_Port, RX_CH2_1_5V_2_CTRL_Pin, GPIO_PIN_RESET);*/
	supply_ctrl(0xFF, 0x1F);
	//supply_ctrl(0x00, 0x1F);
	//supply_ctrl(0xFF, 0x04);
	//synthesizer_init(&hspi4);
	//synthesizer_init(&hspi1);
	HAL_Delay(20);
	synthesizer_user(&hspi4, OUT_POW_8dBm, PRESC_8_9, 4, 64, 12);
	HAL_Delay(20);
	synthesizer_user(&hspi1, OUT_POW_8dBm, PRESC_8_9, 3, 84, 12);
	//attenuator_set_att(3, 6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_Delay(1000);
		HAL_GPIO_TogglePin(TEST_LED_GPIO_Port, TEST_LED_Pin);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 480;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI3|RCC_PERIPHCLK_SPI1;
  PeriphClkInitStruct.PLL2.PLL2M = 12;
  PeriphClkInitStruct.PLL2.PLL2N = 200;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_1;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  hi2c1.Init.Timing = 0x307075B1;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart7, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart7, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * @brief UART8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART8_Init(void)
{

  /* USER CODE BEGIN UART8_Init 0 */

  /* USER CODE END UART8_Init 0 */

  /* USER CODE BEGIN UART8_Init 1 */

  /* USER CODE END UART8_Init 1 */
  huart8.Instance = UART8;
  huart8.Init.BaudRate = 115200;
  huart8.Init.WordLength = UART_WORDLENGTH_8B;
  huart8.Init.StopBits = UART_STOPBITS_1;
  huart8.Init.Parity = UART_PARITY_NONE;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  huart8.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart8.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart8.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart8, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart8, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART8_Init 2 */

  /* USER CODE END UART8_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, ADF4360_1_LE_Pin|ADF4360_1_CE_Pin|RE_DE_Pin|X12_Pin
                          |X13_Pin|X14_Pin|X15_Pin|Alarm_Pin
                          |Soft_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, X22_Pin|X23_Pin|X24_Pin|X25_Pin
                          |X1_Pin|X2_Pin|X3_Pin|RF_5V_CTRL_Pin
                          |RF_5V_RX_CTRL_Pin|ENBD_Pin|ENAC_Pin|TEST_LED_Pin
                          |HMC_LE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ADF4360_2_LE_Pin|ADF4360_2_CE_Pin|X6_Pin|X5_Pin
                          |X4_Pin|X8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CH1_1_5V_CTRL_Pin|POWER_n5V_CTRL_Pin|CH2_1_5V_CTRL_Pin|X21_Pin
                          |CUR_1_Pin|CUR_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RX_CH1_1_5V_1_CTRL_Pin|RX_CH1_1_5V_2_CTRL_Pin|RX_CH2_1_5V_1_CTRL_Pin|RX_CH2_1_5V_2_CTRL_Pin
                          |TX_CH2_1_5V_CTRL_Pin|TX_CH1_1_5V_CTRL_Pin|RF_5V_TX_CTRL_Pin|X9_Pin
                          |X10_Pin|X11_Pin|X16_Pin|X17_Pin
                          |X18_Pin|X19_Pin|X20_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(POWER_3_3_CTRL_GPIO_Port, POWER_3_3_CTRL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : ADF4360_1_LE_Pin */
  GPIO_InitStruct.Pin = ADF4360_1_LE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ADF4360_1_LE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ADF4360_1_CE_Pin RE_DE_Pin X12_Pin X13_Pin
                           X14_Pin X15_Pin Alarm_Pin Soft_Pin */
  GPIO_InitStruct.Pin = ADF4360_1_CE_Pin|RE_DE_Pin|X12_Pin|X13_Pin
                          |X14_Pin|X15_Pin|Alarm_Pin|Soft_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : ADF4360_1_MUX_Pin */
  GPIO_InitStruct.Pin = ADF4360_1_MUX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADF4360_1_MUX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : X22_Pin X23_Pin X24_Pin X25_Pin
                           X1_Pin X2_Pin X3_Pin RF_5V_CTRL_Pin
                           RF_5V_RX_CTRL_Pin ENBD_Pin ENAC_Pin TEST_LED_Pin */
  GPIO_InitStruct.Pin = X22_Pin|X23_Pin|X24_Pin|X25_Pin
                          |X1_Pin|X2_Pin|X3_Pin|RF_5V_CTRL_Pin
                          |RF_5V_RX_CTRL_Pin|ENBD_Pin|ENAC_Pin|TEST_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ADF4360_2_LE_Pin */
  GPIO_InitStruct.Pin = ADF4360_2_LE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ADF4360_2_LE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ADF4360_2_CE_Pin X6_Pin X5_Pin X4_Pin
                           X8_Pin */
  GPIO_InitStruct.Pin = ADF4360_2_CE_Pin|X6_Pin|X5_Pin|X4_Pin
                          |X8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ADF4360_2_MUX_Pin */
  GPIO_InitStruct.Pin = ADF4360_2_MUX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADF4360_2_MUX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CH1_1_5V_CTRL_Pin CH2_1_5V_CTRL_Pin */
  GPIO_InitStruct.Pin = CH1_1_5V_CTRL_Pin|CH2_1_5V_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : POWER_n5V_CTRL_Pin X21_Pin CUR_1_Pin CUR_2_Pin */
  GPIO_InitStruct.Pin = POWER_n5V_CTRL_Pin|X21_Pin|CUR_1_Pin|CUR_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PG_5_5_Pin PG_3_9_Pin */
  GPIO_InitStruct.Pin = PG_5_5_Pin|PG_3_9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RX_CH1_1_5V_1_CTRL_Pin RX_CH1_1_5V_2_CTRL_Pin RX_CH2_1_5V_1_CTRL_Pin RX_CH2_1_5V_2_CTRL_Pin
                           TX_CH2_1_5V_CTRL_Pin TX_CH1_1_5V_CTRL_Pin */
  GPIO_InitStruct.Pin = RX_CH1_1_5V_1_CTRL_Pin|RX_CH1_1_5V_2_CTRL_Pin|RX_CH2_1_5V_1_CTRL_Pin|RX_CH2_1_5V_2_CTRL_Pin
                          |TX_CH2_1_5V_CTRL_Pin|TX_CH1_1_5V_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : POWER_3_3_CTRL_Pin RF_5V_TX_CTRL_Pin X9_Pin X10_Pin
                           X11_Pin X16_Pin X17_Pin X18_Pin
                           X19_Pin X20_Pin */
  GPIO_InitStruct.Pin = POWER_3_3_CTRL_Pin|RF_5V_TX_CTRL_Pin|X9_Pin|X10_Pin
                          |X11_Pin|X16_Pin|X17_Pin|X18_Pin
                          |X19_Pin|X20_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : HMC_LE_Pin */
  GPIO_InitStruct.Pin = HMC_LE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(HMC_LE_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void UART_pack_parser(void)
{
	long_long_std_union long_temp;
	long_std_union temp;
	std_union	short_temp;
	uint16_t adf_b = 0;	
	uint16_t adf_r = 0;	
	float temp_att = 0;
	
	double att_temp = 0;
	switch(UART_RX_BUFF[4])
	{ 
		case ECHO:
			HAL_UART_Transmit(&huart1, UART_TX_BUFF, UART_length.istd, 10);
			uart_command = 0;
		break;	
		case SUPPLY_CTRL:
			supply_ctrl(UART_RX_BUFF[5], UART_RX_BUFF[6]);
			uart_command = 0x01;
		break;
		case ATT_CTRL:
			temp_att = UART_RX_BUFF[5];
			temp_att /= 2;
			attenuator_set_att(temp_att, UART_RX_BUFF[6]);
			uart_command = 0x02;
		break;
		case ADF_CTRL:
			short_temp.cstd[1] = UART_RX_BUFF[9];
			short_temp.cstd[0] = UART_RX_BUFF[10];
			adf_b = short_temp.istd;
		
			short_temp.cstd[1] = UART_RX_BUFF[11];
			short_temp.cstd[0] = UART_RX_BUFF[12];
			adf_r = short_temp.istd;		
			
			if(UART_RX_BUFF[5])
			{
				synthesizer_user(&hspi4, UART_RX_BUFF[6], UART_RX_BUFF[7], UART_RX_BUFF[8], adf_b, adf_r);
			}
			else
			{
				synthesizer_user(&hspi1, UART_RX_BUFF[6], UART_RX_BUFF[7], UART_RX_BUFF[8], adf_b, adf_r);
			}
			uart_command = 0x03;
		break;
		case SET_ATT:
			
			uart_command = 0x04;
		case AMP_MANAGE:
			
			uart_command = 0x05;
			break;
		case ADC_ECHO:
			
			uart_command = 0x06;
		break;
		case CRYSTAL_EN:		
			
			uart_command = 0x07;
		break;
		case AD5932_CTR:
			
			uart_command = 0x08;
		break;
		case ADF4360_CTR:
			
			uart_command = 0x09;
		break;
		case SEND_FREQ_PARAM:

			uart_command = 0x0A;
		break;
		case SEND_FREQ_PARAM_TOTAL:
			
			uart_command = 0x0B;
		break;
		case MODE_SELECT:
			
			uart_command = 0x0C;
		break;
		default:
			uart_command = 255;
		break;
	}
}

uint8_t xor_handler(uint8_t *mass)
{
	uint8_t result = 0;
	std_union temp_1;
	temp_1.cstd[1] = mass[1];
	temp_1.cstd[0] = mass[2];
	for(int i = 0; i < (temp_1.istd); i++)
	{
		result ^= mass[i];
	}
	return result;
}

/**************************************** HMC8073 MAINTAIN FUNCTIONS ***********************************************/

void attenuator_set_att(float attenuation, uint8_t address)
{
	uint8_t SPI_ATT_BUFF[2];
	uint8_t att_address = address;
	
	for (int i = 0; i < 6; i++)											//D7 and D0 are do not care bits
	{
		attenuation -= att_steps[i];
		if(attenuation >= 0)
		{
			SPI_ATT_BUFF[0] += (1 << (6 - i));
		}
		else
		{
			attenuation += att_steps[i];
		}
	}
	
	SPI_ATT_BUFF[1] = address;
	
	HAL_GPIO_WritePin(HMC_LE_GPIO_Port, HMC_LE_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, SPI_ATT_BUFF, 1, 1);
	HAL_GPIO_WritePin(HMC_LE_GPIO_Port, HMC_LE_Pin, GPIO_PIN_SET);
	
	/*ser_att = (ser_att << 1) | 0x81;
	
	HAL_GPIO_WritePin(HMC_LE_GPIO_Port, HMC_LE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HMC_SCK_GPIO_Port, HMC_SCK_Pin, GPIO_PIN_RESET);
	
	for(int i = 7; i >= 0; i--)
	{
		if((ser_att & (1 << i)) != 0)
		{
			HAL_GPIO_WritePin(HMC_DATA_GPIO_Port, HMC_DATA_Pin, GPIO_PIN_SET);
		}
		else if((ser_att & (1 << i)) == 0)
		{
			HAL_GPIO_WritePin(HMC_DATA_GPIO_Port, HMC_DATA_Pin, GPIO_PIN_RESET);
		}
		HAL_GPIO_WritePin(HMC_SCK_GPIO_Port, HMC_SCK_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(HMC_SCK_GPIO_Port, HMC_SCK_Pin, GPIO_PIN_RESET);
	}	
	
	for(int i = 0; i < 8; i++)
	{
		if((att_address & (1 << i)) != 0)
		{
			HAL_GPIO_WritePin(HMC_DATA_GPIO_Port, HMC_DATA_Pin, GPIO_PIN_SET);
		}
		else if((att_address & (1 << i)) == 0)
		{
			HAL_GPIO_WritePin(HMC_DATA_GPIO_Port, HMC_DATA_Pin, GPIO_PIN_RESET);
		}
		HAL_GPIO_WritePin(HMC_SCK_GPIO_Port, HMC_SCK_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(HMC_SCK_GPIO_Port, HMC_SCK_Pin, GPIO_PIN_RESET);
	}	
	
	HAL_GPIO_WritePin(HMC_LE_GPIO_Port, HMC_LE_Pin, GPIO_PIN_SET);*/
}

/**************************************** ADF4360-2 MAINTAIN FUNCTIONS ***********************************************/

void synthesizer_init(SPI_HandleTypeDef *hspi)
{
	HAL_GPIO_WritePin(ADF4360_2_CE_GPIO_Port, ADF4360_2_CE_Pin, GPIO_PIN_SET);
	
	hADF4360i.hctrlregi.core_power_level = 							CPL_20MA;
	hADF4360i.hctrlregi.counter_reset = 								NORMAL_STATE;
	hADF4360i.hctrlregi.cp_gain = 											CP_GAIN_SET_1;
	hADF4360i.hctrlregi.cp_three_state = 								CP_NORMAL;
	hADF4360i.hctrlregi.current_setting_1 = 						CUR_SET_1_250;
	hADF4360i.hctrlregi.current_setting_2 = 						CUR_SET_1_250;
	hADF4360i.hctrlregi.mute_till_lock_detect = 				MUTE_DIS;
	hADF4360i.hctrlregi.muxout_control = 								N_DIV;//DIG_LOCK_DET;//R_DIV;//SDO;
	hADF4360i.hctrlregi.output_power_level = 						OUT_POW_11dBm;	
	hADF4360i.hctrlregi.phase_detector_polarity = 			PH_DET_POS;
	hADF4360i.hctrlregi.power_down_1 = 									PD1_DIS;
	hADF4360i.hctrlregi.power_down_2 = 									PD2_ASYN;
	hADF4360i.hctrlregi.prescaler_value = 							PRESC_8_9;
	hADF4360i.hrcounteri.anti_backlash_pulse_width = 		ABP_3_0NS;
	hADF4360i.hrcounteri.band_select_clock = 						BAND_8;
	hADF4360i.hrcounteri.lock_detect_precision = 				FIVE_CYCLES;
	hADF4360i.hrcounteri.r_counter = 										R_PARAM;
	hADF4360i.hrcounteri.test_mode_bit = 								1;						//CLEARING IN INIT PROCESS
	hADF4360i.hncounteri.a_counter = 										A_PARAM;
	hADF4360i.hncounteri.b_counter = 										B_PARAM;
	hADF4360i.hncounteri.cp_gain_ = 										CP_GAIN_SET_1;
	hADF4360i.hncounteri.divide_by_2 = 									0;
	hADF4360i.hncounteri.divide_by_2_select = 					0;
	
	ADF4360_init(&hADF4360i, hspi);	
}

void synthesizer_user(SPI_HandleTypeDef *hspi, uint8_t out_power, uint8_t presc, uint8_t a_c, uint16_t b_c, uint16_t r_c)
{
	hADF4360i.hctrlregi.core_power_level = 							CPL_20MA;
	hADF4360i.hctrlregi.counter_reset = 								NORMAL_STATE;
	hADF4360i.hctrlregi.cp_gain = 											CP_GAIN_SET_1;
	hADF4360i.hctrlregi.cp_three_state = 								CP_NORMAL;
	hADF4360i.hctrlregi.current_setting_1 = 						CUR_SET_1_250;
	hADF4360i.hctrlregi.current_setting_2 = 						CUR_SET_1_250;
	hADF4360i.hctrlregi.mute_till_lock_detect = 				MUTE_DIS;
	hADF4360i.hctrlregi.muxout_control = 								DIG_LOCK_DET;//N_DIV;//R_DIV;
	hADF4360i.hctrlregi.output_power_level = 						out_power;	
	hADF4360i.hctrlregi.phase_detector_polarity = 			PH_DET_POS;
	hADF4360i.hctrlregi.power_down_1 = 									PD1_DIS;
	hADF4360i.hctrlregi.power_down_2 = 									PD2_ASYN;
	hADF4360i.hctrlregi.prescaler_value = 							presc;
	hADF4360i.hrcounteri.anti_backlash_pulse_width = 		ABP_3_0NS;
	hADF4360i.hrcounteri.band_select_clock = 						BAND_8;
	hADF4360i.hrcounteri.lock_detect_precision = 				FIVE_CYCLES;
	hADF4360i.hrcounteri.r_counter = 										r_c;
	hADF4360i.hrcounteri.test_mode_bit = 								1;						//CLEARING IN INIT PROCESS
	hADF4360i.hncounteri.a_counter = 										a_c;
	hADF4360i.hncounteri.b_counter = 										b_c;
	hADF4360i.hncounteri.cp_gain_ = 										CP_GAIN_SET_1;
	hADF4360i.hncounteri.divide_by_2 = 									0;
	hADF4360i.hncounteri.divide_by_2_select = 					0;
	
	ADF4360_init(&hADF4360i, hspi);	
}

/**************************************** Supply controling func ***********************************************/

void supply_ctrl(uint8_t supply_data_1, uint8_t supply_data_2)
{
	for(int i = 0; i < 8; i++)
	{
		if(supply_data_1 & (1 << i))
		{
			if(i < 6)
			{
				HAL_GPIO_WritePin(GPIOD, (GPIO_PIN_8 << i), GPIO_PIN_SET);
			}
			else if (i == 6)
			{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
			}
			else if (i == 7)
			{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
			}
		}
		else
		{
			if(i < 6)
			{
				HAL_GPIO_WritePin(GPIOD, (GPIO_PIN_8 << i), GPIO_PIN_RESET);
			}
			else if (i == 6)
			{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
			}
			else if (i == 7)
			{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
			}
		}
	}
	for(int i = 0; i < 5; i++)
	{
		if((supply_data_2 & (1 << i)))
		{
			if(i == 0)
			{
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
			}
			else if (i == 1)
			{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
			}
			else if (i == 2)
			{
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
			}
			else if (i == 3)
			{
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
			}
			else if (i == 4)
			{
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			}					
		}
		else
		{
			if(i == 0)
			{
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
			}
			else if (i == 1)
			{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
			}
			else if (i == 2)
			{
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
			}
			else if (i == 3)
			{
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
			}
			else if (i == 4)
			{
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
			}							
		}					
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
