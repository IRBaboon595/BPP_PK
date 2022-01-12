/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "math.h"
#include "string.h"
#include "stdint.h"
#include "stdlib.h"
#include "dwt_delay.h"
#include <stdio.h>
#include <time.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef struct{
	uint16_t														r_counter;
	uint8_t															anti_backlash_pulse_width;
	uint8_t															lock_detect_precision;
	uint8_t															test_mode_bit;
	uint8_t															band_select_clock;
}ADF4360_RCounterTypeDef;
	
typedef struct{
	uint8_t 														a_counter;
	uint16_t 														b_counter;
	uint8_t															cp_gain_;
	uint8_t															divide_by_2;
	uint8_t															divide_by_2_select;
}ADF4360_NCounterTypeDef;

typedef struct{
	uint8_t															core_power_level;
	uint8_t															counter_reset;
	uint8_t															cp_gain;
	uint8_t															cp_three_state;
	uint8_t															current_setting_1;
	uint8_t															current_setting_2;
	uint8_t															mute_till_lock_detect;
	uint8_t															muxout_control;
	uint8_t															output_power_level;
	uint8_t															phase_detector_polarity;
	uint8_t															power_down_1;
	uint8_t															power_down_2;
	uint8_t															prescaler_value;
}ADF4360_ControlRegTypeDef;

typedef struct{
	ADF4360_ControlRegTypeDef						hctrlregi;
	ADF4360_RCounterTypeDef							hrcounteri;
	ADF4360_NCounterTypeDef							hncounteri;		
}ADF4360_InitTypeDef;

typedef union{
	uint16_t 														istd;
	uint8_t 														cstd[2];
}std_union;

typedef union{
	uint32_t 														listd;
	uint16_t 														istd[2];
	uint8_t 														cstd[4];
}long_std_union;

typedef union{
	uint64_t 														llistd;
	uint32_t 														listd[2];
	uint16_t 														istd[4];
	uint8_t 														cstd[8];
}long_long_std_union;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

extern SPI_HandleTypeDef							hspi1;
extern SPI_HandleTypeDef							hspi3;
extern SPI_HandleTypeDef							hspi4;

//extern UART_HandleTypeDef 						huart4;
//extern UART_HandleTypeDef 						huart5;
//extern UART_HandleTypeDef 						huart7;
extern UART_HandleTypeDef 						huart1;
//extern UART_HandleTypeDef 						huart2;
extern UART_HandleTypeDef 						huart3;

extern uint8_t												*UART_RX_BUFF;
extern uint8_t												*UART_TX_BUFF;
extern uint8_t												UART_first_byte;
extern uint8_t												uart_command;
extern std_union											UART_length;

extern uint8_t 												*SPI1_RX_BUFF;
extern uint8_t 												*SPI1_TX_BUFF;
extern uint8_t 												*SPI4_RX_BUFF;
extern uint8_t 												*SPI4_TX_BUFF;
extern uint8_t 												*SPI_BUFF;

extern ADF4360_InitTypeDef 						hADF4360i;
extern ADF4360_RCounterTypeDef				hADF4360_Ri;
extern ADF4360_NCounterTypeDef				hADF4360_Ni;
extern ADF4360_ControlRegTypeDef			hADF4360_CTRLi;

extern double att_steps[6];

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

extern void UART_pack_parser(void);
extern uint8_t xor_handler(uint8_t *mass);
extern void attenuator_set_att(float attenuation, uint8_t address);
extern void ADF4360_init(ADF4360_InitTypeDef *phADF4360i, SPI_HandleTypeDef *hspi);
extern void synthesizer_init(SPI_HandleTypeDef *hspi);
extern void synthesizer_user(SPI_HandleTypeDef *hspi, uint8_t out_power, uint8_t presc, uint8_t a_c, uint16_t b_c, uint16_t r_c);
extern void supply_ctrl(uint8_t supply_data_1, uint8_t supply_data_2);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADF4360_1_SCK_Pin GPIO_PIN_2
#define ADF4360_1_SCK_GPIO_Port GPIOE
#define ADF4360_1_LE_Pin GPIO_PIN_3
#define ADF4360_1_LE_GPIO_Port GPIOE
#define ADF4360_1_CE_Pin GPIO_PIN_4
#define ADF4360_1_CE_GPIO_Port GPIOE
#define ADF4360_1_MUX_Pin GPIO_PIN_5
#define ADF4360_1_MUX_GPIO_Port GPIOE
#define ADF4360_1_DATA_Pin GPIO_PIN_6
#define ADF4360_1_DATA_GPIO_Port GPIOE
#define X22_Pin GPIO_PIN_13
#define X22_GPIO_Port GPIOC
#define X23_Pin GPIO_PIN_14
#define X23_GPIO_Port GPIOC
#define X24_Pin GPIO_PIN_15
#define X24_GPIO_Port GPIOC
#define X25_Pin GPIO_PIN_0
#define X25_GPIO_Port GPIOC
#define X1_Pin GPIO_PIN_1
#define X1_GPIO_Port GPIOC
#define X2_Pin GPIO_PIN_2
#define X2_GPIO_Port GPIOC
#define X3_Pin GPIO_PIN_3
#define X3_GPIO_Port GPIOC
#define RS422_A_TX_Pin GPIO_PIN_0
#define RS422_A_TX_GPIO_Port GPIOA
#define RS422_A_RX_Pin GPIO_PIN_1
#define RS422_A_RX_GPIO_Port GPIOA
#define RS422_D_TX_Pin GPIO_PIN_2
#define RS422_D_TX_GPIO_Port GPIOA
#define RS422_D_RX_Pin GPIO_PIN_3
#define RS422_D_RX_GPIO_Port GPIOA
#define ADF4360_2_LE_Pin GPIO_PIN_4
#define ADF4360_2_LE_GPIO_Port GPIOA
#define ADF4360_2_SCK_Pin GPIO_PIN_5
#define ADF4360_2_SCK_GPIO_Port GPIOA
#define ADF4360_2_CE_Pin GPIO_PIN_6
#define ADF4360_2_CE_GPIO_Port GPIOA
#define ADF4360_2_DATA_Pin GPIO_PIN_7
#define ADF4360_2_DATA_GPIO_Port GPIOA
#define ADF4360_2_MUX_Pin GPIO_PIN_4
#define ADF4360_2_MUX_GPIO_Port GPIOC
#define RF_5V_CTRL_Pin GPIO_PIN_5
#define RF_5V_CTRL_GPIO_Port GPIOC
#define CH1_1_5V_CTRL_Pin GPIO_PIN_0
#define CH1_1_5V_CTRL_GPIO_Port GPIOB
#define POWER_n5V_CTRL_Pin GPIO_PIN_1
#define POWER_n5V_CTRL_GPIO_Port GPIOB
#define CH2_1_5V_CTRL_Pin GPIO_PIN_2
#define CH2_1_5V_CTRL_GPIO_Port GPIOB
#define RS422_C_RX_Pin GPIO_PIN_7
#define RS422_C_RX_GPIO_Port GPIOE
#define RS422_C_TX_Pin GPIO_PIN_8
#define RS422_C_TX_GPIO_Port GPIOE
#define RE_DE_Pin GPIO_PIN_9
#define RE_DE_GPIO_Port GPIOE
#define X12_Pin GPIO_PIN_10
#define X12_GPIO_Port GPIOE
#define X13_Pin GPIO_PIN_11
#define X13_GPIO_Port GPIOE
#define X14_Pin GPIO_PIN_12
#define X14_GPIO_Port GPIOE
#define X15_Pin GPIO_PIN_13
#define X15_GPIO_Port GPIOE
#define Alarm_Pin GPIO_PIN_14
#define Alarm_GPIO_Port GPIOE
#define Soft_Pin GPIO_PIN_15
#define Soft_GPIO_Port GPIOE
#define PG_5_5_Pin GPIO_PIN_12
#define PG_5_5_GPIO_Port GPIOB
#define PG_3_9_Pin GPIO_PIN_13
#define PG_3_9_GPIO_Port GPIOB
#define RX_CH1_1_5V_1_CTRL_Pin GPIO_PIN_8
#define RX_CH1_1_5V_1_CTRL_GPIO_Port GPIOD
#define RX_CH1_1_5V_2_CTRL_Pin GPIO_PIN_9
#define RX_CH1_1_5V_2_CTRL_GPIO_Port GPIOD
#define RX_CH2_1_5V_1_CTRL_Pin GPIO_PIN_10
#define RX_CH2_1_5V_1_CTRL_GPIO_Port GPIOD
#define RX_CH2_1_5V_2_CTRL_Pin GPIO_PIN_11
#define RX_CH2_1_5V_2_CTRL_GPIO_Port GPIOD
#define TX_CH2_1_5V_CTRL_Pin GPIO_PIN_12
#define TX_CH2_1_5V_CTRL_GPIO_Port GPIOD
#define TX_CH1_1_5V_CTRL_Pin GPIO_PIN_13
#define TX_CH1_1_5V_CTRL_GPIO_Port GPIOD
#define POWER_3_3_CTRL_Pin GPIO_PIN_14
#define POWER_3_3_CTRL_GPIO_Port GPIOD
#define RF_5V_TX_CTRL_Pin GPIO_PIN_15
#define RF_5V_TX_CTRL_GPIO_Port GPIOD
#define RF_5V_RX_CTRL_Pin GPIO_PIN_6
#define RF_5V_RX_CTRL_GPIO_Port GPIOC
#define ENBD_Pin GPIO_PIN_7
#define ENBD_GPIO_Port GPIOC
#define ENAC_Pin GPIO_PIN_8
#define ENAC_GPIO_Port GPIOC
#define TEST_LED_Pin GPIO_PIN_9
#define TEST_LED_GPIO_Port GPIOC
#define X6_Pin GPIO_PIN_9
#define X6_GPIO_Port GPIOA
#define X5_Pin GPIO_PIN_10
#define X5_GPIO_Port GPIOA
#define X4_Pin GPIO_PIN_11
#define X4_GPIO_Port GPIOA
#define X8_Pin GPIO_PIN_12
#define X8_GPIO_Port GPIOA
#define HMC_SCK_Pin GPIO_PIN_10
#define HMC_SCK_GPIO_Port GPIOC
#define HMC_LE_Pin GPIO_PIN_11
#define HMC_LE_GPIO_Port GPIOC
#define HMC_DATA_Pin GPIO_PIN_12
#define HMC_DATA_GPIO_Port GPIOC
#define X9_Pin GPIO_PIN_0
#define X9_GPIO_Port GPIOD
#define X10_Pin GPIO_PIN_1
#define X10_GPIO_Port GPIOD
#define X11_Pin GPIO_PIN_2
#define X11_GPIO_Port GPIOD
#define X16_Pin GPIO_PIN_3
#define X16_GPIO_Port GPIOD
#define X17_Pin GPIO_PIN_4
#define X17_GPIO_Port GPIOD
#define X18_Pin GPIO_PIN_5
#define X18_GPIO_Port GPIOD
#define X19_Pin GPIO_PIN_6
#define X19_GPIO_Port GPIOD
#define X20_Pin GPIO_PIN_7
#define X20_GPIO_Port GPIOD
#define X21_Pin GPIO_PIN_5
#define X21_GPIO_Port GPIOB
#define CUR_1_Pin GPIO_PIN_8
#define CUR_1_GPIO_Port GPIOB
#define CUR_2_Pin GPIO_PIN_9
#define CUR_2_GPIO_Port GPIOB
#define RS422_B_RX_Pin GPIO_PIN_0
#define RS422_B_RX_GPIO_Port GPIOE
#define RS422_B_TX_Pin GPIO_PIN_1
#define RS422_B_TX_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

#define	ON														0x01
#define OFF														0x00

/******************************************* UART1 DEFINES *******************************************/

#define	SYNCHRO												0x02
#define UART_ADDR											0x0A

#define ECHO													0x00
#define SUPPLY_CTRL										0x01
#define ATT_CTRL											0x02
#define ADF_CTRL											0x03
#define SET_ATT												0x04
#define AMP_MANAGE										0x05
#define ADC_ECHO											0x06
#define CRYSTAL_EN										0x07
#define AD5932_CTR										0x08
#define ADF4360_CTR										0x09
#define SEND_FREQ_PARAM								0x0A
#define SEND_FREQ_PARAM_TOTAL					0x0B
#define MODE_SELECT              		  0x0C

#define SERVICE_BITS_LEN							0x06

/******************************************* HMC8073 DEFINES *******************************************/

#define SIGMA_RF_ATT_ADDRESS					0x00
#define SIGMA_IF_ATT_ADDRESS					0x01

#define DELTA_RF_ATT_ADDRESS					0x04
#define DELTA_IF_ATT_ADDRESS					0x05

#define OMEGA_RF_ATT_ADDRESS					0x02
#define OMEGA_IF_ATT_ADDRESS					0x03

#define LO_RF_ATT_ADDRESS							0x06


/******************************************* ADF4360 DEFINES *******************************************/

/*		COMMON DEFINES		by default		*/
#define R_PARAM						1 //5	
#define A_PARAM						0 //2
#define B_PARAM						5000 //31
#define REF_FREQ_IN				40000000
#define FVCO							((P_PARAM*B_PARAM)+A_PARAM)*REF_FREQ_IN/R_PARAM
#define P_PARAM						8
#define K_PARAM						((P_PARAM*B_PARAM)+A_PARAM)/R_PARAM
#define FREQ_OUT					2000000000 
#define BASE_CENTER_FREQ	2000000000
#define BASE_BAND					10000000
#define BASE_FREQ_STEP		40000

/******************************************* SWEEP DEFINES *******************************************/

#define SWEEP_LEN				200 //460  380  
#define time_on					400
#define time_off				550

/*		REGISTER ADDRESSES		*/
#define CONTROL_L					0x0
#define R_COUNTER					0x1
#define N_COUNTER					0x2

#define ADD_BITS_ADF4360	0

/*		 CONTROL LATCH BITS		*/
#define CPL								2
#define COUNTER_RESET			4
#define MUXOUT_CTRL				5
#define PH_DET_POL				8
#define CP_3_STATE				9
#define CP_GAIN_CL				10
#define MUTE_LD						11
#define OUT_POWER_LVL			12
#define CURRENT_SET_1			14
#define CURRENT_SET_2			17
#define POWER_DOWN_1			20
#define POWER_DOWN_2			21
#define PRESCALER					22

/*		 R COUNTER BITS		*/
#define REF_COUNTER				2
#define ABP_WID						16
#define LOCK_DET_PREC			18
#define TST_MODE_BIT			19
#define BAND_SEL_CLK			20

/*		 N COUNTER BITS		*/
#define A_COUNTER					2
#define B_COUNTER					8
#define CP_GAIN_N_C				21
#define DIV_2							22
#define DIV_2_SEL					23

/*		BIT ACTIONS		*/
/*		CTRL LATCH BIT ACTIONS		*/
#define	CPL_5MA						0x0
#define	CPL_10MA					0x1
#define	CPL_15MA					0x2
#define	CPL_20MA					0x3

#define	NORMAL_STATE			0x0
#define	RESET_STATE				0x1

#define	TRI_STATE					0x0
#define	DIG_LOCK_DET			0x1
#define	N_DIV							0x2
#define	DVDD							0x3
#define	R_DIV							0x4
#define	N_OP_LD						0x5
#define	SDO								0x6
#define	DGND							0x7

#define	PH_DET_NEG				0x0
#define	PH_DET_POS				0x1

#define	CP_NORMAL					0x0
#define	CP_TRISTATE				0x1

#define	CP_GAIN_SET_1			0x0
#define	CP_GAIN_SET_2			0x1

#define	MUTE_DIS					0x0
#define	MUTE_EN					``0x1

#define	OUT_POW_13dBm			0x0
#define	OUT_POW_11dBm			0x1
#define	OUT_POW_8dBm			0x2
#define	OUT_POW_6dBm			0x3

#define	CUR_SET_1_031			0x0
#define	CUR_SET_1_062			0x1
#define	CUR_SET_1_093			0x2
#define	CUR_SET_1_125			0x3
#define	CUR_SET_1_156			0x4
#define	CUR_SET_1_187			0x5
#define	CUR_SET_1_218			0x6
#define	CUR_SET_1_250			0x7

#define	CUR_SET_2_031			0x0
#define	CUR_SET_2_062			0x1
#define	CUR_SET_2_093			0x2
#define	CUR_SET_2_125			0x3
#define	CUR_SET_2_156			0x4
#define	CUR_SET_2_187			0x5
#define	CUR_SET_2_218			0x6
#define	CUR_SET_2_250			0x7

#define	PD1_DIS						0x0
#define	PD1_EN						0x1

#define	PD2_ASYN					0x0
#define	PD2_SYN						0x1

#define	PRESC_8_9					0x0
#define	PRESC_16_17				0x1
#define	PRESC_32_33				0x2
#define	PRESC_32_33_d			0x3

/*		R COUNTER BIT ACTIONS		*/
#define	ABP_3_0NS					0x0
#define	ABP_1_3NS					0x1
#define	ABP_6_0NS					0x2
#define	ABP_3_0NS_				0x3
	
#define	THREE_CYCLES			0x0
#define	FIVE_CYCLES				0x1

#define	BAND_1						0x0
#define	BAND_2						0x1
#define	BAND_4						0x2
#define	BAND_8						0x3

/*		N COUNTER BIT ACTIONS		*/
#define	CP_GAIN_SET_1			0x0
#define	CP_GAIN_SET_2			0x1
	
#define	FUND_OUT					0x0
#define	DIV_BY_2					0x1

#define	FUND_OUT_SEL			0x0
#define	DIV_BY_2_SEL			0x1

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
