/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define BL_DEBUG_MSG
#define FLASH_SECTOR2_BASE_ADDRESS    0x08008000UL
#define READ_PROTECTION_OPTION_BYTE		0x1FFFC000UL



#define BL_VERSION										0x01
#define BL_ONE_BYTE 									0x01
#define BL_TWO_BYTES								 	0x02
#define BL_THREE_BYTES								0x03
#define BL_FOUR_BYTES								  0x04

#define BL_GET_VER										0x51
#define BL_GET_HELP										0x52
#define BL_GET_CID										0x53
#define BL_GET_RDP_STATUS 						0x54
#define BL_GO_TO_ADDR									0x55
#define BL_FLASH_ERASE								0x56
#define BL_MEM_WRITE									0x57
#define BL_EN_DIS_RW									0x58
#define BL_MEM_READ										0x59
#define BL_READ_SECT_STS    					0x5A
#define BL_OTP_READ										0x5B

#define BL_ACK												0xA5
#define BL_NACK												0x7F

#define VERIFY_CRC_SUCCESS				    0x00
#define VERIFY_CRC_FAIL				  	    0x01

#define 	BL_DATA_LEN									256
#define 	BL_DATA_LEN_IDX							0
#define  	BL_DATA_CMD_IDX							1

#define 	ADDRESS_VALID								0x00
#define 	ADDRESS_INVALID						  0x01

#define   SECTOR_VALID  							0x01
#define   SECTOR_INVALID							0x01

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
