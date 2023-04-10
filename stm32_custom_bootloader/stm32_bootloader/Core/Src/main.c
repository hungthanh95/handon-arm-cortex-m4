/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdarg.h>
#include <string.h>

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
#define C_UART		&huart3
#define D_UART		&huart1
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t BlRxBuffer[BL_DATA_LEN] = {0};
uint8_t supported_commands[] = {
                               BL_GET_VER ,
                               BL_GET_HELP,
                               BL_GET_CID,
                               BL_GET_RDP_STATUS,
                               BL_GO_TO_ADDR,
                               BL_FLASH_ERASE,
                               BL_MEM_WRITE,
															 BL_READ_SECT_STS};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
static void bootloader_uart_read_data(void);
static void bootloader_jump_to_user_app(void);
static void bootloader_handle_getver_cmd(uint8_t * bl_rx_data);
static void bootloader_handle_gethelp_cmd(uint8_t * bl_rx_data);
static void bootloader_handle_getcid_cmd(uint8_t * bl_rx_data);
static void bootloader_handle_getrdp_status_cmd(uint8_t * bl_rx_data);
static void bootloader_handle_goto_addr_cmd(uint8_t * bl_rx_data);
static void bootloader_handle_flash_erase_cmd(uint8_t * bl_rx_data);
static void bootloader_handle_memwrite_cmd(uint8_t * bl_rx_data);
static void bootloader_handle_endis_rw_cmd(uint8_t * bl_rx_data);
static void bootloader_handle_memread_cmd(uint8_t * bl_rx_data);
static void bootloader_handle_read_sector_status_cmd(uint8_t * bl_rx_data);
static void bootloader_handle_otpread_cmd(uint8_t * bl_rx_data);

static uint8_t bootloader_get_version();
static uint16_t get_mcu_chip_id(void);
static uint8_t get_rdp_status(void);
static uint8_t verify_address(uint32_t addr);
static uint8_t execute_erase_flash(uint8_t sector, uint8_t num_sector);
static uint8_t execute_mem_write(uint8_t *pData, uint32_t mem_addr, uint32_t len);

static void bootloader_send_ack(uint8_t cmd, uint8_t len);
static void bootloader_send_nack(void);
static void bootloader_uart_write_data(uint8_t *pData, uint8_t len);
static uint8_t bootloader_verify_crc(uint8_t *pData, uint32_t len, uint32_t crc_host);
static void printmsg(char *format,...);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char HelloInBoot[] = "BL_DEBUG_MSG:Hello from Bootloader\r\n";
char HelloToApp[] = "BL_DEBUG_MSG:Hello to Application\r\n";
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
  	{
  		bootloader_uart_read_data();
  	}
  	else
  	{
  		bootloader_jump_to_user_app();
  	}
    /* USER CODE END 3 */
  }
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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  huart3.Init.BaudRate = 115200;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PG13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void bootloader_uart_read_data()
{
	printmsg(HelloInBoot);
	uint8_t recv_len = 0;
	while(1)
	{
		memset(BlRxBuffer, 0, BL_DATA_LEN);

		// read first byte to get data length
		HAL_UART_Receive(C_UART, BlRxBuffer, 1, HAL_MAX_DELAY);

		recv_len = BlRxBuffer[BL_DATA_LEN_IDX];

		// read remain data
		HAL_UART_Receive(C_UART, &BlRxBuffer[1], recv_len, HAL_MAX_DELAY);

		switch (BlRxBuffer[BL_DATA_CMD_IDX])
		{
		case BL_GET_VER:
			bootloader_handle_getver_cmd(BlRxBuffer);
			break;
		case BL_GET_HELP:
			bootloader_handle_gethelp_cmd(BlRxBuffer);
			break;
		case BL_GET_CID:
			bootloader_handle_getcid_cmd(BlRxBuffer);
			break;
		case BL_GET_RDP_STATUS:
			bootloader_handle_getrdp_status_cmd(BlRxBuffer);
			break;
		case BL_GO_TO_ADDR:
			bootloader_handle_goto_addr_cmd(BlRxBuffer);
			break;
		case BL_FLASH_ERASE:
			bootloader_handle_flash_erase_cmd(BlRxBuffer);
			break;
		case BL_MEM_WRITE:
			bootloader_handle_memwrite_cmd(BlRxBuffer);
			break;
		case BL_EN_DIS_RW:
			bootloader_handle_endis_rw_cmd(BlRxBuffer);
			break;
		case BL_MEM_READ:
			bootloader_handle_memread_cmd(BlRxBuffer);
			break;
		case BL_READ_SECT_STS:
			bootloader_handle_read_sector_status_cmd(BlRxBuffer);
			break;
		case BL_OTP_READ:
			bootloader_handle_otpread_cmd(BlRxBuffer);
			break;
		default:
			printmsg("BL_DEBUG_MSG:Command is not correct.\r\n");
		}

	}
}

void bootloader_jump_to_user_app()
{
	printmsg(HelloToApp);

	// just a function pointer to hold the address of reset handler of user app
	void (*app_reset_handler)(void);

	// 0x80008000: 4 first bytes hold the msp
	// 0x80008004: next 4 bytes hold the reset_handler

	// 1. Config the MSP by reading the value from the base address of sector 2
	uint32_t msp_value = *(volatile uint32_t *) FLASH_SECTOR2_BASE_ADDRESS;
	printmsg("BL_DEBUG_MSG:MSP value: %#x\r\n", msp_value);

	__set_MSP(msp_value);

	// 2. Now fetch the reset handler address of the User application
	uint32_t reset_handler_address = *(volatile uint32_t *) (FLASH_SECTOR2_BASE_ADDRESS + 4);
	app_reset_handler = (void*)reset_handler_address;
	printmsg("BL_DEBUG_MSG:Reset handler address: %#x\r\n", reset_handler_address);

	// 3. Jump to reset handler of the application
	app_reset_handler();
}

static void bootloader_handle_getver_cmd(uint8_t * bl_rx_data)
{
	uint8_t bl_ver;

	// 1. verify checksum
	printmsg("BL_DEBUG_MSG:bootloader_handle_getver_cmd.\r\n");

	// total length of the cmd packet
	uint32_t cmd_packet_len = bl_rx_data[BL_DATA_LEN_IDX] + 1;

	// extract the CRC32 sent by host
	uint32_t host_crc = *((uint32_t *) (bl_rx_data + cmd_packet_len - BL_FOUR_BYTES));
	if (!bootloader_verify_crc(&bl_rx_data[BL_DATA_LEN_IDX], cmd_packet_len - BL_FOUR_BYTES, host_crc))
	{
		printmsg("BL_DEBUG_MSG: Checksum success.\r\n");
		bootloader_send_ack(bl_rx_data[BL_DATA_CMD_IDX], BL_ONE_BYTE);
		bl_ver = bootloader_get_version();
		printmsg("BL_DEBUG_MSG: BL_VER: %d %#x\r\n", bl_ver, bl_ver);
		bootloader_uart_write_data(&bl_ver, BL_ONE_BYTE);
	}
	else
	{
		printmsg("BL_DEBUG_MSG: Checksum failure.\r\n");
		bootloader_send_nack();
	}
}

static void bootloader_handle_gethelp_cmd(uint8_t * bl_rx_data)
{
  printmsg("BL_DEBUG_MSG:bootloader_handle_gethelp_cmd\r\n");

	//Total length of the command packet
	uint32_t command_packet_len = bl_rx_data[BL_DATA_LEN_IDX]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_data+command_packet_len - BL_FOUR_BYTES) ) ;

	if (! bootloader_verify_crc(&bl_rx_data[BL_DATA_LEN_IDX], command_packet_len-BL_FOUR_BYTES, host_crc))
	{
				printmsg("BL_DEBUG_MSG:Checksum success !!\r\n");
				bootloader_send_ack(bl_rx_data[BL_DATA_CMD_IDX],sizeof(supported_commands));
				bootloader_uart_write_data(supported_commands,sizeof(supported_commands) );

	}
	else
	{
				printmsg("BL_DEBUG_MSG:checksum fail !!\r\n");
				bootloader_send_nack();
	}
}

static void bootloader_handle_getcid_cmd(uint8_t * bl_rx_data)
{
	uint16_t cid;

	// 1. verify checksum
	printmsg("BL_DEBUG_MSG:bootloader_handle_getcid_cmd.\r\n");

	// total length of the cmd packet
	uint32_t cmd_packet_len = bl_rx_data[BL_DATA_LEN_IDX] + 1;

	// extract the CRC32 sent by host
	uint32_t host_crc = *((uint32_t *) (bl_rx_data + cmd_packet_len - BL_FOUR_BYTES));
	if (!bootloader_verify_crc(&bl_rx_data[BL_DATA_LEN_IDX], cmd_packet_len - BL_FOUR_BYTES, host_crc))
	{
		printmsg("BL_DEBUG_MSG: Checksum success.\r\n");
		bootloader_send_ack(bl_rx_data[BL_DATA_CMD_IDX], BL_TWO_BYTES);
		cid = get_mcu_chip_id();
		printmsg("BL_DEBUG_MSG: CID: %d %#x\r\n", cid, cid);
		bootloader_uart_write_data((uint8_t *)&cid, BL_TWO_BYTES);
	}
	else
	{
		printmsg("BL_DEBUG_MSG: Checksum failure.\r\n");
		bootloader_send_nack();
	}
}

static void bootloader_handle_getrdp_status_cmd(uint8_t * bl_rx_data)
{
	uint8_t rdp_status;

	// 1. verify checksum
	printmsg("BL_DEBUG_MSG:bootloader_handle_getrdp_status_cmd.\r\n");

	// total length of the cmd packet
	uint32_t cmd_packet_len = bl_rx_data[BL_DATA_LEN_IDX] + 1;

	// extract the CRC32 sent by host
	uint32_t host_crc = *((uint32_t *) (bl_rx_data + cmd_packet_len - BL_FOUR_BYTES));
	if (!bootloader_verify_crc(&bl_rx_data[BL_DATA_LEN_IDX], cmd_packet_len - BL_FOUR_BYTES, host_crc))
	{
		printmsg("BL_DEBUG_MSG: Checksum success.\r\n");
		bootloader_send_ack(bl_rx_data[BL_DATA_CMD_IDX], BL_ONE_BYTE);
		rdp_status = get_rdp_status();
		printmsg("BL_DEBUG_MSG: RDP status: %#x\r\n", rdp_status);
		bootloader_uart_write_data((uint8_t *)&rdp_status, BL_ONE_BYTE);
	}
	else
	{
		printmsg("BL_DEBUG_MSG: Checksum failure.\r\n");
		bootloader_send_nack();
	}
}

static void bootloader_handle_goto_addr_cmd(uint8_t * bl_rx_data)
{
	uint32_t go_address;
	uint8_t addr_valid = ADDRESS_VALID;
	uint8_t addr_invalid = ADDRESS_INVALID;

	// 1. verify checksum
	printmsg("BL_DEBUG_MSG:bootloader_handle_goto_addr_cmd.\r\n");

	// total length of the cmd packet
	uint32_t cmd_packet_len = bl_rx_data[BL_DATA_LEN_IDX] + 1;

	// extract the CRC32 sent by host
	uint32_t host_crc = *((uint32_t *) (bl_rx_data + cmd_packet_len - BL_FOUR_BYTES));
	if (!bootloader_verify_crc(&bl_rx_data[BL_DATA_LEN_IDX], cmd_packet_len - BL_FOUR_BYTES, host_crc))
	{
		printmsg("BL_DEBUG_MSG: Checksum success.\r\n");
		bootloader_send_ack(bl_rx_data[BL_DATA_CMD_IDX], BL_ONE_BYTE);

		// extract address
		go_address = *( (uint32_t*) &bl_rx_data[2] );
		printmsg("BL_DEBUG_MSG: Goto addr: %x\r\n", go_address);

		if (verify_address(go_address) == ADDRESS_VALID)
		{
			// tell host that address is fine
			bootloader_uart_write_data(&addr_valid, 1);
			go_address+=1; // make T bit=1
			void (*lets_jump)(void) = (void *)go_address;
			printmsg("BL_DEBUG_MSG: Jumping to address\r\n");
			lets_jump();
		}
		else
		{
			printmsg("BL_DEBUG_MSG: Address invalid.\r\n");
			bootloader_uart_write_data(&addr_invalid, 1);
		}
	}
	else
	{
		printmsg("BL_DEBUG_MSG: Checksum failure.\r\n");
		bootloader_send_nack();
	}
}

static void bootloader_handle_flash_erase_cmd(uint8_t * bl_rx_data)
{

	// 1. verify checksum
	printmsg("BL_DEBUG_MSG:bootloader_handle_flash_erase_cmd.\r\n");

	// total length of the cmd packet
	uint32_t cmd_packet_len = bl_rx_data[BL_DATA_LEN_IDX] + 1;

	// extract the CRC32 sent by host
	uint32_t host_crc = *((uint32_t *) (bl_rx_data + cmd_packet_len - BL_FOUR_BYTES));
	if (!bootloader_verify_crc(&bl_rx_data[BL_DATA_LEN_IDX], cmd_packet_len - BL_FOUR_BYTES, host_crc))
	{
		printmsg("BL_DEBUG_MSG: Checksum success.\r\n");
		bootloader_send_ack(bl_rx_data[BL_DATA_CMD_IDX], BL_ONE_BYTE);

		printmsg("BL_DEBUG_MSG: Erase_sector: %d\r\nBL_DEBUG_MSG: No_Sector: %d.\r\n", bl_rx_data[2], bl_rx_data[3]);
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
		uint8_t erase_status = execute_erase_flash(bl_rx_data[2], bl_rx_data[3]);
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);
		printmsg("BL_DEBUG_MSG: Erase_status %x.\r\n", erase_status);
		bootloader_uart_write_data((uint8_t *)&erase_status, BL_ONE_BYTE);
	}
	else
	{
		printmsg("BL_DEBUG_MSG: Checksum failure.\r\n");
		bootloader_send_nack();
	}
}

static void bootloader_handle_memwrite_cmd(uint8_t * bl_rx_data)
{
	uint8_t write_status = 0x00;
	// 1. verify checksum
	printmsg("BL_DEBUG_MSG:bootloader_handle_memwrite_cmd.\r\n");

	// total length of the cmd packet
	uint32_t cmd_packet_len = bl_rx_data[BL_DATA_LEN_IDX] + 1;

	// extract the CRC32 sent by host
	uint32_t host_crc = *((uint32_t *) (bl_rx_data + cmd_packet_len - BL_FOUR_BYTES));
	if (!bootloader_verify_crc(&bl_rx_data[BL_DATA_LEN_IDX], cmd_packet_len - BL_FOUR_BYTES, host_crc))
	{
		printmsg("BL_DEBUG_MSG: Checksum success.\r\n");
		bootloader_send_ack(bl_rx_data[0], BL_ONE_BYTE);

		uint32_t mem_addr = *(uint32_t *)(&bl_rx_data[2]);
		printmsg("BL_DEBUG_MSG: Programming at mem: %#x with data length %d.\r\n", mem_addr, bl_rx_data[6]);
		if (verify_address(mem_addr)  == ADDRESS_VALID)
		{
			printmsg("BL_DEBUG_MSG: Valid memory address.\r\n");

			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);

			write_status = execute_mem_write(&bl_rx_data[7], mem_addr, bl_rx_data[6]);

			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);
			printmsg("BL_DEBUG_MSG: Write memory status %x.\r\n", write_status);
			bootloader_uart_write_data((uint8_t *)&write_status, BL_ONE_BYTE);
		}
		else
		{
			printmsg("BL_DEBUG_MSG: Invalid memory address.\r\n");
			write_status = ADDRESS_INVALID;
			bootloader_uart_write_data((uint8_t *)&write_status, BL_ONE_BYTE);
		}

	}
	else
	{
		printmsg("BL_DEBUG_MSG: Checksum failure.\r\n");
		bootloader_send_nack();
	}
}

static void bootloader_handle_endis_rw_cmd(uint8_t * bl_rx_data)
{

}

static void bootloader_handle_memread_cmd(uint8_t * bl_rx_data)
{

}

static void bootloader_handle_read_sector_status_cmd(uint8_t * bl_rx_data)
{

}

static void bootloader_handle_otpread_cmd(uint8_t * bl_rx_data)
{

}

static uint8_t bootloader_get_version()
{
	return BL_VERSION;
}

static uint16_t get_mcu_chip_id(void)
{
	uint16_t cid;
	cid = (uint16_t) (DBGMCU->IDCODE) & 0x0FFF;
	return cid;
}

static uint8_t get_rdp_status(void)
{
	uint8_t rdp;
	rdp = (uint8_t) (((*(volatile uint32_t *)(READ_PROTECTION_OPTION_BYTE))>>8) & 0xFF);
	return rdp;
}

static uint8_t verify_address(uint32_t addr)
{
	uint8_t ret_val = ADDRESS_INVALID;
	if ((FLASH_BASE <= addr) && (FLASH_END >= addr))
	{
		ret_val = ADDRESS_VALID;
	}
	else if ((SRAM1_BASE <= addr) && (SRAM2_BASE > addr))
	{
		ret_val = ADDRESS_VALID;
	}
	else if ((SRAM2_BASE <= addr) && (SRAM3_BASE > addr))
	{
		ret_val = ADDRESS_VALID;
	}
	else
	{
		ret_val = ADDRESS_INVALID;
	}
	return ret_val;
}

static uint8_t execute_erase_flash(uint8_t sector, uint8_t num_sector)
{
	FLASH_EraseInitTypeDef flashEraseInit;
	uint32_t sector_error;
	HAL_StatusTypeDef status;

	if (num_sector > 8)
	{
		return SECTOR_INVALID;
	}
	// if sector == 0xff that mean mass erase
	if ((sector == 0xff) || (sector <=7))
	{
		if (sector == 0xff)
		{
			flashEraseInit.TypeErase = FLASH_TYPEERASE_MASSERASE;
		}
		else
		{
			uint8_t remain_sector = 8 - sector;
			if (num_sector > remain_sector)
			{
				num_sector = remain_sector;
			}
			flashEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
			flashEraseInit.Sector = sector;
			flashEraseInit.NbSectors = num_sector;
		}
		flashEraseInit.Banks = FLASH_BANK_1;

		// unlock FLASH register
		HAL_FLASH_Unlock();
		flashEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
		status = (uint8_t) HAL_FLASHEx_Erase(&flashEraseInit, &sector_error);
		HAL_FLASH_Lock();

		return status;
	}
	return SECTOR_INVALID;
}

static uint8_t execute_mem_write(uint8_t *pData, uint32_t mem_addr, uint32_t len)
{
	uint8_t status=HAL_OK;

	HAL_FLASH_Unlock();

	for (uint32_t i = 0; i < len; ++i) {
		status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, mem_addr+i, pData[i]);
	}
	HAL_FLASH_Lock();
	return status;
}

static void bootloader_send_ack(uint8_t cmd, uint8_t len)
{
	uint8_t ack_buffer[2];
	ack_buffer[0] = BL_ACK;
	ack_buffer[1] = len;
	HAL_UART_Transmit(C_UART, ack_buffer, BL_TWO_BYTES, HAL_MAX_DELAY);
}

static void bootloader_send_nack(void)
{
	uint8_t nack_buffer = BL_NACK;
	HAL_UART_Transmit(C_UART, &nack_buffer, BL_ONE_BYTE, HAL_MAX_DELAY);
}


static void bootloader_uart_write_data(uint8_t *pData, uint8_t len)
{
	HAL_UART_Transmit(C_UART, pData, len, HAL_MAX_DELAY);
}

static uint8_t bootloader_verify_crc(uint8_t *pData, uint32_t len, uint32_t crc_host)
{
	uint32_t uwCRCValue = 0xFF;
	for (uint32_t i = 0; i < len; ++i) {
		uint32_t i_data = pData[i];
		uwCRCValue = HAL_CRC_Accumulate(&hcrc, &i_data, 1);
	}
	 /* Reset CRC Calculation Unit */
	__HAL_CRC_DR_RESET(&hcrc);

	if (uwCRCValue == crc_host)
	{
		return VERIFY_CRC_SUCCESS;
	}
	return VERIFY_CRC_FAIL;
}

/* prints formatted string over USART */
void printmsg(char *format,...)
{
#ifdef BL_DEBUG_MSG
	char str[80];

	va_list args;
	va_start(args, format);
	vsprintf(str, format, args);
	HAL_UART_Transmit(D_UART, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
	va_end(args);
#endif
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
