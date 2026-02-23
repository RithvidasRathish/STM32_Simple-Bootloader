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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BL_DEBUG_MSG_EN	1
#define D_UART &huart1 /* Debug UART*/
#define C_UART &huart2 /* Virtual Communication UART - For BL Commands (C)*/

#define BL_RX_LEN 200


#define FLASH_BOOTLOADER 		0x08000000 /* Bank 1 - Boot loader 16KB*/
#define FLASH_FIRMWARE1 		0x0800C000 /* Bank 1 - 96KB */
#define FLASH_FIRMWARE2 		0x08020000 /* Bank 2 - 128KB */

/* FLASH Active Bank Macros */
#define FLASH_ACTIVE_BANK1 		1
#define FLASH_ACTIVE_BANK2		2

/* FLASH Page for Meta data*/

#define FLASH_METADATA_BASEADDR	0x08008000


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t bl_rx_buffer[BL_RX_LEN]; /* Buffer for general boot loader communications */

uint8_t supported_commands[] = {
                                 BL_GET_VER ,
                                 BL_GET_HELP,
                                 BL_GET_CID,
                                 BL_GET_RDP_STATUS,
                                 BL_GO_TO_ADDR,
                                 BL_FLASH_ERASE,
                                 BL_MEM_WRITE,
                                 BL_READ_SECTOR_P_STATUS,
								 BL_SHOW_ACTIVE_BANK};

uint8_t active_bank_number; /*Variable to store active bank number ACTIVE_BANK_1 or ACTIVE_BANK_2*/
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void boot_manager(void);
static void launch_application(void);
static void start_update_flow(void);
static uint8_t user_requested_update(void);
static uint8_t update_button_active(void);
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
  MX_CRC_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  active_bank_number = fetch_active_bank_number();
  printmsg("BL_DEBUG_MSG: Active Bank: %d \n\r", active_bank_number);

  boot_manager();


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void printmsg(char *format, ...){/* The ... (ellipsis) means that more arguments can follow */

#ifdef BL_DEBUG_MSG_EN

	char str[80];
	/*Extract the argument list using C VA APIs*/
	va_list args;
	va_start(args, format);
	vsprintf(str, format, args);
	HAL_UART_Transmit(D_UART, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
	va_end(args);
#endif
}

static uint8_t update_button_active(void)
{
    return (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_SET);
}

static uint8_t user_requested_update(void)
{
    uint8_t response;

    printmsg("BL_DEBUG_MSG: Would you like to update the firmware? (Y/N): \n\r");

    if (HAL_UART_Receive(D_UART, &response, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        printmsg("BL_DEBUG_MSG: No Response Received\n\r");
        return 0;
    }

    return (response == 'Y' || response == 'y');
}

static void launch_application(void)
{
	active_bank_number = fetch_active_bank_number();
	printmsg("BL_DEBUG_MSG: Active Bank: %d \n\r", active_bank_number);

    bootloader_jump_to_active_bank();
}

static void start_update_flow(void)
{
	printmsg("BL_DEBUG_MSG: Starting firmware Upgrade \n\r");

    handle_firmware_update();

//    printmsg("Update complete\r\n");

    HAL_Delay(5000);

    printmsg("BL_DEBUG_MSG: Jumping To User Application \n\r");


}


static void boot_manager(void)
{
    if (!update_button_active())
    {
    	printmsg("BL_DEBUG_MSG: Normal Boot Selected\n\r");
        launch_application();
        return;
    }

    else{

    	printmsg("BL_DEBUG_MSG: Bootloader Mode Entered\n\r");

    if (user_requested_update())
    {
        start_update_flow();
        launch_application();     // Centralized jump
    }
    else
    {
    	printmsg("BL_DEBUG_MSG: Entering Command Interface\n\r");
        bootloader_command_handler();
    }

    }
}

void bootloader_jump_to_active_bank()
{
	/* Flow of the function:
	 * 1. Reads the applications’s initial MSP and sets it.
	 * 2. Redirects interrupts by re-mapping VTOR.
	 * 3. Fetches the application’s Reset_Handler address.
	 * 4. Calls it, effectively jumping to the user application.
	 */

	printmsg("BL_DEBUG_MSG: bootloader_jump_to_user_app\n\r");

	/*The first byte at address of the FLASH area holds the value of MSP and next byte holds the Reset Handler
	 * according to ARM-Cortex Architecture */

	/*1. Configure the Main Stack Pointer (MSP) by reading the value form the flash base address of desired sector*/
	/* Check which firmware bank is active and run active firmware bank*/

	uint32_t resethandler_address;

	if(active_bank_number == FLASH_ACTIVE_BANK1)
	{

		printmsg("BL_DEBUG_MSG: Firmware Bank 1 Active. \n\r");

		uint32_t msp_value = *(volatile uint32_t*)FLASH_FIRMWARE1;

		/* Set MSP function from CMSIS*/
		 __disable_irq();

		    /* Stop SysTick */
		    SysTick->CTRL = 0;
		    SysTick->LOAD = 0;
		    SysTick->VAL  = 0;
		__set_MSP(msp_value);

		/* Re-map vector table to user application base address */
		SCB->VTOR = FLASH_FIRMWARE1; /* System Control Block - Vector Table Offset Register */

		/* Fetch the reset handler address of the user application
		* from the location FIRMWARE_BASE_ADDRESS + 4 (32bits) */
		resethandler_address = *(volatile uint32_t*)(FLASH_FIRMWARE1 + 4);

	}else if(active_bank_number == FLASH_ACTIVE_BANK2){


		printmsg("BL_DEBUG_MSG: Firmware Bank 2 Active. \n\r");

		uint32_t msp_value = *(volatile uint32_t*)FLASH_FIRMWARE2;

		__disable_irq();

		/* Stop SysTick */
		SysTick->CTRL = 0;
		SysTick->LOAD = 0;
		SysTick->VAL  = 0;

		__set_MSP(msp_value);

		SCB->VTOR = FLASH_FIRMWARE2;

		resethandler_address = *(volatile uint32_t*)(FLASH_FIRMWARE2 + 4);
	}
	else{
		/*If values fetched from FLASH Meta data page is not 0 or 1*/
		printmsg("BL_DEBUG_MSG: Firmware Bank Error! \n\r");
	}

	void (*app_reset_handler)(void); /*A function pointer to hold the address of reset handler*/
	app_reset_handler = (void*) resethandler_address;

	/*3. Jumping to the reset handler of user application - Now this address will be loaded into the Program Counter*/
	app_reset_handler();

}

void bootloader_command_handler()
{
	uint8_t rcv_len = 0;

	while(1)
	{
		memset(bl_rx_buffer, 0, BL_RX_LEN);

		/*First read only one byte, which is the length. Then read the other bytes from UART */
		HAL_UART_Receive(C_UART, (uint8_t*)&bl_rx_buffer, 1, HAL_MAX_DELAY);
		rcv_len = bl_rx_buffer[0];
		/*Receive the command*/
		HAL_UART_Receive(C_UART, (uint8_t*)&bl_rx_buffer[1], rcv_len, HAL_MAX_DELAY);
		printmsg("BL_DEBUG_MSG: Received CMD: 0x%02X\r\n", bl_rx_buffer[1]);

		switch(bl_rx_buffer[1]) /* To check for command codes */
		{
		case BL_GET_VER:
			bootloader_handle_getver_cmd(bl_rx_buffer);
			break;
		case BL_GET_HELP:
			bootloader_handle_gethelp_cmd(bl_rx_buffer);
			break;
		case BL_GET_CID:
			bootloader_handle_getcid_cmd(bl_rx_buffer);
			break;
		case BL_GET_RDP_STATUS:
			bootloader_handle_getrdp_cmd(bl_rx_buffer);
			break;
		case BL_FLASH_ERASE:
			bootloader_handle_flash_erase_cmd(bl_rx_buffer);
			break;
		case BL_MEM_WRITE:
			bootloader_handle_mem_write_cmd(bl_rx_buffer);
			break;
		case BL_EN_RW_PROTECT:
			bootloader_handle_en_rw_protect(bl_rx_buffer);
			break;
		case BL_DIS_R_W_PROTECT:
			bootloader_handle_dis_rw_protect(bl_rx_buffer);
			break;
		case BL_SHOW_ACTIVE_BANK:
			bootloader_show_active_bank(bl_rx_buffer);
			break;
		default:
			printmsg("BL_DEBUG_MSG:Invalid command code received from host \n\r");
			break;

		}

	}
}

/******************* Boot loader handler functions *******************/

void bootloader_handle_getver_cmd(uint8_t *bl_rx_buffer)
{
    /* Handle "Get Version" command */
	uint8_t bl_version;

	uint32_t command_packet_len = bl_rx_buffer[0] + 1; /*Length to follow + First byte*/

	/*Extract the 4 bytes of CRC32 sent by the host*/
	uint32_t host_crc = *((uint32_t*)(bl_rx_buffer + command_packet_len - 4)); /* CRC is always 32 bits (4 bytes) here */

	/*Verify checksum*/
	printmsg("BL_DEBUG_MSG: bootloader_handle_getver_cmd\n\r");
	if(! bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc))
	{
		/*Checksum is correct*/
		printmsg("BL_DEBUG_MSG: Checksum success...!\n\r");
		bootloader_send_ack(1);
		bl_version = get_bootloader_version();
		printmsg("BL_DEBUG_MSG: BL_VER: %d ,%x\n\r", bl_version, bl_version);
		bootloader_uart_write_data(&bl_version, 1); /* Sends data back to the HOST */

	}else{
		printmsg("BL_DEBUG_MSG: Checksum failed...!\n\r");
		bootloader_send_nack();

	}

}

void bootloader_handle_gethelp_cmd(uint8_t *bl_rx_buffer)
{
    /* Handle "Get Help" command */
	printmsg("BL_DEBUG_MSG:bootloader_handle_gethelp_cmd\n\r");

	uint32_t command_packet_len = bl_rx_buffer[0] + 1;

	uint32_t host_crc = *((uint32_t*)(bl_rx_buffer + command_packet_len - 4));

	if(! bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc))
	{
		printmsg("BL_DEBUG_MSG:checksum success !!\n\r");
		bootloader_send_ack(sizeof(supported_commands));
		bootloader_uart_write_data(supported_commands, sizeof(supported_commands));

	}else{
		printmsg("BL_DEBUG_MSG:checksum fail !!\n\r");
		bootloader_send_nack();
	}
}

void bootloader_handle_getcid_cmd(uint8_t *bl_rx_buffer)
{
    /* Handle "Get Chip ID" command */
	printmsg("BL_DEBUG_MSG:bootloader_handle_getcid_cmd\n\r");

	uint16_t bl_cid_num = 0;

	uint32_t command_packet_len = bl_rx_buffer[0] + 1;

	uint32_t host_crc = *((uint32_t*)(bl_rx_buffer + command_packet_len - 4));

	if(! bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc))
	{
		printmsg("BL_DEBUG_MSG:checksum success !!\n\r");
		bootloader_send_ack(2);
		bl_cid_num = get_mcu_chip_id();
		printmsg("BL_DEBUG_MSG:MCU id : %d %#x !!\n\r",bl_cid_num, bl_cid_num);
		bootloader_uart_write_data((uint8_t*)&bl_cid_num, 2);
	}else{
		printmsg("BL_DEBUG_MSG:checksum fail !!\n\r");
		bootloader_send_nack();
	}
}

void bootloader_handle_getrdp_cmd(uint8_t *bl_rx_buffer)
{
    /* Handle "Get Read Protection Level" command */
	printmsg("BL_DEBUG_MSG:bootloader_handle_getrdp_cmd\n\r");

	uint8_t rdp_level = 0x00;

	//Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
		printmsg("BL_DEBUG_MSG:checksum success !!\n\r");
		bootloader_send_ack(1);
		rdp_level = get_flash_rdp_level();
		printmsg("BL_DEBUG_MSG:RDP level: %d %#x\n\r",rdp_level, rdp_level);
		bootloader_uart_write_data(&rdp_level, 1);

	}else{
		printmsg("BL_DEBUG_MSG:checksum fail !!\n\r");
		bootloader_send_nack();
	}
}

void bootloader_handle_flash_erase_cmd(uint8_t *pBuffer)
{
    uint8_t erase_status = 0x00;
    uint32_t command_packet_len;
    uint32_t host_crc;

    printmsg("BL_DEBUG_MSG: bootloader_handle_flash_erase_cmd\n\r");

    /* Get total command length and host CRC */
    command_packet_len = bl_rx_buffer[0] + 1;
    host_crc = *((uint32_t *)(bl_rx_buffer + command_packet_len - 4));

    /* Verify CRC */
    if (!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc))
    {
        printmsg("BL_DEBUG_MSG: CRC check success.\n\r");
        bootloader_send_ack(1);

        uint32_t sector_number     = pBuffer[2];
        uint32_t number_of_sectors = pBuffer[3];

        printmsg("BL_DEBUG_MSG: Sector=%ld  Number of Sectors=%ld\n\r", sector_number, number_of_sectors);

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);

        /* Mass erase check */
        if (sector_number == 0xFF)
        {
            /* Convert to sentinel value used in execute_flash_erase() */
            sector_number = 0xFFFFFFFFU;
        }

        erase_status = execute_flash_erase(sector_number, number_of_sectors);

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

        printmsg("BL_DEBUG_MSG: flash erase status: %#x\n\r", erase_status);

        /* Send erase result to host */
        bootloader_uart_write_data(&erase_status, 1);
    }
    else
    {
        printmsg("BL_DEBUG_MSG: CRC check failed!\n\r");
        bootloader_send_nack();
    }
}



void bootloader_handle_mem_write_cmd(uint8_t *bl_rx_buffer)
{
	/* Handle "Memory Write" command */

	printmsg("BL_DEBUG_MSG:bootloader_handle_mem_write_cmd\n\r");

	uint8_t write_status = 0x00;
	uint8_t payload_len = bl_rx_buffer[6];

	uint32_t mem_addr = *((uint32_t*)(&bl_rx_buffer[2]));

	uint32_t command_packet_len = bl_rx_buffer[0] + 1 ;
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer + command_packet_len - 4));

	if(! bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len -4, host_crc))
	{
		printmsg("BL_DEBUG_MSG:checksum success !!\n\r");
		bootloader_send_ack(1);
		printmsg("BL_DEBUG_MSG: mem write address : %#x\n\r",mem_addr);

		if( verify_address(mem_addr) == ADDR_VALID ) {


			printmsg("BL_DEBUG_MSG: valid mem write address\n\r");

			//glow the led to indicate bootloader is currently writing to memory
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,1);

			/* Execute memory write*/
			write_status = execute_mem_write(&bl_rx_buffer[7], mem_addr, payload_len);

			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,0);
			printmsg("BL_DEBUG_MSG: Write Status : %x\n\r",write_status);

			//inform host about the status
			bootloader_uart_write_data(&write_status,1);

		}else{
			printmsg("BL_DEBUG_MSG: invalid mem write address\n\r");
			write_status = ADDR_INVALID;
			//inform host that address is invalid
			bootloader_uart_write_data(&write_status,1);

		}
	}else{
		printmsg("BL_DEBUG_MSG:checksum fail !!\n\r");
		bootloader_send_nack();
	}
}


void bootloader_handle_en_rw_protect(uint8_t *bl_rx_buffer)
{
    /* Handle "Enable Read/Write Protection" command */
}

void bootloader_handle_dis_rw_protect(uint8_t *bl_rx_buffer)
{
    /* Handle "Disable Read/Write Protection" command */
}

void bootloader_send_nack(void)
{
	uint8_t nack = BL_NACK;
	HAL_UART_Transmit(C_UART, (uint8_t*)&nack, 1, HAL_MAX_DELAY);
}

void bootloader_send_ack(uint8_t follow_len)
{
	/* 2 Bytes are sent, first byte is ACK and second is the length value */
	uint8_t ack_buf[2];
	ack_buf[0] = BL_ACK;
	ack_buf[1] = follow_len;
	HAL_UART_Transmit(C_UART, (uint8_t*)&ack_buf, 2, HAL_MAX_DELAY);

}

uint8_t bootloader_verify_crc(uint8_t *pData, uint32_t len, uint32_t crc_host)
{
	uint32_t uwCRCValue = 0xff;

	for(int i=0; i<len; i++)
	{
		uint32_t i_data = pData[i];
		uwCRCValue = HAL_CRC_Accumulate(&hcrc, &i_data, 1);
	}

	/* Reset CRC Calculation Unit */
	__HAL_CRC_DR_RESET(&hcrc);

	if(uwCRCValue == crc_host)
		return VERIFY_CRC_SUCCESS;
	return VERIFY_CRC_FAIL;

}

void bootloader_uart_write_data(uint8_t *pBuffer,uint32_t len)
{
	HAL_UART_Transmit(C_UART, pBuffer, len, HAL_MAX_DELAY);
}

uint8_t get_bootloader_version()
{
	return (uint8_t)BL_VERSION;
}

uint16_t get_mcu_chip_id(void)
{
	/* RM: The STM32L47x/L48x/L49x/L4Ax MCUs integrate an MCU ID code. This ID identifies the
	   ST MCU part-number and the die revision. It is part of the DBG_MCU component and is
	   mapped on the external PPB bus (see Section 48.16 on page 1832). This code is
	   accessible using the JTAG debug port (4 to 5 pins) or the SW debug port (two pins) or by
	   the user software. It is even accessible while the MCU is under system reset.
	 */

	uint16_t cid;
	/* Reading the register and masking the unnecessary bits */
	cid = (uint16_t)(DBGMCU->IDCODE) & 0x0FFF;
	return cid;
}

uint8_t get_flash_rdp_level(void)
{
	/* !!! VERY IMPORTANT !!!! */
	/*
	 * LEVEL 2 (No debug mode): Option bytes cannot be programmed nor erased. Thus, the level 2 cannot be removed at all:
	 * it is an IRREVERSIBLE operation. 'DO NOT USE' Level 2, it is for end user products.
	 *
	 * When decreased from 'Level 1 to Level 0', the FLASH goes into MASS ERASE.
	 *
	 * */
	/*HAL Implementation*/
	uint8_t rdp_status = 0;
	FLASH_OBProgramInitTypeDef ob_handle;
	HAL_FLASHEx_OBGetConfig(&ob_handle);
	rdp_status = (uint8_t)ob_handle.RDPLevel;
	return rdp_status;
}


uint8_t verify_address(uint32_t go_address)
{
	/*Jump to FLASH, System Memory, SRAM1, SRAM2, Backup SRAM allowed,
	 * others such as peripheral memory not allowed*/

	if ( go_address >= SRAM1_BASE && go_address <= SRAM1_END) {

		return ADDR_VALID;
//	} else if ( go_address >= SRAM2_BASE && go_address <= SRAM2_END) {
//
//		return ADDR_VALID;
	} else if ( go_address >= FLASH_BASE && go_address <= FLASH_END) {

		return ADDR_VALID;
	}else
	return ADDR_INVALID;
}

uint8_t execute_flash_erase(uint8_t sector_number, uint8_t number_of_sectors) {
	//There are 8 sectors in STM32F446RE MCU (Sector[0:7])
	//number_of_sectors must be in the range of 0 to 7
	//if sector_number = 0xFF, that means mass erase

	FLASH_EraseInitTypeDef flashErase_handle;
	uint32_t sectorError;
	HAL_StatusTypeDef status;

	if(number_of_sectors > 5)
		return INVALID_SECTOR;

	if((sector_number == 0xFF) || (sector_number <= 5)) {

		if(sector_number == (uint8_t) 0xFF) {
			flashErase_handle.TypeErase = FLASH_TYPEERASE_MASSERASE;

		} else {
			//Calculate how many sectors need to be erased
			uint8_t remaining_sector = 6 - sector_number;

			if(number_of_sectors > remaining_sector) {
				number_of_sectors = remaining_sector;
			}

			flashErase_handle.TypeErase = FLASH_TYPEERASE_SECTORS;
			flashErase_handle.Sector = sector_number; //The initial sector
			flashErase_handle.NbSectors = number_of_sectors;
		}

//		flashErase_handle.Banks = FLASH_BANK_1;

		//Gain access to modify the FLASH registers
		HAL_FLASH_Unlock();
		flashErase_handle.VoltageRange = FLASH_VOLTAGE_RANGE_3;
		status = (uint8_t) HAL_FLASHEx_Erase(&flashErase_handle, &sectorError);
		HAL_FLASH_Lock();

		return status;
	}

	return INVALID_SECTOR;
}


uint8_t execute_mem_write(uint8_t *pBuffer, uint32_t mem_address, uint32_t len)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Flash word programming requires 4-byte alignment */
    if ((mem_address & 0x3U) != 0U)
        return (uint8_t)HAL_ERROR;

    if (len == 0U)
        return (uint8_t)HAL_OK;

    HAL_FLASH_Unlock();

    for (uint32_t offset = 0; offset < len; offset += 4U)
    {
        uint32_t data32 = 0xFFFFFFFFU;
        uint32_t chunk = ((len - offset) >= 4U) ? 4U : (len - offset);

        /* Pack bytes (little-endian) */
        for (uint32_t b = 0U; b < chunk; b++)
        {
            data32 &= ~(0xFFUL << (8U * b));  // clear byte
            data32 |= ((uint32_t)pBuffer[offset + b]) << (8U * b);
        }

        status = HAL_FLASH_Program(
                     FLASH_TYPEPROGRAM_WORD,
                     mem_address + offset,
                     data32);

        if (status != HAL_OK)
            break;
    }

    HAL_FLASH_Lock();
    return (uint8_t)status;
}


uint8_t fetch_available_firmware_version(void)
{


	uint8_t version_request_command = 0x99;
	uint8_t available_version;
	bootloader_uart_write_data(&version_request_command, 1);
	HAL_UART_Receive(C_UART, &available_version, 1, HAL_MAX_DELAY);
	/*TODO: Add CRC verification for the received function*/
	return available_version;
}

uint8_t handle_firmware_update(void)
{
	uint8_t write_status = 0x00;
	uint8_t erase_status = 0x00;
		/* Find the inactive bank address and corresponding page numbers*/
	uint32_t inactive_bank_adress = (active_bank_number == FLASH_ACTIVE_BANK1) ? FLASH_FIRMWARE2 : FLASH_FIRMWARE1;
//	uint32_t inactive_page_number = (active_bank_number == FLASH_ACTIVE_BANK1) ? 256 : 16;
	uint8_t inactive_bank_number = (active_bank_number == FLASH_ACTIVE_BANK1) ? FLASH_ACTIVE_BANK2 : FLASH_ACTIVE_BANK1;

	printmsg("BL_DEBUG_MSG: Downloading binaries to inactive bank: %d \n\r", inactive_bank_number);

	uint8_t update_request = BL_FW_UPDATE_REQUIRED;
	bootloader_uart_write_data(&update_request, 1);

	uint32_t current_address = inactive_bank_adress;

	uint8_t sector_number = (active_bank_number == FLASH_ACTIVE_BANK1) ? 5 : 3;
	uint8_t number_of_sectors = (active_bank_number == FLASH_ACTIVE_BANK1) ? 1 : 2;

	erase_status = execute_flash_erase(sector_number, number_of_sectors);


	printmsg("BL_DEBUG_MSG: flash erase status: %#x\n\r", erase_status);

	/*TODO: Need a continuous loop here to continuously get the pay load and write*/
	while(1)
	{
		memset(bl_rx_buffer, 0, BL_RX_LEN);
		HAL_UART_Receive(C_UART, (uint8_t*)&bl_rx_buffer, 1, HAL_MAX_DELAY);
		if(bl_rx_buffer[0] == 250)
		{
			printmsg("BL_DEBUG_MSG: Download Complete! \n\r");
			break;
		}
		uint32_t rcv_len = bl_rx_buffer[0];
		HAL_UART_Receive(C_UART, (uint8_t*)&bl_rx_buffer[1], rcv_len, HAL_MAX_DELAY);

		/* Get the length and check if new firmware fit into the banks, <= 480KB (in terms of words) TODO: Add size check, verification, roll back, other features*/
		// uint8_t write_status = 0x00;
		uint8_t payload_len = bl_rx_buffer[6];

		/* Erase the Inactive bank */
		//execute_flash_erase(inactive_page_number , 240);
		//printmsg("Inactive bank erased. Ready to write. \n\r");

		/* Download onto Inactive bank */
		write_status = execute_mem_write(&bl_rx_buffer[7], current_address, payload_len);


		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,0);
		printmsg("BL_DEBUG_MSG: Write Status : %x\n\r",write_status);

					//inform host about the status
		bootloader_uart_write_data(&write_status,1);
		current_address += payload_len;

	}

	/*Update the active bank number in FLASH */
	uint8_t num_status = update_active_bank_number(inactive_bank_number);

	return num_status;
}

void bootloader_show_active_bank(uint8_t *pBuffer)
{
	/* Variable to store the active firmware bank number - To be preserved even after powering off
	 * One method: one dedicated page (2KB) in FLASH for configuration data - meta-data*/
	    uint8_t active_bank_number;

	    uint32_t command_packet_len = bl_rx_buffer[0] + 1; /*Length to follow + First byte*/

		/*Extract the 4 bytes of CRC32 sent by the host*/
		uint32_t host_crc = *((uint32_t*)(bl_rx_buffer + command_packet_len - 4)); /* CRC is always 32 bits (4 bytes) here */

		/*Verify checksum*/
		printmsg("BL_DEBUG_MSG: bootloader_handle_show_active_bank_cmd\n\r");
		if(! bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc))
		{
			/*Checksum is correct*/
			printmsg("BL_DEBUG_MSG: Checksum success...!\n\r");
			bootloader_send_ack(1);
			active_bank_number = fetch_active_bank_number();
			printmsg("BL_DEBUG_MSG: Active Bank : %d \n\r", active_bank_number);
			bootloader_uart_write_data(&active_bank_number, 1); /* Sends data back to the HOST */

		}else{
			printmsg("BL_DEBUG_MSG: Checksum failed...!\n\r");
			bootloader_send_nack();

		}


}

uint8_t update_active_bank_number(uint8_t active_bank)
{
	/*Update the active bank number in FLASH page, after Firmware Update*/
	uint8_t write_status = 0x00;
	   /* Flash metadata must be 4-byte aligned */
	 if ((FLASH_METADATA_BASEADDR & 0x3U) != 0U)
	        return HAL_ERROR;

	FLASH_EraseInitTypeDef flashErase_handle;
	uint32_t pageError;
	HAL_StatusTypeDef status;

	HAL_FLASH_Unlock();

	flashErase_handle.TypeErase = FLASH_TYPEERASE_SECTORS;
	flashErase_handle.Sector = 2; //The initial sector
	flashErase_handle.NbSectors = 1;
	flashErase_handle.VoltageRange = FLASH_VOLTAGE_RANGE_3;



	status = HAL_FLASHEx_Erase(&flashErase_handle, &pageError);
	if (status == HAL_ERROR) return HAL_ERROR;
//	printmsg("BL_DEBUG_MSG: Erase Status : %x\n\r",status);
//
//							//inform host about the status
//				bootloader_uart_write_data(&status,1);



//	write_status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLASH_METADATA_BASEADDR, (uint64_t)active_bank);
	 /* Prepare 32-bit word (pad with erased state) */
	    uint32_t meta_word = 0xFFFFFFFFU;
	    meta_word &= ~0xFFU;
	    meta_word |= (uint32_t)active_bank;

	    write_status = HAL_FLASH_Program(
	                 FLASH_TYPEPROGRAM_WORD,
	                 FLASH_METADATA_BASEADDR,
	                 meta_word);



			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,0);
			printmsg("BL_DEBUG_MSG: Write Status : %x  %d\n\r",write_status,active_bank);

						//inform host about the status
			bootloader_uart_write_data(&write_status,1);





	HAL_FLASH_Lock();

	return status;

}

uint8_t fetch_active_bank_number(void)
{
	/*Fetch the active bank details from dedicated FLASH meta data page*/
	uint64_t retrieved_data;
	retrieved_data = *(const uint64_t*) FLASH_METADATA_BASEADDR;

	return (uint8_t)retrieved_data;
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
