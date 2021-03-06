/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include <stdarg.h>
#include <string.h>
#include <stdio.h>

#include "main.h"
#include "stm32f4xx_hal.h"

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
#define BL_DEBUG_MSG_EN

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

#define D_UART &huart3
#define C_UART &huart2

#define BL_RX_LEN (200)
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
static void printmsg(char *format,...);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

char data_buffer[] = "Hello from Bootloader\r\n";
uint8_t bl_rx_buffer[BL_RX_LEN];

uint8_t supported_cmd[] = {0x51,0x52,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5a,0x5b};


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
  MX_CRC_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
	if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET)
	{
		printmsg("BL_DEBUG_MSG_EN: Button is pressed.. going to BL mode \r\n");	
		bootloader_uart_read_data();
	}
	else
	{
			printmsg("BL_DEBUG_MSG_EN: Button is not pressed.. executing User App \r\n");	
		bootloader_jump_to_user_app();
	}

}

void bootloader_uart_read_data(void)
{
	uint8_t rcv_len = 0;
	
	while(1)
	{
		memset(bl_rx_buffer,0,BL_RX_LEN);
		/* Here is read and decode the command coming from host
		 * first read only one bute from host, which is he length
		 * fied of the command*/
		HAL_UART_Receive(C_UART,bl_rx_buffer,1,HAL_MAX_DELAY);
		rcv_len = bl_rx_buffer[0];
		HAL_UART_Receive(C_UART,&bl_rx_buffer[1],rcv_len,HAL_MAX_DELAY);
		
		switch(bl_rx_buffer[1])
		{
			case BL_GET_VER:
				bootlader_handle_getver_cmd(bl_rx_buffer);
				break;
			case BL_GET_HELP:
				bootlader_handle_gethelp_cmd(bl_rx_buffer);
				break;
			case BL_GET_CID:
				bootlader_handle_getcid_cmd(bl_rx_buffer);
			 break;
			case BL_GET_RDP_STATUS:
			  bootlader_handle_getrdp_cmd(bl_rx_buffer);
				break;
			case BL_GO_TO_ADDR:
		    bootlader_handle_go_cmd(bl_rx_buffer);
				break;
			case BL_FLASH_ERASE:
				bootlader_handle_flash_erase_cmd(bl_rx_buffer);
				break;
			case BL_MEM_WRITE:
				bootlader_handle_mem_write_cmd(bl_rx_buffer);
				break;
			case BL_ENDIS_RW_PROTECT:
				bootlader_handle_endis_rw_protect_cmd(bl_rx_buffer);
				break;
			case BL_MEM_READ:
				bootlader_handle_mem_read_cmd(bl_rx_buffer);
				break;
			case BL_READ_SECTOR_STATUS:
				bootlader_handle_read_sector_status_cmd(bl_rx_buffer);
				break;
			case BL_OTP_READ:
				bootlader_handle_read_otp_cmd(bl_rx_buffer);
				break;
			case BL_DIS_R_W_PROTECT:
				bootlader_handle_dis_rw_protect_cmd(bl_rx_buffer);
				break;
			default:
				printmsg("BL_DEBUG_MSG_EN: Invalid command code received from host \r\n");	
				break;
		}
	}
	
}
	
/* Code to jump tp User App, here is assumed that 
 * FLASH_SECTOR2_BASE_ADDRESS is where the User App is stored*/
void bootloader_jump_to_user_app(void)
{
	/* Function pointer to hold address of the reset handler of User App*/
	void(*app_reset_handler)(void);
	
	printmsg("BL_DEBUG_MSG_EN: bootloader_jump_to_user_app \r\n");
	
	/* 1. Configure the MSP by reading the value from the base address of sector 2*/
	uint32_t msp_value = *(volatile uint32_t *)FLASH_SECTOR2_BASE_ADDRESS;
	printmsg("BL_DEBUG_MSG_EN: MSP value %#x \r\n", msp_value);
	
	/*This function comes from CMSIS*/
	__set_MSP(msp_value);
	
	/* 2. Now fetch the reset handler address of the user app
	 * from the location FLASH_SECTOR2_BASE_ADDRESS+4 */
	
	uint32_t resethandler_address = *(volatile uint32_t *)(FLASH_SECTOR2_BASE_ADDRESS +4);
	app_reset_handler = (void *) resethandler_address;
	printmsg("BL_DEBUG_MSG_EN: app reset handler address %#x \r\n", app_reset_handler);
	
	/* 3. Jump to reset hanlder of the User App */
	app_reset_handler();
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/* Prints formatted string to console over UART */
void printmsg(char *format,...)
{
#ifdef BL_DEBUG_MSG_EN
	char str[80];
  va_list args;
	va_start(args, format);
	vsprintf(str, format, args);
	HAL_UART_Transmit(C_UART, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
	va_end(args);
#endif
}

/********** IMPLEMENTATION OF BOOTLADER COMMAND HANDLE FUNCTIONS *********/

/* Helper function to handle BL_GETVER command */
void bootlader_handle_getver_cmd(uint8_t *bl_rx_buffer)
{
 uint8_t bl_version;
	
	/* 1. Verify the checksum */
	printmsg("BL_DEBUG_MSG_EN: bootlader_getver_cmd \r\n");
	/* total length of the command packet */
	uint32_t command_packet_len = bl_rx_buffer[0]+1;
	/* extract the CRC32 sent by the host */
	uint32_t host_crc = *((uint32_t *)(bl_rx_buffer + command_packet_len - 4));
	if(!bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len - 4,host_crc))
	{
	printmsg("BL_DEBUG_MSG_EN: Checksum success!! \r\n");	
		/* Checksum is correct*/
		bootloader_send_ack(bl_rx_buffer[0],1);
		bl_version = get_bootloader_ver();
		printmsg("BL_DEBUG_MSG_EN: BL_VER %d %#x\r\n", bl_version, bl_version);
		bootloader_uart_write_data(&bl_version,1);
	}
	else
	{
		printmsg("BL_DEBUG_MSG_EN: Checksum fails !!\r\n");	
		/*Cheksum is wrong send nack*/
		bootloader_send_nack();
	}
}
void bootlader_handle_gethelp_cmd(uint8_t *pBuffer)
{
	/* 1. Verify the checksum */
	printmsg("BL_DEBUG_MSG_EN: bootlader_handle_gethelp_cmd \r\n");
	/* total length of the command packet */
	uint32_t command_packet_len = pBuffer[0]+1;
	/* extract the CRC32 sent by the host */
	uint32_t host_crc = *((uint32_t *)(pBuffer + command_packet_len - 4));
	if(!bootloader_verify_crc(&pBuffer[0],command_packet_len - 4,host_crc))
	{
	printmsg("BL_DEBUG_MSG_EN: Checksum success!! \r\n");	
		/* Checksum is correct*/
		bootloader_send_ack(pBuffer[0],sizeof(supported_cmd));
		bootloader_uart_write_data(supported_cmd,sizeof(supported_cmd));
	}
	else
	{
		printmsg("BL_DEBUG_MSG_EN: Checksum fails !!\r\n");	
		/*Cheksum is wrong send nack*/
		bootloader_send_nack();
	}
}
void bootlader_handle_getcid_cmd(uint8_t *pBuffer)
{
	uint16_t cid;
	/* 1. Verify the checksum */
	printmsg("BL_DEBUG_MSG_EN: bootlader_handle_getcid_cmd \r\n");
	/* total length of the command packet */
	uint32_t command_packet_len = pBuffer[0]+1;
	/* extract the CRC32 sent by the host */
	uint32_t host_crc = *((uint32_t *)(pBuffer + command_packet_len - 4));
	if(!bootloader_verify_crc(&pBuffer[0],command_packet_len - 4,host_crc))
	{
	printmsg("BL_DEBUG_MSG_EN: Checksum success!! \r\n");	
		/* Checksum is correct*/
		bootloader_send_ack(pBuffer[0],2);
	  cid = get_mcu_chip_id();
	printmsg("BL_DEBUG_MSG_EN: ID %d %#x \r\n", cid, cid);			
		bootloader_uart_write_data((uint8_t*)&cid,2);
	}
	else
	{
		printmsg("BL_DEBUG_MSG_EN: Checksum fails !!\r\n");	
		/*Cheksum is wrong send nack*/
		bootloader_send_nack();
	}
}
void bootlader_handle_getrdp_cmd(uint8_t *pBuffer)
{
	uint8_t rdp_level = 0x00;
	/* 1. Verify the checksum */
	printmsg("BL_DEBUG_MSG_EN: bootlader_handle_getrdp_cmd \r\n");
	/* total length of the command packet */
	uint32_t command_packet_len = pBuffer[0]+1;
	/* extract the CRC32 sent by the host */
	uint32_t host_crc = *((uint32_t *)(pBuffer + command_packet_len - 4));
	if(!bootloader_verify_crc(&pBuffer[0],command_packet_len - 4,host_crc))
	{
	printmsg("BL_DEBUG_MSG_EN: Checksum success!! \r\n");	
		/* Checksum is correct*/
		bootloader_send_ack(pBuffer[0],1);
	  rdp_level = get_flash_rdp_level();
	  printmsg("BL_DEBUG_MSG_EN: RDP Level %d %#x \r\n", rdp_level, rdp_level);			
		bootloader_uart_write_data((uint8_t*)&rdp_level,1);
	}
	else
	{
		printmsg("BL_DEBUG_MSG_EN: Checksum fails !!\r\n");	
		/*Cheksum is wrong send nack*/
		bootloader_send_nack();
	}
}
void bootlader_handle_go_cmd(uint8_t *pBuffer)
{
	uint32_t go_address = 0;
	uint8_t addr_valid = ADDR_VALID;
	uint8_t addr_invalid = ADDR_INVALID;
	
	/* 1. Verify the checksum */
	printmsg("BL_DEBUG_MSG_EN: bootlader_handle_go_cmd \r\n");
	/* total length of the command packet */
	uint32_t command_packet_len = pBuffer[0]+1;
	/* extract the CRC32 sent by the host */
	uint32_t host_crc = *((uint32_t *)(pBuffer + command_packet_len - 4));
	
	if(!bootloader_verify_crc(&pBuffer[0],command_packet_len - 4,host_crc))
	{
	  printmsg("BL_DEBUG_MSG_EN: Checksum success!! \r\n");	
		/* Checksum is correct*/
		bootloader_send_ack(pBuffer[0],1);
		/* Extract the gor address */
		go_address = *((uint32_t*)&pBuffer[2]);
	  printmsg("BL_DEBUG_MSG_EN: Go Address %#x \r\n", go_address);	

		if(verify_address(go_address) == ADDR_VALID)		
		{
			/* Tell Host that address is ok */
			bootloader_uart_write_data(&addr_valid,1);
			
			go_address +=1; /* make T bit = 1 */
			void(*lets_jump)(void) = (void*)go_address;
			printmsg("BL_DEBUG_MSG_EN: Jumping to go address!! \r\n");
			
			lets_jump();
		}
		else
		{
			printmsg("BL_DEBUG_MSG_EN: Go Address invalid \r\n");
			/* Tell Host that address is not OK */
			bootloader_uart_write_data(&addr_invalid,1);
		}	
	}
	else
	{
		printmsg("BL_DEBUG_MSG_EN: Checksum fails !!\r\n");	
		/*Cheksum is wrong send nack*/
		bootloader_send_nack();
	}
}
void bootlader_handle_flash_erase_cmd(uint8_t *pBuffer)
{
	
	uint8_t erase_stat = 0x00;
	
/* 1. Verify the checksum */
	printmsg("BL_DEBUG_MSG_EN: bootlader_handle_flash_erase_cmd \r\n");
	/* total length of the command packet */
	uint32_t command_packet_len = pBuffer[0]+1;
	/* extract the CRC32 sent by the host */
	uint32_t host_crc = *((uint32_t *)(pBuffer + command_packet_len - 4));
	if(!bootloader_verify_crc(&pBuffer[0],command_packet_len - 4,host_crc))
	{
	  printmsg("BL_DEBUG_MSG_EN: Checksum success!! \r\n");	
		/* Checksum is correct*/
		bootloader_send_ack(pBuffer[0],1);
	  printmsg("BL_DEBUG_MSG_EN: Init sector - %d , no of sector - %d \r\n", pBuffer[2], pBuffer[3]);		
		
	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,GPIO_PIN_SET);
		erase_stat = execute_flash_erase(pBuffer[2], pBuffer[3]);
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,GPIO_PIN_RESET);
		
	  printmsg("BL_DEBUG_MSG_EN: flash erase status - %#x \r\n", erase_stat);			
		bootloader_uart_write_data(&erase_stat,1);
	}
	else
	{
		printmsg("BL_DEBUG_MSG_EN: Checksum fails !!\r\n");	
		/*Cheksum is wrong send nack*/
		bootloader_send_nack();
	}
}
void bootlader_handle_mem_write_cmd(uint8_t *pBuffer)
{
	uint8_t write_status = 0x00;
	uint8_t payload_len = pBuffer[6];
	uint32_t mem_address =*((uint32_t*)(&pBuffer[2]));
	
	
  /* 1. Verify the checksum */
	printmsg("BL_DEBUG_MSG_EN: bootlader_handle_mem_write_cmd \r\n");
	/* total length of the command packet */
	uint32_t command_packet_len = pBuffer[0]+1;
	/* extract the CRC32 sent by the host */
	uint32_t host_crc = *((uint32_t *)(pBuffer + command_packet_len - 4));
	if(!bootloader_verify_crc(&pBuffer[0],command_packet_len - 4,host_crc))
	{
	  printmsg("BL_DEBUG_MSG_EN: Checksum success!! \r\n");	
		/* Checksum is correct*/
		bootloader_send_ack(pBuffer[0],1);
	  printmsg("BL_DEBUG_MSG_EN: mem write address - %#x \r\n", mem_address);	

		if(verify_address(mem_address) == ADDR_VALID)	
		{			
		printmsg("BL_DEBUG_MSG_EN: valid mem write address \r\n");	
			
	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,GPIO_PIN_SET);
			/* execute mem write */
		write_status = execute_mem_write(&pBuffer[7], mem_address, payload_len);
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,GPIO_PIN_RESET);
		
	  printmsg("BL_DEBUG_MSG_EN: flash write status - %#x \r\n", write_status);			
		}
		else
		{
			printmsg("BL_DEBUG_MSG_EN: invalid mem write address \r\n");	
			write_status = ADDR_INVALID;
		}
				bootloader_uart_write_data(&write_status,1);
	}
	else
	{
		printmsg("BL_DEBUG_MSG_EN: Checksum fails !!\r\n");	
		/*Cheksum is wrong send nack*/
		bootloader_send_nack();
	}
}
void bootlader_handle_endis_rw_protect_cmd(uint8_t *pBuffer)
{

	uint8_t status = 0x00;
	
	/* 1. Verify the checksum */
	printmsg("BL_DEBUG_MSG_EN: bootlader_handle_endis_rw_protect_cmd \r\n");
	/* total length of the command packet */
	uint32_t command_packet_len = pBuffer[0]+1;
	/* extract the CRC32 sent by the host */
	uint32_t host_crc = *((uint32_t *)(pBuffer + command_packet_len - 4));
	if(!bootloader_verify_crc(&pBuffer[0],command_packet_len - 4,host_crc))
	{
	  printmsg("BL_DEBUG_MSG_EN: Checksum success!! \r\n");	
		/* Checksum is correct*/
		bootloader_send_ack(pBuffer[0],1);
		status = configure_flash_sector_rw_protection(pBuffer[2], pBuffer[3], 0);
	  printmsg("BL_DEBUG_MSG_EN: flash erase status - %#x \r\n", status);		
		bootloader_uart_write_data(&status,1);		
	}
	else
	{
		printmsg("BL_DEBUG_MSG_EN: Checksum fails !!\r\n");	
		/*Cheksum is wrong send nack*/
		bootloader_send_nack();
	}
}
void bootlader_handle_mem_read_cmd(uint8_t *pBuffer)
{}
void bootlader_handle_read_sector_status_cmd(uint8_t *pBuffer)
{}
void bootlader_handle_read_otp_cmd(uint8_t *pBuffer)
{}
void bootlader_handle_dis_rw_protect_cmd(uint8_t *pBuffer)
{
uint8_t status = 0x00;
	
	/* 1. Verify the checksum */
	printmsg("BL_DEBUG_MSG_EN: bootlader_handle_dis_rw_protect_cmd \r\n");
	/* total length of the command packet */
	uint32_t command_packet_len = pBuffer[0]+1;
	/* extract the CRC32 sent by the host */
	uint32_t host_crc = *((uint32_t *)(pBuffer + command_packet_len - 4));
	if(!bootloader_verify_crc(&pBuffer[0],command_packet_len - 4,host_crc))
	{
	  printmsg("BL_DEBUG_MSG_EN: Checksum success!! \r\n");	
		/* Checksum is correct*/
		bootloader_send_ack(pBuffer[0],1);
		status = configure_flash_sector_rw_protection(0, 0, 1);
	  printmsg("BL_DEBUG_MSG_EN: flash erase status - %#x \r\n", status);		
		bootloader_uart_write_data(&status,1);		
	}
	else
	{
		printmsg("BL_DEBUG_MSG_EN: Checksum fails !!\r\n");	
		/*Cheksum is wrong send nack*/
		bootloader_send_nack();
	}
}
	
/* This function sends ACK if CRC matches along with "len to follow" */
void bootloader_send_ack(uint8_t command_code, uint8_t follow_len)
{
	uint8_t ack_buf[2];
	ack_buf[0] = BL_ACK;
	ack_buf[1] = follow_len;
	HAL_UART_Transmit(C_UART, ack_buf, 2, HAL_MAX_DELAY);
}
	
void bootloader_send_nack(void)
{
	uint8_t nack = BL_NACK;
	HAL_UART_Transmit(C_UART, &nack, 1, HAL_MAX_DELAY);
}

/* This verifies the CRC of the given buffer in pData*/
uint8_t bootloader_verify_crc(uint8_t *pData, uint32_t len, uint32_t crc_host)
{
	uint32_t uwCRCValue = 0xff;
	
	for(uint32_t i=0; i<len; i++)
	{
		uint32_t i_data = pData[i];
		uwCRCValue = HAL_CRC_Accumulate(&hcrc, &i_data, 1);
	}
	
	if(uwCRCValue == crc_host)
	{
		return VERIFY_CRC_SUCCESS;
	}
	return VERIFY_CRC_FAIL;
}

/* Returns the macro value for Bootloader version */
uint8_t get_bootloader_ver(void)
{
	return (uint8_t)BL_VERSION;
}

/* Function that writes data in to C_UART */
void bootloader_uart_write_data(uint8_t *pBuffer, uint32_t len)
{
	/* Can be replace the below ST USART driver API calledwith another
	 * MCU driver */
	HAL_UART_Transmit(C_UART, pBuffer, len, HAL_MAX_DELAY);
}

/* Functions to get the chip identifier or device identifier */
uint16_t get_mcu_chip_id(void)
{
	/* The STM32F446xx MCUs integrate an MCU ID code. This ID identifies the ST MCU partnumber
  * and the die revision. It is part of the DBG_MCU component and is mapped on the
  * external PPB bus */
	
	uint16_t cid;
	cid = (uint16_t)(DBGMCU->IDCODE) & 0x0FFF;
	return cid;
}

/* This functions reads the RDP (Read Protection Option byte) value*/
uint8_t get_flash_rdp_level (void)
{
	uint8_t rdp_status;
	
#if 0
	FLASH_OBProgramInitTypeDef ob_handle;
	HAL_FLASHEx_OBGetConfig(&ob_handle);
	rdp_status = (uint8_t)ob_handle.RDPLevel;
#else
	volatile uint32_t *pOB_addr = (uint32_t*) 0x1FFFC000;
	rdp_status = (uint8_t)(*pOB_addr >>8);
#endif
	
	return rdp_status;
}

/*Function That verifies the address sent by Host */
uint8_t verify_address(uint32_t go_address)
{
	/* what are the valida address to which we can jum?
	 * system memory? yes
	 * sram1 memory? yes
	 * sram2 memory? yes
	 * backup sram memory? yes
	 * peripheral memory? possible, but not allowed, so NO
	 * external memory? yes */
	if (go_address >= SRAM1_BASE && go_address <= SRAM1_END)
	{
		return ADDR_VALID;
	}
	else if(go_address >= SRAM2_BASE && go_address <= SRAM2_END)
	{
		return ADDR_VALID;
	}
	else if(go_address >= FLASH_BASE && go_address <= FLASH_END)
	{
		return ADDR_VALID;
	}
	else if(go_address >= BKPSRAM_BASE && go_address <= BKPSRAM_END)
	{
		return ADDR_VALID;
	}
	else
		{
			return ADDR_INVALID;
		}
}

/*Function that erase flash memory sectors */
uint8_t execute_flash_erase(uint8_t sector_number, uint8_t number_of_sectors)
{
	/* We have 8 sectors in STM32F446RE mcu... sector[0 to 7]
	 * number_of_sectors must be in the range of 0 to 7
	 * if sector_number = 0xff, that means mass erase
	 * Code needs to be modified if the MCU supports more flash sectors */
	FLASH_EraseInitTypeDef flashErase_Handle;
	uint32_t sectorError;
	HAL_StatusTypeDef status;
	
	if(number_of_sectors > 8)
		return INVALID_SECTOR;
	
	if((sector_number == 0xff) || (sector_number <= 7))
	{
		if(sector_number == (uint8_t)0xff)
		{
			flashErase_Handle.TypeErase = FLASH_TYPEERASE_MASSERASE;
		}
		else
			{
				/* Here is calculated how many sectors needs to be erased */
				uint8_t remaining_sector = 8 - sector_number;
				if(number_of_sectors > remaining_sector)
				{
					number_of_sectors = remaining_sector;
				}
				flashErase_Handle.TypeErase = FLASH_TYPEERASE_MASSERASE;
				flashErase_Handle.Sector = sector_number; /* this is the initial sector */
				flashErase_Handle.NbSectors = number_of_sectors;
			}
			flashErase_Handle.Banks = FLASH_BANK_1;
			
			/* Get access to touch the file regiters */
			HAL_FLASH_Unlock();
			flashErase_Handle.VoltageRange = FLASH_VOLTAGE_RANGE_3; /* for STM32F466RE MCU*/
			status =  HAL_FLASHEx_Erase(&flashErase_Handle, &sectorError);
			HAL_FLASH_Lock();
			
			return status;
	}
	return (HAL_StatusTypeDef)INVALID_SECTOR;
}

/* Note 1: This functions supports only writing to flash memory
 * Note 2: This function does not check if mem_address is valid for MCU */
uint8_t execute_mem_write(uint8_t * pBuffer, uint32_t mem_address, uint32_t len)
{
	uint8_t status = HAL_OK;
	
	HAL_FLASH_Unlock();
	
	for(uint32_t i = 0; i<len ; i++)
	{
		/* Here is programmed the flash byte by byte*/
		status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, mem_address + i, pBuffer[i]);
	}
	HAL_FLASH_Lock();
	
	return status;
}

/* Modifying user option bytes
 *To modify the user option value, follow the sequence below:
 *1. Check that no Flash memory operation is ongoing by checking the BSY bit in the
 *   FLASH_SR register
 *2. Write the desired option value in the FLASH_OPTCR register.
 *3. Set the option start bit (OPTSTRT) in the FLASH_OPTCR register
 *4. Wait for the BSY bit to be cleared. */
uint8_t configure_flash_sector_rw_protection(uint8_t sector_detail, uint8_t protection_mode, uint8_t disable)
{
	/* First configure the protection mode
	 * protection_mode = 1, means write protect of the user flash sector
	 * protection_mode = 2, means read/write protect of the user flash sectors
	 * According to Ref Manual of STM32F446xx table 9, we have to modify the address  0x1FFF C008 bit 15 (SPRMOD)*/
	
	/* Flash option control register (OPTCR) */
	volatile uint32_t *pOPTCR = (uint32_t *) 0x40023C14;
	
	if(disable)
	{
		/* Option byte config unlock */
		HAL_FLASH_OB_Unlock();
		/* Wait till no active operation on flash */
		while( __HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
		/* Clear the 31st bit (default state)*/
		*pOPTCR &= ~(0x01 <<31);
		/* Clear the protectio: make all bits belonging to sectors as 1*/
		*pOPTCR |= (0xFF <<16);
		/* Set the option start bit (OPTSTRT) in the FLASH_OPTCR register*/
		*pOPTCR |= (0x01 <<1);
		/* Wait till no active operation in flash */
		while( __HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
		HAL_FLASH_OB_Lock();
		
		return 0;
	}
	
 if(protection_mode == (uint8_t) 1)
 {
	 	/* Option byte config unlock */
		HAL_FLASH_OB_Unlock();
	 	/* Wait till no active operation on flash */
		while( __HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
	 /* Setting write protection for the sectors, clear 31st bit */
	 *pOPTCR &= ~(1 <<31);
	 /* Put write protection on sectors*/
	 *pOPTCR &= ~(sector_detail <<16);
	 		/* Set the option start bit (OPTSTRT) in the FLASH_OPTCR register*/
		*pOPTCR |= (1 <<1);
		/* Wait till no active operation in flash */
		while( __HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
		HAL_FLASH_OB_Lock();
 }
 else if(protection_mode == (uint8_t) 2)
 {
	 	/* Option byte config unlock */
		HAL_FLASH_OB_Unlock();
	 	/* Wait till no active operation on flash */
		while( __HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
	 /* Setting write protection for the sectors, clear 31st bit */
	 *pOPTCR |= (1 <<31);
	 /* Put read and write protection on sectors*/
	 *pOPTCR &= ~(0xFF <<16);
	 *pOPTCR |= (sector_detail <<16);
	 		/* Set the option start bit (OPTSTRT) in the FLASH_OPTCR register*/
		*pOPTCR |= (1 <<1);
		/* Wait till no active operation in flash */
		while( __HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
		HAL_FLASH_OB_Lock();
 }
 
 return 0;
	
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
