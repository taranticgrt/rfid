

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "stm32l4xx_hal.h"


#include "hc_rfid.h"
#include "RFID_read_script.h"

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

extern SPI_HandleTypeDef hspi1;


#define PIN_CAN_LEDG GPIO_PIN_3
#define PORT_CAN_LEDG GPIOB
#define PIN_CAN_LEDR GPIO_PIN_15
#define PORT_CAN_LEDR GPIOA

#define RFID_IRQ_Pin GPIO_PIN_3
#define RFID_IRQ_GPIO_Port GPIOA
#define RFID_RST_Pin GPIO_PIN_4
#define RFID_RST_GPIO_Port GPIOA

void Error_Handler(void);


//-----------------------------------------------

SPI_HandleTypeDef hspi1;

void LEDR()
{
  HAL_GPIO_WritePin(PORT_CAN_LEDR, PIN_CAN_LEDR, GPIO_PIN_SET);
  HAL_GPIO_WritePin(PORT_CAN_LEDG, PIN_CAN_LEDG, GPIO_PIN_RESET);
}

void LEDG()
{
  HAL_GPIO_WritePin(PORT_CAN_LEDR, PIN_CAN_LEDR, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PORT_CAN_LEDG, PIN_CAN_LEDG, GPIO_PIN_SET);
}

void LEDO()
{
  HAL_GPIO_WritePin(PORT_CAN_LEDR, PIN_CAN_LEDR, GPIO_PIN_SET);
  HAL_GPIO_WritePin(PORT_CAN_LEDG, PIN_CAN_LEDG, GPIO_PIN_SET);
}

void LEDOff()
{
  HAL_GPIO_WritePin(PORT_CAN_LEDR, PIN_CAN_LEDR, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PORT_CAN_LEDG, PIN_CAN_LEDG, GPIO_PIN_RESET);
}






int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_SPI1_Init();

	MFRC522_hard_reset();
  while (1)
  {
		uint8_t registers[NUM_REGS];
		HC_RFID_info tag_data;
		MFRC522_read_tag_data(registers,HC_READ_TAG_SCRIPT,&tag_data);
		if(tag_data.uid_valid){
			LEDG();
		}
		else{
			LEDR();
		}
  }
}

////////////////////////////////////////////

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32; // SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PIN_CAN_LEDR*/
  GPIO_InitStruct.Pin = PIN_CAN_LEDR;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PORT_CAN_LEDR, &GPIO_InitStruct);
  HAL_GPIO_WritePin(PORT_CAN_LEDR, PIN_CAN_LEDR, GPIO_PIN_RESET);

  /*Configure GPIO pin : PIN_CAN_LEDG*/
  GPIO_InitStruct.Pin = PIN_CAN_LEDG;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PORT_CAN_LEDG, &GPIO_InitStruct);
  HAL_GPIO_WritePin(PORT_CAN_LEDG, PIN_CAN_LEDG, GPIO_PIN_RESET);



  /*Configure GPIO pin : PIN_RFID_IRQ */
  GPIO_InitStruct.Pin = PIN_RFID_IRQ;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PORT_RFID_IRQ, &GPIO_InitStruct);

  /*Configure GPIO pin : PIN_RFID_RST */
  GPIO_InitStruct.Pin = PIN_RFID_RST;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PORT_RFID_RST, &GPIO_InitStruct);
  HAL_GPIO_WritePin(PORT_RFID_RST, PIN_RFID_RST, GPIO_PIN_RESET);

  /*Configure GPIO pin : PIN_RFID_NSS */
  GPIO_InitStruct.Pin = PIN_RFID_NSS;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PORT_RFID_NSS, &GPIO_InitStruct);
  HAL_GPIO_WritePin(PORT_RFID_NSS, PIN_RFID_NSS, GPIO_PIN_RESET);
}

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

