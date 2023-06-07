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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define spiSize  18  /* Number of bytes to be transmitted */
#define spiWords  9  /* Number of words (byte pairs) to be transmitted */
#define uartChars 6  /* Number of characters needed to represent 1 UART data element */
#define adcWords  6  /* Number of ADC words to be received */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* SPI TRANSMIT & RECEIVE BUFFERS*/
uint8_t txData[spiSize]; /* Transmit data buffer */
uint8_t rxData[spiSize]; /* Receive data buffer */

/* MOSI VARIABLES CONFIGURATION */
bool testMode = 1; /* testMode = 1 --> test mode, 0 --> auto mode */
bool out0     = 1; /* GPO_0 */
bool out1     = 0; /* GPO_1 */
bool out2     = 1; /* GPO_2 */

/* MISO VARIABLES */
uint8_t misoData[spiSize];   /* Buffer for obtaining the status word */
uint16_t adcData[adcWords];  /* ADC data buffer */
uint16_t adcDataCpy[adcWords] = {2180, 3125, 2180, 3125, 2180, 3125}; /* Dummy buffer for testing */

/* DMA and TIM1 interrupt management */
uint8_t dmaTim = 0;
uint8_t spiTxCpltFlag = 0;

/* For printing results to UART */
char uartBuf[spiWords][6];
char statusWord[16];

/* Benchmarking */
float elapsedTime = 0.0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

uint8_t* setTxData(uint8_t* txData, bool* testMode, bool* out0, bool* out1, bool* out2);
uint16_t* readADC(uint16_t* adcData);
void handleReceivedData (uint8_t* rxData, uint16_t* adcData);

/* Printing ADC values to the terminal via UART */
void uartPrint(uint16_t* adcData, char(*uartBuf)[uartChars]);

/* Converting numerical ADC values to strings */
char* uint16_to_char_array(uint16_t adcData, char* uartBuf, size_t bufSize);

/* Configuring the DMA SPI peripheral */
void SPI_DMA_TxRx_CONFIG(SPI_HandleTypeDef *hspi ,DMA_HandleTypeDef *hdma_spi_tx, DMA_HandleTypeDef *hdma_spi_rx, uint8_t* txData, uint8_t* rxData);

/* Full duplex SPI transaction via DMA channels 2(RX) and 3(TX) */
void SPI_DMA_TxRx(SPI_HandleTypeDef *hspi ,DMA_HandleTypeDef *hdma_spi_tx, DMA_HandleTypeDef *hdma_spi_rx, uint8_t* txData, uint8_t* rxData);

/* SPI transmit-receive complete callback */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);

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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  /* CONFIIGURE THE CONTROL DATA BUFFER */

  /* Initialize the entire TX data buffer to 0. */
  for(int i = 0; i < spiSize; i++)
      txData[i] = 0;

  /* Configure data to send via SPI*/
  setTxData(txData, &testMode, &out0, &out1, &out2);

  /* Initialize the UART data buffer to all blank characters */
  for(int i = 0; i < spiWords; i++)
  	for(int j = 0; j < uartChars; j++)
  	{
  	   uartBuf[i][j] = ' ';
  	}

  /*SPI DMA config, 1st call test*/
  SPI_DMA_TxRx_CONFIG(&hspi1, &hdma_spi1_tx, &hdma_spi1_rx, txData, rxData);

  /* Enable TIM1 interrupt */
  HAL_TIM_Base_Start_IT(&htim1);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL10;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  __HAL_SPI_ENABLE(&hspi1);
  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1250-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  __HAL_RCC_TIM1_CLK_ENABLE();
  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */
  __HAL_RCC_USART1_CLK_ENABLE();
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 111731;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_8;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : SPI1_SS_Pin */
  GPIO_InitStruct.Pin = SPI1_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI1_SS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* Configuring the MOSI line bits */
uint8_t* setTxData(uint8_t* txData, bool* testMode, bool* out0, bool* out1, bool* out2)
  {

    /* Test mode is the MSbit of the CW (second byte, last bit). If it is 0, mode is automatic. If 1, mode is manual. */
    if(*testMode)
      txData[0] |= (1u << 7);
    else
      txData[0] &= ~(1u << 7);

    /* Configuring commands for GPO pins */
    if(*out0)
      txData[1] |= (1u << 0);
    else
      txData[1] &= ~(1u << 0);

    if(*out1)
      txData[1] |= (1u << 1);
    else
      txData[1] &= ~(1u << 1);

    if(*out2)
      txData[1] |= (1u << 2);
    else
      txData[1] &= ~(1u << 2);

    return txData;
   }


/* Reading the ADC data from the MISO line, by byte pairs */
inline uint16_t* readADC(uint16_t* adcData)
  {
     for(uint8_t i = 1; i <= adcWords; i++)
         adcData[i-1] = (uint16_t)((rxData[2*i] & 0x0F) << 8 | rxData[2*i+1] );

     return adcData;
  }

/* Handling received data */
void handleReceivedData (uint8_t* rxData, uint16_t* adcData)
  {
      /* OBTAINING THE ENTIRE STATUS WORD */
      //memcpy(misoData, rxData, spiSize);

      /* Obtaining the latest ADC data */
      readADC(adcData);
  }

/* Converting data to char arrays, to print via UART */
char* uint16_to_char_array(uint16_t adcData, char* uartBuf, size_t bufSize)
  {
     int i = bufSize - 1;
	 uartBuf[i] = '\0';  /* Null-terminate the buffer */

	 if (adcData == 0)
		 uartBuf[--i] = '0';
	 else
		 while (adcData != 0 && i >= 0)
		     {
			    uartBuf[--i] = '0' + (adcData % 10);
				adcData /= 10;
		      }

	 return uartBuf;
   }

/* Printing data to UART */
void uartPrint(uint16_t* adcData, char(*uartBuf)[uartChars])
  {
	    /* Convert the status word (1st 2 bytes of MISO data to characters, '1' or '0') */
	    int k = 0;
	    for(int i = 0; i < 2; i++)
	    for(int j = 7; j >= 0; j--)
	       statusWord[k++] = (misoData[i] & (1u << j))?'1':'0';

        /* Print the status word */
	    HAL_UART_Transmit(&huart1, (uint8_t *)"Status word: \r\n", 15, 50);
	    HAL_UART_Transmit(&huart1, (uint8_t *)statusWord, 16, 50);
	    HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, 50);

		/* Print ADC data: */
		for(int j = 0; j < adcWords; j++)
		   {
			  if(j == 0)
			    HAL_UART_Transmit(&huart1, (uint8_t *)"ADC data: \r\n", 12, 50);

			  uint16_to_char_array(adcDataCpy[j], uartBuf[j], sizeof(uartBuf[j])); /* Currently the dummy buffer, adcDataCpy is used */
			  HAL_UART_Transmit(&huart1, (uint8_t *)uartBuf[j], sizeof(uartBuf[j]), 50);
			  HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, 50);
			}

		/* New blank line */
		HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, 50);
  }

/* UART transmit interrupt callback function */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
   if (huart == &huart1)
    {
	  dmaTim = 0;
	  HAL_UART_DMAStop(&huart1); /* Circular DMA buffer, has to be stopped and explicitly modified */
      uartPrint(adcDataCpy, uartBuf);
    }
}


void SPI_DMA_TxRx_CONFIG(SPI_HandleTypeDef *hspi ,DMA_HandleTypeDef *hdma_spi_tx, DMA_HandleTypeDef *hdma_spi_rx, uint8_t* txData, uint8_t* rxData)
{

   /*---------DISABLE DMA TX AND RX CHANNELS FOR CONFIGURATION------------------- */

   /* Disable the SPI DMA receive channel, for configuration */
   hdma_spi_rx->Instance->CCR &= ~DMA_CCR_EN;

   /* Disable the SPI DMA transmit channel, for configuration */
   hdma_spi_tx->Instance->CCR &= ~DMA_CCR_EN;

   /*---------CONFIGURE SPI DATA SIZE FOR TX AND RX------------------------------ */
   /* Configure the SPI send and receive buffer pointers and data length  */
   hspi->pTxBuffPtr = txData;
   hspi->TxXferSize = spiSize;
   hspi->TxXferCount = spiSize;
   hspi->pRxBuffPtr = rxData;
   hspi->RxXferSize = spiSize;
   hspi->RxXferCount = spiSize;

   /*---------CONFIGURE DMA CHANNEL FOR SPI DATA LENGTH, SRC AND DST ADDRESSES--- */

   /* Configure DMA SPI RX Channel data length */
   hdma_spi_rx->Instance->CNDTR = spiSize;

   /* Configure DMA SPI RX Channel source address */
   hdma_spi_rx->Instance->CPAR = (uint32_t)(&(hspi->Instance->DR));

   /* Configure DMA SPI RX Channel destination address */
   hdma_spi_rx->Instance->CMAR = (uint32_t)rxData;

   /* Configure DMA SPI TX Channel data length */
   hdma_spi_tx->Instance->CNDTR = spiSize;

   /* Configure DMA SPI TX Channel source address */
   hdma_spi_tx->Instance->CMAR = (uint32_t)txData;

   /* Configure DMA SPI TX Channel destination address */
   hdma_spi_tx->Instance->CPAR = (uint32_t)(&(hspi->Instance->DR));

   /*---------ENABLE THE DMA CHANNELS FOR SPI------------------------------------ */

   /* Enable the SPI DMA receive channel */
   hdma_spi_rx->Instance->CCR |= DMA_CCR_EN;

   /* Enable the SPI DMA transmit channel */
   hdma_spi_tx->Instance->CCR |= DMA_CCR_EN;

   /* Enable the SPI transmit via DMA complete interrupt  */
   hdma_spi_tx->Instance->CCR |= DMA_IT_TC;

   /* Set the SPI TxRx busy flag */
   hspi->State = HAL_SPI_STATE_BUSY_TX_RX;

}


void SPI_DMA_TxRx(SPI_HandleTypeDef *hspi ,DMA_HandleTypeDef *hdma_spi_tx, DMA_HandleTypeDef *hdma_spi_rx, uint8_t* txData, uint8_t* rxData)
{

/*---------------------------Config-Modified--------------------------------------*/

	   /* Disable requests for SPI transmit and receive via DMA */
	   SPI1->CR2 &= ~SPI_CR2_RXDMAEN;
	   SPI1->CR2 &= ~SPI_CR2_TXDMAEN;

	   /* Disable the SPI DMA receive channel, for configuration */
	   //hdma_spi_rx->Instance->CCR &= ~DMA_CCR_EN;
	   DMA1_Channel2->CCR &= ~DMA_CCR_EN;

	   /* Disable the SPI DMA transmit channel, for configuration */
	   //hdma_spi_tx->Instance->CCR &= ~DMA_CCR_EN;
	   DMA1_Channel3->CCR &= ~DMA_CCR_EN;

		/* Data update TEST */
	   for(int k = 0; k < spiSize; k++)
		txData[k]++;

	/*---------CONFIGURE DMA CHANNEL FOR SPI DATA LENGTH, SRC AND DST ADDRESSES--- */

	   /* Configure DMA SPI RX Channel data length */
	   //hdma_spi_rx->Instance->CNDTR = spiSize;
	   DMA1_Channel2->CNDTR = spiSize;

	   /* Configure DMA SPI TX Channel data length */
	   //hdma_spi_tx->Instance->CNDTR = spiSize;
	   DMA1_Channel3->CNDTR = spiSize;

	/*---------ENABLE THE DMA CHANNELS FOR SPI------------------------------------ */

	   /* Enable the SPI DMA receive channel */
	   //hdma_spi_rx->Instance->CCR |= DMA_CCR_EN;
	   DMA1_Channel2->CCR |= DMA_CCR_EN;

	   /* Enable the SPI DMA transmit channel */
	   //hdma_spi_tx->Instance->CCR |= DMA_CCR_EN;
	   DMA1_Channel3->CCR |= DMA_CCR_EN;

	   /* Set the SPI TxRx busy flag */
	   //hspi->State = HAL_SPI_STATE_BUSY_TX_RX;

/*------------------------------------------------------------------------------*/


	   /* Benchmarking */
	   /* Reset TIM6 counter */
	   TIM6->CNT = 0;

	   /* Start TIM6 counter */
	   TIM6->CR1 |= TIM_CR1_CEN;


	/* Pull NSS pin LOW */
	SPI1_SS_GPIO_Port->BRR = SPI1_SS_Pin;

    /* Set the SPI TxRx busy flag */
    //hspi->State = HAL_SPI_STATE_BUSY_TX_RX;

	/* Enable requests for SPI transmit and receive via DMA, this actually transfers the data */
	SPI1->CR2 |= SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;  /* Transmit and receive (FULL-DUPLEX) */

	/* Wait until DMA transfers are complete */
	while (!(DMA1->ISR & DMA_ISR_TCIF3) || !(DMA1->ISR & DMA_ISR_TCIF2)){}

	/* Wait for SPI to be not busy */
	while (SPI1->SR & SPI_SR_BSY);

	/* Call the SPI TxRx transfer complete callback function*/
	//HAL_SPI_TxRxCpltCallback(hspi);

	/* Pull NSS pin HIGH */
	SPI1_SS_GPIO_Port->BSRR = SPI1_SS_Pin;


	/* Benchmarking */
	/* Stop TIM6 counter */
	TIM6->CR1 &= ~TIM_CR1_CEN;

	/*Obtain the elapsed time*/
	elapsedTime = TIM6->CNT*0.025;

	/* Clear the SPI TxRx busy flag */
    //hspi->State = HAL_SPI_STATE_READY;
}

/* 31.25us timer callback function, for SPI communication */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* Timer 1 elapsed */
    if (htim->Instance == TIM1)
        {
    	     /* Benchmarking */
    	     /* Reset TIM6 counter */
    	     //TIM6->CNT = 0;

    	     /* Start TIM6 counter */
    	    // TIM6->CR1 |= TIM_CR1_CEN;

    	     /* SPI data transaction via DMA */
    	     SPI_DMA_TxRx(&hspi1 ,&hdma_spi1_tx, &hdma_spi1_rx, txData, rxData);

			 /* Handle received data */
			 readADC(adcData);

			 /* Initialize new UART transmission via DMA interrupt */
			 /*if(!dmaTim)
			   {
			      HAL_UART_Transmit_DMA(&huart1, (uint8_t *)"/", 1);
			      dmaTim = 1;
			   }*/

			 /* Benchmarking */
		     /* Stop TIM6 counter */
		    // TIM6->CR1 &= ~TIM_CR1_CEN;

		     /*Obtain the elapsed time*/
		     //elapsedTime = TIM6->CNT*0.025; /* Elapsed time for TIM6 in microseconds, DMA or SPI clock? */
         }
}

/* SPI transmit complete callback */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	/* Callback test */
	spiTxCpltFlag = !spiTxCpltFlag;
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
