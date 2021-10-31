/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
volatile uint8_t k_request = 0;
volatile uint8_t ReceiveFlag;
volatile uint8_t channel = 1;
volatile uint8_t LEDmsecCntr = 0;
volatile uint8_t LEDmsecFlag = 0;
volatile uint8_t FLOWmsecCntr = 0;
volatile uint32_t FLOW = 0;
volatile uint8_t ADCmsecCntr = 0;
volatile uint8_t ADCmsecFlag = 0;
volatile uint8_t cplt = 1;
volatile uint8_t noflow = 1;
volatile uint8_t state = 0;
volatile uint8_t prev = 0;
float Vdd = 3.3;
uint32_t V_RMS;
uint16_t I_RMS;
uint16_t TEMP1 = 0;
uint16_t TEMP2 = 0;
uint8_t num;
int Element = 0;
int ValveOpen = 0;
int kprocess = 1;

int16_t bits = 4095;
uint8_t TransmitBufferA[] = "$A,18301088\r\n";
uint8_t TransmitBufferF[] = "$F\r\n";
uint8_t TransmitBufferG[] = "$G,0\r\n";
uint8_t TransmitBufferGT[] = "$G,00\r\n";
uint8_t TransmitBufferGH[] = "$G,000\r\n";
uint8_t TransmitBufferK[35] = "$K,\r\n";

uint8_t ReceiveBuffer[1];
uint8_t count = 0;
uint8_t factorone = 0;
uint8_t factorten = 0;
uint8_t factorhundred = 0;
char HoldBuffer[10];
uint8_t TempBuffer[10];
uint8_t TempBufferD[10];
uint8_t SizeG = 10;
int16_t SignedInteger;
int16_t TempInteger = 0;
int16_t CurrentSignal;
int16_t VoltageSignal;
int16_t ADC_CurrentArray[71];
int16_t ADC_VoltageArray[71];
int16_t ADC_Temp1Array[71];
int16_t ADC_Temp2Array[71];
float VALUE_CurrentArray[71];
float VALUE_VoltageArray[71];
float VALUE_Temp1Array[71];
float VALUE_Temp2Array[71];
volatile uint8_t  ADCReceive = 1;
int c = 0;
int v = 0;
int t1 = 0;
int t2 = 0;
int bounce = 0;
float vnew[100];
float inew[100];
int inumvalue = 0;
int vnumvalue = 0;
int position = 0;

char ibuff[30];
char vbuff[30];
char t1buff[30];
char t2buff[30];
char flowbuff[30];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_SYSTICK_Callback(void);
void Number_Display(int8_t d1,int8_t d2,int8_t d3,int8_t d4);
void Disable_Digits(void);
uint8_t Number_Maker(int16_t digit);
uint8_t Digit_Maker(int8_t digit);
void set_all(void);
void reset_all(void);
void ADCprocess(void);
uint8_t convertfromstring (char* string, int16_t* value);
uint8_t converttostring (int16_t* value, char* string, int16_t length);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
float vconvert(int16_t* v_convarray,float*v_address);
float iconvert(int16_t* i_convarray,float*i_address);
float t1convert(int16_t* t1_convarray);
float t2convert(int16_t* t2_convarray);

uint8_t Int2String(char* output_string, int32_t val, uint8_t maxlen);
//void InputK(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, ReceiveBuffer,1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  HAL_ADC_Start_IT(&hadc1);
	  Number_Maker(TempInteger);
	  if (ReceiveFlag)
	  {
	  	ReceiveFlag = 0;
	  	HAL_UART_Receive_IT(&huart1, ReceiveBuffer,1);
	  	HoldBuffer[count++]= ReceiveBuffer[0];

	  	if (ReceiveBuffer[0] == '\n')
	  	{
	  		HoldBuffer[count] = '\0';
	  		convertfromstring(HoldBuffer, &SignedInteger);

	  		if (HoldBuffer[0] == '$')
	  		{
	  			switch(HoldBuffer[1])
	  			{
	    	  	  	  	  	case 'A':
	    				    	HAL_UART_Transmit(&huart1, TransmitBufferA,sizeof(TransmitBufferA)-1,5);
	    				    	break;
	    	  	  	  	  	case 'B':
	    	  	  	  	  		ValveOpen = HoldBuffer[3];
	    	  	  	  	  		ValveOpen = ValveOpen - 48;
	    	  	  	  		    break;
	    	  	  	  		case 'D':
	    	  	  	  	  		Element = HoldBuffer[3];
	    	  	  	  	  		Element = Element - 48;
	    	  	  	  		    break;
	    				    case 'F':
	    				       	TempBuffer[0] = HoldBuffer[3];
	    				    	TempBuffer[1] = HoldBuffer[4];
	    				    	TempBuffer[2] = HoldBuffer[5];
	    				    	TempBuffer[3] = HoldBuffer[6];
	    				    	TempBuffer[4] = HoldBuffer[7];
	    				    	convertfromstring(HoldBuffer+3, &TempInteger);

	    				    	HAL_UART_Transmit(&huart1, TransmitBufferF,sizeof(TransmitBufferF)-1,5);
	    				    	break;
	    				    case 'G':
	    				    	if(TempBuffer[1] == '\r' && TempBuffer[2] == '\n')
	    				    	{
	    				    		TransmitBufferG[3] = TempBuffer[0];
	    				    		TransmitBufferG[4] = '\r';
	    				    		TransmitBufferG[5] = '\n';
	    				    		HAL_UART_Transmit(&huart1, TransmitBufferG,6,5);
	    				    	}
	    				    	if(TempBuffer[2] == '\r' && TempBuffer[3] == '\n')
	    						{
	    							TransmitBufferGT[3] = TempBuffer[0];
	    							TransmitBufferGT[4] = TempBuffer[1];
	    							TransmitBufferGT[5] = '\r';
	    							TransmitBufferGT[6] = '\n';
	    							HAL_UART_Transmit(&huart1, TransmitBufferGT,7,5);
	    						}
	    				    	if(TempBuffer[3] == '\r' && TempBuffer[4] == '\n')
	    						{
	    							TransmitBufferGH[3] = TempBuffer[0];
	    							TransmitBufferGH[4] = TempBuffer[1];
	    							TransmitBufferGH[5] = TempBuffer[2];
	    							TransmitBufferGH[6] = '\r';
	    							TransmitBufferGH[7] = '\n';
	    							HAL_UART_Transmit(&huart1, TransmitBufferGH,8,5);
	    						}
	    				    	break;
	    				    case 'K':
	    				    {
//	    				      	HAL_ADC_Stop(&hadc1);
	    				    	I_RMS = iconvert(ADC_CurrentArray,VALUE_CurrentArray);
	    				    	V_RMS = vconvert(ADC_VoltageArray,VALUE_VoltageArray);
	    				    	TEMP1 = t1convert(ADC_Temp1Array);
	    				    	TEMP2 = t2convert(ADC_Temp2Array);
	    				    	//kprocess = 1;
	    				      	num = Int2String(ibuff,I_RMS,10);
	    				      	inumvalue = num;
	    				      	position = 3+inumvalue;
	    				      	while (num >0)
	    				      	{
	    				      		TransmitBufferK[position-num] = ibuff[inumvalue-num];
									num--;
	    				      	}
	    				      	TransmitBufferK[position] = 0x2C;
	    				      	position = position + 1;
	    				      	num = Int2String(vbuff,V_RMS,10);
	    				      	vnumvalue = num;
	    				      	position = position + vnumvalue;
	    				      	while (num >0)
	    				      	{
	    				      		TransmitBufferK[position-num] = vbuff[vnumvalue-num];
									num--;
	    				      	}
	    				      	TransmitBufferK[position] = 0x2C;
	    				      	position = position + 1;
	    				      	num = Int2String(t1buff,TEMP1,10);
	    				      	vnumvalue = num;
	    				      	position = position + vnumvalue;
	    				      	while (num >0)
	    				      	{
	    				      		TransmitBufferK[position-num] = t1buff[vnumvalue-num];
									num--;
	    				      	}
	    				      	TransmitBufferK[position] = 0x2C;
	    				      	position++;
	    				      	num = Int2String(t2buff,TEMP2,10);
	    				      	vnumvalue = num;
	    				      	position = position + vnumvalue;
	    				      	while (num >0)
	    				      	{
	    				      		TransmitBufferK[position-num] = t2buff[vnumvalue-num];
									num--;
	    				      	}
	    				      	TransmitBufferK[position] = 0x2C;
	    				      	position++;
	    				      	num = Int2String(flowbuff,FLOW,10);
	    				      	vnumvalue = num;
	    				      	position = position + vnumvalue;
	    				      	while (num >0)
	    				      	{
	    				      		TransmitBufferK[position-num] = flowbuff[vnumvalue-num];
									num--;
	    				      	}
	    				      	TransmitBufferK[position] = 0x2C;
	    				      	position++;
    				      		TransmitBufferK[position] = 'O';
    				      		position++;

	    				      	if(Element==1)
	    				      	{
	    				      		TransmitBufferK[position] = 'N';
	    				      		position++;
	    				      	}
	    				      	if(Element==0)
	    				      	{
	    				      		TransmitBufferK[position] = 'F';
	    				      		position++;
	    				      		TransmitBufferK[position] = 'F';
	    				      		position++;
	    				      	}
	    				      	TransmitBufferK[position] = 0x2C;
	    				      	position++;
	    				      	if(ValveOpen==1)
	    				      	{
	    				      		TransmitBufferK[position] = 'O';
	    				      		position++;
	    				      		TransmitBufferK[position] = 'P';
	    				      		position++;
	    				      		TransmitBufferK[position] = 'E';
	    				      		position++;
    				      			TransmitBufferK[position] = 'N';
    				      			position++;
	    				      	}
    				      		if(ValveOpen==0)
	    				      	{
	    				      		TransmitBufferK[position] = 'C';
	    				      		position++;
	    				      		TransmitBufferK[position] = 'L';
	    				      		position++;
	    				      		TransmitBufferK[position] = 'O';
	    				      		position++;
    				      			TransmitBufferK[position] = 'S';
    				      			position++;
	    				      		TransmitBufferK[position] = 'E';
	    				      		position++;
	    				      		TransmitBufferK[position] = 'D';
	    				      		position++;
	    				    	}

				      		TransmitBufferK[position] = '\r';
				      		position++;
				      		TransmitBufferK[position] = '\n';
				      		HAL_UART_Transmit(&huart1, TransmitBufferK,position+1,5);
	    				    }
	    						break;

	  			}
	  		}
	  		count = 0;
	  	}

	  }


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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV128;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the ADC multi-mode 
    */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_181CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA2   ------> USART2_TX
     PA3   ------> USART2_RX
*/

static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC6 PC8 
                           PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_8 
                          |GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 PA7 PA8 
                           PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB3 PB4 PB5 
                           PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint8_t Digit_Maker(int8_t digit)
{
  // PB10 (DIGIT 1) PB4 (DIGIT 2) PB5 (DIGIT 3) PB3 (DIGIT 4)
  // PA5=(SEG1) PA6=(SEG2) PA7=(SEG3) PB6=(SEG 4) PC7=(SEG5) PA9=(SEG 6) PA8=(SEG 7)
	switch (digit)
	{
	case 0x01:
		set_all();
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET); //2
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET); //3
		  break;
	case 0x02:
		reset_all();
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET); //3
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET); //7

		break;
	case 0x03:
		reset_all();
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET); //5
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET); //7
		break;
	case 0x04:
		reset_all();
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET); //1
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET); //4
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET); //5
		break;
	case 0x05:
		reset_all();
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET); //2
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET); //5
		break;
	case 0x06:
		reset_all();
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET); //2
		break;
	case 0x07:
		set_all();
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET); //1
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET); //2
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET); //3
		break;
	case 0x08:
		reset_all();
		break;
	case 0x09:
		reset_all();
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET); //5
		break;
	case 0x00:
		reset_all();
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET); //6
		break;
	}

	return 1;
}
	
uint8_t Number_Maker(int16_t number)
{
	int8_t minus =-1;
	int8_t digit4 =0;
	int8_t digit3 =0;
	int8_t digit2 =0;
	int8_t digit1 =0;
	if (number < 0)
	{
		number = number * minus;
	}

	digit4 = number % 10;
	number = number/10;
	digit3 = number % 10;
	number = number/10;
	digit2 = number % 10;
	number = number/10;
	digit1 = number % 10;

	Number_Display(digit1, digit2, digit3, digit4);
	return 1;
}

void Number_Display(int8_t d1,int8_t d2,int8_t d3,int8_t d4)
{
	int process = 1;
	int d1cplt = 0;
	int d2cplt = 0;
	int d3cplt = 0;
	int d4cplt = 0;

	while (process)
	{

		if(d1==0)
		{
			d1cplt = 1;
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET);
			if(d2==0)
			{
				d2cplt = 1;
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
				if(d3==0)
				{
					d3cplt = 1;
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);
				}
			}
		}

	if (LEDmsecFlag==1 && d1cplt==0)
		{
		LEDmsecFlag = 0;
		Disable_Digits();
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET);
				Digit_Maker(d1);
		d1cplt=1;
		}
	if (LEDmsecFlag==1 && d1cplt==1 && d2cplt==0)
		{
		LEDmsecFlag = 0;
		Disable_Digits();
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
				Digit_Maker(d2);
		d2cplt=1;
		}
	if (LEDmsecFlag==1 && d1cplt==1 && d2cplt==1 && d3cplt==0)
		{
		LEDmsecFlag = 0;
		Disable_Digits();
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);
				Digit_Maker(d3);
		d3cplt=1;
		}
	if (LEDmsecFlag==1 && d1cplt==1 && d2cplt==1 && d3cplt==1 && d4cplt==0)
		{
		LEDmsecFlag = 0;
		Disable_Digits();
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_RESET);
		Digit_Maker(d4);
		d4cplt=1;
		}
	if (d4cplt == 1)
	{
		d1cplt = 0;
		d2cplt = 0;
		d3cplt = 0;
		d4cplt = 0;
	}
	if (ReceiveFlag == 1 || ADCmsecFlag == 1)
	{
		process = 0;
	}
	}
}

void Disable_Digits(void)
{
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET); //1
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET); //2
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET); //3
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET); //4
}

void set_all(void)
{
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET); //1
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET); //2
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET); //3
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET); //4
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET); //5
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET); //6
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET); //7
}

void reset_all(void)
{
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET); //1
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET); //2
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET); //3
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET); //4
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET); //5
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET); //6
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET); //7
}

uint8_t convertfromstring (char* string, int16_t* value)
{
	int16_t minus = 1;
	int16_t sum = 0;
	if (*string == '\0')
	{
		return 0;
	}
	if (*string == '-')
	{
		minus = -1;
		string++;
	}
	while ((*string >= '0') &&(*string <= '9'))
	{
		sum = 10 * sum;
		sum = sum + (*string - 0x30);
		string++;
	}

	*value = minus * sum;
	return 1;
}


//uint8_t converttostring (int16_t* value, char* string, int16_t length)
//{
//	int8_t minus =-1;
//	int8_t digit[10];
//	int8_t number;
//	int8_t count2;
//	int8_t num;
//
//while(count2<length)
//{
//	digit[count2] = *value % 10;
//	number = *value/10;
//	count2++;
//
//	if (number == 0)
//	{
//		num = count2;
//		count2 = length;
//	}
//}
//count2 = 0;
//while(num>=0)
//{
//	*string = digit[num] - 0x30;
//	string++;
//	num--;
//}
//	return 1;
//}

void HAL_SYSTICK_Callback(void)
{

	LEDmsecCntr++; // New millisecond timer for LED
	state = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10);

	     if (LEDmsecCntr>3)
	     {
	    	 LEDmsecCntr=0;
	    	 LEDmsecFlag = 1; // Flag to indicate 3ms event
	     }
	     if(FLOWmsecCntr>6)
	     {
	    	 bounce = 0;
	    	 if ((prev ^ state) == 1)
	    	 {
	    		 FLOW = FLOW + 100;
	    		 prev = state;
	    	 }
	     }

	     if(bounce)
	     {
	    	 FLOWmsecCntr++;
	     }

	     if(noflow)
	     {
	     if (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10) == 1U)
	     {
	    	 bounce = 1;
	    	 noflow = 0;
	     }
	     }

}	

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	switch(channel)
	{
	case '\001':
		if(c<70)
		{
	        	ADC_CurrentArray[c]=HAL_ADC_GetValue(&hadc1);
	        	c++;
	        	channel=2;
		}
		break;
	case '\002':
		if(v<70)
		{
		        ADC_VoltageArray[v]=HAL_ADC_GetValue(&hadc1);
	        	v++;
	        	channel=3;
		}
		break;
	case '\003':
		if(t1<70)
		{
				ADC_Temp1Array[t1]=HAL_ADC_GetValue(&hadc1);
				t1++;
				channel=4;
		}
		break;
	case '\004':
		if(t2<70)
		{
				ADC_Temp2Array[t2]=HAL_ADC_GetValue(&hadc1);
				t2++;
				channel=1;
		}
		break;
	}

	if(c==70 && v==70 && t1==70 && t2==70)
		{
			c=0;
			v=0;
			t1=0;
			t2=0;
			HAL_ADC_Stop_IT(&hadc1);
		}
}

float vconvert(int16_t* v_convarray, float* v_address)
{
	int vcount = 0;
	int vsumcount = 0;
	float vnew = 0;
	float vrms = 0;
	float vsum = 0;
	float vsquare = 0;

	while (vcount<71)
	{
	vnew = *v_convarray; //turns into millivolts
	vnew = vnew - 1860.265;
	vnew = vnew * Vdd;
	vnew = vnew * 1000; //turns into millivolts
	vnew = vnew * 220;
	vnew = vnew / 4095;

	*v_address = vnew;
	v_address++;
	v_convarray++;
	vcount++;
	}

	while (vsumcount<71)
	{
		vsquare =  VALUE_VoltageArray[vsumcount] * VALUE_VoltageArray[vsumcount];
		vsum = vsum+vsquare;
		vsumcount++;
	}

	vrms = sqrtf(vsum/70);
	if (vrms>200000)
	{
		vrms=0;
	}
	return vrms;
}

float iconvert(int16_t* i_convarray, float*i_address)
{
	int icount = 0;
	float inew = 0;
	int isumcount = 0;
	float irms = 0;
	float isum = 0;
	float isquare = 0;

	while (icount<71)
	{
	inew = *i_convarray; //turns into millivolts
	inew = inew - 1860.265;
	inew = inew * Vdd;
	inew = inew * 1000; //turns into millivolts
	inew = inew * 13;
	inew = inew / 4095;

	*i_address = inew;
	i_address++;
	i_convarray++;
	icount++;
	}

	while (isumcount<71)
	{
		isquare =  VALUE_CurrentArray[isumcount] * VALUE_CurrentArray[isumcount];
		isum = isum+isquare;
		isumcount++;
	}

	irms = sqrtf(isum/70);

	if (irms>10000)
		{
			irms=0;
		}
	return irms;
}

float t1convert(int16_t* t1_convarray)
{
	float t1new = 0;

	t1new = *t1_convarray;
	t1new = t1new - 620.4545455;
	t1new = t1new * 3.3;
	t1new = t1new / 4095;
	t1new = t1new * 100; //conversion to celsius

	return t1new;
}

float t2convert(int16_t* t2_convarray)
{
	float t2new = 0;

	t2new = *t2_convarray;
	t2new = t2new - 620.4545455;
	t2new = t2new * 3.3;
	t2new = t2new / 4095;
	t2new = t2new * 100; //conversion to celsius

	return t2new;
}


uint8_t Int2String(char* output_string, int32_t val, uint8_t maxlen)
{
	if (maxlen == 0)
		return 0;

	int numwritten = 0;

	if (val < 0)
	{
		output_string[0] = '-';
		output_string++;
		maxlen--;
		val = -val;
		numwritten = 1;
	}

	uint8_t digits = 0;
	if (val < 10)
		digits = 1;
	else if (val < 100)
		digits = 2;
	else if (val < 1000)
		digits = 3;
	else if (val < 10000)
		digits = 4;
	else
		digits = 5;

	if (digits > maxlen)
		return 0; // error - not enough space in output string!

	int writepos = digits;
	while (writepos > 0)
	{
		output_string[writepos-1] = (char) ((val % 10) + 48);
		val /= 10;
		writepos--;
		numwritten++;
	}

	return numwritten;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
