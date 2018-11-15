#include "main.h"
#include "stm32f4xx_hal.h"
#include <string.h>

/*
 * Driver for HC-SR04 ultrasonic distance sensor
 * for STM32-F401RE ARM Processor.
 * To get a reading from the sensor the Trig pin must be held high for 10us
 * the echo pin then goes high for X amount of time, which corresponds with
 * the time of flight of the ultrasonic burst. Once we have the time we divide by
 * the speed of sound and then divide by two to get the distnance from the object.
 * The code uses Output Compare to generate the 10us trigger and Input Compare to
 * determine the time taken. The IC is set in DMA mode and the Trigger timer is set
 * to fire only once (each time getReading() is called).
 * Uses pins D8 and D12 on nucleo (Arduino headers)
 * Take measurements at least 60ms apart.
 */


void SystemClock_Config(void);
void UART_Init();
void GPIO_Init();
void TIM3_Init();
void TIM1_Init();
void DMA_Init();
float computeDistance(void);  /*call after calling getReadings
				 *and checking that captureDone flag is set*/
void  getReadings(void);      /*Starts both timers and gets the readings */

UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim3, htim1;
DMA_HandleTypeDef hdma_tim3_IC;

volatile uint16_t captures[] = { 0, 0 };
volatile uint8_t captureDone = 0, secondValueUpdated = 0; /* when captureDone is set,
							computeDistance can be called */


int main(void) {

	HAL_Init();
	SystemClock_Config();

	GPIO_Init();  /* Init GPIO */
	DMA_Init();   /* Init DMA */
	UART_Init();  /* Init UART */
	TIM3_Init();  /* Init Timer 3 for IC */
	TIM1_Init();  /* Init Timer 1 for OC */

	HAL_UART_Transmit(&huart2, (uint8_t*) "Starting...\r\n", strlen("Starting...\r\n"), HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);


	while (1) {
		getReadings();
		while (captureDone == 1) {

			captureDone = 0;
			char msg[50];

			sprintf(msg, "Distance: %.10f\r\n", computeDistance());
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
					HAL_MAX_DELAY);

		}
		HAL_Delay(200);
	}

}

void  getReadings(void){

	HAL_TIM_IC_Start_DMA(&htim3, TIM_CHANNEL_1, (uint32_t*) captures, 2);
	HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_2);

}
float computeDistance(void) {
	float distance = 0;
	uint16_t diffCapture = 0;

	/*Computes time of flight */
	if (captures[1] >= captures[0])
		diffCapture = captures[1] - captures[0];

	else
		diffCapture = (htim3.Instance->ARR - captures[0]) + captures[1];

	distance = 84000000 / (htim3.Instance->PSC + 1);
	distance = (float) ((diffCapture / distance) * 340) / 2; /*Convert readings into distance */
	return distance;
}
void GPIO_Init() {
	GPIO_InitTypeDef GPIO_InitStruct;

	__GPIOA_CLK_ENABLE();

	/*Initialise LED*/
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
void UART_Init() {

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 38400;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	HAL_UART_Init(&huart2);

}
void TIM3_Init() {
	TIM_IC_InitTypeDef sConfigIC;

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 83; /*lower the timer frequency to 1Mhz */
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_IC_Init(&htim3);

	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 4;
	HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1);
}
void DMA1_Stream4_IRQHandler(void) {
	HAL_DMA_IRQHandler(&hdma_tim3_IC);
}
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim) {

	GPIO_InitTypeDef GPIO_InitStruct;

	if (htim->Instance == TIM3) {
		__TIM3_CLK_ENABLE();

		GPIO_InitStruct.Pin = GPIO_PIN_6;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM3_IRQn);

		__HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
		__HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC1);

		hdma_tim3_IC.Instance = DMA1_Stream4;
		hdma_tim3_IC.Init.Channel = DMA_CHANNEL_5;
		hdma_tim3_IC.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_tim3_IC.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_tim3_IC.Init.MemInc = DMA_MINC_ENABLE;
		hdma_tim3_IC.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
		hdma_tim3_IC.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
		hdma_tim3_IC.Init.Mode = DMA_NORMAL;
		hdma_tim3_IC.Init.Priority = DMA_PRIORITY_LOW;
		hdma_tim3_IC.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		HAL_DMA_Init(&hdma_tim3_IC);

		__HAL_LINKDMA(htim, hdma[TIM_DMA_ID_CC1], hdma_tim3_IC);

	}
}

void DMA_Init(void) {

	__DMA1_CLK_ENABLE();
	__DMA2_CLK_ENABLE();

	HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
}
void TIM3_IRQHandler() {

	HAL_TIM_IRQHandler(&htim3);
}
void TIM1_UP_TIM10_IRQHandler(void) {
	//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	HAL_TIM_IRQHandler(&htim1);

}
void TIM1_CC_IRQHandler() {
	HAL_TIM_IRQHandler(&htim1);
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		captureDone = 1;
	}
}

void TIM1_Init(void) {
	TIM_OC_InitTypeDef sConfig;

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 83;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	HAL_TIM_OC_Init(&htim1);

	htim1.Instance->CR1 &= ~TIM_CR1_OPM;
	htim1.Instance->CR1 |= TIM_OPMODE_SINGLE; /*Single shot mode enabled */

	sConfig.OCMode = TIM_OCMODE_TOGGLE;
	sConfig.Pulse = 10;
	sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
	HAL_TIM_OC_ConfigChannel(&htim1, &sConfig, TIM_CHANNEL_2);
}
void HAL_TIM_OC_MspInit(TIM_HandleTypeDef* htim_base) {

	GPIO_InitTypeDef GPIO_InitStruct;
	if (htim_base->Instance == TIM1) {

		__HAL_RCC_TIM1_CLK_ENABLE();

		GPIO_InitStruct.Pin = GPIO_PIN_9;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


		HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);

		HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

		__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
		__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
	}

}
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
		if (secondValueUpdated == 0) {
			/* Set the Capture Compare Register value */
			__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, 20);
		}
		secondValueUpdated = 1;
	}

}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM1) {
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}
	if (htim->Instance == TIM3) {

	}
}
