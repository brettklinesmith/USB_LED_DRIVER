/**
 ******************************************************************************
 * File Name          : main.c
 * Note for GitHub    : Looks best with tabs and indent size of 4
 * Description        : All the work and magic pixie dust to make the LED
 * 						driver work.
 *
 * 						TODO:
 * 						change to current pattern in RAM
 * 						modify working pattern over USB
 * 							design communication protocol
 * 								read
 * 								write
 * 								swap??
 * 								remove??
 * 						store working pattern to ROM
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "usb_device.h"

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;						//From CubeMX
TIM_HandleTypeDef htim3;					//From CubeMX

// Basically bools
volatile uint8_t 	updatePWM 		= 1,	// Will cause the PWM period to step
					changePattern 	= 0;	// Change to next saved pattern

uint16_t 			fadeOngoing 	= 0;	// bool for fading TODO: could this be a uint8_t?

// Timing values
volatile uint32_t 	currentTicks 	= 0,	// Value of SYSTICK
					updateTicks 	= 1,	// Value of next PWM update
					nextUpdateTime 	= 1,	// Duration of current output
					fadeTicks 		= 0,	// Duration of fade
					endOfFade 		= 0,	// Systick value at end of fade
					stateTicks 		= 0,	// Duration of state
					endOfState 		= 0,	// Systick value at end of state
					*nextState;				// Pointer to first word of next state



/* Initialize patterns. Basically a Malloc for pattern memory */
static __attribute__((section("PATTERN_1")))   const volatile uint32_t pattern1[250];	// ROM
static __attribute__((section("PATTERN_2")))   const volatile uint32_t pattern2[250];
static __attribute__((section("PATTERN_3")))   const volatile uint32_t pattern3[250];
static __attribute__((section("PATTERN_4")))   const volatile uint32_t pattern4[250];
static __attribute__((section("PATTERN_5")))   const volatile uint32_t pattern5[250];

volatile uint32_t workingPattern[250];	// RAM

// Array of pointers to patterns to allow pattern changes and index for current pattern
const volatile uint32_t *patternAddess[5] = { &pattern1, &pattern2, &pattern3,
						&pattern4, &pattern5 };
uint8_t patternIndex = 0;

/* Private function prototypes -----------------------------------------------*/
void 		SystemClock_Config(void);						// Configure clocks
static void MX_GPIO_Init(void);								// Setup GPIO (HAL)
static void MX_CRC_Init(void);								// Setup CRC timer (HAL)
static void MX_TIM3_Init(void);								// Setup PWM timer (HAL)
static void SYSTICK_Init(void);								// Setup Systick (HAL)
void 		HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);	// PWM middle step? (HAL)
void 		set_pwm_value(uint16_t[4]);						// Change PWM periods to these values
void 		decodeState(uint32_t, uint32_t, uint16_t *);	// From words in memory to pattern state
uint16_t 	decodeTime(uint16_t);							// Interpret time value
void		arrayCopy(volatile uint32_t[250],const volatile uint32_t[250]);	// To handle array assignment because C doesn't
void		setPattern(uint8_t);							// Change to given pattern number
void		inputHandler(uint8_t*, uint32_t);				// Handle input over virtual COM port

int main(void) {

	// Reset of all peripherals, Initializes the Flash interface and the Systick.
	HAL_Init();

	// Configure the system clocks
	SystemClock_Config();

	// Initialize all configured peripherals
	MX_GPIO_Init();
	MX_TIM3_Init();
	MX_USB_DEVICE_Init();
	MX_CRC_Init();
	SYSTICK_Init();

	//Start the PWM timers
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	uint16_t 			PWMPeriods[4] = { 0, 0, 0, 0 },		// Current/next PWM periods
						targetPeriods[4] = { 0, 0, 0, 0 },	// PWM periods fading to
						fadeStepTicks = 10;   				// # of ms between fade steps

	int16_t 			offsetPeriods[4] = { 0, 0, 0, 0 };	// Fade change per step

	uint32_t 			stateFirst,							// First 32-bit word of pattern state
						stateSecond;						// Second 32-bit word of pattern state

	setPattern(patternIndex);
	// TODO delete -> arrayCopy(workingPattern,pattern1);				// Start with first pattern
	//workingPattern = pattern1;

	//nextState = &workingPattern[0];					// Initialize nextState

	while (!(RCC->CR & RCC_CR_HSERDY));				// Wait until external oscillator has settled

	// temporary code for debugging
	uint8_t temp1 = 6;

	inputHandler(&temp1,100);
	// end of temporary code

	while (1) {
		/* When updatePWM has a value of 1 the SysTick interrupt has determined
		 * that the current state time has elapsed. The next state is read and
		 * interpreted from memory and then current PWM periods are updated.
		 */

		if (updatePWM != 0) {

			if (fadeOngoing == 1) {

				// Last step of fade jumps values to target period from memory
				if ((HAL_GetTick()) > endOfFade) {
					PWMPeriods[0] = targetPeriods[0];
					PWMPeriods[1] = targetPeriods[1];
					PWMPeriods[2] = targetPeriods[2];
					PWMPeriods[3] = targetPeriods[3];

					fadeOngoing = 0;
					updateTicks = endOfState;
				} else {  // Any other step of fade offsets by change per step
					PWMPeriods[0] += offsetPeriods[0];
					PWMPeriods[1] += offsetPeriods[1];
					PWMPeriods[2] += offsetPeriods[2];
					PWMPeriods[3] += offsetPeriods[3];

					nextUpdateTime = fadeStepTicks;
				}
				
				set_pwm_value(PWMPeriods);				// Commit new PWM periods
				
			} else {
				// Should only run once per state at the very end of the fade
				// TODO: check if this runs first or last now

				// read next state from memory
				stateFirst = *nextState;
				nextState++;
				stateSecond = *nextState;

				// check for EoF
				if ((stateSecond & 0x3) != 0x3) {
					nextState++;						// If no EoF then increment to next pattern state
				} else {
					nextState = &workingPattern[0]; 	// Reset to beginning of pattern
				}

				decodeState(stateFirst, stateSecond, &targetPeriods); // Interpret pattern state

				// Calculate change per fade step to achieve target period (cast everything for safety)
				// TODO: redesign this so it handles large and small changes in the same algorithm
				offsetPeriods[0] = ((int16_t) targetPeriods[0] - (int16_t) PWMPeriods[0]) / (int16_t) (fadeTicks / fadeStepTicks);
				offsetPeriods[1] = ((int16_t) targetPeriods[1] - (int16_t) PWMPeriods[1]) / (int16_t) (fadeTicks / fadeStepTicks);
				offsetPeriods[2] = ((int16_t) targetPeriods[2] - (int16_t) PWMPeriods[2]) / (int16_t) (fadeTicks / fadeStepTicks);
				offsetPeriods[3] = ((int16_t) targetPeriods[3] - (int16_t) PWMPeriods[3]) / (int16_t) (fadeTicks / fadeStepTicks);

				fadeOngoing = 1; 					// Set flag to begin fade after update interrupt

				nextUpdateTime = fadeStepTicks;		// Store time to read next pattern state
			}

			//set_pwm_value(PWMPeriods);				// Commit new PWM periods

			updatePWM = 0;							// Reset PWM update flag
		}

		//If button is pressed then change patterns
		if (changePattern == 1) {

			//Check if current pattern is last one and inncrement appropriately
			if (patternIndex < 4) {
				patternIndex++;
			} else {
				patternIndex = 0;
			}

			setPattern(patternIndex);				// Change to next pattern
			
			changePattern = 0;						// Reset pattern change flag
		}
	}

}

/** System Clock Configuration
 *  Unchanged from CubeMX import
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_1);

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CRC init function 
 *  Unchanged from CubeMX import
 */
static void MX_CRC_Init(void) {

	hcrc.Instance = CRC;
	if (HAL_CRC_Init(&hcrc) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM3 init function 
 * Mostly unchanged from CubeMX import
 * Only PWM resolution and initial period changed 
 */
static void MX_TIM3_Init(void) {

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 0xFFF;					// Resolution of PWM timer
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0x000;					// Channel 1 initial PWM period
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.Pulse = 0x000;					// Channel 2 initial PWM period
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.Pulse = 0x000;					// Channel 3 initial PWM period
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.Pulse = 0x000;					// Channel 4 initial PWM period
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim3);

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 PA2   ------> USART2_TX
 PA3   ------> USART2_RX
 PA8   ------> RCC_MCO
  
 * Unchanged from CubeMX import
 */
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	// GPIO Ports Clock Enable
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	// Configure GPIO pin : B1_Pin
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	// Configure GPIO pins : USART_TX_Pin USART_RX_Pin
	GPIO_InitStruct.Pin = USART_TX_Pin | USART_RX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Configure GPIO pin : LD2_Pin
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);	// Initialize interrupt for button press
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* SysTick init function  
 * Unchanged from CubeMX import
 */
static void SYSTICK_Init(void) {
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	if (HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000) != HAL_OK) {// Configure SysTick to generate an interrupt every millisecond
		_Error_Handler(__FILE__, __LINE__);
	}
}

/* Function to commit new PWM periods */
void set_pwm_value(uint16_t periods[4]) {
	TIM_OC_InitTypeDef sConfigOC;

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = periods[0];
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.Pulse = periods[1];
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.Pulse = periods[2];
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.Pulse = periods[3];
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	//Must restart PWM outputs (TIM_CHANNEL_ALL doesn't work)
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

/* A single pattern STEP requires two 32-bit registers. The have the form:
 * [red 10-bit value]   [green 10-bit value] [blue 10-bit value]  [command 2-bit]
 * [white 10-bit value] [fade 10-bit value]  [dwell 10-bit value] [command 2-bit]
 *
 * Command bits are 0x01 for first step register, 0x10 for second step register.
 * If the second step register is 0x11 that signifies the end of the pattern (EOF).
 *
 * This function takes two 32-bit words from memory and extracts the 4 PWM periods,
 * fade time, and dwell time. While extracting PWM period a 10-bit value from the
 * pattern step is basically left shifted 2 bits to fit put the values into the MSBs
 * of the 12-bit resolution of PWM timer. Techically 99.993% is max intensity.
 */
void decodeState(uint32_t first, uint32_t second, uint16_t *periods) {

	*periods = ((first & 0xFFC00000) >> 20);				// Channel 1 PWM period
	periods++;
	*periods = ((first & 0x003FF000) >> 10);				// Channel 2 PWM period
	periods++;
	*periods = ((first & 0x00000FFC) >> 0);					// Channel 3 PWM period
	periods++;
	*periods = ((second & 0xFFC00000) >> 20);				// Channel 4 PWM period

	stateTicks 	= decodeTime((second & 0xFFC) >> 2);		// Get dwell time
	fadeTicks 	= decodeTime((second & 0x3FF000) >> 12);	// Get fade time

	//Store relevant times for fade and PWM update
	endOfFade = HAL_GetTick() + fadeTicks;
	endOfState = endOfFade + stateTicks;
}

/* Extract encoded time value
 *
 * Fade and dwell times have 2 components. The two MSB bits define the time base:
 * 		00 = 100ms 		range of    0:25.5 -    0:00.1
 * 		01 = seconds	range of    4:15.0 -    0:01.0
 * 		10 = 5 seconds	range of   21:15.0 -    0:05.0
 * 		11 = minutes	range of 4:15:00.0 - 0:01:00.0
 *
 * The eight LSB bits of the time values define the actual value in the form:
 *
 * 		[8-bit value] * timebase
 *
 * For patterns of greater timing resolution multiple pattern values should
 * be used with the same color values, a fade of 0 ms, and the necessary
 * dwell time to achieve the desired total dwell time.
 *
 * 		ie. for 5 minutes and 17.6 seconds
 * 			10 * 3C   //5 minutes
 * 			00 * B0   //17.6 seconds
 */
uint16_t decodeTime(uint16_t time) {
	int timeBase;
	timeBase = ((time & 0x300) >> 8);

	switch (timeBase) {
	case 0x0:
		return (100 * (time & 0xFF));
		break;

	case 0x1:
		return (1000 * (time & 0xFF));
		break;

	case 0x2:
		return (5000 * (time & 0xFF));
		break;

	case 0x3:
		return (60000 * (time & 0xFF));
		break;
	}

	return (0);
}

// TODO: optimize for shorter patterns
void arrayCopy(volatile uint32_t destination[250],const volatile uint32_t source[250]){
	for(int i = 0; i < 250; i++){
		destination[i] = source[i];
	}
}

void setPattern(uint8_t number){
	patternIndex = number;

	arrayCopy(workingPattern,patternAddess[patternIndex]);

	nextState = &workingPattern[0];			// Restart pattern

	fadeOngoing = 0;						// Clear fade flag so state is read from memory
	updatePWM = 1;							// Force PWM period update
}

void inputHandler(uint8_t* buffer, uint32_t length){
	uint8_t index;

	//TODO: change to switch case statement
	if(*buffer == 0) {
		if(*(buffer+1) < 5 ){
			setPattern(*(buffer+1));
			CDC_Transmit_FS("Set pattern number\n",18);
		} else {
			CDC_Transmit_FS("Set pattern error\n",17);
		}

	} else if(*buffer == 2) {
		CDC_Transmit_FS(&patternIndex,1);
	} else if(*buffer == 4) {
		CDC_Transmit_FS("Set pattern\n",11);
	} else if(*buffer == 6) {
		//CDC_Transmit_FS("Get pattern\n",11);

		index = 1;

		while(index<250){
			if((workingPattern[index] & 0x3) == 0x3) break;
			index += 2;				// increment by 2 because a pattern step is 64-bits
		}

		index++;	//Must increment because of 0 vs 1 based arithmetic

		uint32_t reversed[index],temp;

		/* Reversing algorithm based on info from from:
		   https://graphics.stanford.edu/~seander/bithacks.html#BitReverseObvious*/

		for(uint8_t i=0 ; i < index; i++){
			temp = workingPattern[i];
		    temp = (((temp & 0xff00ff00) >> 8) | ((temp & 0x00ff00ff) << 8));
		    reversed[i] = ((temp >> 16) | (temp << 16));

		}

		CDC_Transmit_FS(reversed,(index+1)*4); //TODO: double check if +1 still necessary
	} else {
		// If not valid input spit out the recieved buffer
		index=0;

		while(index<length){
			if(*(buffer + index) == 0x0D) break;
			index++;
		}

		CDC_Transmit_FS(buffer,index);
	}
}

/* SysTick interrupt handler */
void SysTick_Handler(void) {
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();

	currentTicks = HAL_GetTick();		// Get current ticks value
	if (currentTicks >= updateTicks) {	// Check if it's time to update PWM
		updatePWM = 1;					// If so set flag and
		updateTicks += nextUpdateTime;	// Store time for next update
	}
}

/* If button is pressed set changePattern flag */
void EXTI15_10_IRQHandler(void) {
	__HAL_GPIO_EXTI_CLEAR_IT(B1_Pin);
	changePattern = 1;
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 * TODO: Either implement or remove. Prefer implement
 */
void _Error_Handler(char * file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 * TODO: Either implement or remove. Prefer implement
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif
