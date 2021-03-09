#include "WB-PWMX.h"
#include "WB-PWMX_private.h"
#include <stdarg.h>
#include <stdbool.h>

/* Alternate function mapping table */
static TAFT hw_table_stm32f429[AFT_SIZE] = {
{PA0, {{TIM2, TIM_CHANNEL_1, 1}, {TIM5, TIM_CHANNEL_1, 2}, {0,0,0}}, GPIOA, 0},
{PA1, {{TIM2, TIM_CHANNEL_2, 1}, {TIM5, TIM_CHANNEL_2, 2}, {0,0,0}}, GPIOA, 1},
{PA2, {{TIM2, TIM_CHANNEL_3, 1}, {TIM5, TIM_CHANNEL_3, 2}, {TIM9, TIM_CHANNEL_1, 3}}, GPIOA, 2},
{PA3, {{TIM2, TIM_CHANNEL_4, 1}, {TIM5, TIM_CHANNEL_4, 2}, {TIM9, TIM_CHANNEL_1, 3}}, GPIOA, 3},
{PA5, {{TIM2, TIM_CHANNEL_1, 1}, {0, 0, 0}, {0, 0, 0}}, GPIOA, 5},
{PA6, {{TIM3, TIM_CHANNEL_1, 2}, {TIM13, TIM_CHANNEL_1, 9}, {0, 0, 0}}, GPIOA, 6},
{PA7, {{TIM3, TIM_CHANNEL_2, 2}, {TIM14, TIM_CHANNEL_1, 9}, {0, 0, 0}}, GPIOA, 7},
{PA8, {{TIM1, TIM_CHANNEL_1, 1}, {0, 0, 0}, {0, 0, 0}}, GPIOA, 8},
{PA9, {{TIM1, TIM_CHANNEL_2, 1}, {0, 0, 0}, {0, 0, 0}}, GPIOA, 9},
{PA10, {{TIM1, TIM_CHANNEL_3, 1}, {0, 0, 0}, {0, 0, 0}}, GPIOA, 10},
{PA11, {{TIM1, TIM_CHANNEL_4, 1}, {0, 0, 0}, {0, 0, 0}}, GPIOA, 11},
{PA15, {{TIM2, TIM_CHANNEL_1, 1}, {0, 0, 0}, {0, 0, 0}}, GPIOA, 15},
{PB0, {{TIM3, TIM_CHANNEL_3, 2}, {0, 0, 0}, {0, 0, 0}}, GPIOB, 0},
{PB1, {{TIM3, TIM_CHANNEL_4, 2}, {0, 0, 0}, {0, 0, 0}}, GPIOB, 1},
{PB3, {{TIM2, TIM_CHANNEL_2, 1}, {0, 0, 0}, {0, 0, 0}}, GPIOB, 3},
{PB4, {{TIM3, TIM_CHANNEL_1, 2}, {0, 0, 0}, {0, 0, 0}}, GPIOB, 4},
{PB5, {{TIM3, TIM_CHANNEL_2, 2}, {0, 0, 0}, {0, 0, 0}}, GPIOB, 5},
{PB6, {{TIM4, TIM_CHANNEL_1, 2}, {0, 0, 0}, {0, 0, 0}}, GPIOB, 6},
{PB7, {{TIM4, TIM_CHANNEL_2, 2}, {0, 0, 0}, {0, 0, 0}}, GPIOB, 7},
{PB8, {{TIM4, TIM_CHANNEL_3, 2}, {TIM10, TIM_CHANNEL_1, 3}, {0, 0, 0}}, GPIOB, 8},
{PB9, {{TIM4, TIM_CHANNEL_4, 2}, {TIM11, TIM_CHANNEL_1, 3}, {0, 0, 0}}, GPIOB, 9},
{PB10, {{TIM2, TIM_CHANNEL_3, 1}, {0, 0, 0}, {0, 0, 0}}, GPIOB, 10},
{PB11, {{TIM2, TIM_CHANNEL_4, 1}, {0, 0, 0}, {0, 0, 0}}, GPIOB, 11},
{PB14, {{TIM12, TIM_CHANNEL_1, 9}, {0, 0, 0}, {0, 0, 0}}, GPIOB, 14},
{PB15, {{TIM12, TIM_CHANNEL_2, 9}, {0, 0, 0}, {0, 0, 0}}, GPIOB, 15},
{PC6, {{TIM3, TIM_CHANNEL_1, 2}, {TIM8, TIM_CHANNEL_1, 3}, {0, 0, 0}}, GPIOC, 6},
{PC7, {{TIM3, TIM_CHANNEL_2, 2}, {TIM8, TIM_CHANNEL_2, 3}, {0, 0, 0}}, GPIOC, 7},
{PC8, {{TIM3, TIM_CHANNEL_3, 2}, {TIM8, TIM_CHANNEL_3, 3}, {0, 0, 0}}, GPIOC, 8},
{PC9, {{TIM3, TIM_CHANNEL_4, 2}, {TIM8, TIM_CHANNEL_4, 3}, {0, 0, 0}}, GPIOC, 9},
{PD12, {{TIM4, TIM_CHANNEL_1, 2}, {0, 0, 0}, {0, 0, 0}}, GPIOD, 12},
{PD13, {{TIM4, TIM_CHANNEL_2, 2}, {0, 0, 0}, {0, 0, 0}}, GPIOD, 13},
{PD14, {{TIM4, TIM_CHANNEL_3, 2}, {0, 0, 0}, {0, 0, 0}}, GPIOD, 14},
{PD15, {{TIM4, TIM_CHANNEL_4, 2}, {0, 0, 0}, {0, 0, 0}}, GPIOD, 15},
{PE5, {{TIM9, TIM_CHANNEL_1, 3}, {0, 0, 0}, {0, 0, 0}}, GPIOE, 5},
{PE6, {{TIM9, TIM_CHANNEL_2, 3}, {0, 0, 0}, {0, 0, 0}}, GPIOE, 6},
{PE9, {{TIM1, TIM_CHANNEL_1, 1}, {0, 0, 0}, {0, 0, 0}}, GPIOE, 9},
{PE11, {{TIM1, TIM_CHANNEL_2, 1}, {0, 0, 0}, {0, 0, 0}}, GPIOE, 11},
{PE13, {{TIM1, TIM_CHANNEL_3, 1}, {0, 0, 0}, {0, 0, 0}}, GPIOE, 13},
{PE14, {{TIM1, TIM_CHANNEL_4, 1}, {0, 0, 0}, {0, 0, 0}}, GPIOE, 14},
{PF6, {{TIM10, TIM_CHANNEL_1, 3}, {0, 0, 0}, {0, 0, 0}}, GPIOF, 6},
{PF7, {{TIM11, TIM_CHANNEL_1, 3}, {0, 0, 0}, {0, 0, 0}}, GPIOF, 7},
{PF8, {{TIM13, TIM_CHANNEL_1, 9}, {0, 0, 0}, {0, 0, 0}}, GPIOF, 8},
{PF9, {{TIM14, TIM_CHANNEL_1, 9}, {0, 0, 0}, {0, 0, 0}}, GPIOF, 9},
{PH6, {{TIM12, TIM_CHANNEL_1, 9}, {0, 0, 0}, {0, 0, 0}}, GPIOH, 6},
{PH9, {{TIM12, TIM_CHANNEL_2, 9}, {0, 0, 0}, {0, 0, 0}}, GPIOH, 9},
{PH10, {{TIM5, TIM_CHANNEL_1, 2}, {0, 0, 0}, {0, 0, 0}}, GPIOH, 10},
{PH11, {{TIM5, TIM_CHANNEL_2, 2}, {0, 0, 0}, {0, 0, 0}}, GPIOH, 11},
{PH12, {{TIM5, TIM_CHANNEL_3, 2}, {0, 0, 0}, {0, 0, 0}}, GPIOH, 12},
{PI0, {{TIM5, TIM_CHANNEL_4, 2}, {0, 0, 0}, {0, 0, 0}}, GPIOI, 0},
{PI2, {{TIM8, TIM_CHANNEL_4, 3}, {0, 0, 0}, {0, 0, 0}}, GPIOI, 2},
{PI5, {{TIM8, TIM_CHANNEL_1, 3}, {0, 0, 0}, {0, 0, 0}}, GPIOI, 5},
{PI6, {{TIM8, TIM_CHANNEL_2, 3}, {0, 0, 0}, {0, 0, 0}}, GPIOI, 6},
{PI7, {{TIM8, TIM_CHANNEL_3, 3}, {0, 0, 0}, {0, 0, 0}}, GPIOI, 7}
};

// Table determines a clock frequency signal connected to specific timer
static TTimerBusClock Clocks[] = 
	{
		{TIM1, APB2_TIMER_Clock}, {TIM2, APB1_TIMER_Clock}, {TIM3, APB1_TIMER_Clock}, {TIM4, APB1_TIMER_Clock}, {TIM5, APB1_TIMER_Clock},
    {TIM6, APB1_TIMER_Clock}, {TIM7, APB1_TIMER_Clock}, {TIM8, APB2_TIMER_Clock}, {TIM9, APB2_TIMER_Clock},
    {TIM10, APB2_TIMER_Clock}, {TIM11, APB2_TIMER_Clock}, {TIM12, APB1_TIMER_Clock}, {TIM13, APB1_TIMER_Clock}, {TIM14, APB1_TIMER_Clock}
	};

// Main PWM List
static TPWMChannel PWMChannelList[MAX_NUMBER_OF_CHANNELS];
static uint8_t ChannelsCounter = 0;

// All configured timers
TIM_HandleTypeDef TimersArray[MAX_NUMBER_OF_TIMERS];
static uint8_t TimersCounter = 0;
	
int getIndex(GPIOPin pin){
	for(int i = 0; i < AFT_SIZE; i++){
		if (hw_table_stm32f429[i].pin == pin){
			return i;
		}
	}
	return -1;
}

uint32_t getTimerClock(TIM_TypeDef * tim){
	for (int i = 0; i < 15; i++){
		if (Clocks[i].TimerPtr == tim){
			return Clocks[i].clock;
		}
	}
	return 0;
}

void addPort(GPIO_TypeDef* port, GPIO_TypeDef* portsArr[], uint8_t* pc){
	bool found = false;
	for (int i = 0; i < (*pc); i++){
		if (portsArr[i] == port){
			found = true;
		}
	}		
	if (!(found)){
		portsArr[(*pc)] = port;
		(*pc)++;
	}
}

void addTimer(TIM_TypeDef* timer, TIM_TypeDef* timersArr[], uint8_t* pc){
	bool found = false;
	for (int i = 0; i < (*pc); i++){
		if (timersArr[i] == timer){
			found = true;
		}
	}		
	if (!(found)){
		timersArr[(*pc)] = timer;
		(*pc)++;
	}
}

void addPWMChannel(TIM_HandleTypeDef* TimerHandlePtr, uint8_t TimerChannel, GPIOPin pin, GPIO_TypeDef* port, uint8_t pin_number, uint8_t AlternateFunction){
	TPWMChannel ch = {0};
	ch.TimerHandlePtr = TimerHandlePtr;
	ch.TimerChannel = TimerChannel;
	ch.gpio.pin = pin;
	ch.gpio.port = port;
	ch.gpio.pin_number = pin_number;
	ch.gpio.AlternateFunction = AlternateFunction;
	PWMChannelList[ChannelsCounter++] = ch;
}

// I know its a very bad code, but i have so little time...
void initializePorts(GPIO_TypeDef * portsToBeInitialized[], uint8_t portsCnt){
	for (int i = 0; i < portsCnt; i++){
		if (portsToBeInitialized[i] == GPIOA){
			__HAL_RCC_GPIOA_CLK_ENABLE();
		} else
		if (portsToBeInitialized[i] == GPIOB){
			__HAL_RCC_GPIOB_CLK_ENABLE();
		} else
		if (portsToBeInitialized[i] == GPIOC){
			__HAL_RCC_GPIOC_CLK_ENABLE();
		} else
		if (portsToBeInitialized[i] == GPIOD){
			__HAL_RCC_GPIOD_CLK_ENABLE();
		} else
		if (portsToBeInitialized[i] == GPIOE){
			__HAL_RCC_GPIOE_CLK_ENABLE();
		} else
		if (portsToBeInitialized[i] == GPIOF){
			__HAL_RCC_GPIOF_CLK_ENABLE();
		} else
		if (portsToBeInitialized[i] == GPIOG){
			__HAL_RCC_GPIOG_CLK_ENABLE();
		} else
		if (portsToBeInitialized[i] == GPIOH){
			__HAL_RCC_GPIOH_CLK_ENABLE();
		} else
		if (portsToBeInitialized[i] == GPIOI){
			__HAL_RCC_GPIOI_CLK_ENABLE();
		} else
		if (portsToBeInitialized[i] == GPIOJ){
			__HAL_RCC_GPIOJ_CLK_ENABLE();
		} else
	  if (portsToBeInitialized[i] == GPIOK){
			__HAL_RCC_GPIOK_CLK_ENABLE();
		}
	}
}

void initializeTimers(int freq, TIM_TypeDef * timersToBeInitialized[], uint8_t timersCnt){
	for (int i = 0; i < timersCnt; i++){
		// Enable clocking for timers
		if (timersToBeInitialized[i] == TIM1){
		    __HAL_RCC_TIM1_CLK_ENABLE();
		} else
		if (timersToBeInitialized[i] == TIM2){
		    __HAL_RCC_TIM2_CLK_ENABLE();
		} else
		if (timersToBeInitialized[i] == TIM3){
		    __HAL_RCC_TIM3_CLK_ENABLE();
		} else
		if (timersToBeInitialized[i] == TIM4){
		    __HAL_RCC_TIM4_CLK_ENABLE();
		} else
		if (timersToBeInitialized[i] == TIM5){
		    __HAL_RCC_TIM5_CLK_ENABLE();
		} else
		if (timersToBeInitialized[i] == TIM6){
		    __HAL_RCC_TIM6_CLK_ENABLE();
		} else
		if (timersToBeInitialized[i] == TIM7){
		    __HAL_RCC_TIM7_CLK_ENABLE();
		} else
		if (timersToBeInitialized[i] == TIM8){
		    __HAL_RCC_TIM8_CLK_ENABLE();
		} else
		if (timersToBeInitialized[i] == TIM9){
		    __HAL_RCC_TIM9_CLK_ENABLE();
		} else
		if (timersToBeInitialized[i] == TIM10){
		    __HAL_RCC_TIM10_CLK_ENABLE();
		} else
		if (timersToBeInitialized[i] == TIM11){
		    __HAL_RCC_TIM11_CLK_ENABLE();
		} else
		if (timersToBeInitialized[i] == TIM12){
		    __HAL_RCC_TIM12_CLK_ENABLE();
		} else
		if (timersToBeInitialized[i] == TIM13){
		    __HAL_RCC_TIM13_CLK_ENABLE();
		} else
		if (timersToBeInitialized[i] == TIM14){
		    __HAL_RCC_TIM14_CLK_ENABLE();
		}
		
		TIM_HandleTypeDef* timerHandle = &TimersArray[i];
		TIM_ClockConfigTypeDef sClockSourceConfig = {0};
		TIM_MasterConfigTypeDef sMasterConfig = {0};

		timerHandle->Instance = timersToBeInitialized[i];
		timerHandle->Init.Prescaler = (getTimerClock(timersToBeInitialized[i])/8000) - 1; // 8000 is enough to not overflow 
		timerHandle->Init.CounterMode = TIM_COUNTERMODE_UP;															  // 16-bit Prescaler 
		timerHandle->Init.Period = ( getTimerClock(timersToBeInitialized[i])) / (freq * (timerHandle->Init.Prescaler + 1));
		timerHandle->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		timerHandle->Init.RepetitionCounter = 0;
		timerHandle->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		if (HAL_TIM_Base_Init(timerHandle) != HAL_OK)
		{
			__disable_irq(); while(1){};
		}
		sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
		if (HAL_TIM_ConfigClockSource(timerHandle, &sClockSourceConfig) != HAL_OK)
		{
			__disable_irq(); while(1){};
		}
		if (HAL_TIM_PWM_Init(timerHandle) != HAL_OK)
		{
			__disable_irq(); while(1){};
		}
		sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
		sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
		if (HAL_TIMEx_MasterConfigSynchronization(timerHandle, &sMasterConfig) != HAL_OK)
		{
			__disable_irq(); while(1){};
		}
	}
}

void initializeGPIO_PWM(TGPIOInfo* info){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = (1 << info->pin_number);
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = info->AlternateFunction;
  HAL_GPIO_Init(info->port, &GPIO_InitStruct);
}
	
void initializePWMChannels(TPWMChannel* pwm_ch_list, uint8_t pwm_ch_cnt){
	for (int i = 0; i < pwm_ch_cnt; i++){
		TIM_OC_InitTypeDef sConfigOC = {0};
		TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
		sConfigOC.OCMode = TIM_OCMODE_PWM1;
		sConfigOC.Pulse = 0;
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
		sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
		if (HAL_TIM_PWM_ConfigChannel(pwm_ch_list[i].TimerHandlePtr, &sConfigOC, pwm_ch_list[i].TimerChannel) != HAL_OK)
		{
			__disable_irq(); while(1){};
		}
		sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
		sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
		sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
		sBreakDeadTimeConfig.DeadTime = 0;
		sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
		sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
		sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
		if (HAL_TIMEx_ConfigBreakDeadTime(pwm_ch_list[i].TimerHandlePtr, &sBreakDeadTimeConfig) != HAL_OK)
		{
			__disable_irq(); while(1){};
		}
		initializeGPIO_PWM(&pwm_ch_list[i].gpio);
		// All PWM channels starting in different time
		// TO-DO: synchronize them
	  HAL_TIM_PWM_Start(pwm_ch_list[i].TimerHandlePtr, pwm_ch_list[i].TimerChannel);
	}
}


bool parsePins(int frequency, GPIOPin* pinArr, uint8_t pc){
	// Determine wich ports and timers shoud be initialized
	GPIO_TypeDef * portsToBeInitialized[11]; uint8_t portsCnt = 0;
	TIM_TypeDef * timersToBeInitialized[14]; uint8_t timersCnt = 0;
	for (int i = 0; i < pc;  i++){
		// if sellected pin can be associated to any timer PWM output channel
		int8_t index = getIndex(pinArr[i]);
		if (index >= 0){
			  // TO-DO: refactor functions calls
				addPort(hw_table_stm32f429[index].port, &portsToBeInitialized[0], &portsCnt);
				addTimer(hw_table_stm32f429[index].functions[0].TimerPtr, &timersToBeInitialized[0], &timersCnt);
			  // TO-DO: rewrite arguments for addPWMChannel to include TAFRecord;
				addPWMChannel(&TimersArray[TimersCounter++], hw_table_stm32f429[index].functions[0].TimerChannel,
											pinArr[i], hw_table_stm32f429[index].port, hw_table_stm32f429[index].pin_number,
											hw_table_stm32f429[index].functions[0].AlternateFunction);
			
		} else {
			// Pin not supported in hardware table so we can`t initialize library
			return false;
		}
	}
	// Enabling clocks for ports
	initializePorts(&portsToBeInitialized[0], portsCnt);
	// Initialize timers
	initializeTimers(frequency, &timersToBeInitialized[0], timersCnt);
	// Configure PWMs Channels
	initializePWMChannels(&PWMChannelList[0], ChannelsCounter);
	return true;
}

PWMLibInitStatus PWM_Init(int frequency,  uint8_t NumberOfCh, GPIOPin Pin, ...){
	if ((frequency > 0) && (frequency <= 1000)){
			va_list ap;
			va_start(ap, Pin);
			GPIOPin nextPin = Pin;
			GPIOPin pins[MAX_NUMBER_OF_CHANNELS]; uint8_t pinsCnt = 0;
			pins[pinsCnt++] =  nextPin;
			for (int i = 0; i < (NumberOfCh-1); i++){
				nextPin  = (GPIOPin)va_arg(ap, int);
				pins[pinsCnt++] =  nextPin;
			}
			va_end(ap);
			if (parsePins(frequency, &pins[0], pinsCnt)){
					return OK;
			}
	 }
	 return Err;
}

void PWM_DeInit(void){
	//TO-DO: write deinitialization code
}

void pwm_out_set(uint8_t channel_index, uint16_t value){
	if ((channel_index > 0) && (channel_index <= ChannelsCounter)){
		if (value <= 100){
					TIM_OC_InitTypeDef sConfigOC = {0};
					sConfigOC.OCMode = TIM_OCMODE_PWM1;
					sConfigOC.Pulse = (value * PWMChannelList[channel_index-1].TimerHandlePtr->Init.Period)/100;
					sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
					sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
					sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
					sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
				  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
			    HAL_TIM_PWM_ConfigChannel(PWMChannelList[channel_index-1].TimerHandlePtr, &sConfigOC, PWMChannelList[channel_index-1].TimerChannel);
					HAL_TIM_PWM_Start(PWMChannelList[channel_index-1].TimerHandlePtr, PWMChannelList[channel_index-1].TimerChannel);
		}
	}
}
