/* PWM library private header file 
Dev: Anton Shulyaev 09.03.21
*/

#ifndef PWMX_PRIVATE_H
#define PWMX_PRIVATE_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

typedef struct PWMChannel_t {
	TIM_HandleTypeDef* TimerHandlePtr;
  uint8_t TimerChannel;
  TGPIOInfo gpio;
} TPWMChannel;

typedef struct TimerBusClosk_t {
	TIM_TypeDef * TimerPtr;
	uint32_t clock;
} TTimerBusClock;	

typedef struct AFRecord_t{
	TIM_TypeDef * TimerPtr;
	uint8_t TimerChannel;
	uint8_t AlternateFunction;
} TAFRecord;

typedef struct aft_t {
	GPIOPin pin;
	TAFRecord functions[AFT_VARIATIONS];
	GPIO_TypeDef* port;
	uint8_t pin_number;
} TAFT;

#endif
