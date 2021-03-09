/* PWM library header file 
Dev: Anton Shulyaev 08.03.21
*/

#ifndef WB_PWMX_H
#define WB_PWMX_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

#define MAX_NUMBER_OF_CHANNELS      32
#define MAX_NUMBER_OF_TIMERS        4
#define AFT_SIZE                    53
#define AFT_VARIATIONS              3
#define APB1_TIMER_Clock            90000000
#define APB2_TIMER_Clock            180000000

typedef enum {OK, NotSupported, Err} PWMLibInitStatus;
typedef enum {PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA8, PA9, PA10, PA11, PA12, PA13, PA14, PA15,
						  PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9, PB10, PB11, PB12, PB13, PB14, PB15,
						  PC0, PC1, PC2, PC3, PC4, PC5, PC6, PC7, PC8, PC9, PC10, PC11, PC12, PC13, PC14, PC15,
							PD0, PD1, PD2, PD3, PD4, PD5, PD6, PD7, PD8, PD9, PD10, PD11, PD12, PD13, PD14, PD15,
							PE0, PE1, PE2, PE3, PE4, PE5, PE6, PE7, PE8, PE9, PE10, PE11, PE12, PE13, PE14, PE15,
						  PF0, PF1, PF2, PF3, PF4, PF5, PF6, PF7, PF8, PF9, PF10, PF11, PF12, PF13, PF14, PF15,			
							PG0, PG1, PG2, PG3, PG4, PG5, PG6, PG7, PG8, PG9, PG10, PG11, PG12, PG13, PG14, PG15,
							PH0, PH1, PH2, PH3, PH4, PH5, PH6, PH7, PH8, PH9, PH10, PH11, PH12, PH13, PH14, PH15,
						  PI0, PI1, PI2, PI3, PI4, PI5, PI6, PI7, PI8, PI9, PI10, PI11, PI12, PI13, PI14, PI15,
							PJ0, PJ1, PJ2, PJ3, PJ4, PJ5, PJ6, PJ7, PJ8, PJ9, PJ10, PJ11, PJ12, PJ13, PJ14, PJ15,
							PK0, PK1, PK2, PK3, PK4, PK5, PK6, PK7, PK8, PK9, PK10, PK11, PK12, PK13, PK14, PK15
} GPIOPin;

typedef struct GPIOInfo_t {
	GPIOPin pin;
	GPIO_TypeDef* port;
	uint8_t pin_number;
  uint8_t AlternateFunction;
} TGPIOInfo;


/** PWM Configurator
		Before using library this function shoud be called first
    to configure all necessary perephery
		@param frequency pulse frequency in Hz
		@param NumberOfCh how much PWM channels to initialize
		@param pin PWM output pin name
		@param ... comma separated pin names
**/
PWMLibInitStatus PWM_Init(int frequency, uint8_t NumberOfCh, GPIOPin Pin, ...);

/** PWM Library deinitialization **/
void PWM_DeInit(void);

/** Configure duty cycle on sellected PWM channel 
		@param channel_index obviously its a PWM channel number
		@param value detemines a PWM dute cycle in % (100% - always high lvl, 0% - always low lvl)
**/
void pwm_out_set(uint8_t channel_index, uint16_t value);

#endif
