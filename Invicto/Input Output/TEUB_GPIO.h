#ifndef IRHAM_GPIO_H
#define IRHAM_GPIO_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

#define P_0  	GPIO_Pin_0
#define P_1  	GPIO_Pin_1
#define P_2  	GPIO_Pin_2
#define P_3  	GPIO_Pin_3
#define P_4  	GPIO_Pin_4
#define P_5  	GPIO_Pin_5
#define P_6  	GPIO_Pin_6
#define P_7  	GPIO_Pin_7
#define P_8  	GPIO_Pin_8
#define P_9  	GPIO_Pin_9
#define P_10  	GPIO_Pin_10
#define P_11  	GPIO_Pin_11
#define P_12  	GPIO_Pin_12
#define P_13  	GPIO_Pin_13
#define P_14  	GPIO_Pin_14
#define P_15  	GPIO_Pin_15

#define ON 	0x01
#define OFF 0x02
#define TO 	0x04

#define SYSTICK_RELOAD_VAL 0xA80000
#define SYSTICK_DIV(x) ((x*0x0186)>>16)

GPIO_TypeDef* 		GPIO;
GPIO_InitTypeDef   	GPIO_InitStructure;

static __IO uint32_t 		TimmingDelay;
unsigned int 		Tick;
uint32_t 			mlls;
uint32_t 			mcrs;
uint32_t 			Count;

void init_IO(uint32_t RCC_AHB1Periph_GPIO, uint16_t Pin, GPIOMode_TypeDef GPIO_Mode,GPIOOType_TypeDef GPIO_OType, GPIOPuPd_TypeDef GPIO_PuPd );
void Pin(GPIO_TypeDef* GPIO,uint8_t Stat, uint16_t Pin);
uint8_t Pin_In(GPIO_TypeDef* GPIO, uint16_t Pin);
uint8_t Pin_Out(GPIO_TypeDef* GPIO, uint16_t Pin);
void Delayms(__IO uint32_t time);
void Delayus(__IO uint32_t time);
void SysTick_Handler(void);
void RST_cnt();

uint32_t micros(void);
uint32_t milis(void);

void Delay_ms(__IO uint32_t nCount);

#ifdef __cplusplus
}
#endif

#endif
