/*****************************************************
Project : INPUT OUTPUT LIBRARY
Version : Library
Date    : 06/04/2015
Author  : Irham Tantowi Hamdi
Company : Teknik Elektro UB 2011
Comments:

** This library created to simplify the program for beginners
** library ini dibuat untuk mempermudah pemrograman bagi pemula

Chip type               : STM32F407VGT6
Program type            : Application
*****************************************************/

#include "TEUB_GPIO.h"

void init_IO(uint32_t RCC_AHB1Periph_GPIO, uint16_t Pin, GPIOMode_TypeDef GPIO_Mode,GPIOOType_TypeDef GPIO_OType, GPIOPuPd_TypeDef GPIO_PuPd )
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIO, ENABLE);
	switch(RCC_AHB1Periph_GPIO){
		case ((uint32_t)0x00000001) : GPIO = GPIOA; break;
		case ((uint32_t)0x00000002) : GPIO = GPIOB;	break;
		case ((uint32_t)0x00000004) : GPIO = GPIOC; break;
		case ((uint32_t)0x00000008) : GPIO = GPIOD;	break;
		case ((uint32_t)0x00000010) : GPIO = GPIOE; break;
		case ((uint32_t)0x00000020) : GPIO = GPIOF;	break;
	}

	GPIO_InitStructure.GPIO_Pin = Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode;
	GPIO_InitStructure.GPIO_OType = GPIO_OType;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd;
	GPIO_Init(GPIO, &GPIO_InitStructure);
}

void Pin(GPIO_TypeDef* GPIO,uint8_t Stat, uint16_t Pin){
	switch(Stat){
		case 0x01 : GPIO->BSRRL = Pin; break;
		case 0x02 : GPIO->BSRRH = Pin; break;
		case 0x04 : GPIO->ODR  ^= Pin; break;
	}
}
uint8_t Pin_In(GPIO_TypeDef* GPIO, uint16_t Pin){
	uint8_t bit = 0;
		if((GPIO->IDR & Pin) != 0) bit = 1;	else bit = 0;
	return bit;
}

uint8_t Pin_Out(GPIO_TypeDef* GPIO, uint16_t Pin){
	uint8_t bit = 0;
		if((GPIO->ODR & Pin) != 0) bit = 1;	else bit = 0;
	return bit;
}

void Delayms(__IO uint32_t time){
	SysTick_Config(SystemCoreClock/1000000);
	TimmingDelay = time*1000;
	while(TimmingDelay!=0);
}

void Delayus(__IO uint32_t time){
	SysTick_Config(SystemCoreClock/1000000);
	TimmingDelay = time;
	while(TimmingDelay!=0);
}

//void SysTick_Handler(void){
//	if(TimmingDelay !=0 )TimmingDelay--;
//
//	mcrs++;
//	if(mcrs%1000==0)mlls++;
//}

uint32_t micros( void ){
    uint32_t count;
    SysTick_Config(SystemCoreClock/1000000);
    do {
    	count = mcrs;
        Tick = SysTick->VAL;
    } while (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk);
    return count;
}

uint32_t milis( void ){
    uint32_t count;
    SysTick_Config(SystemCoreClock/1000000);
    do {
    	count = mlls;
        Tick = SysTick->VAL;
    } while (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk);
    return count;
}

void RST_cnt(){
	mlls=0;
	mcrs=0;
}

void Delay_ms(__IO uint32_t nCount){
	Count = nCount * 15272;
	while(Count--);
}
