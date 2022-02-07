#ifndef IRHAM_USART_H
#define IRHAM_USART_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#include "TEUB_GPIO.h"

#include "stdio.h"
#include "stdlib.h"
#include "stdarg.h"

#include "misc.h"

USART_InitTypeDef 	USART_InitStruct; 		// this is for the USART3 initilization
NVIC_InitTypeDef 	NVIC_InitStructure; 	// this is used to configure the NVIC (nested vector interrupt controller)

typedef enum
{
  TX   	= 0x00, /*!< GPIO Input Mode */
  RX  	= 0x01, /*!< GPIO Output Mode */
  TX_RX = 0x02, /*!< GPIO Output Mode */
}USART_EN;

typedef struct{
	char str[100];
	uint8_t isi;
	uint8_t cnt;
}TERIMA;

TERIMA US1;
TERIMA US2;
TERIMA US3;
TERIMA US6;

GPIO_TypeDef* 		GPIO;
USART_TypeDef* 		USART;
uint8_t				USART_INT;
uint8_t 			n;
uint8_t 			GPIO_AF;


void init_USART(uint32_t RCC_AHB1Periph_USART, uint32_t baudrate, uint32_t RCC_AHB1Periph_GPIO, uint16_t Pin, uint8_t STATUS);
void cetak(USART_TypeDef* USARTx, const char *pFormat, ... );
void cetak_float(USART_TypeDef* USARTx, float data, int counter);

#ifdef __cplusplus
}
#endif

#endif
