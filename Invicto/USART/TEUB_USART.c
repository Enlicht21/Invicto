/*****************************************************
Project : USART LIBRARY
Version : Library
Date    : 07/04/2015
Author  : Irham Tantowi Hamdi
Company : Teknik Elektro UB 2011
Comments:

** This library created to simplify the program for beginners
** library ini dibuat untuk mempermudah pemrograman bagi pemula

Chip type               : STM32F407VGT6
Program type            : Application
*****************************************************/

#include "TEUB_USART.h"

void init_USART(uint32_t RCC_AHB1Periph_USART, uint32_t baudrate, uint32_t RCC_AHB1Periph_GPIO, uint16_t Pin, uint8_t STATUS){

	switch(RCC_AHB1Periph_GPIO){
		case ((uint32_t)0x00000001) : GPIO = GPIOA; break;
		case ((uint32_t)0x00000002) : GPIO = GPIOB;	break;
		case ((uint32_t)0x00000004) : GPIO = GPIOC; break;
		case ((uint32_t)0x00000008) : GPIO = GPIOD;	break;
		case ((uint32_t)0x00000010) : GPIO = GPIOE; break;
		case ((uint32_t)0x00000020) : GPIO = GPIOF;	break;
	}

	init_IO(RCC_AHB1Periph_GPIO, Pin, GPIO_Mode_AF,GPIO_OType_PP, GPIO_PuPd_UP);

	switch(RCC_AHB1Periph_USART){
		case ((uint32_t)0x00000010) : {
			GPIO_AF = GPIO_AF_USART1; USART = USART1; RCC_APB2PeriphClockCmd(RCC_AHB1Periph_USART, ENABLE); USART_INT = USART1_IRQn; break;
		}
		case ((uint32_t)0x00020000) : {
			GPIO_AF = GPIO_AF_USART2; USART = USART2; RCC_APB1PeriphClockCmd(RCC_AHB1Periph_USART, ENABLE); USART_INT = USART2_IRQn; break;
		}
		case ((uint32_t)0x00040000) : {
			GPIO_AF = GPIO_AF_USART3; USART = USART3; RCC_APB1PeriphClockCmd(RCC_AHB1Periph_USART, ENABLE); USART_INT = USART3_IRQn; break;
		}
		case ((uint32_t)0x00000020) : {
			GPIO_AF = GPIO_AF_USART6; USART = USART6; RCC_APB2PeriphClockCmd(RCC_AHB1Periph_USART, ENABLE); USART_INT = USART6_IRQn; break;
		}
	}

	for(n=0; n<16; n++) if((Pin & (0x1<<n)) == (0x1<<n)) GPIO_PinAFConfig(GPIO, n, GPIO_AF);

	USART_InitStruct.USART_BaudRate = baudrate;			// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;	// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)

	if(STATUS == TX) USART_InitStruct.USART_Mode = USART_Mode_Tx;
	else if(STATUS == RX) USART_InitStruct.USART_Mode = USART_Mode_Rx;
	else if(STATUS == TX_RX) USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

	if(STATUS != TX){
		USART_ITConfig(USART, USART_IT_RXNE, ENABLE); // enable the USART receive interrupt
		NVIC_InitStructure.NVIC_IRQChannel = USART_INT;		 // we want to configure the USART3 interrupts
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART3 interrupts
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		 // this sets the subpriority inside the group
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART3 interrupts are globally enabled
		NVIC_Init(&NVIC_InitStructure);							 // the properties are passed to the NVIC_Init function which takes care of the low level stuff
	}

	USART_Init(USART, &USART_InitStruct);							// again all the properties are passed to the USART_Init function which takes care of all the bit setting
	USART_Cmd(USART, ENABLE);
}

void cetak(USART_TypeDef* USARTx, const char *pFormat, ... )
{
    va_list ap;
    char pStr[100];

    va_start(ap, pFormat);
    vsprintf(pStr, pFormat, ap);
    va_end(ap);

    int i=0;
    int n = strlen(pStr);
    for(i=0;i<n;i++)
    {
        USART_SendData(USARTx, (uint8_t)pStr[i]);
        while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
    }
}

void cetak_float(USART_TypeDef* USARTx, float data, int counter)
{
    char pStr[100];

	gcvt(data, counter , pStr);

    int i=0;
    int n = strlen(pStr);
    for(i=0;i<n;i++)
    {
        USART_SendData(USARTx, (uint8_t)pStr[i]);
        while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
    }
}


