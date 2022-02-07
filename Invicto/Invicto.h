#ifndef __Invicto_H
#define __Invicto_H

#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_usart.h"
#include "misc.h"
#include "TEUB_USART.h"
#include "TEUB_GPIO.h"

//RealtimeOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#define mtr1 		0x1
#define mtr2 		0x2
#define mtr3 		0x3
#define mtr4		0x4
#define mtr5		0x5
#define mtr6		0x6
#define mtr7		0x10
#define maju 		0x7
#define mundur 		0x8
#define stop 		0x9

#define pi 			3.1416
//
//CCR Val
#define CCR1_Val 5000
#define CCR2_Val 50000
#define CCR3_Val 1000
#define CCR4_Val 65535

uint16_t ennow1, ennow2, ennow3,ennow4,ennow5,ennow6,ennow7;
int16_t delta1, delta2, delta3,delta4,delta5,delta6,delta7;
volatile int16_t encoderlast1, encoderlast2,encoderlast3,encoderlast4,encoderlast5
				,encoderlast6,encoderlast7,encoder1,encoder2,encoder3,encoder4;
volatile int rpm1,rpm2,rpm3,rpm4,rpm5,rpm6,rpm7,ex_rpm1,ex_rpm2,ex_rpm3;
float I_error1,I_error2,I_error3,I_error4;
uint16_t duty1,duty2,duty3,duty4,d1,d2,d3;

int point,rximu[6],capture,bit_ke,prt[9],btstate,btcheck,btwait,count1,count2,count3,cnt_shagai,mode_odo,tone,no_sensor;
unsigned char rxdata[9],slect,L1,L2,L3,R1,R2,R3,start,up,right,down,left,X,O,A,Q,od[2],hcsr[3];
long int speed[9];
float er[4],sudut,sudut_left,tmp_sudut_left,sudut_right,tmp_sudut_right,sudut_smt,yaw,vX,vY,theta1,theta2,theta3,tmp[4],runyaw,t1,t2,t3,t4;
int aksel_v,aksel_grip,track,bitimu,anarix,anariy,analex,analey;
int press,push,pneumod,hold,mode_stick,kec,sdt,test_mode,mode_imu,shagai,mode_lapangan,button[5],xx,hcsr1,hcsr2,hcsr3,ls_shagai;
float vx,vy,sudut_in,eks,Y,Vx,Vy,sudut_w,ww;
int deadband1,deadband2,deadband3,deadband4;
float degpertick2,deltaTinSec2,ticksPerSec2;
float degPerSec2,lastSpeed2;
volatile int exenc1,exenc2,exenc3,exenc4;
float time_taken,L,R,pose_x,pose_y,pose_theta;
int timecurrent,timeprev;
float velo,oldVel, velVec[40],out_old;

/* Mode */
#define V12
//#define V24

#ifdef V12
	#define fpx 		2.2
	#define aksel_spd 	300
	#define aksel_std 	200
#endif

#ifdef V24
	#define fpx 		1.3
	#define aksel_spd 	450
	#define aksel_std 	300
	#define aksel_dec	200
#endif

#define merah	0
#define biru	1


//
GPIO_InitTypeDef 			GPIO_InitStruct;
GPIO_InitTypeDef  			GPIO_InitStructure;
EXTI_InitTypeDef 			EXTI_InitStruct;
NVIC_InitTypeDef 			NVIC_InitStruct;
NVIC_InitTypeDef			NVIC_InitStructure;
TIM_ICInitTypeDef  			TIM_ICInitStructure;
TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
TIM_OCInitTypeDef			TIM_OCInitStructure;
//
//#define baca			GPIO_ReadInputDataBit
//#define RED_ON 			Pin(GPIOE,ON,P_0);
//#define RED_OFF 		Pin(GPIOE,OFF,P_0);
//#define TOGGLE_RED		GPIO_ToggleBits(GPIOE,GPIO_Pin_0);
//#define GREEN_ON 		Pin(GPIOB,ON,P_3);
//#define GREEN_OFF		Pin(GPIOB,OFF,P_3);
//#define TOGGLE_GREEN	GPIO_ToggleBits(GPIOB,GPIO_Pin_3);
//#define BLUE_ON			Pin(GPIOB,ON,P_4);
//#define BLUE_OFF		Pin(GPIOB,OFF,P_4);
//#define TOGGLE_BLUE		GPIO_ToggleBits(GPIOB,GPIO_Pin_4);
//#define YELLOW_ON		Pin(GPIOB,ON,P_5);
//#define YELLOW_OFF		Pin(GPIOB,OFF,P_5);
//#define TOGGLE_YELLOW	GPIO_ToggleBits(GPIOB,GPIO_Pin_5);
////#define REDOB_ON		Pin(GPIOE,ON,P_0);
////#define REDOB_OFF		Pin(GPIOE,OFF,P_0);
////#define TOGGLE_REDOB	GPIO_ToggleBits(GPIOE,GPIO_Pin_0);
#define BUZZER_ON		Pin(GPIOA,ON,P_12);
#define BUZZER_OFF		Pin(GPIOA,OFF,P_12);
#define TOGGLE_BUZZER	GPIO_ToggleBits(GPIOA,GPIO_Pin_12);
#define PNEU1_ON		Pin(GPIOC,ON,P_1);
#define PNEU1_OFF		Pin(GPIOC,OFF,P_1);
#define PNEU2_ON		Pin(GPIOE,ON,P_4);
#define PNEU2_OFF		Pin(GPIOE,OFF,P_4);
#define PNEU3_ON		Pin(GPIOE,ON,P_2);
#define PNEU3_OFF		Pin(GPIOE,OFF,P_2);
#define PNEU4_ON		Pin(GPIOE,ON,P_3);
#define PNEU4_OFF		Pin(GPIOE,OFF,P_3);

void configuration();
////Fungsi Timer
void initTIM1(void);
void initTIM2(void);
void initTIM6(void);
void initTIM5(void);
void initTIM4(void);
void initTIM3(void);
void initTIM7(void);
void initTIM8(void);
//void initTIM9(void);
void initTIM12(void);

//fungsi encoder to rpm
int getspd1();
int getspd2();
int getspd3();
int getspd4();
int ext_enc_spd1();
int ext_enc_spd2();
int ext_enc_spd3();

void odometry();
void pneumatic();
void External_Interrupt();
void Inverse_Kinematics(float vx, float vy, float W);
void eksekusi_motor(uint16_t mtr, uint8_t dir);

//fungsi PID
uint16_t PID_Kecepatan1();
uint16_t PID_Kecepatan2();
uint16_t PID_Kecepatan3();
uint16_t PID_Kecepatan4();
//void PID_Sudut1();
//void PID_Yaw();

void stick_data(); void anatospeed();
void EXTI2_IRQHandler(); void EXTI3_IRQHandler(); void EXTI4_IRQHandler();
void EXTI9_5_IRQHandler();



typedef struct{
	volatile int 	encoder;	//--> Data encoder
	uint16_t 		enc_now;	//--> new data encoder
	uint16_t 		enc_last;	//--> old data encoder
	int16_t			delta;		//--> selisih new dan old
	volatile int 	rpm;		//--> Kecepatan putaran
	float 			sudut;		//--> Data sudut
	int 			ppr;		//--> Data pulse per rotation encoder
}Rotary;
Rotary rt_1,rt_2,rt_3,rt_4,rt_ex1,rt_ex2,rt_ex3;

typedef struct{
	float P;
	float I;
	float D;
	int MV;
	float error;
	float last1_error;
	float last2_error;
	int temp; //variabel penyimpan speed sementara
	float set;
	float Kp;
	float Ki; //konstanta d tidak dipakai
	float Kd; //konstanta i tidak dipakai
	int SP; //setpoint kecepatan motor
	int MAX;
	int MIN;
	float I_error;
	unsigned char arah;
	int status;
}PID;
PID V1;		PID V2;		PID V3;
PID V4;

typedef struct{
	uint16_t 	frequency;	//1 --> Frequency Hz (1s)
	uint32_t 	period;		//2 --> MAKS COUNT REGISTER CNT
	uint16_t	prescaler;	//3 --> Prescaler Value
	}
    TIMER;
    TIMER TIM_1;
    TIMER TIM_2;
    TIMER TIM_3;
    TIMER TIM_4;
    TIMER TIM_8;


#endif //__Invicto_H
