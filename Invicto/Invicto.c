#include "Invicto.h"

PID V1;
PID V2;
PID V3;
PID V4;

TIMER TIM_1 = {0, 2799,11};

void configuration(){
	V1.MAX 	= TIM_1.period;
	V2.MAX	= TIM_1.period;
	V3.MAX	= TIM_1.period;
	V4.MAX	= TIM_1.period;

//	if ((fabsf(Vx)||fabsf(Vy))==0.7){
//
//	}
//	else{
		V1.Kp	= 0.6;
		V1.Ki	= 3.8;

		V2.Kp	= 0.6;
		V2.Ki	= 3.8;

		V3.Kp	= 0.6;
		V3.Ki	= 3.8;

		V4.Kp	= 0.6;
		V4.Ki	= 3.8;
//	}
}

//TIMER Section
	//TIMER for Counter
void initTIM2(){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseStructure.TIM_Prescaler = 69;//6;
	TIM_TimeBaseStructure.TIM_Period = 59999;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
//	TIM_SetCounter(TIM2,0);
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);

	TIM_Cmd(TIM2, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_EnableIRQ(TIM2_IRQn);
}
void initTIM6(){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

	TIM_TimeBaseStructure.TIM_Prescaler = 1343;//6;
	TIM_TimeBaseStructure.TIM_Period = 62499;//59999;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);

	TIM_Cmd(TIM6, ENABLE);
}

void initTIM7(){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

	TIM_TimeBaseStructure.TIM_Prescaler = 69;
	TIM_TimeBaseStructure.TIM_Period = 59999;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV2;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);

	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);

	TIM_Cmd(TIM7, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_EnableIRQ(TIM7_IRQn);
}

//TIMER for Encoder
void initTIM3(void){
//init Pin Encoder
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOB, &GPIO_InitStruct);

GPIO_PinAFConfig(GPIOB,GPIO_PinSource4, GPIO_AF_TIM3);
GPIO_PinAFConfig(GPIOB,GPIO_PinSource5, GPIO_AF_TIM3);

RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI1,
    								TIM_ICPolarity_Rising,
    								TIM_ICPolarity_Rising);
//TIM_SetAutoreload(TIM5,700);
TIM_SetCounter(TIM3,0);
TIM_Cmd(TIM3, ENABLE);
}

void initTIM4(void){
//init Pin Encoder
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOB, &GPIO_InitStruct);

GPIO_PinAFConfig(GPIOB,GPIO_PinSource6, GPIO_AF_TIM4);
GPIO_PinAFConfig(GPIOB,GPIO_PinSource7, GPIO_AF_TIM4);

RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI1,
    								TIM_ICPolarity_Rising,
    								TIM_ICPolarity_Rising);
//TIM_SetAutoreload(TIM4,704);
TIM_SetCounter(TIM4,0);
TIM_Cmd(TIM4, ENABLE);
}

void initTIM5(void){
//init Pin Encoder
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOA, &GPIO_InitStruct);

GPIO_PinAFConfig(GPIOA,GPIO_PinSource0, GPIO_AF_TIM5);
GPIO_PinAFConfig(GPIOA,GPIO_PinSource1, GPIO_AF_TIM5);

RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI1,
    								TIM_ICPolarity_Rising,
    								TIM_ICPolarity_Rising);
//TIM_SetAutoreload(TIM5,700);
TIM_SetCounter(TIM5,0);
RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

TIM5->CR2 = 0x030;                                                      //MMS = 101

TIM3->PSC = 0x03;
//TIM2->CR2 |= TIM_CR2_TI1S;
TIM3->SMCR = 0x24;                                                      //TS = 010 for ITR2, SMS = 100 (reset counter at edge)
TIM3->CCMR1 = 0x3;                                                      // CC1S = 11, IC1 mapped on TRC

//TIM2->CR2 |= TIM_CR2_TI1S;
TIM3->CCER |= TIM_CCER_CC1P;
//TIM2->CCER |= TIM_CCER_CC1NP;
TIM3->CCER |= TIM_CCER_CC1E;
TIM_Cmd(TIM3, ENABLE);
TIM_Cmd(TIM5, ENABLE);
}

float GetMechVelocity(){

    float out = 0;
    float rawPeriod = TIM3->CCR1; //Clock Ticks
    int currentTime = TIM3->CNT;
    if(currentTime > 2000000){rawPeriod = currentTime;}
    float  dir = -2.0f*(float)(((TIM5->CR1)>>4)&1)+1.0f;    // +/- 1
    float meas = dir*168000000.0f*(6.283185/100)/rawPeriod;
    if(isinf(meas)){ meas = 1;}
    out = meas;
    //if(meas == oldVel){
     //   out = .9f*out_old;
     //   }


    oldVel = meas;
    out_old = out;
    int n = 16;
    float sum = out;
    for (int i = 1; i < (n); i++){
        velVec[n - i] = velVec[n-i-1];
        sum += velVec[n-i];
        }
    velVec[0] = out;
    velo = sum/(float)n;
    return sum/(float)n;
    }


//void initTIM9(void){
////init Pin Encoder
//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
//
//GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
//GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
//GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
//GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
//GPIO_Init(GPIOE, &GPIO_InitStruct);
//
//GPIO_PinAFConfig(GPIOE,GPIO_PinSource5, GPIO_AF_TIM9);
//GPIO_PinAFConfig(GPIOE,GPIO_PinSource6, GPIO_AF_TIM9);
//
//RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
//
//TIM_EncoderInterfaceConfig(TIM9, TIM_EncoderMode_TI12,
//    								TIM_ICPolarity_Rising,
//    								TIM_ICPolarity_Rising);
////TIM_SetAutoreload(TIM9,700);
//TIM_SetCounter(TIM9,0);
//TIM_Cmd(TIM9, ENABLE);
//}
//void initTIM12(){
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
//
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
//	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
//	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
//	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//	GPIO_PinAFConfig(GPIOB,GPIO_PinSource14, GPIO_AF_TIM12);
//	GPIO_PinAFConfig(GPIOB,GPIO_PinSource15, GPIO_AF_TIM12);
//
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);
//
//	TIM_EncoderInterfaceConfig(TIM12, TIM_EncoderMode_TI12,
//	    								TIM_ICPolarity_Rising,
//	    								TIM_ICPolarity_Rising);
//	//TIM_SetAutoreload(TIM9,700);
//	TIM_SetCounter(TIM12,0);
//	TIM_Cmd(TIM12, ENABLE);
//}

//TIMER for Stick
void initTIM12(){

	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);

	  /* Time base configuration */
	  TIM_TimeBaseStructure.TIM_Period = 50000;
	  TIM_TimeBaseStructure.TIM_Prescaler = 0;
	  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	  TIM_TimeBaseInit(TIM12, &TIM_TimeBaseStructure);

	  /* Output Compare Timing Mode configuration: Channel1 */
	  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
	  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	  TIM_OC1Init(TIM12, &TIM_OCInitStructure);

	  TIM_OC1PreloadConfig(TIM12, TIM_OCPreload_Disable);

	  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	  TIM_OCInitStructure.TIM_Pulse = CCR2_Val;
	  TIM_OC2Init(TIM12, &TIM_OCInitStructure);

	  TIM_OC2PreloadConfig(TIM12, TIM_OCPreload_Disable);

	  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	  TIM_OCInitStructure.TIM_Pulse = CCR3_Val;
	  TIM_OC3Init(TIM12, &TIM_OCInitStructure);

	  TIM_OC3PreloadConfig(TIM12, TIM_OCPreload_Disable);

	  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	  TIM_OCInitStructure.TIM_Pulse = CCR4_Val;
	  TIM_OC4Init(TIM12, &TIM_OCInitStructure);

	  TIM_OC4PreloadConfig(TIM12, TIM_OCPreload_Disable);

	  /* TIM Interrupts enable */
	  TIM_ITConfig(TIM12, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3, ENABLE);
	  TIM_ITConfig(TIM12, TIM_IT_CC4, DISABLE);

	  /* TIM12 enable counter */
	  TIM_Cmd(TIM12, ENABLE);

	  NVIC_InitStructure.NVIC_IRQChannel = TIM8_BRK_TIM12_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
}


	//TIMER for PWM
void initTIM1(void){
//	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);//MOTOR 3
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);

	TIM_TimeBaseStructure.TIM_Prescaler = TIM_1.prescaler;//1;//5;//83; 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = TIM_1.period;//1999;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

//	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
//	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
//	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
//	TIM_BDTRInitStructure.TIM_DeadTime = 11;
//	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
//	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
//	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
//	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

//	TIM_ARRPreloadConfig(TIM8, ENABLE);

	TIM_Cmd(TIM1, ENABLE);

	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}
void initTIM8(void){
//	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);//MOTOR 3
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);//MOTOR 5
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);//MOTOR 4


	TIM_TimeBaseStructure.TIM_Prescaler = TIM_1.prescaler;//5;//83;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = TIM_1.period;//1999;33599
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

//	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
//	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
//	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
//	TIM_BDTRInitStructure.TIM_DeadTime = 11;
//	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
//	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
//	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
//	TIM_BDTRConfig(TIM8, &TIM_BDTRInitStructure);

	TIM_OC1Init(TIM8, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);

	TIM_OC2Init(TIM8, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);

	TIM_OC3Init(TIM8, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);

//	TIM_ARRPreloadConfig(TIM8, ENABLE);

	TIM_Cmd(TIM8, ENABLE);

	TIM_CtrlPWMOutputs(TIM8, ENABLE);
}

/*
 * no motor pergerakan:
 * 				encoder2
 * 				O
 * 				O
 * 				O
 * 			O		O
 *		O				O
 * encoder1				encoder3*/

void odometry(){
	//radius rotary external wheel

	float v_enc1 = ex_rpm1 * R;
	float v_enc2 = ex_rpm2 * R;
	float v_enc3 = ex_rpm3 * R;
	TIM_SetCounter(TIM2,0);
	timecurrent = TIM2->CNT;
	time_taken = ((float)((timecurrent - timeprev)/62500));
	timeprev = timecurrent;
	float vxm = ((2*v_enc2)/3.0)-(v_enc1/3.0)-(v_enc3/3.0);
	float vym = (0.577*v_enc1)-(0.577*v_enc3);
	float omega = (v_enc1+v_enc2+v_enc3)/(3*L);
	pose_x 		= (vxm*time_taken)	+ pose_x;
	pose_y 		= (vym*time_taken)	+ pose_y;
	pose_theta	= (omega*time_taken)+ pose_theta;

//run.dist = sqrt(pow((coXmm - run.setx), 2) + pow((coYmm - run.sety), 2));// misal x^y = pow (x,y)
//
////Target Sudut
//run.sdt_tgt = atan2f((coXmm - run.setx), (coYmm - run.sety));
//run.sdt_tgt *= 57.295779; //konversi dari sudut ke radian
//run.sdt_tgt -= (yaw - run.setyaw);
//if (run.sdt_tgt < 0) run.sdt_tgt = 360 + run.sdt_tgt;
}

//define max rpm rata2 1200
int getspd1(){
	//	PPR 	= 135
	//	N		= 2
	//	T		= 0.05
	//wmax(m1) 	= 1234		max angular velocity
	rt_1.delta = (int16_t)(rt_1.encoder-rt_1.enc_last);
	rt_1.enc_last = rt_1.encoder;

	rt_1.rpm = 4.44*rt_1.delta;

	return rt_1.rpm;
}

int getspd2(){
//	PPR = 135
//	N	= 2
//	T	= 0.05
//	1460
	rt_2.delta = (int16_t)(rt_2.encoder-rt_2.enc_last);
	rt_2.enc_last = rt_2.encoder;

	rt_2.rpm = 4.44*rt_2.delta;

	return rt_2.rpm;
}

int getspd3(){
	//	PPR = 135
	//	N	= 2
	//	T	= 0.05
	//1212
	rt_3.delta = (int16_t)(rt_3.encoder-rt_3.enc_last);
	rt_3.enc_last = rt_3.encoder;

	rt_3.rpm = 4.44*rt_3.delta;

	return rt_3.rpm;
}

int getspd4(){
	//	PPR = 135
	//	N	= 2
	//	T	= 0.05
	//1411
	rt_4.delta = (int16_t)(rt_4.encoder-rt_4.enc_last);
	rt_4.enc_last = rt_4.encoder;

	rt_4.rpm = 4.44*rt_4.delta;

	return rt_4.rpm;
}

int ext_enc_spd1(){
//	PPR = 4000
//	N	= 2
//	T	= 0.05
	rt_ex1.enc_now = TIM_GetCounter(TIM3);
	rt_ex1.delta = (int16_t)(rt_ex1.enc_now-rt_ex1.enc_last);
	rt_ex1.enc_last = rt_ex1.enc_now;

	rt_ex1.rpm = 4.44*rt_ex1.delta;

	return rt_ex1.rpm;
}

int ext_enc_spd2(){
//	PPR = 4000
//	N	= 2
//	T	= 0.05
	rt_ex2.enc_now = TIM_GetCounter(TIM4);
	rt_ex2.delta = (int16_t)(rt_ex2.enc_now-rt_ex2.enc_last);
	rt_ex2.enc_last = rt_ex2.enc_now;

	rt_ex2.rpm = 4.44*rt_ex2.delta;

	return rt_ex2.rpm;
}

int ext_enc_spd3(){
//	PPR = 4000
//	N	= 2
//	T	= 0.05
	rt_ex3.enc_now = TIM_GetCounter(TIM5);
	rt_ex3.delta = (int16_t)(rt_ex3.enc_now-rt_ex3.enc_last);
	rt_ex3.enc_last = rt_ex3.enc_now;

	rt_ex3.rpm = 4.44*rt_ex3.delta;

	return rt_ex3.rpm;
}

void pneumatic(){
	if(!L2 && R2 && !A)
		pneumod=20;		//Pendek
	else if(L2 && !A)				//Perpanjangan
		pneumod=21;		//Panjang
	else if(R2 && !L2 && !O)		//Gripper Shagai
		pneumod=30;		//Buka
	else if(L2 && !O){
		pneumod = 31;
//pneumod = 11;
//		if(count3>600){
//		shagai = 1;
//		count3 = 0;
		}
//	}
//	else if(!left){
//		pneumod = 21; //Grip
//		shagai = 0; //Angkat Shagai
//		hold = 0;
//		track = 11;
//	}		//Ungrip
//	else if(L2 && !R2 && !right){
//		pneumod = 30;
//	}
//	else if(!right){
//		pneumod = 31;
//	}
//	else if(L2 && !R2 && !down)
//		pneumod=30;		//Reload
//	else if(!down)				//Lontar
//		pneumod=31;		//Shoot
//	else 				//ALL OFF
//		pneumod=0;

	switch(pneumod){
	case 0: PNEU1_OFF PNEU2_OFF
			PNEU3_OFF  break;			//All OFF
	case 10: PNEU1_OFF break;			//Extender Retract
	case 11: PNEU1_ON break;			//Extender ON
	case 20: PNEU2_OFF break;			//Gripper naik
	case 21: PNEU2_ON break;			//Gripper turun
	case 30: PNEU3_OFF break;			//ungrip
	case 31: PNEU3_ON break;			//grip
	case 40: PNEU4_OFF break;			//ungrip
	case 41: PNEU4_ON break;			//grip
	case 99: PNEU1_ON PNEU2_ON PNEU3_ON  break;	//Extender ON & Grip Shagai
	}
}
void eksekusi_motor(uint16_t mtr, uint8_t dir){

	switch(mtr){
	case mtr1:{	switch(dir){
				case maju	:{	Pin(GPIOB,OFF,P_10);
								Pin(GPIOE,ON,P_15);
								} break;

				case mundur	:{	Pin(GPIOB,ON,P_10);
								Pin(GPIOE,OFF,P_15);
								} break;
				case stop	:{	Pin(GPIOB,ON,P_10);
								Pin(GPIOE,ON,P_15);
								} break;}
			  } break;
	case mtr2:{	switch(dir){
				case maju	:{	Pin(GPIOB,OFF,P_11);
								Pin(GPIOB,ON,P_12);
								} break;
				case mundur	:{	Pin(GPIOB,ON,P_11);
								Pin(GPIOB,OFF,P_12);
								} break;
				case stop	:{	Pin(GPIOB,ON,P_11);
								Pin(GPIOB,ON,P_12);
								} break;}
			  } break;
	case mtr3:{	switch(dir){
				case maju	:{	Pin(GPIOB,OFF,P_13);
								Pin(GPIOD,ON,P_8);
								} break;
				case mundur	:{	Pin(GPIOB,ON,P_13);
								Pin(GPIOD,OFF,P_8);
								} break;
				case stop	:{	Pin(GPIOB,ON,P_13);
								Pin(GPIOD,ON,P_8);
								} break;}
			  } break;
	case mtr4:{	switch(dir){
				case maju	:{	Pin(GPIOD,ON,P_9);
								Pin(GPIOD,OFF,P_10);
								} break;
				case mundur	:{	Pin(GPIOD,OFF,P_9);
								Pin(GPIOD,ON,P_10);
								} break;
				case stop	:{	Pin(GPIOD,ON,P_9);
								Pin(GPIOD,ON,P_10);
								} break;}
			  } break;
	case mtr5:{	switch(dir){
				case maju	:{	Pin(GPIOD,OFF,P_11);
								Pin(GPIOD,ON,P_12);
								} break;
				case mundur	:{	Pin(GPIOD,ON,P_11);
								Pin(GPIOD,OFF,P_12);
								} break;
				case stop	:{	Pin(GPIOD,ON,P_11);
								Pin(GPIOD,ON,P_12);
								} break;}
			  } break;
	case mtr6:{	switch(dir){
				case mundur	:{	Pin(GPIOD,OFF,P_13);
								Pin(GPIOD,ON,P_14);
								} break;
				case maju	:{	Pin(GPIOD,ON,P_13);
								Pin(GPIOD,OFF,P_14);
								} break;
				case stop	:{	Pin(GPIOD,ON,P_13);
								Pin(GPIOD,ON,P_14);
								} break;}
			  } break;
	}
}

/*	O(M2/PWM5)				O(M1/PWM4)
			-		-
				-
			-		-
	O(M3/PWM6)				O(M4/PWM2)
*/
void Inverse_Kinematics(float VX, float VY, float W){
	//Set Point RPM
//	sudut = (sudut/180)*pi;
//	V2.SP = -(speed*(-1)*sin(sudut + ((2*pi)/3))+W);
//	V1.SP = -(speed*(-1)*sin(sudut - ((2*pi)/3))+W);
//	V3.SP = -(speed*(-1)*sin(sudut)+W);

	V1.SP = 1210*(((-1)*(0.707)*VX)+((0.707)*VY)+W);
	V2.SP = 1210*(((-1)*(0.707)*VX)+((-1)*(0.707)*VY)+W);
	V3.SP = 1210*(((0.707)*VX)+((-1)*(0.707)*VY)+W);
	V4.SP = 1210*(((0.707)*VX)+((0.707)*VY)+W);



	if (V1.SP<0)	{ 	V1.SP = -V1.SP; 	V1.arah = mundur; 	}
	else if(V1.SP>0){ 	V1.SP = V1.SP; 		V1.arah = maju;		}
	else 			{ 	V1.SP = 0; 			V1.arah	= stop;		}

	if (V2.SP<0)	{	V2.SP = -V2.SP; 	V2.arah = mundur;	}
	else if(V2.SP>0){	V2.SP = V2.SP; 		V2.arah = maju;		}
	else			{	V2.SP = 0; 			V2.arah = stop;		}

	if (V3.SP<0)	{	V3.SP = -V3.SP; 	V3.arah = mundur;	}
	else if(V3.SP>0){	V3.SP = V3.SP; 		V3.arah = maju;		}
	else 			{	V3.SP = 0; 			V3.arah = stop;		}

	if (V4.SP<0)	{	V4.SP = -V4.SP; 	V4.arah = mundur;	}
	else if(V4.SP>0){	V4.SP = V4.SP; 		V4.arah = maju;		}
	else 			{	V4.SP = 0; 			V4.arah = stop;		}

//	if(V1.SP<3)V1.SP=0;
//	if(V2.SP<3)V2.SP=0;
//	if(V3.SP<3)V3.SP=0;
}

uint16_t PID_Kecepatan1(){
	V1.error 	= abs(V1.SP) - abs(rt_1.rpm);
//	deadband1 	= abs(V1.SP*(2/100));
//	if(V1.error<deadband1){V1.error = 0;}
	V1.P 		= (float)(V1.Kp * V1.error);//(V1.error - V1.last1_error);
	V1.I_error	= (V1.I_error + V1.error);
	//Anti-windup
//	if (V1.I_error <1200 && V1.I_error>-1200) V1.I_error = V1.I_error;
//	else if		(V1.I_error > 1200)V1.I_error = 1200;
//	else if (V1.I_error < -1200)V1.I_error = -1200;

	V1.I 		= V1.Ki * 0.05 * V1.I_error; //delta = 0.05 (perlu perbaikan penetuan delta)
//	V1.D 		= (V1.Kd * (V1.error - 2*V1.last1_error+V1.last2_error));
	V1.MV 		= V1.P + V1.I ;//+ V1.D;

	if(abs(V1.MV)>V1.MAX)V1.MV = V1.MAX;
	duty1=abs(V1.MV);

	return duty1;
}

uint16_t PID_Kecepatan2(){
	V2.error 	= abs(V2.SP) - abs(rt_2.rpm);
//	deadband2 	= abs(V2.SP*(2/100));
//	if(V2.error<deadband2){V2.error = 0;}
	V2.P 		= (float)(V2.Kp * V2.error);//(V2.error - V2.last1_error);
	V2.I_error	= (V2.I_error + V2.error);
	//Anti-windup
//	if (V2.I_error <1200 && V2.I_error>-1200) V2.I_error = V2.I_error;
//	else if		(V2.I_error > 1200)V2.I_error = 1200;
//	else if (V2.I_error < -1200)V2.I_error = -1200;

	V2.I 		= V2.Ki * 0.05 * V2.I_error;//delta = 0.05 (perlu perbaikan penetuan delta)
//	V2.D 		= (V2.Kd * (V2.error - 2*V2.last1_error+V2.last2_error));
	V2.MV 		= V2.P + V2.I ;//+ V2.D;

	if(abs(V2.MV)>V2.MAX)V2.MV = V2.MAX;
	duty2=abs(V2.MV);

	return duty2;
}

uint16_t PID_Kecepatan3(){
	V3.error 	= abs(V3.SP) - abs(rt_3.rpm);
//	deadband3 	= abs(V3.SP*(2/100));
//	if(V3.error<deadband3){V3.error = 0;}
	V3.P 		= (float)(V3.Kp * V3.error);//(V3.error - V3.last1_error);
	V3.I_error	= (V3.I_error + V3.error);
	//Anti-windup
//	if (V3.I_error <1200 && V3.I_error>-1200) V3.I_error = V3.I_error;
//	else if		(V3.I_error > 1200)V3.I_error = 1200;
//	else if (V3.I_error < -1200)V3.I_error = -1200;

	V3.I 		= V3.Ki * 0.05 * V3.I_error;//delta = 0.05 (perlu perbaikan penetuan delta)
//	V3.D 		= (V3.Kd * (V3.error - 2*V3.last1_error+V3.last2_error));
	V3.MV 		= V3.P + V3.I ;//+ V3.D;

	if(abs(V3.MV)>V3.MAX)V3.MV = V3.MAX;
	duty3=abs(V3.MV);

	return duty3;
}

uint16_t PID_Kecepatan4(){
	V4.error 	= abs(V4.SP) - abs(rt_4.rpm);
//	deadband4 	= V4.SP*(2/100);
//	if(V4.error<deadband4){V4.error = 0;}
	V4.P 		= (float)(V4.Kp * V4.error);//(V4.error - V4.last1_error);
	V4.I_error	= (V4.I_error + V4.error);
	//Anti-windup
//	if (V4.I_error <2700 && V4.I_error>-2700) V4.I_error = V4.I_error;
//	else if		(V4.I_error > TIM_1.period)V4.I_error = 2700;
//	else if (V4.I_error < -2700)V4.I_error = -2700;

	V4.I 		= V4.Ki * 0.05 * V4.I_error;//delta = 0.05 (perlu perbaikan penetuan delta)
//	V4.D 		= (V4.Kd * (V4.error - 2*V4.last1_error+V4.last2_error));
	V4.MV 		= V4.P + V4.I ;//+ V4.D;

	if(abs(V4.MV)>V4.MAX)V4.MV = V4.MAX;
	duty4=abs(V4.MV);

	return duty4;
}

//Joystick Section
void stick_data(){
	slect=(rxdata[0])&0x01;
	L3=(rxdata[0])&0x02;
	R3=(rxdata[0])&0x4;
	start=(rxdata[0])&0x08;
	up=(rxdata[0])&0x10;
	right=(rxdata[0])&0x20;
	down=(rxdata[0])&0x40;
	left=(rxdata[0])&0x80;

	L2=(rxdata[1])&0x01;
	R2=(rxdata[1])&0x02;
	L1=(rxdata[1])&0x04;
	R1=(rxdata[1])&0x08;
	A=(rxdata[1])&0x10;
	O=(rxdata[1])&0x20;
	X=(rxdata[1])&0x40;
	Q=(rxdata[1])&0x80;
	anarix=(rxdata[2])-128;
	anariy=-(rxdata[3]-127);
	analex=(rxdata[4])-128;
	analey=-(rxdata[5]-127);
}
void anatospeed()
{
	/* Left Stick */
	//Default Depan 0 Belakang 180 Kanan 90 Kiri 270
	sudut_left = atan2f(anarix, anariy);
	sudut_left *= 57.295779;	//Rad to Deg
	if (sudut_left < 0) sudut_left = 360 + sudut_left;

	//Depan 180 Belakang 0 Kanan 270 Kiri 90
	if(!mode_stick){
		if(sudut_left<=180)
			sudut_left += 180;
		else
			sudut_left -= 180;
	}

	if(!L1 && R1 && L2 && R2)			ww=0.2;
	else if(!R1 && L1 && L2 && R2)		ww=-0.2;
	else 								ww=0;

	if(!up){
		vx = 0;
		vy = 1;
	}
	else if(!left){
		vx = 1;
		vy = 0;
	}
	else if(!right){
		vx = -1;
		vy = 0;
	}
	else if(!down){
		vx = 0;
		vy = -1;
	}
	else if(up && left && right && down){
		vx=0;
		vy=0;
	}
//	if(anarix||anariy){
//		sudut_left = sudut_left;
//		if(sudut_left==360){
//			vx = 0;
//			vy = 1;
//		}
//		else if(sudut_left>0&&sudut_left<90){
//			vx = -0.7;
//			vy = 0.7;
//		}
//		else if(sudut_left==90){
//			vx = -1;
//			vy = 0;
//		}
//		else if(sudut_left>90&&sudut_left<180){
//			vx = -0.7;
//			vy = -0.7;
//		}
//		else if(sudut_left==180){
//			vx = 0;
//			vy = -1;
//		}
//		else if(sudut_left>180&&sudut_left<270){
//			vx = 0.7;
//			vy = -0.7;
//		}
//		else if(sudut_left==270){
//			vx = 1;
//			vy = 0;
//		}
//		else if(sudut_left>270&&sudut_left<360){
//			vx = 0.7;
//			vy = 0.7;
//		}
//	}
//			else{
//			vx = 0;
//			vy = 0;}


}


////Interrupt Section
void External_Interrupt(){
	//Pin Config
    init_IO(RCC_AHB1Periph_GPIOA, P_2|P_3|P_4|P_7, GPIO_Mode_IN, GPIO_OType_PP, GPIO_PuPd_UP );

    //Exti Init
    EXTI_InitStruct.EXTI_Line = EXTI_Line2|EXTI_Line3|EXTI_Line4|EXTI_Line7;
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_Init(&EXTI_InitStruct);

    //NVIC Config

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource2);
    NVIC_InitStruct.NVIC_IRQChannel = EXTI2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x02;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource3);
    NVIC_InitStruct.NVIC_IRQChannel = EXTI3_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource4);
    NVIC_InitStruct.NVIC_IRQChannel = EXTI4_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x02;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource7);
    NVIC_InitStruct.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x02;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x02;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}
