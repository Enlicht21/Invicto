#include "Invicto.h"

void vTaskRUN1(void *pvParameters) {
	for (;;) {
//		TOGGLE_RED
		/* General Purpose Counter */
		count1++;
		count2++;
		count3++;
		/* Read All Sensor */
//		read_sensor();

		/* Pneu Control */
		pneumatic();
		/* Heading Control */
//		PID_Yaw();
//		PID_Odometry();

		/* Move to the Next Coordinate Target */
//		odometry_track();

//		if(hold && (track=8 || track==10 || track==13) && (!rt_ext1.Rpm && !rt_ext2.Rpm && !rt_ext3.Rpm))
//			hold = 2;
//
//		/* Autonomous Movement */
//		if(run.mode){
//			if(hold==2)		Inverse_Kinematics(aksel_v,sudut_left,odo_yaw.MV+sudut_w);
//			else			Inverse_Kinematics(run.spd_tgt,run.sdt_tgt,odo_yaw.MV+sudut_w);
//		}


		/* Bluetooth Status */
		if(!btstate){
			if(test_mode)
				Inverse_Kinematics(Vx,Vy,sudut_w);
			else
				Inverse_Kinematics(0,0,0);
		}
		else{
			/* Stick Data Capture */
			stick_data();

			/* Convert Analog Data to Velocity & Degree Input */
			anatospeed();
//			anatogrip();
			/* Preparations & Input Data */
//			Preparations();

			/* Main Odometrical Movement Calculation */
//			odometrical_movement();

//			/* Heading Control */
//			PID_Yaw();
//			PID_Odometry();
//
//			/* Move to the Next Coordinate Target */
//			odometry_track();
//
//			if(hold && (track=8 || track==10 || track==13) && (!rt_ext1.Rpm && !rt_ext2.Rpm && !rt_ext3.Rpm))
//				hold = 2;

			/* Autonomous Movement */
//			if(run.mode){
//				if(hold==2)		Inverse_Kinematics(aksel_v,sudut_left,odo_yaw.MV+sudut_w);
//				else			Inverse_Kinematics(run.spd_tgt,run.sdt_tgt,odo_yaw.MV+sudut_w);
//			}

			/* Stick Controled Movement */
//			else{
				if(!mode_imu)	Inverse_Kinematics(vx,vy,ww);
//				else			Inverse_Kinematics(aksel_v,sudut_left,odo_yaw.MV+sudut_w);
//			}

		}
	}
}

void vTaskRUN2(void *pvParameters) {
	TickType_t xLastWakeTime;
	const TickType_t xPeriod = pdMS_TO_TICKS( 5 );
	for (;;) {
		xLastWakeTime = xTaskGetTickCount();
		/**********************************************************/
		/* Motor Set & Control */

//		encoder1 = TIM_GetCounter(TIM9);
//		encoder2 = TIM_GetCounter(TIM4);
//		encoder3 = TIM_GetCounter(TIM5);
//		getspd2();
//		PID_Kecepatan1();
//		PID_Kecepatan2();
//		PID_Kecepatan3();
//		PID_Kecepatan4();
////		PID_Sudut1();
		d1 = TIM2->CNT;

		eksekusi_motor(mtr4, V1.arah);
		eksekusi_motor(mtr5, V2.arah);
		eksekusi_motor(mtr6, V3.arah);
		eksekusi_motor(mtr2, V4.arah);
//		eksekusi_motor(mtr1, V5.arah);
//		eksekusi_motor(mtr2, V6.arah);

		/**********************************************************/
//		d1=(( V1.set * (TIM_8.period - 1)) / 100);
//		NOT USE PID
//		TIM8->CCR1 = (( V1.SP * (1999 - 1)) / 100); //motor1
//		TIM8->CCR3 = (( V2.SP * (1999 - 1)) / 100);	//motor2
//		TIM8->CCR2 = (( V3.SP * (1999 - 1)) / 100); //motor3
//		TIM1->CCR3 = (( V4.SP * (1999 - 1)) / 100); //motor4
//		TIM8->CCR1 = (( V3.SP * (1999 - 1)) / 100);
//		TIM1->CCR1 = (( V1.SP * (1999 - 1)) / 100); //mtr pelontar 1
//		TIM1->CCR2 = (( V2.SP * (1999 - 1)) / 100); //mtr pelontar 2


//		PID MODE
		TIM8->CCR1 = duty1; 	//motor1
		TIM8->CCR3 = duty2;		//motor2
		TIM8->CCR2 = duty3; 	//motor3
		TIM1->CCR3 = duty4;//duty4; 	//motor4
//		TIM1->CCR2 = V6.set; //mtr pelontar 2
//		TIM1->CCR3 = V4.set; //motor4 gripper

	}
}

int main(void)
{
	configuration();
//Timer for motor
	initTIM1();
	initTIM8();
//Timer for encoder
//	initTIM3();
	initTIM4();
	initTIM5();
//	initTIM9();
//	initTIM12();
//Timer for stick
	initTIM12();
//Timer for counter
	//PID Counter
	initTIM2();

	initTIM6();
	//Encoder Counter
	initTIM7();
//EXTI encoder
	External_Interrupt();

	//Encoder
	init_IO(RCC_AHB1Periph_GPIOD, P_2|P_3|P_4|P_7, GPIO_Mode_IN, GPIO_OType_PP, GPIO_PuPd_UP );
	//DIR Motor
	init_IO(RCC_AHB1Periph_GPIOB, P_10|P_11|P_12|P_13, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL );
	init_IO(RCC_AHB1Periph_GPIOE, P_15, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL );
	init_IO(RCC_AHB1Periph_GPIOD, P_8|P_9|P_10|P_11|P_12|P_13|P_14, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL );
	//Pneumatic
	init_IO(RCC_AHB1Periph_GPIOC, P_1, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_UP );
	init_IO(RCC_AHB1Periph_GPIOE, P_2|P_3|P_4, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_UP );
	//USART
//	init_USART(RCC_APB2Periph_USART1, 38400, RCC_AHB1Periph_GPIOA, P_9|P_10, TX_RX); 	 	//IMU
//	init_USART(RCC_APB1Periph_USART2, 38400, RCC_AHB1Periph_GPIOC, P_5|P_6, TX_RX);  	//Stick
	init_USART(RCC_APB1Periph_USART3, 38400, RCC_AHB1Periph_GPIOC, P_10|P_11, TX_RX); 	//IMU

	xTaskCreate( vTaskRUN1, ( signed char * ) "RUN1", configMINIMAL_STACK_SIZE, NULL, 1,( xTaskHandle * ) NULL);
	xTaskCreate( vTaskRUN2, ( signed char * ) "RUN2", configMINIMAL_STACK_SIZE, NULL, 1,( xTaskHandle * ) NULL);
//	xTaskCreate( vTaskRUN3, ( signed char * ) "RUN3", configMINIMAL_STACK_SIZE, NULL, 1,( xTaskHandle * ) NULL);
	vTaskStartScheduler();
    while(1)
    {
    }
}

//IRQ Section
	//IRQ for PID
void TIM2_IRQHandler(){
/*delta = 50ms*/
	if(TIM_GetITStatus(TIM2, TIM_IT_Update)!= RESET){
		PID_Kecepatan1();
		PID_Kecepatan2();
		PID_Kecepatan3();
		PID_Kecepatan4();
//		PID_Sudut1();

		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
	}
}
	//IRQ for encoder to rpm
void TIM7_IRQHandler(){
/*Speed(Hz)=encoder/(ppr*T*N)
 * encoder 	= number of encoder ticks
 * ppr =  encoder resolution (pulse per revolution
 * T(0.05s)	= sampling interval (disini sampling interval = waktu TIM7 Update)
 * N		= quadrature encoding mode look at tim_encoderconfig
 *
 * RPM 		= Speed*60
 * rad/s(w)	= speed*2*phi
 * m/s(v)		= (w*R)
 * */
	if(TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET ) /*Check the TIM7 update interrupt occurs or not*/
	{
		getspd1();
		getspd2();
		getspd3();
		getspd4();
		GetMechVelocity();
		ext_enc_spd1();
		ext_enc_spd2();
		ext_enc_spd3();
	TIM_ClearITPendingBit(TIM7, TIM_IT_Update); /*Remove TIMx update interrupt flag */
	rt_1.enc_last 	=0;
	rt_2.enc_last 	=0;
	rt_3.enc_last 	=0;
	rt_4.enc_last	=0;
	rt_1.encoder 	=0;
	rt_2.encoder 	=0;
	rt_3.encoder 	=0;
	rt_4.encoder	=0;
	TIM_SetCounter(TIM3,0);
	TIM_SetCounter(TIM4,0);
	TIM_SetCounter(TIM5,0);
	}
}

void TIM8_BRK_TIM12_IRQHandler(){
	if (TIM_GetITStatus(TIM12, TIM_IT_CC1)){
		TIM_ClearITPendingBit(TIM12, TIM_IT_CC1);
		capture = TIM_GetCapture1(TIM12);
		TIM_SetCompare1(TIM12, capture + CCR1_Val);

	}
	else if (TIM_GetITStatus(TIM12, TIM_IT_CC2)){
		TIM_ClearITPendingBit(TIM12, TIM_IT_CC2);
		capture = TIM_GetCapture2(TIM12);
		TIM_SetCompare2(TIM12, capture + CCR2_Val);

		btwait++;				//Perlu perbaikan
		if(btwait>50){
			btstate=btcheck;
			btcheck=0;
			btwait=0;
		}

	}
	else if (TIM_GetITStatus(TIM12, TIM_IT_CC3)){
		TIM_ClearITPendingBit(TIM12, TIM_IT_CC3);
		capture = TIM_GetCapture3(TIM12);
		TIM_SetCompare3(TIM12, capture + CCR3_Val);

	}
	else
	{
		TIM_ClearITPendingBit(TIM12, TIM_IT_CC4);
		capture = TIM_GetCapture4(TIM12);
		TIM_SetCompare4(TIM12, capture + CCR4_Val);
	}
}

//External Interrupt section
void EXTI2_IRQHandler(){
if (EXTI_GetITStatus(EXTI_Line2)){
		if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2))
			rt_1.encoder++;
		else rt_1.encoder--;


		EXTI_ClearITPendingBit(EXTI_Line2);
	}
}
void EXTI3_IRQHandler(){
if (EXTI_GetITStatus(EXTI_Line3)){
		if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_3))
			rt_4.encoder++;
		else rt_4.encoder--;

		EXTI_ClearITPendingBit(EXTI_Line3);
	}
}
void EXTI4_IRQHandler(){
if (EXTI_GetITStatus(EXTI_Line4)){
		if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_4))
			rt_3.encoder++;
		else rt_3.encoder--;



		EXTI_ClearITPendingBit(EXTI_Line4);
	}
}
void EXTI9_5_IRQHandler(){
if (EXTI_GetITStatus(EXTI_Line7)) {

	if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_7))rt_2.encoder++;
	else rt_2.encoder--;

	EXTI_ClearITPendingBit(EXTI_Line7);
 }
}


//void EXTI9_5_IRQHandler(){
//if (EXTI_GetITStatus(EXTI_Line5)){ //Max stable read 500RPM
//	  speed[5]++;
////    TOGGLE_REDOB
//
//	rt_ext1.ensud++;
//	if(rt_ext1.ensud>rt_ext1.ppr) rt_ext1.ensud=1;
//	else if(rt_ext1.ensud<0) 		rt_ext1.ensud=rt_ext1.ppr-1;
//
//	rt_ext1.sudut=(rt_ext1.ensud*360)/rt_ext1.ppr;
//    if(!rt_ext1.CaptureNumber)
//    {
//		rt_ext1.EXTI_ReadValue1 = TIM2->CNT;
//		rt_ext1.CaptureNumber = 1;
//	}
//	else if(rt_ext1.CaptureNumber)
//	{
//		rt_ext1.EXTI_ReadValue2 = TIM2->CNT;
//// Hitung Data Periode
//		if (rt_ext1.EXTI_ReadValue2 > rt_ext1.EXTI_ReadValue1)rt_ext1.Capture = (rt_ext1.EXTI_ReadValue2 - rt_ext1.EXTI_ReadValue1);
//		else if (rt_ext1.EXTI_ReadValue2 < rt_ext1.EXTI_ReadValue1)rt_ext1.Capture = ((TIM2 -> ARR - rt_ext1.EXTI_ReadValue1) + rt_ext1.EXTI_ReadValue2);
//		else rt_ext1.Capture = 0;
//// Hitung Frekuensi
//		rt_ext1.Freq = TIM_2.period / rt_ext1.Capture;
//		rt_ext1.Rpm = (rt_ext1.Freq*60)/rt_ext1.ppr;
//		rt_ext1.CaptureNumber = 0;
//	}
//
//    if(rt_ext1.Rpm<maxrpm){
//    	if(!GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_7)){
//    		rt_ext1.last_dir = inc;
//
//    		rt_ext1.encoder++;
//    		theta1=((yaw-run.setyaw)+sdt_ext1)*pi/180.0;
//
//    		coX.real += sinf(theta1);
//    		coY.real += cosf(theta1);
//    	}
//    	else{
//    		rt_ext1.last_dir = dec;
//
//    		rt_ext1.encoder--;
//    		theta1=((yaw-run.setyaw)+sdt_ext1)*pi/180.0;
//
//    		coX.real -= sinf(theta1);
//    		coY.real -= cosf(theta1);
//    	}
//    }
//    else if(rt_ext1.last_dir == inc){
//		rt_ext1.encoder++;
//		theta1=((yaw-run.setyaw)+sdt_ext1)*pi/180.0;
//
//		coX.real += sinf(theta1);
//		coY.real += cosf(theta1);
//    }
//    else if(rt_ext1.last_dir == dec){
//		rt_ext1.encoder--;
//		theta1=((yaw-run.setyaw)+sdt_ext1)*pi/180.0;
//
//		coX.real -= sinf(theta1);
//		coY.real -= cosf(theta1);
//    }
//
//	EXTI_ClearITPendingBit(EXTI_Line5);
// }
//
//if (EXTI_GetITStatus(EXTI_Line6)){ //Max stable read 500RPM
//	speed[6]++;
////    TOGGLE_REDOB
//
//	rt_ext2.ensud++;
//	if(rt_ext2.ensud>rt_ext2.ppr) rt_ext2.ensud=1;
//	else if(rt_ext2.ensud<0) 		rt_ext2.ensud=rt_ext2.ppr-1;
//
//	rt_ext2.sudut=(rt_ext2.ensud*360)/rt_ext2.ppr;
//	if(!rt_ext2.CaptureNumber)
//	{
//		rt_ext2.EXTI_ReadValue1 = TIM2->CNT;
//		rt_ext2.CaptureNumber = 1;
//	}
//	else if(rt_ext2.CaptureNumber)
//	{
//		rt_ext2.EXTI_ReadValue2 = TIM2->CNT;
//// Hitung Data Periode
//		if (rt_ext2.EXTI_ReadValue2 > rt_ext2.EXTI_ReadValue1)rt_ext2.Capture = (rt_ext2.EXTI_ReadValue2 - rt_ext2.EXTI_ReadValue1);
//		else if (rt_ext2.EXTI_ReadValue2 < rt_ext2.EXTI_ReadValue1)rt_ext2.Capture = ((TIM2 -> ARR - rt_ext2.EXTI_ReadValue1) + rt_ext2.EXTI_ReadValue2);
//		else rt_ext2.Capture = 0;
//// Hitung Frekuensi
//		rt_ext2.Freq = TIM_2.period / rt_ext2.Capture;
//		rt_ext2.Rpm = (rt_ext2.Freq*60)/rt_ext2.ppr;
//		rt_ext2.CaptureNumber = 0;
//	}
//
//    if(rt_ext2.Rpm<maxrpm){
//    	if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3)){
//    		rt_ext2.last_dir = inc;
//
//    		rt_ext2.encoder++;
//    		theta2=((yaw-run.setyaw)+sdt_ext2)*pi/180.0;
//
//    		coX.real += sinf(theta2);
//    		coY.real += cosf(theta2);
//    	}
//    	else{
//    		rt_ext2.last_dir = dec;
//
//    		rt_ext2.encoder--;
//    		theta2=((yaw-run.setyaw)+sdt_ext2)*pi/180.0;
//
//    		coX.real -= sinf(theta2);
//    		coY.real -= cosf(theta2);
//    	}
//    }
//    else if(rt_ext2.last_dir == inc){
//		rt_ext2.encoder++;
//		theta2=((yaw-run.setyaw)+sdt_ext2)*pi/180.0;
//
//		coX.real += sinf(theta2);
//		coY.real += cosf(theta2);
//    }
//    else if(rt_ext2.last_dir == dec){
//		rt_ext2.encoder--;
//		theta2=((yaw-run.setyaw)+sdt_ext2)*pi/180.0;
//
//		coX.real -= sinf(theta2);
//		coY.real -= cosf(theta2);
//    }
//	EXTI_ClearITPendingBit(EXTI_Line6);
// }
//
//if (EXTI_GetITStatus(EXTI_Line7)) { //Max stable read 500RPM
//	speed[7]++;
////    TOGGLE_REDOB
//
//	rt_ext3.ensud++;
//	if(rt_ext3.ensud>rt_ext3.ppr) rt_ext3.ensud=1;
//	else if(rt_ext3.ensud<0) 		rt_ext3.ensud=rt_ext3.ppr-1;
//
//	rt_ext3.sudut=(rt_ext3.ensud*360)/rt_ext3.ppr;
//	if(!rt_ext3.CaptureNumber)
//	{
//		rt_ext3.EXTI_ReadValue1 = TIM2->CNT;
//		rt_ext3.CaptureNumber = 1;
//	}
//	else if(rt_ext3.CaptureNumber)
//	{
//		rt_ext3.EXTI_ReadValue2 = TIM2->CNT;
//	// Hitung Data Periode
//	if (rt_ext3.EXTI_ReadValue2 > rt_ext3.EXTI_ReadValue1)rt_ext3.Capture = (rt_ext3.EXTI_ReadValue2 - rt_ext3.EXTI_ReadValue1);
//	else if (rt_ext3.EXTI_ReadValue2 < rt_ext3.EXTI_ReadValue1)rt_ext3.Capture = ((TIM2 -> ARR - rt_ext3.EXTI_ReadValue1) + rt_ext3.EXTI_ReadValue2);
//	else rt_ext3.Capture = 0;
//	// Hitung Frekuensi
//	rt_ext3.Freq = TIM_2.period / rt_ext3.Capture;
//	rt_ext3.Rpm = (rt_ext3.Freq*60)/rt_ext3.ppr;
//	rt_ext3.CaptureNumber = 0;
//	}
//
//    if(rt_ext3.Rpm<maxrpm){
//    	if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4)){
//    		rt_ext3.last_dir = inc;
//
//    		rt_ext3.encoder++;
//    		theta3=((yaw-run.setyaw)+sdt_ext3)*pi/180.0;
//
//    		coX.real += sinf(theta3);
//    		coY.real += cosf(theta3);
//    	}
//    	else{
//    		rt_ext3.last_dir = dec;
//
//    		rt_ext3.encoder--;
//    		theta3=((yaw-run.setyaw)+sdt_ext3)*pi/180.0;
//
//    		coX.real -= sinf(theta3);
//    		coY.real -= cosf(theta3);
//    	}
//    }
//    else if(rt_ext3.last_dir == inc){
//		rt_ext3.encoder++;
//		theta3=((yaw-run.setyaw)+sdt_ext3)*pi/180.0;
//
//		coX.real += sinf(theta3);
//		coY.real += cosf(theta3);
//    }
//    else if(rt_ext3.last_dir == dec){
//		rt_ext3.encoder--;
//		theta3=((yaw-run.setyaw)+sdt_ext3)*pi/180.0;
//
//		coX.real -= sinf(theta3);
//		coY.real -= cosf(theta3);
//    }
//	EXTI_ClearITPendingBit(EXTI_Line7);
// }
//}

void USART3_IRQHandler(void)
{
	if( USART_GetITStatus(USART3, USART_IT_RXNE) ){
//		TOGGLE_REDOB

		char bit=USART3->DR; // the character from the USART2 data register is saved in bitt
		char static data_terakhir;

		switch(bit_ke)
		{
			case 0 : rxdata[bit_ke]=USART3->DR; bit_ke++; break;
			case 1 : rxdata[bit_ke]=USART3->DR; bit_ke++; break;
			case 2 : rxdata[bit_ke]=USART3->DR; bit_ke++; break;
			case 3 : rxdata[bit_ke]=USART3->DR; bit_ke++; break;
			case 4 : rxdata[bit_ke]=USART3->DR; bit_ke++; break;
			case 5 : rxdata[bit_ke]=USART3->DR; bit_ke++; break;
			case 6 : rxdata[bit_ke]=USART3->DR; bit_ke++; break;
			case 7 : rxdata[bit_ke]=USART3->DR; bit_ke++; break;
			case 8 : rxdata[bit_ke]=USART3->DR; bit_ke=100; break;
		}
		if(bit==0b10101010&&data_terakhir==0b01010101){
			bit_ke=0;
//			REDOB_ON
		}
		else
		data_terakhir=bit;

		if((rxdata[2]==255 && rxdata[3]==255 && rxdata[4]==255 && rxdata[5]==255)
			|| (rxdata[3]==128 && rxdata[5]==128))
			btcheck=0;
		else btcheck=1;
	}
}
