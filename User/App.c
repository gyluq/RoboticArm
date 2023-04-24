#include "include.h"

#define ADC_BAT		13		//电池电压的AD检测通道
// static bool UartBusy = FALSE;

u32 gSystemTickCount = 0;	//系统从启动到现在的毫秒数
uint8 BuzzerState = 0;
uint16 Ps2TimeCount = 0;
uint16 BatteryVoltage;

static u8  fac_us=0;//us延时倍乘数
static u16 fac_ms=0;//ms延时倍乘数


/* 初始化延迟函数 */
void InitDelay(u8 SYSCLK) {
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8); //SYSTICK的时钟固定为HCLK时钟的1/8
	fac_us=SYSCLK/8;//微秒数计时次数
	fac_ms=(u16)fac_us*1000;//毫秒数计时次数
}


/* us延时 */
void DelayUs(u32 nus){
	u32 temp;
	SysTick->LOAD = nus*fac_us; //时间加载
	SysTick->VAL = 0x00;        //清空计数器
	SysTick->CTRL = 0x01 ;      //开始倒数
	do {
		temp = SysTick->CTRL;
	}
	while(temp & 0x01 && !(temp & (1<<16)));//等待时间到达
	SysTick->CTRL = 0x00;//关闭计数器
	SysTick->VAL = 0X00; //清空计数器
}


/* 毫秒延时，最大延时为1864ms */
void DelayMs(u16 ms) {
	u32 temp;
	SysTick->LOAD = (u32)ms * fac_ms;//时间加载
	SysTick->VAL = 0x00;//清空计数器
	SysTick->CTRL = 0x01;//开始倒数
	do {
		temp=SysTick->CTRL;
	}
	while(temp & 0x01 && !(temp & (1<<16)));//等待时间到达
	SysTick->CTRL = 0x00;//关闭计数器
	SysTick->VAL = 0X00;//清空计数器
}


/* 秒延时 */
void DelayS(u16 s) {
	while(s-- > 0){
		DelayMs(1000);
	}
}


/* 初始化LED */
void InitLED(void){
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}


/* 初始化按键 */
void InitKey(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;			//上拉输入
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}


/* 初始化蜂鸣器 */
void InitBuzzer(void){
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}


/* 初始化计时器2 */
void InitTimer2(void) {		//100us
	NVIC_InitTypeDef NVIC_InitStructure;
	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //时钟使能

	TIM_TimeBaseStructure.TIM_Period = (10 - 1); //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =(720-1); //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

	TIM_ITConfig(  //使能或者失能指定的TIM中断
	    TIM2, //TIM2
	    TIM_IT_Update  |  //TIM 中断源
	    TIM_IT_Trigger,   //TIM 触发中断源
	    ENABLE  //使能
	);

	TIM_Cmd(TIM2, ENABLE);  //使能TIMx外设
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM2中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	//从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
}


/* 初始化ADC */
void InitADC(void) {
	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_ADC1, ENABLE);	   //使能ADC1通道时钟
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);//72M/6=12,ADC最大时间不能超过14M
	//PA0/1/2/3 作为模拟通道输入引脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	ADC_DeInit(ADC1);  //将外设 ADC1 的全部寄存器重设为缺省值
	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC工作模式:ADC1和ADC2工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//模数转换工作在单通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//模数转换工作在单次转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;	//顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitStructure);	//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器

	ADC_Cmd(ADC1, ENABLE);	//使能指定的ADC1
	ADC_ResetCalibration(ADC1);	//重置指定的ADC1的校准寄存器
	while(ADC_GetResetCalibrationStatus(ADC1));	//获取ADC1重置校准寄存器的状态,设置状态则等待
	
	ADC_StartCalibration(ADC1);		//开始指定ADC1的校准状态
	while(ADC_GetCalibrationStatus(ADC1));		//获取指定ADC1的校准程序,设置状态则等待
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能
}


/* 获取ADC采集值 */
uint16 GetADCResult(BYTE ch) {
	//设置指定ADC的规则组通道，设置它们的转化顺序和采样时间
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5);	//ADC1,ADC通道3,规则采样顺序值为1,采样时间为239.5周期
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)); //等待转换结束
	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果
}


/* 更新电源电压值 */
void CheckBatteryVoltage(void) {
	uint8 i;
	uint32 v = 0;
	for(i = 0;i < 8;i++) {
		v += GetADCResult(ADC_BAT);
	}
	v >>= 3;
	v = v * 2475 / 1024;//adc / 4096 * 3300 * 3(3代表放大3倍，因为采集电压时电阻分压了)
	BatteryVoltage = v;
	
	printf("voltage:%d", BatteryVoltage);
}


/* 获取电源电压 */
uint16 GetBatteryVoltage(void) {//电压毫伏
	return BatteryVoltage;
}


/* 未外接电源适配器时舵机舵机驱动电压不足，蜂鸣器发出警报 */
void Buzzer(void) {
	static bool fBuzzer = FALSE;
	static uint32 t1 = 0;
	static uint32 t2 = 0;
	
	if(fBuzzer){//蜂鸣器发出警报
		if(++t1 >= 2){//BUZZER以2.5KHZ的频率跳变，频率越高声音越尖锐
			t1 = 0;
			BUZZER = !BUZZER;
		}
	} else {//关闭蜂鸣器
		BUZZER = 0;
	}

	if(BuzzerState == 0){
		fBuzzer = FALSE;
		t2 = 0;
	}
	else if(BuzzerState == 1){
		t2++;
		if(t2 < 5000){//0.5s后开启蜂鸣器
			fBuzzer = TRUE;
		} else if(t2 < 10000) {//0.5s后关闭蜂鸣器，从而达到嘀嘀嘀的目的
			fBuzzer = FALSE;
		} else {
			t2 = 0;
		}
	}
}


BOOL manual = FALSE;


/**
  * @brief  更新系统启动时间，ms
  * 		更新ps2手柄启动时间，ms
  * 		电源电压低于6.4v时开始计时，超过5s后置位BuzzerState
  * @return void
  */
void TIM2_IRQHandler(void) { //100us
	static uint32 time = 0;
	static uint16 timeBattery = 0;
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) { //检查指定的TIM中断发生与否:TIM 中断源
	
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);    //清除TIMx的中断待处理位:TIM 中断源

		Buzzer();
		if(++time >= 10){//100us*10 = 1ms
			time = 0;
			gSystemTickCount++;
			Ps2TimeCount++;
			if(GetBatteryVoltage() < 6400){ //电源电压小于6.4V时报警
				timeBattery++;//1ms自增一次，65s后溢出
				if(timeBattery > 5000){//持续5秒后准备开启蜂鸣器
					BuzzerState = 1;
				}
			} else {
				timeBattery = 0;
				if(manual == FALSE) {
					BuzzerState = 0;
				}
			}
		}
	}
}


/* 定时调用调整PWM脉宽函数和检测电源电压函数 */
void TaskTimeHandle(void) {
	static uint32 time = 10;
	static uint32 times = 0;
	if(gSystemTickCount > time){
		time += 10;//用于if语句，每10ms进入一次
		times++;
		if(times % 2 == 0)//20ms
			ServoPwmDutyCompare();
		if(times % 50 == 0)//500ms
			CheckBatteryVoltage();
	}
}


uint8 i;


void TaskRun(u8 ps2_ok) {
	static bool Ps2State = FALSE;
	static uint8 mode = 0;
	//uint16 lx, ly, rx, ry;
	uint8 PS2KeyValue;
	static uint8 keycount = 0;
	
	TaskTimeHandle();//调整PWM脉宽和检测电源电压
	TaskPCMsgHandle();//使用串口与电脑通信
	TaskBLEMsgHandle();//处理蓝牙接受数据
	TaskRobotRun();//运行动作组

	//按钮长按和短按执行动作组效果不同
	if(KEY == 0) {
		DelayMs(60); {
			if(KEY == 0) {
				keycount++;
			} else {
				if (keycount > 20) {
					keycount = 0;
					FullActRun(100,0);
					return;
				} else {
					keycount = 0;
					LED = ~LED;
					FullActRun(100,1);	
				}
			}
		}
	}
	
	if (ps2_ok == 0) {
		if(Ps2TimeCount > 50) {//每50ms执行
			Ps2TimeCount = 0;
			PS2KeyValue = PS2_DataKey();
			if(mode == 0) {
				if( PS2_Button( PSB_SELECT ) & PS2_ButtonPressed( PSB_START ) ) {//切换模式
					FullActStop();  //停止动作组运行
					ServoSetPluseAndTime( 1, 1500, 1000 );  //将机械臂的舵机都转到1500的位置
					ServoSetPluseAndTime( 2, 1500, 1000 );
					ServoSetPluseAndTime( 3, 1500, 1000 );
					ServoSetPluseAndTime( 4, 1500, 1000 );
					ServoSetPluseAndTime( 5, 1500, 1000 );
					ServoSetPluseAndTime( 6, 1500, 1000 );

					mode = 1;
					Ps2State = 1;
					manual = TRUE;
					BuzzerState = 1;
					LED=~LED;
					DelayMs(80);
					manual = FALSE;
					DelayMs(50);
					manual = TRUE;
					BuzzerState = 1;
					DelayMs(80);
					manual = FALSE;
					LED=~LED;
				}
				else {
					if(PS2KeyValue && !PS2_Button(PSB_SELECT)) {
						LED=~LED;
					}

				switch( PS2KeyValue ) {
					printf("switch key\n");
					//根据按下的按键，控制舵机转动
					case PSB_PAD_LEFT:
						ServoSetPluseAndTime( 6, ServoPwmDutySet[6] + 20, 50 );
						printf("left\n");
						break;

					case PSB_PAD_RIGHT:
						ServoSetPluseAndTime( 6, ServoPwmDutySet[6] - 20, 50 );
						printf("right\n");
						break;

					case PSB_PAD_UP:
						ServoSetPluseAndTime( 5, ServoPwmDutySet[5] + 20, 50 );
						printf("up\n");
						break;

					case PSB_PAD_DOWN:
						ServoSetPluseAndTime( 5, ServoPwmDutySet[5] - 20, 50 );
						printf("down\n");
						break;

					case PSB_L1:
						ServoSetPluseAndTime( 2, ServoPwmDutySet[2] + 20, 50 );
						printf("L1\n");
						break;

					case PSB_R1:
						ServoSetPluseAndTime( 2, ServoPwmDutySet[2] - 20, 50 );
						printf("R1\n");
						break;

					case PSB_L2:
						ServoSetPluseAndTime( 1, ServoPwmDutySet[1] + 20, 50 );
						printf("L2\n");
						break;

					case PSB_R2:
						ServoSetPluseAndTime( 1, ServoPwmDutySet[1] - 20, 50 );
						printf("R2\n");
						break;

					case PSB_TRIANGLE:
						ServoSetPluseAndTime( 4, ServoPwmDutySet[4] - 20, 50 );
						printf("triangle\n");
						break;

					case PSB_CROSS:
						ServoSetPluseAndTime( 4, ServoPwmDutySet[4] + 20, 50 );
						printf("cross\n");
						break;

					case PSB_CIRCLE:
						ServoSetPluseAndTime( 3, ServoPwmDutySet[3] + 20, 50 );
						printf("circle\n");
						break;

					case PSB_SQUARE:
						ServoSetPluseAndTime( 3, ServoPwmDutySet[3] - 20, 50 );
						printf("square\n");
						break;

					case PSB_START:
						printf("start\n");
						ServoSetPluseAndTime( 1, 1500, 1000 );
						ServoSetPluseAndTime( 2, 1500, 1000 );
						ServoSetPluseAndTime( 3, 1500, 1000 );
						ServoSetPluseAndTime( 4, 1500, 1000 );
						ServoSetPluseAndTime( 5, 1500, 1000 );
						ServoSetPluseAndTime( 6, 1500, 1000 );
						break;

					default:
						if (PS2_AnologData(PSS_LX) == 255)//未拨动摇杆时为127
							ServoSetPluseAndTime( 3, ServoPwmDutySet[3] + 30, 60 );

						if (PS2_AnologData(PSS_LX) == 0)
							ServoSetPluseAndTime( 3, ServoPwmDutySet[3] - 30, 60 );

						if (PS2_AnologData(PSS_RY) == 0)
							ServoSetPluseAndTime( 4, ServoPwmDutySet[4] + 30, 60 );

						if (PS2_AnologData(PSS_RY) == 255)
							ServoSetPluseAndTime( 4, ServoPwmDutySet[4] - 30, 60 );

						if (PS2_AnologData(PSS_LY) == 0)
							ServoSetPluseAndTime( 5, ServoPwmDutySet[5] - 30, 60 );

						if (PS2_AnologData(PSS_LY) == 255)
							ServoSetPluseAndTime( 5, ServoPwmDutySet[5] + 30, 60 );

						if (PS2_AnologData(PSS_RX) == 0)
							ServoSetPluseAndTime( 6, ServoPwmDutySet[6] + 30, 60 );

						if (PS2_AnologData(PSS_RX) == 255)
							ServoSetPluseAndTime( 6, ServoPwmDutySet[6] - 30, 60 );
					}
				}
			} else { //mode == 1
				 if( PS2_Button( PSB_SELECT ) && PS2_ButtonPressed( PSB_START ) ) { //检查是不是 SELECT按钮被按住，然后按下START按钮， 是的话，切换模式
						mode = 0; //将模式变为0，就动作组模式
						Ps2State = 0;  //保证动作组运行时不被中断
						manual = TRUE;
						BuzzerState = 1;
						LED=~LED;
						DelayMs(80);
						manual = FALSE;
						DelayMs(50);
						manual = TRUE;
						BuzzerState = 1;
						DelayMs(80);
						manual = FALSE;
						LED=~LED;
				} else {
					if(PS2KeyValue && !Ps2State && !PS2_Button(PSB_SELECT)) {
						LED=~LED;
					}
					
					switch(PS2KeyValue) {
						case 0:
							if(Ps2State) Ps2State = FALSE;
							break;
						
						case PSB_START:
							if(!Ps2State) FullActRun(0,1);
							Ps2State = TRUE;
							break;
						
						case PSB_PAD_UP:
							if(!Ps2State) FullActRun(1,1);
							Ps2State = TRUE;
							break;
						
						case PSB_PAD_DOWN:
							if(!Ps2State) FullActRun(2,1);
							Ps2State = TRUE;
							break;
						
						case PSB_PAD_LEFT:
							if(!Ps2State) FullActRun(3,1);
							Ps2State = TRUE;
							break;
						
						case PSB_PAD_RIGHT:
							if(!Ps2State) FullActRun(4,1);
							Ps2State = TRUE;
							break;

						case PSB_TRIANGLE:
							if(!Ps2State) FullActRun(5,1);
							Ps2State = TRUE;
							break;
							
						case PSB_CROSS:
							if(!Ps2State) FullActRun(6,1);
							Ps2State = TRUE;
							break;
							
						case PSB_SQUARE:
							if(!Ps2State) FullActRun(7,1);
							Ps2State = TRUE;
							break;
							
						case PSB_CIRCLE:
							if(!Ps2State) FullActRun(8,1);
							Ps2State = TRUE;
							break;

						case PSB_L1:
							if(!Ps2State) FullActRun(9,1);
							Ps2State = TRUE;
							break;
							
						case PSB_R1:
							if(!Ps2State) FullActRun(10,1);
							Ps2State = TRUE;
							break;
							
						case PSB_L2:
							if(!Ps2State) FullActRun(11,1);
							Ps2State = TRUE;
							break;
							
						case PSB_R2:
							if(!Ps2State) FullActRun(12,1);
							Ps2State = TRUE;
							break;
				}
			}
		}
	}
}
}
