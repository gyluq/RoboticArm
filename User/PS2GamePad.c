#include "include.h"


u16 Handkey;
u8 Comd[2]={0x01,0x42};	//开始命令。请求数据
u8 Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //数据存储数组
u16 MASK[]={
    PSB_SELECT,
    PSB_L3,
    PSB_R3 ,
    PSB_START,
    PSB_PAD_UP,
    PSB_PAD_RIGHT,
    PSB_PAD_DOWN,
    PSB_PAD_LEFT,
    PSB_L2,
    PSB_R2,
    PSB_L1,
    PSB_R1 ,
    PSB_TRIANGLE,
    PSB_CIRCLE,
    PSB_CROSS,
    PSB_SQUARE
};	//按键值与按键明



static void Delay(unsigned int time)
{
// 	time *=6;
// 	while(--time);
	DelayUs(time);
}


/* 初始化PS2手柄接收器 */
u8 InitPS2(void) {
	u8 mode;
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	mode = PS2_SetInit();//配配置初始化,配置“红绿灯模式”，并选择是否可以修改		
	return mode;
}


//向手柄发送命令
void PS2_Cmd(u8 CMD) {
	volatile u16 ref=0x01;
	Data[1] = 0;
	for(ref = 0x01; ref < 0x0100; ref <<= 1) {
		if(ref & CMD) {
			DO_H;                   //输出以为控制位
		}
		else DO_L;

		Delay(10);
		CLK_L;
		Delay(40);
		CLK_H;
		
		if(DI) Data[1] = ref | Data[1];
		Delay(10);
	}
}


//判断是否为红灯模式
//返回值；0，红灯模式
//		  其他，其他模式
u8 PS2_RedLight(void) {
	CS_L;
	PS2_Cmd(Comd[0]);  //开始命令
	PS2_Cmd(Comd[1]);  //请求数据
	CS_H;
	if( Data[1] == 0X73)   return 0 ;
	else return 1;
}


/* 读取手柄数据 */
void PS2_ReadData(void) {
	volatile u8 byte;
	volatile u16 ref;

	CS_L;
	Delay(10);
	PS2_Cmd(Comd[0]);  //开始命令
	PS2_Cmd(Comd[1]);  //请求数据
	for(byte=2;byte<9;byte++) {          //开始接受数据
		for(ref=0x01;ref<0x100;ref<<=1) {
			CLK_L; //在下降沿发送数据
			Delay(50);
			CLK_H;
		    if(DI) {
				Data[byte] = ref|Data[byte];
			}
			Delay(20);
		}
			Delay(40);
	}
	CS_H;
}


//对读出来的PS2的数据进行处理，只处理了按键部分，默认数据是红灯模式，只有一个按键按下时
//按下为0，未按下为1
u16 LastHandkey = 0xFFFF;


/* 检测某一按下的按键，返回按键索引值+1 */
u8 PS2_DataKey() {
	u8 index;

	PS2_ClearData();
	PS2_ReadData();
	LastHandkey = Handkey;
	Handkey=(Data[4]<<8)|Data[3];     //这是16个按键  按下为0， 未按下为1
	for(index=0;index<16;index++) {	    
		if((Handkey&(1<<(MASK[index]-1)))==0)
		return index+1;
	}
	return 0;//没有任何按键按下
}



/**
  * @brief  检测按键状态是否变化，将上次的按键数据和这次的按键数据进行异或运算，结果就是两次不同的部分会是1
  * @button  button:按键在数据中所在bit的值+1，例如PSB_SELECT宏的值是1，在数据中的位是0位
  * @retval 
  * 	0：未改变
  * 	1：发生变化
  */
bool PS2_NewButtonState( u16 button ) {
	button = 0x0001u << ( button - 1 );
	return ( ( ( LastHandkey ^ Handkey ) & button ) > 0 );
}


/**
  * @brief  检测按键状态
  * @button  button:按键在数据中所在bit的值+1，例如PSB_SELECT宏的值是1，在数据中的位是0位
  * @retval 
  * 	0：松开状态
  * 	1：按下状态
  */
bool PS2_Button( u16 button ) {
  button = 0x0001u << ( button - 1 );
  return ( ( (~Handkey) & button ) > 0 );
}


/**
  * @brief  检测按键是否被按下，按键被按住，并且这个是按键的一个新的状态，那么就是按键刚被按下
  * @button  button:按键在数据中所在bit的值+1，例如PSB_SELECT宏的值是1，在数据中的位是0位
  * @retval 
  * 	0：表示按键未按下
  * 	1：表示按键被按下
  */
bool PS2_ButtonPressed( u16 button ) {
  return (PS2_NewButtonState( button ) && PS2_Button( button ));  //
}


/**
  * @brief  检测按键是否被释放，按键没被按住，并且这个是按键的一个新的状态，那么就是按键刚被松开
  * @button  button:按键在数据中所在bit的值+1，例如PSB_SELECT宏的值是1，在数据中的位是0位
  * @retval 
  * 	0：表示按键未释放
  * 	1：表示按键被释放
  */
bool PS2_ButtonReleased( u16 button ) {
  return ( PS2_NewButtonState( button ) && !PS2_Button( button ));
}


//得到一个摇杆的模拟量，范围0~256
u8 PS2_AnologData(u8 button) {
	return Data[button];
}


//清除数据缓冲区
void PS2_ClearData() {
	u8 a;
	for(a=0;a<9;a++)
		Data[a]=0x00;
}


/******************************************************
Function:    void PS2_Vibration(u8 motor1, u8 motor2)
Description: 手柄震动函数，
Calls:		 void PS2_Cmd(u8 CMD);
Input: motor1:右侧小震动电机 0x00关，其他开
	   motor2:左侧大震动电机 0x40~0xFF 电机开，值越大 震动越大
******************************************************/
void PS2_Vibration(u8 motor1, u8 motor2) {
	CS_L;
	Delay(50);
    PS2_Cmd(0x01);  //开始命令
	PS2_Cmd(0x42);  //请求数据
	PS2_Cmd(0X00);
	PS2_Cmd(motor1);
	PS2_Cmd(motor2);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	Delay(50);  
}


//short poll
void PS2_ShortPoll(void) {
	CS_L;
	Delay(50);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x42);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x00);
	CS_H;
	Delay(50);	
}


//进入配置
void PS2_EnterConfing(void) {
    CS_L;
	Delay(50);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);
	PS2_Cmd(0X00);
	PS2_Cmd(0x01);
	PS2_Cmd(0x00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	Delay(50);
}


//发送模式设置
void PS2_TurnOnAnalogMode(void) {
	CS_L;
	Delay(50);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x44);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01); //analog=0x01;digital=0x00  软件设置发送模式
	PS2_Cmd(0x03); //Ox03锁存设置，即不可通过按键“MODE”设置模式。
				   //0xEE不锁存软件设置，可通过按键“MODE”设置模式。
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	Delay(50);
}


//振动设置
void PS2_VibrationMode(void) {
	CS_L;
	Delay(50);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x4D);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0X01);
	CS_H;
	Delay(50);	
}


//完成并保存配置
void PS2_ExitConfing(void) {
    CS_L;
	Delay(50);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	CS_H;
	Delay(50);
}


//手柄配置初始化
u8 PS2_SetInit(void) {
	DelayS(4);
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_EnterConfing();		//进入配置模式
	PS2_TurnOnAnalogMode();	//“红绿灯”配置模式，并选择是否保存
	PS2_VibrationMode();	//开启震动模式
	PS2_ExitConfing();		//完成并保存配置
	if (PS2_RedLight() == 0)
		return 0;
	else
		return 1;
}





