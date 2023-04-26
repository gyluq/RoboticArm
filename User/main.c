#include "include.h"


int main(void) {
	uint8 ps_ok = 1;
	
	SystemInit(); 			//系统时钟初始化为72M	  SYSCLK_FREQ_72MHz
	InitDelay(72);	     	//延时初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//设置NVIC中断分组2:2位抢占优先级，2位响应优先级

	InitPWM();
	InitTimer2();	//检测电源电压
	InitUart1();	//用于与PC端进行通信
	InitUart3();	//外接蓝牙模块
	InitADC();
	InitLED();
	InitKey();
	InitBuzzer();
	ps_ok = InitPS2();//PS2游戏手柄接收器初始化，为0
	
	InitFlash();
	InitMemory();
	InitBusServoCtrl();
	LED = LED_ON;
	
	while(1) {
		TaskRun(ps_ok);
	}
}






