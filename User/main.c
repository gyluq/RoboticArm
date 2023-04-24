#include "include.h"



// 发送数据
int fputc(int ch, FILE *f) {
	USART_SendData(USART1, (unsigned char) ch);
	while (!(USART1->SR & USART_FLAG_TXE));
	return (ch);
}

// 接收数据
int GetKey (void) {
	while (!(USART1->SR & USART_FLAG_RXNE));
	return ((int)(USART1->DR & 0x1FF));
}



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
	
	/* 测试 */
	int te = 0;
	DelayS(4);
	printf("start\n");
	printf("ps_ok=%d\n", ps_ok);
	ps_ok = InitPS2();//PS2游戏手柄接收器初始化，为0
	printf("ps_ok=%d\n", ps_ok);
	
	while(1) {
		TaskRun(ps_ok);
	}
}






