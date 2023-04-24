#include "include.h"



// ��������
int fputc(int ch, FILE *f) {
	USART_SendData(USART1, (unsigned char) ch);
	while (!(USART1->SR & USART_FLAG_TXE));
	return (ch);
}

// ��������
int GetKey (void) {
	while (!(USART1->SR & USART_FLAG_RXNE));
	return ((int)(USART1->DR & 0x1FF));
}



int main(void) {
	uint8 ps_ok = 1;
	
	SystemInit(); 			//ϵͳʱ�ӳ�ʼ��Ϊ72M	  SYSCLK_FREQ_72MHz
	InitDelay(72);	     	//��ʱ��ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�

	InitPWM();
	InitTimer2();	//����Դ��ѹ
	InitUart1();	//������PC�˽���ͨ��
	InitUart3();	//�������ģ��
	InitADC();
	InitLED();
	InitKey();
	InitBuzzer();
	ps_ok = InitPS2();//PS2��Ϸ�ֱ���������ʼ����Ϊ0
	
	InitFlash();
	InitMemory();
	InitBusServoCtrl();
	LED = LED_ON;
	
	/* ���� */
	int te = 0;
	DelayS(4);
	printf("start\n");
	printf("ps_ok=%d\n", ps_ok);
	ps_ok = InitPS2();//PS2��Ϸ�ֱ���������ʼ����Ϊ0
	printf("ps_ok=%d\n", ps_ok);
	
	while(1) {
		TaskRun(ps_ok);
	}
}






