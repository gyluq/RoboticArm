#include "include.h"


static bool fUartRxComplete = FALSE;
static uint8 UartRxBuffer[260];
uint8 Uart1RxBuffer[260];

// static bool UartBusy = FALSE;

uint8  frameIndexSumSum[256];


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


/* 初始化UART1 */
void InitUart1(void) {
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
	//USART1_TX   PA.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//USART1_RX	  PA.10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//USART 初始化设置
	USART_InitStructure.USART_BaudRate = 9600;//一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启中断
	USART_Cmd(USART1, ENABLE);                    //使能串口
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器USART1
}


/* 通过串口发送一字节数据 */
void Uart1SendData(BYTE dat) {
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕
	USART1->DR = (u8) dat;
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕
}


/* 通过串口发送多个字节数据 */
void UART1SendDataPacket(uint8 dat[],uint8 count) {
	uint32 i;
	for(i = 0; i < count; i++) {
		while((USART1->SR&0X40)==0);//循环发送,直到发送完毕
		USART1->DR = dat[i];
		while((USART1->SR&0X40)==0);//循环发送,直到发送完毕
	}
}


/* 接受电脑传送的数据，并解析指令 */
void USART1_IRQHandler(void) {
	uint8 i;
	uint8 rxBuf;

	static uint8 startCodeSum = 0;
	static bool fFrameStart = FALSE;
	static uint8 messageLength = 0;
	static uint8 messageLengthSum = 2;

    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        rxBuf = USART_ReceiveData(USART1); //读取接收到的数据
		if(!fFrameStart) {
			if(rxBuf == 0x55) { //连续发送两个0x55后，再发送数据
				startCodeSum++;
				if(startCodeSum == 2) {
					startCodeSum = 0;
					fFrameStart = TRUE; //进入下一个if，开始解析数据
					messageLength = 1;
				}
			} else {
				fFrameStart = FALSE;
				messageLength = 0;
				startCodeSum = 0;
			}
		} if(fFrameStart) {
			Uart1RxBuffer[messageLength] = rxBuf;
			if(messageLength == 2)//索引0、1为0x55,索引2为数据长度
			{ 
				messageLengthSum = Uart1RxBuffer[messageLength];
				if(messageLengthSum < 2) {// || messageLengthSum > 30 
					messageLengthSum = 2;
					fFrameStart = FALSE;
				}
			}
			messageLength++;
	
			if(messageLength == messageLengthSum + 2) { //最后一个字节数据
				if(fUartRxComplete == FALSE) {
					fUartRxComplete = TRUE;
					for(i = 0;i < messageLength;i++) {
						UartRxBuffer[i] = Uart1RxBuffer[i]; //复制数据到UartRxBuffer
					}
				}
				fFrameStart = FALSE;
			}
		}
    }
}


/* 向电脑发送数据,用于串口控制机械臂时回传数据 */
void McuToPCSendData(uint8 cmd,uint8 prm1,uint8 prm2) {
	uint8 dat[8];
	uint8 datlLen = 2;
	switch(cmd) {
//		case CMD_ACTION_DOWNLOAD:
//			datlLen = 2;
//			break;
		default:
			datlLen = 2;
			break;
	}
	dat[0] = 0x55;
	dat[1] = 0x55;
	dat[2] = datlLen;
	dat[3] = cmd;
	dat[4] = prm1;
	dat[5] = prm2;
	UART1SendDataPacket(dat,datlLen + 2);
}


/* 串口接受完毕 */
static bool UartRxOK(void) {
	if(fUartRxComplete) {
		fUartRxComplete = FALSE;
		return TRUE;
	} else {
		return FALSE;
	}
}


void FlashEraseAll(void);
void SaveAct(uint8 fullActNum,uint8 frameIndexSum,uint8 frameIndex,uint8* pBuffer);


/* 接受电脑传输指令并执行 */
void TaskPCMsgHandle(void) {
	uint16 i;
	uint8 cmd;
	uint8 id;
	uint8 servoCount;
	uint16 time;
	uint16 pos;
	uint16 times;
	uint8 fullActNum;
	
	if(UartRxOK())
	{
		LED = !LED;
		cmd = UartRxBuffer[3];
 		switch(cmd)
		{
 			case CMD_MULT_SERVO_MOVE://控制多个舵机转动
				servoCount = UartRxBuffer[4];
				time = UartRxBuffer[5] + (UartRxBuffer[6]<<8);
				for(i = 0; i < servoCount; i++) {
					id =  UartRxBuffer[7 + i * 3];
					pos = UartRxBuffer[8 + i * 3] + (UartRxBuffer[9 + i * 3]<<8);
					ServoSetPluseAndTime(id,pos,time);
//					BusServoCtrl(id,SERVO_MOVE_TIME_WRITE,pos,time);
				}				
 				break;

			case CMD_FULL_ACTION_RUN://动作组运行
				fullActNum = UartRxBuffer[4];//动作组编号
				times = UartRxBuffer[5] + (UartRxBuffer[6]<<8);//运行次数
				McuToPCSendData(CMD_FULL_ACTION_RUN, 0, 0);
				FullActRun(fullActNum,times);
				break;

			case CMD_FULL_ACTION_STOP://停止正在运行的动作
				FullActStop();
				break;

			case CMD_FULL_ACTION_ERASE://将下载到控制板的动作组查出
				FlashEraseAll();
				McuToPCSendData(CMD_FULL_ACTION_ERASE,0,0);
				break;

			case CMD_ACTION_DOWNLOAD://下载动作组，一帧一帧地下载
				SaveAct(UartRxBuffer[4], UartRxBuffer[5], UartRxBuffer[6], UartRxBuffer + 7);
				McuToPCSendData(CMD_ACTION_DOWNLOAD,0,0);
				break;
 		}
	}
}


/**
  * @brief  动作组一帧数据写入flash
  * @param  fullActNum:		要下载到几号动作组
  * @param  frameIndexSum: 	该动作的总帧数
  * @param  frameIndex：	要写入第几帧数据
  * @param  pBuffer：		其他数据
  * @retval void
  */
void SaveAct(uint8 fullActNum, uint8 frameIndexSum, uint8 frameIndex, uint8* pBuffer) {
	uint8 i;
	if(frameIndex == 0) {//写入第一帧数据时擦除动作组扇区
		//一个动作组占16k大小，擦除一个扇区是4k，所以要擦4次
		for(i = 0;i < 4;i++) {
			FlashEraseSector((MEM_ACT_FULL_BASE) + (fullActNum * ACT_FULL_SIZE) + (i * 4096));
		}
	}

	//写入一帧数据，共64字节
	FlashWrite((MEM_ACT_FULL_BASE) + (fullActNum * ACT_FULL_SIZE) + (frameIndex * ACT_SUB_FRAME_SIZE) , ACT_SUB_FRAME_SIZE, pBuffer);
	
	if((frameIndex + 1) ==  frameIndexSum) {
		//读取256组动作的动作数并保存到frameIndexSumSum
		FlashRead(MEM_FRAME_INDEX_SUM_BASE, 256, frameIndexSumSum);
		//更新该组动作的动作数
		frameIndexSumSum[fullActNum] = frameIndexSum;
		//擦除页
		FlashEraseSector(MEM_FRAME_INDEX_SUM_BASE);
		//写入修改后的数据
		FlashWrite(MEM_FRAME_INDEX_SUM_BASE,256,frameIndexSumSum);
	}
}


/* 将所有255个动作组的动作数设置为0，即代表将所有动作组擦除 */
void FlashEraseAll(void) {
	uint16 i;
	for(i = 0;i <= 255;i++) {
		frameIndexSumSum[i] = 0;
	}
	FlashEraseSector(MEM_FRAME_INDEX_SUM_BASE);
	FlashWrite(MEM_FRAME_INDEX_SUM_BASE, 256, frameIndexSumSum);
}


/* 初始化flash */
void InitMemory(void) {
	uint8 i;
	uint8 logo[] = "LOBOT";
	uint8 datatemp[8];
	FlashRead(MEM_LOBOT_LOGO_BASE,5,datatemp);
	for(i = 0; i < 5; i++) {
		if(logo[i] != datatemp[i]) {
		LED = LED_ON;
			//如果发现不相等的，则说明是新FLASH，需要初始化
			FlashEraseSector(MEM_LOBOT_LOGO_BASE);
			FlashWrite(MEM_LOBOT_LOGO_BASE,5,logo);
			FlashEraseAll();
			break;
		}
	}
}



