#include "include.h"


u8 SPIx_ReadWriteByte(u8 TxData);
SPI_InitTypeDef  SPI_InitStructure;


/************************************************
SPI初始化
入口参数: 无
出口参数: 无
************************************************/
void InitSpi(void) {
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;  //SPI CS
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_4);	

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_SPI1, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//选择了串行时钟的稳态:时钟悬空高
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//数据捕获于第二个时钟沿
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;		//定义波特率预分频的值:波特率预分频值为256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式
	SPI_Init(SPI1, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器

	SPI_Cmd(SPI1, ENABLE); //使能SPI外设
	SPIx_ReadWriteByte(0xff);//启动传输
}


void InitFlash(void) {
	InitSpi();
	DelayMs(150);
}

// /************************************************
// 使用SPI方式与Flash进行数据交换
// 入口参数:
//     dat : 准备写入的数据
// 出口参数:
//     从Flash中读出的数据
// ************************************************/
// BYTE SpiShift(BYTE dat)
// {
//     SPDAT = dat;                                //触发SPI发送
//     while (!(SPSTAT & SPIF));                   //等待SPI数据传输完成
//     SPSTAT = SPIF | WCOL;                       //清除SPI状态
//     
//     return SPDAT;
// }


u8 SPIx_ReadWriteByte(u8 TxData) {
	u8 retry=0;
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) { //检查指定的SPI标志位设置与否:发送缓存空标志位
		retry++;
		if(retry>200)return 0;
	}
	SPI_I2S_SendData(SPI1, TxData); //通过外设SPIx发送一个数据
	retry=0;

	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) { //检查指定的SPI标志位设置与否:接受缓存非空标志位
		retry++;
		if(retry>200)return 0;
	}
	return SPI_I2S_ReceiveData(SPI1); //返回通过SPIx最近接收的数据
}


// /************************************************
// 检测Flash的忙状态
// 入口参数: 无
// 出口参数:
//     0 : Flash处于空闲状态
//     1 : Flash处于忙状态
// ************************************************/
// BOOL IsFlashBusy()
// {
//     BYTE dat;
//     
//     SS = 0;
//     SpiShift(SFC_RDSR);                         //发送读取状态命令
//     dat = SpiShift(0);                          //读取状态
//     SS = 1;
//     
//     return (dat & 0x01);                        //状态值的Bit0即为忙标志
// }


u8 SPI_Flash_ReadSR(void) {
	u8 byte=0;
	SPI_FLASH_CS=0;                            //使能器件
	SPIx_ReadWriteByte(W25X_ReadStatusReg);    //发送读取状态寄存器命令
	byte=SPIx_ReadWriteByte(0Xff);             //读取一个字节
	SPI_FLASH_CS=1;                            //取消片选
	return byte;
}

//等待空闲
void SPI_Flash_Wait_Busy(void) {
	while((SPI_Flash_ReadSR()&0x01)==0x01);    // 等待BUSY位清空
}

// /************************************************
// 使能Flash写命令
// 入口参数: 无
// 出口参数: 无
// ************************************************/
// void FlashWriteEnable()
// {
//     while (IsFlashBusy());                      //Flash忙检测
//     SS = 0;
//     SpiShift(SFC_WREN);                         //发送写使能命令
//     SS = 1;
// }


//SPI_FLASH写使能
//将WEL置位
void SPI_FLASH_Write_Enable(void) {
	SPI_FLASH_CS=0;                            //使能器件
	SPIx_ReadWriteByte(W25X_WriteEnable);      //发送写使能
	SPI_FLASH_CS=1;                            //取消片选
}

// /************************************************
// 擦除整片Flash
// 入口参数: 无
// 出口参数: 无
// ************************************************/
// void FlashErase()
// {
//     FlashWriteEnable();                     //使能Flash写命令
//     SS = 0;
//     SpiShift(SFC_CHIPER);                   //发送片擦除命令
//     SS = 1;
// }

/************************************************
擦除扇区 扇区大小4096,Flash最小擦除大小是以扇区为单位擦除
入口参数: 
		addr
出口参数: 无
************************************************/
void FlashEraseSector(DWORD addr) {
	SPI_FLASH_Write_Enable();                  //SET WEL
	SPI_Flash_Wait_Busy();
	SPI_FLASH_CS=0;                            //使能器件
	SPIx_ReadWriteByte(W25X_SectorErase);      //发送扇区擦除指令
	SPIx_ReadWriteByte((u8)((addr)>>16));  //发送24bit地址
	SPIx_ReadWriteByte((u8)((addr)>>8));
	SPIx_ReadWriteByte((u8)addr);
	SPI_FLASH_CS=1;                            //取消片选
	SPI_Flash_Wait_Busy();   				   //等待擦除完成
}

/************************************************
从Flash中读取数据
入口参数:
    addr   : 地址参数
    size   : 数据块大小
    buffer : 缓冲从Flash中读取的数据
出口参数:
    无
************************************************/
void FlashRead(DWORD addr, DWORD size, BYTE *buffer) {
    u16 i;
	SPI_FLASH_CS=0;                            //使能器件
	SPIx_ReadWriteByte(W25X_ReadData);         //发送读取命令
	SPIx_ReadWriteByte((u8)((addr)>>16));  //发送24bit地址
	SPIx_ReadWriteByte((u8)((addr)>>8));
	SPIx_ReadWriteByte((u8)addr);
	for(i=0; i<size; i++) {
		buffer[i]=SPIx_ReadWriteByte(0XFF);   //循环读数
	}
	SPI_FLASH_CS=1;                            //取消片选
}


/************************************************
写数据到Flash中
入口参数:
    addr   : 地址参数
    size   : 数据块大小
    buffer : 缓冲需要写入Flash的数据
出口参数: 无
************************************************/
void FlashWrite(DWORD addr, DWORD size, BYTE *buffer)
{
    u16 i;
	SPI_FLASH_Write_Enable();                  //SET WEL
	SPI_FLASH_CS=0;                            //使能器件
	SPIx_ReadWriteByte(W25X_PageProgram);      //发送写页命令
	SPIx_ReadWriteByte((u8)((addr)>>16)); //发送24bit地址
	SPIx_ReadWriteByte((u8)((addr)>>8));
	SPIx_ReadWriteByte((u8)addr);
	for(i=0; i<size; i++)SPIx_ReadWriteByte(buffer[i]); //循环写数
	SPI_FLASH_CS=1;                            //取消片选
	SPI_Flash_Wait_Busy();					   //等待写入结束
}










