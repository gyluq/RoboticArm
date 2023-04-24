#ifndef _FLASH_H_
#define _FLASH_H_

#define	SPI_FLASH_CS PAout(4)  //选中FLASH	

//W25X16读写
#define W25Q16			0XEF14
#define W25Q128			0XEF17
#define SPI_FLASH_TYPE	W25Q128
#define FLASH_ID		SPI_FLASH_TYPE

//指令表
#define W25X_WriteEnable		0x06
#define W25X_WriteDisable		0x04
#define W25X_ReadStatusReg		0x05
#define W25X_WriteStatusReg		0x01
#define W25X_ReadData			0x03
#define W25X_FastReadData		0x0B
#define W25X_FastReadDual		0x3B
#define W25X_PageProgram		0x02
#define W25X_BlockErase			0xD8
#define W25X_SectorErase		0x20
#define W25X_ChipErase			0xC7
#define W25X_PowerDown			0xB9
#define W25X_ReleasePowerDown	0xAB
#define W25X_DeviceID			0xAB
#define W25X_ManufactDeviceID	0x90
#define W25X_JedecDeviceID		0x9F



void InitFlash(void);
void InitSpi(void);
// BYTE SpiShift(BYTE dat);
// BOOL IsFlashBusy();
// void FlashWriteEnable(void);
void FlashEraseSector(DWORD addr);
// void FlashErase();
void FlashRead(DWORD addr, DWORD size, BYTE *buffer);
void FlashWrite(DWORD addr, DWORD size, BYTE *buffer);

#endif

