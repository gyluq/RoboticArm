#ifndef _BUS_SERVO_CTRL_H_
#define _BUS_SERVO_CTRL_H_


#define UART_RX_ENABLE()		PAout(0) = 1;PAout(1) = 0
#define UART_TX_ENABLE()		PAout(0) = 0;PAout(1) = 1


#define BROADCAST_ID 0xFE


#define SERVO_MOVE_TIME_WRITE 1 //通过时间控制舵机打到指定角度


#define SERVO_MOVE_TIME_DATA_LEN 7

void InitBusServoCtrl(void);

void BusServoCtrl(uint8 id,uint8 cmd,uint16 prm1,uint16 prm2);

#endif
