#include "include.h"

bool fRobotRun = FALSE;//总运行标志位
uint8 ActFullNum = 0;//动作组编号
uint32 ActFullRunTimesSum = 1;//该值为0代表无限次

uint32 ActFullRunTimes = 0;//当前已经运行的次数

uint32 TimeActionRunTotal = 0;//运行时间总计
bool fFrameRunFinish = TRUE;//每帧运行完毕标志位

uint8 FrameIndexSum = 0;//一个动作组里面总共有多少动作
uint8 FrameIndex = 0;//动作组里面某一动作的编号，从0开始


/**
  * @brief  初始化并运行新的动作
  * @param  actFullnum:		动作组编号
  * @param  times: 			运行次数
  * @retval void
  */
void FullActRun(uint8 actFullnum, uint32 times) {
	uint8 frameIndexSum;
	FlashRead(MEM_FRAME_INDEX_SUM_BASE + actFullnum, 1, &frameIndexSum);//获取该动作组的动作数
	
	//该动作组的动作数大于0，说明是有效的，已经下载过动作了。
	if(frameIndexSum > 0) {
		FrameIndexSum = frameIndexSum;
		if(ActFullNum != actFullnum) {
			if(actFullnum == 0) {//0号动作组强制运行，可以中断当前正在运行的其他动作组
				fRobotRun = FALSE;
				ActFullRunTimes = 0;
				fFrameRunFinish = TRUE;
			}
		} else {//只用前后两次动作组编号相同才能修改次数
			ActFullRunTimesSum = times;
		}
		
		if(FALSE == fRobotRun) {//机械臂结束运动
			ActFullNum = actFullnum;
			ActFullRunTimesSum = times;//总运行次数
			FrameIndex = 0;//从第0帧开始
			ActFullRunTimes = 0;//新动作，使当前运动次数为0
			fRobotRun = TRUE;//开始运行
			TimeActionRunTotal = gSystemTickCount;
		}
	}
}


void FullActStop(void) {
	fRobotRun = FALSE;
	ActFullRunTimes = 0;
	fFrameRunFinish = TRUE;
	FrameIndex = 0;
}


/**
  * @brief  运行动作组的某一帧，并返回运行时间
  * @param  actFullnum:		动作组编号
  * @param  frameIndex: 	帧索引
  * @retval 运行时间（ms）
  */
uint16 ActSubFrameRun(uint8 actFullnum,uint8 frameIndex) {
	uint32 i = 0;

	//uint16 frameSumSum = 0;	//由于子动作都是连续存放的，子动作的帧数又是不确定的数
	//所以要总在一起算。所有前面子动作的帧加起来
	uint8 frame[ACT_SUB_FRAME_SIZE];
	uint8 servoCount;
	uint32 time;
	uint8 id;
	uint16 pos;

	//将动作组的所有帧全部读入frame中
	FlashRead((MEM_ACT_FULL_BASE) + (actFullnum * ACT_FULL_SIZE) + (frameIndex * ACT_SUB_FRAME_SIZE) , ACT_SUB_FRAME_SIZE, frame);
	
	servoCount = frame[0];
	time = frame[1] + (frame[2]<<8);

	if(servoCount > 8) {//舵机数超过8个，说明下载了错误动作
		FullActStop();
		return 0;
	}
	
	for(i = 0; i < servoCount; i++) {
		id =  frame[3 + i * 3];
		pos = frame[4 + i * 3] + (frame[5 + i * 3]<<8);
		ServoSetPluseAndTime(id, pos, time);
		//BusServoCtrl(id,SERVO_MOVE_TIME_WRITE,pos,time);
	}
	return time;
}


/* 执行动作组 */
void TaskRobotRun(void){
	if(fRobotRun)
	{
		if(TRUE == fFrameRunFinish)//运行完成后开始下一帧动作运行
		{
			fFrameRunFinish = FALSE;
			TimeActionRunTotal += ActSubFrameRun(ActFullNum, FrameIndex);//运行FrameIndex动作帧，并加上动作时间
		}
		else
		{
			if(gSystemTickCount >= TimeActionRunTotal)//不断检测这帧动作在指定时间内运行完成
			{
				fFrameRunFinish = TRUE;
				if(++FrameIndex >= FrameIndexSum)//已运行完该动作组最后一个动作
				{
					FrameIndex = 0;
					if(ActFullRunTimesSum != 0)//如果运行次数等于0，即代表无限次运行，就不进入if语句，就一直运行了
					{
						if(++ActFullRunTimes >= ActFullRunTimesSum)//到达运行次数，运行停止
						{
							McuToPCSendData(CMD_FULL_ACTION_STOP,0,0);
							fRobotRun = FALSE;
							if(ActFullNum == 100)
								FullActRun(101, 1);
						}
					}
				}
			}
		}
	}
	else
	{
		FrameIndex = 0;
		ActFullRunTimes = 0;
		fFrameRunFinish = TRUE;
		TimeActionRunTotal = gSystemTickCount;
		//只需要在运行完整动作组的最开始赋一下初值就可以，避免累积误差
	}
}







