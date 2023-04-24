#include "include.h"

bool fRobotRun = FALSE;//�����б�־λ
uint8 ActFullNum = 0;//��������
uint32 ActFullRunTimesSum = 1;//��ֵΪ0�������޴�

uint32 ActFullRunTimes = 0;//��ǰ�Ѿ����еĴ���

uint32 TimeActionRunTotal = 0;//����ʱ���ܼ�
bool fFrameRunFinish = TRUE;//ÿ֡������ϱ�־λ

uint8 FrameIndexSum = 0;//һ�������������ܹ��ж��ٶ���
uint8 FrameIndex = 0;//����������ĳһ�����ı�ţ���0��ʼ


/**
  * @brief  ��ʼ���������µĶ���
  * @param  actFullnum:		��������
  * @param  times: 			���д���
  * @retval void
  */
void FullActRun(uint8 actFullnum, uint32 times) {
	uint8 frameIndexSum;
	FlashRead(MEM_FRAME_INDEX_SUM_BASE + actFullnum, 1, &frameIndexSum);//��ȡ�ö�����Ķ�����
	
	//�ö�����Ķ���������0��˵������Ч�ģ��Ѿ����ع������ˡ�
	if(frameIndexSum > 0) {
		FrameIndexSum = frameIndexSum;
		if(ActFullNum != actFullnum) {
			if(actFullnum == 0) {//0�Ŷ�����ǿ�����У������жϵ�ǰ�������е�����������
				fRobotRun = FALSE;
				ActFullRunTimes = 0;
				fFrameRunFinish = TRUE;
			}
		} else {//ֻ��ǰ�����ζ���������ͬ�����޸Ĵ���
			ActFullRunTimesSum = times;
		}
		
		if(FALSE == fRobotRun) {//��е�۽����˶�
			ActFullNum = actFullnum;
			ActFullRunTimesSum = times;//�����д���
			FrameIndex = 0;//�ӵ�0֡��ʼ
			ActFullRunTimes = 0;//�¶�����ʹ��ǰ�˶�����Ϊ0
			fRobotRun = TRUE;//��ʼ����
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
  * @brief  ���ж������ĳһ֡������������ʱ��
  * @param  actFullnum:		��������
  * @param  frameIndex: 	֡����
  * @retval ����ʱ�䣨ms��
  */
uint16 ActSubFrameRun(uint8 actFullnum,uint8 frameIndex) {
	uint32 i = 0;

	//uint16 frameSumSum = 0;	//�����Ӷ�������������ŵģ��Ӷ�����֡�����ǲ�ȷ������
	//����Ҫ����һ���㡣����ǰ���Ӷ�����֡������
	uint8 frame[ACT_SUB_FRAME_SIZE];
	uint8 servoCount;
	uint32 time;
	uint8 id;
	uint16 pos;

	//�������������֡ȫ������frame��
	FlashRead((MEM_ACT_FULL_BASE) + (actFullnum * ACT_FULL_SIZE) + (frameIndex * ACT_SUB_FRAME_SIZE) , ACT_SUB_FRAME_SIZE, frame);
	
	servoCount = frame[0];
	time = frame[1] + (frame[2]<<8);

	if(servoCount > 8) {//���������8����˵�������˴�����
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


/* ִ�ж����� */
void TaskRobotRun(void){
	if(fRobotRun)
	{
		if(TRUE == fFrameRunFinish)//������ɺ�ʼ��һ֡��������
		{
			fFrameRunFinish = FALSE;
			TimeActionRunTotal += ActSubFrameRun(ActFullNum, FrameIndex);//����FrameIndex����֡�������϶���ʱ��
		}
		else
		{
			if(gSystemTickCount >= TimeActionRunTotal)//���ϼ����֡������ָ��ʱ�����������
			{
				fFrameRunFinish = TRUE;
				if(++FrameIndex >= FrameIndexSum)//��������ö��������һ������
				{
					FrameIndex = 0;
					if(ActFullRunTimesSum != 0)//������д�������0�����������޴����У��Ͳ�����if��䣬��һֱ������
					{
						if(++ActFullRunTimes >= ActFullRunTimesSum)//�������д���������ֹͣ
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
		//ֻ��Ҫ������������������ʼ��һ�³�ֵ�Ϳ��ԣ������ۻ����
	}
}







