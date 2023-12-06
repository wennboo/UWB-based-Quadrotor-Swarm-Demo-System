#include "wCommunication.h"
#include "cNRF24l01.h"


extern QUAV_STATE_P qState;				 //无人机姿态参数
extern CTRL_PID_P qRpid, qPpid, qYpid, qGRpid, qGPpid, qGYpid;
extern CTRL_PID_P qCXpid, qCYpid, qCZpid, qVXpid, qVYpid, qVZpid;
extern QUAV_POWER_P qPW;



//接收遥控器的控制信号
QUAV_CTRL_SIG qRC_Sig;

static u8 Get_PID[4] = {0}; //修改PID参数标志位，调试用，可以用信号量代替

static u8 suTxBuf[COM_TX_WIDTH];
static u8 suANO_Tx_Flag = 1;	 //发送标志位
static u8 suANO_Tx_PID_Flag = 0; //发送pid标志位
static u8 suNRF_Tx_Flag = 0;	 //发送标志位

u8 uU_FHead[2] = {0xAA, 0xAA}; //帧头飞控->上位机(遥控)，上行
u8 uD_FHead[2] = {0xAA, 0xAF};	  //帧头上位机（遥控）->飞控，下行

u8 uANO_Head_Len = 4; //帧头2个加1个功能字加1位数据长度
u8 uNRF_Head_Len = 4; //帧头2个加1个功能字加1位数据长度



QUAV_RC_T qRC_T = {
	.fB_PidS = 0.001, // pid变换因子
	.fB_EuS = 60,	  //欧拉角变换因子
	.fB_PwS = 900,	  //动力变换因子
	.uLim = 60,		  //整数型输入门限
	.uBias = 2048,	  //整数型输入偏置
	.uRange = 4096,	  //整数型输入范围
	.uLow = 500	,  //用于解锁
	.uMiddle = 1000	,  //用于解锁
	.uHigh = 2048	  //用于解锁

};

//遥控通道值
static Q_RC_KEY qRC_Key; 


static QUAV_FTOB_P qFtob = {
	.fF_PidS = 1000, // FtoB	Pid系数变换因子
	.fF_PwS = 100,	 // FtoB	电压变换因子
	.fF_AcS = 1,	 // FtoB	加速度变换因子
	.fF_GyS = 1,	 // FtoB	角速度变换因子
	.fF_MaS = 1,	 // FtoB	磁力计变换因子
	.fF_EuS = 100,	 // FtoB	欧拉角变换因子
	.fF_PoS = 1,	 // FtoB	位置变换因子
	.fF_VeS = 1,	 // FtoB	绝对速度变换因子
	.fF_TeS = 1,	 // FtoB	温度变换因子
	.fF_PrS = 1,	 // FtoB	气压变换因子
	.fF_HeS = 1		 // FtoB	海拔m变换因子

};

/*************************************************************
 * vComm_Task任务
 * 获得上位机传来的数据，获取PID参数和无人机姿态数据
 **************************************************************/
void vComm_Task(void *pvParameters)
{
	u32 uLastWakeTime = uGetSysTickCnt();
	u8 uCnt = 0, uSum = 0;
	while (1)
	{
		if (qState.uFly_F)
		{
			uSum++;
			uCnt += nNRF_RX_Frame_Decode();
			//若一秒内没有接收到2.4G的数据，无人机进入锁定状态，防止空中失控
			if (uSum % 100 == 0)
			{
				if (uCnt == 0)
				{
					qState.Mode = LOCK;
				}
				else
					uCnt = 0;
				uSum = 0;
			}
			//上位机发送数据到无人机
			nANO_RX_Frame_Decode();
			if (suANO_Tx_Flag) //上传数据判定（串口控制）
			{
				vANO_TX_Frame_Encode(); //上传无人机状态、传感及pid等数据
			}
		}

		//此任务以200Hz的频率运行
		vTaskDelayUntil(&uLastWakeTime, F2T(COM_TASK_RATE));
	}
}


/*************************************************************
 * vRC_Task任务
 * 解析遥控器数据
 **************************************************************/
void vRC_Task(void *pvParameters)
{
	u32 uLastWakeTime = uGetSysTickCnt();
	EQ_RC_M Status = WAITING_1;
	qState.Mode = LOCK;
	uint16_t uCnt = 0;
	while (1)
	{
		nNRF_RX_Frame_Decode();//接收遥控器数据
		//若飞机上锁
		if (qState.Mode == LOCK)
		{
			switch (Status) //解锁状态机
			{
				case WAITING_1: //等待解锁
				{
					if (qRC_Key.sThr < qRC_T.uMiddle)
					{
//						printf("waiting2");
						Status = WAITING_2;
					}
					break;
				}
				case WAITING_2: //解锁2
				{
					if (qRC_Key.sThr > qRC_T.uHigh)
					{
						vDelay_ms(100);
						if (qRC_Key.sThr > qRC_T.uHigh) //最高油门需保持100ms以上
						{
//							printf("waiting3");
							Status = WAITING_3;
						}
						else
						{
							Status = WAITING_1;
						}
					}

					break;
				}
				case WAITING_3: //解锁3
				{
					if (qRC_Key.sThr < qRC_T.uMiddle) //拉低油门解锁
					{
//						printf("working1");
						Status = WORK; //解锁标志位
					}
					else
					{
						vDelay_ms(60);
						uCnt++;
					if (uCnt > 5)
					{
						uCnt = 0;
//						printf("back");
						Status = WAITING_1;
					}
//						vDelay_ms(300);//再延长100ms等下看看
//						if (qRC_Key.sThr < qRC_T.uMiddle)
//						{
//							printf("working2");
//							Status = WORK; //解锁标志位

//						}
//						else
//						{
//							printf("back");
//							Status = WAITING_1;
//						}
					}
					break;
				}
				case WORK: //遥控器进入工作状态
				{
					uCnt=0;
					//printf(" change!!! ");
					qState.Mode = RCMODE; // NORMAL HEIGHT
					Status = WAITING_1;
					break;
				}
				default:
				{
					Status = WAITING_1;
					break;
				}
					
			}
		}
		//否则解锁
		else 
		{
			if (qRC_Key.sThr < qRC_T.uLow)
			{
				uCnt++;
				vDelay_ms(500);
				if(uCnt > 50)
				{
					uCnt=0;
//					printf(" lock!!! ");
					qState.Mode = LOCK;
					Status = WAITING_1;
				}
			}
			else
			{
				uCnt=0;
			}
			// Yaw遥控值范围-30~+30
			// qYpid.fRef = -fInt_To_Float(qRC_Key.sYaw, qRC_T.uLim, qRC_T.uBias, qRC_T.uRange, qRC_T.fB_EuS);
			qRC_Sig.fYaw = -fInt_To_Float(qRC_Key.sYaw, qRC_T.uLim, qRC_T.uBias, qRC_T.uRange, qRC_T.fB_EuS);
			// //油门范围0-900
			// qPW.sPw = (s16)(fInt_To_Float(qRC_Key.sThr, qRC_T.uLim, 0, qRC_T.uRange, qRC_T.fB_PwS));
			qRC_Sig.fThrust = fInt_To_Float(qRC_Key.sThr, qRC_T.uLim, 0, qRC_T.uRange, qRC_T.fB_PwS);
			// // Roll,Pitch遥控值范围-30~+30
			// qRpid.fRef = fInt_To_Float(qRC_Key.sRoll, qRC_T.uLim, qRC_T.uBias, qRC_T.uRange, qRC_T.fB_EuS);
			qRC_Sig.fRoll = fInt_To_Float(qRC_Key.sRoll, qRC_T.uLim, qRC_T.uBias, qRC_T.uRange, qRC_T.fB_EuS);
			// qPpid.fRef = -fInt_To_Float(qRC_Key.sPitch, qRC_T.uLim, qRC_T.uBias, qRC_T.uRange, qRC_T.fB_EuS);
			qRC_Sig.fPitch = -fInt_To_Float(qRC_Key.sPitch, qRC_T.uLim, qRC_T.uBias, qRC_T.uRange, qRC_T.fB_EuS);
		}
		vTaskDelayUntil(&uLastWakeTime, F2T(COM_TASK_RATE));
	}
}


/*************************************************************
 *vLED_Task任务
 * 无人机接收到PID参数后亮灯显示
 **************************************************************/
void vLED_Task(void *pvParameters)
{
	u32 uLastWakeTime = uGetSysTickCnt();
	while (1)
	{
		vVoltage_Read(&qState.fVoltage);
		//若接收到PID1-6的数据，4灯闪烁2次
		if (Get_PID[0] == 1 && Get_PID[1] == 1)
		{
			LED_LOOP2();
			Get_PID[0] = 0;
			Get_PID[1] = 0;
		}
		//若接收到PID1-6的数据，2灯轮流闪烁2次
		else if (Get_PID[2] == 1 && Get_PID[3] == 1)
		{
			LED_LOOP1();
			Get_PID[2] = 0;
			Get_PID[3] = 0;
		}
		//若处于锁定状态，流水灯
		else if (qState.Mode == LOCK)
		{
			LED_LOCK();
		}
		//若解锁姿态模式，四灯常亮
		else if (qState.Mode == RCMODE)
		{
			LED_NORMAL();
			vTaskDelay(100);
		}
		//若解锁定高模式，四灯闪烁
		else if (qState.Mode == HEIGHT)
		{
			LED_LOOP2();
		}
		LED_Reset();
	}
}



/*************************************************************
 *vANO串口通信解码，默认接收到的有效数据为18字节
 **************************************************************/
u8 nANO_RX_Frame_Decode(void)
{
	u8 *uBuf;
	uBuf = uANO_Rx_OK();
	if (uBuf != NULL)
	{
		E_FUC_MODE eFuc_M = *(uBuf + uANO_Head_Len - 2); //取出功能字
		u8 uBias = uANO_Head_Len;						  //数据开始长度
		s16 sTem[9] = {0};
		u8 uLen = 9;
		switch (eFuc_M)
		{
		case FUC_PID1:
		{
			Get_PID[0] = 1;
			// Pid参数设置
			vBuf_BtoI(sTem, uBuf, uBias, uLen);
			qRpid.fKp = sTem[0] * qRC_T.fB_PidS;
			qRpid.fKi = sTem[1] * qRC_T.fB_PidS;
			qRpid.fKd = sTem[2] * qRC_T.fB_PidS;
			qPpid.fKp = sTem[3] * qRC_T.fB_PidS;
			qPpid.fKi = sTem[4] * qRC_T.fB_PidS;
			qPpid.fKd = sTem[5] * qRC_T.fB_PidS;
			qYpid.fKp = sTem[6] * qRC_T.fB_PidS;
			qYpid.fKi = sTem[7] * qRC_T.fB_PidS;
			qYpid.fKd = sTem[8] * qRC_T.fB_PidS;
			break;
		}
		case FUC_PID2:
		{
			Get_PID[1] = 1;
			// Pid参数设置
			vBuf_BtoI(sTem, uBuf, uBias, uLen);
			qGRpid.fKp = sTem[0] * qRC_T.fB_PidS;
			qGRpid.fKi = sTem[1] * qRC_T.fB_PidS;
			qGRpid.fKd = sTem[2] * qRC_T.fB_PidS;
			qGPpid.fKp = sTem[3] * qRC_T.fB_PidS;
			qGPpid.fKi = sTem[4] * qRC_T.fB_PidS;
			qGPpid.fKd = sTem[5] * qRC_T.fB_PidS;
			qGYpid.fKp = sTem[6] * qRC_T.fB_PidS;
			qGYpid.fKi = sTem[7] * qRC_T.fB_PidS;
			qGYpid.fKd = sTem[8] * qRC_T.fB_PidS;
			break;
		}
		case FUC_PID3: //上位机修改PID参数Height、X轴、Y轴位置环
		{
			Get_PID[2] = 1;
			// Pid参数设置
			vBuf_BtoI(sTem, uBuf, uBias, uLen);
			qCZpid.fKp = sTem[0] * qRC_T.fB_PidS;
			qCZpid.fKi = sTem[1] * qRC_T.fB_PidS;
			qCZpid.fKd = sTem[2] * qRC_T.fB_PidS;
			qCXpid.fKp = sTem[3] * qRC_T.fB_PidS;
			qCXpid.fKi = sTem[4] * qRC_T.fB_PidS;
			qCXpid.fKd = sTem[5] * qRC_T.fB_PidS;
			qCYpid.fKp = sTem[6] * qRC_T.fB_PidS;
			qCYpid.fKi = sTem[7] * qRC_T.fB_PidS;
			qCYpid.fKd = sTem[8] * qRC_T.fB_PidS;

			break;
		}
		case FUC_PID4: //上位机修改PID参数Height、X轴、Y轴速度环
		{
			Get_PID[3] = 1;
			// Pid参数设置
			vBuf_BtoI(sTem, uBuf, uBias, uLen);
			qVZpid.fKp = sTem[0] * qRC_T.fB_PidS;
			qVZpid.fKi = sTem[1] * qRC_T.fB_PidS;
			qVZpid.fKd = sTem[2] * qRC_T.fB_PidS;
			qVXpid.fKp = sTem[3] * qRC_T.fB_PidS;
			qVXpid.fKi = sTem[4] * qRC_T.fB_PidS;
			qVXpid.fKd = sTem[5] * qRC_T.fB_PidS;
			qVYpid.fKp = sTem[6] * qRC_T.fB_PidS;
			qVYpid.fKi = sTem[7] * qRC_T.fB_PidS;
			qVYpid.fKd = sTem[8] * qRC_T.fB_PidS;
			break;
		}

		default:
			break;
		}
		return 1;
	}
	else
		return 0;
}

/*************************************************************
 *无人机NRF通信解码
 **************************************************************/
u8 nNRF_RX_Frame_Decode(void)
{
	u8 *uBuf = NULL;
	s16 sTem[20] = {0};

	uBuf = uNRF_Rx_OK();

	u8 sum = 0;
	if (uBuf != NULL)
	{
		// uLen接收的数据长度：uBuf[3]+4
		u8 uLen = *(uBuf + uNRF_Head_Len - 1) + uNRF_Head_Len;
		//功能字Buf[2]
		E_FUC_MODE eFuc_M = *(uBuf + uNRF_Head_Len - 2);
		//帧头2+功能字1+数据长度1=4
		u8 uBias = uNRF_Head_Len;
		for (u8 i = 0; i < uLen; i++)
			sum += *(uBuf + i);

		if (!(sum == uBuf[uLen]))
			return 0; //判断sum校验位
		if (!(*(uBuf) == 0xAA && *(uBuf + 1) == 0xAF))
			return 0; //判断帧头(遥控到无人机

		switch (eFuc_M)
		{
		case NRF_PID: //修改RPY角度环PID系数
		{
			Get_PID[0] = 1;
			// Pid参数设置
			vBuf_BtoI(sTem, uBuf, uBias, (uLen - uBias) / 2);
			qRpid.fKp = sTem[0] * qRC_T.fB_PidS;
			qRpid.fKi = sTem[1] * qRC_T.fB_PidS;
			qRpid.fKd = sTem[2] * qRC_T.fB_PidS;
			qPpid.fKp = sTem[3] * qRC_T.fB_PidS;
			qPpid.fKi = sTem[4] * qRC_T.fB_PidS;
			qPpid.fKd = sTem[5] * qRC_T.fB_PidS;
			qYpid.fKp = sTem[6] * qRC_T.fB_PidS;
			qYpid.fKi = sTem[7] * qRC_T.fB_PidS;
			qYpid.fKd = sTem[8] * qRC_T.fB_PidS;
			break;
		}
		case NRF_GPID: //修改RPY角速度环PID系数
		{
			Get_PID[1] = 1;
			// Pid参数设置
			vBuf_BtoI(sTem, uBuf, uBias, (uLen - uBias) / 2);
			qGRpid.fKp = sTem[0] * qRC_T.fB_PidS;
			qGRpid.fKi = sTem[1] * qRC_T.fB_PidS;
			qGRpid.fKd = sTem[2] * qRC_T.fB_PidS;
			qGPpid.fKp = sTem[3] * qRC_T.fB_PidS;
			qGPpid.fKi = sTem[4] * qRC_T.fB_PidS;
			qGPpid.fKd = sTem[5] * qRC_T.fB_PidS;
			qGYpid.fKp = sTem[6] * qRC_T.fB_PidS;
			qGYpid.fKi = sTem[7] * qRC_T.fB_PidS;
			qGYpid.fKd = sTem[8] * qRC_T.fB_PidS;
			break;
		}
		case FUC_RCDATA: //获得遥控值RPY thr
		{
			//期望的参考信号设置
			vBuf_BtoI(sTem, uBuf, uBias, (uLen - uBias) / 2);
			qRC_Key.sYaw = sTem[0];
			qRC_Key.sThr = sTem[1];
			qRC_Key.sRoll = sTem[2];
			qRC_Key.sPitch = sTem[3];
			qRC_Key.sAux1 = sTem[4];
			qRC_Key.sAux2 = sTem[5];
			qRC_Key.sAux3 = sTem[6];
			qRC_Key.sAux4 = sTem[7];
			qRC_Key.sAux5 = sTem[8];
			qRC_Key.sAux6 = sTem[9];
			break;
		}
		case MODE: //获得无人机的飞行模式
		{
			//期望的参考信号设置
			vBuf_BtoI(sTem, uBuf, uBias, (uLen - uBias) / 2);
			if (qState.Mode != LOCK)
			{
				if (sTem[0] == 1)
				{
					qState.Mode = HEIGHT;
				}
				else
				{
					qState.Mode = RCMODE;
				}
			}
			break;
		}
		default:
			printf("eFuc_M = %x Out\r\n", eFuc_M);
			break;
		}
		return 1;
	}
	else
		return 0;
}

/*************************************************************
 *NRF通信解码，默认接收到的有效数据为18字节
 **************************************************************/
void vNRF_TX_Frame_Decode(void)
{
	// 发送欧拉角等状态，
	u8 uHlen = sizeof(uU_FHead) / sizeof(uU_FHead[0]);
	u8 uCnt;
	float fEu_State[] = {qState.fEu[0], qState.fEu[1], qState.fEu[2], 0, 0, 0};
	u8 uLen = sizeof(fEu_State) / sizeof(fEu_State[0]);

	uCnt = uTx_FtoS16(uU_FHead, uHlen, FUC_STATUS, FCHECK_SUM, suTxBuf, fEu_State, uLen, qFtob.fF_EuS);
	uNRF_Frame_Send(suTxBuf, uCnt);

	//发送加速度计、陀螺仪、磁力计数据
	float fAGM_State[] = {qState.fAc[0] * qFtob.fF_AcS, qState.fAc[1] * qFtob.fF_AcS, qState.fAc[2] * qFtob.fF_AcS,
						  qState.fGy[0] * qFtob.fF_GyS, qState.fGy[1] * qFtob.fF_GyS, qState.fGy[2] * qFtob.fF_GyS,
						  qState.fMa[0] * qFtob.fF_MaS, qState.fMa[1] * qFtob.fF_MaS, qState.fMa[2] * qFtob.fF_MaS};

	uLen = sizeof(fAGM_State) / sizeof(fAGM_State[0]);
	uCnt = uTx_FtoS16(uU_FHead, uHlen, FUC_SENSOR, FCHECK_SUM, suTxBuf, fAGM_State, uLen, 1);
	uNRF_Frame_Send(suTxBuf, uCnt);

	//发送电量等数据
	float fPower[] = {qState.fVoltage, 0};
	uLen = sizeof(fPower) / sizeof(fPower[0]);
	uCnt = uTx_FtoS16(uU_FHead, uHlen, FUC_POWER, FCHECK_SUM, suTxBuf, fPower, uLen, 1);
	uNRF_Frame_Send(suTxBuf, uCnt);

	//将角速度PID发送到遥控器，对应上位机pid13 14 15
	float fRPY_Eu_Pid[] = {qRpid.fKp, qRpid.fKi, qRpid.fKd,
						   qPpid.fKp, qPpid.fKi, qPpid.fKd,
						   qYpid.fKp, qYpid.fKi, qYpid.fKd};

	uLen = sizeof(fRPY_Eu_Pid) / sizeof(fRPY_Eu_Pid[0]);
	uCnt = uTx_FtoS16(uU_FHead, uHlen, FUC_PID5, FCHECK_SUM, suTxBuf, fRPY_Eu_Pid, uLen, qFtob.fF_PidS);
	uNRF_Frame_Send(suTxBuf, uCnt);

	//将角度PID发送到遥控器，对应上位机pid16 17 18
	float fRPY_GEu_Pid[] = {qGRpid.fKp, qGRpid.fKi, qGRpid.fKd,
							qGPpid.fKp, qGPpid.fKi, qGPpid.fKd,
							qGYpid.fKp, qGYpid.fKi, qGYpid.fKd};

	uLen = sizeof(fRPY_GEu_Pid) / sizeof(fRPY_GEu_Pid[0]);
	uCnt = uTx_FtoS16(uU_FHead, uHlen, FUC_PID6, FCHECK_SUM, suTxBuf, fRPY_GEu_Pid, uLen, qFtob.fF_PidS);
	uNRF_Frame_Send(suTxBuf, uCnt);
}

/****************************************************************
 * Data_Exchange函数,无人机向上位机发送各种数据
 ****************************************************************/
void vANO_TX_Frame_Encode()
{
	// // 发送欧拉角等状态，
	u8 uHlen = sizeof(uU_FHead) / sizeof(uU_FHead[0]);
	u8 uCnt;
	float fEu_State[] = {qState.fEu[0], qState.fEu[1], qState.fEu[2], qState.fGy[0], 0};
	u8 uLen = sizeof(fEu_State) / sizeof(fEu_State[0]);

	uCnt = uTx_FtoS16z(FCHECK_SUM, suTxBuf, fEu_State, uLen, qFtob.fF_EuS);
//	uCnt = uTx_FtoS16(uU_FHead, uHlen, FUC_STATUS, FCHECK_SUM, suTxBuf, fEu_State, uLen, qFtob.fF_EuS);
	vUSART_Frame_Send(DEBUG_USARTx, suTxBuf, uCnt);

	//发送加速度计、陀螺仪、磁力计数据
	float fAGM_State[] = {100*qState.fAc[0] * qFtob.fF_AcS, 100*qState.fAc[1] * qFtob.fF_AcS, 100*qState.fAc[2] * qFtob.fF_AcS,
						  qState.fGy[0] * qFtob.fF_GyS, qState.fGy[1] * qFtob.fF_GyS, qState.fGy[2] * qFtob.fF_GyS,
						  qState.fMa[0] * qFtob.fF_MaS, qState.fMa[1] * qFtob.fF_MaS, qState.fMa[2] * qFtob.fF_MaS};

	uLen = sizeof(fAGM_State) / sizeof(fAGM_State[0]);
	uCnt = uTx_FtoS16(uU_FHead, uHlen, FUC_SENSOR, FCHECK_SUM, suTxBuf, fAGM_State, uLen, 1);
	vUSART_Frame_Send(DEBUG_USARTx, suTxBuf, uCnt);

	//发送电量等数据
	float fPower[] = {qState.fVoltage, 0};
	uLen = sizeof(fPower) / sizeof(fPower[0]);
	uCnt = uTx_FtoS16(uU_FHead, uHlen, FUC_POWER, FCHECK_SUM, suTxBuf, fPower, uLen, qFtob.fF_PwS);
	vUSART_Frame_Send(DEBUG_USARTx, suTxBuf, uCnt);

	//发送电机PWM数据
  float fPWM[] = {qPW.sM_PWM[0], qPW.sM_PWM[1], qPW.sM_PWM[2], qPW.sM_PWM[3], 0, 0, 0, 0};
	uLen = sizeof(fPWM) / sizeof(fPWM[0]);
	uCnt = uTx_FtoS16(uU_FHead, uHlen, FUC_MOTO, FCHECK_SUM, suTxBuf, fPWM, uLen, 1);
	vUSART_Frame_Send(DEBUG_USARTx, suTxBuf, uCnt);

	if (suANO_Tx_PID_Flag)
	{
		suANO_Tx_PID_Flag = 0;
		//将RPY角度环PID数据发送到上位机
		float fRPY_Eu_Pid[] = {qRpid.fKp, qRpid.fKi, qRpid.fKd,
							   qPpid.fKp, qPpid.fKi, qPpid.fKd,
							   qYpid.fKp, qYpid.fKi, qYpid.fKd};

		uLen = sizeof(fRPY_Eu_Pid) / sizeof(fRPY_Eu_Pid[0]);
		uCnt = uTx_FtoS16(uU_FHead, uHlen, FUC_PID1, FCHECK_SUM, suTxBuf, fRPY_Eu_Pid, uLen, qFtob.fF_PidS);
		vUSART_Frame_Send(DEBUG_USARTx, suTxBuf, uCnt);

		//将RPY速度环PID发送到上位机
		float fRPY_GEu_Pid[] = {qGRpid.fKp, qGRpid.fKi, qGRpid.fKd,
								qGPpid.fKp, qGPpid.fKi, qGPpid.fKd,
								qGYpid.fKp, qGYpid.fKi, qGYpid.fKd};

		uLen = sizeof(fRPY_GEu_Pid) / sizeof(fRPY_GEu_Pid[0]);
		uCnt = uTx_FtoS16(uU_FHead, uHlen, FUC_PID2, FCHECK_SUM, suTxBuf, fRPY_GEu_Pid, uLen, qFtob.fF_PidS);
		vUSART_Frame_Send(DEBUG_USARTx, suTxBuf, uCnt);

		//将位置环PID数据发送到上位机
		float fCVP_Pid[] = {qCZpid.fKp, qCZpid.fKi, qCZpid.fKd,
							 qCXpid.fKp, qCXpid.fKi, qCXpid.fKd,
							 qCYpid.fKp, qCYpid.fKi, qCYpid.fKd};

		uLen = sizeof(fCVP_Pid) / sizeof(fCVP_Pid[0]);
		uCnt = uTx_FtoS16(uU_FHead, uHlen, FUC_PID3, FCHECK_SUM, suTxBuf, fCVP_Pid, uLen, qFtob.fF_PidS);
		vUSART_Frame_Send(DEBUG_USARTx, suTxBuf, uCnt);

		//将速度PID发送到上位机
		float fCVel_Pid[] = {qVZpid.fKp, qVZpid.fKi, qVZpid.fKd,
							 qVXpid.fKp, qVXpid.fKi, qVXpid.fKd,
							 qVYpid.fKp, qVYpid.fKi, qVYpid.fKd};

		uLen = sizeof(fCVel_Pid) / sizeof(fCVel_Pid[0]);
		uCnt = uTx_FtoS16(uU_FHead, uHlen, FUC_PID4, FCHECK_SUM, suTxBuf, fCVel_Pid, uLen, qFtob.fF_PidS);
		vUSART_Frame_Send(DEBUG_USARTx, suTxBuf, uCnt);
	}
}

/****************************************************************************************
 *带数据长度的发送：发送float类型的帧，帧头，帧头长度，校验类型，字节数组地址，float数组地址，长度，返回字节数目
 ******************`**********************************************************************/
u8 uTx_FtoS32(u8 *uHead, u8 uH_Len, E_FUC_MODE uFuc, E_FRAME_CHECK uFrameCheck, u8 *uData, float *fData, u8 uLen, float fScale)
{
	u8 i;
	u8 uCnt;
	float fTem;
	for (i = 0; i < uH_Len; i++)
	{
		uData[i] = *(uHead + i);
	}
	uData[uH_Len] = uFuc;
	uData[uH_Len + 1] = 4 * uLen; //发送字节长度
	uCnt = uH_Len + 2;
	for (i = 0; i < uLen; i++)
	{
		fTem = *(fData + i) * fScale;
		uData[4 * i + uCnt] = BYTE3(fTem);
		uData[4 * i + uCnt + 1] = BYTE2(fTem);
		uData[4 * i + uCnt + 2] = BYTE1(fTem);
		uData[4 * i + uCnt + 3] = BYTE0(fTem);
	}
	uCnt = uCnt + uData[uH_Len + 1];
	if (uFrameCheck == FCHECK_SUM)
	{
		u8 uSum = 0;
		for (i = 0; i < uCnt; i++)
			uSum += uData[i];
		uData[uCnt] = uSum;
		uCnt = uCnt + 1;
	}
	return uCnt;
}


/****************************************************************************************
*发送符号16整数类型的帧：uHead帧头，uH_Len帧头长度，uFuc功能字，uData字节数组地址，fData数组地址，uLen长度，fScale变换因子子
	返回字节数目
****************************************************************************************/
u8 uTx_FtoS16(u8 *uHead, u8 uH_Len, E_FUC_MODE uFuc, E_FRAME_CHECK uFrameCheck, u8 *uData, float *fData, u8 uLen, float fScale)
{

	u8 i;
	u8 uCnt;
	vs16 fTem;
	for (i = 0; i < uH_Len; i++)
	{
		uData[i] = *(uHead + i);
	}
	uData[uH_Len] = uFuc;
	uData[uH_Len + 1] = 2 * uLen; //发送字节长度
	uCnt = uH_Len + 2;
	for (i = 0; i < uLen; i++)
	{
		fTem = (s16)((*(fData + i)) * fScale);

		uData[2 * i + uCnt] = BYTE1(fTem);
		uData[2 * i + uCnt + 1] = BYTE0(fTem);
	}

	uCnt = uCnt + uData[uH_Len + 1];
	if (uFrameCheck == FCHECK_SUM)
	{
		u8 uSum = 0;
		for (i = 0; i < uCnt; i++)
			uSum += uData[i];
		uData[uCnt] = uSum;
		uCnt = uCnt + 1;
	}

	return uCnt;
}


u8 uTx_FtoS16z( E_FRAME_CHECK uFrameCheck, u8 *uData, float *fData, u8 uLen, float fScale)
{

	u8 i;
	u8 uCnt=0;
	vs16 fTem;
	vs32 fTem2;

		uData[uCnt++] =0xAA;
    uData[uCnt++] =0xAA;
	  uData[uCnt++] = 0X01;//AAAA01，udata[2]=01
	  uData[uCnt++] =12; //发送字节长度12
	uCnt = 4;//4
	for (i = 0; i < 3; i++)
	{
		fTem = (s16)((*(fData + i)) * fScale);

		uData[2 * i + 4] = BYTE1(fTem);
		uData[2 * i + 5] = BYTE0(fTem);
	}
	uCnt=10;

		fTem2 = *(fData + 3)*100 ;
		uData[10] = BYTE3(fTem2);
		uData[11] = BYTE2(fTem2);
		uData[12] = BYTE1(fTem2);
		uData[13] = BYTE0(fTem2);

	  fTem2 = *(fData + 4) ;
		uData[14] = BYTE3(fTem2);
		uData[15] = BYTE2(fTem2);
		
	
	uCnt = 16;
	if (uFrameCheck == FCHECK_SUM)
	{
		u8 uSum = 0;
		for (i = 0; i < uCnt; i++)
			uSum += uData[i];
		uData[uCnt] = uSum;
		uCnt = uCnt + 1;
	}

	return uCnt;
}

