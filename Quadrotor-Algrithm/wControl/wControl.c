/*******************************************************************************
	@file    wControl.c
	@version V1.0
	@date    2022.03.04
*******************************************************************************/
#include <math.h>
#include "wSystem.h"
#include "wI2C.h"
#include "wSysTick_Delay.h"
#include "wMPU6050.h"

#include "wCommonAlg.h"
#include "wEularQ.h"
#include "wOpticFlow.h"
#include "wEstimation.h"
#include "wControl.h"


extern QUAV_STATE_P qState;
extern QUAV_CTRL_SIG qRC_Sig; //遥控器参考信号
extern QUAV_RC_T qRC_T;//遥控器变换因子 


static bool bPZ_Adjust = 0;/*调整Z位置*/
static bool bKey_Fly = 1;/*调整Z位置*/
static bool bLock_Fly = 1;/*调整Z位置*/
static float fRef_Lp_P = 0.2; //低通滤波参数

//低通滤波后的遥控器参考信号


//电机PWM参数存储
static s16 sM_PWM[4]={0};

//自主下降模式，位移及速度参考信号
QUAV_REF_SIG qCtr_Ref = {
	.fP[0] = 0,
	.fP[1] = 0,
	.fP[2] = 1,//定高最低高度
	.fPErr[2]=0,
	.fV[0] = 0, //速度
	.fV[1] = 0,
	.fV[2] = -0.1,//下降参考速度
	.fThrust=0,
	.fRoll=0,
	.fPitch=0,
	.fYaw=0
};

// 参考信号笛卡尔坐标下的xyz位置设置

/*************************************************************
 *欧拉角PId参数设置
 **************************************************************/
// Rol角度环pid设置
CTRL_PID_P qRpid = {
	.fKp = 5.0,
	.fKi = 0,
	.fKd = 0,
	.fDt = 1.0f / CTRL_TASK_RATE,
	.fE_Pre = 0,
	.fInt_Lim = 500

};

// Pit角度环pid设置
CTRL_PID_P qPpid = {
	.fKp = 5.0,
	.fKi = 0,
	.fKd = 0,
	.fDt = 1.0f / CTRL_TASK_RATE,
	.fE_Pre = 0,
	.fInt_Lim = 500

};

// Yaw角度环pid设置
CTRL_PID_P qYpid = {
	.fRef = 0,
	.fKp = 7.0,
	.fKi = 0,
	.fKd = 0,
	.fDt = 1.0f / CTRL_TASK_RATE,
	.fE_Pre = 0,
	.fInt_Lim = 50

};

/*************************************************************
 *欧拉角角速度PId参数设置
 **************************************************************/

// Rol角速度环pid设置
CTRL_PID_P qGRpid = {

	.fKp = 0.8f,
	.fKi = 0.5,
	.fKd = 0.06,
	.fDt = 1.0f / CTRL_TASK_RATE,
	.fE_Pre = 0,
	.fInt_Lim = 100

};

// Pit角速度环pid设置
CTRL_PID_P qGPpid = {
	.fKp = 0.8f,
	.fKi = 0.5,
	.fKd = 0.06,
	.fDt = 1.0f / CTRL_TASK_RATE,
	.fE_Pre = 0,
	.fInt_Lim = 100

};

// Yaw角速度环pid设置
CTRL_PID_P qGYpid = {
	.fKp = 8.0f,
	.fKi = 4.0f,
	.fKd = 0.1,
	.fDt = 1.0f / CTRL_TASK_RATE,
	.fE_Pre = 0,
	.fInt_Lim = 500

};

/*************************************************************
 *世界坐标系下位置xyzPId参数设置
 **************************************************************/

// 世界坐标系下x位置pid设置
CTRL_PID_P qCXpid = {
	.fKp = 1,
	.fKi = 0,
	.fKd = 0,
	.fDt = 1.0f / CTRL_TASK_RATE,
	.fE_Pre = 0,
	.fInt_Lim = 100};

// 世界坐标系下y位置pid设置
CTRL_PID_P qCYpid = {
	.fKp = 1,
	.fKi = 0,
	.fKd = 0,
	.fDt = 1.0f / CTRL_TASK_RATE,
	.fE_Pre = 0,
	.fInt_Lim = 100

};

// 世界坐标系下z位置pid设置
CTRL_PID_P qCZpid = {
	.fRef = 1,
	.fKp = 1.2,
	.fKi = 0,
	.fKd = 0.085,
	.fDt = 1.0f / CTRL_TASK_RATE,
	.fE_Pre = 0,
	.fInt_Lim = 100

};

/*************************************************************
 *世界坐标系下xyz速度PId参数设置
 **************************************************************/


// 世界坐标系下x速度pid设置
CTRL_PID_P qVXpid = {
	.fKp = 0.045,
	.fKi = 0,
	.fKd = 0,
	.fDt = 1.0f / CTRL_TASK_RATE,
	.fE_Pre = 0,
	.fInt_Lim = 100

};

// 世界坐标系下y速度pid设置
CTRL_PID_P qVYpid = {
	.fKp = 0.045,
	.fKi = 0,
	.fKd = 0,
	.fDt = 1.0f / CTRL_TASK_RATE,
	.fE_Pre = 0,
	.fInt_Lim = 100

};

// 世界坐标系下z速度pid设置
CTRL_PID_P qVZpid = {
	.fKp = 1.2,
	.fKi = 0.04f,
	.fKd = 0.085f,
	.fDt = 1.0f / CTRL_TASK_RATE,
	.fE_Pre = 0,
	.fInt_Lim = 100

};


/**************************************************************************
	vCtrl_Task任务，串级PID，控制无人机飞行
	此任务以200hz运行
**************************************************************************/
void vCtrl_Task(void *pvParameters)
{
	u32 uLastWakeTime = uGetSysTickCnt();
	u32 uCnt = 1;//用于分频

	
	while (1)
	{
		
		//目标姿态和飞行模式设定（100Hz）	
		if (EXECUTE_RATE(RC_UPDATE_RATE, uCnt))
		{
			
			vRC_Ctrl_Ref_Set();//	
					
		}
		vPid_Ctrl();
		uCnt++;
		//此任务以200Hz的频率运行
		vTaskDelayUntil(&uLastWakeTime, F2T(CTRL_TASK_RATE));   
	}
}


/**************************************************************************
	pid控制，不同模式下的控制不一样
**************************************************************************/

vPid_Ctrl(void)
{

	//目标姿态和飞行模式设定（100Hz）	
	switch(qState.Mode)
	{
		case HEIGHT:
		{
			vPid_Ref_Set(&qRpid.fRef, qCtr_Ref.fRoll);//参考信号设置
			vPid_Ref_Set(&qPpid.fRef, qCtr_Ref.fPitch);
			vPid_Ref_Set(&qYpid.fRef, qCtr_Ref.fYaw);
			if(bKey_Fly) //一键起飞
			{
				vPid_Ref_Set(&qCZpid.fRef, qCtr_Ref.fP[2]);
				//z方向位置pid
				vPID_Control(&qCZpid, qCZpid.fRef - qState.fPo[2]);
				vPID_Control(&qVZpid, qCZpid.fOut - qState.fVe[2]); // qCYpid.fOut - qState.fVe[1]

			}
			else //一键降落
			{
				vPid_Ref_Set(&qVZpid.fRef, qCtr_Ref.fV[2]);
				//z方向速度pid
				vPID_Control(&qVZpid, qVZpid.fRef - qState.fVe[2]); // qCYpid.fOut - qState.fVe[1]

			}

			vAng_Ctrl(qVZpid.fOut);
			bLock_Fly =0;
			break;
		}
			
		case RCMODE:
		{
			vPid_Ref_Set(&qRpid.fRef, qCtr_Ref.fRoll);//参考信号设置
			vPid_Ref_Set(&qPpid.fRef, qCtr_Ref.fPitch);
			vPid_Ref_Set(&qYpid.fRef, qCtr_Ref.fYaw);
			vAng_Ctrl(qCtr_Ref.fThrust);
			bLock_Fly =0;
			break;
		}
				
		case LOCK:
		{
			if(!bLock_Fly)
			{
				vLock_Ctrl();
				bLock_Fly=1; //状态机置位
			}	
			break;
		}
		default:
		{			
			bLock_Fly=1;
			break;
		}
	}
}


/**************************************************************************
	参考信号设置，不同模式下的控制不一样
**************************************************************************/

vRC_Ctrl_Ref_Set(void)
{
	switch(qState.Mode)
	{
		case HEIGHT:
		{	
								
			qCtr_Ref.fThrust=0;
			qCtr_Ref.fPitch=0;
			qCtr_Ref.fRoll=0;
			qCtr_Ref.fYaw=0;

			if(qRC_Sig.fThrust>MAX_THRUST*0.7)//比较大的动力，一键起飞
			{	
				bKey_Fly=1;//一键起飞标志

				if(qState.fPo[2]< 1)
					qCtr_Ref.fP[2]=1;//定高1m
				else if(bPZ_Adjust)//调整参考信号
				{
					bPZ_Adjust=0;
					qCtr_Ref.fP[2]= qState.fPo[2]+qCtr_Ref.fPErr[2];
									
				}
				else if(!bPZ_Adjust)
				{
					bPZ_Adjust=1;
					qCtr_Ref.fPErr[2]=qCtr_Ref.fP[2]-qState.fPo[2];
					qCtr_Ref.fPErr[2]=fThreshold(qCtr_Ref.fPErr[2], -0.1, 0.1);//误差0.1m
				}
									
			}
			if(qRC_Sig.fThrust<MIN_THRUST*0.2)//比较小的动力，一键降落
			{
				bKey_Fly = 0;//一键降落标志
				qCtr_Ref.fV[2]=-1;
				if(qState.fPo[2]<1)
					qCtr_Ref.fV[2]=-0.25;
							
			}
			
			break;
							
		}
		case RCMODE:
		{
			//目标姿态和飞行模式设定（100Hz）
			vBut_LPF_First(&qCtr_Ref.fThrust, &qRC_Sig.fThrust, &fRef_Lp_P, 1);
			vBut_LPF_First(&qCtr_Ref.fPitch, &qRC_Sig.fPitch, &fRef_Lp_P, 1);
			vBut_LPF_First(&qCtr_Ref.fRoll, &qRC_Sig.fRoll, &fRef_Lp_P, 1);
			vBut_LPF_First(&qCtr_Ref.fYaw, &qRC_Sig.fYaw, &fRef_Lp_P, 1);

			if (qCtr_Ref.fThrust< MIN_THRUST)
				qCtr_Ref.fThrust = 0;	
			else 		
				qCtr_Ref.fThrust = (qCtr_Ref.fThrust >= MAX_THRUST) ? MAX_THRUST:qCtr_Ref.fThrust;
					
			break;
		}

		case LOCK:
			break;
		default:
			break;
	}

}





/**************************************************************************
	位置pid控制
**************************************************************************/			

void vPos_Ctrl(void)
{
	QUAV_CTRL_SIG qCtrl_Input; //存储pid输出的控制信号
	//位置pid
	vPID_Control(&qCXpid, qCXpid.fRef - qState.fPo[0]);
	vPID_Control(&qCYpid, qCYpid.fRef - qState.fPo[1]);
	vPID_Control(&qCZpid, qCZpid.fRef - qState.fPo[2]);

	//速度pid
	vPID_Control(&qVXpid, qCXpid.fOut - qState.fVe[0]); // qCXpid.fOut - qState.fVe[0]
	vPID_Control(&qVYpid, qCYpid.fOut - qState.fVe[1]); // qCYpid.fOut - qState.fVe[1]
	vPID_Control(&qVZpid, qCZpid.fOut - qState.fVe[2]); // qCYpid.fOut - qState.fVe[1]

	//角度环
	vPID_Control(&qRpid, (qVYpid.fOut - qState.fEu[0])); // Roll角度环y轴速度pid输出
	vPID_Control(&qPpid, (qVXpid.fOut - qState.fEu[1])); // Pitch角度环x轴速度pid输出
	vPID_Control(&qYpid, (qYpid.fRef  - qState.fEu[2])); // Yaw角度环,直传控制量

	//角速度环,误差用度刻画

	vPID_Control(&qGRpid, (qRpid.fOut - qState.fGy[0])); // Roll角速度环
	vPID_Control(&qGPpid, (qPpid.fOut - qState.fGy[1])); // Pitch角速度环
	vPID_Control(&qGYpid, (qYpid.fOut - qState.fGy[2])); // Yaw角速度环

	//pid输出得到控制输入
	qCtrl_Input.fThrust = qVZpid.fOut;
	qCtrl_Input.fRoll = qGRpid.fOut;
	qCtrl_Input.fPitch = qGPpid.fOut;
	qCtrl_Input.fYaw = qGYpid.fOut;
	//将控制输入转为pwm的输出
	vPower_PWM_Set(&qCtrl_Input, sM_PWM);

}





/**************************************************************************
角度pid控制
**************************************************************************/			

void vAng_Ctrl(float fThrust)
{

	QUAV_CTRL_SIG qCtrl_Input; //存储pid输出的控制信号

	//角度环
	vPID_Control(&qRpid, (qRpid.fRef - qState.fEu[0])); // Roll角度环y轴速度pid输出
	vPID_Control(&qPpid, (qPpid.fRef - qState.fEu[1])); // Pitch角度环x轴速度pid输出
	vPID_Control(&qYpid, (qYpid.fRef  - qState.fEu[2])); // Yaw角度环,直传控制量

	//角速度环,误差用度刻画

	vPID_Control(&qGRpid, (qRpid.fOut - qState.fGy[0])); // Roll角速度环
	vPID_Control(&qGPpid, (qPpid.fOut - qState.fGy[1])); // Pitch角速度环
	vPID_Control(&qGYpid, (qYpid.fOut - qState.fGy[2])); // Yaw角速度环

	//pid输出得到控制输入
	qCtrl_Input.fThrust = fThrust;
	qCtrl_Input.fRoll = qGRpid.fOut;
	qCtrl_Input.fPitch = qGPpid.fOut;
	qCtrl_Input.fYaw = qGYpid.fOut;
	//将控制输入转为pwm的输出
	vPower_PWM_Set(&qCtrl_Input, sM_PWM);
					
}



/******************************************
 * LOCK模式
 ******************************************/

void vLock_Ctrl(void)
{
	vCtrl_Par_Reset();//pwm控制参数复位
	vEst_Par_Reset();//估计参数复位
}


/**************************************************************************
pwm输出，依赖于电机0-电机3的分布，硬件相关
**************************************************************************/	
void vPower_PWM_Set(const QUAV_CTRL_SIG * qIn, s16 *sM_PWM)
{
	s16 sPwm[4];
	float fR = qIn->fRoll;
    float fP = qIn->fPitch;
	sPwm[0]=(s16)(qIn->fThrust + fR - fP + qIn->fYaw);//参考硬件电路，电机1234的位置
	sPwm[1]=(s16)(qIn->fThrust + fR + fP - qIn->fYaw);
	sPwm[2]=(s16)(qIn->fThrust - fR + fP + qIn->fYaw);
	sPwm[3]=(s16)(qIn->fThrust - fR - fP - qIn->fYaw);
	
	sM_PWM[0] = sThreshold(sPwm[0],0,MOTOR_PWM_LIM);//pwm限幅输出，为0-sM_PWMmax
	sM_PWM[1] = sThreshold(sPwm[1],0,MOTOR_PWM_LIM);
	sM_PWM[2] = sThreshold(sPwm[2],0,MOTOR_PWM_LIM);
	sM_PWM[3] = sThreshold(sPwm[3],0,MOTOR_PWM_LIM);

	vM_PWM_Set(sM_PWM);

}



/******************************************
 * 控制参数初始化
 ******************************************/

void vCtrl_Par_Reset(void)
{
	//pid参数复位
	vPid_Par_Reset(&qRpid);
	vPid_Par_Reset(&qPpid);
	vPid_Par_Reset(&qYpid);
	vPid_Par_Reset(&qGRpid);
	vPid_Par_Reset(&qGPpid);
	vPid_Par_Reset(&qGYpid);

	vPid_Par_Reset(&qCXpid);
	vPid_Par_Reset(&qCYpid);
	vPid_Par_Reset(&qCZpid);
	vPid_Par_Reset(&qVXpid);
	vPid_Par_Reset(&qVYpid);
	vPid_Par_Reset(&qVZpid);

	//pwm相关复位
  	sM_PWM[0]=0;
	sM_PWM[1]=0;
	sM_PWM[2]=0;
	sM_PWM[3]=0;

	//遥控参考信号复位
	qCtr_Ref.fPitch=0;
	qCtr_Ref.fRoll=0;
	qCtr_Ref.fThrust=0;
	qCtr_Ref.fYaw=0;

	//pwm输出置0
	vM_PWM_Set(sM_PWM);

}

// /******************************************
//  * 状态机置位，用于不同控制模式
//  ******************************************/

// void vMode_Machine(u8 * uF, u8 uPos,u8 uLen)
// {
// 	u8 i;
// 	for (i=0;i<uLen;i++)
// 	{
// 		if(i==uPos)
// 		{
// 			uF[i]=1;//置位
// 		}
// 		else
// 		{
// 			uF[i]=0;
// 		}
// 	}
// }
