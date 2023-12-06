/**************************************************************************
    @file    wControl.h
    @version V1.0
    @date    2022.02.25
**************************************************************************/

#ifndef __WCONTROL_H__
#define __WCONTROL_H__

#include <stdbool.h>
#include <math.h>
#include "stm32f10x.h"
#include "wCommonAlg.h"






#define CTRL_TASK_RATE RATE_400_HZ

#define EXECUTE_RATE(RATE, CNT)   ((CNT % (CTRL_TASK_RATE / RATE)) == 0)

#define RC_UPDATE_RATE  CTRL_TASK_RATE/4 //位置环（外环）PID速率

#define MAX_THRUST 900
#define MIN_THRUST 100
#define MID_THRUST 500



// typedef struct
// {
// 	u8 uKeyFlight 	: 1;	/*bit0 一键起飞*/
// 	u8 uKeyLand 	: 1;	/*bit1 一键降落*/
// 	u8 uReserved	: 6;	/*bit2~7 保留*/
// }QUAV_CTRL_BIT;


//不同模式参考信号参数或门限
typedef struct
{
	float fP[3]; //三轴位置参考
    float fPErr[3];//三轴位置误差
    float fV[3]; //三轴速度参考
    float fVErr[3]; //三轴速度误差
    float fThrust=0;
	float fRoll=0;
	float fPitch=0;
	float fYaw=0;

} QUAV_REF_SIG; 


/**************************************************************************
    vFlight_Control_Task任务，无人机飞控任务
    此任务以100hz运行
**************************************************************************/
void vCtrl_Task(void *pvParameters);



/**************************************************************************
	pid控制，不同模式下的控制不一样
**************************************************************************/

vPid_Ctrl(void);

/**************************************************************************
	参考信号设置，不同模式下的控制不一样
**************************************************************************/

vRC_Ctrl_Ref_Set(void);

/**************************************************************************
	位置pid控制
**************************************************************************/			
void vPos_Ctrl(void);


/**************************************************************************
角度pid控制
**************************************************************************/			
void vAng_Ctrl(float fThrust)

		
/**************************************************************************
 * LOCK模式下初始化
 **************************************************************************/
void vLock_Ctrl(void);


/**************************************************************************
pwm输出，依赖于电机1-电机4的分布，硬件相关，
根据电路 上边 左1，右2，下边左3，右4
roll的话1,3为正，pitch的话0,1为正，yaw的话1,4为正，2,3为负
**************************************************************************/	
void vPower_PWM_Set(const QUAV_CTRL_SIG * qIn,  s16 *sM_PWM);

/******************************************
 * pid等控制参数初始化
 ******************************************/
void vCtrl_Par_Reset(void);

// /******************************************
//  * 状态机置位
//  ******************************************/
// void vMode_Machine(u8 * uF, u8 uPos,u8 uLen);

#endif