#ifndef __WESTIMATION_H__
#define __WESTIMATION_H__

#include "stm32f10x.h"
/******************************wEstimation.c需要用的宏*******************************/
#ifndef RAD2DEG
#define RAD2DEG        57.29578f  // 360/2pi,  弧度到角度转换
#endif

#ifndef CONTROL_DELAY
#define CONTROL_DELAY 500 //主要通过统计平均1000次求解加速度计及陀螺测量偏差，用于校正
#endif

#ifndef BLPF_CUT_F
#define BLPF_CUT_F 30.0f //低通滤波器截止频率
#endif


//任务执行速率
#define EST_AG_TASK_RATE   RATE_400_HZ
#define EST_PTH_TASK_RATE  RATE_25_HZ
#define EST_VP_TASK_RATE  RATE_200_HZ
#define EST_OFXY_TASK_RATE  RATE_20_HZ



/******************************结构体定义******************************/

//互补滤波器的参数结构体
typedef struct
{
    float fKp;
    float fKi;
    float fDt; //步长
    float fErr[3];//互补滤波积分误差；
} QUAV_CF_P;

//飞行模式切换命令
enum FlightMode
{
    LOCK = 0x00, //锁定模式
    RCMODE,      //遥控模式
    HEIGHT,      //定高模式
    FALL,  //下降模式
    // Flow_POSITION,   //GPS定点模式
};

//无人机姿态等参数结构体
typedef struct
{
    float fAc[3];         //传感器输出三轴加速度
    float fEAc[3];        //世界坐标系下加速度对应xyz m/s²
    float fGy[3];         //角速度对应弧度
    float fMa[3];         //磁力计输出，没有磁力计，默认为0
    float fEu[3];         //欧拉角对应Roll,Pitch,Yaw.度表示
    float fPo[3];         //世界坐标系位移xyz（相对高度）m
    float fVe[3];         //绝对速度xyz，m/s

    float fPTH[3];        //气压，温度，海拔
    enum FlightMode Mode; //飞行模式

    uint8_t uFly_F; //姿态完成初始化标志，起飞标志
    float fVoltage; //电池电量
} QUAV_STATE_P;


//无人机加速度角速度等滤波器参数结构体
typedef struct
{
    float fCutoff[3];      //滤波器截止频率hz
    float fDeadband[3];    //静止时波动范围
    float fDt; //采样频率
    float fFP[3];//低通滤波器参数（待求）
} QUAV_SENSOR_F_P;



/**************************************************************************
    vALT_AG__Task函数，获取加速度输出，角速度，输出，欧拉角
**************************************************************************/
void vEst_AG_Task(void *pvParameters);

/**************************************************************************
    vEST_PTH__Task函数，获取温度，气压，高度
**************************************************************************/
void vEst_PTH_Task(void *pvParameters);

/**************************************************************************
    vEst_OFXY_Task任务，获取xy轴位移及速度m/s
**************************************************************************/
void vEst_OFXY_Task(void *pvParameters);
/**************************************************************************
    vEst_VP_Task任务，地面坐标系下，速度V与位置P估计与融合，后续结合卡尔曼滤波做
**************************************************************************/
void vEst_VP_Task(void *pvParameters);

/**************************************************************************
    姿态参数初始化
**************************************************************************/
void vqState_Init(void);

/**************************************************************************
姿态参数复位
**************************************************************************/
void vEst_Par_Reset(void);

/**************************************************************************
完整约束二阶动力学方程，输入为位移，速度，采样周期，加速度
**************************************************************************/
void vPV_Dynamics(float * fP,float *fV, float fDt, float fAcc);
/**************************************************************************
利用传感器数据对位置及速度预测值进行校正，输入为位移，速度，采样周期，误差，权重
本质还是低通滤波；
**************************************************************************/
void vPV_Correct(float * fP,float *fV, float fDt, float fErr,float fWeight);

#endif
