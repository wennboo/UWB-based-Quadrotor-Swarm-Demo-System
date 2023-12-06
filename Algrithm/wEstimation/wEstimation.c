/**************************************************************************
    @file    wEstimation.c 实现传感器的读取及相关状态估计
    @version V2.0
    @date    2023.3.21


    载体坐标系选用东北天(ENU)，东为x，北为y，天为z，北向为机头
    x轴：俯仰角，y轴：横滚角，z轴：偏航角
    大地坐标系n，	载体坐标系b
    Un绕ZYX轴旋转得到Ub
    Ub = Cx * Cy * Cz * Un，
    Un为大地坐标系下的坐标.
         [ 1     0   0  ]
    Cx = [ 0   cosφ sinφ]
         [ 0  -sinφ cosφ]

        [cosθ   0 -sinθ]
    Cy = [ 0     1   0  ]
         [sinθ   0  cosθ]

         [cosψ -sinψ  0	]
    Cz = [sinψ  cosψ  0 ]
         [ 0     0    1 ]

    大地坐标系到载体坐标系的方向余弦矩阵
    φ:roll θ:pitch ψ:yaw
                        
    Cn->b =	Cx*Cy*Cz 

    Cb->n = (cn->b)T
**************************************************************************/
#include <math.h>
#include "wSystem.h"
#include "wI2C.h"
#include "wSysTick_Delay.h"
#include "wMPU6050.h"
#include "wCommonAlg.h"
#include "wEularQ.h"
#include "wOpticFlow.h"
#include "wEstimation.h"

QUAV_STATE_P qState; //无人机姿态参数

static u16 suAG_Bias_Cnt;   //控制启动时间前计数,全局变量默认值为0，用于姿态校正
static float sfQuat[4] = {1,0,0,0};      //四元数输出
// static float sfAcc_LP_P[3],sfGy_LP_P[3],sfPTH_LP_P[3];       //一阶低通滤波参数，x+=(y-x)sfAcc_LP_P，统计平均求前十次平均

static short ssGy[3], ssAcc[3];        //, sMag[3];	//陀螺仪值、机体坐标系下加速度值与磁力计值
static float sfGy[3], sfAcc[3],fPTH[3],fMag[3]={0,0,0}; //, fMag[3];//转化后的陀螺仪、机体坐标系下加速度计和磁力计的输出、压力温度海拔输出
static float sfVxy_OF[2], sfPxy_OF[2]; // 光流输出xy位置和速度
static float sfGy_D1[3], sfAcc_D1[3]; //, fMag_D1[3];		//转化后的陀螺仪及加速度计上一刻的输出
static float sfGy_D2[3], sfAcc_D2[3]; //, fMag_D2[3];		//转化后的陀螺仪及加速度计的上一两刻输出
static float sfAlt_Bias; //起飞前的海拔
 
static float sfBF_P[5];     //二阶Butterworth滤波器参数

static float sfAcc_Bias[3], sfGy_Bias[3];

//互补滤波器控制参数
static QUAV_CF_P qCf = {
    .fKp = 1.0f,
    .fKi = 0.05f,
    .fDt = 1.0f / EST_AG_TASK_RATE,
    .fErr[0]=0 ,
	.fErr[1]=0 ,
    .fErr[2]=0 ,
};

//传感器滤波器参数
static QUAV_SENSOR_F_P qAcc_FP = {
    .fCutoff[0]=30, //加速度截止频率
	.fCutoff[1]=30,
	.fCutoff[2]=30,
    .fDeadband[0]=0.02, //加速度扰动门限截止频率m/s^2
	.fDeadband[1]=0.02,
	.fDeadband[2]=0.02,
    .fDt = 1.0f / EST_AG_TASK_RATE  //加速度采样频率
};


static QUAV_SENSOR_F_P qGy_FP = {
    .fCutoff[0]=30, //角速度截止频率
	.fCutoff[1]=30,
	.fCutoff[2]=30,
    .fDeadband[0]=0.02, //角速度扰动门限截止频率m/s^2
	.fDeadband[1]=0.02,
	.fDeadband[2]=0.02,
    .fDt = 1.0f / EST_AG_TASK_RATE  //角速度采样频率
};

static QUAV_SENSOR_F_P qPTH_FP = {
    .fCutoff[0]=30, //气压温度海拔加速度截止频率
	.fCutoff[1]=30,
	.fCutoff[2]=30,
    .fDeadband[0]=0.02, //气压温度海拔扰动门限截止频率m/s^2
	.fDeadband[1]=0.02,
	.fDeadband[2]=0.35,
    .fDt = 1.0f / EST_PTH_TASK_RATE  //气压温度海拔采样频率
};

/**************************************************************************
    vEst_AG_Task任务，获取加速度输出，角速度，输出，欧拉角
**************************************************************************/

void vEst_AG_Task(void *pvParameters)
{
    u32 uLastWakeTime = uGetSysTickCnt();

    float fStep; //, fG;  //后者存储机体坐标系下的加速度

    while (1)
    {
        //开机前,读取陀螺仪零点
        uMPU6050_Get_Acc(ssAcc); //通过IIC读取加速度信息
        uMPU6050_Get_Gyro(ssGy); //通过IIC读取角速度信息   
        //将读取加速度及角速度转换成真实值
        vGet_Gy_Acc(ssAcc, ssGy, sfAcc, sfGy);
        //求初始时刻姿态的偏差，用于对测量值进行校正
        if (!qState.uFly_F)
        {
            suAG_Bias_Cnt++;
            fStep = 1.0f / suAG_Bias_Cnt;
            //统计平均求测量偏差g在机体坐标下的投影，理想情况下sfAcc[2]=g;
//            vLarge_Num_F(qState.fAc, sfAcc, fStep, 3); //统计平均求测量偏差g在机体坐标下的投影
//            vLarge_Num_F(qState.fGy, sfGy, fStep, 3);   //统计平均求测量偏差
			vLarge_Num_F(sfAcc_Bias, sfAcc, fStep, 3); //统计平均求测量偏差g在机体坐标下的投影
            vLarge_Num_F(sfGy_Bias, sfGy, fStep, 3);   //统计平均求测量偏差
            vSPL06_Get_PTH(&fPTH[0], &fPTH[1], &fPTH[2]); //通过IIC读取气压、温度、海拔值，速度较慢大概28ms
            vLarge_Num_F(qState.fPTH, fPTH, fStep, 3); //统计平均求起飞前气压、温度、海拔
            
            if (suAG_Bias_Cnt == CONTROL_DELAY)
            {
                vQuat_Init(sfAcc_Bias, fMag, sfQuat); //初始化四元数
                vQuatToEular(qState.fEu, sfQuat);
                vBToE_EXCH(sfQuat, qState.fAc, qState.fEAc);//得到地面坐标系下的加速度；是否是00g???

//                qState.fEAc[2] = qState.fEAc[2] - CONSTANT_G; //地面坐标系z放向加速度校正，去掉重力加速度的影响；
							  sfAcc_Bias[2] -= CONSTANT_G;
                qState.uFly_F = 1;
                suAG_Bias_Cnt = 0;
                sfAlt_Bias = qState.fPTH[2];//存储起飞前的海拔
            }
        }
        else
        {
			vData_Bias_Movf(sfAcc, sfAcc_Bias, 3);                       //加速度校正
            vData_Bias_Movf(sfGy, sfGy_Bias, 3);  
            vBut_LPF_O(qState.fAc, sfAcc, sfAcc_D1, sfAcc_D2, sfBF_P, 3); //加速度滤波
            vBut_LPF_O(qState.fGy, sfGy, sfGy_D1, sfGy_D2, sfBF_P, 3);    //角速度滤波
//            vBut_LPF_First(qState.fAc, sfAcc, qAcc_FP.fFP, 3); //低通滤波获得加速度
//            vBut_LPF_First(qState.fGy, sfGy, qGy_FP.fFP, 3);  //低通滤波获得角速度            
            vMahony_Comp_F(qState.fGy, qState.fAc, qState.fMa, &qCf, sfQuat); //互补滤波输出四元数
            //四元数转欧拉角Roll,Pitch,Yaw 单位 度°
            vQuatToEular(qState.fEu, sfQuat);
            vBToE_EXCH(sfQuat, qState.fAc, qState.fEAc);//得到地面坐标系下的加速度；
            qState.fEAc[2] = qState.fEAc[2] - CONSTANT_G; //地面坐标系z放向加速度校正，去掉重力加速度的影响；

        }

        //此任务以200Hz的频率运行
        vTaskDelayUntil(&uLastWakeTime, F2T(EST_AG_TASK_RATE));
    }
}

/**************************************************************************
    vEst_PTH_Task任务，获取温度，气压，高度
    此任务以50hz运行
**************************************************************************/
void vEst_PTH_Task(void *pvParameters)
{
    u32 uLastWakeTime = uGetSysTickCnt();
    while (1)
    {
        if (qState.uFly_F)
        {
            vSPL06_Get_PTH(&fPTH[0], &fPTH[1], &fPTH[2]);
            vBut_LPF_First(qState.fPTH, fPTH, qPTH_FP.fFP, 3);    //低通滤波获得气压，温度，海拔

        }
        //此任务以25Hz的频率运行
        vTaskDelayUntil(&uLastWakeTime, F2T(EST_PTH_TASK_RATE));
    }
}

/**************************************************************************
    vEst_OFXY_Task任务，光流获取xy轴位移及速度m/s，用sfVxy_OF,sfPxy_OF
**************************************************************************/
void vEst_OFXY_Task(void *pvParameters)
{
    u32 uLastWakeTime = uGetSysTickCnt();
    u32 uCurrentTime, uLastTime=0;
    s16 sVx, sVy;
    float fScale = 1; //m的尺度
    float fStep;

    while (1)
    {
        if((qState.fPo[2]>0.1)&&(qState.fPo[2]<4)&&qState.uFly_F)
        {
            if(vOpticFlow_Read(&sVx,&sVy,fScale)) //转成m
            {
                uCurrentTime = uGetSysTickCnt();//获取当前时刻
                sfPxy_OF[0] += sVx;//求位移
                sfPxy_OF[1] += sVy;
                fStep = (uCurrentTime-uLastTime)/ configTICK_RATE_HZ;//获取时间间隔
                sfVxy_OF[0] = sVx/fStep;//获取x方向速度m/s
                sfVxy_OF[1] = sVy/fStep;//获取y方向速度m/s
                // vBut_LPF_First(sfVxy_OF, fV, sfAcc_LP_P[1], 2); //速度滤波
                uLastTime = uCurrentTime;//存储当前时刻
            }           
        } 
                    //此任务以200Hz的频率运行
        vTaskDelayUntil(&uLastWakeTime, F2T(EST_OFXY_TASK_RATE));  		
    }

}


/**************************************************************************
    vEst_VP_Task任务，地面坐标系下，速度V与位置P估计与融合，后续结合卡尔曼滤波做
**************************************************************************/
void vEst_VP_Task(void *pvParameters)
{
    u32 uLastWakeTime = uGetSysTickCnt();
    float fATem[3] = {0};
    float fStep = 1.0f / EST_VP_TASK_RATE;
    float fTem=0;

    while (1)
    {
        if (qState.uFly_F)
        {
            // 加速度测量去死区
            fATem[0] = fDeadband_S(qState.fEAc[0], qAcc_FP.fDeadband[0]);
            fATem[1] = fDeadband_S(qState.fEAc[1], qAcc_FP.fDeadband[1]);
            fATem[2] = fDeadband_S(qState.fEAc[2], qAcc_FP.fDeadband[2]);
            // 位移及速度更新，完整约束动力学方程
            vPV_Dynamics(&qState.fPo[0],&qState.fVe[0],fStep,fATem[0]);
            vPV_Dynamics(&qState.fPo[1],&qState.fVe[1],fStep,fATem[1]);
            vPV_Dynamics(&qState.fPo[2],&qState.fVe[2],fStep,fATem[2]);
            //根据气压计测量值对位移及速度进行校正；

            fTem = qState.fPTH[2]-sfAlt_Bias-qState.fPo[2];//根据气压计得到的高度
            // 与气压计测量值相差很大时，用气压计值进行校正
            if(qState.fPo[2]>2 && fabs(fTem)> 2*qPTH_FP.fDeadband[2])
            {
                vPV_Correct(&qState.fPo[2], &qState.fVe[2], fTem, fStep, qPTH_FP.fFP[2]);//对高度及速度进行校正进行融合，侧重于气压计的值
            }   
        }
        //此任务以100Hz的频率运行
        vTaskDelayUntil(&uLastWakeTime, F2T(EST_VP_TASK_RATE));
    }
}


/**************************************************************************
    姿态参数初始化
**************************************************************************/
void vqState_Init(void)
{
    //迭代次数及起飞标志
    qState.uFly_F = 0;
    // qState.uNavi_F = 0;
    qState.Mode = LOCK;
    suAG_Bias_Cnt = 0;
    // //得到二阶滤波器的参数
    vBut_LPF_Par_Set(MPU6050_SMP_RATE, BLPF_CUT_F, sfBF_P);
    vLPF_First_Par_Set(qAcc_FP.fFP, qAcc_FP.fCutoff, qAcc_FP.fDt,3);
    vLPF_First_Par_Set(qGy_FP.fFP, qGy_FP.fCutoff, qGy_FP.fDt,3);
    vLPF_First_Par_Set(qPTH_FP.fFP, qPTH_FP.fCutoff, qPTH_FP.fDt,3);

}

/**************************************************************************
姿态参数复位
**************************************************************************/
void vEst_Par_Reset(void)
{
    //迭代次数及起飞标志
    //qState.Mode = LOCK;

	qState.uFly_F = 0;
    suAG_Bias_Cnt = 0;

    vVector_Par_Reset(qCf.fErr,3);//复位互补滤波器的积分误差

}

/**************************************************************************
完整约束二阶动力学方程，输入为位移，速度，采样周期，加速度
**************************************************************************/
void vPV_Dynamics(float * fP,float *fV, float fDt, float fAcc)
{
    *fP +=  *fV * fDt +  fAcc*fDt*fDt/2;
    *fV += fAcc*fDt;
}

/**************************************************************************
利用传感器数据对位置及速度预测值进行校正，输入为位移，速度，采样周期，误差，权重
本质还是低通滤波；
**************************************************************************/
void vPV_Correct(float * fP,float *fV, float fDt, float fErr,float fWeight)
{
    float fTem = fErr * fDt * fWeight;
    *fP +=  fTem;
    *fV +=  fWeight*fDt;
}



