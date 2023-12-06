#ifndef __WCOMMUNICATION_H__
#define __WCOMMUNICATION_H__

#include "stm32f10x.h"
#include "wsystem.h"

typedef struct //接收遥控的控制数据，在0~4096间
{
    s16 sRoll;
    s16 sPitch;
    s16 sThr;
    s16 sYaw;
    s16 sAux1;
    s16 sAux2;
    s16 sAux3;
    s16 sAux4;
    s16 sAux5;
    s16 sAux6;
} Q_RC_KEY;


//无人机控制输入
typedef struct
{
	float fThrust; //动力
    float fRoll;   //roll控制
    float fPitch;    //pitch控制
    float fYaw; //yaw控制
      
} QUAV_CTRL_SIG;


typedef enum //遥控解锁状态机
{
    WAITING_1 = 0,
    WAITING_2,
    WAITING_3,
    WAITING_4,
    WORK = 255,
    EXIT = 0xf1,
    
} EQ_RC_M;

#define COM_TASK_RATE RATE_200_HZ
#define vDATA_EXCH_TASK_RATE RATE_200_HZ
#define COM_TX_WIDTH 50

//上位机数据转无人机物理数据变换参数
typedef struct
{
    float fB_PidS; // BtoFpid变换因子
    float fB_EuS;  // BtoF欧拉角变换因子
    float fB_PwS;  // BtoF动力变换因子
    u8 uLim;       //整数型输入门限
    u16 uBias;     //整数型输入偏置
    u16 uRange;    //整数型输入范围
    u16 uLow;    //用于解锁判定
    u16 uMiddle;    //用于解锁判定
    u16 uHigh;    //用于解锁判定

} QUAV_RC_T;

//无人机物理数据转上位机数据变换参数
typedef struct
{

    float fF_PidS; // FtoBPid变换因子
    float fF_PwS;  // FtoBPid变换因子
    float fF_AcS;  // FtoB加速度变换因子
    float fF_GyS;  // FtoB角速度变换因子
    float fF_MaS;  // FtoB磁力计变换因子
    float fF_EuS;  // FtoB欧拉角变换因子
    float fF_PoS;  // FtoB位置变换因子
    float fF_VeS;  // FtoB绝对速度变换因子
    float fF_TeS;  // FtoB温度变换因子
    float fF_PrS;  // FtoB气压变换因子
    float fF_HeS;  // FtoB海拔m变换因子

} QUAV_FTOB_P;

/**************************************************************************
 * 配置发送接收功能关键字
 **************************************************************************/
typedef enum
{
    FUC_VER = 0x00,       //版本信息
    FUC_STATUS = 0x01,    //飞机姿态等基本信息
    FUC_SENSOR = 0x02,    //飞机传感器数据
    FUC_RCDATA = 0x03,    //飞机收到的控制数据
    FUC_GPSDATA = 0x04,   //机载GPS 信息
    FUC_POWER = 0x05,     //飞机电压电流信息
    FUC_MOTO = 0x06,      //"马达PWM(范围0-1000)"
    FUC_ALT = 0x07,       //飞机高度信息
    FUC_FLYMODE = 0x0A,   //飞行模式
    FUC_PID1 = 0x10,      // PID数据帧1,PID1 2 3
    FUC_PID2 = 0x11,      // PID数据帧2,PID4 5 6
    FUC_PID3 = 0x12,      // PID数据帧3,PID7 8 9
    FUC_PID4 = 0x13,      // PID数据帧4,PID10 11 12
    FUC_PID5 = 0x14,      // PID数据帧5,PID13 14 15
    FUC_PID6 = 0x15,      // PID数据帧6,PID16 17 18
    Self_Define1 = 0xF1,  //用户自定义数据
    Self_Define2 = 0xF2,  //用户自定义数据
    Self_Define3 = 0xF3,  //用户自定义数据
    Self_Define4 = 0xF4,  //用户自定义数据
    Self_Define5 = 0xF5,  //用户自定义数据
    Self_Define6 = 0xF6,  //用户自定义数据
    Self_Define7 = 0xF7,  //用户自定义数据
    Self_Define8 = 0xF8,  //用户自定义数据
    Self_Define9 = 0xF9,  //用户自定义数据
    Self_Define10 = 0xFA, //用户自定义数据

    /*---------------------以上全为飞控与上位机之间的功能字----------------------*/

    NRF_PID = 0x17,      //发送RPY角度PID参数
    NRF_GPID = 0x18,     //发送RPY角速度PID参数
    CONTROL_ADC = 0x16,  //发送RPY和油门值
    FUC_TX_OPEN = 0xAA,  //遥控器进入发送模式，无人机进入接受模式(默认)
    FUC_TX_CLOSE = 0xBB, //遥控器进入接收模式，无人机进入发送模式
    MODE = 0xFF,
    /*---------------------以上全为飞控与遥控器之间的功能字----------------------*/

} E_FUC_MODE;

/**************************************************************************
 * 帧校验方式
 **************************************************************************/

typedef enum
{
    FCHECK_SUM = 1,
    FCHECK_ODD,
    FCHECK_EVEN
} E_FRAME_CHECK;

/*************************************************************
 *vLED_Task任务
 * 无人机修改PID参数时亮灯
 **************************************************************/
void vLED_Task(void *pvParameters);

/*************************************************************
 * vComm_Task任务
 * 获得遥控器传来的数据，获取PID参数和遥控值
 **************************************************************/
void vComm_Task(void *pvParameters);
/*************************************************************
 * vRC_Task任务
 * 分析遥控值
 **************************************************************/
void vRC_Task(void *pvParameters);

/*************************************************************
 *解码，全局变量参数赋值,默认接收到有效字节数为18
 **************************************************************/
u8 nANO_RX_Frame_Decode(void);

/****************************************************************
 * vANO_TX_Frame_Encode函数,无人机向上位机发送各种数据
 ****************************************************************/
void vANO_TX_Frame_Encode(void);

/*************************************************************
 * NRF通信解码,接收遥控器给的数据并解析
 **************************************************************/
u8 nNRF_RX_Frame_Decode(void);

/*************************************************************
 * vANO_TX_Frame_Encode函数,无人机向遥控器发送各种数据
 **************************************************************/
void vNRF_TX_Frame_Decode(void);

/****************************************************************************************
 *发送float类型数据帧，一个float转成4字节的int的帧：帧头，帧头长度，校验类型，字节数组地址，float数组地址，长度，返回字节数目
 ****************************************************************************************/
u8 uTx_FtoS32(u8 *uHead, u8 uH_Len, E_FUC_MODE uFuc, E_FRAME_CHECK uFrameCheck, u8 *uData, float *fData, u8 uLen, float fScale);

/******************************************************************************************
 *发送float类型数据帧，一个float转成2字节的int的帧：帧头，帧头长度，校验类型，字节数组地址，float数组地址，长度，返回字节数目
 ******************************************************************************************/
u8 uTx_FtoS16(u8 *uHead, u8 uH_Len, E_FUC_MODE uFuc, E_FRAME_CHECK uFrameCheck, u8 *uData, float *fData, u8 uLen, float fScale);

u8 uTx_FtoS16z( E_FRAME_CHECK uFrameCheck, u8 *uData, float *fData, u8 uLen, float fScale);

#endif
