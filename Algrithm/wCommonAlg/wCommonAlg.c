/**************************************************************************
    @file    wCommonAlg.c
    @version V1.0
    @date    2022.02.25
    滤波器采用二阶Butterworth滤波器
**************************************************************************/

#include <math.h>
#include "wCommonAlg.h"
#include <stdint.h>
#include <stdlib.h>
/**************************************************************************
 * 	Function: 二阶Butterworth滤波器参数求解,输入采样频率及截止频率,输出长度为5的一维数组
 **************************************************************************/
void vBut_LPF_Par_Set(float fSam_F, float fCut_F, float *fF_Par)
{
    float M_PI = 3.14159265;
    float fFr = 0;
    float fOhm = 0;
    float fTem = 0;
    fFr = fSam_F / fCut_F;
    fOhm = tanf(M_PI / fFr);
    fTem = 1.0f + 2.0f * cosf(M_PI / 4.0f) * fOhm + fOhm * fOhm;
    fF_Par[0] = fOhm * fOhm / fTem;
    fF_Par[1] = 2.0f * fF_Par[0];
    fF_Par[2] = fF_Par[0];
    fF_Par[3] = 2.0f * (fOhm * fOhm - 1.0f) / fTem;
    fF_Par[4] = (1.0f - 2.0f * cosf(M_PI / 4.0f) * fOhm + fOhm * fOhm) / fTem;
}




/**************************************************************************
 * 	Function: 二阶Butterworth滤波器输出,输入为fIn(t),fIn(t-1),fIn(t-2),滤波器参数fF_Par
 * 数组长度uDim，输出为fOut
 **************************************************************************/
void vBut_LPF_O(float *fOut, float *fIn, float *fIn_D1, float *fIn_D2, float *fF_Par, uint8_t uDim)
{
    float fTem[uDim];

    uint8_t i;
    for (i = 0; i < uDim; i++)
    {
        fTem[i] = fIn[i] - fIn_D1[i] * fF_Par[3] - fIn_D2[i] * fF_Par[4];
        if (isnan(fTem[i]) || isinf(fTem[i]))
        {
            fTem[i] = fIn[i]; // 防止不合法值传播
        }
        fOut[i] = fTem[i] * fF_Par[0] + fIn_D1[i] * fF_Par[1] + fIn_D2[i] * fF_Par[2];
        fIn_D2[i] = fIn_D1[i];
        fIn_D1[i] = (float)fTem[i];
    }
}


/**************************************************************************
 * 	Function: 一阶低通滤波器参数求解,输入采样频率及截止频率,输出长度为uDim的数组
 **************************************************************************/
void vLPF_First_Par_Set(float *fF_Par, float *fCut_F, float fSam_F, uint8_t uDim)
{
    float M_PI = 3.14159265;
    float fTau;//时间常数
    uint8_t i;
    for (i = 0; i < uDim; i++)
    {
        fTau = 1/(2*M_PI*fCut_F[i]);
        fF_Par[i] = 1/(1+fTau*fSam_F);//Ts/(Ts+fTau)
    }
    
   
}

/**************************************************************************
 * 	Function: 一阶低通滤波器输出,输入为fIn,滤波器参数fF_Par,输出为fOut
 **************************************************************************/
void vBut_LPF_First(float *fOut, float *fIn, float *fF_Par, uint8_t uDim)
{
    uint8_t i;
    for (i = 0; i < uDim; i++)
    {
        *(fOut+i) =  (1.0f - *(fF_Par+i)) * (*(fOut+i)) + *(fF_Par+i) * (*(fIn+i));
    }
}

/**************************************************************************
 * 	Function: PID算法，输入为结构体格式的pid参数及当前的误差
 * 输出为fOut存入pid结构体对应位置
 **************************************************************************/
void vPID_Control(CTRL_PID_P *Pid, float fErr)
{

    Pid->fErr = fErr;

    Pid->fE_Int += Pid->fErr * Pid->fDt;
    Pid->fDE = (float)(Pid->fErr - Pid->fE_Pre) / Pid->fDt;

    Pid->fKP_Out = Pid->fKp * Pid->fErr;
    Pid->fKI_Out = Pid->fKi * Pid->fE_Int;
    Pid->fKD_Out = Pid->fKd * Pid->fDE;

    Pid->fE_Pre = Pid->fErr;

    if (Pid->fKI_Out > Pid->fInt_Lim)
    {
        Pid->fKI_Out = Pid->fInt_Lim;
    }
    else if (Pid->fKI_Out < -Pid->fInt_Lim)
    {
        Pid->fKI_Out = -Pid->fInt_Lim;
    }

    Pid->fOut = Pid->fKP_Out + Pid->fKI_Out + Pid->fKD_Out;
}

/******************************************
 * 重置pid的输出
 ******************************************/
void vPid_Par_Reset(CTRL_PID_P *pid)
{
	pid->fRef =0; //参考信号
    pid->fE_Pre = 0; // 前一个误差
    pid->fE_Int = 0;   // 累计误差
}


/******************************************
 * pid参考信号设置
 ******************************************/
void vPid_Ref_Set(float *fRef, float fData)
{
	*fRef =fData; //参考信号
}




/**************************************************************************
    平分根求逆的快速计算
**************************************************************************/
float fCal_Inv_Sqrt(float fTem)
{
    volatile long lI;
    volatile float fX;
    volatile float fY;
    volatile const float fF = 1.5f;

    fX = fTem * 0.5f;
    fY = fTem;
    lI = *((long *)&fY);
    lI = 0X5F375A86 - (lI >> 1);
    fY = *((float *)&lI);
    fY = fY * (fF - (fX * fY * fY));
    return fY;
}

/**************************************************************************
 * 	FIFO 队列，末尾为新存入数据fData
 **************************************************************************/
void vFIFO_Queuf(float *fBuff, float fData, uint16_t uDim)
{
    uint16_t i;

    for (i = 1; i < uDim; i++)
    {
        fBuff[i - 1] = fBuff[i];
    }

    fBuff[uDim - 1] = fData;
}

// iBuff缓存空间，iData缓存数据，uDim缓存长度
void vFIFO_Queui(uint32_t *iBuff, int32_t iData, uint16_t uDim)
{
    uint16_t i;
    //压栈
    for (i = 1; i < uDim; i++)
    {
        iBuff[i - 1] = iBuff[i];
    }
    //将数据压入栈尾
    iBuff[uDim - 1] = iData;
}

/**************************************************************************
    求返回FIFO平均值
**************************************************************************/
float fAve_Get(float *fBuff, uint16_t uDim)
{
    uint16_t i;
    float fSum = 0.0f;

    for (i = 0; i < uDim; i++)
    {
        fSum += fBuff[i];
    }

    return fSum / uDim;
}

uint32_t uAve_Get(uint32_t *uBuff, uint16_t uDim)
{
    uint16_t i;
    uint64_t uSum = 0;

    for (i = 0; i < uDim; i++)
    {
        uSum += uBuff[i];
    }

    return (uint32_t)(uSum / uDim);
}
/**************************************************************************
 * 	FIFO 队列，末尾为新存入数据fData
 **************************************************************************/
uint8_t uCheck_FIFO(float *fBuff, uint16_t uDim)
{
    uint16_t i, uCnt;

    for (i = 1, uCnt = 0; i < uDim; i++)
    {
        if (fBuff[i] < 0)
            uCnt++;
        else
            uCnt--;
    }
    if (abs(uCnt) <= 2)
        return 0;
    else
        return 1;
}
/**************************************************************************
    求两个数的差结果存在fout
**************************************************************************/
void vData_Bias_Movf(float *fOut, float *fIn, uint8_t uDim)
{
    uint8_t i;
    for (i = 0; i < uDim; i++)
    {
        fOut[i] -= fIn[i];
    }
}

void vData_Bias_Movi(int32_t *iOut, int32_t *iIn, uint8_t uDim)
{
    uint8_t i;
    for (i = 0; i < uDim; i++)
    {
        iOut[i] -= iIn[i];
    }
}

/**************************************************************************
    大数定律求统计平均，输入新的值fIn,输出fOut，步长fDt,维数uDim，
    uDim为3，fDt每次更新
    fOut = fIn * fDt + fOut * (1 - fDt)
**************************************************************************/
void vLarge_Num_F(float *fOut, float *fIn, float fDt, uint8_t uDim)
{
    uint8_t i;
    for (i = 0; i < uDim; i++)
    {
        fOut[i] = fIn[i] * fDt + fOut[i] * (1 - fDt);
    }
}


/**************************************************************************
    求两个数组的和结果存在fout
**************************************************************************/
void vSum_Two_Num(float *fOut, float *fIn, uint8_t uDim)
{
    uint8_t i;
    for (i = 0; i < uDim; i++)
    {
        fOut[i] += fIn[i];
    }
}

/**************************************************************************
    数组除以一个数后保存数组
**************************************************************************/
void vDiv_Two_Num(float *fOut, float fIn, uint8_t uDim)
{
    uint8_t i;
    for (i = 0; i < uDim; i++)
    {
        fOut[i] = (float)(fOut[i] / fIn);
    }
}

/**************************************************************************
    标量死区函数，当Data绝对值小于deadband时为0，其他值时Data-deadband
**************************************************************************/
float fDeadband_S(float fData, float fDeadb)
{

    if (fData > fDeadb)
    {
        fData -= fDeadb;
    }
    else if (fData < -fDeadb)
    {
        fData += fDeadb;
    }
    else
        fData = 0;
    return fData;
}


/**************************************************************************
    向量死区函数，当Data绝对值小于deadband时为0，其他值时Data-deadband
**************************************************************************/
void vDeadband_V(float *fData, float *fDeadb, uint8_t uDim)
{
    uint8_t i;
    for (i=0; i<uDim; i++)
    {
        if (fData[i] > fDeadb[i])
        {
            fData[i] -= fDeadb[i];
        }
        else if (fData[i] < -fDeadb[i])
        {
            fData[i] += fDeadb[i];
        }
        else
            fData[i] = 0;
    }

}




/**************************************************************************
    接收帧中字节转成float型，输入为两个字节,偏置，限幅及变化尺度，返回为float型
************************************************************************/

float vByte_To_Float(u8 uData1, u8 uData2, u8 uLim, u16 uBias, u16 uRange, float fScale)
{
    s16 sTem = (s16)(uData1 << 8) | uData2;

    if (abs(sTem - uBias) < uLim)
        return 0;
    else
        return (sTem - uBias) / uRange * fScale;
}

/**************************************************************************
    16int转成float型，输入为16位int,限幅,偏置量，范围及变换尺度，返回为float型
************************************************************************/

float fInt_To_Float(s16 sData, u8 uLim, u16 uBias, u16 uRange, float fScale)
{

    if (abs(sData - uBias) < uLim)
        return 0;
    else
        return (float)(sData - uBias) / uRange * fScale;
}

/************************************************************************************************
    Buf中无符号字节数组转成16Int型数组，输入为8位int型数组地址,字节数组地址，字节地址偏置及int数组长度
**********************************************************************************************/

void vBuf_BtoI(s16 *sOut, u8 *uIn, u8 uBias, u8 uLen)
{
    u8 i;
    for (i = 0; i < uLen; i++)
    {
        *(sOut + i) = BYTETOWORD(uIn[2 * i + uBias], uIn[2 * i + uBias + 1]);
    }
}


/************************************************************************************************
    批量复位
**********************************************************************************************/

void vVector_Par_Reset(float *fIn,  u8 uLen)
{
    u8 i;
    for (i = 0; i < uLen; i++)
    {
        *(fIn + i) = 0;
    }
}



/************************************************************************************************
    输出限幅
**********************************************************************************************/

s16 sThreshold(s16  sIn, s16 sL, s16 sH)
{
    if (sIn <= sL)
        sIn = sL;
    if (sIn >= sH)
        sIn = sH;
    return sIn;
}




float fThreshold(float  fIn, float fL, float fH)
{
    if (fIn <= fL)
        fIn = fL;
    if (fIn >= fH)
        fIn = fH;
    return fIn;

}
