/**************************************************************************
    @file    wEularQ.c
    @version V1.0
    @date    2022.01.22
**************************************************************************/

#include <math.h>
#include "wEularQ.h"




 /**************************************************************************
#     fR, fP, fY分别对应绕x，y，z旋转，输入为弧度
#    以下所有的变换都是基于地面坐标系沿z-y-x顺序变换到机体坐标系
# **************************************************************************/


 /**************************************************************************
#     欧拉角求地面到机体方向余弦矩阵EBDCM。
# **************************************************************************/
void vEularToEBDCM(float fDCM[3][3], float fR, float fP, float fY)
{
    float fSr, fSp, fSy, fCr, fCp, fCy;

    fSr = sinf(fR); //括号内角度转弧度
    fCr = cosf(fR);
    fSp = sinf(fP);
    fCp = cosf(fP);
    fSy = sinf(fY);
    fCy = cosf(fY);

    fDCM[0][0] = fCy * fCp;
    fDCM[0][1] = fCp * fSy;
    fDCM[0][2] = -fSp;
    fDCM[1][0] = -fSy * fCr + (fCy * fSr * fSp);
    fDCM[1][1] = fCy * fCr + (fSy * fSr * fSp);
    fDCM[1][2] = fSr * fCp;
    fDCM[2][0] = fSy * fSr + (fCy * fCr * fSp);
    fDCM[2][1] = -(fCy * fSr) + (fSy * fCr * fSp);
    fDCM[2][2] = fCp * fCr;
}


 /**************************************************************************
#     欧拉角求地面到机体方向余弦矩阵BEDCM。
# **************************************************************************/
void vEularToBEDCM(float fDCM[3][3], float fR, float fP, float fY)
{
    float fSr, fSp, fSy, fCr, fCp, fCy;

    fSr = sinf(fR); //括号内角度转弧度
    fCr = cosf(fR);
    fSp = sinf(fP);
    fCp = cosf(fP);
    fSy = sinf(fY);
    fCy = cosf(fY);

    fDCM[0][0] = fCy * fCp; 
    fDCM[0][1] = -fSy * fCr + (fCy * fSr * fSp);  
    fDCM[0][2] = fSy * fSr + (fCy * fCr * fSp); 

    fDCM[1][0] = fCp * fSy; 
    fDCM[1][1] = fCy * fCr + (fSy * fSr * fSp);  
    fDCM[1][2] = -(fCy * fSr) + (fSy * fCr * fSp); 

    fDCM[2][0] = -fSp;  
    fDCM[2][1] = fSr * fCp;  
    fDCM[2][2] = fCp * fCr;  
}






/**************************************************************************
    四元数求地面到机体方向余弦矩阵BEDCM
**************************************************************************/
void vQuatToBEDCM(float fDCM[3][3], float *fQ)
{
    fDCM[0][0] = fQ[0] * fQ[0] + fQ[1] * fQ[1] - fQ[2] * fQ[2] - fQ[3] * fQ[3];  
    fDCM[0][1] = 2.0 * (fQ[1] * fQ[2] - fQ[0] * fQ[3]);  
    fDCM[0][2] = 2.0 * (fQ[1] * fQ[3] + fQ[0] * fQ[2]);  

    fDCM[1][0] = 2.0 * (fQ[1] * fQ[2] + fQ[0] * fQ[3]);  
    fDCM[1][1] = fQ[0] * fQ[0] - fQ[1] * fQ[1] + fQ[2] * fQ[2] - fQ[3] * fQ[3];  
    fDCM[1][2] = 2.0 * (fQ[2] * fQ[3] - fQ[0] * fQ[1]);  

    fDCM[2][0] = 2.0 * (fQ[1] * fQ[3] - fQ[0] * fQ[2]); 
    fDCM[2][1] = 2.0 * (fQ[2] * fQ[3] + fQ[0] * fQ[1]);  
    fDCM[2][2] = fQ[0] * fQ[0] - fQ[1] * fQ[1] - fQ[2] * fQ[2] + fQ[3] * fQ[3];  
}




# /**************************************************************************
#     输入为四元数fQ,得到从地面到机体的变换矩阵EBDCM
# **************************************************************************/

void vQuatToEBDCM(float fDCM[3][3], float *fQ)
{

    fDCM[0][0] = fQ[0] * fQ[0] + fQ[1] * fQ[1] - fQ[2] * fQ[2] - fQ[3] * fQ[3]; 
    fDCM[0][1] = 2.0 * (fQ[1] * fQ[2] + fQ[0] * fQ[3]); 
    fDCM[0][2] = 2.0 * (fQ[1] * fQ[3] - fQ[0] * fQ[2]); 
    
    fDCM[1][0] = 2.0 * (fQ[1] * fQ[2] - fQ[0] * fQ[3]); 
    fDCM[1][1] = fQ[0] * fQ[0] - fQ[1] * fQ[1] + fQ[2] * fQ[2] - fQ[3] * fQ[3]; 
    fDCM[1][2] = 2.0 * (fQ[2] * fQ[3] + fQ[0] * fQ[1]); 
    
    fDCM[2][0] = 2.0 * (fQ[1] * fQ[3] + fQ[0] * fQ[2]); 
    fDCM[2][1] = 2.0 * (fQ[2] * fQ[3] - fQ[0] * fQ[1]); 
    fDCM[2][2] = fQ[0] * fQ[0] - fQ[1] * fQ[1] - fQ[2] * fQ[2] + fQ[3] * fQ[3]; 


}

/**************************************************************************
    欧拉角求四元数
**************************************************************************/

void vEularToQuat(float fR, float fP, float fY, float *fQ)
{
    float fCr, fSr, fCp, fSp;
    float fCy, fSy;
    fCr = cosf(fR * 0.5);
    fSr = sinf(fR * 0.5);
    fCp = cosf(fP * 0.5);
    fSp = sinf(fP * 0.5);
    fCy = cosf(fY * 0.5);
    fSy = sinf(fY * 0.5);
    fQ[0] = fCr * fCp * fCy + fSr * fSp * fSy;
    fQ[1] = fSr * fCp * fCy - fCr * fSp * fSy;
    fQ[2] = fCr * fSp * fCy + fSr * fCp * fSy;
    fQ[3] = fCr * fCp * fSy - fSr * fSp * fCy;
}



/**************************************************************************
    四元数求欧拉角Roll,Pitch,Yaw.单位度
**************************************************************************/
void vQuatToEular(float *fE, float *fQ)
{
    float fTem0, fTem1, fTem2, fTem3, fTem4;
    fTem0 = 2.0 * (fQ[2] * fQ[3] + fQ[0] * fQ[1]);
    fTem1 = fQ[0] * fQ[0] - fQ[1] * fQ[1] - fQ[2] * fQ[2] + fQ[3] * fQ[3];
    fTem2 = 2.0 * (-fQ[1] * fQ[3] + fQ[0] * fQ[2]);
    fTem3 = 2.0 * (fQ[1] * fQ[2] + fQ[0] * fQ[3]);
    fTem4 = fQ[0] * fQ[0] + fQ[1] * fQ[1] - fQ[2] * fQ[2] - fQ[3] * fQ[3];

    fE[0] = atan2f(fTem0, fTem1) * RAD2DEG; // Roll.
    fE[1] = asinf(fTem2)* RAD2DEG;        // Pitch.
    fE[2] = atan2f(fTem3, fTem4)* RAD2DEG; // Yaw.

}


/**************************************************************************
    从地面到机体坐标的变换输入为fQ,fB，fE，先由四元数fQ从地面到机体的变换矩阵fEBDCM，再求fB
**************************************************************************/
void vEToB_EXCH(float *fQ, float *fB, float *fE)
{
    float fDCM[3][3];
    vQuatToEBDCM(fDCM, fQ);
    fB[0] = fDCM[0][0] * fE[0] + fDCM[0][1] * fE[1] + fDCM[0][2] * fE[2];
    fB[1] = fDCM[1][0] * fE[0] + fDCM[1][1] * fE[1] + fDCM[1][2] * fE[2];
    fB[2] = fDCM[2][0] * fE[0] + fDCM[2][1] * fE[1] + fDCM[2][2] * fE[2];
}

/**************************************************************************
    从机体到地面坐标的变换输入为四元数fQ，fB，fE，先由四元数fQ得到从机体到地面的变换矩阵fBEDCM，再求fE
**************************************************************************/
void vBToE_EXCH(float *fQ, float *fB, float *fE)
{
    float fDCM[3][3];
    vQuatToBEDCM(fDCM, fQ);
    fE[0] = fDCM[0][0] * fB[0] + fDCM[0][1] * fB[1] + fDCM[0][2] * fB[2];
    fE[1] = fDCM[1][0] * fB[0] + fDCM[1][1] * fB[1] + fDCM[1][2] * fB[2];
    fE[2] = fDCM[2][0] * fB[0] + fDCM[2][1] * fB[1] + fDCM[2][2] * fB[2];
}

/**************************************************************************
    四元数初始化：利用加速度计三轴加速度fA及磁力计测量fM实现
**************************************************************************/
void vQuat_Init(float *fA, float *fM, float *fQ)
{
    float fRoll, fPitch;
    float fCr, fSr, fCp, fSp;
    //    float fTem0, fTem1;
    float fYaw, fCy, fSy;
    float fTem;
    
    if (!((fA[0] == 0.0f) && (fA[1] == 0.0f) && (fA[2] == 0.0f)))//保证加速度测量输出有效
    {   

        fRoll = -atan2f(fA[1], fA[2]);
        fTem = -fA[0]/sqrt(fA[1] * fA[1] + fA[2] * fA[2]);
        fPitch = asin(fTem);
        // if (fA[2] < 0 && fA[0] > 0)
        // {
        //     fPitch = CONST_PI - atan2f(fA[0], sqrt(fA[1] * fA[1] + fA[2] * fA[2]));
        // }
        // else if (fA[2] < 0 && fA[0] < 0)
        // {
        //     fPitch = -CONST_PI - atan2f(fA[0], sqrt(fA[1] * fA[1] + fA[2] * fA[2]));
        // }
        // else
        //     fPitch = atan2f(fA[0], sqrt(fA[1] * fA[1] + fA[2] * fA[2]));

        // fTem0 = fM[0] * cosf(fPitch) + fM[1] * sinf(fRoll) * sinf(fPitch) + fM[2] * cosf(fRoll) * sinf(fPitch);
        // fTem1 = fM[1] * cosf(fRoll) - fM[2] * sinf(fRoll);
        //  fYaw = atan2f(-fTem1, fTem0);
        //加速度计无法求得偏航角yaw
        // fYaw = 0;
        // vEularToQuat(fRoll, fPitch, fYaw, fQ);
        fYaw = 0;
        vEularToQuat(fRoll, fPitch, fYaw, fQ);
    }
}

/**************************************************************************
    Mahony互补滤波，输入滤波后角速度fGF，加速度fAF及测力计fMF，增益参数fKp，fKi，
    步长fDt，角速度校正误差的积分fG_E，输出四元数
**************************************************************************/
void vMahony_Comp_F(float *fGF, float *fAF, float *fMF,
                    QUAV_CF_P *qCF, float *fQ)
{
    uint8_t i;
    float fInv_N;
    float fG[3], fA[3], fM[3];
    float fEex = 0.0f;
    float fEey = 0.0f;
    float fEez = 0.0f;
    for (i = 0; i < 3; i++)
    {
        fG[i] = fGF[i]; //弧度
        fA[i] = fAF[i];
        fM[i] = fMF[i];
    }
    //如果磁力计测量值有效
    if (!((fM[0] == 0.0f) && (fM[1] == 0.0f) && (fM[2] == 0.0f)))
    {
        float fEx, fEy, fEz, fBx, fBz;
        float fEwx, fEwy, fEwz;
        // 归一化磁场测量值
        fInv_N = fCal_Inv_Sqrt(fM[0] * fM[0] + fM[1] * fM[1] + fM[2] * fM[2]);
        fM[0] *= fInv_N;
        fM[1] *= fInv_N;
        fM[2] *= fInv_N;
        // 地球磁场参考方向
        fEx = 2.0f * (fM[0] * (0.5f - fQ[2] * fQ[2] - fQ[3] * fQ[3]) + fM[1] * (fQ[1] * fQ[2] - fQ[0] * fQ[3]) +
                      fM[2] * (fQ[1] * fQ[3] + fQ[0] * fQ[2]));
        fEy = 2.0f * (fM[0] * (fQ[1] * fQ[2] + fQ[0] * fQ[3]) + fM[1] * (0.5f - fQ[1] * fQ[1] - fQ[3] * fQ[3]) +
                      fM[2] * (fQ[2] * fQ[3] - fQ[0] * fQ[1]));
        fEz = 2.0f * fM[0] * (fQ[1] * fQ[3] - fQ[0] * fQ[2]) + 2.0f * fM[1] * (fQ[2] * fQ[3] + fQ[0] * fQ[1]) +
              2.0f * fM[2] * (0.5f - fQ[1] * fQ[1] - fQ[2] * fQ[2]);
        fBx = sqrtf(fEx * fEx + fEy * fEy);
        fBz = fEz;
        // 磁场估计方向
        fEwx = fBx * (0.5f - fQ[2] * fQ[2] - fQ[3] * fQ[3]) + fBz * (fQ[1] * fQ[3] - fQ[0] * fQ[2]);
        fEwy = fBx * (fQ[1] * fQ[2] - fQ[0] * fQ[3]) + fBz * (fQ[0] * fQ[1] + fQ[2] * fQ[3]);
        fEwz = fBx * (fQ[0] * fQ[2] + fQ[1] * fQ[3]) + fBz * (0.5f - fQ[1] * fQ[1] - fQ[2] * fQ[2]);
        // 矢量叉积得到修正误差
        fEex = (fM[1] * fEwz - fM[2] * fEwy);
        fEey = (fM[2] * fEwx - fM[0] * fEwz);
        fEez = (fM[0] * fEwy - fM[1] * fEwx);
    }

    // 如果加速度计测量值有效
    if (!((fA[0] == 0.0f) && (fA[1] == 0.0f) && (fA[2] == 0.0f)))
    {
        float fEax, fEay, fEaz;
        fInv_N = fCal_Inv_Sqrt(fA[0] * fA[0] + fA[1] * fA[1] + fA[2] * fA[2]);
        fA[0] *= fInv_N;
        fA[1] *= fInv_N;
        fA[2] *= fInv_N;
        // 重力g*[0,0,1]转到机体坐标系下vx，vy,vz
        fEax = 2 * (fQ[1] * fQ[3] - fQ[0] * fQ[2]);
        fEay = 2 * (fQ[0] * fQ[1] + fQ[2] * fQ[3]);
        fEaz = 1.0f - 2 * (fQ[1] * fQ[1] + fQ[2] * fQ[2]);
        // 四元数求得的ax,ay,az方向与加速度计测量方向叉积表示误差
        fEex = (fA[1] * fEaz - fA[2] * fEay);
        fEey = (fA[2] * fEax - fA[0] * fEaz);
        fEez = (fA[0] * fEay - fA[1] * fEax);
    }

    // 比例积分进行误差处理，没有考虑积分限幅的影响
    if ((fEex != 0.0f) && (fEey != 0.0f) && (fEez != 0.0f))
    {
        // 误差积分部分
        qCF->fErr[0] += qCF->fKi * fEex * qCF->fDt;
        qCF->fErr[1] += qCF->fKi * fEey * qCF->fDt;
        qCF->fErr[2] += qCF->fKi * fEez * qCF->fDt;
        // 误差求解.
        fG[0] += qCF->fErr[0];
        fG[1] += qCF->fErr[1];
        fG[2] += qCF->fErr[2];
    }
    else
    {
        qCF->fErr[0] = 0;
        qCF->fErr[1] = 0;
        qCF->fErr[2] = 0;
    }

    fG[0] += qCF->fKp * fEex;
    fG[1] += qCF->fKp * fEey;
    fG[2] += qCF->fKp * fEez;

    vQuat_Update(fG, qCF->fDt, fQ);
    
}


/**************************************************************************
    单阶四元数迭代计算，输入fG计算的校正信号（迭代模型的系数），fDt步长，fQ为四元数
**************************************************************************/
void vQuat_Update(float *fG, float fDt, float *fQ)
{
    float fInv_N;
    float fQTem[4];
    fQTem[0] = 0.5f * (-fQ[1] * fG[0] - fQ[2] * fG[1] - fQ[3] * fG[2]);
    fQTem[1] = 0.5f * (fQ[0] * fG[0] + fQ[2] * fG[2] - fQ[3] * fG[1]);
    fQTem[2] = 0.5f * (fQ[0] * fG[1] - fQ[1] * fG[2] + fQ[3] * fG[0]);
    fQTem[3] = 0.5f * (fQ[0] * fG[2] + fQ[1] * fG[1] - fQ[2] * fG[0]);
    fQ[0] += fDt * fQTem[0];
    fQ[1] += fDt * fQTem[1];
    fQ[2] += fDt * fQTem[2];
    fQ[3] += fDt * fQTem[3];
    //归一化
    fInv_N = fCal_Inv_Sqrt(fQ[0] * fQ[0] + fQ[1] * fQ[1] + fQ[2] * fQ[2] + fQ[3] * fQ[3]);
    fQ[0] *= fInv_N;
    fQ[1] *= fInv_N;
    fQ[2] *= fInv_N;
    fQ[3] *= fInv_N;
}
