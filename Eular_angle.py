

import numpy as np
import math
import sympy

#/**************************************************************************
#     fR, fP, fY分别对应绕x，y，z旋转，输入为弧度
#    以下所有的变换都是基于地面坐标系沿z-y-x顺序变换到机体坐标系
# **************************************************************************/
#
# /**************************************************************************
#     欧拉角求地面到机体方向余弦矩阵EBDCM
# **************************************************************************/
def EularToEBDCM(roll,pitch,yaw): 

    fSr = math.sin(roll) 
    fCr = math.cos(roll)  
    fSp = math.sin(pitch)  
    fCp = math.cos(pitch)  
    fSy = math.sin(yaw)  
    fCy = math.cos(yaw)  

    fDCM = np.zeros((3,3))

    fDCM[0][0] = fCy * fCp  
    fDCM[0][1] = fCp * fSy  
    fDCM[0][2] = -fSp  
    fDCM[1][0] = -fSy * fCr + (fCy * fSr * fSp)  
    fDCM[1][1] = fCy * fCr + (fSy * fSr * fSp)  
    fDCM[1][2] = fSr * fCp  
    fDCM[2][0] = fSy * fSr + (fCy * fCr * fSp)  
    fDCM[2][1] = -(fCy * fSr) + (fSy * fCr * fSp)  
    fDCM[2][2] = fCp * fCr  



    return fDCM


# /**************************************************************************
#     欧拉角求机体到地面方向余弦矩阵BEDCM。
# **************************************************************************/
def EularToBEDCM(roll,pitch,yaw): 

    fSr = math.sin(roll) 
    fCr = math.cos(roll)  
    fSp = math.sin(pitch)  
    fCp = math.cos(pitch)  
    fSy = math.sin(yaw)  
    fCy = math.cos(yaw)  

    fDCM = np.zeros((3,3))

    fDCM[0][0] = fCy * fCp 
    fDCM[0][1] = -fSy * fCr + (fCy * fSr * fSp)  
    fDCM[0][2] = fSy * fSr + (fCy * fCr * fSp) 

    fDCM[1][0] = fCp * fSy  
    fDCM[1][1] = fCy * fCr + (fSy * fSr * fSp)  
    fDCM[1][2] = -(fCy * fSr) + (fSy * fCr * fSp)  

    fDCM[2][0] = -fSp  
    fDCM[2][1] = fSr * fCp  
    fDCM[2][2] = fCp * fCr  

    return fDCM


# /**************************************************************************
#     输入为四元数fQ,四元数求机体到地面方向余弦矩阵BEDCM
# **************************************************************************/

def vQuatToBEDCM(fQ):

    fDCM = np.zeros((3,3))

    fDCM[0][0] = fQ[0] * fQ[0] + fQ[1] * fQ[1] - fQ[2] * fQ[2] - fQ[3] * fQ[3]  
    fDCM[0][1] = 2.0 * (fQ[1] * fQ[2] - fQ[0] * fQ[3])  
    fDCM[0][2] = 2.0 * (fQ[1] * fQ[3] + fQ[0] * fQ[2])  
    fDCM[1][0] = 2.0 * (fQ[1] * fQ[2] + fQ[0] * fQ[3])  
    fDCM[1][1] = fQ[0] * fQ[0] - fQ[1] * fQ[1] + fQ[2] * fQ[2] - fQ[3] * fQ[3]  
    fDCM[1][2] = 2.0 * (fQ[2] * fQ[3] - fQ[0] * fQ[1])  
    fDCM[2][0] = 2.0 * (fQ[1] * fQ[3] - fQ[0] * fQ[2])  
    fDCM[2][1] = 2.0 * (fQ[2] * fQ[3] + fQ[0] * fQ[1])  
    fDCM[2][2] = fQ[0] * fQ[0] - fQ[1] * fQ[1] - fQ[2] * fQ[2] + fQ[3] * fQ[3]  

    return fDCM


# /**************************************************************************
#     输入为四元数fQ,得到从地面到机体的变换矩阵EBDCM
# **************************************************************************/

def vQuatToEBDCM(fQ):

    fDCM = np.zeros((3,3))

    fDCM[0][0] = fQ[0] * fQ[0] + fQ[1] * fQ[1] - fQ[2] * fQ[2] - fQ[3] * fQ[3] 
    fDCM[0][1] = 2.0 * (fQ[1] * fQ[2] + fQ[0] * fQ[3]) 
    fDCM[0][2] = 2.0 * (fQ[1] * fQ[3] - fQ[0] * fQ[2]) 
    
    fDCM[1][0] = 2.0 * (fQ[1] * fQ[2] - fQ[0] * fQ[3]) 
    fDCM[1][1] = fQ[0] * fQ[0] - fQ[1] * fQ[1] + fQ[2] * fQ[2] - fQ[3] * fQ[3] 
    fDCM[1][2] = 2.0 * (fQ[2] * fQ[3] + fQ[0] * fQ[1]) 
    
    fDCM[2][0] = 2.0 * (fQ[1] * fQ[3] + fQ[0] * fQ[2]) 
    fDCM[2][1] = 2.0 * (fQ[2] * fQ[3] - fQ[0] * fQ[1]) 
    fDCM[2][2] = fQ[0] * fQ[0] - fQ[1] * fQ[1] - fQ[2] * fQ[2] + fQ[3] * fQ[3] 

    return fDCM



# /**************************************************************************
#     欧拉角求四元数分别绕zyx旋转
# **************************************************************************/

def vEularToQuat(fR, fP, fY):

    fQ = np.zeros(4)

    fCr = math.cos(fR * 0.5) 
    fSr = math.sin(fR * 0.5) 
    fCp = math.cos(fP * 0.5) 
    fSp = math.sin(fP * 0.5) 
    fCy = math.cos(fY * 0.5) 
    fSy = math.sin(fY * 0.5) 
    fQ[0] = fCr * fCp * fCy + fSr * fSp * fSy 
    fQ[1] = fSr * fCp * fCy - fCr * fSp * fSy 
    fQ[2] = fCr * fSp * fCy + fSr * fCp * fSy 
    fQ[3] = fCr * fCp * fSy - fSr * fSp * fCy 

    return fQ


# /**************************************************************************
#     四元数求欧拉角Roll,Pitch,Yaw，分别绕zyx旋转，输出为弧度
# **************************************************************************/

def vQuatToEular(fQ):
    fTem0 = 2.0 * (fQ[2] * fQ[3] + fQ[0] * fQ[1])
    fTem1 = fQ[0] * fQ[0] - fQ[1] * fQ[1] - fQ[2] * fQ[2] + fQ[3] * fQ[3]
    fTem2 = 2.0 * (-fQ[1] * fQ[3] + fQ[0] * fQ[2])
    fTem3 = 2.0 * (fQ[1] * fQ[2] + fQ[0] * fQ[3])
    fTem4 = fQ[0] * fQ[0] + fQ[1] * fQ[1] - fQ[2] * fQ[2] - fQ[3] * fQ[3]

    fE = np.zeros(3)
    ##转成角度
    ftem = fTem0/fTem1
    fE[0] = math.atan(ftem)
    fE[1] = math.asin(fTem2)
    ftem = fTem3/fTem4
    fE[2] = math.atan(ftem)
    return fE


# /**************************************************************************
#  从地面到机体坐标的变换输入为fQ,fE，先由四元数fQ从地面到机体的变换矩阵fEBDCM，再求fB
# **************************************************************************/



def vEToB_EXCH(fQ, fE):

    fDCM = vQuatToEBDCM(fQ)
    fB = np.zeros(3)
    fB[0] = fDCM[0][0] * fE[0] + fDCM[0][1] * fE[1] + fDCM[0][2] * fE[2] 
    fB[1] = fDCM[1][0] * fE[0] + fDCM[1][1] * fE[1] + fDCM[1][2] * fE[2] 
    fB[2] = fDCM[2][0] * fE[0] + fDCM[2][1] * fE[1] + fDCM[2][2] * fE[2] 

    return fB


# /**************************************************************************
#     从机体到地面坐标的变换输入为四元数fQ，fB，先由四元数fQ得到从机体到地面的变换矩阵fBEDCM，再求fE
# **************************************************************************/

def vBToE_EXCH(fQ, fB):

    fDCM = vQuatToBEDCM(fQ)
    fE = np.zeros(3)
    fE[0] = fDCM[0][0] * fB[0] + fDCM[0][1] * fB[1] + fDCM[0][2] * fB[2] 
    fE[1] = fDCM[1][0] * fB[0] + fDCM[1][1] * fB[1] + fDCM[1][2] * fB[2] 
    fE[2] = fDCM[2][0] * fB[0] + fDCM[2][1] * fB[1] + fDCM[2][2] * fB[2] 

    return fE


# /**************************************************************************
#     四元数初始化：利用加速度计三轴加速度fA及磁力计测量fM实现，传递array传地址
# **************************************************************************/

def vQuat_Init(fA, fM):

    if (not((fA[0] == 0.0) and (fA[1] == 0.0) and (fA[2] == 0.0))):   
        ftem = fA[1]/fA[2]
        fRoll = math.atan(ftem) 
        ftem = -fA[0]/(np.sqrt(fA[0] * fA[0]+fA[1] * fA[1] + fA[2] * fA[2]))
        ##(np.sqrt(fA[0] * fA[0]+fA[1] * fA[1] + fA[2] * fA[2]))=g
        fPitch = math.asin(ftem) 



        fTem0 = fM[0] * math.cos(fPitch) + fM[1] * math.sin(fRoll) * math.sin(fPitch) + fM[2] * math.cos(fRoll) * math.sin(fPitch) 
        fTem1 = fM[1] * math.cos(fRoll) - fM[2] * math.sin(fRoll) 
        if (not ((fTem1==0) and(fTem0 ==0))): 
            fYaw = math.atan(-fTem0/fTem1) 
        else:
            fYaw = 0
        fQ = vEularToQuat(fRoll, fPitch, fYaw)
        return fQ


# /**************************************************************************
#     Mahony互补滤波，输入滤波后角速度fGF，加速度fAF及测力计fMF，增益参数qCF[3]={fKp，fKi，fDt}
#     角速度校正误差的积分fG_E，输出四元数fQ,fG_E
# **************************************************************************/


def vMahony_Comp_F(fGF, fAF, fMF, qCF, fG_E, fQ):

    fEex = 0.0
    fEey = 0.0
    fEez = 0.0

    fG = fGF.copy() #
    fA = fAF.copy()
    fM = fMF.copy()
    #如果磁力计测量值有效
    if (not((fM[0] == 0.0) and (fM[1] == 0.0) and (fM[2] == 0.0))):
 
        # 归一化磁场测量值
        fInv_N = 1/np.sqrt(fM[0] * fM[0] + fM[1] * fM[1] + fM[2] * fM[2])
        fM[0] *= fInv_N
        fM[1] *= fInv_N
        fM[2] *= fInv_N
        #地球磁场参考方向
        fEx = 2.0 * (fM[0] * (0.5 - fQ[2] * fQ[2] - fQ[3] * fQ[3]) + fM[1] * (fQ[1] * fQ[2] - fQ[0] * fQ[3]) + fM[2] * (fQ[1] * fQ[3] + fQ[0] * fQ[2]))
        fEy = 2.0 * (fM[0] * (fQ[1] * fQ[2] + fQ[0] * fQ[3]) + fM[1] * (0.5 - fQ[1] * fQ[1] - fQ[3] * fQ[3]) + fM[2] * (fQ[2] * fQ[3] - fQ[0] * fQ[1]))
        fEz = 2.0 * fM[0] * (fQ[1] * fQ[3] - fQ[0] * fQ[2]) + 2.0 * fM[1] * (fQ[2] * fQ[3] + fQ[0] * fQ[1]) + 2.0 * fM[2] * (0.5 - fQ[1] * fQ[1] - fQ[2] * fQ[2])
        fBx = np.sqrt(fEx * fEx + fEy * fEy)
        fBz = fEz
        # 磁场估计方向
        fEwx = fBx * (0.5 - fQ[2] * fQ[2] - fQ[3] * fQ[3]) + fBz * (fQ[1] * fQ[3] - fQ[0] * fQ[2])
        fEwy = fBx * (fQ[1] * fQ[2] - fQ[0] * fQ[3]) + fBz * (fQ[0] * fQ[1] + fQ[2] * fQ[3])
        fEwz = fBx * (fQ[0] * fQ[2] + fQ[1] * fQ[3]) + fBz * (0.5 - fQ[1] * fQ[1] - fQ[2] * fQ[2])
        # 矢量叉积得到修正误差
        fEex = (fM[1] * fEwz - fM[2] * fEwy)
        fEey = (fM[2] * fEwx - fM[0] * fEwz)
        fEez = (fM[0] * fEwy - fM[1] * fEwx)


    # 如果加速度计测量值有效
    if (not((fA[0] == 0.0) and (fA[1] == 0.0) and (fA[2] == 0.0))):

        fInv_N =  np.sqrt(fA[0] * fA[0] + fA[1] * fA[1] + fA[2] * fA[2])
        fA[0] *= fInv_N
        fA[1] *= fInv_N
        fA[2] *= fInv_N
        #重力g*[0,0,1]转到地面坐标系下vx，vy,vz
        fEax = 2 * (fQ[1] * fQ[3] - fQ[0] * fQ[2])
        fEay = 2 * (fQ[0] * fQ[1] + fQ[2] * fQ[3])
        fEaz = 1.0 - 2 * (fQ[1] * fQ[1] + fQ[2] * fQ[2])
        #四元数求得的ax,ay,az方向与加速度计测量方向叉积表示误差
        fEex = (fA[1] * fEaz - fA[2] * fEay)
        fEey = (fA[2] * fEax - fA[0] * fEaz)
        fEez = (fA[0] * fEay - fA[1] * fEax)

    # 比例积分进行误差处理，没有考虑积分限幅的影响
    if ((fEex != 0.0) and (fEey != 0.0) and (fEez != 0.0)):
        #误差积分部分
        fG_E[0] +=  qCF[0] * fEex * qCF[2]
        fG_E[1] +=  qCF[0] * fEey * qCF[2]
        fG_E[2] +=  qCF[0] * fEez * qCF[2]
        # 误差求解.
        fG[0] += fG_E[0]
        fG[1] += fG_E[1]
        fG[2] += fG_E[2]

    else:
        fG_E[0] = 0
        fG_E[1] = 0
        fG_E[2] = 0

    fG[0] += qCF[1] * fEex
    fG[1] += qCF[1] * fEey
    fG[2] += qCF[1] * fEez

    fQ_=vQuat_Update(fG, qCF[2], fQ)



    return fQ_,fG_E


    
# /**************************************************************************
#     单阶四元数迭代计算，输入fG计算的校正信号（迭代模型的系数），fDt步长，fQ为四元数
# **************************************************************************/


def vQuat_Update(fG, fDt, fQ):

    fQTem=fQ
    fQTem[0] = 0.5 * (-fQ[1] * fG[0] - fQ[2] * fG[1] - fQ[3] * fG[2])
    fQTem[1] = 0.5 * (fQ[0] * fG[0] + fQ[2] * fG[2] - fQ[3] * fG[1])
    fQTem[2] = 0.5 * (fQ[0] * fG[1] - fQ[1] * fG[2] + fQ[3] * fG[0])
    fQTem[3] = 0.5 * (fQ[0] * fG[2] + fQ[1] * fG[1] - fQ[2] * fG[0])
    fQ[0] += fDt * fQTem[0]
    fQ[1] += fDt * fQTem[1]
    fQ[2] += fDt * fQTem[2]
    fQ[3] += fDt * fQTem[3]
    # 归一化
    fInv_N = np.sqrt(fQ[0] * fQ[0] + fQ[1] * fQ[1] + fQ[2] * fQ[2] + fQ[3] * fQ[3])
    fQ[0] *= fInv_N
    fQ[1] *= fInv_N
    fQ[2] *= fInv_N
    fQ[3] *= fInv_N
    return fQ




e_roll = math.pi/3
e_pitch = math.pi/6
e_yaw = math.pi/4
CONSTANT_G = 9.7936




fAngle_Init = np.zeros(3)

fAngle_Init[0]= e_roll
fAngle_Init[1]= e_pitch
fAngle_Init[2]= e_yaw


###保证ax^2+ay^2+az^2=g^2
f_acc_ = np.random.uniform(-1,1,3) #对应x,y,z三轴加速度
ftem = np.power(CONSTANT_G,2)-np.power(f_acc_ [0],2)-np.power(f_acc_ [1],2)
f_acc_[2]=np.sqrt(ftem)

### 磁力计
f_m_=np.zeros(3)
### 角速度
f_gyro_ = np.random.uniform(-1,1,3) #对应三轴角速度
fe_acc = np.zeros(3)
fe_acc[2]=CONSTANT_G 
###


dcm = EularToEBDCM(fAngle_Init[0],fAngle_Init[1],fAngle_Init[2]) ##输入欧拉角得到地面到机体的变换矩阵
print("dcm",dcm)

fQuat = vEularToQuat(fAngle_Init[0],fAngle_Init[1],fAngle_Init[2]) ##欧拉角到四元数；

fEu = vQuatToEular(fQuat) ##四元数到欧拉角

print("angle error",fEu-fAngle_Init)

fdcm = vQuatToEBDCM(fQuat) #### 四元数到dcm无问题，说明欧拉角到四元数无问题

print("dcm error",dcm-fdcm)

dcm = EularToBEDCM(fAngle_Init[0],fAngle_Init[1],fAngle_Init[2])

fdcm = vQuatToBEDCM(fQuat) #### 四元数到dcm无问题，说明欧拉角到四元数无问题
print("dcm error",dcm-fdcm)

fB=vEToB_EXCH(fQuat, f_acc_)

facc=vBToE_EXCH(fQuat, fB)

print("acc_err",f_acc_-facc)

f_a=f_acc_.copy()
fQ=vQuat_Init(f_a, f_m_)

facc=vEToB_EXCH(fQ, fe_acc)


print("acc_err",f_acc_-facc)
# vMahony_Comp_F(fGF, fAF, fMF, qCF, fG_E, fQ):

# vQuat_Update(fG, fDt, fQ):
