/**************************************************************************************************
Copyright, 2013-2015, Robotics Lab., Dept. of M.E., National Taiwan University
File Name: SlidingModeCtrl.h

Author: Kenneth Yi-wen Chao
Version: 0.85
Date: 2013/07/13 by Slongz

Functions:
     SdParameterInit();SFunction();SdBengbengControl();

Classes: N/A

Description:
     本函式庫包含了Computed Torque Control中Vairable-structure compensation項的計算與建立
	 計算採用的方式為基本的Sliding mode control
	 使用目的為增加Controller的robustness,利用此項來補償unmodeled dynamics or uncertainties,加強position tracking的效果
	 (trade off: chattering)
	 此補償項亦可藉由其他robust control的方法來計算而得

Note: 
	 在非線性度高的系統中,需配合feedbacklinearization
	 參數需依照實驗情況來進行調整,尚未最佳化
	 依照walking phases的不同,或許可以採用swithcing control的方式進行參數的調整
***************************************************************************************************/

#ifndef SD_H //使用參數之初始化
#define SD_H
double SdGainK[12]={0,0,0,0,0,0,0,0,0,0,0,0 };//
double SdLanda[12]={0,0,0,0,0,0,0,0,0,0,0,0 };//
double SdBeta[12]={0,0,0,0,0,0,0,0,0,0,0,0};
bool SdEnable[12]={0,0,0,0,0,0,0,0,0,0,0,0};
#endif

void SdParameterInit( void); //參數設定
void SFunction( double* VelDiff,long* EncDiff,double* Result); //計算Sliding Surface的值
void SdBengbengControl(double* Sfunc,double* Result  ,short* ResultBuf);//計算補償項
