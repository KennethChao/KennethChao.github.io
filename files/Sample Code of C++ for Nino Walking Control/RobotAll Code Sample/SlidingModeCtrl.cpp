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

	 另外在減少chattering的部分,本函式使用saturation function,亦可使用其他方式來取代或改善
***************************************************************************************************/
#include "SlidingModeCtrl.h"



void SdParameterInit( void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 初始化所有需要用到的變數

	******************************************************************/
	for(int i=0;i<12;i++)
	{
		SdGainK[i]=0;
		SdLanda[i]=0;
		SdBeta[i]=0;
		SdEnable[i]=false;
	}


	//Hip Roll
	SdGainK[1]=300;
	SdLanda[1]=10;
	SdBeta[1]=0.02;

	SdGainK[7]=300;
	SdLanda[7]=10;
	SdBeta[7]=0.02;

	//Knee Pitch
	SdGainK[3]=300;
	SdLanda[3]=10;
	SdBeta[3]=0.03;

	SdGainK[9]=300;
	SdLanda[9]=10;
	SdBeta[9]=0.03;

			//SdGainK[3]=200;
			//SdLanda[3]=10;
			//SdBeta[3]=0.02;

	//Hip Pitch
	SdGainK[2]=300;
	SdLanda[2]=10;
	SdBeta[2]=0.02;

	SdGainK[8]=300;
	SdLanda[8]=10;
	SdBeta[8]=0.02;

	//Ankle Pitch
	SdGainK[4]=300;//350;
	SdLanda[4]=10;
	SdBeta[4]=0.02;

	SdGainK[10]=300;//350;
	SdLanda[10]=10;
	SdBeta[10]=0.02;
	//Ankle Roll
	SdGainK[5]=300;//350;
	SdLanda[5]=10;
	SdBeta[5]=0.02;

	SdGainK[11]=300;//350;
	SdLanda[11]=10;
	SdBeta[11]=0.02;

	//Enable Section
	SdEnable[3]=1;
	SdEnable[9]=1;

	SdEnable[1]=1;
	SdEnable[7]=1;

	SdEnable[2]=1;
	SdEnable[8]=1;

	SdEnable[4]=1;
	SdEnable[10]=1;

	SdEnable[5]=1;
	SdEnable[11]=1;
}
void SFunction( double* VelDiff,long* EncDiff,double* Result)
{
	/******************************************************************
	input: double* VelDiff(RPM),long* EncDiff(Counts)
	output: double* Result

	Note:
	// 計算Sliding Surface
	   定義:Sliding Surface S = VelError + Landa*PosError
	   在運算式中,有進行單位的轉換
	   VelError: RPM -> Output Vel rad/s
	   PosError: Encoder Count -> Ouput Angle rad
	******************************************************************/
	for(int i=0;i<12;i++)
	{
		if(i!=0)
		Result[i]= VelDiff[i]/2/3.1415926535*60/160/50*34+SdLanda[i]*(double (EncDiff[i]) )/78353.2027529;//error
		else
		Result[i]= VelDiff[i]/2/3.1415926535*60/160/50*34+SdLanda[i]*(double (EncDiff[i]) )/74896.4438101;
	}
}

void SdBengbengControl(double* Sfunc,double* Result  ,short* ResultBuf)
{
	/******************************************************************
	input: double* Sfunc,double* Result 
	output: double* Result

	Note:
	// 計算Variable Structure 補償項
	   定義:Compensation = GainK * Sat(S/Beta)

	   Sat(X/Beta) Saturation Func:
	   Sat(X/Beta) = 1 when X>=Beta
				   = -1when X<=Beta*-1
				   = X/Beta when -Beta<X<Beta
	******************************************************************/
	for(int i=0;i<12;i++)
	{
		Result[i]=0;
					
		double STR=0;
		if(SdEnable[i]==1)
		{
			if(   (Sfunc[i])/SdBeta[i]>1)
				STR= SdGainK[i];
			else if ((Sfunc[i])/SdBeta[i]<-1)
				STR= SdGainK[i]*-1;
			else
				STR=SdGainK[i]* (Sfunc[i])/SdBeta[i];

			Result[i]=( (STR));//short (Landa*VelDiff[i+12*LogEncCount]/2/3.1415926535*60
		}

		if(SdEnable[i]==1)
			ResultBuf[i]=short(Result[i]);
		else
			ResultBuf[i]=0;//short(Result[i]);
	}

}