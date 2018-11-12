/**************************************************************************************************
Copyright, 2010-2012, Robotics Lab., Dept. of M.E., National Taiwan University
File Name: RobotAllDlg.cpp

Author: Many People
Version: 1.0
Date: 2012/03/20

Functions:
      gIKStep() gThreadTest() gInitTurnLeft()
      gInitWalkStraight() gInitStepHere()
      gInitStair() gInitSquat() gReadSkinSensor()
	  gInitDownStair()
	  gintneck() 
	  所有按鈕
	  OnBnClickedButton1(); // Start Button
	  OnBnClickedButton2(); // Auto Button
	  OnBnClickedButton3(); // Go Button
	  OnBnClickedOk(); // OK 按鈕 結束程式用
	  OnBnClickedCancel(); // 此按鈕不使用 節省空間
	  OnBnClickedButton4(); // 被整合到 Auto button 中 會自動被壓下
	  OnBnClickedButton5(); // 被整合到 Auto button 中 會自動被壓下
	  OnCbnSelchangeCombo1(); // 設定手動模式時 要對機器人進行的動作
	  OnBnClickedButton6(); // 按鈕 "Send" 手動模式專用
	  OnBnClickedButton7(); // 按鈕 PMS/BMS
	  OnBnClickedButton8(); // 按鈕 Init_PBMS
	  OnBnClickedButton9(); // 按鈕 PMS/BMS Save
	  OnBnClickedCheck1(); // 切換 模擬 ADAMS 實驗 三種模式
	  OnBnClickedCheck2(); // 切換 模擬 ADAMS 實驗 三種模式
	  OnBnClickedCheck3(); // 切換 模擬 ADAMS 實驗 三種模式
	  OnBnClickedCheck4(); // Checkbox --> Read Encoder
	  OnBnClickedCheck5(); // Checkbox --> Read 6 axis force sensor
	  OnBnClickedCheck6(); // 是否繪製ZMP紅色小球
	  OnBnClickedCheck11(); // 是否繪製OpenGL model

Classes: None

Description:
     本程式是整個程式的GUI控制核心，所有按鈕與事件
	 所有人機介面都由這個程式來控管
	 各函式與變數之說明請詳見下方宣告與定義處之說明

Note: None
***************************************************************************************************/

#include "stdafx.h"
#include "RobotAll.h"
#include "RobotAllDlg.h"
#include "afxdialogex.h"
#ifdef _DEBUG
#define new DEBUG_NEW
#endif
#include "SoundPlay.h" // 聲音模組

#include <iostream>
#include <stdio.h>
#include <conio.h>//掃描鍵盤用
using namespace std;

// CAboutDlg dialog used for App About
#include "MainLoops.h"
#include "serial_port.h"
#include "DataProcess.h"


#if TwinCAT_Mode
#include "TwinCAT_COM.h"
TwinCAT_COM *TCAT;
#endif

#ifndef NECK_H
#define NECK_H
#include "neck.h"
#endif

#ifndef ESTIMATE_H
#define ESTIMATE_H
#include "estimate.h"
#endif

#ifndef IMU_H
#define IMU_H
#include "IMU.h"
#endif
	//0529
	double ZMPRatio=1;//0.75
	//Slongz 20130516
	bool gUpStair = false;
	//Slongz 20130516

	CRobotAllDlg* gDlg; // 取出視窗本身的指標
	// 手臂離線軌跡讀檔BUFFER
	double gRHandTrajFileBuf[40000][6];
	double gLHandTrajFileBuf[40000][6];
	//************torqueoffset buffer DORA
	const int gDataTotal = 60000;//8816;//64344;//trajectory length
	/*long TempCMD[12][DataTotal];
	long TempENC[12][DataTotal];*/
	double gTorqueOffsetFileBuf1[gDataTotal*12];
	double gTorqueOffsetFileBuf2[gDataTotal*12];
	double gTorqueOffsetFileBuf3[gDataTotal*12];
	short gShortTorqueOffsetFileBuf1[gDataTotal*12];
	short gShortTorqueOffsetFileBuf2[gDataTotal*12];
	short gShortTorqueOffsetFileBuf3[gDataTotal*12];

	//*************

	// 此變數決定是否進行模擬模式
	// 模擬模式不控制機器人 不開com port
	// 在GUI也可以改變此變數值
	int gFlagTimer; // 初始值設在 OnInitDialog()
	int gFlagSimulation;	// 初始值設在 OnInitDialog()
	int gFlagReadEncoder;	// 初始值設在 OnInitDialog()
	int gFlagReadForceSensor;	// 初始值設在 OnInitDialog()
	int gFlagInfrared = 0;	// 初始值設在 OnInitDialog()
	int gFlagManualMode;
	int gFlagGoPass =1;
	int gFlagWelcomeSaid = 0; // 記憶第一句話說了沒

	bool gFlagStairSaid = false; // 記憶第一句話說了沒
	bool gFlagSlSaid01 = false; // 記憶第二句話說了沒
	bool gFlagSlSaid02 = false; // 記憶第三句話說了沒
	bool gFlagSlSaid03 = false; // 記憶第四句話說了沒
	//int gKineAll.FlagSumoMode = 0; // 若要長時間單腳站立 需要開啟這個模式
	int gFlagSendTraj = 0;//讓motioncontrolthread告訴armcontrolthread說要送軌跡了
	extern bool gSetPIDDone; // PID是否設定完成旗標
	

	int gFlagEmergentStop = 0; // 
	bool gFlagArmCtrl = 0; // 
	bool gFlagHandCtrl = 0;// 至峻 20130410
	bool gFlagHandStart = 0;// 至峻 20130410
	//泓逸測試用變數
	double TestDeg[2000000];
	int TestPos = 0;

	//泓逸start120228
	//跟adams溝通部分
	void Trans2Matlab(void);
	double Trans2MatlabData[26];
	double Trans2MatlabLastData[26];
	//泓逸end120228

//===============
// Timer
//===============
	void gInitTimerAndThread(void); // 初始化timer以及thread 注意!! 目前本程式不使用timer了
	void gTestHighTimer(void); // 取得電腦時脈等資訊 到時候計時要用
	void gSaveJointData(void); // 記住當下解出的joint data 以及轉換成C32聽得懂的格式 準備傳輸出去

	void gOnSizeMyGLPMS(int w, int h); // PMS視窗改變大小時的callback函數
	void gRenderPMSThread(void); // 繪製PMS視窗的函數 可以自行指定FPS
	void gRenderPMSEmpty(void); // 空函式 不要讓GL自動重新畫圖來干擾動作
	//void gReadSkinSensor(void); // 讀取皮膚感應值
	//void gReadLegEncoder(void); // 讀取雙腳Encoder 改用 TCAT->EtherCATReadEncoder
	void gCalculateZMP(void); // 依照當前狀況重新計算Walking pattern
	void gCalculateCOG(double COGDown); // 依照當前狀況重新計算Walking pattern
	void gRefreshControlSystem(void); // 初始化控制系統，讓機器人可以執行下一段軌跡
	void gPrepareScenarioScript(int SecNo); // 選擇動作分段要做的事情

	const int gnBufSize = 256; // 印出CPU時脈變數的buffer長度
	TCHAR gchBuf[gnBufSize]; // 印出CPU時脈變數的buffer

	////20120914//////
	void gCalculate_NEW_ZMP(void); //新ZMP軌跡
	////20120914/////

	//邊走邊跨 doratom///
	void gStepWhileWalk(void);

	//跨一步 doratom//
    void gInitWalkOneStep(void);
    int checkonestep = 0;
  
	// EPOS3 joint limit warning
	void gWarningJointLimit(long * buf);
	int gJointUpLimit[12] = { 61600, 21600, 98000,124000, 83000, 23000, 30800, 44000, 18000,  14000, 53000, 46000}; // 腳上限
	int gJointLoLimit[12] = {-30800,-44000,-18000,-14000,-53000,-46000,-61600,-21600,-98000,-124000,-83000,-23000}; // 腳下限

	//只有一腳踩上障礙物doratom//
	//20120925//
	void gInitWalkOneStepHigh(void);
	int checkWalkOneStepHigh = 0;
	int this_step =0;
	//20120925//

	//走斜坡doratom//
	void gWalkSlopeDora(void);//走斜坡的程式
	void gWalkSlopeDora1(void);//走斜坡的程式
	float slopeangle;
	double gRRotAngPitch[10000]; //右腳腳底板pitch轉的角度
	double gLRotAngPitch[10000]; //左腳腳底板pitch轉的角度
	int check_slopeangle = 0; //走斜坡的時候 這個旗標會被舉起來 正常應宣告為0
	double rotate_pitch_time_ratio;//要早一點把角度旋轉完 不然很容易因為還沒旋轉完而撞到斜坡
	//20121214//
	void gInitStaySquatMode(double StayTime);//機器人可以在motioncontrolthread中，只有第二段不動的模式

	//20130422左腳踏一步上去 然後左腳再踏下來
	void gInitLeft_leg_up_and_down_OneStep(void);
	void gInitLeft_leg_up_and_down_TwoStep(void);
    bool checkLeft_leg_up_and_down_OneStep = 0;
    bool checkLeft_leg_up_and_down_TwoStep = 0;
	//20130422//

	// 脖子機構 哲軒 20130319
	void gInitneck( int mode ); //脖子程式
	
    void gIntLEDFace (int mode ) ; //LED FACE
	
	int  theta ;
    int  thetastart ;// 初始角度
    int  thetafinal; // 最終角度 
    int  neck_omega ; //轉動速度  300~ 500  一般速度300  
	int  neck_point ;// 初始角度與最終角度間的內差點數
    int  neck_motor ;// 控制脖子機構的馬達index  上馬達1  下馬達0
	neck neck1;
	IMU IMU1 ;  
	estimate COGestimate;

	extern bool gSendContTraj; // 腳是否要傳輸連續軌跡之旗標
	extern bool gSendContTrajArm; // 手臂是否要傳輸連續軌跡之旗標

	//extern SerialPort *gpPortLL; // 左腳COM PORT
	//extern SerialPort *gpPortRL; // 右腳COM PORT
	extern SerialPort *gpPortLA; // 左手COM PORT
	extern SerialPort *gpPortRA; // 右手COM PORT
	extern SerialPort *gpPortTorso; // 腰COM PORT
	//extern SerialPort *gpPortLSkin; // 左手皮膚 PORT
	//extern SerialPort *gpPortRSkin;  // 右手皮膚 PORT
	extern SerialPort *gpPortHead; // 頭部port
	 
	SerialPort *gneck; 	  //脖子port

	//======================// 至峻 20130410=================================
	///////////////////////////////////////////////////////////////////////////
	extern SerialPort *gpPortLH; // 左手掌COM PORT
	extern SerialPort *gpPortRH; // 右手掌COM PORT
	extern unsigned char *gfContTrajDataRH;
	extern unsigned char *gfContTrajDataLH;
	extern int gfContTrajLenRH;
	extern int gfContTrajLenLH;
	//////////////////////////////////////////////////////////////////////////

	extern int gContTrajLen; // 連續軌跡長度 (腳)
	extern int gStopTrajLen; // 傳送停止軌跡位置
	extern DataProcess *gpDP ; // 處理C32命令轉換物件

	LARGE_INTEGER gGlobalTime; // 電腦計算時間開始點 for control 總時間
	LARGE_INTEGER gGlobalCurrentTime; // 現在global時間


	LARGE_INTEGER gStartTime; // 電腦計算時間開始點 for control
	LARGE_INTEGER gStartTimePMS; // 電腦計算時間開始點 for pms
	LARGE_INTEGER gCurrentTime; // 電腦計算時間現在點
	LARGE_INTEGER gnFreq; // CPU頻率 使用64位元儲存
	float gFreqT = 0; //// CPU頻率 使用float儲存 因為要相除
	bool gStartTimeAcquired = 0; // 已取得初始時間旗標
	bool gStartTimeAcquiredPMS = false; // 已取得初始時間旗標 for pms
	double gSysTime = 0; // 計算出來的系統時間 單位是秒
	unsigned int DataCount = 0; // 不包含初始動作的counts
	unsigned int TotalDataCount = 0; // 包含初始動作的counts
	unsigned int TickNumber = 0; // 包含Motion Ctrl Thread的三段counts
	
	bool gContTrajLock = 0; // 防止不小心連續兩次押到 traj模式 導致機器人暴走
	bool gContTrajLockArm = 0; // 防止不小心連續兩次押到 traj模式 導致機器人暴走
	
	extern int gGLID; // GL視窗的ID 將來殺掉視窗的時候要傳入function
	extern bool gIKGLOpened; // IK的GL視窗是否打開了的旗標

	bool gPMSGLOpened = false; // PMS的GL視窗是否打開了的旗標

	bool gRenderPMSWorking = false; // 讓兩個thread不要互相搶繪圖資源 這樣GL會混亂
	bool gRenderKineWorking = false; // 讓兩個thread不要互相搶繪圖資源 這樣GL會混亂

	extern HANDLE gThreadRender; // IK視窗 thread handle
    extern DWORD gTIDRender; // IK視窗 thread ID
	extern bool gRenderLife; // IK視窗 是否工作旗標

	extern HANDLE gThreadLaserRender; // IK視窗 thread handle
    extern DWORD gTIDLaserRender; // IK視窗 thread ID
	extern bool gLaserRender; // IK視窗 是否工作旗標

	extern HANDLE gThreadControl; // control視窗 thread handle
    extern DWORD gTIDControl;// leg control thread ID
	extern bool gControlLife; // leg control 是否工作旗標

	extern HANDLE gThreadFace; // control視窗 thread handle
    extern DWORD gTIDFace;// leg control thread ID
	extern bool gFaceLife; // leg control 是否工作旗標


	extern HANDLE gThreadArmControl; // arm control thread handle
    extern DWORD gTIDArmControl;//  arm control thread ID
	extern bool gArmControlLife;//  arm control 是否工作旗標

	//Slongz 0218
	extern HANDLE gThreadTimer; //  Timer thread handle
    extern DWORD gTIDTimer;// Timer thread ID
	extern bool gTimerLife; // Timer 是否工作旗標
	//Slongz 0218

	bool gRenderPMSThreadOpened = false; // PMS 繪圖視窗是否已經打開之旗標

	bool gRenderThreadOpened = false; // 指示IK繪圖thread是否打開
	bool gControlThreadOpened = false; // 指示腳部控制thread是否打開
	bool gArmControlThreadOpened = false; // 指示手臂部控制thread是否打開
	bool gFaceControlThreadOpened = false; // 指示表情控制是否打開
	bool gLaserRenderThreadOpened = false; // 指示表情控制是否打開
	bool gIMUThreadOpened = false;         // 指示IMU是否打開
	//_______________________________________(Dora)_________________________________________
	#if LaserCatch
	extern HANDLE gThreadLaserRender; // 畫圖thread handle
    extern DWORD gTIDLaserRender; // 畫圖thread ID
	extern bool gLaserRenderLife; // 是否結束繪圖視窗

	laser* eye;
	int laserbufX[681];
	int laserbufY[681];

	#endif
	//_______________________________________(Dora)_________________________________________

		//chehsuan 20130508
//#if IMUCatch 
	extern HANDLE gThreadIMU; //  Timer thread handle
	extern DWORD gTIDIMU;// Timer thread ID
	extern bool gIMULife; // Timer 是否工作旗標
	int startIMU = 0 ; //開始啟動IMU的旗標
	int IMUglplot = 1 ;  //開始啟動IMUGL 繪圖旗標
//#endif
	//chehsuan 20130508



	extern HANDLE gThreadRenderPMS; //  PMS視窗 thread handle
	extern DWORD gTIDRenderPMS; // PMS視窗 thread ID
	extern bool gRenderPMSLife; // PMS視窗 是否工作旗標

	extern bool gDrawingBufCreated; // 指示drawing buffer 是否已經創建
	extern int gDrawingSeqSize; // 繪圖buffer size
	extern int gDrawingSeqSizeArm; // 繪圖buffer size for arm
	extern float* gpRobotDrawingBuffer; // 儲存機器腳繪圖資料
	extern float* gpRobotDrawingBufferArm; // 儲存機器手繪圖資料
		   
	Kine gKineAll; // 機器人所有運動鍊
	LQSISolver gLQs;  // LQSI controller gLQs.LQDataLen 要在各個初始化的時候指定 詳見start button

	double gInpCOG[LQSIBufferSize]; // 機器人重心高度軌跡
	double gInpZMPx[LQSIBufferSize];	 // 機器人ZMPx方向軌跡
	double gInpZMPy[LQSIBufferSize];	 // 機器人ZMPy方向軌跡
	bool gCOGZMPInitialized = false; // 指示COG ZMP 初始化是否完成

	//泓逸start120222
	//以下六個存放走路時擺手軌跡
	double gRArmWalkX[LQSIBufferSize];
	double gRArmWalkY[LQSIBufferSize];
	double gRArmWalkZ[LQSIBufferSize];
	double gLArmWalkX[LQSIBufferSize];
	double gLArmWalkY[LQSIBufferSize];
	double gLArmWalkZ[LQSIBufferSize];
	double gRArmWalkRot[LQSIBufferSize*9];
	double gLArmWalkRot[LQSIBufferSize*9];

	double gRArmOfflineTraj[LQSIBufferSize*6];//這可以offline丟軌跡
	double gLArmOfflineTraj[LQSIBufferSize*6];//這可以offline丟軌跡

	int gArmOfflineTrajCount = 0;
	bool AfterSignLanguageFlag = 0;


	
	//泓逸end120222

	double gInpZMPHeight[LQSIBufferSize]; //機器人ZMP軌跡高度
	double gInpInvPendulumHeight[LQSIBufferSize];  //機器人倒單擺高度軌跡

	int gNumOfStep = 0; //預設為0 在 start按鈕裡面會依照情境改
	double gCOGDown	= 0;  //預設為0 在 start按鈕裡面會依照情境改
	//Slongz
	double gCOGUp	= 0;  //預設為0 在 start按鈕裡面會依照情境改
	//Slongz

	//哲軒20121127
	 double stairheight = 0 ;// 樓梯高度
    //哲軒20121127

	int mode1; 
	int infraredcalicount  =0 ; 
	
	//上下樓梯的內插模式   2012 哲軒
	double gAngChange = 0.0;	//預設為0 在 start按鈕裡面會依照情境改

	int gStepSample	= (T_P/dt); // 每步的sample數
	//int gTrajLength	= (T_P/dt*4.0); // preview 4步

	//泓逸start120409
	//這個變數用來存放轉彎時手部要轉的角度
	double gDth1[10000];
	double gDth2[10000];

	double gAngRArm = 0;//右手現在角度
	double gAngLArm = 0;//左手現在角度

	double gLastAngRArm = 0;
	double gLastAngLArm = 0;

	int gDemoArmTrajFlag = 0;//用來判別arm不同traj用的(demo用的) 0代表直走 1代表左轉
	int gIKMethod = 0;//用來判別要用哪種IK 0代表解手腳共24軸 1代表只解腳12軸
	int gTurnDirection = 0;//用來判別要左轉還是右轉 0代表左轉 1代表右轉
	//泓逸end120409


	//Slongz 20130417
	bool ArmOfflinMethod = 0;
	//Slongz 20130417
	int gTrajLength	= (T_P/dt*gNumOfStep); // gNumOfStep步

	unsigned char gPIDUpdLSup[40]; // 值被設在按下initialize的時候
	unsigned char gPIDUpdRSup[40]; // 值被設在按下initialize的時候
	unsigned char gPIDUpdDSup[40]; // 值被設在按下initialize的時候

	int gSupLegNow = 2; // 指示現在的support腳狀態 (SSP DSP)
	int gSupLegNNext = 2;// 指示下一個的support腳狀態 (SSP DSP)
	bool gChgPIDSent = false; // 指示在步行周期內是否有送出更改的PID數值


	// for online IK
	unsigned int gIthIK = 0; // 記住現在是第幾格IK
	double gXcogIK[3]; // 輸入IK COG 目標軌跡軌跡點
	double gSwingInputIK[3]; // 輸入IK swing 目標軌跡點


	//泓逸start111227
	double gLArmInputIK[3];//輸入IK LArm 目標軌跡點
	double gRArmInputIK[3];//輸入IK RArm 目標軌跡點
	//泓逸end111227



	void gMotionControlThread(void); // 用於控制機械腳的thread所執行的函數
	void gArmControlThread(void);  // 用於控制機械手的thread所執行的函數
	void gFaceControlThread(void);  // 用於控制臉部的thread所執行的函數
	void gLaserRenderThread(void);  // 用於控制Laser的thread所執行的函數
	void gTimerThread(void); // 用於控制Timer的thread所執行的函數
	void gIMUThread(void)  ; // 用於控制IMU的thread所執行的函數
	// online IK 變數
	//double rem_motion[12][20000]; //
	int gMotionIndex = 0; // 記住motion到第幾格

	int BufferLen = 240; // 傳送20ms buffer長度，要跟下面創的array一樣大!!
	unsigned char gShortTrajBuf[240]; // 傳送20ms 命令用buffer
	unsigned char gShortTrajBufLLeg[120]; // 傳送20ms 命令用buffer
	unsigned char gShortTrajBufRLeg[120]; // 傳送20ms 命令用buffer

	int BufferLenLArm = 120; // 傳送20ms buffer長度，要跟下面創的array一樣大!!
	int BufferLenRArm = 120; // 傳送20ms buffer長度，要跟下面創的array一樣大!!
	int BufferLenTorso = 40; // 傳送20ms buffer長度!!

	unsigned char gShortTrajBufLArm[120]; // 傳送20ms 命令用buffer
	unsigned char gShortTrajBufRArm[120]; // 傳送20ms 命令用buffer
	unsigned char gShortTrajBufTorso[120]; // 傳送20ms 命令用buffer
	unsigned char gShortTrajBufLArmSent[120];//因為手臂要用10ms丟這個是真正要丟的軌跡
	unsigned char gShortTrajBufRArmSent[120];//因為手臂要用10ms丟這個是真正要丟的軌跡
	int gArmTrajFleg = 0;//當這個fleg為0時代表第一次進入ArmControlThread為1時代表第二次進入這時才真正要送軌跡


    //int gPNJoints[12] = {-1,1,1,-1,1,-1,-1,1,1,-1,1,-1}; // 轉化IK裡面的軸向跟機器人真實的軸向
    //int gPNJoints[12] = {1,1,1,1,1,1,1,1,1,1,1,1};
	//                   1 2 3 4  5 6  7 8 9 10 11 12

	//泓逸start111226
	//int gPNJoints[26] = {-1,1,1,-1,1,-1,-1,1,1,-1,1,-1,1,1,1,1,1,1,1,1,-1,-1,1,-1,1,1};//DH中正反轉方向和機構正反轉方向關係
	int gPNJoints[26] = {1,-1,-1,1,-1,1,1,-1,-1,1,-1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,1,1};//DH中正反轉方向和機構正反轉方向關係
	int gOfflineJoint[12] = {1,1,1,1,1,1,-1,-1,-1,-1,1,1};
	//{LL,RL,T,LA,RA}
	//泓逸end111226

	int gWalkStartPoint = 0;//算zmp時要記得是從哪個點開始走的

	int gBufIndex = 0; // IK要填寫值給 傳送之buffer 

	//泓逸start20120308
	int gBufIndexTorso = 0;
	int gBufIndexLArm = 0;
	int gBufIndexRArm = 0;

	double gDeltaLArmX = 0; //計算內差完之後手末端和重心的偏移量
	double gDeltaLArmY = 0;
	double gDeltaLArmZ = 0;
	double gDeltaRArmX = 0;
	double gDeltaRArmY = 0;
	double gDeltaRArmZ = 0;
	//泓逸end20120308
	long gTempEnc; // encoder value
	long gTempEncs[12]={0,0,0,0,0,0,0,0,0,0,0,0};

	// 把機構從0度~初始角度的內插動作
	double gInitThetas[26];

	int gInterNum = (5/dt);//1000; // 內插格數
	double gInitMotion[26][int(5/dt)]; // 注意!! 第二個維度要跟gInterNum設一樣

	int gInterNumEnd = 1000;


	double gInitTime = gInterNum*dt; // 機器人初始的時間長度 單位是秒
	int gInitCount = 0; // 計算到第幾格

	char gGlobalMessage[60]; // GL視窗左上角會post出來的訊息

	int gIKStatus = 1; // 1 = succeed;  0 = fail;

    unsigned char gBufIn[500]; // 讀取機器人回傳用之Buffer
    //int gReadEnc[26][LQSIBufferSize]; // 儲存所有機器人回傳之ENC資料 半小時

	// PMS/BMS
	int gGL_ID_PMS; // 拿掉底線會造成混淆 所以如此命名
	int gPMSSizeX; // PMS繪圖size資訊
	int gPMSSizeY; // PMS繪圖size資訊

	// COM port for PMS
	SerialPort* gPMS_LArm; // 左手PMS COM PORT
	SerialPort* gPMS_RArm; // 右手PMS COM PORT
	SerialPort* gPMS_LLeg; // 左腳PMS COM PORT
	SerialPort* gPMS_RLeg; // 右腳PMS COM PORT
	SerialPort* gPMS_Head; // 頭PMS COM PORT
	SerialPort* gBMS; // 電池PMS COM PORT
	SerialPort* gIMU; // IMU PORT

	// 初始電壓電流溫度值 數字是用來debug用的 真實使用之時 會隨著真實情況被複寫 不用擔心
	double gV_LArm = 22.8;
	double gV_RArm = 21;
	double gA_LArm = 3;
	double gA_RArm = 2.5;
	double gV_LLeg = 44;
	double gV_RLeg = 44;
	double gA_LLeg = 6;
	double gA_RLeg = 12;
	double gV_Head = 0.1;
	double gA_Head = 0.04;
	double gV_24Battery = 23.5;
	double gA_24Battery = 6.4;
	double gV_48Battery = 46;
	double gA_48Battery = 12;
	double gT_24Battery = 32;
	double gT_48Battery = 33;

	//泓逸start120224

	double gForceDataLLeg[SensorBufferSize*10]; // force sensor data for left leg    25 mins
	double gForceDataRLeg[SensorBufferSize*10]; // force sensor data for right leg    25 mins
	//double gRangeDataLLeg[SensorBufferSize]; // range sensor data for left leg    25 mins
	//double gRangeDataRLeg[SensorBufferSize]; // range sensor data for right leg    25 mins
	double gEnc_FKCOG[SensorBufferSize];	// 利用ENC FK FindCOG得到的COG位置 存COG的位置
	
	
	int gForceDataCount = -1;	//20130528 更改為-1 因為在進入PMS的頭++ 將傳值之前的力規值洗掉 Orig = 0
	int gRangeDataCount = 0;
	double gIMUResult[SensorBufferSize];//IMU傳上來的cmd
	int gIMUCount = 0;

	//double PosCalZMP[SensorBufferSize];//當力規值紀錄時記住當時位置
	//int PosDataCount = 0;

	//泓逸end120224

	// Arm motions
	int gLAMotionIndex = 0; // 記住左手動到的格數
	int gRAMotionIndex = 0; // 記住右手動到的格數
	int gTorsoMotionIndex = 0; // 記住腰動到的格數
	int gLAMotionLen = 0; // 左手軌跡總長
	int gRAMotionLen = 0; // 右手軌跡總長
	int gTorsoMotionLen = 0; // 腰軌跡總長
	unsigned char gLAMotion[30][LQSIBufferSize]; // 儲存左手軌跡
	unsigned char gRAMotion[30][LQSIBufferSize]; // 儲存右手軌跡
	unsigned char gTorsoMotion[30][LQSIBufferSize]; // 儲存腰軌跡
	// Arm motions

	double gAngLBuf[10000]; // 左腳每步旋轉角度軌跡 提供最長每步50秒之暫存空間 用以儲存單步軌跡
	double gAngRBuf[10000]; // 右腳每步旋轉角度軌跡 提供最長每步50秒之暫存空間 用以儲存單步軌跡

	//20121214doratom//
	double gPitchAngLBuf[10000]; // 左腳pitch每步旋轉角度軌跡 提供最長每步50秒之暫存空間 用以儲存單步軌跡
	double gPitchAngRBuf[10000]; // 右腳pitch每步旋轉角度軌跡 提供最長每步50秒之暫存空間 用以儲存單步軌跡
	//20121214doratom//

	double gGroundHeight[10000]; // ground height, 10000步

	int gNza; // 將一步切成三段 第一段格點數  gNza = gStepSample * DSP /2.0
	int gNab; // 將一步切成三段 第二段格點數 gNab = gStepSample * SSP
	int gNzb; // 將一步切成三段 第三段格點數  gNb = gStepSample - gNza - gNab

	double gStrideX; // 機器人每踏一步 X方向 大小
	double gStrideY; // 機器人每踏一步 Y方向 大小
	double gStrideZ; // 機器人每踏一步 Z方向 大小

	double gLLInitZMPFB = 0; // initial zmp reference
	double gRLInitZMPFB = 0; // initial zmp reference
	double gLLInitZMPLR = 0; // initial zmp reference
	double gRLInitZMPLR = 0; // initial zmp reference


	bool gFlagZMPPlot = 0; // 是否繪製出ZMP位置小紅球之旗標

	double gWaitInitSpeechTime = 0.0; // second

	int gEncoderReadIndex = 0;

	bool gFlagBoostSimu = false; // 旗標: 是否不管時間直接全速跑IK
	bool gFlagIMU = false; // 是否讀取IMU旗標

	bool gFlagGLMode = false; // 是否讀取OPENGL model旗標

	double gTotalInitCOG[3]; // 腳是直的 完全初始時候的COG
	double gDHInitCOG[3]; // 到達DH初始值 機器人不再singular 的 初始COG

	double gLAngZWorld = 0; // Angle of the LLeg in  world coordinates
	double gRAngZWorld = 0; // Angle of the RLeg in  world coordinates

	int gFlagGoBackPos = 1; // 0 for zero point (完全沒蹲), 1 for initial point (剛蹲完)
	int gFlagCurrentPos = 0; // 0 for zero point (完全沒蹲), 1 for initial point (剛蹲完)

	/////////////ZMP Traj Planning///////////////////////
	// FootPrint Data
	double gFstpX[10000];  // 每步腳步位置 X 世界座標 以橫軸為X 但在真正計算Stride時轉換座標以StrideY表示 故是正負跳動
	double gFstpY[10000];  // 每步腳步位fg置 Y 和上面一樣 故是一直往前
	double gLRotAngZ[10000]; // 左腳在z方向旋轉角度(rad)
	double gRRotAngZ[10000]; // 右腳在z方向旋轉角度(rad) 

	double gRotAngBody[10000]; // 身體在Z方向旋轉角度

	double gDirRFoot[3][10000]; // 右腳在世界中的方向向量
	double gDirLFoot[3][10000]; // 左腳，同上
	double gDirBody[3][10000]; // 身體，同上

	double gDirLLateral[3][10000]; // 左腳在世界中的側向方向
    double gDirRLateral[3][10000]; // 右腳，同上

	double gLateralZMPShift = 0; // 側向ZMP移動量 for 自然ZMP
	double gSagittalZMPFront = 0.0; // 前方ZMP移動量 for 自然ZMP
	double gSagittalZMPBack = 0.0;  // 後方ZMP移動量 for 自然ZMP

	double gPFront[3][10000]; // 記住現這一步的 離開 ZMP 位置點
	double gPCenter[3][10000]; // 記住現這一步的 中心 ZMP 位置點
	double gPBack[3][10000]; // 記住現這一步的 進入 ZMP 位置點
	double gPAll[3][30000]; // 將上面三種點都串再一起

	#if ConstantCOGMode
		double gDivideVel = 6;//12;//6 // 長度除以 gDivideVel 當作速度
	#else
		double gDivideVel = 12; // 長度除以 gDivideVel 當作速度
	#endif


	double varSSP1FB[10000]; // 存放多項式生出的內插點 要乘上的量 SSP 第一段 左右
	double varSSP1LR[10000]; // 存放多項式生出的內插點 要乘上的量 SSP 第一段 前後
	double varSSP2FB[10000]; // 存放多項式生出的內插點 要乘上的量 SSP 第二段 左右
	double varSSP2LR[10000]; // 存放多項式生出的內插點 要乘上的量 SSP 第二段 前後
	double varDSP[10000]; // 存放多項式生出的內插點 要乘上的量 DSP 

	/////////////////////////////////////////////////////

	///////////////////語音模組///////////////////////////
	bool gFlagSpeechModule = false; // 是否啟動語音模組 
	CSoundPlay gSpeaker; // 機器人語音物件
	/////////////////////////////////////////////////////

	/////////////////////皮膚模組//////////////////////////
	//bool gFlagSkinModule = false; // 是否啟動皮膚模組
	//unsigned char gCmdReadSkin[1] = {65}; //  65 = 讀取命令 到時候直接傳入指標給COM Port
	//unsigned char gRArmSkin[129]; // 右手皮膚 RS232回傳陣列
	////unsigned char gLArmSkin[129]; // 左手皮膚 RS232回傳陣列
	//int gRSkinArray[64]; // 右手皮膚 Sensor 值
	////int gLSkinArray[64]; // 左手皮膚 Sensor 值

	//////////////////////////////////////////////////////

	//__________________新ZMP軌跡_________________________________//
	double new_gPFront[3][10000]; // 記住現這一步的 離開 ZMP 位置點
	double new_gPCenter[3][10000]; // 記住現這一步的 中心 ZMP 位置點
	double new_gPBack[3][10000]; // 記住現這一步的 進入 ZMP 位置點
	double new_gPAll[3][30000]; // 將上面三種點都串再一起
	double newvector[3][30000];
	//__________________新ZMP軌跡_________________________________//




	//_______________________________________(Slongz)_______________________________________

	char gBipedInfo1[60]; // GL視窗左上角print訊息
	char gBipedInfo2[60]; // GL視窗左上角print訊息
	char gBipedInfo3[60]; // GL視窗左上角print訊息
	bool LFlag=1; 
	bool RFlag=1; 
	double minL=1000;
	double minR=-1000;
	double WalkRatio=1.0;
	double MinRKnee=1000;
	double MinLKnee=-1000;
	double MaxIterationTime=0;

	bool gFlagPE =0;

	void gDeleteLogFile(void);
	void gRecordLogFile(void);
	void gReadPreloadTorque(void);
	void gReadArmTraj(void);
	int gSecNumb=0;

	//short LogTorqueOffset[12]={0,0,0,0,0,0,0,0,0,0,0,0};

	const int DataTotal = 100000;
	int LogPreLoadCount=0;
	int LogEncCount=0;
	long LogEnc[12*DataTotal];
	long LogVel[12*DataTotal];	
	short LogTorque[12*DataTotal];
	long LogVelBuf[12]={0,0,0,0,0,0,0,0,0,0,0,0};	
	short LogTorqueBuf[12]={0,0,0,0,0,0,0,0,0,0,0,0};

	#if BangBangControl
		double LogTorqueOffset[12*DataTotal];
		double LogVelDiff[12*DataTotal];
		double LogVelDiffBuf[12]={0,0,0,0,0,0,0,0,0,0,0,0};
		double LogDesireVel[12*DataTotal];
		long LogDesireVelBuf[12]={0,0,0,0,0,0,0,0,0,0,0,0};
		double LogSFunction[12*DataTotal];
		long LogMovAveVel[12*DataTotal];
		long LogEncDiff[12*DataTotal];
	#endif

	double LogLanda = 1;

	float LogIMUAngVelMA[3*DataTotal];
	float LogIMUAngVel[3*DataTotal];
	float LogIMUAngVelBuf[3]={0,0,0};

	//IMU  Uprightfeedback 相關  處理掉值問題 20140218 哲軒
	float LogIMUanglex[DataTotal];
	float LogIMUangley[DataTotal];
	float LogIMUvelx[DataTotal];
	float LogIMUvely[DataTotal];
	float LogIMUaccelx[DataTotal];
	float LogIMUaccely[DataTotal];
	double Lfeedbacktheta[6] = {0,0,0,0,0,0};	//  upright feedback 初始等於0
	double Rfeedbacktheta[6] = {0,0,0,0,0,0}; 

	//float LogLaser[2DataTotal];
	//_______________________________________(Slongz)_______________________________________

	bool gEndOfScript = false; // 劇情結束旗標 舉起後機器人停止
	int gNoMotionTime = 0.0; // 機器人不解IK 自我介紹的時間
	double gZeroJointPosShift[26]; // 機器人演說時偷偷重心往後移動 各軸位置

    double gTempPoly[10000]; // 內插軸軌跡時 暫存空間

	////20130308 WeiZh
	int CaliFlagL = 0;
	int CaliFlagR = 0;
	extern double CaliPitchAngL;
	extern double CaliPitchAngR;
	extern double CaliRollAngL;
	extern double CaliRollAngR;
	extern double CaliRollHipL;
	extern double CaliRollHipR;
	extern double CaliPitchHipL;
	extern double CaliPitchHipR;
	int axisNumber;
	double CaliAngle;
	int CaliAngleMode;
	extern double CaliTemp[12];
	////WZ
	unsigned char EposError[12]={0,0,0,0,0,0,0,0,0,0,0,0};//Dora

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// Dialog Data
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

// Implementation
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);

}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


CRobotAllDlg::CRobotAllDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CRobotAllDlg::IDD, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CRobotAllDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_BUTTON1,mButton_Start); // button Start
	DDX_Control(pDX, IDC_BUTTON2,mButton_Auto); // button Auto
	DDX_Control(pDX, IDC_BUTTON3,mButton_Go); // button GO
	DDX_Control(pDX, IDC_BUTTON8,mButton_Init_PBMS); // button Init_PBMS
	DDX_Control(pDX, IDC_BUTTON7,mButton_PMSBMS); // button PMS/BMS
	DDX_Control(pDX, IDC_COMBO1, SendMode); // combo box that sets the mode of communication
	DDX_Control(pDX, IDC_CHECK1, CheckSimu); // 設定模式 C++ simulation
	DDX_Control(pDX, IDC_CHECK2, CheckExp); // 設定模式 真實實驗
	DDX_Control(pDX, IDC_CHECK3, CheckADAMS); // 設定模式 ADAMS模擬
	DDX_Control(pDX, IDC_CHECK4, CheckEncoder); // check box 決定是否要讀encoder
	DDX_Control(pDX, IDC_CHECK5, CheckForceSensor);// check box 決定是否要讀力規與紅外線
	DDX_Control(pDX, IDC_CHECK6, CheckZMPPlot); // 決定是否畫出ZMP
	DDX_Control(pDX, IDC_CHECK7, CheckInfrared); // 決定是否讀取紅外線(皮膚)
	DDX_Control(pDX, IDC_CHECK8, CheckSpeech); // 決定是否說話(目前都會說)
	DDX_Control(pDX, IDC_CHECK9, CheckBoost); // 決定是否全速模擬
	DDX_Control(pDX, IDC_CHECK10, CheckIMU); // 決定是否讀取IMU
	DDX_Control(pDX, IDC_CHECK11, CheckOpenGL); // 決定是否畫出OpenGL model
	DDX_Control(pDX, IDC_CHECK12, CheckManualMode); // 決定是否使用Manual Mode
	DDX_Control(pDX, IDC_CHECK13, CheckPE); // 決定是否使用PE
	DDX_Control(pDX, IDC_BUTTON10, CheckEmergentStop); // Emergent Stop
	DDX_Control(pDX, IDC_CHECK14, CheckArmCtrl); // 決定是否開啟ArmThread ArmControl
	DDX_Control(pDX, IDC_CHECK15, CheckHandCtrl); // 決定是否開啟ArmThread HandControl
}

BEGIN_MESSAGE_MAP(CRobotAllDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BUTTON1, &CRobotAllDlg::OnBnClickedButton1)
	ON_BN_CLICKED(IDC_BUTTON2, &CRobotAllDlg::OnBnClickedButton2)
	ON_BN_CLICKED(IDC_BUTTON3, &CRobotAllDlg::OnBnClickedButton3)
	ON_BN_CLICKED(IDOK, &CRobotAllDlg::OnBnClickedOk)
	ON_BN_CLICKED(IDCANCEL, &CRobotAllDlg::OnBnClickedCancel)
	ON_BN_CLICKED(IDC_BUTTON4, &CRobotAllDlg::OnBnClickedButton4)
	ON_BN_CLICKED(IDC_BUTTON5, &CRobotAllDlg::OnBnClickedButton5)
	ON_CBN_SELCHANGE(IDC_COMBO1, &CRobotAllDlg::OnCbnSelchangeCombo1)
	ON_BN_CLICKED(IDC_BUTTON6, &CRobotAllDlg::OnBnClickedButton6)
	ON_BN_CLICKED(IDC_BUTTON7, &CRobotAllDlg::OnBnClickedButton7)
	ON_BN_CLICKED(IDC_BUTTON8, &CRobotAllDlg::OnBnClickedButton8)
	ON_BN_CLICKED(IDC_BUTTON9, &CRobotAllDlg::OnBnClickedButton9)
	ON_BN_CLICKED(IDC_CHECK1, &CRobotAllDlg::OnBnClickedCheck1)
	ON_BN_CLICKED(IDC_CHECK2, &CRobotAllDlg::OnBnClickedCheck2)
	ON_BN_CLICKED(IDC_CHECK3, &CRobotAllDlg::OnBnClickedCheck3)
	ON_BN_CLICKED(IDC_CHECK4, &CRobotAllDlg::OnBnClickedCheck4)
	ON_BN_CLICKED(IDC_CHECK5, &CRobotAllDlg::OnBnClickedCheck5)
	ON_BN_CLICKED(IDC_CHECK6, &CRobotAllDlg::OnBnClickedCheck6)
	ON_BN_CLICKED(IDC_CHECK8, &CRobotAllDlg::OnBnClickedCheck8)
	ON_BN_CLICKED(IDC_CHECK7, &CRobotAllDlg::OnBnClickedCheck7)
	ON_BN_CLICKED(IDC_CHECK9, &CRobotAllDlg::OnBnClickedCheck9)
	ON_BN_CLICKED(IDC_CHECK10, &CRobotAllDlg::OnBnClickedCheck10)
	ON_BN_CLICKED(IDC_CHECK11, &CRobotAllDlg::OnBnClickedCheck11)
	ON_BN_CLICKED(IDC_CHECK12, &CRobotAllDlg::OnBnClickedCheck12)
	ON_BN_CLICKED(IDC_CHECK13, &CRobotAllDlg::OnBnClickedCheck13)
	ON_BN_CLICKED(IDC_BUTTON10, &CRobotAllDlg::OnBnClickedButton10)
	ON_BN_CLICKED(IDC_CHECK14, &CRobotAllDlg::OnBnClickedCheck14)
	ON_BN_CLICKED(IDC_CHECK15, &CRobotAllDlg::OnBnClickedCheck15)
	ON_BN_CLICKED(IDC_BUTTON11, &CRobotAllDlg::OnBnClickedButton11)
END_MESSAGE_MAP()


// CRobotAllDlg message handlers

BOOL CRobotAllDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// Add "About..." menu item to system menu.

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

	// TODO: Add extra initialization here

	// 設定MFC GL視窗大小
	rect.left   = 5;
	rect.right  = 6;
	rect.top    = 5;
	rect.bottom = 6;

	// 開起MFC GL視窗
	openGLControl.Create(rect,this);
	//SetTimer(0,100,NULL);
	m_scale = 1.0;

	gDlg = this;

	// 預設在cpp模擬模式
	gFlagSimulation = 0; 
	gFlagTimer = MControl; 
	if (gFlagSimulation == 0) // cpp simulation mode
	{
		gFlagReadEncoder = 0;
		gFlagReadForceSensor = 0;
		//gFlagSkinModule = 0;
		gFlagInfrared = 0;
		gFlagSpeechModule = 0;
		gFlagManualMode = 0;
		CheckEncoder.SetCheck(0);
		CheckForceSensor.SetCheck(0);
		CheckInfrared.SetCheck(0);
		CheckEncoder.EnableWindow(0);
		CheckForceSensor.EnableWindow(1);
		CheckInfrared.EnableWindow(0);
		CheckIMU.SetCheck(0);
		CheckIMU.EnableWindow(0);
		gFlagManualMode = 0;
		mButton_Init_PBMS.EnableWindow(false);
		mButton_PMSBMS.EnableWindow(false);
		CheckManualMode.EnableWindow(false);
	}
	else if (gFlagSimulation == 1) // Adams Mode
	{
		gFlagReadEncoder = 0;
		gFlagReadForceSensor = 0;
		//gFlagSkinModule = 0;
		gFlagInfrared = 0;
		gFlagSpeechModule = 0;
		gFlagManualMode = 0;
		CheckEncoder.SetCheck(0);
		CheckForceSensor.SetCheck(0);
		CheckInfrared.SetCheck(0);
		CheckEncoder.EnableWindow(0);
		CheckForceSensor.EnableWindow(1);
		CheckInfrared.EnableWindow(0);
		CheckIMU.SetCheck(0);
		CheckIMU.EnableWindow(0);
		gFlagManualMode = 0;
		mButton_Init_PBMS.EnableWindow(false);
		mButton_PMSBMS.EnableWindow(false);
		CheckManualMode.EnableWindow(false);
	}
	else if (gFlagSimulation == 2) // Experiment Mode
	{
		gFlagReadEncoder = 0;
		gFlagReadForceSensor = 0;
		//gFlagSkinModule = 0;
		gFlagInfrared = 0;
		gFlagSpeechModule = 0;
		gFlagManualMode = 0;
		CheckEncoder.SetCheck(0);
		CheckForceSensor.SetCheck(0);
		CheckInfrared.SetCheck(0);
		CheckEncoder.EnableWindow(1);
		CheckForceSensor.EnableWindow(1);
		CheckInfrared.EnableWindow(1);
		CheckIMU.SetCheck(0);
		CheckIMU.EnableWindow(1);
		gFlagManualMode = 0;
		mButton_Init_PBMS.EnableWindow(true);
		mButton_PMSBMS.EnableWindow(false);
		CheckManualMode.EnableWindow(true);
	}

	if (gFlagSimulation == 0) // C++ simulation
	{
		CheckSimu.SetCheck(1);
		CheckExp.SetCheck(0);
		CheckADAMS.SetCheck(0);
		CheckSimu.EnableWindow(1);
		CheckExp.EnableWindow(0);
		CheckADAMS.EnableWindow(0);
	}
	else if (gFlagSimulation == 1) // ADAMS simulation
	{
		CheckSimu.SetCheck(0);
		CheckExp.SetCheck(0);
		CheckADAMS.SetCheck(1);
		CheckSimu.EnableWindow(0);
		CheckExp.EnableWindow(0);
		CheckADAMS.EnableWindow(1);
	}
	else if (gFlagSimulation == 2) // Real experiment
	{
		CheckSimu.SetCheck(0);
		CheckExp.SetCheck(1);
		CheckADAMS.SetCheck(0);
		CheckSimu.EnableWindow(0);
		CheckExp.EnableWindow(1);
		CheckADAMS.EnableWindow(0);
	}


	gpRobotDrawingBuffer = new float[gDrawingSeqSize]; // 雙足繪圖buffer
	gpRobotDrawingBufferArm = new float[gDrawingSeqSizeArm]; // 手臂繪圖buffer

	gKineAll.ArmDrawingBuffer = gpRobotDrawingBufferArm; // 指定繪圖暫存區位置給FK使用
	gKineAll.LegDrawingBuffer = gpRobotDrawingBuffer; // 指定繪圖暫存區位置給FK使用


	double ShiftAngle = 1.0/180.0*3.1415926; // 身體往後一點點 抵擋手臂舉起來的COG移動

	gZeroJointPosShift[0] = 0.0; // L hip yaw
	gZeroJointPosShift[1] = 0.0; // L hip roll
	gZeroJointPosShift[2] = -ShiftAngle; // L hip pitch
	gZeroJointPosShift[3] = 0.0; // L knee pitch
	gZeroJointPosShift[4] = ShiftAngle; // L ankle pitch
	gZeroJointPosShift[5] = 0.0; // L ankle roll
	gZeroJointPosShift[6] = 0.0; // R hip yaw
	gZeroJointPosShift[7] = 0.0; // R hip roll
	gZeroJointPosShift[8] = ShiftAngle; // R hip pitch
	gZeroJointPosShift[9] = 0.0; // R knee pitch
	gZeroJointPosShift[10] = -ShiftAngle; // R ankle pitch
	gZeroJointPosShift[11] = 0.0; // R ankle roll

	return TRUE;  // return TRUE  unless you set the focus to a control
}

void CRobotAllDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CRobotAllDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// The system calls this function to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CRobotAllDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



void CRobotAllDlg::OnBnClickedButton1() // the button "Start"
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 按下start會執行
	// 內容包含 初始化變數 如ZMP COG 所有軌跡
	// 總步數 每步旋轉角度 機器人行為 都在這邊指定
	******************************************************************/

		//gSpeaker.speak("大家好 我是由國立台灣大學 機器人實驗室所研發的人型機器人");
	 //   gSpeaker.speak("準備開始 行走模式");

		// 開場 寒暄
	    //gSpeaker.speak("大家好 歡迎大家來參加這次的人形機器人成果發表會 ");
	    //gSpeaker.speak("歡迎大家的光臨 我是在國科會計畫下所誕生的人型機器人 我的名字是 約翰 從出生的時間來看，我可以算是邦妮的弟弟");
 
		//gSpeaker.speak("我是由國立台灣大學 機械所 機器人實驗室所研發的人型機器人 在黃漢邦教授的指導下 耗時三年所打造 我全身上下總共有三十二的自由度 包含每個手臂與腳各六個 腰兩個以及臉部六個 我能做出各式各樣的動作與表情 請大家多多指教");
	    //gSpeaker.speak("我之所以能夠在這邊跟大家這樣說話 是因為我整合了從三年前開始執行的國科會仿生機器人計畫的成果 從機構 電路 到控制全部都由我們團隊自行設計與規劃 人型機器人是一個高度技術整合的研究題目 從機構設計 電路設計 自動控制 軌跡規劃到人工智慧 都有所關聯 雖然我還不能進行跑與跳等比較激烈的動作 但是基於目前我身上所用到的技術 下一代的人形機器人會更加靈巧與聰明 同時也希望可以將我身上的技術用到各個相關領域 以促進台灣在機器人與工業上知識能量的累積");

	//gSpeaker.speak("安裝在我身上的磷酸鋰鐵電池 可以提供長效並且穩定的電源供應 針對我身上的各個部分 經由安裝在我身上的店員管理系統 可以進行電流 電壓 與溫度的監控達到即時的電源管理 並且也能在電池快要沒電的時候 及早進行充電");
	//gSpeaker.speak("在這邊我們要特別感謝工研院所提供的語音模組 讓我跟邦妮能夠順利地跟大家說話");



		// 自我介紹
	// 自我介紹 Nino
		//gSpeaker.speak("好久不見，因為我在為自己增加新功能  等等就表演給你們看",0);
		////system("pause");
		//gSpeaker.speak("我現在開始展示上下樓梯的動作囉",-2);
		//system("pause");
		//gSpeaker.speak("對於人形機器人來說 平地行走要保持平衡是一個相當困難的問題，而樓梯行走又是更加地不容易呢。",-1);
		//system("pause");

		//gSpeaker.speak("嗯，走樓梯真是很辛苦啊!!不過這樣就能更加融入人類的生活環境囉!",0);
		//system("pause");
		//gSpeaker.speak("好耶 那我門就好好期待囉",0);05
		//system("pause");

		//gSpeaker.speak("沒錯，接下來就是要表演我準備已久的手語",-1);
		//system("pause");
		//gSpeaker.speak("另外遺風同學也會跟我一起表演優",-2);
		//system("pause");
	

		//gSpeaker.speak("我是由國立台灣大學 機械所 機器人實驗室所研發的人型機器人 在黃漢邦教授的指導下 耗時三年所打造 我全身上下總共有三十二的自由度 包含每個手臂與腳各六個 腰兩個以及臉部六個 我能做出各式各樣的動作與表情 請大家多多指教");

		// 中間 伸展身體時
	    // gSpeaker.speak("我之所以能夠在這邊跟大家這樣說話 是因為我整合了從三年前開始執行的國科會仿生機器人計畫的成果 從機構 電路 到控制全部都由我們團隊自行設計與規劃");
		//gSpeaker.speak("人型機器人是一個高度技術整合的研究題目 從機構設計 電路設計 自動控制 軌跡規劃到人工智慧 都有所關聯 雖然我還不能進行跑與跳等比較激烈的動作 但是基於目前我身上所用到的技術 下一代的人形機器人會更加靈巧與聰明 同時也希望可以將我身上的技術用到各個相關領域 以促進台灣在機器人與工業上知識能量的累積");

		//gSpeaker.speak("接下來我會進行幾種動作的展示 包含側向移動 轉彎 單腳站立 以及平衡的動作 請大家仔細看囉");
		//gSpeaker.LoadSoundFile("SoundFiles/InitialWelcome.wav");
		//gSpeaker.PlaySoundByFile(gSpeaker.Sound1);

		//gSpeaker.speak("安裝在我身上的磷酸鋰鐵電池 可以提供長效並且穩定的電源供應 針對我身上的各個部分 可以進行電流 電壓 與溫度的監控 可以即時判斷電源供應的狀態 並且也能在電池快要沒電的時候 及早進行充電 ");
		//gSpeaker.speak("掰掰 我要回家囉 期待我們下次的相見 謝謝大家");

	//gSpeaker.speak("害 邦妮你好 謝謝你的開場囉");

	 //   gpRobotDrawingBuffer = new float[gDrawingSeqSize]; // 雙足繪圖buffer
		//gpRobotDrawingBufferArm = new float[gDrawingSeqSizeArm];

     //gSpeaker.speak("我之所以能夠在這邊跟大家說話 是因為整合了國科會仿生機器人計畫的成果 從機構 電路 到控制全都由我們自行設計與規劃 人型機器人是個高度技術整合的題目 從機電設計 自動控制 軌跡規劃到人工智慧 都有所關聯 雖然我還不能進行跑與跳等比較激烈的動作 但是基於目前我身上用到的技術 下一代的機器人會更加靈巧與聰明 也希望可以將我身上的技術用到相關領域 促進台灣在機器人與工業上知識能量的累積");
	//我之所以能夠在這邊跟大家說話 是因為我整合了國科會仿生機器人計畫的成果 從機構 電路 到控制全部都由我們團隊自行設計與規劃 人型機器人是一個高度技術整合的題目 從機構設計 電路設計 自動控制 軌跡規劃到人工智慧 都有所關聯 雖然我還不能進行跑與跳等比較激烈的動作 但是基於目前我身上所用到的技術 下一代的人形機器人會更加靈巧與聰明 也希望可以將我身上的技術用到相關領域 以促進台灣在機器人與工業上知識能量的累積
	 
	 	mButton_Start.EnableWindow(false);

		//啟動EPOS3一定放在最前面!!!!!!
		if (gFlagSimulation == RealExp && gFlagReadForceSensor==0)	
		{
		  #if TwinCAT_Mode
		  printf("Wait 5 seconds for TwinCAT...\n");
		  Sleep(5000);		  
		  TCAT=new TwinCAT_COM();
		  gDeleteLogFile();
		  #else
		  printf("TwinCAT_Mode in MainLoops.h: disable, Press any key to continue \n");
		  system("pause");
		  #endif
		}
		//啟動EPOS3一定放在最前面!!!!!!
		
	 	//gInitneck(0);	// 脖子初始化
		
		//Vector Test
		#if SoundCtrl
					gSpeaker.ReadSignLanq(37);
					//for(int i=0;i<37;i++)
					//{
					//	gSpeaker.PlaySoundByFile(gSpeaker.SignLanq[i]);
					//	system("pause");
					//}
		#endif
		//Vector Test

	
		for (int i =0 ; i < gDrawingSeqSize ; i++)
			gpRobotDrawingBuffer[i] = 0;

	    gDrawingBufCreated = true;

		// 將所有初始左右腳角度軌跡清除成零
		for (int i = 0 ; i < 10000 ; i++)
		{
			gAngLBuf[i] = 0;
			gAngRBuf[i] = 0;

			//20121214doratom//
			gPitchAngLBuf[i] = 0;
			gPitchAngRBuf[i] = 0;
			//20121214doratom//
		}

		gNza = gStepSample * DSP /2.0;    // 將一步切成三段 第一段格點數  gNza = gStepSample * DSP /2.0
		gNab = gStepSample * SSP;        // 將一步切成三段 第二段格點數 gNab = gStepSample * SSP
		gNzb = gStepSample - gNza - gNab;  // 將一步切成三段 第三段格點數  gNzb = gStepSample - gNza - gNab

		// Kinematics initialization
		gKineAll.InitKineTrains();
		//gKineAll.InitInertia();

		#if OfflineTraj	
			gSendOfflineTraj();	// 送ENC
			gEncFeedback();	// 讀ENC 得FK COG
			gKineAll.FindFK();
			gKineAll.FindCOG();
		#else
			gKineAll.FindFK();
			gKineAll.FindCOG();
			gKineAll.ComputeJacobians();
		
			//#if RunDynamics						
			//gKineAll.FindDIni();	// FindDIni請務必放在FindCOG之後 因為在計算上半身轉動慣量時會用到 130925關閉 KF初始在Kine一開始
			//#endif
			#if BangBangControl
			SdParameterInit();
			#endif

			gKineAll.initCOG[0] = gKineAll.COG[0];
			gKineAll.initCOG[1] = gKineAll.COG[1];
			gKineAll.initCOG[2] = gKineAll.COG[2];

			gDHInitCOG[0] = gKineAll.COG[0]; // 到達DH初始值 機器人不再singular 的 初始COG
			gDHInitCOG[1] = gKineAll.COG[1]; // 到達DH初始值 機器人不再singular 的 初始COG
			gDHInitCOG[2] = gKineAll.COG[2]; // 到達DH初始值 機器人不再singular 的 初始COG

			gKineAll.initLL[0] = gKineAll.CrdAll->data[18];
			gKineAll.initLL[1] = gKineAll.CrdAll->data[19];
			gKineAll.initLL[2] = gKineAll.CrdAll->data[20];

			gKineAll.initRL[0] = gKineAll.CrdAll->data[57];
			gKineAll.initRL[1] = gKineAll.CrdAll->data[58];
			gKineAll.initRL[2] = gKineAll.CrdAll->data[59];

			gKineAll.SetIdentity(gKineAll.TarRotMSw);
			gKineAll.SetIdentity(gKineAll.TarRotMFx);

			//20121214doratom//
			gKineAll.SetIdentity(gKineAll.TarRotMSwPitch);
			gKineAll.SetIdentity(gKineAll.TarRotMFxPitch);
			//20121214doratom//

			//泓逸start111223
			gKineAll.initLA[0] = gKineAll.CrdAll->data[105];
			gKineAll.initLA[1] = gKineAll.CrdAll->data[106];
			gKineAll.initLA[2] = gKineAll.CrdAll->data[107];

			gKineAll.initRA[0] = gKineAll.CrdAll->data[135];
			gKineAll.initRA[1] = gKineAll.CrdAll->data[136];
			gKineAll.initRA[2] = gKineAll.CrdAll->data[137];		
			//泓逸end111223
		
			//Slongz start120531		
			gKineAll.InitQKineTrains();
			//Slongz start120531

			// step zmp X positions
			for (int i = 0 ; i < 10000 ; i++)
			{
				gFstpX[i] = 0;
				gFstpY[i] = 0;
			}

			//0522
			gReadArmTraj();
			//0522

			// 腳本號碼從零開始選擇 初始腳本
			gPrepareScenarioScript(0); // 從零開始 選擇初始劇本

			// 設定腳本結束時 會自動初始化 COG ZMP
			gCOGZMPInitialized = true;

			////////////////////////////// Added by slongz////////////////////////////// 
			#if WriteInitZMPtxt 
				fstream  originCOGf;
				//originCOGf.open("C:/Users/user/Desktop/MatlabAdamsSimu/originCOG.txt",ios::out| ios::trunc);

				originCOGf.open("originCOG.txt",ios::out| ios::trunc);
				for(int i=0;i<gLQs.LQDataLen;i++)
				{
					originCOGf<<gLQs.YState[i].data[0]<<"\t"<<gLQs.XState[i].data[0]<<"\t"<<gInpInvPendulumHeight[i+4];
					//originCOGf<<gLQs.YState[i].data[0]<<"\t"<<gLQs.XState[i].data[0]<<"\t"<<0;
					originCOGf<<"\n";
				}
				originCOGf.close();	

				originCOGf.open("originZMP.txt",ios::out| ios::trunc);
				for(int i=0;i<gLQs.LQDataLen;i++)
				{
					originCOGf<<gLQs.YState[i].data[2]<<"\t"<<gLQs.XState[i].data[2]<<"\t"<<0;
					originCOGf<<"\n";
				}
				originCOGf.close();	
			#endif
			////////////////////////////////////////////////////////////////////////////


			// 記住初始角度
			for (int i = 0 ; i < 6 ; i++)
			{
				gInitThetas[i] = gKineAll.FKLLeg->theta[i+1]*gPNJoints[i];
				gKineAll.FKLLeg->theta[i+1] = 0;
			}
			for (int i = 0 ; i < 6 ; i++)
			{
				gInitThetas[i+6] = gKineAll.FKRLeg->theta[i+1]*gPNJoints[i+6];
				gKineAll.FKRLeg->theta[i+1] = 0;
			}

			//泓逸start111226
			for (int i = 0 ; i < 2 ; i++)//Trunk
			{
				gInitThetas[i+12] = gKineAll.FKLArm->theta[i+1]*gPNJoints[i+12];
				gKineAll.FKLArm->theta[i+1] = 0;
			}
			for (int i = 0 ; i < 6 ; i++)//LA
			{
				gInitThetas[i+14] = gKineAll.FKLArm->theta[i+4]*gPNJoints[i+14];
				gKineAll.FKLArm->theta[i+4] = 0;
			}
			for (int i = 0 ; i < 6 ; i++)//RA
			{
				gInitThetas[i+20] = gKineAll.FKRArm->theta[i+4]*gPNJoints[i+20];
				gKineAll.FKRArm->theta[i+4] = 0;
			}
			//泓逸end111226

			gKineAll.FindFK();
			gKineAll.FindCOG();

			gTotalInitCOG[0] = gKineAll.COG[0]; // 腳是直的 完全初始時候的COG
			gTotalInitCOG[1] = gKineAll.COG[1]; // 腳是直的 完全初始時候的COG
			gTotalInitCOG[2] = gKineAll.COG[2]; // 腳是直的 完全初始時候的COG

			//// 把機構從0度~初始角度的內插動作
			//double gInitThetas[12];

			//int gInterNum = 300; // 內插格數
			//double gInitMotion[12][300]; // 注意!! 第二個維度要跟gInterNum設一樣
			//unsigned char gInitTrajBuf[18000]; // 18000 = 300*12*5 小心這個buffer的大小要 = gInterNum * 軸數 * 5

			//double gInitTime = gInterNum*dt; 
			//int gInitCount = 0; // 計算到第幾格

			// 先內插好

			double tempVal;
			int index_cmd = 0;
			long temp_encoder;
			double temp_poly[1000];

			for (int k = 0 ; k < 26 ; k++)
			{
				//gKineAll.GenSmoothZMPShift(0,gInitThetas[k],gInterNum,temp_poly);
				gKineAll.GenSmoothZMPShift_ZeroJerk(0,gInitThetas[k],gInterNum,gTempPoly);

				for (int j=0 ; j < gInterNum; j++)
				{
					gInitMotion[k][j] = gTempPoly[j];
					//printf("%f  ",gInitMotion[k][j]);
					//cout<<endl;
				}
			}
		#endif	// OfflineTraj

		#if WriteInitZMPtxt// 若要匯出 ZMP 軌跡 請打開下面幾行
			fstream ZMPPlot;

			ZMPPlot.open("ZMPPlot.txt",ios::out | ios::trunc);	// gInpZMPx左右 gInpZMPy前後 轉換成機器人座標
			for (int i = 0 ; i < 60000 ; i++){
				ZMPPlot << gInpZMPy[i] <<"\t"<< gInpZMPx[i]<<endl;
			}
			ZMPPlot.close();

			// 需要檢查特徵點是否算錯
			// gPAll
			ZMPPlot.open("PALL.txt",ios::out | ios::trunc);
			for (int i = 0 ; i < 200 ; i++){
				ZMPPlot << gPAll[0][i] << " "<< gPAll[1][i] << " "<< gPAll[2][i] << " \n";
			}
			ZMPPlot.close();
		#endif

		mButton_Auto.EnableWindow(true);
		
		// start openGL window 這一行一定要在最下面，因為比它下面的code不會被執行 
		//(因為它的最後一行是 	glutMainLoop(); 會把程式跳去執行 openGL state machine)
		gInitializeKinematicsGL();
		// 不要放任何程式再這一行後面，在這個function中

}


void CRobotAllDlg::OnBnClickedButton2()  // auto
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 按下auto會執行
	// 自動執行機器人 PID值設定 Calibration各軸位置 設定PWM上下限
	// 開啟 threads 所有行為自動設定
	// 最後會停在連續軌跡模式的開端 再壓下Go 機器人就會開始行動
	******************************************************************/
	if (gFlagSimulation == RealExp)
	{	
		 // 清空字串
		 for (int i = 0 ; i < 60 ; i++)
		 {
			gGlobalMessage[i] = ' ';
		 }

		  // 讀取重心高度文字檔用這邊
		  // Load COGz Results

		  ////fstream Cz_result;
		  ////Cz_result.open("CzData.txt",ios::in);
		  ////for (int i = 0 ; i < 12000; i++)
		  ////{
			 //// Cz_result >> gInpCOG[i] ;
		  ////}
		  ////Cz_result.close();

		  //Load COGz Results
			
		  // initialize parameter and open threads	
 		  sprintf_s(gGlobalMessage,"Initializing controller and start threads...\n");
		  printf(gGlobalMessage);
		  OnBnClickedButton4();

		  //#if TwinCAT_Mode
		  //TCAT=new TwinCAT_COM();
		  //system("del EncLogData.txt");
		  //system("del EncDiffLogData.txt");
		  //system("del EncTorqueLogData.txt");
		  //#else
		  //printf("TwinCAT_Mode in MainLoops.h: disable, Press any key to continue \n");
		  //system("pause");
		  //#endif

		  printf("Controller initialized.\n");
		  sprintf_s(gGlobalMessage,"Threads started.\n\n");
		  printf(gGlobalMessage);

		  Sleep(200);

		  //printf("Loading Trajectory Files.\n");
		  //sprintf_s(gGlobalMessage,"Loading Trajectory Files.\n\n");
		  //printf(gGlobalMessage);

		  ////gLoadfile(); // 將手臂手掌等軌跡讀取進來   // ??????????????? Not required ?????????????

		  //printf("Trajectory Files Loaded.\n");
		  //sprintf_s(gGlobalMessage,"Trajectory Files Loaded.\n\n");
		  //printf(gGlobalMessage);


		  if(gFlagHandCtrl)
		  {
			  gLoadfileHand();
			  //gSpeaker.LoadSoundFile("SoundFiles/N01.wav");
			  //gOpenPort();
		  }
		  #if PreloadTorque
		  //********load torqueoffset file test
		  gReadPreloadTorque();

		 // 	fstream gTorqueOffsetFile;
			//gTorqueOffsetFile.open("TorqueOffset_up3_201305211635_ILC06.txt",ios::in);//FilterEncTorqueLogDataQQ.txt
			//for (int j=0;j<gDataTotal;j++)
			//{
			//	for (int i=0;i<12;i++)
			//	{
			//		//TempENC[i][j] = 0;
			//		gTorqueOffsetFile>>gTorqueOffsetFileBuf[j*12+i];
			//		gShortTorqueOffsetFileBuf[j*12+i]=short(gTorqueOffsetFileBuf[j*12+i]);
			//	}
			//}
			//gTorqueOffsetFile.close();
			//***********************
		  #endif


		  //// open port
		  //sprintf_s(gGlobalMessage,"Open USB port (Leg)...\n");
		  //printf(gGlobalMessage);
		  //// OnBnClickedButton3();
		  if(gFlagArmCtrl)
		  {
		  sprintf_s(gGlobalMessage,"Open USB port (Arm)...\n");
		  //gOpenPort();
		  }

		  if(gFlagArmCtrl||gFlagHandCtrl)
		  {
			  gOpenPort();
			  //gTestHand();
		  }
		  //Sleep(200);
		  //printf("USB port opened.\n\n");

		  //// set all PID
		  //sprintf_s(gGlobalMessage,"Seting All PID...\n");
		  //printf(gGlobalMessage);

		  //SendIndex = SetAllPID; // set all PID
		  //cout<<"Sending Mode : "<< "set all PID" <<endl;
		  //gPreProcessData(SendIndex);
		  //sprintf_s(gGlobalMessage,"Set all PID Done.\n\n");
		  //printf(gGlobalMessage);

		  //Sleep(200);

		  // set arm PID
		  if(gFlagArmCtrl)
		  {
		  sprintf_s(gGlobalMessage,"Seting Arm PID...\n");
		  printf(gGlobalMessage);

		  SendIndex = SetAllPID; // set all PID
		  cout<<"Sending Mode : "<< "Set Arm PID" <<endl;
		  gPreProcessData(SendIndex);
		  sprintf_s(gGlobalMessage,"Set Arm PID Done.\n\n");
		  printf(gGlobalMessage);

		  Sleep(200);
		  }
		  //// set all PWM
		  //sprintf_s(gGlobalMessage,"Seting All PWM...\n");
		  //printf(gGlobalMessage);

		  //SendIndex = SetPWMLims; // set all PWM
		  //cout<<"Sending Mode : "<< "set all PWM" <<endl;
		  //gPreProcessData(SendIndex);
		  //sprintf_s(gGlobalMessage,"Set all PWM Done.\n\n");
		  //printf(gGlobalMessage);

		  //Sleep(200);
	 
		  //if (gFlagReadEncoder == 1)
		  //{
			 // SendIndex = SetEncReadFlag; // set encoder feedback
			 // cout<<"Sending Mode : "<< "set encoder feedback" <<endl;
			 // gPreProcessData(SendIndex);
			 // sprintf_s(gGlobalMessage,"Set encoder feedback.\n\n");
			 // printf(gGlobalMessage);

			 // Sleep(200);
		  //}

		 // SendIndex = SetCali; // set all cailbrate
		 // cout<<"Sending Mode : "<< "set all calibrate" <<endl;
		 // gPreProcessData(SendIndex);
		 //// sprintf_s(gGlobalMessage,"Set all calibrate Done.\n\n");
		 //// printf(gGlobalMessage);

		 // Sleep(200);




		// Manual 或是 Homing完記下調整後得ENC值 再餵到SaveJointData中當作新的原點 4/4WZ
		for(int i = 0 ; i < 12 ; i++)
			CaliTemp[i] = 0;

		  if(gFlagManualMode == 0)
		  {
		  // auto set to traj. mode		  
				SendIndex = Traj; // traj mode
				cout<<"Sending Mode : "<<"Traj"<<endl;
				CaliAngleMode = 0;
		  }
		  else
		  {
				SendIndex = 2; // traj mode
				cout<<"Sending Mode : "<<"Manual"<<endl;
				cout<<"\n"<<"欲手動調整各軸角度 請鍵入 1 "<<"\n"<<endl;
				cin>> CaliAngleMode;
		  }
		  // auto set to traj. mode
		  //SendIndex = 5; // traj mode
		  //cout<<"Sending Mode : "<<"Traj"<<endl;

		if(CaliAngleMode==1) // 手動調整各軸
		{
			gCaliManual();
			for(int i = 0 ; i < 12; i++)
			cout<<"第"<<i+1<<"軸轉了"<<CaliTemp[i]<<endl;
		}

		//Cali要開力規勾勾
		if(gFlagReadForceSensor == 1)
		{		
			// 如果Manual調完之後 要把調的ENC值存為新的原點
				CaliRollHipL = CaliTemp[1] ;
				CaliRollHipR = CaliTemp[7] ;
				CaliPitchHipL = CaliTemp[2];
				CaliPitchHipR = CaliTemp[8];
				CaliPitchAngL = CaliTemp[4];
				CaliPitchAngR = CaliTemp[10];
				CaliRollAngL = CaliTemp[5];
				CaliRollAngR = CaliTemp[11];

			cout<<"Start "<<"CaliAnkleAngle"<<endl;
			system("pause");
			  for(int i = 0 ; i < 6 ; i++)
			  {
				cout<<"\n"<<"Start "<<"CaliAnkleAngle"<< "第 " << i+1 << "次" <<endl;
				
				CaliFlagL = 0;
				CaliFlagR = 0;
				while(1)
				{
					gCoPCali(&CaliFlagL, &CaliFlagR);

					if(CaliFlagL==4 && CaliFlagR==4)
					{
						cout<<"ZMP: "<<"done"<<endl;
						cout<<"CaliPitchAngL: "<< CaliPitchAngL <<endl;
						cout<<"CaliPitchAngR: "<< CaliPitchAngR <<endl;
						cout<<"CaliRollAngL: "<< CaliRollAngL <<endl;
						cout<<"CaliRollAngR: "<< CaliRollAngR <<endl;
						cout<<"CaliRollHipL: "<< CaliRollHipL <<endl;
						cout<<"CaliRollHipR: "<< CaliRollHipR <<endl;
						cout<<"CaliPitchHipL: "<< CaliPitchHipL <<endl;
						cout<<"CaliPitchHipR: "<< CaliPitchHipR <<endl;
						break;
					}

					if(CaliFlagL==3 && CaliFlagR==3)
					{
						cout<<"ReadForceSensor: "<<"done"<<endl;
						cout<<"CaliPitchAngL: "<< CaliPitchAngL <<endl;
						cout<<"CaliPitchAngR: "<< CaliPitchAngR <<endl;
						cout<<"CaliRollAngL: "<< CaliRollAngL <<endl;
						cout<<"CaliRollAngR: "<< CaliRollAngR <<endl;
						cout<<"CaliRollHipL: "<< CaliRollHipL <<endl;
						cout<<"CaliRollHipR: "<< CaliRollHipR <<endl;
						cout<<"CaliPitchHipL: "<< CaliPitchHipL <<endl;
						cout<<"CaliPitchHipR: "<< CaliPitchHipR <<endl;
						cout<<"FS_ZMPx: "<<gKineAll.FS_ZMP[0]<<endl;
						cout<<"FS_ZMPy: "<<gKineAll.FS_ZMP[1]<<endl;
						cout<<"CoPLx: "<<gKineAll.CoPL[0]<<endl;
						cout<<"CoPLy: "<<gKineAll.CoPL[1]<<endl;
						cout<<"CoPRx: "<<gKineAll.CoPR[0]<<endl;
						cout<<"CoPRy: "<<gKineAll.CoPR[1]<<endl;
						cout<<"\n"<<"DONE"<<endl;
						break;
					}	
					Sleep(10);
				}
			  }
			
			  // Homing調完之後 要把調的ENC值存為新的原點
			  // CaliRollHipL 等是絕對的ENC
				CaliTemp[1] = CaliRollHipL;
				CaliTemp[7] = CaliRollHipR;
				CaliTemp[2] = CaliPitchHipL;
				CaliTemp[8] = CaliPitchHipR;
				CaliTemp[4] = CaliPitchAngL;
				CaliTemp[10] = CaliPitchAngR;
				CaliTemp[5] = CaliRollAngL;
				CaliTemp[11] = CaliRollAngR;

			///////////////////////// infrared cali /////////////////////
			//if (gFlagInfrared == 1)
			//{			
			//	cout<<"Press Any Key to Start Infrared Calibration"<<endl;				
			//	system("pause");
			//	while(1)	
			//	{	
			//		if (infraredcalicount < 1000){
			//			gKineAll.infraredcaliL1 = gKineAll.infraredcaliL1 + gKineAll.InfraredLdisdata1 [gKineAll.infraredcount-1] ;
			//			gKineAll.infraredcaliL2 = gKineAll.infraredcaliL2 + gKineAll.InfraredLdisdata2 [gKineAll.infraredcount-1] ;
			//			gKineAll.infraredcaliL3 = gKineAll.infraredcaliL3 + gKineAll.InfraredLdisdata3 [gKineAll.infraredcount-1] ;
			//			gKineAll.infraredcaliL4 = gKineAll.infraredcaliL4 + gKineAll.InfraredLdisdata4 [gKineAll.infraredcount-1] ;
			//			gKineAll.infraredcaliR1 = gKineAll.infraredcaliR1 + gKineAll.InfraredRdisdata1 [gKineAll.infraredcount-1] ;
			//			gKineAll.infraredcaliR2 = gKineAll.infraredcaliR2 + gKineAll.InfraredRdisdata2 [gKineAll.infraredcount-1] ;
			//			gKineAll.infraredcaliR3 = gKineAll.infraredcaliR3 + gKineAll.InfraredRdisdata3 [gKineAll.infraredcount-1] ;
			//			gKineAll.infraredcaliR4 = gKineAll.infraredcaliR4 + gKineAll.InfraredRdisdata4 [gKineAll.infraredcount-1] ;

			//			infraredcalicount++ ;
			//			Sleep(5);
			//		}
			//		if (infraredcalicount == 1000){
			//			gKineAll.infraredbiasL1  =  gKineAll.infraredcaliL1 /1000;
			//			gKineAll.infraredbiasL2  =  gKineAll.infraredcaliL2 /1000;
			//			gKineAll.infraredbiasL3  =  gKineAll.infraredcaliL3 /1000;
			//			gKineAll.infraredbiasL4  =  gKineAll.infraredcaliL4 /1000;

			//			gKineAll.infraredbiasR1  =  gKineAll.infraredcaliR1 /1000;
			//			gKineAll.infraredbiasR2  =  gKineAll.infraredcaliR2 /1000;
			//			gKineAll.infraredbiasR3  =  gKineAll.infraredcaliR3 /1000;
			//			gKineAll.infraredbiasR4  =  gKineAll.infraredcaliR4 /1000;
			//			cout<<"Infrared Calibration Completed"<<endl;
			//			break;
			//		}
			//	}
			//}
		

		   if(gFlagIMU==1)	///Imu plotting test 20140218
			{   
				startIMU =1 ;
				while(1)
				{   
					if	(IMU1.count == IMU1.calicount){  //表示cali全部完成  可以行走
						cout<<"\t\t IMU Calibration Completed"<<endl;
						break;
					}
	  		        Sleep(1);
				}
			//	cout<<"\t\t start IMU upright calibration "<<endl;
			//
			//while(1)
			//	{
			//	
			//		if ((abs(IMU1.finalanglex [IMU1.count-1]*180/PI) > 0.5 )   ||  (abs( IMU1.finalangley [IMU1.count-1] *180/PI) > 0.5 ) ){
			//			//誤差在一度內就跳出calibration
			//			//posturecalibration(IMU1.finalanglex ,IMU1.finalangley ,IMU1.count );
			//			//IMU1.finalanglex roll方向  ,   IMU1.finalangley pitch方向
			//			break; 
			//            }
			//		else{
			//			cout<<" \n\n\n \t\t  IMU upright calibration completed "<<endl;	
			//			IMUglplot = 0 ;//停止IMU繪圖		
			//			break; 
			//			}
			//		    Sleep(10);
			//	}
		   
			}

		}//if(gFlagReadForceSensor == 1)
		#if TwinCAT_Mode	// 讀入TCAT初始值 LogEncCount = 0 之後進SaveJointData會再把第一筆洗掉
			TCAT->EtherCATReadEncoder(LogEnc); 
			TCAT->EtherCATReadVel(LogVel);
			TCAT->EtherCATReadTorque(LogTorque);	

			#if BangBangControl
			TCAT->EtherCATReadEncDiff(LogEncDiff);
			TCAT->MovingAve(20,LogVel,LogMovAveVel); //對讀取Joint Vel 作Moving Average
			#endif
		#endif
	}//Exp Mode
	else
	{

		  OnBnClickedButton4();
		  SendIndex = Traj; // traj mode
		  gSetPIDDone = true;

		  cout<<"Sending Mode : "<<"Traj"<<endl;
	}
	
	mButton_Auto.EnableWindow(false);
	mButton_Go.EnableWindow(true);


}


void CRobotAllDlg::OnBnClickedButton3() // the button "Go"
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 按下Go 會自動執行
	// 機器人就會開始行走
	******************************************************************/
	if (gFlagGoPass)
	{
		gFlagGoPass=0;
		QueryPerformanceCounter(&gGlobalTime);
		gWalkStartPoint = gForceDataCount;
	}

		#if SoundCtrl
	//Go
		Sleep(36000);
			gSpeaker.LoadSoundFile("SoundFiles/N01.wav");
			gSpeaker.PlaySoundByFile(gSpeaker.Sound1); 
		
		
			Sleep(12000);
			gSpeaker.LoadSoundFile("SoundFiles/N02.wav");
			gSpeaker.PlaySoundByFile(gSpeaker.Sound1);
	//上樓梯

			Sleep(4000);
		#endif
		//走路/下樓梯
		//gSpeaker.LoadSoundFile("SoundFiles/N03.wav");
		//gSpeaker.PlaySoundByFile(gSpeaker.Sound1); 
		//Sleep(2000);

	//給Mobile Robot組count數(轉彎+直走)
		//Sleep(17000);
		//gSpeaker.LoadSoundFile("SoundFiles/N04.wav");
		//gSpeaker.PlaySoundByFile(gSpeaker.Sound1); 

		//Sleep(14000);

		//gSpeaker.LoadSoundFile("SoundFiles/N06.wav");
		//gSpeaker.PlaySoundByFile(gSpeaker.Sound1); 
		//Sleep(4500);

		//gSpeaker.LoadSoundFile("SoundFiles/N07.wav");
		//gSpeaker.PlaySoundByFile(gSpeaker.Sound1); 
		//Sleep(5000);
		//system("pause");



	gPreProcessData(SendIndex);
	if (gFlagSimulation == RealExp)
	{
		gPreProcessDataHand(5,2,0.001,15);

	}

	if(gFlagManualMode == 0)
	{
			//mButton_Go.EnableWindow(true);
			mButton_Go.EnableWindow(false);
	}
	else
	{

	}
}



void gInitTimerAndThread(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	//  開始整個程式的所有thread 與 timer 
	******************************************************************/

		// 初始化timer
        gTestHighTimer();

		gThreadRender = CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)gRenderSceneThread,(void*)0,0,&gTIDRender);
		gRenderThreadOpened = true;
		SetThreadPriority(gThreadRender,THREAD_PRIORITY_NORMAL);

		gThreadControl = CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)gMotionControlThread,(void*)0,0,&gTIDControl);
		gControlThreadOpened = true;
		SetThreadPriority(gThreadControl,THREAD_PRIORITY_TIME_CRITICAL);
		
		if(gFlagArmCtrl ||gFlagHandCtrl)
		{
		gThreadArmControl = CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)gArmControlThread,(void*)0,0,&gTIDArmControl);
		gArmControlThreadOpened = true;
		SetThreadPriority(gThreadArmControl,THREAD_PRIORITY_TIME_CRITICAL);
		}

		//gThreadFace = CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)gFaceControlThread,(void*)0,0,&gTIDFace);
		//gFaceControlThreadOpened = true;
		//SetThreadPriority(gThreadFace,THREAD_PRIORITY_NORMAL);
		if(gFlagTimer== Thread)
		{
		gThreadTimer = CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)gTimerThread,(void*)0,0,&gTIDTimer);
		//gControlThreadOpened = true;
		SetThreadPriority(gThreadTimer,THREAD_PRIORITY_TIME_CRITICAL);
		}
	#if LaserCatch
		gThreadLaserRender = CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)gLaserRenderThread,(void*)0,0,&gTIDLaserRender);
		gLaserRenderThreadOpened = true;
		SetThreadPriority(gThreadRender,THREAD_PRIORITY_NORMAL);
	#endif
//#if IMUCatch
		if (gFlagIMU){
		gThreadIMU = CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)gIMUThread,(void*)0,0,&gTIDIMU);
		gIMUThreadOpened = true;
		SetThreadPriority(gThreadIMU,THREAD_PRIORITY_NORMAL);
		}
//#endif




}

void gTestHighTimer(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	//  初始化計時所需要的所有變數
	******************************************************************/
	  cout << "注意!! 這種Timer觸發有誤差，可能是4.88ms或是 5.88ms之類的時間，" << endl;
	  cout << "所以丟軌跡請使用絕對時間，才不會有累積時間誤差" << endl;

        if (QueryPerformanceFrequency(&gnFreq))
         {
               //
               const int gnBufSize = 256;

			   gFreqT = (float)(gnFreq.QuadPart);

               TCHAR gchBuf[gnBufSize];
        
               // CPU Freq
               wsprintf(gchBuf,_T("CPU Frequency = %I64d\r\n"),gnFreq);
               printf("%s\n",gchBuf);  
         }  
}

void gLaserRenderThread(void)
{
#if LaserCatch
	eye= new laser();
	for(int i=0; i<681;i++)
	{
		laserbufX[i]=0;
		laserbufY[i]=0;	 
	}

	while(1)
	{
		if(gLaserRenderLife)
		{
			if(gLaserRenderThreadOpened)
			{
				eye->transfer();
				for(int i=0;i<681;i++)
				{
					laserbufX[i]=eye->x_data[i];
					laserbufY[i]=eye->y_data[i];
				}	
			}	
		}
		else
			break;
		Sleep(200); 
	}
#endif
}

void gIMUThread(void)
{
	while(1)
	{
		if(gIMULife)
		{
			if(gIMUThreadOpened)
			{
				if (startIMU==1){
					IMU1.ReadContinuousData(15);  //設定IMU comport
				}
				else{
					Sleep(10);
					//break;
				}
			}	
		}
		else{
			break;
		}
		Sleep(200); 
	}
}


void gMotionControlThread(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	//  機器人會連續行動就是因為MotionControlThread會執行這個function裡面的while(1)
	// 停止時會跳出while迴圈 然後這個thread就會自動結束生命週期
	******************************************************************/
	    const char reason = 0;
		int Home_Step = 0;

		double Home_Traj[26][int(5/dt)]; //S 130404 Home_Traj[26][1000]
		double Home_temp[int(5/dt)]; //S 130404 Home_temp[1000]
		int Home_Count = 0;

		double PieceEndTime = 0.0; // 每一片段動作結束時，會記下當時的 gSysTime

		//int EndCount = 0; // 使用1000 count = 5秒 回到利用 gFlagGoBackPos 指定的原始位置
		int EndTotalCount = 5/dt; // 使用1000 count = 5秒 回到利用 gFlagGoBackPos 指定的原始位置  //S 130404
		//int ShortEndTotalCount = 200; // 使用1000 count = 5秒 回到利用 gFlagGoBackPos 指定的原始位置

		// record encoder data
		fstream enc_data;

		// 指出場景進行到哪邊
		int ScenarioSecNo = 0; 

		//_______________________________________For C2M(Slongz)_______________________________________
		#if AdamsSMode
			#define C2MTrajSize 34
			double C2MTrajBuff[C2MTrajSize];
		#endif
		fstream C2MTrajf;

		double thetaQQ[36];
		//_______________________________________For C2M(Slongz)_______________________________________

		// Offline ENC input
		if (OfflineTraj)
		{
			gInitTime = gKineAll.OfflineNum*dt;
		}
		//*******error code variable
		//bool EposErrorFlag=0;

	  while(1)
	  {
		  if (gControlLife)
		  {
			  if(gSendContTraj == 1 && gContTrajLock == 0)
			  { 	
				  bool WriteOK = false;
				  bool NothingSent = true;

//////////////////////////////////////////////////////////////////////////////////////S 130404 要重改
				  if(gFlagSimulation == CppSimu || gFlagSimulation == RealExp && gFlagTimer== MControl) 
				  {
	  					if (gStartTimeAcquired == 0)
						{
							gStartTimeAcquired = 1;
							QueryPerformanceCounter(&gStartTime);
						}
						QueryPerformanceCounter(&gCurrentTime);
						gSysTime = (gCurrentTime.QuadPart - gStartTime.QuadPart)/gFreqT - gWaitInitSpeechTime;// 延遲開始時間
						if (gSysTime <= 0.0)  // 延遲開始時間
						{
							gSysTime = 0.0;
						}
						//if (gSysTime > 25.0 && ScenarioSecNo == 0 && gFlagWelcomeSaid == 0) // 說第一句話的時間
						//{
						//	//gSpeaker.PlaySoundByFile(gSpeaker.Sound1);
						//	gFlagWelcomeSaid = 1;
						//}
						if (gSysTime > 10.0 && ScenarioSecNo == 0 && gFlagStairSaid == true) // 說第一句話的時間
						{
							gSpeaker.LoadSoundFile("SoundFiles/N03.wav");//N:對於人形機器人來說平地行走要保持平衡是一個相當困難的問題，
											//N:而上下樓梯又是更加的不容易呢。
							gSpeaker.PlaySoundByFile(gSpeaker.Sound1);
							gFlagStairSaid = false;
						}
						if (gSysTime > 8.0 && ScenarioSecNo == 2 && 	gFlagSlSaid01 == true) // 說第一句話的時間
						{
							//N:對於人形機器人來說平地行走要保持平衡是一個相當困難的問題，
											//N:而上下樓梯又是更加的不容易呢。
							//gSpeaker.LoadSoundFile("SoundFiles/N06.wav");
							gSpeaker.PlaySoundByFile(gSpeaker.Sound1);
						
							gFlagSlSaid01 = false;
						}
						if (gSysTime > 22.0 && ScenarioSecNo == 2 && 	gFlagSlSaid02 == true) // 說第一句話的時間
						{
							//N:對於人形機器人來說平地行走要保持平衡是一個相當困難的問題，
											//N:而上下樓梯又是更加的不容易呢。
							gSpeaker.LoadSoundFile("SoundFiles/N06.wav");
							gSpeaker.PlaySoundByFile(gSpeaker.Sound1);
						
							gFlagSlSaid02 = false;
						}
						if (gSysTime > 31.0 && ScenarioSecNo == 2 && 	gFlagSlSaid03 == true) // 說第一句話的時間
						{
							//N:對於人形機器人來說平地行走要保持平衡是一個相當困難的問題，
											//N:而上下樓梯又是更加的不容易呢。
							gSpeaker.LoadSoundFile("SoundFiles/N07.wav");
							gSpeaker.PlaySoundByFile(gSpeaker.Sound1);
							//gSpeaker.LoadSoundFile("SoundFiles/N07.wav");
							gFlagSlSaid03 = false;
						}

						if (gFlagBoostSimu && (gInitCount >= gInterNum)) // 初始化完成後 就可以開始快速解IK ///S 130404
						{
							gSysTime = 10000000000000;
						}
				  } 
//////////////////////////////////////////////////////////////////////////////////////S 130404 要重改
				  				  //***********error code check
				  #if TwinCAT_Mode

				  bool FaultResetFlag=false;
				  if(gFlagSimulation == RealExp)
				  {
					  	  TCatIoInputUpdate( TASK_LLEG_PORTNUMBER ) ;
						  TCatIoInputUpdate( TASK_RLEG_PORTNUMBER ) ;
						if( (EposError[0] = TCAT->EPOS_ErrorCheck(TCAT->pTLL1msIn->LL_I_StatWord_01)) != 0 )
							printf( "Epos3 [01] error: %d \n",EposError[0]);
						if( (EposError[1] = TCAT->EPOS_ErrorCheck(TCAT->pTLL1msIn->LL_I_StatWord_02)) != 0 )
							printf( "Epos3 [02] error: %d \n",EposError[1]);
						if( (EposError[2] = TCAT->EPOS_ErrorCheck(TCAT->pTLL1msIn->LL_I_StatWord_03)) != 0 )
							printf( "Epos3 [03] error: %d \n",EposError[2]);
						if( (EposError[3] = TCAT->EPOS_ErrorCheck(TCAT->pTLL1msIn->LL_I_StatWord_04)) != 0 )
							printf( "Epos3 [04] error: %d \n",EposError[3]);
						if( (EposError[4] = TCAT->EPOS_ErrorCheck(TCAT->pTLL1msIn->LL_I_StatWord_05)) != 0 )
							printf( "Epos3 [05] error: %d \n",EposError[4]);
						if( (EposError[5] = TCAT->EPOS_ErrorCheck(TCAT->pTLL1msIn->LL_I_StatWord_06)) != 0 )
							printf( "Epos3 [06] error: %d \n",EposError[5]);
						if( (EposError[6] = TCAT->EPOS_ErrorCheck(TCAT->pTRL1msIn->RL_I_StatWord_01)) != 0 )
							printf( "Epos3 [07] error: %d \n",EposError[6]);
						if( (EposError[7] = TCAT->EPOS_ErrorCheck(TCAT->pTRL1msIn->RL_I_StatWord_02)) != 0 )
							printf( "Epos3 [08] error: %d \n",EposError[7]);
						if( (EposError[8] = TCAT->EPOS_ErrorCheck(TCAT->pTRL1msIn->RL_I_StatWord_03)) != 0 )
							printf( "Epos3 [09] error: %d \n",EposError[8]);
						if( (EposError[9] = TCAT->EPOS_ErrorCheck(TCAT->pTRL1msIn->RL_I_StatWord_04)) != 0 )
							printf( "Epos3 [10] error: %d \n",EposError[9]);
						if( (EposError[10] = TCAT->EPOS_ErrorCheck(TCAT->pTRL1msIn->RL_I_StatWord_05)) != 0 )
							printf( "Epos3 [11] error: %d \n",EposError[10]);
						if( (EposError[11] = TCAT->EPOS_ErrorCheck(TCAT->pTRL1msIn->RL_I_StatWord_06)) != 0 )
							printf( "Epos3 [12] error: %d \n",EposError[11]);
						for(int i=0;i<12;i++)
							if(EposError[i]!=0)
							{
									cout<<i+1<<" th EPOS Error! Press Any Key to Disable Device "<<"\n";
									system("pause");
									gRecordLogFile();
									TCAT->EtherCATEmergentStop();
									cout<<"Device Disabled! Press Any Key to Reset Fault "<<"\n";
									system("pause");
									FaultResetFlag=true;
							}
						if(FaultResetFlag)
						{
							for(int i=0;i<12;i++)
							{
								if(EposError[i])
									TCAT->EtherCATFaultReset(i);
							}
							FaultResetFlag=false;
						}

						if (gFlagEmergentStop == 1)
						{
							TCAT->EtherCATEmergentStop();
							gRecordLogFile();

							cout<<" Emergent Stop!!! "<<"\n";
							system("pause");
						}				    
				  }
				  #endif


				  gLQs.tic();

				  ////////////////////還是要算FK為了DH原點
				  #if OfflineTraj	// Offline 傳軌跡 
				  
					if (gFlagBoostSimu  && gFlagSimulation == CppSimu) // 初始化完成後 就可以開始快速解IK ///S 130404
					{
						gSysTime = 10000000000000;
					}
				  
				    if ((gSysTime >= TickNumber*dt)) // 在count數內
					{	
						gSendOfflineTraj();	// 送離線ENC
						gEncFeedback();	// 線上收ENC 得FK COG
						gKineAll.FindFK();	// 手的FK和COG
						gKineAll.FindCOG();
						gSaveJointData();
						
						if (gFlagSimulation == RealExp && gKineAll.FlagStayMode == 0) // experiment mode and robot doesn't hold position
						{
							//if(TickNumber%int(dtUSB2C32/dt)==3 && (gFlagArmCtrl || gFlagHandCtrl ))
							//gFlagSendTraj = 1;//讓ArmControlThread那裡也知道要傳了 S 130404 要重改
							#if TwinCAT_Mode
							//___________________________________________________________________________________
							long nError;
							long nError2;
							static int iii=0; 
							if ( ((nError = TCatIoInputUpdate( TASK_RLEG_PORTNUMBER )) == 0 )&& ((nError2 = TCatIoOutputUpdate( TASK_RLEG_PORTNUMBER )) == 0 )&&
									(( nError = TCatIoInputUpdate( TASK_LLEG_PORTNUMBER )) == 0 )&& ( (nError2 = TCatIoOutputUpdate( TASK_LLEG_PORTNUMBER )) == 0 )
								) 
							{ 
								/*TCatIoOutputUpdate( TASK_RLEG_PORTNUMBER ); 
								TCatIoOutputUpdate( TASK_LLEG_PORTNUMBER ); */
							}
							else 
								printf( "TCatInputUpdate(%d) %d failed with 0x%x !\n",
									    TASK_RLEG_PORTNUMBER, iii++, nError ); 
							#endif
							//___________________________________________________________________________________
						}

						TickNumber += 1;
					}
					  
					if (TickNumber == gKineAll.OfflineNum)// end of Offline trajectory 
					{
					//TickNumber = 0;
					NothingSent = false;
					gContTrajLock = 1;
					sprintf_s(gGlobalMessage,"Leg: End of Trajectory\n");
					printf(gGlobalMessage);

					gSendContTraj = 0;
					printf("End of scenario");	
					cout<<endl;
					#if TwinCAT_Mode						
						if (gFlagSimulation == RealExp)
						{
							gRecordLogFile(); //TWINCAT DATA RECORD
							cout<<"Press Any Key To Disable TWINCAT"<<endl;
							system("pause");
					
							TCAT->EtherCATEmergentStop();
							TCAT->~TwinCAT_COM();
						}
					#endif
					}
				  #else

					if ((gSysTime <= gInitTime-0.01)) // 初始化機器人_蹲到初始位置 直接FK
					{
						if (gSysTime > gInitCount*dt*1-0.010 || gFlagBoostSimu) // 滿足時間條件 或者 全速模式 會進入 不然等待 ///XDXD
						{
							if (gInitCount < gInterNum) ///XDXD
							{
								for (int k = 0 ; k < 1 ; k++) // 解4次 每次解5ms 動作 共20ms ///XDXD 只for一次
								{
									if (gKineAll.FlagStayMode == 0) // 機器人要不要stay不動
									{
										// 餵給機器人內插軌跡 gInitMotion*gPNJoints 是因為要乘回DH環境
										for (int h = 0 ; h < 6 ; h++)
										{
											gKineAll.FKLLeg->theta[h+1] = gInitMotion[h][gInitCount+k]*gPNJoints[h];///XDXD
										}
										for (int h = 0 ; h < 6 ; h++)
										{
											gKineAll.FKRLeg->theta[h+1] = gInitMotion[h+6][gInitCount+k]*gPNJoints[h+6];///XDXD
										}
										//泓逸start120228
										for (int h = 0 ; h < 2 ; h++)
										{
											gKineAll.FKRArm->theta[h+1] = gInitMotion[h+12][gInitCount+k]*gPNJoints[h+12];///XDXD
											gKineAll.FKLArm->theta[h+1] = gInitMotion[h+12][gInitCount+k]*gPNJoints[h+12];///XDXD
										}
										for (int h = 0 ; h < 6 ; h++)
										{
											gKineAll.FKLArm->theta[h+4] = gInitMotion[h+14][gInitCount+k]*gPNJoints[h+14];///XDXD
										}
										for (int h = 0 ; h < 6 ; h++)
										{
											gKineAll.FKRArm->theta[h+4] = gInitMotion[h+20][gInitCount+k]*gPNJoints[h+20];///XDXD
										}
									}
									else
									{
										for(int i = 0;i<6;i++)
										{
											gKineAll.FKLArm->theta[i+4] = gLArmOfflineTraj[gArmOfflineTrajCount]*gOfflineJoint[i];
											gKineAll.FKRArm->theta[i+4] = gRArmOfflineTraj[gArmOfflineTrajCount]*gOfflineJoint[i+6];
										/*	gLArmOfflineTraj[gArmOfflineTrajCount] = 0;
											gRArmOfflineTraj[gArmOfflineTrajCount] = 0;*/
											gArmOfflineTrajCount ++;
										}
										gKineAll.FindFK();	// 手的FK和COG
										gKineAll.FindCOG();
									}

									#if RunDynamics				
										//gKineAll.FindD();
										//gAnkleStraytegy();
									#endif

									#if BangBangControl
									gKineAll.NumDiff();
									#endif

                                    #if cogestimate		//開啟cogestimate by 哲軒
									gEncFeedback();	// 線上收ENC 得FK COG											
									COGestimate.compute(gEnc_FKCOG[LogEncCount*3+1], gKineAll.FS_ZMP+2*LogEncCount+1, LogIMUaccely[LogEncCount], LogEncCount,1);	// 機器人saggital方向
									COGestimate.compute(gEnc_FKCOG[LogEncCount*3], gKineAll.FS_ZMP+2*LogEncCount, LogIMUaccelx[LogEncCount], LogEncCount,2); 							                               
                                    #endif	   
                                    
                                    # if uprightfeedback									
									uprightcontrol(LogIMUanglex, LogIMUangley, Lfeedbacktheta, Rfeedbacktheta , LogEncCount);											
									IMU1.Lfeedbackroll[LogEncCount] = Lfeedbacktheta[1] ;
									IMU1.Lfeedbackpitch[LogEncCount]= Lfeedbacktheta[2] ;
									IMU1.Rfeedbackroll[LogEncCount]= Rfeedbacktheta[1] ;
									IMU1.Rfeedbackpitch[LogEncCount]= Rfeedbacktheta[2] ;

									gKineAll.smoothfilter(IMU1.Lfeedbackroll,LogEncCount);
									gKineAll.smoothfilter(IMU1.Lfeedbackpitch,LogEncCount);
									gKineAll.smoothfilter(IMU1.Rfeedbackroll,LogEncCount);
									gKineAll.smoothfilter(IMU1.Rfeedbackpitch,LogEncCount);
									
									Lfeedbacktheta[1] = IMU1.Lfeedbackroll[LogEncCount] ;
									Lfeedbacktheta[2] = IMU1.Lfeedbackpitch[LogEncCount];
									Rfeedbacktheta[1] = IMU1.Rfeedbackroll[LogEncCount] ;
									Rfeedbacktheta[2] = IMU1.Rfeedbackpitch[LogEncCount] ;
									//cout<<"PITCH      "<<IMU1.finalangley[LogEncCount]<<"         "<<LogEncCount<<endl;
									//cout << "LFEED"<< "  " << Lfeedbacktheta[1]<<"  " << Lfeedbacktheta[2] <<endl ;
								    //cout << "RFEED"<< "  " << Rfeedbacktheta[1]<<"  " << Rfeedbacktheta[2] <<endl ;
									/*cout << "LFEED"<< "  " <<  IMU1.Lfeedbackroll[LogEncCount]<<"  " << IMU1.Lfeedbackpitch[LogEncCount] <<" " << LogEncCount <<endl ;
									cout << "RFEED"<< "  " <<  IMU1.Rfeedbackroll[LogEncCount]<<"  " << IMU1.Rfeedbackpitch[LogEncCount]<<" " << LogEncCount <<endl ;*/
									//gEnc_FKCOG (2) 為Lateral 方向 accel from adams 要加負號
									#endif										
									
									//泓逸end120228
									gSaveJointData();
									if (gFlagSimulation == ADAMSSimu)
									{
										if (gFlagEmergentStop == 1){
											break;
										}

										if (gFlagReadForceSensor == 1){
											gKineAll.ForceSensorData(gFlagSimulation, gFlagGoPass, gForceDataCount, LogEncCount, gForceDataLLeg, gForceDataRLeg);
										}
										//_______________________________________For C2M_______________________________________
										#if AdamsSMode
											C2MLoadTraj(C2MTrajSize,C2MTrajBuff,gIthIK,C2MTrajf);
											C2MWrite2Txt(C2MTrajSize,C2MTrajBuff,0,3,C2MTrajf);
										#else
										Trans2Matlab();
										#endif
										gSysTime=gSysTime+0.005;
										//_______________________________________For C2M_______________________________________
									}
								}

								gKineAll.FindFK();
								gKineAll.FindCOG();
								gKineAll.initCOG[0] = gKineAll.COG[0];
								gKineAll.initCOG[1] = gKineAll.COG[1];
								gKineAll.initCOG[2] = gKineAll.COG[2];


								for (int i = 0 ; i < 1 ; i++) ///XDXD 只for一次
								{
									for (int j =  0 ; j < 30 ; j++)
									{
										gShortTrajBufLLeg[i*30+j] = gShortTrajBuf[i*60+j]; // ID 1~6 左腳
										gShortTrajBufRLeg[i*30+j] = gShortTrajBuf[i*60+j+30]; // ID 7~12 右腳
									}
								}
								if (gFlagSimulation == 2 && gKineAll.FlagStayMode == 1) // experiment mode and robot hold position
								{
									if(TickNumber%int(dtUSB2C32/dt)==3  && (gFlagArmCtrl || gFlagHandCtrl) )
									gFlagSendTraj = 1;//讓ArmControlThread那裡也知道要傳了 S 130404 要重改
								}

								if (gFlagSimulation == RealExp && gKineAll.FlagStayMode == 0) // experiment mode and robot doesn't hold position
								{
									if(TickNumber%int(dtUSB2C32/dt)==3 && (gFlagArmCtrl || gFlagHandCtrl ))
									gFlagSendTraj = 1;//讓ArmControlThread那裡也知道要傳了 S 130404 要重改

									#if TwinCAT_Mode
									//___________________________________________________________________________________
									long nError;
									long nError2;
									static int iii=0; 
									if ( ((nError = TCatIoInputUpdate( TASK_RLEG_PORTNUMBER )) == 0 )&& ((nError2 = TCatIoOutputUpdate( TASK_RLEG_PORTNUMBER )) == 0 )&&
										 (( nError = TCatIoInputUpdate( TASK_LLEG_PORTNUMBER )) == 0 )&& ( (nError2 = TCatIoOutputUpdate( TASK_LLEG_PORTNUMBER )) == 0 )
										) 
									{ 
										/*TCatIoOutputUpdate( TASK_RLEG_PORTNUMBER ); 
										TCatIoOutputUpdate( TASK_LLEG_PORTNUMBER ); */
									}
									else 
										printf( "TCatInputUpdate(%d) %d failed with 0x%x !\n",
									     		TASK_RLEG_PORTNUMBER, iii++, nError ); 
									#endif
									//___________________________________________________________________________________
								}

								// read encoder feedback
								//if (gFlagReadEncoder == 1 && gKineAll.FlagStayMode == 0)
								//{
								//	//gReadLegEncoder();	// 改用 TCAT->EtherCATReadEncoder
								//}
							    NothingSent = false;
								gInitCount += 1;
								TickNumber += 1;
							}
						}
					}
					else if ((gSysTime > gInitTime + (DataCount*dt*1)-0.010) && (Home_Step == 0)) // send earlier  ///XDXD 解IK傳軌跡
					{
						//#if ZMPFeedbackMode
						//double* ZZZX=0;
						//double* ZZZY=0;
						//double* CCCX=0;
						//double* CCCY=0;
						//#endif
						////////////////////////////////////////////////////////////////////////////	
						for (int k = 0 ; k < 1 ; k++) // 解4次 每次解5ms 動作 共20ms ///XDXD 只for一次
						{

							//printf("SelIK = %d\n",gKineAll.selIK);
							//#if ZMPFeedbackMode
							//gLQs.ZMPFeedbackControl(ZZZX,ZZZY,CCCX,CCCY,gIthIK);
							//#endif
							////////////////////////////////////////////////////////////////////////////
							if (gKineAll.FlagStayMode == 0) // 機器人要不要完全stay不動
							{
								//gLQs.tic();	
								if (gKineAll.FlagStaySquatMode == 0)
								{
									gIKStep();	
								} 
								else //20140324 DEMO stabilizer 控制可以加在這，staytime足夠長或使用FlagStayBreak跳出
								{
									//stabilizer and imitation
									//if break
									
									if (kbhit())//鍵盤有按到
									{
										char key=getch();
										if ( key=='p')
										{
											gKineAll.FlagStayBreak = 1;
										}
										
									}
								}
										
								//gLQs.toc2();	
								
							}
							else
							{
								for(int i = 0;i<6;i++)
								{
									gKineAll.FKLArm->theta[i+4] = gLArmOfflineTraj[gArmOfflineTrajCount]*gOfflineJoint[i];
									gKineAll.FKRArm->theta[i+4] = gRArmOfflineTraj[gArmOfflineTrajCount]*gOfflineJoint[i+6];
								/*	gLArmOfflineTraj[gArmOfflineTrajCount] = 0;
									gRArmOfflineTraj[gArmOfflineTrajCount] = 0;*/
									gArmOfflineTrajCount ++;
								}
								gKineAll.FindFK();	// 手的FK和COG
								gKineAll.FindCOG();
							}														
							
							#if RunDynamics
								//gKineAll.FindD();
								//gAnkleStraytegy();
								gAKHS(delta_frame+12*LogEncCount);
							#endif

							#if BangBangControl
							gKineAll.NumDiff();
							#endif							
							
                            #if cogestimate	//開啟cogestimate by 哲軒
							gEncFeedback();	// 線上收ENC 得FK COG
							COGestimate.compute(gEnc_FKCOG[LogEncCount*3+1], gKineAll.FS_ZMP+2*LogEncCount+1, LogIMUaccely[LogEncCount], LogEncCount,1);	// 機器人saggital方向
							COGestimate.compute(gEnc_FKCOG[LogEncCount*3], gKineAll.FS_ZMP+2*LogEncCount, LogIMUaccelx[LogEncCount], LogEncCount,2);					
							#endif						   
                              
                            #if uprightfeedback
							uprightcontrol(LogIMUanglex, LogIMUangley, Lfeedbacktheta, Rfeedbacktheta , LogEncCount);
																	
							IMU1.Lfeedbackroll[LogEncCount] = Lfeedbacktheta[1] ;
							IMU1.Lfeedbackpitch[LogEncCount]= Lfeedbacktheta[2] ;
							IMU1.Rfeedbackroll[LogEncCount]= Rfeedbacktheta[1] ;
							IMU1.Rfeedbackpitch[LogEncCount]= Rfeedbacktheta[2] ;

							gKineAll.smoothfilter(IMU1.Lfeedbackroll,LogEncCount);
							gKineAll.smoothfilter(IMU1.Lfeedbackpitch,LogEncCount);
							gKineAll.smoothfilter(IMU1.Rfeedbackroll,LogEncCount);
							gKineAll.smoothfilter(IMU1.Rfeedbackpitch,LogEncCount);
									
							Lfeedbacktheta[1] = IMU1.Lfeedbackroll[LogEncCount] ;
							Lfeedbacktheta[2] = IMU1.Lfeedbackpitch[LogEncCount];
							Rfeedbacktheta[1] = IMU1.Rfeedbackroll[LogEncCount] ;
							Rfeedbacktheta[2] = IMU1.Rfeedbackpitch[LogEncCount] ;	
							//cout<<"PITCH      "<<IMU1.finalangley[LogEncCount]<<"         "<<LogEncCount<<endl;
							//cout << "LFEED"<< "  " << Lfeedbacktheta[1]<<"  " << Lfeedbacktheta[2] <<endl ;
							//cout << "RFEED"<< "  " << Rfeedbacktheta[1]<<"  " << Rfeedbacktheta[2] <<endl ;
							//cout << "LFEED"<< "  " <<  IMU1.Lfeedbackroll[LogEncCount]<<"  " << IMU1.Lfeedbackpitch[LogEncCount] <<" " << LogEncCount <<endl ;
							//cout << "RFEED"<< "  " <<  IMU1.Rfeedbackroll[LogEncCount]<<"  " << IMU1.Rfeedbackpitch[LogEncCount]<<" " << LogEncCount <<endl ;	//gEnc_FKCOG (2) 為Lateral 方向 accel from adams 要加負號
						   	#endif
							
							gSaveJointData();

							#if CheckingMode 
								if(gFlagReadForceSensor==0)
								{
							//cout<<KineAll.selIK<<"\n";
							//C2MLoadTraj(456,thetaQQ,gIthIK,C2MTrajf);
							//C2MWrite2Txt(36,thetaQQ,0,2,C2MTrajf);
									//C2MLoadTraj(12,thetaQQ,gIthIK,C2MTrajf);	// 每軸軌跡
									//C2MWrite2Txt(12,thetaQQ,0,2,C2MTrajf);

							//cout<<gSysTime<<endl;
							//20121218doratom為了要看腳尖的軌跡(走斜坡用)//
							//C2MLoadTraj(6,thetaQQ,gIthIK,C2MTrajf);
							//C2MWrite2Txt(6,thetaQQ,0,2,C2MTrajf);
							//20121218doratom為了要看腳尖的軌跡(走斜坡用)//
								}
							#endif

							if (gFlagSimulation == ADAMSSimu)
							{
								if (gFlagEmergentStop == 1){
									break;
								}

								if (gFlagReadForceSensor == 1){
									gKineAll.ForceSensorData(gFlagSimulation, gFlagGoPass, gForceDataCount, LogEncCount, gForceDataLLeg, gForceDataRLeg);
								}
								//_______________________________________For C2M_______________________________________
								#if AdamsSMode
									C2MLoadTraj(C2MTrajSize+FootpadRotation,C2MTrajBuff,gIthIK,C2MTrajf);
									C2MWrite2Txt(C2MTrajSize,C2MTrajBuff,0,3,C2MTrajf);
								#else
									Trans2Matlab();
								#endif
								gSysTime=gSysTime+0.005;
								//C2MLoadTraj(C2MTrajSize,C2MTrajBuff,gIthIK,C2MTrajf);
								//#if ZMPFeedbackMode
								//C2MWrite2Txt(C2MTrajSize,C2MTrajBuff,gIthIK,5,C2MTrajf);
								//#else
								//C2MWrite2Txt(C2MTrajSize,C2MTrajBuff,0,3,C2MTrajf);
								//#endif

								//_______________________________________For C2M_______________________________________
							}
						}

						for (int i = 0 ; i < 1 ; i++) ///XDXD 只for一次
						{
							for (int j =  0 ; j < 30 ; j++)
							{
								gShortTrajBufLLeg[i*30+j] = gShortTrajBuf[i*60+j];
								gShortTrajBufRLeg[i*30+j] = gShortTrajBuf[i*60+j+30];
							}
						}
						if (gFlagSimulation == 2 && gKineAll.FlagStayMode == 1) // experiment mode and robot doesn't hold position
						{
									if(TickNumber%int(dtUSB2C32/dt)==3 &&  (gFlagArmCtrl || gFlagHandCtrl) )
									gFlagSendTraj = 1;//讓ArmControlThread那裡也知道要傳了 S 130404 要重改
						}

						if (gFlagSimulation == RealExp && gKineAll.FlagStayMode == 0) // experiment mode and robot doesn't hold position
						{
									if(TickNumber%int(dtUSB2C32/dt)==3 && (gFlagArmCtrl || gFlagHandCtrl ))
									gFlagSendTraj = 1;//讓ArmControlThread那裡也知道要傳了 S 130404 要重改
							#if TwinCAT_Mode
							//___________________________________________________________________________________


									long nError;
									long nError2;
									static int iii=0; 
									if ( ((nError = TCatIoInputUpdate( TASK_RLEG_PORTNUMBER )) == 0 )&& ((nError2 = TCatIoOutputUpdate( TASK_RLEG_PORTNUMBER )) == 0 )&&
										 (( nError = TCatIoInputUpdate( TASK_LLEG_PORTNUMBER )) == 0 )&& ( (nError2 = TCatIoOutputUpdate( TASK_LLEG_PORTNUMBER )) == 0 )
										) 
									{
										TCatIoOutputUpdate( TASK_RLEG_PORTNUMBER ); 
										TCatIoOutputUpdate( TASK_LLEG_PORTNUMBER ); 
									}
									else 
										printf( "TCatInputUpdate(%d) %d failed with 0x%x !\n",
												TASK_RLEG_PORTNUMBER, iii++, nError );
							//___________________________________________________________________________________							
							#endif
						}

						// read encoder feedback
						//if (gFlagReadEncoder == 1 && gKineAll.FlagStayMode == 0)
						//{
						//	//gReadLegEncoder();	// 改用 TCAT->EtherCATReadEncoder
						//}
   					    NothingSent = false;
						DataCount += 1;
						TotalDataCount += 1;
						TickNumber += 1;
				  }

				  if (TotalDataCount >= gStopTrajLen || gKineAll.FlagStayBreak == 1)
				  {

					if (Home_Step == HomeInitial) // 正要開始回家 給軌跡 更改home_step後就不會再進之前解IK的if
					{
						if (gKineAll.FlagStayMode == 1) // stay 模式，機器人理應是腳伸直 並且直接結束
						{
							for (int jp = 0 ; jp < 6 ; jp++) 	// 左腳
							{
								EndTotalCount = 300;
								gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.FKLLeg->theta[jp+1],gInitThetas[jp]*gPNJoints[jp],EndTotalCount,Home_temp);
								gFlagCurrentPos = 1;
								for (int kp = 0 ; kp < EndTotalCount ; kp++)
									Home_Traj[jp][kp] = Home_temp[kp];
							}
							for (int jp = 6 ; jp < 12 ; jp++)	// 右腳
							{
								EndTotalCount = 300;
								gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.FKRLeg->theta[jp-5],gInitThetas[jp]*gPNJoints[jp],EndTotalCount,Home_temp);
								gFlagCurrentPos = 1;
								for (int kp = 0 ; kp < EndTotalCount ; kp++)
									Home_Traj[jp][kp] = Home_temp[kp];
							}

							for (int jp = 0 ; jp < 6 ; jp++)	// 左手
							{
								EndTotalCount = 300;
								gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.FKLArm->theta[jp+4],0,EndTotalCount,Home_temp);
								gFlagCurrentPos = 1;
								for (int kp = 0 ; kp < EndTotalCount ; kp++)
									Home_Traj[jp+14][kp] = Home_temp[kp];
							}

							for (int jp = 0 ; jp < 6 ; jp++)	// 右手
							{
								EndTotalCount = 300;
								gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.FKRArm->theta[jp+4],0,EndTotalCount,Home_temp);
								gFlagCurrentPos = 1;
								for (int kp = 0 ; kp < EndTotalCount ; kp++)
									Home_Traj[jp+20][kp] = Home_temp[kp];
							}
						}
						else // 正常模式 由jp與kp分配軌跡count數
						{
							for (int jp = 0 ; jp < 6 ; jp++)	// 左腳
							{
								if (gFlagGoBackPos == ZeroHome)
								{
									EndTotalCount = 1000;
									gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.FKLLeg->theta[jp+1],0,EndTotalCount,Home_temp);
									gFlagCurrentPos = 0;
								}
								else if (gFlagGoBackPos == BentHome)
								{
			 						EndTotalCount = 600;
									gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.FKLLeg->theta[jp+1],gInitThetas[jp]*gPNJoints[jp],EndTotalCount,Home_temp);	
									gFlagCurrentPos = 1;
								}
								else if (gFlagGoBackPos == ShiftZeroHome)
								{
									//gZeroJointPosShift
									EndTotalCount = 1000;
									gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.FKLLeg->theta[jp+1],gZeroJointPosShift[jp]*gPNJoints[jp],EndTotalCount,Home_temp);	
									gFlagCurrentPos = 2;
								}

								else if (gFlagGoBackPos == SlopeHome) // 哲瑄
								{
									EndTotalCount = 1000;
									gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.FKLLeg->theta[jp+1],gKineAll.FKLLeg->theta[jp+1],EndTotalCount,Home_temp);	
									gFlagCurrentPos = 3;
								}

								for (int kp = 0 ; kp < EndTotalCount ; kp++)
									Home_Traj[jp][kp] = Home_temp[kp];
							}
							for (int jp = 6 ; jp < 12 ; jp++)	// 右腳
							{
								if (gFlagGoBackPos == ZeroHome)
								{
									EndTotalCount = 1000;
									gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.FKRLeg->theta[jp-5],0,EndTotalCount,Home_temp);
									gFlagCurrentPos = 0;
								}
								else if (gFlagGoBackPos == BentHome)
								{
									EndTotalCount = 600;
									gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.FKRLeg->theta[jp-5],gInitThetas[jp]*gPNJoints[jp],EndTotalCount,Home_temp);
									gFlagCurrentPos = 1;
								}
								else if (gFlagGoBackPos == ShiftZeroHome)
								{
									EndTotalCount = 1000;
									gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.FKRLeg->theta[jp-5],gZeroJointPosShift[jp]*gPNJoints[jp],EndTotalCount,Home_temp);
									gFlagCurrentPos = 2;
								}

									else if (gFlagGoBackPos == SlopeHome) // 哲軒
								{
									//gZeroJointPosShift
									EndTotalCount = 1000;
									gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.FKRLeg->theta[jp-5],gKineAll.FKRLeg->theta[jp-5],EndTotalCount,Home_temp);	
									gFlagCurrentPos = 3;
								}
		
								for (int kp = 0 ; kp < EndTotalCount ; kp++)
									Home_Traj[jp][kp] = Home_temp[kp];
							}
							for (int jp = 0 ; jp < 6 ; jp++)	// 左手
							{
								if (gFlagGoBackPos == ZeroHome)
								{
									EndTotalCount = 1000;
									gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.FKLArm->theta[jp+4],0,EndTotalCount,Home_temp);
									gFlagCurrentPos = 0;
								}
								else if (gFlagGoBackPos == BentHome)
								{
									EndTotalCount = 600;
									gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.FKLArm->theta[jp+4],gInitThetas[jp+14]*gPNJoints[jp+14],EndTotalCount,Home_temp);	
									gFlagCurrentPos = 1;
								}
								else if (gFlagGoBackPos == ShiftZeroHome)
								{
									EndTotalCount = 1000;
									gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.FKLArm->theta[jp+4],gZeroJointPosShift[jp+14]*gPNJoints[jp+14],EndTotalCount,Home_temp);
									gFlagCurrentPos = 2;
								}
								else if (gFlagGoBackPos == SlopeHome) // 哲軒
								{
									EndTotalCount = 1000;
									gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.FKLArm->theta[jp+4],gKineAll.FKLArm->theta[jp+4],EndTotalCount,Home_temp);
									gFlagCurrentPos = 3;
								}
								
								for (int kp = 0 ; kp < EndTotalCount ; kp++)
									Home_Traj[jp+14][kp] = Home_temp[kp];
							}
							for (int jp = 0 ; jp < 6 ; jp++)	// 右手
							{
								if (gFlagGoBackPos == ZeroHome)
								{
									EndTotalCount = 1000;
									gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.FKRArm->theta[jp+4],0,EndTotalCount,Home_temp);
									gFlagCurrentPos = 0;
								}
								else if (gFlagGoBackPos == BentHome)
								{
									EndTotalCount = 600;
									gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.FKRArm->theta[jp+4],gInitThetas[jp+20]*gPNJoints[jp+20],EndTotalCount,Home_temp);	
									gFlagCurrentPos = 1;
								}
								else if (gFlagGoBackPos == ShiftZeroHome)
								{
									EndTotalCount = 1000;
									gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.FKRArm->theta[jp+4],gZeroJointPosShift[jp+20]*gPNJoints[jp+20],EndTotalCount,Home_temp);
									gFlagCurrentPos = 2;
								}
								else if (gFlagGoBackPos == SlopeHome) // 哲瑄
								{
									EndTotalCount = 1000;
									gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.FKRArm->theta[jp+4],gKineAll.FKRArm->theta[jp+4],EndTotalCount,Home_temp);
									gFlagCurrentPos = 2;
								}

								for (int kp = 0 ; kp < EndTotalCount ; kp++)
									Home_Traj[jp+20][kp] = Home_temp[kp];		
							}
						}

						Home_Step = Homing; // 這個 if 只會執行一次
						// 記下現在時間 等等要用
						QueryPerformanceCounter(&gCurrentTime);
						PieceEndTime = (gCurrentTime.QuadPart - gStartTime.QuadPart)/gFreqT - gWaitInitSpeechTime;// 延遲開始時間
					}

					if (Home_Step == Homing) // HomeInitial 0執行完 馬上接著執行Homing 1，開始慢慢歸位
					{

						if (gSysTime > PieceEndTime + (Home_Count*dt)-0.010) // send earlier  //S 130404 要重改
						{
								for (int k = 0 ; k < 1 ; k++) // 解4次 每次解5ms 動作 共20ms  //S 130404 要重改 只for一次
								{
									for (int h = 0 ; h < 6 ; h++)	// 左腳
									{
										gKineAll.FKLLeg->theta[h+1] = Home_Traj[h][Home_Count+k];
									}
									for (int h = 0 ; h < 6 ; h++)	// 右腳
									{
										gKineAll.FKRLeg->theta[h+1] = Home_Traj[h+6][Home_Count+k];
									}
									for (int h = 0 ; h < 6 ; h++)	// 左手
									{
										gKineAll.FKLArm->theta[h+4] = Home_Traj[h+14][Home_Count+k];//S 130404 要重改
									}
									for (int h = 0 ; h < 6 ; h++)	// 右手
									{
										gKineAll.FKRArm->theta[h+4] = Home_Traj[h+20][Home_Count+k];//S 130404 要重改
									}
	
									#if RunDynamics
										//gEncFeedback();	// 20130831 WZ 把ENC直接在此輸入
										//gKineAll.FindD();
										//gAnkleStraytegy();
										gAKHS(delta_frame+12*LogEncCount);
									#endif

									#if BangBangControl
									gKineAll.NumDiff();
									#endif
									
									#if cogestimate		//開啟cogestimate by 哲軒
									gEncFeedback();	// 線上收ENC 得FK COG
									COGestimate.compute(gEnc_FKCOG[LogEncCount*3+1], gKineAll.FS_ZMP+2*LogEncCount+1, LogIMUaccely[LogEncCount], LogEncCount,1);                                			    // 機器人saggital方向
									COGestimate.compute(gEnc_FKCOG[LogEncCount*3], gKineAll.FS_ZMP+2*LogEncCount, LogIMUaccelx[LogEncCount], LogEncCount,2);
		                            #endif						   
                                    # if uprightfeedback
									uprightcontrol(LogIMUanglex, LogIMUangley, Lfeedbacktheta, Rfeedbacktheta , LogEncCount);
																		
									IMU1.Lfeedbackroll[LogEncCount] = Lfeedbacktheta[1] ;
									IMU1.Lfeedbackpitch[LogEncCount]= Lfeedbacktheta[2] ;
									IMU1.Rfeedbackroll[LogEncCount]= Rfeedbacktheta[1] ;
									IMU1.Rfeedbackpitch[LogEncCount]= Rfeedbacktheta[2] ;

									gKineAll.smoothfilter(IMU1.Lfeedbackroll,LogEncCount);
									gKineAll.smoothfilter(IMU1.Lfeedbackpitch,LogEncCount);
									gKineAll.smoothfilter(IMU1.Rfeedbackroll,LogEncCount);
									gKineAll.smoothfilter(IMU1.Rfeedbackpitch,LogEncCount);
									
									Lfeedbacktheta[1] = IMU1.Lfeedbackroll[LogEncCount] ;
									Lfeedbacktheta[2] = IMU1.Lfeedbackpitch[LogEncCount];
									Rfeedbacktheta[1] = IMU1.Rfeedbackroll[LogEncCount] ;
									Rfeedbacktheta[2] = IMU1.Rfeedbackpitch[LogEncCount] ;
									
									//cout<<"PITCH      "<<IMU1.finalangley[LogEncCount]<<"         "<<LogEncCount<<endl;
									//cout << "LFEED"<< "  " <<  IMU1.Lfeedbackroll[LogEncCount]<<"  " << IMU1.Lfeedbackpitch[LogEncCount] <<" " << LogEncCount <<endl ;
									//cout << "RFEED"<< "  " <<  IMU1.Rfeedbackroll[LogEncCount]<<"  " << IMU1.Rfeedbackpitch[LogEncCount]<<" " << LogEncCount <<endl ;
								    // cout << "LFEED"<< "  " << Lfeedbacktheta[1]<<"  " << Lfeedbacktheta[2] <<endl ;
								    //cout << "RFEED"<< "  " << Rfeedbacktheta[1]<<"  " << Rfeedbacktheta[2] <<endl ;//gEnc_FKCOG (2) 為Lateral 方向 accel from adams 要加負號
									#endif
									
									gSaveJointData();

									if (gFlagSimulation == ADAMSSimu)
									{
										if (gFlagEmergentStop == 1){
											break;
										}

										if (gFlagReadForceSensor == 1){
											gKineAll.ForceSensorData(gFlagSimulation, gFlagGoPass, gForceDataCount, LogEncCount, gForceDataLLeg, gForceDataRLeg);
										}
										//_______________________________________For C2M_______________________________________
										#if AdamsSMode									
										C2MLoadTraj(C2MTrajSize,C2MTrajBuff,gIthIK,C2MTrajf);
										C2MWrite2Txt(C2MTrajSize,C2MTrajBuff,0,3,C2MTrajf);
										#else
										Trans2Matlab();
										#endif
										gSysTime=gSysTime+0.005;
										//_______________________________________For C2M_______________________________________
									}
								}

								gKineAll.FindFK();
								gKineAll.FindCOG();
								gKineAll.initCOG[0] = gKineAll.COG[0];
								gKineAll.initCOG[1] = gKineAll.COG[1];
								gKineAll.initCOG[2] = gKineAll.COG[2];

								for (int i = 0 ; i < 1 ; i++) //XDXD 只for一次
								{
									for (int j =  0 ; j < 30 ; j++)
									{
										gShortTrajBufLLeg[i*30+j] = gShortTrajBuf[i*60+j]; // ID 1~6 左腳
										gShortTrajBufRLeg[i*30+j] = gShortTrajBuf[i*60+j+30]; // ID 7~12 右腳
									}
								}

								if (gFlagSimulation == RealExp)
								{

									if(TickNumber%int(dtUSB2C32/dt)==3 && (gFlagArmCtrl || gFlagHandCtrl ))
									gFlagSendTraj = 1;//讓ArmControlThread那裡也知道要傳了 S 130404 要重改
									#if TwinCAT_Mode
									//___________________________________________________________________________________

									long nError;
									long nError2;
									static int iii=0; 
									if ( ((nError = TCatIoInputUpdate( TASK_RLEG_PORTNUMBER )) == 0 )&& ((nError2 = TCatIoOutputUpdate( TASK_RLEG_PORTNUMBER )) == 0 )&&
										 (( nError = TCatIoInputUpdate( TASK_LLEG_PORTNUMBER )) == 0 )&& ( (nError2 = TCatIoOutputUpdate( TASK_LLEG_PORTNUMBER )) == 0 )
										) 
									{ 
										TCatIoOutputUpdate( TASK_RLEG_PORTNUMBER ); 
										TCatIoOutputUpdate( TASK_LLEG_PORTNUMBER ); 
									}
										else 
											printf( "TCatInputUpdate(%d) %d failed with 0x%x !\n",
												TASK_RLEG_PORTNUMBER, iii++, nError ); 
									//___________________________________________________________________________________									
									#endif 
								}

								// read encoder feedback
								//if (gFlagReadEncoder == 1)
								//{
								//	//gReadLegEncoder();	// 改用 TCAT->EtherCATReadEncoder
								//}
	
							    NothingSent = false;
								
								Home_Count += 1;

								if (Home_Count >= EndTotalCount) // EndTotalCount
								{
									Home_Step = HomeFinished;
								}

								DataCount += 1;
								TotalDataCount += 1;
								TickNumber += 1;
						}
					}
					else if (Home_Step == HomeFinished)// 代表回家完成
					{
						 gSendContTraj = 0;
						 DataCount = 0;
						 TotalDataCount = 0;
						 gInitCount = 0;
						 Home_Count = 0;
						 Home_Step = HomeInitial;
						 TickNumber = 0;
						 gKineAll.FlagStayBreak = 0;
						 gKineAll.FlagStaySquatMode = 0;
						 LogPreLoadCount= 0;// 要改!
						 if(gFlagCurrentPos == 1)
							 LogPreLoadCount= 1000;

						 if (gFlagSimulation == RealExp)
						 {
							 // 新版 先不丟end of traj. 這樣接下來可以繼續 到真正完成 再丟
							/// gpDP->State[2]=ID_EndOfContTraj; // tell C32 it is the end of the continuous trajectory
							/// gpPortLL->_write(gpDP->State,5);
							/// gpPortRL->_write(gpDP->State,5);
						 }
						 else if (gFlagSimulation == ADAMSSimu)
						 {
							 if (gFlagEmergentStop == 1){
								 break;
							 }
							 //_______________________________________For C2M_______________________________________
							 #if AdamsSMode
								 C2MWrite2Txt(C2MTrajSize,C2MTrajBuff,0,4,C2MTrajf);
							 #endif
							 //_______________________________________For C2M_______________________________________
						 }
						 NothingSent = false;
						 gContTrajLock = 1;
						 sprintf_s(gGlobalMessage,"Leg: End of Trajectory\n");
						 printf(gGlobalMessage);

						 gPrepareScenarioScript(++ScenarioSecNo); // 初始劇本在壓下start的時候就先設定好了!! 這邊是每次的後續劇本!!

						 if (gEndOfScript) // 劇情結束會跳出 結束 機器人停止
						 {
							 gSendContTraj = 0;
							 printf("End of scenario");	
						 	 cout<<endl;

						  if(gFlagSimulation == RealExp)
						  {
							 gRecordLogFile(); //TWINCAT DATA RECORD

							 if (gFlagReadEncoder == 1)
							 {
								//// save encoder
								//enc_data.open("enc_data.txt",ios::out);
								//for (int k = 0 ; k < 12 ; k++){
								//	for (int jj = 0 ; jj < gEncoderReadIndex; jj++){
								//		enc_data << gReadEnc[k][jj]  << " ";
								//	}
								//	enc_data << endl;
								//}
								//enc_data.close();
								//// save encoder
							 }
							 cout<<"Press Any Key To Disable TWINCAT"<<endl;
							 system("pause");
							 #if TwinCAT_Mode
							 TCAT->EtherCATEmergentStop();
							 TCAT->~TwinCAT_COM();
							 #endif
							}
						 }

   					     if (gFlagCurrentPos == 0 || gFlagCurrentPos == 2) // 完全站直 或者等待手臂舉起自我介紹
						 {
							  gInterNum = 1000;
							  gInitTime = dt*gInterNum;
						 }
						 else if (gFlagCurrentPos == 1) // 蹲的初始化位置
						 {
							  gInterNum = 0;
							  gInitTime = 0; // 一定不會執行 直接跳到開始
						 }

						 // 重新生成初始準備動作 若為 BentHome 則直接接下一個場景解IK 不進MotionControl第一段
						 for (int k = 0 ; k < 6 ; k++)	// 左腳
						 {
								if (gInterNum == 0)	// BentHome
								{
									// do nothing
								}
								else
								{
									gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.FKLLeg->theta[k+1]*gPNJoints[k],gInitThetas[k],gInterNum,gTempPoly);
									for (int j=0 ; j < gInterNum; j++)
									{
										gInitMotion[k][j] = gTempPoly[j];
										//cout<<gTempPoly[j]<<endl;
									}
								}
						 }
						 for (int k = 6 ; k < 12 ; k++)	// 右腳
						 {
								if (gInterNum == 0)	// BentHome
								{
									// do nothing
								}
								else
								{
									gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.FKRLeg->theta[k-5]*gPNJoints[k],gInitThetas[k],gInterNum,gTempPoly);
									for (int j=0 ; j < gInterNum; j++)
									{
										gInitMotion[k][j] = gTempPoly[j];
									}
								}
						 }

						 for (int k = 0 ; k <  6 ; k++)
						 {
								if (gInterNum == 0)
								{
									// do nothing
								}
								else
								{
									gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.FKLArm->theta[k+4],gInitThetas[k+14],gInterNum,gTempPoly);
									for (int j=0 ; j < gInterNum; j++)
									{
										gInitMotion[k+14][j] = gTempPoly[j];
									}
								}
						 }

						 for (int k = 0 ; k <  6 ; k++)
						 {
								if (gInterNum == 0)
								{
									// do nothing
								}
								else
								{
									gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.FKRArm->theta[k+4],gInitThetas[k+20],gInterNum,gTempPoly);
									for (int j=0 ; j < gInterNum; j++)
									{
										gInitMotion[k+20][j] = gTempPoly[j];
									}
								}
						 }
					}
			
				  }
				  else
				  #endif	// OfflineTraj
				  {
					 sprintf_s(gGlobalMessage,"Trajectory Count = %d\n",TickNumber);
					 //sprintf_s(gGlobalMessage,"Trajectory Count = %d\n",TotalDataCount);
					 //sprintf_s(gBipedInfo1,"Min R Knee Angle = %f \n",MinRKnee);
					 //sprintf_s(gBipedInfo2,"Min L Knee Angle = %f \n",MinLKnee);
					 //sprintf_s(gBipedInfo1,"Max R Knee Pitch Angle = %f \n",MinRKnee);
					 //sprintf_s(gBipedInfo2,"Max L Knee Pitch Angle = %f \n",MinLKnee);
					 sprintf_s(gBipedInfo1,"Max R Knee Pitch Angle = %d \n",int(LogEnc[9+12*(LogEncCount-1)])  );
					 sprintf_s(gBipedInfo2,"Max L Knee Pitch Angle = %d \n",int(LogEnc[3+12*(LogEncCount-1)])   );
					 //Sleep(5);
				  }

				  if (NothingSent)
				  {
					  if (gFlagBoostSimu == false)
						Sleep(1); // saving CPU resource and fast check
					  else
						  Sleep(1);
				  }
				  gLQs.toc3();	// 配合三段最開頭的gLQs.tic(); 在command window 顯示最大計時時間
			  }
			  else
			  {
				  Sleep(200); // save the CPU resource and idle
			  }

			  // 保護裝置，正常的話不會啟動
			  if(gSendContTraj == 1 && gContTrajLock == 1)
			  {
				 gSendContTraj = 0;
				 cout << "請將下拉式選單重新切換到 traj " << endl;
			  }
			  // 保護裝置，正常的話不會啟動

		  }
		  else
		  {
			  break;
		  }

		  if (gFlagBoostSimu)
		  {
			  // 調整這個值 直到boost mode 眼睛可視
			  Sleep(1);
		  }

	  }

}

void gTimerThread(void)
{
	/******************************************************************
	input: void
	output: void

	Note:

	// Timer於1ms寄時使用的執行緒(未完成,且可能需要耗費額外資源)
	   
	******************************************************************/	
	while(1)
	{
		if (gTimerLife)
		{
			if(gSendContTraj == 1 )
			 { 	 
				 bool WriteOK = false;
				 //bool NothingSent = true;

					if (gStartTimeAcquired == 0)
					{
						gStartTimeAcquired = 1;
						QueryPerformanceCounter(&gStartTime);
					}
					QueryPerformanceCounter(&gCurrentTime);
					gSysTime = (gCurrentTime.QuadPart - gStartTime.QuadPart)/gFreqT - gWaitInitSpeechTime;// 延遲開始時間
					if (gSysTime <= 0.0)  // 延遲開始時間
					{
						gSysTime = 0.0;
					}
					//if (gSysTime>= TickNumber*0.001 )// || (gSysTime)>= (TickNumber*0.001-0.00005) )
					//{
					//
					////TimeRecord[TickNumber]=gSysTime*10000;

					////if (TickNumber%1000==0)
					////	{
					////		TimeRecord[(TickNumber/1000)]=gSysTime;
					////	}

					//TickNumber+=1;
					//
					//}
					//TimeRecord[DataCount]=gSysTime;
					//Sleep(1);
					//if (gSysTime > 0.02+0.02*DataCount)
					//{
						//if (handindex == 119760)
						//	handflag=false;
						//else if (handindex==0)
						//	handflag=true;
						//	
						//WriteOK = port1->_write(ContTrajData,240); // < 2ms
						//if (!WriteOK)
						//	printf("Download Failed: HHHHHHHHHHHHHHHH\n");
						//if(handflag==1)
						//{
						//	ContTrajData+=240;
						//	handindex+=240;
						//}
						//else
						//{
						//	handindex-=240;
						//	ContTrajData-=240;
						//}
						//DataCount += 1;
					//}
				if (gSysTime >= 10000)
				{
				break;
				}
			}
		}//gControlLife

		//Sleep(1);
	}// while(1)
	//for (int i=0;i<10000;i++)
	//	cout<<i<<" th"<<"SystemTime (ms) : "<<TimeRecord[i]<<endl;
}

  void gFaceControlThread(void)
  {
	/******************************************************************
	input: void
	output: void

	Note:
	//  機器人臉部會連續行動就是因為MotionControlThread會執行這個function裡面的while(1)
	// 停止時會跳出while迴圈 然後這個thread就會自動結束生命週期
	// 目前臉部動作暫時不利用此function 因為已經改成LED臉了!!
	******************************************************************/

	  int DataCountHead = 0;
	  int SampTimeHead = 0.025;
	  unsigned char CmdHead[3] = {255,1,100};



	  //while(1)
	  //{
		 // if (gFaceLife)
		 // {

			//	  bool WriteOK = false;
			//	  bool NothingSent = true;
			//		
			//		if (gSysTime > SampTimeHead*DataCountHead) // send earlier delta_t = 10ms
			//		{
			//			
			//			if (gFlagSimulation == 2)
			//			{
			//				//WriteOK = gpPortHead->_write(CmdHead,3); // < 2ms
			//				//if (!WriteOK)
			//					// printf("Download Failed: Head\n");

			//				NothingSent = false;
			//			}

			//			DataCountHead++;
			//			//Sleep(95); // 大約每秒10次 可以改
			//		}

			//	  if (NothingSent)
			//	  {
			//		  if (gFlagBoostSimu == false)
			//			Sleep(1); // saving CPU resource and fast check
			//	  }

		 // }
		 // else
		 // {
			//  break;
		 // }
	  //}

  }


  void gArmControlThread(void)
  {
	  /******************************************************************
	  input: void
	  output: void

	  Note:
	  //  機器人手臂會連續行動就是因為MotionControlThread會執行這個function裡面的while(1)
	  // 停止時會跳出while迴圈 然後這個thread就會自動結束生命週期
	  ******************************************************************/
	  int index_buf = 0;
	  int index_read = 0;
	  const char reason = 0;

	  bool WriteOK;
	  int DataCount_Arm = 0;

	  int LenMax = 0;

	  int DataCountHand = 0; //至峻20130410
	  int DataCountStart = 0;
	  //int DataCountStart = 0;



	  while(1)
	  {
		  if (gArmControlLife)
		  {

			  if(gSendContTrajArm == 1 && gContTrajLockArm == 0)
			  {
				  bool WriteOK = false;
				  bool USB_Used = false;
				  bool Nothing_Sent = true;
				  if (gFlagSendTraj == 1)
				  {
					  gFlagSendTraj = 0;
					  Nothing_Sent = false;

					  if (gFlagSimulation == 2) // experiment mode
					  {

						  if (gArmTrajFleg == 0)
						  {
							  //////////for(int i = 0;i<4;i++)
							  ////////// {
							  //////////for(int j = 0;j<4;j++)
							  //////////{
							  //////////	gShortTrajBufLArm[i*30+26+j] = 0;
							  //////////	gShortTrajBufRArm[i*30+26+j] = 0;
							  //////////}
							  ////////// } //QQ 0517
							  //for(int i = 0;i<30;i++)
							  //{
							  // ShortTrajBufLArmSent[i] = ShortTrajBufLArm[i];
							  // ShortTrajBufLArmSent[i+30] = ShortTrajBufLArm[i+60];
							  // ShortTrajBufRArmSent[i] = ShortTrajBufRArm[i];
							  // ShortTrajBufRArmSent[i+30] = ShortTrajBufRArm[i+60];
							  //}
							  //printf("%f   \n",SysTime);

							  //gArmTrajFleg = 1;

							  if(gFlagArmCtrl == 1)
							  {
								  WriteOK = gpPortLA->_write(gShortTrajBufLArm,120); // < 2ms
								  if (!WriteOK)
									  printf("Download Failed: Left Arm\n");
								  /*				  }
								  else if (gArmTrajFleg == 1)
								  {*/

								  //for(int i = 0;i<30;i++)
								  //{
								  // ShortTrajBufLArmSent[i+60] = ShortTrajBufLArm[i];
								  // ShortTrajBufLArmSent[i+90] = ShortTrajBufLArm[i+60];
								  // ShortTrajBufRArmSent[i+60] = ShortTrajBufRArm[i];
								  // ShortTrajBufRArmSent[i+90] = ShortTrajBufRArm[i+60];
								  //}
								  //gArmTrajFleg = 0;
								  WriteOK = gpPortRA->_write(gShortTrajBufRArm,120); // < 2ms
								  if (!WriteOK)
									  printf("Download Failed: Right Arm\n");
							  }

							  if(gFlagHandCtrl == 1)
							  {
								  if(gFlagHandStart==1)
								  {
									  DataCountStart++;

									  if( DataCountStart>=250)
									  {

										  WriteOK = gpPortLH->_write(gfContTrajDataLH+DataCountHand*60,60); // < 2ms
										  if (!WriteOK)
											  printf("Download Failed: Left Hand\n");
										  WriteOK = gpPortRH->_write(gfContTrajDataRH+DataCountHand*60,60); // < 2ms
										  if (!WriteOK)
											  printf("Download Failed: Right Arm\n");


										  DataCountHand++;
										  #if SoundCtrl
										  gSpeakSignLanguage(DataCountHand);
										  #endif
										  //cout<<DataCountHand<<endl;
										  //???
										  if (DataCountHand >=gfContTrajLenRH/2)
										  {
											  gFlagHandCtrl=0;
											  //cout << "End of Trajectory" << endl;
										  }
									  }

								  }
							  }
						  }

					  }

				  }
				  if (gFlagReadEncoder == 1)
				  {

					  //// 紀錄encoder data 現在先關掉手臂底層程式暫時不能用
					  //	gpPortLA->read(gBufIn,96,1000,&reason);

					  //for (int j = 0 ; j < 4 ; j++)
					  //{
					  //	for (int i=0 ; i < 6;i++)
					  //	{
					  //		if (buf_in[index_buf] > 128)
					  //		{
					  //			//read_enc[i][index_read] = -((255-buf_in[index_buf])*16777216+buf_in[index_buf+1]*65536+buf_in[index_buf+2]*256+buf_in[index_buf+3]);
					  //			ReadEncArm[i][index_read] = -(buf_in[index_buf+1]*65536+buf_in[index_buf+2]*256+buf_in[index_buf+3]);
					  //		}
					  //		else
					  //		{
					  //			//read_enc[i][index_read] = buf_in[index_buf]*16777216+buf_in[index_buf+1]*65536+buf_in[index_buf+2]*256+buf_in[index_buf+3];
					  //			ReadEncArm[i][index_read] = buf_in[index_buf+1]*65536+buf_in[index_buf+2]*256+buf_in[index_buf+3];
					  //		}
					  //		index_buf += 4;
					  //	}
					  //	index_read += 1;
					  //}

					  //index_read-= 4;
					  //index_buf = 0;

					  //	gpPortRA->read(gBufIn,96,1000,&reason);

					  //for (int j = 0 ; j < 4 ; j++)
					  //{
					  //	for (int i=6 ; i < 12;i++)
					  //	{
					  //		if (buf_in[index_buf] > 128)
					  //		{
					  //			//read_enc[i][index_read] = -((255-buf_in[index_buf])*16777216+buf_in[index_buf+1]*65536+buf_in[index_buf+2]*256+buf_in[index_buf+3]);
					  //			ReadEncArm[i][index_read] = -(buf_in[index_buf+1]*65536+buf_in[index_buf+2]*256+buf_in[index_buf+3]);
					  //		}
					  //		else
					  //		{
					  //			//read_enc[i][index_read] = buf_in[index_buf]*16777216+buf_in[index_buf+1]*65536+buf_in[index_buf+2]*256+buf_in[index_buf+3];
					  //			ReadEncArm[i][index_read] = buf_in[index_buf+1]*65536+buf_in[index_buf+2]*256+buf_in[index_buf+3];
					  //		}
					  //		index_buf += 4;
					  //	}
					  //	index_read += 1;
					  //}
					  //index_buf = 0;

				  }


				  if (Nothing_Sent)
				  {
					  if (gFlagBoostSimu == false)
						  Sleep(1); // saving CPU resource and fast check
					  else
						  Sleep(1);
				  }

			  }
			  else
			  {
				  Sleep(200); // save the CPU resource and idle
			  }

			  // 保護裝置，正常的話不會啟動
			  if(gSendContTrajArm == 1 && gContTrajLockArm == 1)
			  {
				  gSendContTrajArm = 0;
				  cout << "請將下拉式選單重新切換到 traj " << endl;
			  }
			  // 保護裝置，正常的話不會啟動

		  }
		  else
		  {
			  break;
		  }
	  }

  }

  void gIKStep(void)
  {
	/******************************************************************
	input: void
	output: void

	Note:
	//  每次機器人要解一格IK就會要呼叫這個函式一次
	//  此函示會自動抓取一格軌跡輸入 然後丟到IK solver之中進行IK
	// 機器人換腳時候的切換準備，所需要記下的東西也都寫在這個函式的最後面
	******************************************************************/

	  double StepProcess = 0.0; // 計算每步進行的百分比 從0~1
	  double AngL, AngR; // 左右腳在z軸的旋轉方向

	  double AngPitchL, AngPitchR;	  //20121214doratom//

	  StepProcess = (gIthIK % gStepSample)/double(gStepSample);

	  //if (gIthIK < gStepSample) // 內差補起來剛開始的gap
	  //{
			//gXcogIK[0] = gLQs.YState[gIthIK].data[0]+gKineAll.initCOG[0]*(gStepSample-gIthIK)/double(gStepSample);
			//gXcogIK[1] = gLQs.XState[gIthIK].data[0]+gKineAll.initCOG[1]*(gStepSample-gIthIK)/double(gStepSample);
			//gXcogIK[2] = gInpCOG[gIthIK];
	  //}
	  //else
	  //{
			//gXcogIK[0] = gLQs.YState[gIthIK].data[0];
			//gXcogIK[1] = gLQs.XState[gIthIK].data[0];
			//gXcogIK[2] = gInpCOG[gIthIK];
	  //}

	   // 設定目標COG
		gXcogIK[0] = gLQs.YState[gIthIK].data[0]+gKineAll.initCOG[0];
		gXcogIK[1] = gLQs.XState[gIthIK].data[0]+gKineAll.initCOG[1];
		gXcogIK[2] = gInpCOG[gIthIK];


		// 設定swing腳位置
		if (gKineAll.selIK == LeftSupport)
		{
			// 要記得寫 左腳右腳 記住換腳時的位置
			gSwingInputIK[0] = gKineAll.remRL[0]+gKineAll.SwingBufferx[gIthIK%gStepSample];
			gSwingInputIK[1] = gKineAll.remRL[1]+gKineAll.SwingBuffery[gIthIK%gStepSample];
			gSwingInputIK[2] = gKineAll.remRL[2]+gKineAll.SwingBufferz[gIthIK%gStepSample];				
		}
		else if (gKineAll.selIK == RightSupport)
		{
			gSwingInputIK[0] = gKineAll.remLL[0]+gKineAll.SwingBufferx[gIthIK%gStepSample];
			gSwingInputIK[1] = gKineAll.remLL[1]+gKineAll.SwingBuffery[gIthIK%gStepSample];
			gSwingInputIK[2] = gKineAll.remLL[2]+gKineAll.SwingBufferz[gIthIK%gStepSample];		
		}
		else if (gKineAll.selIK == DoubleSupport)
		{
			gSwingInputIK[0] = gKineAll.remLL[0];
			gSwingInputIK[1] = gKineAll.remLL[1];
			gSwingInputIK[2] = gKineAll.remLL[2];				
		}
		
		// 設定 swing fix 腳 角度  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		AngL = gAngLBuf[gIthIK%gStepSample];
		AngR = gAngRBuf[gIthIK%gStepSample];
		

		//20121214doratom//
		AngPitchL = gPitchAngLBuf[gIthIK%gStepSample];
		AngPitchR = gPitchAngRBuf[gIthIK%gStepSample];
		//20121214doratom//




		//cout<<"Ang = "<<(AngL+AngR)/2<<"  AngL = "<<AngL<<"  AngR = "<<AngR<<endl;
		//___________________________________________________________________________________________________________-
		//#if QCCDMode
		//gKineAll.QFKLLeg->TarRot[0]=cos(AngL);gKineAll.QFKLLeg->TarRot[1]=sin(AngL);gKineAll.QFKLLeg->TarRot[2]=0;
		//gKineAll.QFKLLeg->TarRot[3]=0;gKineAll.QFKLLeg->TarRot[4]=0;gKineAll.QFKLLeg->TarRot[5]=1;
		//gKineAll.QFKLLeg->TarRot[6]=sin(AngL);gKineAll.QFKLLeg->TarRot[7]=-cos(AngL);gKineAll.QFKLLeg->TarRot[8]=0;
		//#endif
		//#if QCCDMode
		//gKineAll.QFKRLeg->TarRot[0]=cos(AngR);gKineAll.QFKRLeg->TarRot[1]=sin(AngR);gKineAll.QFKRLeg->TarRot[2]=0;
		//gKineAll.QFKRLeg->TarRot[3]=0;gKineAll.QFKRLeg->TarRot[4]=0;gKineAll.QFKRLeg->TarRot[5]=1;
		//gKineAll.QFKRLeg->TarRot[6]=sin(AngR);gKineAll.QFKRLeg->TarRot[7]=-cos(AngR);gKineAll.QFKRLeg->TarRot[8]=0;
		//#endif
		//AngL = gKineAll.AnklePitchRef[gIthIK%gStepSample];
		//AngR = gKineAll.AnklePitchRef[gIthIK%gStepSample];		
		//___________________________________________________________________________________________________________

		if (gKineAll.selIK == LeftSupport)
		{
			gKineAll.TarRotMSw[0] = cos(AngR); gKineAll.TarRotMSw[1] = -sin(AngR); gKineAll.TarRotMSw[2] = 0;
			gKineAll.TarRotMSw[3] = sin(AngR); gKineAll.TarRotMSw[4] = cos(AngR); gKineAll.TarRotMSw[5] = 0;
			gKineAll.TarRotMSw[6] = 0; gKineAll.TarRotMSw[7] = 0; gKineAll.TarRotMSw[8] = 1;
			//gKineAll.TarRotMSw[0] = cos(AngR); gKineAll.TarRotMSw[1] =0 ; gKineAll.TarRotMSw[2] = sin(AngR);
			//gKineAll.TarRotMSw[3] = 0; gKineAll.TarRotMSw[4] = 1; gKineAll.TarRotMSw[5] = 0;
			//gKineAll.TarRotMSw[6] = -sin(AngR); gKineAll.TarRotMSw[7] = 0; gKineAll.TarRotMSw[8] = cos(AngR);

			//20121214doratom//
			if(check_slopeangle ==1)
			{    gKineAll.TarRotMSwPitch[0] = cos(AngPitchR); gKineAll.TarRotMSwPitch[1] = 0 ; gKineAll.TarRotMSwPitch[2] = sin(AngPitchR);
				 gKineAll.TarRotMSwPitch[3] = 0; gKineAll.TarRotMSwPitch[4] = 1; gKineAll.TarRotMSwPitch[5] = 0;
				 gKineAll.TarRotMSwPitch[6] = -sin(AngPitchR); gKineAll.TarRotMSwPitch[7] = 0; gKineAll.TarRotMSwPitch[8] = cos(AngPitchR);
			}

			gKineAll.TarRotMFx[0] = cos(AngL); gKineAll.TarRotMFx[1] = -sin(AngL); gKineAll.TarRotMFx[2] = 0;
			gKineAll.TarRotMFx[3] = sin(AngL); gKineAll.TarRotMFx[4] = cos(AngL); gKineAll.TarRotMFx[5] = 0;
			gKineAll.TarRotMFx[6] = 0; gKineAll.TarRotMFx[7] = 0; gKineAll.TarRotMFx[8] = 1;

			//20121214doratom//
			if(check_slopeangle ==1)
			{ 
				gKineAll.TarRotMFxPitch[0] = cos(AngPitchL); gKineAll.TarRotMFxPitch[1] = 0; gKineAll.TarRotMFxPitch[2] = sin(AngPitchL);
			    gKineAll.TarRotMFxPitch[3] = 0; gKineAll.TarRotMFxPitch[4] = 1; gKineAll.TarRotMFxPitch[5] = 0;
			    gKineAll.TarRotMFxPitch[6] = -sin(AngPitchL); gKineAll.TarRotMFxPitch[7] = 0; gKineAll.TarRotMFxPitch[8] = cos(AngPitchL);
			}

			#if QCCDMode
			//AngR=-AngR;
			//gKineAll.QFKRLeg->TarRot[0]=cos(AngR);gKineAll.QFKRLeg->TarRot[1]=0;gKineAll.QFKRLeg->TarRot[2]=sin(AngR);
			//gKineAll.QFKRLeg->TarRot[3]=-sin(AngR);gKineAll.QFKRLeg->TarRot[4]=0;gKineAll.QFKRLeg->TarRot[5]=cos(AngR);
			//gKineAll.QFKRLeg->TarRot[6]=0;gKineAll.QFKRLeg->TarRot[7]=-1;gKineAll.QFKRLeg->TarRot[8]=0;

			//gKineAll.QFKRLeg->TarRot[0]=cos(0.5*AngR-0.5*AngL);gKineAll.QFKRLeg->TarRot[1]=sin(0.5*AngR-0.5*AngL);gKineAll.QFKRLeg->TarRot[2]=0;
			//gKineAll.QFKRLeg->TarRot[3]=0;gKineAll.QFKRLeg->TarRot[4]=0;gKineAll.QFKRLeg->TarRot[5]=1;
			//gKineAll.QFKRLeg->TarRot[6]=sin(0.5*AngR-0.5*AngL);gKineAll.QFKRLeg->TarRot[7]=-cos(0.5*AngR-0.5*AngL);gKineAll.QFKRLeg->TarRot[8]=0;

			//gKineAll.QFKLLeg->TarRot[0]=cos(AngL);gKineAll.QFKLLeg->TarRot[1]=sin(AngL);gKineAll.QFKLLeg->TarRot[2]=0;
			//gKineAll.QFKLLeg->TarRot[3]=0;gKineAll.QFKLLeg->TarRot[4]=0;gKineAll.QFKLLeg->TarRot[5]=1;
			//gKineAll.QFKLLeg->TarRot[6]=sin(AngL);gKineAll.QFKLLeg->TarRot[7]=-cos(AngL);gKineAll.QFKLLeg->TarRot[8]=0;
			#endif
		}
		else if (gKineAll.selIK == RightSupport || gKineAll.selIK == DoubleSupport)
		{
			gKineAll.TarRotMSw[0] = cos(AngL); gKineAll.TarRotMSw[1] = -sin(AngL); gKineAll.TarRotMSw[2] = 0;
			gKineAll.TarRotMSw[3] = sin(AngL); gKineAll.TarRotMSw[4] = cos(AngL); gKineAll.TarRotMSw[5] = 0;
			gKineAll.TarRotMSw[6] = 0; gKineAll.TarRotMSw[7] = 0; gKineAll.TarRotMSw[8] = 1;

			gKineAll.TarRotMFx[0] = cos(AngR); gKineAll.TarRotMFx[1] = -sin(AngR); gKineAll.TarRotMFx[2] = 0;
			gKineAll.TarRotMFx[3] = sin(AngR); gKineAll.TarRotMFx[4] = cos(AngR); gKineAll.TarRotMFx[5] = 0;
			gKineAll.TarRotMFx[6] = 0; gKineAll.TarRotMFx[7] = 0; gKineAll.TarRotMFx[8] = 1;

			//20121214doratom//
			if(check_slopeangle ==1)
			{
				 gKineAll.TarRotMSwPitch[0] = cos(AngPitchL); gKineAll.TarRotMSwPitch[1] = 0 ; gKineAll.TarRotMSwPitch[2] = sin(AngPitchL);
				 gKineAll.TarRotMSwPitch[3] = 0; gKineAll.TarRotMSwPitch[4] = 1; gKineAll.TarRotMSwPitch[5] = 0;
				 gKineAll.TarRotMSwPitch[6] = -sin(AngPitchL); gKineAll.TarRotMSwPitch[7] = 0; gKineAll.TarRotMSwPitch[8] = cos(AngPitchL);

				 gKineAll.TarRotMFxPitch[0] = cos(AngPitchR); gKineAll.TarRotMFxPitch[1] = 0; gKineAll.TarRotMFxPitch[2] = sin(AngPitchR);
			     gKineAll.TarRotMFxPitch[3] = 0; gKineAll.TarRotMFxPitch[4] = 1; gKineAll.TarRotMFxPitch[5] = 0;
			     gKineAll.TarRotMFxPitch[6] = -sin(AngPitchR); gKineAll.TarRotMFxPitch[7] = 0; gKineAll.TarRotMFxPitch[8] = cos(AngPitchR);
			}

			#if QCCDMode
			//gKineAll.QFKLLeg->TarRot[0]=cos(AngL);gKineAll.QFKLLeg->TarRot[1]=0;gKineAll.QFKLLeg->TarRot[2]=-sin(AngL);
			//gKineAll.QFKLLeg->TarRot[3]=sin(AngL);gKineAll.QFKLLeg->TarRot[4]=0;gKineAll.QFKLLeg->TarRot[5]=cos(AngL);
			//gKineAll.QFKLLeg->TarRot[6]=0;gKineAll.QFKLLeg->TarRot[7]=-1;gKineAll.QFKLLeg->TarRot[8]=0;
			//gKineAll.QFKLLeg->TarRot[0]=cos(0.5*AngL-0.5*AngR);gKineAll.QFKLLeg->TarRot[1]=sin(0.5*AngL-0.5*AngR);gKineAll.QFKLLeg->TarRot[2]=0;
			//gKineAll.QFKLLeg->TarRot[3]=0;gKineAll.QFKLLeg->TarRot[4]=0;gKineAll.QFKLLeg->TarRot[5]=1;
			//gKineAll.QFKLLeg->TarRot[6]=sin(0.5*AngL-0.5*AngR);gKineAll.QFKLLeg->TarRot[7]=-cos(0.5*AngL-0.5*AngR);gKineAll.QFKLLeg->TarRot[8]=0;

			//gKineAll.QFKRLeg->TarRot[0]=cos(AngR);gKineAll.QFKRLeg->TarRot[1]=sin(AngR);gKineAll.QFKRLeg->TarRot[2]=0;
			//gKineAll.QFKRLeg->TarRot[3]=0;gKineAll.QFKRLeg->TarRot[4]=0;gKineAll.QFKRLeg->TarRot[5]=1;
			//gKineAll.QFKRLeg->TarRot[6]=sin(AngR);gKineAll.QFKRLeg->TarRot[7]=-cos(AngR);gKineAll.QFKRLeg->TarRot[8]=0;
			#endif
		}

		// by 泓逸 以下 ARM IK/////////////////////////////////////////////////////
		if(gIthIK == 0)
		{
			gDeltaLArmX = gKineAll.CrdAll->data[105] - gKineAll.COG[0];
			gDeltaLArmY = gKineAll.CrdAll->data[106] - gKineAll.COG[1];
			gDeltaLArmZ = gKineAll.CrdAll->data[107] - gKineAll.COG[2];
			gDeltaRArmX = gKineAll.CrdAll->data[135] - gKineAll.COG[0];
			gDeltaRArmY = gKineAll.CrdAll->data[136] - gKineAll.COG[1];
			gDeltaRArmZ = gKineAll.CrdAll->data[137] - gKineAll.COG[2];

			double DeltaLArmX = gKineAll.CrdAll->data[105] - gKineAll.COG[0];
			double DeltaLArmY = gKineAll.CrdAll->data[106] - gKineAll.COG[1];
			double DeltaLArmZ = gKineAll.CrdAll->data[107] - gKineAll.COG[2];
			double DeltaRArmX = gKineAll.CrdAll->data[135] - gKineAll.COG[0];
			double DeltaRArmY = gKineAll.CrdAll->data[136] - gKineAll.COG[1];
			double DeltaRArmZ = gKineAll.CrdAll->data[137] - gKineAll.COG[2];

			if(gDemoArmTrajFlag == 1)
			{
				gDeltaLArmX = DeltaLArmX*cos(-gAngLArm) - DeltaLArmY*sin(-gAngLArm);
				gDeltaLArmY = DeltaLArmX*sin(-gAngLArm) + DeltaLArmY*cos(-gAngLArm);
				gDeltaLArmZ = DeltaLArmZ;
				gDeltaRArmX = DeltaRArmX*cos(-gAngRArm) - DeltaRArmY*sin(-gAngRArm);
				gDeltaRArmY = DeltaRArmX*sin(-gAngRArm) + DeltaRArmY*cos(-gAngRArm);
				gDeltaRArmZ = DeltaRArmZ;
			}	
			
			gKineAll.GenSmoothZMPShift_ZeroJerk(0,gAngChange,gStepSample,gDth1);
			gKineAll.GenSmoothZMPShift_ZeroJerk(0,gAngChange*2,gStepSample,gDth2);

			//for(int i = 0;i<60000;i++)
			//{
			//	//走路和提東西用的
			//	gLArmWalkX[i] += (gLQs.YState[i].data[0]+gKineAll.initCOG[0]);
			//	gLArmWalkY[i] += (gLQs.XState[i].data[0]+gKineAll.initCOG[1]);
			//	gLArmWalkZ[i] += gInpCOG[i];
			//	gRArmWalkX[i] += (gLQs.YState[i].data[0]+gKineAll.initCOG[0]);
			//	gRArmWalkY[i] += (gLQs.XState[i].data[0]+gKineAll.initCOG[1]);
			//	gRArmWalkZ[i] += gInpCOG[i];

			//	////推推車時用的
			//	//gLArmWalkX[i] += gKineAll.initCOG[0];
			//	//gLArmWalkY[i] += (gLQs.XState[i].data[0]+gKineAll.initCOG[1]);
			//	//gLArmWalkZ[i] += gInpCOG[i];
			//	//gRArmWalkX[i] += gKineAll.initCOG[0];
			//	//gRArmWalkY[i] += (gLQs.XState[i].data[0]+gKineAll.initCOG[1]);
			//	//gRArmWalkZ[i] += gInpCOG[i];
			//}
		}

		////走路和提東西和推推車用的
		//gLArmInputIK[0] = gDeltaLArmX + gLArmWalkX[gIthIK];
		//gLArmInputIK[1] = gDeltaLArmY + gLArmWalkY[gIthIK];
		//gLArmInputIK[2] = gDeltaLArmZ + gLArmWalkZ[gIthIK];
		////tRArm		RArmInput_IK
		//gRArmInputIK[0] = gDeltaRArmX + gRArmWalkX[gIthIK];
		//gRArmInputIK[1] = gDeltaRArmY + gRArmWalkY[gIthIK];
		//gRArmInputIK[2] = gDeltaRArmZ + gRArmWalkZ[gIthIK];

		int TurnStep = (gIthIK - gIthIK%gStepSample)/gStepSample;

		// by 泓逸
		if(gDemoArmTrajFlag == 0)
		{
			double dxRArm = gRArmWalkX[gIthIK];
			double dyRArm = gRArmWalkY[gIthIK];
			double dxLArm = gLArmWalkX[gIthIK];
			double dyLArm = gLArmWalkY[gIthIK];
			//直走時使用
			gRArmInputIK[0] = -dyRArm*sin(gAngRArm) + dxRArm*cos(gAngRArm) + gLQs.YState[gIthIK].data[0] + gKineAll.initCOG[0] + gDeltaRArmX;
			gRArmInputIK[1] = dxRArm*sin(gAngRArm) + dyRArm*cos(gAngRArm) + gLQs.XState[gIthIK].data[0] + gKineAll.initCOG[1] + gDeltaRArmY;
			gRArmInputIK[2] = gDeltaRArmZ + gRArmWalkZ[gIthIK] + gInpCOG[gIthIK];

			gLArmInputIK[0] = -dyLArm*sin(gAngLArm) + dxLArm*cos(gAngLArm) + gLQs.YState[gIthIK].data[0] + gKineAll.initCOG[0] + gDeltaLArmX;
			gLArmInputIK[1] = dxLArm*sin(gAngLArm) + dyLArm*cos(gAngLArm) + gLQs.XState[gIthIK].data[0] + gKineAll.initCOG[1] + gDeltaLArmY;
			gLArmInputIK[2] = gDeltaLArmZ + gLArmWalkZ[gIthIK] + gInpCOG[gIthIK];
		}		
		else if(gDemoArmTrajFlag = 1)
		{
			double dxRArm = gDeltaRArmX + gRArmWalkX[gIthIK];
			double dyRArm = gDeltaRArmY + gRArmWalkY[gIthIK];
			double dxLArm = gDeltaLArmX + gLArmWalkX[gIthIK];
			double dyLArm = gDeltaLArmY + gLArmWalkY[gIthIK];
			//轉彎時使用
			if(gIthIK < gStepSample)
			{
				gRArmInputIK[0] = -dyRArm*sin(gAngRArm) + dxRArm*cos(gAngRArm) + gLQs.YState[gIthIK].data[0] + gKineAll.initCOG[0];
				gRArmInputIK[1] = dxRArm*sin(gAngRArm) + dyRArm*cos(gAngRArm) + gLQs.XState[gIthIK].data[0] + gKineAll.initCOG[1];
				gRArmInputIK[2] = gDeltaRArmZ + gRArmWalkZ[gIthIK] + gInpCOG[gIthIK];

				gLArmInputIK[0] = -dyLArm*sin(gAngLArm) + dxLArm*cos(gAngLArm) + gLQs.YState[gIthIK].data[0] + gKineAll.initCOG[0];
				gLArmInputIK[1] = dxLArm*sin(gAngLArm) + dyLArm*cos(gAngLArm) + gLQs.XState[gIthIK].data[0] + gKineAll.initCOG[1];
				gLArmInputIK[2] = gDeltaLArmZ + gLArmWalkZ[gIthIK] + gInpCOG[gIthIK];
			}
			else if(gIthIK >= gStepSample && gIthIK < gStepSample*2)
			{
				gAngRArm = gDth1[gIthIK-gStepSample] + gLastAngRArm;
				gRArmInputIK[0] = -dyRArm*sin(gAngRArm) + dxRArm*cos(gAngRArm) + gLQs.YState[gIthIK].data[0] + gKineAll.initCOG[0];
				gRArmInputIK[1] = dxRArm*sin(gAngRArm) + dyRArm*cos(gAngRArm) + gLQs.XState[gIthIK].data[0] + gKineAll.initCOG[1];
				gRArmInputIK[2] = gDeltaRArmZ + gRArmWalkZ[gIthIK] + gInpCOG[gIthIK];

				gLArmInputIK[0] = -dyLArm*sin(gAngLArm) + dxLArm*cos(gAngLArm) + gLQs.YState[gIthIK].data[0] + gKineAll.initCOG[0];
				gLArmInputIK[1] = dxLArm*sin(gAngLArm) + dyLArm*cos(gAngLArm) + gLQs.XState[gIthIK].data[0] + gKineAll.initCOG[1];
				gLArmInputIK[2] = gDeltaLArmZ + gLArmWalkZ[gIthIK] + gInpCOG[gIthIK];
			}
			else if(gIthIK >= gStepSample*2 && gIthIK < gStepSample*(gNumOfStep-4))
			{
				if(gTurnDirection == 0)
				{
					if(gKineAll.selSupport[TurnStep] == 0)//左腳support
					{
						gAngRArm = gAngChange * (TurnStep - 1) + gLastAngRArm;
						gAngLArm = gAngChange * (TurnStep - 2) + gDth2[gIthIK-gStepSample*TurnStep] + gLastAngLArm;
						gRArmInputIK[0] = -dyRArm*sin(gAngRArm) + dxRArm*cos(gAngRArm) + gLQs.YState[gIthIK].data[0] + gKineAll.initCOG[0];
						gRArmInputIK[1] = dxRArm*sin(gAngRArm) + dyRArm*cos(gAngRArm) + gLQs.XState[gIthIK].data[0] + gKineAll.initCOG[1];
						gRArmInputIK[2] = gDeltaRArmZ + gRArmWalkZ[gIthIK] + gInpCOG[gIthIK];

						gLArmInputIK[0] = -dyLArm*sin(gAngLArm) + dxLArm*cos(gAngLArm) + gLQs.YState[gIthIK].data[0] + gKineAll.initCOG[0];
						gLArmInputIK[1] = dxLArm*sin(gAngLArm) + dyLArm*cos(gAngLArm) + gLQs.XState[gIthIK].data[0] + gKineAll.initCOG[1];
						gLArmInputIK[2] = gDeltaLArmZ + gLArmWalkZ[gIthIK] + gInpCOG[gIthIK];

					}
					else//右腳support
					{
						gAngRArm = gAngChange * (TurnStep - 2) + gDth2[gIthIK-gStepSample*TurnStep] + gLastAngRArm;
						gAngLArm = gAngChange * (TurnStep - 1) + gLastAngLArm;
						gRArmInputIK[0] = -dyRArm*sin(gAngRArm) + dxRArm*cos(gAngRArm) + gLQs.YState[gIthIK].data[0] + gKineAll.initCOG[0];
						gRArmInputIK[1] = dxRArm*sin(gAngRArm) + dyRArm*cos(gAngRArm) + gLQs.XState[gIthIK].data[0] + gKineAll.initCOG[1];
						gRArmInputIK[2] = gDeltaRArmZ + gRArmWalkZ[gIthIK] + gInpCOG[gIthIK];

						gLArmInputIK[0] = -dyLArm*sin(gAngLArm) + dxLArm*cos(gAngLArm) + gLQs.YState[gIthIK].data[0] + gKineAll.initCOG[0];
						gLArmInputIK[1] = dxLArm*sin(gAngLArm) + dyLArm*cos(gAngLArm) + gLQs.XState[gIthIK].data[0] + gKineAll.initCOG[1];
						gLArmInputIK[2] = gDeltaLArmZ + gLArmWalkZ[gIthIK] + gInpCOG[gIthIK];
					}
				}
				else
				{
					if(gKineAll.selSupport[TurnStep] == 1)//左腳support
					{
						gAngRArm = gAngChange * (TurnStep - 1) + gLastAngRArm;
						gAngLArm = gAngChange * (TurnStep - 2) + gDth2[gIthIK-gStepSample*TurnStep] + gLastAngLArm;
						gRArmInputIK[0] = -dyRArm*sin(gAngRArm) + dxRArm*cos(gAngRArm) + gLQs.YState[gIthIK].data[0] + gKineAll.initCOG[0];
						gRArmInputIK[1] = dxRArm*sin(gAngRArm) + dyRArm*cos(gAngRArm) + gLQs.XState[gIthIK].data[0] + gKineAll.initCOG[1];
						gRArmInputIK[2] = gDeltaRArmZ + gRArmWalkZ[gIthIK] + gInpCOG[gIthIK];

						gLArmInputIK[0] = -dyLArm*sin(gAngLArm) + dxLArm*cos(gAngLArm) + gLQs.YState[gIthIK].data[0] + gKineAll.initCOG[0];
						gLArmInputIK[1] = dxLArm*sin(gAngLArm) + dyLArm*cos(gAngLArm) + gLQs.XState[gIthIK].data[0] + gKineAll.initCOG[1];
						gLArmInputIK[2] = gDeltaLArmZ + gLArmWalkZ[gIthIK] + gInpCOG[gIthIK];

					}
					else//右腳support
					{
						gAngRArm = gAngChange * (TurnStep - 2) + gDth2[gIthIK-gStepSample*TurnStep] + gLastAngRArm;
						gAngLArm = gAngChange * (TurnStep - 1) + gLastAngLArm;
						gRArmInputIK[0] = -dyRArm*sin(gAngRArm) + dxRArm*cos(gAngRArm) + gLQs.YState[gIthIK].data[0] + gKineAll.initCOG[0];
						gRArmInputIK[1] = dxRArm*sin(gAngRArm) + dyRArm*cos(gAngRArm) + gLQs.XState[gIthIK].data[0] + gKineAll.initCOG[1];
						gRArmInputIK[2] = gDeltaRArmZ + gRArmWalkZ[gIthIK] + gInpCOG[gIthIK];

						gLArmInputIK[0] = -dyLArm*sin(gAngLArm) + dxLArm*cos(gAngLArm) + gLQs.YState[gIthIK].data[0] + gKineAll.initCOG[0];
						gLArmInputIK[1] = dxLArm*sin(gAngLArm) + dyLArm*cos(gAngLArm) + gLQs.XState[gIthIK].data[0] + gKineAll.initCOG[1];
						gLArmInputIK[2] = gDeltaLArmZ + gLArmWalkZ[gIthIK] + gInpCOG[gIthIK];
					}				
				}
			}
			else
			{
				gRArmInputIK[0] = -dyRArm*sin(gAngRArm) + dxRArm*cos(gAngRArm) + gLQs.YState[gIthIK].data[0] + gKineAll.initCOG[0];
				gRArmInputIK[1] = dxRArm*sin(gAngRArm) + dyRArm*cos(gAngRArm) + gLQs.XState[gIthIK].data[0] + gKineAll.initCOG[1];
				gRArmInputIK[2] = gDeltaRArmZ + gRArmWalkZ[gIthIK] + gInpCOG[gIthIK];

				gLArmInputIK[0] = -dyLArm*sin(gAngLArm) + dxLArm*cos(gAngLArm) + gLQs.YState[gIthIK].data[0] + gKineAll.initCOG[0];
				gLArmInputIK[1] = dxLArm*sin(gAngLArm) + dyLArm*cos(gAngLArm) + gLQs.XState[gIthIK].data[0] + gKineAll.initCOG[1];
				gLArmInputIK[2] = gDeltaLArmZ + gLArmWalkZ[gIthIK] + gInpCOG[gIthIK];
			}
		}

		for(int i = 0;i<9;i ++)
		{
			gKineAll.TarRotMRA[i] = gRArmWalkRot[i+gIthIK*9];
			gKineAll.TarRotMLA[i] = gLArmWalkRot[i+gIthIK*9];
		}

		gKineAll.TarRotMRA[0] = gRArmWalkRot[0+gIthIK*9]*cos(AngL) - gRArmWalkRot[3+gIthIK*9]*sin(AngL);
		gKineAll.TarRotMRA[1] = gRArmWalkRot[1+gIthIK*9]*cos(AngL) - gRArmWalkRot[4+gIthIK*9]*sin(AngL);
		gKineAll.TarRotMRA[2] = gRArmWalkRot[2+gIthIK*9]*cos(AngL) - gRArmWalkRot[5+gIthIK*9]*sin(AngL);
		gKineAll.TarRotMRA[3] = gRArmWalkRot[0+gIthIK*9]*sin(AngL) + gRArmWalkRot[3+gIthIK*9]*cos(AngL);
		gKineAll.TarRotMRA[4] = gRArmWalkRot[1+gIthIK*9]*sin(AngL) + gRArmWalkRot[4+gIthIK*9]*cos(AngL);
		gKineAll.TarRotMRA[5] = gRArmWalkRot[2+gIthIK*9]*sin(AngL) + gRArmWalkRot[5+gIthIK*9]*cos(AngL);
		gKineAll.TarRotMRA[6] = gRArmWalkRot[6+gIthIK*9];
		gKineAll.TarRotMRA[7] = gRArmWalkRot[7+gIthIK*9];
		gKineAll.TarRotMRA[8] = gRArmWalkRot[8+gIthIK*9];

		gKineAll.TarRotMLA[0] = gLArmWalkRot[0+gIthIK*9]*cos(AngR) - gLArmWalkRot[3+gIthIK*9]*sin(AngR);
		gKineAll.TarRotMLA[1] = gLArmWalkRot[1+gIthIK*9]*cos(AngR) - gLArmWalkRot[4+gIthIK*9]*sin(AngR);
		gKineAll.TarRotMLA[2] = gLArmWalkRot[2+gIthIK*9]*cos(AngR) - gLArmWalkRot[5+gIthIK*9]*sin(AngR);
		gKineAll.TarRotMLA[3] = gLArmWalkRot[0+gIthIK*9]*sin(AngR) + gLArmWalkRot[3+gIthIK*9]*cos(AngR);
		gKineAll.TarRotMLA[4] = gLArmWalkRot[1+gIthIK*9]*sin(AngR) + gLArmWalkRot[4+gIthIK*9]*cos(AngR);
		gKineAll.TarRotMLA[5] = gLArmWalkRot[2+gIthIK*9]*sin(AngR) + gLArmWalkRot[5+gIthIK*9]*cos(AngR);
		gKineAll.TarRotMLA[6] = gLArmWalkRot[6+gIthIK*9];
		gKineAll.TarRotMLA[7] = gLArmWalkRot[7+gIthIK*9];
		gKineAll.TarRotMLA[8] = gLArmWalkRot[8+gIthIK*9];

		////轉彎時的手臂揮動用的
		//gKineAll.TarRotMRA[0] = gKineAll.TarRotMRA[0]*cos(AngL) - gKineAll.TarRotMRA[3]*sin(AngL);
		//gKineAll.TarRotMRA[1] = gKineAll.TarRotMRA[1]*cos(AngL) - gKineAll.TarRotMRA[4]*sin(AngL);
		//gKineAll.TarRotMRA[2] = gKineAll.TarRotMRA[2]*cos(AngL) - gKineAll.TarRotMRA[5]*sin(AngL);
		//gKineAll.TarRotMRA[3] = gKineAll.TarRotMRA[0]*sin(AngL) + gKineAll.TarRotMRA[3]*cos(AngL);
		//gKineAll.TarRotMRA[4] = gKineAll.TarRotMRA[1]*sin(AngL) + gKineAll.TarRotMRA[4]*cos(AngL);
		//gKineAll.TarRotMRA[5] = gKineAll.TarRotMRA[2]*sin(AngL) + gKineAll.TarRotMRA[5]*cos(AngL);
		//gKineAll.TarRotMRA[6] = gKineAll.TarRotMRA[6];
		//gKineAll.TarRotMRA[7] = gKineAll.TarRotMRA[7];
		//gKineAll.TarRotMRA[8] = gKineAll.TarRotMRA[8];

		//gKineAll.TarRotMLA[0] = gKineAll.TarRotMLA[0]*cos(AngR) - gKineAll.TarRotMLA[3]*sin(AngR);
		//gKineAll.TarRotMLA[1] = gKineAll.TarRotMLA[1]*cos(AngR) - gKineAll.TarRotMLA[4]*sin(AngR);
		//gKineAll.TarRotMLA[2] = gKineAll.TarRotMLA[2]*cos(AngR) - gKineAll.TarRotMLA[5]*sin(AngR);
		//gKineAll.TarRotMLA[3] = gKineAll.TarRotMLA[0]*sin(AngR) + gKineAll.TarRotMLA[3]*cos(AngR);
		//gKineAll.TarRotMLA[4] = gKineAll.TarRotMLA[1]*sin(AngR) + gKineAll.TarRotMLA[4]*cos(AngR);
		//gKineAll.TarRotMLA[5] = gKineAll.TarRotMLA[2]*sin(AngR) + gKineAll.TarRotMLA[5]*cos(AngR);
		//gKineAll.TarRotMLA[6] = gKineAll.TarRotMLA[6];
		//gKineAll.TarRotMLA[7] = gKineAll.TarRotMLA[7];
		//gKineAll.TarRotMLA[8] = gKineAll.TarRotMLA[8];
		
		//gKineAll.TarRotMRA[0] = 0;
		//gKineAll.TarRotMRA[1] = cos(AngL);
		//gKineAll.TarRotMRA[2] = sin(AngL);
		//gKineAll.TarRotMRA[3] = 0;
		//gKineAll.TarRotMRA[4] = sin(AngL);
		//gKineAll.TarRotMRA[5] = -cos(AngL);
		//gKineAll.TarRotMRA[6] = -1;
		//gKineAll.TarRotMRA[7] = 0;
		//gKineAll.TarRotMRA[8] = 0;

		//gKineAll.TarRotMLA[0] = 0;
		//gKineAll.TarRotMLA[1] = cos(AngR);
		//gKineAll.TarRotMLA[2] = sin(AngR);
		//gKineAll.TarRotMLA[3] = 0;
		//gKineAll.TarRotMLA[4] = sin(AngR);
		//gKineAll.TarRotMLA[5] = -cos(AngR);
		//gKineAll.TarRotMLA[6] = -1;
		//gKineAll.TarRotMLA[7] = 0;
		//gKineAll.TarRotMLA[8] = 0;

		//printf("gIthIK = %d\n",gIthIK);
		
		if(gIKMethod == 1)
		{
			if (ArmOfflinMethod)
			for(int i = 0;i<6;i++)
			{
			/////!!! Flag
				gKineAll.FKLArm->theta[i+4] = gLArmOfflineTraj[gIthIK*6 + i];
				gKineAll.FKRArm->theta[i+4] = gRArmOfflineTraj[gIthIK*6 + i];
			}
			else if(AfterSignLanguageFlag == 0)
			{
				for(int i = 0;i<6;i++)
				{
					gKineAll.FKLArm->theta[i+4] += gLArmOfflineTraj[gIthIK*6 + i];
					gKineAll.FKRArm->theta[i+4] += gRArmOfflineTraj[gIthIK*6 + i];
				}
			}
			else if(AfterSignLanguageFlag == 1)
			{
				for(int i = 0;i<6;i++)
				{
					gKineAll.FKLArm->theta[i+4] += 0;
					gKineAll.FKRArm->theta[i+4] += 0;
				}
			}
			gKineAll.FindFK();
			gKineAll.FindCOG();
		}
		// by 泓逸 以上 ARM IK/////////////////////////////////////////////////////

		//printf("gIthIK = %d\n",gIthIK);
			double QQ=gKineAll.FKLLeg->theta[4]*180/3.1415926;
			double OT=gKineAll.FKRLeg->theta[4]*180/3.1415926;

			if(QQ>MinLKnee)
				MinLKnee=QQ;
			//cout<<"min L Knee Angle = "<<minL<<endl;

			if(OT<MinRKnee)
				MinRKnee=OT;
		#if QCCDMode
		/*_______________________________________QCCD with COG Jacobian (Slongz)_______________________________________
			IK with CCD method

			Note:
			  Test 希望可以完成!

		_______________________________________QCCD with COG Jacobian (Slongz)_______________________________________*/
		/// solve IK
			double QQ=gKineAll.FKLLeg->theta[4]*180/3.1415926;
			double OT=gKineAll.FKRLeg->theta[4]*180/3.1415926;

			if(QQ<MinLKnee)
				MinLKnee=QQ;
			//cout<<"min L Knee Angle = "<<minL<<endl;

			if(OT>MinRKnee)
				MinRKnee=OT;
			//cout<<"min L Knee Angle = "<<minL<<"   min R Knee Angle = "<<minR<<endl;

			if(QQ<QCCDSwAngle)
			LFlag=0;
			else if(gKineAll.selIK==1||gKineAll.selIK==2)
			LFlag=1;

			if(abs(OT)<QCCDSwAngle)
			RFlag=0;
			else if(gKineAll.selIK==0)
			RFlag=1;

		
			if(LFlag && RFlag)
			{
				//20121214doratom//
				if(check_slopeangle==1)
                gKineAll.IKSolve(gXcogIK,gSwingInputIK,gKineAll.TarRotMSwPitch,gKineAll.TarRotMFxPitch,gLArmInputIK , gRArmInputIK , gKineAll.TarRotMLA,gKineAll.TarRotMRA,gIKMethod, &gIKStatus);
				else
				gKineAll.IKSolve(gXcogIK,gSwingInputIK,gKineAll.TarRotMSw,gKineAll.TarRotMFx,gLArmInputIK , gRArmInputIK , gKineAll.TarRotMLA,gKineAll.TarRotMRA,gIKMethod, &gIKStatus);
				//cout<<" "<<gSwingInputIK[0]<<" "<<gSwingInputIK[1]<<" "<<gSwingInputIK[2]<<endl;
				//cout<<"CC"<<endl;
			}
			else
			{
				#if SupportCCD
				//20121214doratom//
				if(check_slopeangle==1)
                gKineAll.IKCCDSolve(gXcogIK,gSwingInputIK,gKineAll.TarRotMSwPitch,gKineAll.TarRotMFxPitch,&gIKStatus);
				else
				gKineAll.IKCCDSolve(gXcogIK,gSwingInputIK,gKineAll.TarRotMSw,gKineAll.TarRotMFx, &gIKStatus);
				cout<<"CC"<<endl;
				#else
				//gLQs.tic();
				//20121214doratom//
				if(check_slopeangle==1)
                gKineAll.IKFixSolve(gXcogIK,gSwingInputIK,gKineAll.TarRotMSwPitch,gKineAll.TarRotMFxPitch,&gIKStatus);
				else
				gKineAll.IKFixSolve(gXcogIK,gSwingInputIK,gKineAll.TarRotMSw,gKineAll.TarRotMFx, &gIKStatus);
				//gLQs.toc();
				#endif
				//gLQs.tic();
				//gKineAll.IKCCDSolve(gXcogIK,gSwingInputIK,gKineAll.TarRotMSw,gKineAll.TarRotMFx, &gIKStatus);
				//cout<<" "<<gSwingInputIK[0]<<" "<<gSwingInputIK[1]<<" "<<gSwingInputIK[2]<<endl;
				//gLQs.toc();
			}

			//	cout<<QQ<<endl;
		#else
		// 依照輸入解IK 直到收斂
		//20121214doratom//
		if(check_slopeangle==1)
        gKineAll.IKSolve(gXcogIK, gSwingInputIK, gKineAll.TarRotMSwPitch, gKineAll.TarRotMFxPitch, gLArmInputIK, gRArmInputIK, gKineAll.TarRotMLA,gKineAll.TarRotMRA, gIKMethod, &gIKStatus);
		else
		gKineAll.IKSolve(gXcogIK, gSwingInputIK, gKineAll.TarRotMSw, gKineAll.TarRotMFx, gLArmInputIK, gRArmInputIK, gKineAll.TarRotMLA, gKineAll.TarRotMRA, gIKMethod, &gIKStatus);
		//if(gIKStatus == 0)
		//	printf("%d",gIthIK);
		#endif 

		// 解完一步IK 準備生下一步的軌跡
		if ((gIthIK+1) % gStepSample == 0)
		{
			gKineAll.stepIndex += 1;
			gKineAll.selIK = gKineAll.selSupport[gKineAll.stepIndex];
			// selIK = 0 : left foot is the supporter
			// selIK = 1 : right foot is the supporter
			if (gKineAll.FlagSumoMode == 1)   //FlagSumoMode == 1 長時間站立
			{
				// 展示單腳站立 這是寫死的 gKineAll.FlagSumoMode = 0 是一般模式 可接受各種軌跡
				if (gKineAll.stepIndex == 1) // 1~3 共三個step 腳抬起來
				{
					// gLAngZWorld 因為預想兩隻腳要平行 所以只用取一隻腳的角度
					gStrideX = -25*sin(gLAngZWorld);
					gStrideY = 25*cos(gLAngZWorld);
					gStrideZ = 45;
				}
				else if (gKineAll.stepIndex == 2) // 1~3 共三個step 腳再抬高
				{
					gStrideX = -45*sin(gLAngZWorld);
					gStrideY = 45*cos(gLAngZWorld);
					gStrideZ = 45;
				}
				else if (gKineAll.stepIndex == 3) // 1~3 共三個step 放下腳
				{
					gStrideX = 70*sin(gLAngZWorld);
					gStrideY = -70*cos(gLAngZWorld);
					gStrideZ = -90;
					//system("pause");
				}
				else if (gKineAll.stepIndex == 4) // 換重心 準備換腳
				{
					gStrideX = 25*sin(gLAngZWorld);
					gStrideY = -25*cos(gLAngZWorld);
					gStrideZ = 45;
					//system("pause");
				}
				else if (gKineAll.stepIndex == 5) // 5~7 共三個step 腳抬起來
				{
					gStrideX = 45*sin(gLAngZWorld);
					gStrideY = -45*cos(gLAngZWorld);
					gStrideZ = 45;
				}
				else if (gKineAll.stepIndex == 6) // 5~7 共三個step 腳再抬高
				{
					gStrideX = -70*sin(gLAngZWorld);
					gStrideY = 70*cos(gLAngZWorld);
					gStrideZ = -90;
				}
				else if (gKineAll.stepIndex == 7) // 5~7 共三個step 放下腳
				{
					gStrideX = 0;
					gStrideY = 0;
					gStrideZ = 0;
				}
				else if (gKineAll.stepIndex == 8) // 收勢
				{

				}

				 //平滑連接
				for (int pz = 0 ; pz < 10; pz++)
				{
					gKineAll.SwingBufferx[pz] = 0;
					gKineAll.SwingBuffery[pz] = 0;
					gKineAll.SwingBufferz[pz] = 0;
				}
				// 從開始第10格就動
				gKineAll.GenSmoothZMPShift_ZeroJerk(0,gStrideX,gNab+gNza-10,gKineAll.SwingBufferx+10);
				gKineAll.GenSmoothZMPShift_ZeroJerk(0,gStrideY,gNab+gNza-10,gKineAll.SwingBuffery+10);
				gKineAll.GenSmoothZMPShift_ZeroJerk(0,gStrideZ,gNab+gNza-10,gKineAll.SwingBufferz+10);
				for (int pz = gNab+gNza ; pz < gStepSample; pz++)
				{
					gKineAll.SwingBufferx[pz] = gStrideX;
					gKineAll.SwingBuffery[pz] = gStrideY;
					gKineAll.SwingBufferz[pz] = gStrideZ;
				}

			}
			else  //不需要長時間站立 一般模式
			{
				if (gKineAll.stepIndex == 1 || gKineAll.stepIndex == 2) // 第一步或第二步
				{
					if (gKineAll.selIK == RightSupport)
					{
						// 注意!! X 與 Y 相反!! 因為定義不同!!
						// 在 前進方向 是 X 左右方向是 Y
						gStrideY = gFstpX[gKineAll.stepIndex+1]-gLLInitZMPLR;   //因為是右腳support 所以要移動的是左腳 故要減的是左腳原始的ZMP
						gStrideX = gFstpY[gKineAll.stepIndex+1]-gLLInitZMPFB;
						gStrideZ = gGroundHeight[gKineAll.stepIndex+1]-gGroundHeight[gKineAll.stepIndex-1];
					}
					else if (gKineAll.selIK == LeftSupport)
					{
						gStrideY = gFstpX[gKineAll.stepIndex+1]-gRLInitZMPLR;
						gStrideX = gFstpY[gKineAll.stepIndex+1]-gRLInitZMPFB;
						gStrideZ = gGroundHeight[gKineAll.stepIndex+1]-gGroundHeight[gKineAll.stepIndex-1];
					}
					else if (gKineAll.selIK == DoubleSupport)
					{
						gStrideY = 0.0;
						gStrideX = 0.0;
						gStrideZ = 0.0;
					}
				}
				else   //非第一步或第二步
				{
					if (gKineAll.selSupport[gKineAll.stepIndex+1] == 2 && gKineAll.selSupport[gKineAll.stepIndex] != 2) // goning to stop
					{
						if (gKineAll.selIK == RightSupport) // right support
						{
							gStrideY = gFstpX[gKineAll.stepIndex] - gFstpX[gKineAll.stepIndex-1] + 160.0*sin(AngR+3.1415926/2.0);  
							gStrideX = gFstpY[gKineAll.stepIndex] - gFstpY[gKineAll.stepIndex-1] + 160.0*cos(AngR+3.1415926/2.0);	
							gStrideZ = gGroundHeight[gKineAll.stepIndex]-gGroundHeight[gKineAll.stepIndex-1];					
						}
						else if (gKineAll.selIK == LeftSupport)
						{
							gStrideY = gFstpX[gKineAll.stepIndex] - gFstpX[gKineAll.stepIndex-1] - 160.0*sin(AngL+3.1415926/2.0);  
							gStrideX = gFstpY[gKineAll.stepIndex] - gFstpY[gKineAll.stepIndex-1] - 160.0*cos(AngL+3.1415926/2.0);
							gStrideZ = gGroundHeight[gKineAll.stepIndex]-gGroundHeight[gKineAll.stepIndex-1];	
						}
					}
					else
					{
						if (gKineAll.selIK == LeftSupport || gKineAll.selIK == RightSupport)
						{
							// 注意!! X 與 Y 相反!! 因為定義不同!!
							// 在 前進方向 是 X 左右方向是 Y
							gStrideY = gFstpX[gKineAll.stepIndex+1] - gFstpX[gKineAll.stepIndex-1];
							gStrideX = gFstpY[gKineAll.stepIndex+1] - gFstpY[gKineAll.stepIndex-1];	
							gStrideZ = gGroundHeight[gKineAll.stepIndex+1]-gGroundHeight[gKineAll.stepIndex-1];
						}
						else if (gKineAll.selIK == DoubleSupport)
						{
							gStrideY = 0.0;
							gStrideX = 0.0;	
							gStrideZ = 0.0;
						}
					}
				}


				if (gKineAll.stepIndex == 0)  // 基本上不會進這個if 因為一開始就會是1
				{
					for (int i =0 ; i< gKineAll.N_step ; i++)
					{
						gKineAll.SwingBufferx[i] = 0;
						gKineAll.SwingBuffery[i] = 0;
						gKineAll.SwingBufferz[i] = 0;
					}
				}
				else
				{
					if (gStrideZ <= -15 ) //  -15mm~ -80 down stairs
					{
						gKineAll.GenSwingTrajMod(0,0,0.3,gKineAll.StepHeight[gKineAll.stepIndex],0,gStrideX,gStrideZ,0,0,gKineAll.Nab,0, gKineAll.SwingBufferx, gKineAll.SwingBufferz);
						gKineAll.GenSwingTrajMod(0,0,0.3,gKineAll.StepHeight[gKineAll.stepIndex],0,gStrideY,gStrideZ,0,0,gKineAll.Nab,0, gKineAll.SwingBuffery, gKineAll.SwingBufferz);
					}
					//else if (gStrideZ <= -80) // < -80mm down stairs
					//{
					//	gKineAll.GenSwingTrajMod(0,0,0.27,gKineAll.StepHeight[gKineAll.stepIndex],0,gStrideX,gStrideZ,0,0,gKineAll.Nab,0, gKineAll.SwingBufferx, gKineAll.SwingBufferz);
					//	gKineAll.GenSwingTrajMod(0,0,0.4,gKineAll.StepHeight[gKineAll.stepIndex],0,gStrideY,gStrideZ,0,0,gKineAll.Nab,0, gKineAll.SwingBuffery, gKineAll.SwingBufferz);
				
					//
					//}
					else if (gStrideZ >= 15) // > 15mm 踏高1.5cm以上 upstairs
					{
						gKineAll.GenSwingTrajMod(0,0,0.2,gKineAll.StepHeight[gKineAll.stepIndex]+gStrideZ,0,gStrideX,gStrideZ,0,0,gKineAll.Nab,0, gKineAll.SwingBufferx, gKineAll.SwingBufferz);
						gKineAll.GenSwingTrajMod(0,0,0.2,gKineAll.StepHeight[gKineAll.stepIndex]+gStrideZ,0,gStrideY,gStrideZ,0,0,gKineAll.Nab,0, gKineAll.SwingBuffery, gKineAll.SwingBufferz);
				
					}
					else // -15~15 平地附近
					{
						if(gKineAll.stepIndex<11)
						{
							#if FootpadRotation
								gKineAll.GenSwingTrajMod2(0,0,0.5,gKineAll.StepHeight[gKineAll.stepIndex]+gStrideZ,0,gStrideX,gStrideZ,0,0,gKineAll.Nab,0, gKineAll.SwingBufferx, gKineAll.SwingBufferz);
								gKineAll.GenSwingTrajMod2(0,0,0.5,gKineAll.StepHeight[gKineAll.stepIndex]+gStrideZ,0,gStrideY,gStrideZ,0,0,gKineAll.Nab,0, gKineAll.SwingBuffery, gKineAll.SwingBufferz);
							#else
								gKineAll.GenSwingTrajMod(0,0,0.5,gKineAll.StepHeight[gKineAll.stepIndex]+gStrideZ,0,gStrideX,gStrideZ,0,0,gKineAll.Nab,0, gKineAll.SwingBufferx, gKineAll.SwingBufferz);
								gKineAll.GenSwingTrajMod(0,0,0.5,gKineAll.StepHeight[gKineAll.stepIndex]+gStrideZ,0,gStrideY,gStrideZ,0,0,gKineAll.Nab,0, gKineAll.SwingBuffery, gKineAll.SwingBufferz);
							#endif
						}
						else
						{
							#if FootpadRotation
								gKineAll.GenSwingTrajMod2(0,0,0.5,gKineAll.StepHeight[gKineAll.stepIndex]+gStrideZ,0,gStrideX,gStrideZ,0,0,gKineAll.Nab,0, gKineAll.SwingBufferx, gKineAll.SwingBufferz);
								gKineAll.GenSwingTrajMod2(0,0,0.5,gKineAll.StepHeight[gKineAll.stepIndex]+gStrideZ,0,gStrideY,gStrideZ,0,0,gKineAll.Nab,0, gKineAll.SwingBuffery, gKineAll.SwingBufferz);
							#else
								gKineAll.GenSwingTrajMod(0,0,0.5,gKineAll.StepHeight[gKineAll.stepIndex]+gStrideZ,0,gStrideX,gStrideZ,0,0,gKineAll.Nab,0, gKineAll.SwingBufferx, gKineAll.SwingBufferz);
								gKineAll.GenSwingTrajMod(0,0,0.5,gKineAll.StepHeight[gKineAll.stepIndex]+gStrideZ,0,gStrideY,gStrideZ,0,0,gKineAll.Nab,0, gKineAll.SwingBuffery, gKineAll.SwingBufferz);
							#endif
						}
					}
				}
			}


			if (gKineAll.stepIndex == 0) // initial step (DSP) // 基本上不會進這個if 因為一開始就會是1
			{
				// do nothing
			}
			else
			{
				// 算出兩腳角度旋轉軌跡
				if (gKineAll.selSupport[gKineAll.stepIndex] == 2)
				{
					// don't rotate
					for (int pp = 0 ; pp < gStepSample ; pp++)
					{
						gAngLBuf[pp] = AngL;
						gAngRBuf[pp] = AngR;
					}
				}
				else
				{
					for (int pp = 0 ; pp < gNza ; pp++)
					{
						gAngLBuf[pp] = gLRotAngZ[gKineAll.stepIndex];
						gAngRBuf[pp] = gRRotAngZ[gKineAll.stepIndex];

						//20121214doratom//
						if(check_slopeangle==1)
						{
						gPitchAngLBuf[pp] = gLRotAngPitch[gKineAll.stepIndex];
						gPitchAngRBuf[pp] = gRRotAngPitch[gKineAll.stepIndex];
						}
					}

					if (gKineAll.selSupport[gKineAll.stepIndex+1] == 2 && gKineAll.selSupport[gKineAll.stepIndex] != 2) // goning to stop
					{
						if (gKineAll.selIK == RightSupport) // right support
						{
							//20121214doratom//
							if(check_slopeangle==1)
							{
							gKineAll.GenSmoothZMPShift_ZeroJerk(gLRotAngPitch[gKineAll.stepIndex],gRRotAngPitch[gKineAll.stepIndex],gNab-200,gPitchAngLBuf+gNza);
							gKineAll.GenSmoothZMPShift_ZeroJerk(gRRotAngPitch[gKineAll.stepIndex],gRRotAngPitch[gKineAll.stepIndex],gNab-200,gPitchAngRBuf+gNza);
							}
							else
							{
							gKineAll.GenSmoothZMPShift_ZeroJerk(gLRotAngZ[gKineAll.stepIndex],gRRotAngZ[gKineAll.stepIndex],gNab,gAngLBuf+gNza);
							gKineAll.GenSmoothZMPShift_ZeroJerk(gRRotAngZ[gKineAll.stepIndex],gRRotAngZ[gKineAll.stepIndex],gNab,gAngRBuf+gNza);
							}
							//20121214doratom//

							for (int pp = gNza+gNab ; pp < gStepSample ; pp++)
							{
								gAngLBuf[pp] = gRRotAngZ[gKineAll.stepIndex+1];
								gAngRBuf[pp] = gRRotAngZ[gKineAll.stepIndex+1];
							}		

							for (int pp = gNza+gNab-200 ; pp < gStepSample ; pp++)		//20121214doratom//
							{
								if(check_slopeangle==1)
								{
									gPitchAngLBuf[pp] = gRRotAngPitch[gKineAll.stepIndex+1];
									gPitchAngRBuf[pp] = gRRotAngPitch[gKineAll.stepIndex+1];
								}
							}
						}
						else if (gKineAll.selIK == LeftSupport) // left support
						{
							//20121214doratom//
							if(check_slopeangle==1)
							{
							gKineAll.GenSmoothZMPShift_ZeroJerk(gLRotAngPitch[gKineAll.stepIndex],gLRotAngPitch[gKineAll.stepIndex],gNab-200,gPitchAngLBuf+gNza);
							gKineAll.GenSmoothZMPShift_ZeroJerk(gRRotAngPitch[gKineAll.stepIndex],gLRotAngPitch[gKineAll.stepIndex],gNab-200,gPitchAngRBuf+gNza);
							}
							else
							{							
							gKineAll.GenSmoothZMPShift_ZeroJerk(gLRotAngZ[gKineAll.stepIndex],gLRotAngZ[gKineAll.stepIndex],gNab,gAngLBuf+gNza);
							gKineAll.GenSmoothZMPShift_ZeroJerk(gRRotAngZ[gKineAll.stepIndex],gLRotAngZ[gKineAll.stepIndex],gNab,gAngRBuf+gNza);
							}

							for (int pp = gNza+gNab ; pp < gStepSample ; pp++)
							{
								gAngLBuf[pp] = gLRotAngZ[gKineAll.stepIndex+1];
								gAngRBuf[pp] = gLRotAngZ[gKineAll.stepIndex+1];
							}
							
							for (int pp = gNza+gNab-200 ; pp < gStepSample ; pp++)//20121214doratom//
							{
								if(check_slopeangle==1)
								{
									gPitchAngLBuf[pp] = gLRotAngPitch[gKineAll.stepIndex+1];
									gPitchAngRBuf[pp] = gLRotAngPitch[gKineAll.stepIndex+1];
								}
							}//20121214doratom//
						}
					}
					else
					{
						//20121214doratom//
						if(check_slopeangle==1)
						{
							//gKineAll.GenSmoothZMPShift_ZeroJerk(gLRotAngPitch[gKineAll.stepIndex],gLRotAngPitch[gKineAll.stepIndex+1],int(gNab*rotate_pitch_time_ratio),gPitchAngLBuf+gNza);
							//gKineAll.GenSmoothZMPShift_ZeroJerk(gRRotAngPitch[gKineAll.stepIndex],gRRotAngPitch[gKineAll.stepIndex+1],int(gNab*rotate_pitch_time_ratio),gPitchAngRBuf+gNza);

							gKineAll.GenSmoothZMPShift_ZeroJerk(gLRotAngPitch[gKineAll.stepIndex],gLRotAngPitch[gKineAll.stepIndex+1],int(gNab*rotate_pitch_time_ratio),gPitchAngLBuf+gNza);
							gKineAll.GenSmoothZMPShift_ZeroJerk(gRRotAngPitch[gKineAll.stepIndex],gRRotAngPitch[gKineAll.stepIndex+1],int(gNab*rotate_pitch_time_ratio),gPitchAngRBuf+gNza);
						}
						else
						{	
							gKineAll.GenSmoothZMPShift_ZeroJerk(gLRotAngZ[gKineAll.stepIndex],gLRotAngZ[gKineAll.stepIndex+1],gNab,gAngLBuf+gNza);
							gKineAll.GenSmoothZMPShift_ZeroJerk(gRRotAngZ[gKineAll.stepIndex],gRRotAngZ[gKineAll.stepIndex+1],gNab,gAngRBuf+gNza);
						}

						if(check_slopeangle==1)
						{
							for (int pp = gNza+int(gNab*rotate_pitch_time_ratio) ; pp < gStepSample ; pp++)
							{
								gPitchAngLBuf[pp] = gLRotAngPitch[gKineAll.stepIndex+1];
								gPitchAngRBuf[pp] = gRRotAngPitch[gKineAll.stepIndex+1];
							}
						}
						else
						{
							for (int pp = gNza+gNab ; pp < gStepSample ; pp++)
							{
								gAngLBuf[pp] = gLRotAngZ[gKineAll.stepIndex+1];
								gAngRBuf[pp] = gRRotAngZ[gKineAll.stepIndex+1];
							}
						}
						//20121214doratom//		
					}
				}
			}

			// 判斷下一格是否要換腳 要的話就進入進行資料記憶以及之後切換腳的切換準備
			// 記下所有換腳時的資訊 很重要 不然機器人不知道接下來固定腳的位置與角度
			if (gKineAll.selIK == LeftSupport) // 
			{
				gKineAll.remLL[0] = gKineAll.CrdAll->data[18];
				gKineAll.remLL[1] = gKineAll.CrdAll->data[19];
				gKineAll.remLL[2] = gKineAll.CrdAll->data[20];
				gKineAll.shiftLL[0] = gKineAll.CrdAll->data[21];
				gKineAll.shiftLL[1] = gKineAll.CrdAll->data[22];
				gKineAll.shiftLL[2] = gKineAll.CrdAll->data[23];
				
				gKineAll.remRL[0] = gKineAll.CrdAll->data[57];
				gKineAll.remRL[1] = gKineAll.CrdAll->data[58];
				gKineAll.remRL[2] = gKineAll.CrdAll->data[59];
				gKineAll.shiftRL[0] = gKineAll.CrdAll->data[60];
				gKineAll.shiftRL[1] = gKineAll.CrdAll->data[61];
				gKineAll.shiftRL[2] = gKineAll.CrdAll->data[62];


				gKineAll.GetLegsCoords();
				gKineAll.LSwitchRMot[0] = gKineAll.LLegRotM[0];
				gKineAll.LSwitchRMot[1] = gKineAll.LLegRotM[1];
				gKineAll.LSwitchRMot[2] = gKineAll.LLegRotM[2];
				gKineAll.LSwitchRMot[4] = gKineAll.LLegRotM[3];
				gKineAll.LSwitchRMot[5] = gKineAll.LLegRotM[4];
				gKineAll.LSwitchRMot[6] = gKineAll.LLegRotM[5];
				gKineAll.LSwitchRMot[8] = gKineAll.LLegRotM[6];
				gKineAll.LSwitchRMot[9] = gKineAll.LLegRotM[7];
				gKineAll.LSwitchRMot[10] = gKineAll.LLegRotM[8];
				gKineAll.RSwitchRMot[0] = gKineAll.RLegRotM[0];
				gKineAll.RSwitchRMot[1] = gKineAll.RLegRotM[1];
				gKineAll.RSwitchRMot[2] = gKineAll.RLegRotM[2];
				gKineAll.RSwitchRMot[4] = gKineAll.RLegRotM[3];
				gKineAll.RSwitchRMot[5] = gKineAll.RLegRotM[4];
				gKineAll.RSwitchRMot[6] = gKineAll.RLegRotM[5];
				gKineAll.RSwitchRMot[8] = gKineAll.RLegRotM[6];
				gKineAll.RSwitchRMot[9] = gKineAll.RLegRotM[7];
				gKineAll.RSwitchRMot[10] = gKineAll.RLegRotM[8];

				gLAngZWorld = AngL; // 記下世界中的z方向角度 在連接軌跡還有軌跡生成的時候要用
				gRAngZWorld = AngR; // 記下世界中的z方向角度 在連接軌跡還有軌跡生成的時候要用

			}
			else if (gKineAll.selIK == RightSupport || gKineAll.selIK == DoubleSupport)
			{
				gKineAll.remRL[0] = gKineAll.CrdAll->data[57];
				gKineAll.remRL[1] = gKineAll.CrdAll->data[58];
				gKineAll.remRL[2] = gKineAll.CrdAll->data[59];
				gKineAll.shiftRL[0] = gKineAll.CrdAll->data[60];
				gKineAll.shiftRL[1] = gKineAll.CrdAll->data[61];
				gKineAll.shiftRL[2] = gKineAll.CrdAll->data[62];
				

				gKineAll.remLL[0] = gKineAll.CrdAll->data[18];
				gKineAll.remLL[1] = gKineAll.CrdAll->data[19];
				gKineAll.remLL[2] = gKineAll.CrdAll->data[20];
				gKineAll.shiftLL[0] = gKineAll.CrdAll->data[21];
				gKineAll.shiftLL[1] = gKineAll.CrdAll->data[22];
				gKineAll.shiftLL[2] = gKineAll.CrdAll->data[23];


				gKineAll.GetLegsCoords();
				gKineAll.LSwitchRMot[0] = gKineAll.LLegRotM[0];
				gKineAll.LSwitchRMot[1] = gKineAll.LLegRotM[1];
				gKineAll.LSwitchRMot[2] = gKineAll.LLegRotM[2];
				gKineAll.LSwitchRMot[4] = gKineAll.LLegRotM[3];
				gKineAll.LSwitchRMot[5] = gKineAll.LLegRotM[4];
				gKineAll.LSwitchRMot[6] = gKineAll.LLegRotM[5];
				gKineAll.LSwitchRMot[8] = gKineAll.LLegRotM[6];
				gKineAll.LSwitchRMot[9] = gKineAll.LLegRotM[7];
				gKineAll.LSwitchRMot[10] = gKineAll.LLegRotM[8];
				gKineAll.RSwitchRMot[0] = gKineAll.RLegRotM[0];
				gKineAll.RSwitchRMot[1] = gKineAll.RLegRotM[1];
				gKineAll.RSwitchRMot[2] = gKineAll.RLegRotM[2];
				gKineAll.RSwitchRMot[4] = gKineAll.RLegRotM[3];
				gKineAll.RSwitchRMot[5] = gKineAll.RLegRotM[4];
				gKineAll.RSwitchRMot[6] = gKineAll.RLegRotM[5];
				gKineAll.RSwitchRMot[8] = gKineAll.RLegRotM[6];
				gKineAll.RSwitchRMot[9] = gKineAll.RLegRotM[7];
				gKineAll.RSwitchRMot[10] = gKineAll.RLegRotM[8];

				gLAngZWorld = AngL; // 記下世界中的z方向角度 在連接軌跡還有軌跡生成的時候要用
				gRAngZWorld = AngR; // 記下世界中的z方向角度 在連接軌跡還有軌跡生成的時候要用
			}

			if (gKineAll.selIK == DoubleSupport) // 右腳的時候要補記用以固定腳的位置
			{
				gKineAll.remLL[0] = gKineAll.CrdAll->data[18];
				gKineAll.remLL[1] = gKineAll.CrdAll->data[19];
				gKineAll.remLL[2] = gKineAll.CrdAll->data[20];
				gKineAll.shiftLL[0] = gKineAll.CrdAll->data[21];
				gKineAll.shiftLL[1] = gKineAll.CrdAll->data[22];
				gKineAll.shiftLL[2] = gKineAll.CrdAll->data[23];

				gKineAll.remRL[0] = gKineAll.CrdAll->data[57];
				gKineAll.remRL[1] = gKineAll.CrdAll->data[58];
				gKineAll.remRL[2] = gKineAll.CrdAll->data[59];
				gKineAll.shiftRL[0] = gKineAll.CrdAll->data[60];
				gKineAll.shiftRL[1] = gKineAll.CrdAll->data[61];
				gKineAll.shiftRL[2] = gKineAll.CrdAll->data[62];

			}

		}
		gIthIK += 1;
  }


  void gThreadTest(void)
  {
	/******************************************************************
	input: void
	output: void

	Note:
	//  測試Sleep function 的精確度
	// 一般電腦的精確度大約在1ms左右 也就是會加減1ms
	******************************************************************/

	  while(1)
	  {
		  gLQs.tic();
		  Sleep(1);
		  gLQs.toc();
	  }
  }

void gSaveJointData(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	//  儲存解完IK 或是獲得新軌跡點的時候的各軸軌跡值
	//  並且將其直接轉換為C32讀得懂的資料格式
	******************************************************************/

	  double temp_joint;
		#if OfflineTraj	// 直接輸入離線Enc軌跡
			for (int i = 0 ; i < 12 ; ++i)
			{
				gTempEncs[i] = gKineAll.COG_ENCload[TickNumber*12+i];
			}
		#else
		for (int i=1 ; i < 7 ; i++)	//左腳
		{
			temp_joint = (gKineAll.FKLLeg->theta[i] + Lfeedbacktheta[i-1]) *gPNJoints[i-1];	//temp_joint = gKineAll.FKLLeg->theta[i]  *gPNJoints[i-1];
			////加入deltatheta for upright control  20140218 哲軒

			#if FootpadRotation
				if(gKineAll.selIK == 1 && i==5) //left support
				{
					temp_joint=(gKineAll.FKLLeg->theta[5]+gKineAll.AnklePitchRef[gIthIK%gStepSample]/cos(gKineAll.FKLLeg->theta[6]))*gPNJoints[i-1];
				}
			#endif

			if  (i == 1) // yaw axis is 26:40
				gTempEnc = (long)(temp_joint * 78353.2027529); // *2000*160*40/26/2pi 減速比
			else // other axes
				gTempEnc = (long)(temp_joint * 74896.4438101); // *2000*160*50/34/2pi 減速比

			if (i == 1) // yaw axis
				gTempEncs[i-1] = (long)(temp_joint * 78353.2027529); // *2000*160*40/26/2pi 減速比
			else // other axes
				gTempEncs[i-1] = (long)(temp_joint * 74896.4438101); // *2000*160*50/34/2pi 減速比

			gShortTrajBuf[gBufIndex++] = i;

			if (gTempEnc > 0)
			{
				gShortTrajBuf[gBufIndex++] = (unsigned char)(gTempEnc);
				gShortTrajBuf[gBufIndex++] = (unsigned char)(gTempEnc>>8);
				gShortTrajBuf[gBufIndex++] = (unsigned char)(gTempEnc>>16);
				gShortTrajBuf[gBufIndex++] = (unsigned char)(gTempEnc>>24);
			}
			else if (gTempEnc == 0)
			{
				gShortTrajBuf[gBufIndex++] = 0;
				gShortTrajBuf[gBufIndex++] = 0;
				gShortTrajBuf[gBufIndex++] = 0;
				gShortTrajBuf[gBufIndex++] = 0;
			}
			else
			{
				gTempEnc = -gTempEnc;
				gShortTrajBuf[gBufIndex++] = (unsigned char)(gTempEnc);
				gShortTrajBuf[gBufIndex++] = (unsigned char)(gTempEnc>>8);
				gShortTrajBuf[gBufIndex++] = (unsigned char)(gTempEnc>>16);
				gShortTrajBuf[gBufIndex++] = 255-(unsigned char)(gTempEnc>>24);
			}
		}		

		for (int i=1 ; i < 7 ; i++)	//右腳
		{
			temp_joint = (gKineAll.FKRLeg->theta[i]+Rfeedbacktheta[i-1])*gPNJoints[i+5];	////加入deltatheta for upright control  20140218 哲軒
			//temp_joint = gKineAll.FKRLeg->theta[i]*gPNJoints[i+5];
			#if FootpadRotation
				if(gKineAll.selIK == 0 && i==5) //left support
				{
					temp_joint=(gKineAll.FKRLeg->theta[5]-gKineAll.AnklePitchRef[gIthIK%gStepSample]/cos(gKineAll.FKRLeg->theta[6]))*gPNJoints[i+5];
				}
			#endif

			if (i == 1) // yaw axis
				gTempEnc = (long)(temp_joint * 78353.2027529); // *2000*160*40/26/2pi 減速比
			else // other axes
				gTempEnc = (long)(temp_joint * 74896.4438101); // *2000*160*50/34/2pi 減速比

			if (i == 1) // yaw axis
				gTempEncs[i-1+6] = (long)(temp_joint * 78353.2027529); // *2000*160*40/26/2pi 減速比
			else // other axes
				gTempEncs[i-1+6] = (long)(temp_joint * 74896.4438101); // *2000*160*50/34/2pi 減速比

			gShortTrajBuf[gBufIndex++] = i+6;

			if (gTempEnc > 0)
			{
				gShortTrajBuf[gBufIndex++] = (unsigned char)(gTempEnc);
				gShortTrajBuf[gBufIndex++] = (unsigned char)(gTempEnc>>8);
				gShortTrajBuf[gBufIndex++] = (unsigned char)(gTempEnc>>16);
				gShortTrajBuf[gBufIndex++] = (unsigned char)(gTempEnc>>24);
			}
			else if (gTempEnc == 0)
			{
				gShortTrajBuf[gBufIndex++] = 0;
				gShortTrajBuf[gBufIndex++] = 0;
				gShortTrajBuf[gBufIndex++] = 0;
				gShortTrajBuf[gBufIndex++] = 0;
			}
			else
			{
				gTempEnc = -gTempEnc;
				gShortTrajBuf[gBufIndex++] = (unsigned char)(gTempEnc);
				gShortTrajBuf[gBufIndex++] = (unsigned char)(gTempEnc>>8);
				gShortTrajBuf[gBufIndex++] = (unsigned char)(gTempEnc>>16);
				gShortTrajBuf[gBufIndex++] = 255-(unsigned char)(gTempEnc>>24);
			}
		}
		#endif
 		
		gWarningJointLimit(gTempEncs);//check 目前軌跡接近EPOS3 limit的程度

		#if cogestimate
			if(IMU1.IMU_Lock==false) //針對IMU Thread 同步問題設置的lock
			{
				LogIMUanglex[LogEncCount]=IMU1.finalanglex[IMU1.count-2];
				LogIMUangley[LogEncCount]=IMU1.finalangley[IMU1.count-2];
				LogIMUvelx[LogEncCount] =  IMU1.velx[IMU1.count-2] ;
				LogIMUvely[LogEncCount] =  IMU1.vely[IMU1.count-2] ;
				LogIMUaccelx[LogEncCount] = IMU1.accelx[IMU1.count-2];
				LogIMUaccely[LogEncCount] = IMU1.accely[IMU1.count-2];	
			}
			else
			{
				LogIMUanglex[LogEncCount]=IMU1.finalanglex[IMU1.count-1];
				LogIMUangley[LogEncCount]=IMU1.finalangley[IMU1.count-1];
				LogIMUvelx[LogEncCount] =  IMU1.velx[IMU1.count-1] ;
				LogIMUvely[LogEncCount] =  IMU1.vely[IMU1.count-1] ;
				LogIMUaccelx[LogEncCount] = IMU1.accelx[IMU1.count-1];
				LogIMUaccely[LogEncCount] = IMU1.accely[IMU1.count-1];					
			}
		#endif	

		if (gFlagSimulation == RealExp && gKineAll.FlagStayMode == 0) // experiment mode and robot doesn't hold position
		{
			#if TwinCAT_Mode
				for(int i = 0 ; i < 12 ; i++)
					gTempEncs[i] += CaliTemp[i];
		
				TCAT->EtherCATReadEncoder(LogEnc+12*LogEncCount); 
				TCAT->EtherCATReadVel(LogVel+12*LogEncCount);
			
				#if BangBangControl
				TCAT->EtherCATReadEncDiff(LogEncDiff+12*LogEncCount);
				TCAT->MovingAve(20,LogVel+12*LogEncCount,LogMovAveVel+12*LogEncCount); //對讀取Joint Vel 作Moving Average

				for(int i = 0 ; i < 12 ; i++)
				{
					LogDesireVel[i+12*LogEncCount]=(gKineAll.ThetaD[i]*gPNJoints[i]+30*LogDesireVelBuf[i])/31;//對自己數值微分的 Vel 作Filtering
					LogDesireVelBuf[i]=LogDesireVel[i+12*LogEncCount];
					LogVelDiff[i+12*LogEncCount]=double (LogMovAveVel[i+12*LogEncCount])/160/50*34/60*2*3.1415926535-LogDesireVel[i+12*LogEncCount];
				}

				SFunction(LogVelDiff+12*LogEncCount,LogEncDiff+12*LogEncCount,LogSFunction+12*LogEncCount);
				SdBengbengControl(LogSFunction+12*LogEncCount,LogTorqueOffset+12*LogEncCount,LogTorqueBuf); 
					#if PreloadTorque 
						if(gSecNumb==0)//upstair   first scene
						{
							for(int i=0;i<12;i++)
							{
								LogTorqueBuf[i] += *(gShortTorqueOffsetFileBuf1+12*LogPreLoadCount+i);//上樓梯用
								//LogTorqueBuf[i] += *(gShortTorqueOffsetFileBuf2+12*LogPreLoadCount+i);//第一scenario down stair 用
							}
						}
						else if(gSecNumb==1)//downstair   second scene
						{
							for(int i=0;i<12;i++)
							{
								LogTorqueBuf[i] += *(gShortTorqueOffsetFileBuf2+12*LogPreLoadCount+i);//下樓梯用
							}
						}
						/*else if(gSecNumb==2) //now no use
						{
							for(int i=0;i<12;i++)
							{
								LogTorqueBuf[i] += *(gShortTorqueOffsetFileBuf3+12*LogPreLoadCount+i);
							}
						}*/
					#endif
				
				TCAT->EtherCATSetTorqueOffset(LogTorqueBuf);

				for(int i=0;i<12;i++)
				{
					LogVelBuf[i]=long(LogDesireVel[i+12*LogEncCount]*160*50/34*60/2/3.1415926535);//Yaw的部分有 Error需要重改
				}
				//TCAT->EtherCATSetVelOffset(LogVelBuf); //目前沒有使用
				#endif
				
				TCAT->EtherCATReadTorque(LogTorque+12*LogEncCount);		
				TCAT->EtherCATSetEncoder(gTempEncs);
			#endif				
				
			if(IMU1.IMU_Lock) //針對IMU Thread 同步問題設置的lock
			{
				LogIMUAngVel[0+3*LogEncCount]=IMU1.velx[IMU1.count-2];
				LogIMUAngVel[1+3*LogEncCount]=IMU1.vely[IMU1.count-2];
				LogIMUAngVel[2+3*LogEncCount]=IMU1.velz[IMU1.count-2];
			}
			else
			{
				LogIMUAngVel[0+3*LogEncCount]=IMU1.velx[IMU1.count-1];
				LogIMUAngVel[1+3*LogEncCount]=IMU1.vely[IMU1.count-1];
				LogIMUAngVel[2+3*LogEncCount]=IMU1.velz[IMU1.count-1];				
			}

			for(int i = 0 ; i < 3 ; i++)
			{
				LogIMUAngVelMA[i+3*LogEncCount]=(LogIMUAngVel[i+3*LogEncCount]+100*LogIMUAngVelBuf[i])/101;//對IMU的Angular Vel 作Filtering
				LogIMUAngVelBuf[i]=LogIMUAngVelMA[i+3*LogEncCount];
			}
		}
		if(LogEncCount<DataTotal-1)
		{
		LogEncCount+=1;
		LogPreLoadCount+=1;
		}
		/////////////////////////////// Added by Wei-Zh & Slongz
		
		if (gBufIndex == BufferLen)
			gBufIndex = 0;

		//泓逸start20120308 WARNING!!!要重改
	if(	TickNumber%int(dt/dtArm) ==0 )
	{
		//torso
		for (int i=0 ; i < 2 ; i++)
		{
			temp_joint = gKineAll.FKLArm->theta[i+1]*gPNJoints[i+12];
			if  (i == 1) // yaw axis 
				gTempEnc = (long)(temp_joint * 1); // *2000*160*40/26/2pi 減速比
			else // other axes
				gTempEnc = (long)(temp_joint * 1); // *2000*160*50/34/2pi 減速比

			gShortTrajBufTorso[gBufIndexTorso++] = i+1;

			if (gTempEnc > 0)
			{
				gShortTrajBufTorso[gBufIndexTorso++] = (unsigned char)(gTempEnc);
				gShortTrajBufTorso[gBufIndexTorso++] = (unsigned char)(gTempEnc>>8);
				gShortTrajBufTorso[gBufIndexTorso++] = (unsigned char)(gTempEnc>>16);
				gShortTrajBufTorso[gBufIndexTorso++] = (unsigned char)(gTempEnc>>24);
			}
			else if (gTempEnc == 0)
			{
				gShortTrajBufTorso[gBufIndexTorso++] = 0;
				gShortTrajBufTorso[gBufIndexTorso++] = 0;
				gShortTrajBufTorso[gBufIndexTorso++] = 0;
				gShortTrajBufTorso[gBufIndexTorso++] = 0;
			}
			else
			{
				gTempEnc = -gTempEnc;
				gShortTrajBufTorso[gBufIndexTorso++] = (unsigned char)(gTempEnc);
				gShortTrajBufTorso[gBufIndexTorso++] = (unsigned char)(gTempEnc>>8);
				gShortTrajBufTorso[gBufIndexTorso++] = (unsigned char)(gTempEnc>>16);
				gShortTrajBufTorso[gBufIndexTorso++] = 255-(unsigned char)(gTempEnc>>24);
			}
		}

		if (gBufIndexTorso == BufferLenTorso)
			gBufIndexTorso = 0;

		//LArm
		for (int i=0 ; i < 6 ; i++)
		{
			temp_joint = gKineAll.FKLArm->theta[i+4]*gPNJoints[i+14];
			if  (i == 5) // last axis 
				gTempEnc = (long)(temp_joint / (2*PI) *268*512*4); //  減速比  268
			else // other axes
				gTempEnc = (long)(temp_joint / (2*PI) *150*512*4); //  減速比  150

			gShortTrajBufLArm[gBufIndexLArm++] = i+1;

			if (gTempEnc > 0)
			{
				gShortTrajBufLArm[gBufIndexLArm++] = (unsigned char)(gTempEnc);
				gShortTrajBufLArm[gBufIndexLArm++] = (unsigned char)(gTempEnc>>8);
				gShortTrajBufLArm[gBufIndexLArm++] = (unsigned char)(gTempEnc>>16);
				gShortTrajBufLArm[gBufIndexLArm++] = (unsigned char)(gTempEnc>>24);
			}
			else if (gTempEnc == 0)
			{
				gShortTrajBufLArm[gBufIndexLArm++] = 0;
				gShortTrajBufLArm[gBufIndexLArm++] = 0;
				gShortTrajBufLArm[gBufIndexLArm++] = 0;
				gShortTrajBufLArm[gBufIndexLArm++] = 0;
			}
			else
			{
				gTempEnc = -gTempEnc;
				gShortTrajBufLArm[gBufIndexLArm++] = (unsigned char)(gTempEnc);
				gShortTrajBufLArm[gBufIndexLArm++] = (unsigned char)(gTempEnc>>8);
				gShortTrajBufLArm[gBufIndexLArm++] = (unsigned char)(gTempEnc>>16);
				gShortTrajBufLArm[gBufIndexLArm++] = 255-(unsigned char)(gTempEnc>>24);
			}
		}

		if (gBufIndexLArm == BufferLenLArm)
			gBufIndexLArm = 0;

		//RArm
		for (int i=0 ; i < 6 ; i++)
		{
			temp_joint = gKineAll.FKRArm->theta[i+4]*gPNJoints[i+20];
			if  (i == 5) // last axis 
				gTempEnc = (long)(temp_joint / (2*PI) *268*512*4); //  減速比  268
			else // other axes
				gTempEnc = (long)(temp_joint / (2*PI) *150*512*4); //  減速比  150

			gShortTrajBufRArm[gBufIndexRArm++] = i+1;

			if (gTempEnc > 0)
			{
				gShortTrajBufRArm[gBufIndexRArm++] = (unsigned char)(gTempEnc);
				gShortTrajBufRArm[gBufIndexRArm++] = (unsigned char)(gTempEnc>>8);
				gShortTrajBufRArm[gBufIndexRArm++] = (unsigned char)(gTempEnc>>16);
				gShortTrajBufRArm[gBufIndexRArm++] = (unsigned char)(gTempEnc>>24);
			}
			else if (gTempEnc == 0)
			{
				gShortTrajBufRArm[gBufIndexRArm++] = 0;
				gShortTrajBufRArm[gBufIndexRArm++] = 0;
				gShortTrajBufRArm[gBufIndexRArm++] = 0;
				gShortTrajBufRArm[gBufIndexRArm++] = 0;
			}
			else
			{
				gTempEnc = -gTempEnc;
				gShortTrajBufRArm[gBufIndexRArm++] = (unsigned char)(gTempEnc);
				gShortTrajBufRArm[gBufIndexRArm++] = (unsigned char)(gTempEnc>>8);
				gShortTrajBufRArm[gBufIndexRArm++] = (unsigned char)(gTempEnc>>16);
				gShortTrajBufRArm[gBufIndexRArm++] = 255-(unsigned char)(gTempEnc>>24);
			}
		}
		
		if (gBufIndexRArm == BufferLenRArm)
			gBufIndexRArm = 0;
  }
		//泓逸end20120308 WARNING!!!要重改
}



  void CRobotAllDlg::OnBnClickedOk()
  {
	/******************************************************************
	input: void
	output: void

	Note:
	//  按下OK 程式準備結束
	//  在這邊有很多動態記憶體需要被清除
	//  已經開啟的thread 以及 GL視窗也要依照順序刪除
	******************************************************************/
	if (gIMUThreadOpened)
	{     
		gIMULife = false;
		WaitForSingleObject(gThreadIMU,INFINITE);
		CloseHandle(gThreadIMU);
	}

	if (gRenderPMSThreadOpened)
	{
		gRenderPMSLife = false;
		WaitForSingleObject(gThreadRenderPMS,INFINITE);
		CloseHandle(gThreadRenderPMS);
	}

	if (gRenderThreadOpened)
	{
		gRenderLife = false;
		WaitForSingleObject(gThreadRender,INFINITE);
		CloseHandle(gThreadRender);
	}

	if (gControlThreadOpened)
	{
		gControlLife = false;
		WaitForSingleObject(gThreadControl,INFINITE);
		CloseHandle(gThreadControl);
	}

	if (gArmControlThreadOpened)
	{
		gArmControlLife = false;
		WaitForSingleObject(gThreadArmControl,INFINITE);
		CloseHandle(gThreadArmControl);
	}

	if (gFaceControlThreadOpened)
	{
		gFaceLife = false;
		WaitForSingleObject(gThreadFace,INFINITE);
		CloseHandle(gThreadFace);
	}
	
	if (gFlagSimulation == RealExp)
		gClosePort();



	CDialogEx::OnOK();
	//OnOK();

	if (gIKGLOpened == true)
	{
		glutDestroyWindow(gGLID);
		exit(0);
	}

	if (gPMSGLOpened == true)
	{
		glutDestroyWindow(gGL_ID_PMS);
		exit(0);
	}

	if (gDrawingBufCreated)
	{
		delete[] gpRobotDrawingBuffer;
		delete[] gpRobotDrawingBufferArm;
	}

	if (gFlagHandCtrl == 1)
	{
		delete[] gfContTrajDataRH;
		delete[] gfContTrajDataLH;
	}

  }


  void CRobotAllDlg::OnBnClickedCancel()
  {
	/******************************************************************
	input: void
	output: void

	Note:
	// 此按鈕已被隱藏 為了要節省GUI空間
	// 要使用請直接押按 OK 按鈕即可
	//  按下cancel 程式準備結束
	//  在這邊有很多動態記憶體需要被清除
	//  已經開啟的thread 以及 GL視窗也要依照順序刪除
	******************************************************************/

	if (gRenderPMSThreadOpened)
	{
		gRenderPMSLife = false;
		WaitForSingleObject(gThreadRenderPMS,INFINITE);
		CloseHandle(gThreadRenderPMS);
	}

	if (gRenderThreadOpened)
	{
		gRenderLife = false;
		WaitForSingleObject(gThreadRender,INFINITE);
		CloseHandle(gThreadRender);
	}

	if (gControlThreadOpened)
	{
		gControlLife = false;
		WaitForSingleObject(gThreadControl,INFINITE);
		CloseHandle(gThreadControl);
	}

	if (gArmControlThreadOpened)
	{
		gArmControlLife = false;
		WaitForSingleObject(gThreadArmControl,INFINITE);
		CloseHandle(gThreadArmControl);
	}

	if (gFaceControlThreadOpened)
	{
		gFaceLife = false;
		WaitForSingleObject(gThreadFace,INFINITE);
		CloseHandle(gThreadFace);
	}

		if (gIMUThreadOpened)
	{
		gIMULife = false;
		WaitForSingleObject(gThreadIMU,INFINITE);
		CloseHandle(gThreadIMU);
	}


	gClosePort();


    CDialogEx::OnCancel();
	//OnCancel();

	if (gIKGLOpened == true)
	{
		glutDestroyWindow(gGLID);
		exit(0);
	}

	if (gPMSGLOpened == true)
	{
		glutDestroyWindow(gGL_ID_PMS);
		exit(0);
	}

	if (gDrawingBufCreated)
	{
		delete[] gpRobotDrawingBuffer;
		delete[] gpRobotDrawingBufferArm;
	}

  }


  void CRobotAllDlg::OnBnClickedButton4() // 被整合到 Auto button 中 會自動被壓下
  {

	/******************************************************************
	input: void
	output: void

	Note:
	// 機器人自動初始化與數值設定
	// 主要牽涉到真正與機器人溝通與設定的部分
	******************************************************************/

	gInitialization();
	  // TODO: Add your control notification handler code here
	gPIDUpdLSup[0] = 255;		gPIDUpdLSup[1] = 255;		gPIDUpdLSup[2] = 255; 
	gPIDUpdLSup[3] = 10;		   // gPIDUpdLSup[3] 是 scale, pid值放大之倍率，因為給原始值會超過255, 所以把所有值同除這個數字
	// Left Leg : support
	/* L Hip Yaw     */  gPIDUpdLSup[4] = 255;		gPIDUpdLSup[5] = 0;		gPIDUpdLSup[6] = 0;								   // 第1軸 P I D / 10
	/* L Hip Roll    */  gPIDUpdLSup[7] = 160;		gPIDUpdLSup[8] = 0;		gPIDUpdLSup[9] = 0;							       // 第2軸 P I D / 10
	/* L Hip Pitch   */  gPIDUpdLSup[10] =110;		gPIDUpdLSup[11] = 0;		gPIDUpdLSup[12] = 0;							       // 第3軸 P I D / 10
	/* L Knee Pitch  */  gPIDUpdLSup[13] = 80;		gPIDUpdLSup[14] = 0;		gPIDUpdLSup[15] = 0;							       // 第4軸 P I D / 10
	/* L Ankle Pitch */  gPIDUpdLSup[16] = 110;		gPIDUpdLSup[17] = 0;		gPIDUpdLSup[18] = 0;							       // 第5軸 P I D / 10
	/* L Ankle Roll  */  gPIDUpdLSup[19] = 110;		gPIDUpdLSup[20] = 0;		gPIDUpdLSup[21] = 0;							       // 第6軸 P I D / 10

	// Right Leg : swing
	/* R Hip Yaw     */  gPIDUpdLSup[22] = 255;		gPIDUpdLSup[23] = 0;		gPIDUpdLSup[24] = 0;							       // 第7軸 P I D / 10
	/* R Hip Roll    */  gPIDUpdLSup[25] = 110;		gPIDUpdLSup[26] = 0;		gPIDUpdLSup[27] = 0;							       // 第8軸 P I D / 10
	/* R Hip Pitch   */  gPIDUpdLSup[28] = 75;		gPIDUpdLSup[29] = 0;		gPIDUpdLSup[30] = 0;							       // 第9軸 P I D / 10
	/* R Knee Pitch  */  gPIDUpdLSup[31] = 50;		gPIDUpdLSup[32] = 0;		gPIDUpdLSup[33] = 0;							       // 第10軸 P I D / 10
	/* R Ankle Pitch */  gPIDUpdLSup[34] = 70;		gPIDUpdLSup[35] = 0;		gPIDUpdLSup[36] = 0;							       // 第11軸 P I D / 10
	/* R Ankle Roll  */  gPIDUpdLSup[37] = 75;		gPIDUpdLSup[38] = 0;		gPIDUpdLSup[39] = 0;							       // 第12軸 P I D / 10


	gPIDUpdRSup[0] = 255;		gPIDUpdRSup[1] = 255;		gPIDUpdRSup[2] = 255; 
	gPIDUpdRSup[3] = 10;		   // gPIDUpdLSup[3] 是 scale, pid值放大之倍率，因為給原始值會超過255, 所以把所有值同除這個數字
	//  Left Leg : swing
	/* L Hip Yaw     */  gPIDUpdRSup[4] = 255;			gPIDUpdRSup[5] = 0;		gPIDUpdRSup[6] = 0;								   // 第1軸 P I D / 10
	/* L Hip Roll    */  gPIDUpdRSup[7] = 110;		gPIDUpdRSup[8] = 0;		gPIDUpdRSup[9] = 0;							       // 第2軸 P I D / 10
	/* L Hip Pitch   */  gPIDUpdRSup[10] = 75;		gPIDUpdRSup[11] = 0;		gPIDUpdRSup[12] = 0;							       // 第3軸 P I D / 10
	/* L Knee Pitch  */  gPIDUpdRSup[13] = 50;		gPIDUpdRSup[14] = 0;		gPIDUpdRSup[15] = 0;							       // 第4軸 P I D / 10
	/* L Ankle Pitch */  gPIDUpdRSup[16] = 70;		gPIDUpdRSup[17] = 0;		gPIDUpdRSup[18] = 0;							       // 第5軸 P I D / 10
	/* L Ankle Roll  */  gPIDUpdRSup[19] = 75;		gPIDUpdRSup[20] = 0;		gPIDUpdRSup[21] = 0;							       // 第6軸 P I D / 10

	// Right Leg : support
	/* R Hip Yaw     */	 gPIDUpdRSup[22] = 255;		gPIDUpdRSup[23] = 0;		gPIDUpdRSup[24] = 0;							       // 第7軸 P I D / 10
	/* R Hip Roll    */  gPIDUpdRSup[25] = 160;		gPIDUpdRSup[26] = 0;		gPIDUpdRSup[27] = 0;							       // 第8軸 P I D / 10
	/* R Hip Pitch   */  gPIDUpdRSup[28] = 110;		gPIDUpdRSup[29] = 0;		gPIDUpdRSup[30] = 0;							       // 第9軸 P I D / 10
	/* R Knee Pitch  */  gPIDUpdRSup[31] = 80;		gPIDUpdRSup[32] = 0;		gPIDUpdRSup[33] = 0;							       // 第10軸 P I D / 10
	/* R Ankle Pitch */  gPIDUpdRSup[34] = 110;		gPIDUpdRSup[35] = 0;		gPIDUpdRSup[36] = 0;							       // 第11軸 P I D / 10
	/* R Ankle Roll  */	 gPIDUpdRSup[37] = 110;		gPIDUpdRSup[38] = 0;		gPIDUpdRSup[39] = 0;							       // 第12軸 P I D / 10


	
	gPIDUpdDSup[0] = 255;		gPIDUpdDSup[1] = 255;		gPIDUpdDSup[2] = 255; 
	gPIDUpdDSup[3] = 10;		   // gPIDUpdLSup[3] 是 scale, pid值放大之倍率，因為給原始值會超過255, 所以把所有值同除這個數字
	// Left Leg : double support
	/* L Hip Yaw     */  gPIDUpdDSup[4] = 250;			gPIDUpdDSup[5] = 0;		gPIDUpdDSup[6] = 0;								   // 第1軸 P I D / 10
	/* L Hip Roll    */  gPIDUpdDSup[7] = 85;		gPIDUpdDSup[8] = 0;		gPIDUpdDSup[9] = 0;							       // 第2軸 P I D / 10
	/* L Hip Pitch   */  gPIDUpdDSup[10] = 80;		gPIDUpdDSup[11] = 0;		gPIDUpdDSup[12] = 0;							       // 第3軸 P I D / 10
	/* L Knee Pitch  */  gPIDUpdDSup[13] = 65;		gPIDUpdDSup[14] = 0;		gPIDUpdDSup[15] = 0;							       // 第4軸 P I D / 10
	/* L Ankle Pitch */  gPIDUpdDSup[16] = 80;		gPIDUpdDSup[17] = 0;		gPIDUpdDSup[18] = 0;							       // 第5軸 P I D / 10
	/* L Ankle Roll  */  gPIDUpdDSup[19] = 85;		gPIDUpdDSup[20] = 0;		gPIDUpdDSup[21] = 0;							       // 第6軸 P I D / 10

	// Right Leg : double support
	/* R Hip Yaw     */  gPIDUpdDSup[22] = 255;		gPIDUpdDSup[23] = 0;		gPIDUpdDSup[24] = 0;							       // 第7軸 P I D / 10
	/* R Hip Roll    */  gPIDUpdDSup[25] = 85;		gPIDUpdDSup[26] = 0;		gPIDUpdDSup[27] = 0;							       // 第8軸 P I D / 10
	/* R Hip Pitch   */  gPIDUpdDSup[28] = 80;		gPIDUpdDSup[29] = 0;		gPIDUpdDSup[30] = 0;							       // 第9軸 P I D / 10
	/* R Knee Pitch  */  gPIDUpdDSup[31] = 65;		gPIDUpdDSup[32] = 0;		gPIDUpdDSup[33] = 0;							       // 第10軸 P I D / 10
	/* R Ankle Pitch */  gPIDUpdDSup[34] = 80;		gPIDUpdDSup[35] = 0;		gPIDUpdDSup[36] = 0;							       // 第11軸 P I D / 10
	/* R Ankle Roll  */  gPIDUpdDSup[37] = 85;		gPIDUpdDSup[38] = 0;		gPIDUpdDSup[39] = 0;							       // 第12軸 P I D / 10

	gInitTimerAndThread(); // every 3 ms

  }


  void CRobotAllDlg::OnBnClickedButton5() // 被整合到 Auto button 中 會自動被壓下
  {
	/******************************************************************
	input: void
	output: void

	Note:
	// 機器人打開所有COM PORT 
	******************************************************************/
	  // TODO: Add your control notification handler code here
	  gOpenPort();

  }

  void CRobotAllDlg::OnCbnSelchangeCombo1() // 設定手動模式時 要對機器人進行的動作
  {
	/******************************************************************
	input: void
	output: void

	Note:
	// 要手動設定機器人控制模式就要在GUI更改這邊的設定
	// 手動模式專用
	******************************************************************/
	SendIndex = SendMode.GetCurSel();
	CString strCBText;
	SendMode.GetLBText( SendIndex, strCBText);
	cout<<"Sending Mode : "<<strCBText<<endl;
  }


  void CRobotAllDlg::OnBnClickedButton6() // 按鈕 "Send" 手動模式專用
  {
	/******************************************************************
	input: void
	output: void

	Note:
	// 要手動設定機器人控制模式就要在GUI更改這邊的設定
	// 手動模式專用
	******************************************************************/  
	  gPreProcessData(SendIndex);
  }


  void CRobotAllDlg::OnBnClickedButton7() // 按鈕 PMS/BMS
  {
	/******************************************************************
	input: void
	output: void

	Note:
	// 開啟電源管理執行緒
	// 機器人開始擷取各部分(包含電池) 的電壓 電流 溫度 資料
	******************************************************************/
		gThreadRenderPMS = CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)gRenderPMSThread,(void*)0,0,&gTIDRenderPMS);
		gRenderPMSThreadOpened = true;
		//SetThreadPriority(gThreadRenderPMS,THREAD_PRIORITY_NORMAL);
		SetThreadPriority(gThreadRenderPMS,THREAD_PRIORITY_TIME_CRITICAL);	// 20130829 WZ 改PMSThread priority

		gStartTimeAcquiredPMS = true;
		QueryPerformanceCounter(&gStartTimePMS);

  }


  void CRobotAllDlg::OnBnClickedButton8() // 按鈕 Init_PBMS
  {
	/******************************************************************
	input: void
	output: void

	Note:
	// 開啟電源管理通訊埠
	// 打開所需要的COM PORT
	// 若是只要部分COM PORT 請在程式中註解掉不要的部分
	******************************************************************/
	if (gFlagReadForceSensor==1)
	{
		  #if TwinCAT_Mode
		  printf("Wait 5 seconds for TwinCAT...\n");
		  Sleep(5000);
		  TCAT=new TwinCAT_COM();		  
		  gDeleteLogFile();
		  #else
		  printf("TwinCAT_Mode in MainLoops.h: disable, Press any key to continue \n");
		  system("pause");
		  #endif
	}
	try
	{
		//gPMS_LArm = new SerialPort();
		//gPMS_RArm = new SerialPort();
		gPMS_LLeg = new SerialPort();
		gPMS_RLeg = new SerialPort();
		//gPMS_Head = new SerialPort();
		//gBMS = new SerialPort();


		//gPMS_LArm->open("\\\\.\\COM7",NORMAL_RS232);
		//gPMS_LArm->_set_baudrate(115200);
		//gPMS_RArm->open("\\\\.\\COM10",NORMAL_RS232);
		//gPMS_RArm->_set_baudrate(115200);
		gPMS_LLeg->open("\\\\.\\COM4",NORMAL_RS232);  
		gPMS_LLeg->_set_baudrate(115200);
		gPMS_RLeg->open("\\\\.\\COM3",NORMAL_RS232);
		gPMS_RLeg->_set_baudrate(115200);
		
		//gPMS_Head->open("\\\\.\\COM56",NORMAL_RS232);
		//gPMS_Head->_set_baudrate(115200);
		
		
		//gBMS->open("\\\\.\\COM56",NORMAL_RS232);
		//gBMS->_set_baudrate(115200);
		
		
		//if (gFlagIMU)
		//{   /*gIMU = new SerialPort();
		//	gIMU->open("\\\\.\\COM84",NORMAL_RS232);
		//	gIMU->_set_baudrate(115200);
		//	gIMUThreadOpened = true;
		//	
		//}

	}
	catch(...)
	{
		cout<<"COM open faied"<<endl;
	}

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH); // double buffering + depth test


	gPMSSizeX = 320;
	gPMSSizeY = 640;

	glutInitWindowSize(gPMSSizeX,gPMSSizeY);
	glutInitWindowPosition(895,0); 
	gGL_ID_PMS = glutCreateWindow("PMS/BMS");

	glutReshapeFunc(gOnSizeMyGLPMS);					// 當變更 Windows size 時呼叫 OnSize

	glutDisplayFunc(gRenderPMSEmpty);

	glEnable(GL_DEPTH_TEST);

	glClearColor(0.0f,0.0f,0.0f,1.0f);
	glutSwapBuffers();
	glClearColor(0.0f,0.0f,0.0f,1.0f);
	glutSwapBuffers();

	gPMSGLOpened = true;
	OnBnClickedButton7() ;
	glutMainLoop();
	
  }

void gOnSizeMyGLPMS(int w, int h)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// PMS的GL視窗被改變大小時
	// 自動刷新與重設視窗
	******************************************************************/
	GLfloat fAspect;
	//GLfloat nRange = 100.0f;

    // Prevent a divide by zero
    if(h == 0)
        h = 1;

    // 設定 Viewport 
    glViewport(0, 0, w, h);

	fAspect = (GLfloat)w /(GLfloat)h;
	// 顯像的步驟
    // 重新設定 Projection matrix
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

	// 作 Perspective 投射
	//gluPerspective(proj_ang, fAspect, 1.0, 425.0);
	//gluPerspective(45.0f, 1.0, gViewRangeNear, gViewRangeFar);
	//gluPerspective(45.0f, 1.0, gViewRangeNear, gViewRangeFar);
	gluPerspective(45.0f, 1.0, 0.1, 4000);
	// 重新設定 View matrix
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

}


void gRenderPMSEmpty(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 空執行緒 防止GL視窗自動執行不必要的刷新
	// 所有刷新都使用計時器中的FPS來控制速度
	******************************************************************/
}

void gRenderPMSThread(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// PMS視窗刷新專用函式 PMS的繪圖執行續會執行這個function繪圖以及資料擷取
	// 所有行為都在這個function的while(1) Loop之中
	// 力規的單位 ForceUnits="N" TorqueUnits="N-m"
	//
	// 20130424 WZ更新timer 將繪圖與收資料分開 
	// sensor不再用sleep是因為SENSOR在5ms下直接跑 敢這樣用是因為sensor經測試來回小於2ms
	// PMS時間開始點在開啟PMSthread時就已經打開
	// 若不想畫圖 請將 PMSUpdate = 0
	// PMS的timer命名與主程式中不太相同 因為主程式裡還會需要記下每段軌跡的local time這裡不用
	// 若要使用舊timer請將下面蓋起來的gRenderPMSThread打開測試
	// WZ
	******************************************************************/
	int IndexBuf = 0;
	int IndexBufRev = 0;

	////timer
	//LARGE_INTEGER StartTime_Render;	// 電腦計算時間開始點 for control
	LARGE_INTEGER CurrentTime_Render;	// 電腦計算時間現在點
	LARGE_INTEGER FreqSys;	 // CPU頻率 使用64位元儲存
	double SysTime_Render = 0;	// 計算出來的當前系統時間 單位是秒
	float Freq_Render; //// CPU頻率 使用float儲存 因為要相除
	QueryPerformanceFrequency(&FreqSys);	//抓頻率
	Freq_Render = (float)FreqSys.QuadPart;	//抓頻率
	//QueryPerformanceCounter(&StartTime_Render);	//record initial time
	unsigned int TickNumber_Render = 0;	// Timer Counts
	unsigned int TickNumber_Sensor = 0;	// Timer Counts for Sensors
	/////////___________________________________________________________________________________________________
	float FPS = 10.0;
	/////////___________________________________________________________________________________________________
	//double TimeDiff;
	//double Now_Time; 已由SysTime_Render取代
	////timer

	char time[60];
	char *c;

	char msg_title[60];
	char *m;

	float w_scale_x;
	float w_scale_y;

	unsigned char CallStat[5];
	CallStat[0] = 255;
	CallStat[1] = 0;
	CallStat[2] = 100;
	CallStat[3] = 0;
	CallStat[4] = 0;//Q為CallStat[2]+CallStat[3]*256+CallStat[2]*65536

	unsigned char IMUCmd[1];//IMU傳下去的cmd
	IMUCmd[0] = 204;
	
	double MV48Voltage = 48; // max voltage 3.2*15
	double mV48Voltage = 37.5; // min voltage 2.5*15
	double MV24Voltage = 25.6; // max voltage 3.2*15
	double mV24Voltage = 20; // min voltage 2.5*15
	double MA48Current = 20; // max voltage 3.2*15
	double mA48Current = 0; // min voltage 2.5*15
	double MA24Current = 8; // max voltage 3.2*15
	double mA24Current = 0; // min voltage 2.5*15
	double averagetime = 5;
	int	   averagecal = 0;
	int    average = 0;
	int    start = 0;//1代表可以開始量測
	double tL1 = 0;
	double tL2 = 0;
	double tL3 = 0;
	double tL4 = 0;
	double tL5 = 0;
	double tL6 = 0;
	double tL7 = 0;
	double tL8 = 0;
	double tL9 = 0;
	double tL10 = 0;
	double tLmean = 0; //mean 
	double tLslope = 0;//angle

	double tR1 = 0;
	double tR2 = 0;
	double tR3 = 0;
	double tR4 = 0;
	double tR5 = 0;
	double tR6 = 0;
	double tR7 = 0;
	double tR8 = 0;
	double tR9 = 0;
	double tR10 = 0;
	double tRmean = 0;
	double tRslope = 0;

	double ForceXLLeg = 0;
	double ForceYLLeg = 0;
	double ForceZLLeg = 0;

	double ForceXRLeg = 0;
	double ForceYRLeg = 0;
	double ForceZRLeg = 0;

	double MomentXLLeg = 0;
	double MomentYLLeg = 0;
	double MomentZLLeg = 0;

	double MomentXRLeg = 0;
	double MomentYRLeg = 0;
	double MomentZRLeg = 0;
	
	//20130308 WeiZh
	double SensorCaliBuf[12];

    const char reason = 0;
	unsigned char gBufIn[500];

	//20140218 IMUGL繪圖
    int glcount = 0  ;
	int index = 0   ;
	
	for (int i  = 0 ; i < 500 ; i++)
		gBufIn[i] = 0;
	
	for(int i = 0 ; i < 12 ; i++)
		SensorCaliBuf[i] = 0;

	while(1)
	{
		if (gRenderPMSLife)
		{
			QueryPerformanceCounter(&CurrentTime_Render);	//record current time
			SysTime_Render = (CurrentTime_Render.QuadPart - gStartTimePMS.QuadPart)/Freq_Render;	//progress time	

			if (SysTime_Render >= TickNumber_Sensor*dt)	//收資料 unit:5ms() over a little or equal
			{
				TickNumber_Sensor+=1;	// Timer
///////////////////////////////////////////////////////////////////////////////////資料收值WZ//////////////////////////////////////////////////
				if(LogEncCount==0)
				{
					gForceDataCount ++;
				}
				else if(LogEncCount > 0)
				{
					gForceDataCount = 0;
				}

				//gLQs.tic();//計時
		
				//if (gFlagIMU)
				//{
				//	gIMU->_write(IMUCmd,1);
				//	gIMU->read(gBufIn,79,1000,&reason);
				//	float f = 0;
				//	for(int i = 0;i<18;i++)
				//	{
				//		f = 0;
				//		((BYTE*)(&f))[0] = gBufIn[i*4+4];
				//	    ((BYTE*)(&f))[1] = gBufIn[i*4+3];
				//	    ((BYTE*)(&f))[2] = gBufIn[i*4+2];
				//	    ((BYTE*)(&f))[3] = gBufIn[i*4+1];
				//		gIMUResult[gIMUCount] = f;
				//		gIMUCount ++;
				//	}
				//}

				if (gFlagReadForceSensor == 1)//24V電池  板子號碼 4 5
				{
					 //讀加寫經測試少於2ms
					 gPMS_LLeg->_write(CallStat,5); // < 2ms
					 //Sleep(1);
					 gPMS_LLeg->read(gBufIn,26,1000,&reason);
				}

				tL1 = gBufIn[0]+gBufIn[1]*256; //gV_48Battery
				tL2 = gBufIn[2]+gBufIn[3]*256; //gA_48Battery
				tL3 = gBufIn[4]+gBufIn[5]*256;
				tL4 = gBufIn[6]+gBufIn[7]*256;
				tL5 = gBufIn[8]+gBufIn[9]*256;
				tL6 = gBufIn[10]+gBufIn[11]*256;
				tL7 = gBufIn[12]+gBufIn[13]*256;
				tL8 = gBufIn[14]+gBufIn[15]*256;
				tL9 = gBufIn[16]+gBufIn[17]*256;
				tL10 = gBufIn[18]+gBufIn[19]*256;

				tL1 = (tL1-512)*10/512;
				tL2 = (tL2-512)*10/512;
				tL3 = (tL3-512)*10/512;
				tL4 = (tL4-512)*10/512;
				tL5 = (tL5-512)*10/512;
				tL6 = (tL6-512)*10/512;

				////infrared 20130419 哲軒
				//gKineAll.InfraredLdata1[gKineAll.infraredcount]  = tL7;
				//gKineAll.InfraredLdata2[gKineAll.infraredcount]  = tL8;
				//gKineAll.InfraredLdata3[gKineAll.infraredcount]  = tL9;
				//gKineAll.InfraredLdata4[gKineAll.infraredcount]  = tL10;

				//gKineAll.kalmanfilter(gKineAll.InfraredLdata1[gKineAll.infraredcount],gKineAll.InfraredLkalmanfilterdata1[gKineAll.infraredcount-1],gKineAll.InfraredLkalmanfilterdata1+gKineAll.infraredcount);
				//if (gKineAll.infraredcount > 5){ 
				//gKineAll.smoothfilter(gKineAll.InfraredLdata1+gKineAll.infraredcount,gKineAll.InfraredLfilterdata1+gKineAll.infraredcount); 
				//}

				//gKineAll.kalmanfilter(gKineAll.InfraredLdata2[gKineAll.infraredcount],gKineAll.InfraredLkalmanfilterdata2[gKineAll.infraredcount-1],gKineAll.InfraredLkalmanfilterdata2+gKineAll.infraredcount);
				//if (gKineAll.infraredcount > 5){ 
				//gKineAll.smoothfilter(gKineAll.InfraredLdata2+gKineAll.infraredcount,gKineAll.InfraredLfilterdata2+gKineAll.infraredcount); 
				//}

				//gKineAll.kalmanfilter(gKineAll.InfraredLdata3[gKineAll.infraredcount],gKineAll.InfraredLkalmanfilterdata3[gKineAll.infraredcount-1],gKineAll.InfraredLkalmanfilterdata3+gKineAll.infraredcount); 
				//if (gKineAll.infraredcount > 5){ 
				//	gKineAll.smoothfilter(gKineAll.InfraredLdata3+gKineAll.infraredcount,gKineAll.InfraredLfilterdata3+gKineAll.infraredcount); 
				//}

				//gKineAll.kalmanfilter(gKineAll.InfraredLdata4[gKineAll.infraredcount],gKineAll.InfraredLkalmanfilterdata4[gKineAll.infraredcount-1],gKineAll.InfraredLkalmanfilterdata4+gKineAll.infraredcount);
				//if (gKineAll.infraredcount > 5){ 
				//	gKineAll.smoothfilter(gKineAll.InfraredLdata4+gKineAll.infraredcount,gKineAll.InfraredLfilterdata4+gKineAll.infraredcount); 
				//}
					 
				////5,6電池
				//	gA_24Battery = (tL4*2.0)/1024.0/0.066;//i
				//	gV_24Battery = (tL5*2.5/1024.0+2.5)*6;

				////1,2	4號板
				//	gA_LArm = (tL8*2.0)/1024.0/0.1;//i
				//	gV_LArm = (tL1*2.5/1024+2.5)*6; 

				////3,4	5號板
				//	gA_RArm = (tL2*2.0)/1024.0/0.1;//i
				//	gV_RArm = (tL3*2.5/1024+2.5)*6;

				if (gFlagReadForceSensor == 1)
				{
						// 讀加寫經測試少於2ms
						gPMS_RLeg->_write(CallStat,5); // < 2ms
						//Sleep(1);
						gPMS_RLeg->read(gBufIn,26,1000,&reason);
				}
				
				tR1 = gBufIn[0]+gBufIn[1]*256; //gV_48Battery
				tR2 = gBufIn[2]+gBufIn[3]*256; //gA_48Battery
				tR3 = gBufIn[4]+gBufIn[5]*256;
				tR4 = gBufIn[6]+gBufIn[7]*256;
				tR5 = gBufIn[8]+gBufIn[9]*256;
				tR6 = gBufIn[10]+gBufIn[11]*256;
				tR7 = gBufIn[12]+gBufIn[13]*256;
				tR8 = gBufIn[14]+gBufIn[15]*256;
				tR9 = gBufIn[16]+gBufIn[17]*256;
				tR10 = gBufIn[18]+gBufIn[19]*256;
					 
				tR1 = (tR1-512)*10/512;
				tR2 = (tR2-512)*10/512;
				tR3 = (tR3-512)*10/512;
				tR4 = (tR4-512)*10/512;
				tR5 = (tR5-512)*10/512;
				tR6 = (tR6-512)*10/512;				

				//gKineAll.InfraredRdata1[gKineAll.infraredcount]  = tR7;
				//gKineAll.InfraredRdata2[gKineAll.infraredcount]  = tR8;
				//gKineAll.InfraredRdata3[gKineAll.infraredcount]  = tR9;
				//gKineAll.InfraredRdata4[gKineAll.infraredcount]  = tR10;
				//gKineAll.kalmanfilter(gKineAll.InfraredRdata1[gKineAll.infraredcount],gKineAll.InfraredRkalmanfilterdata1[gKineAll.infraredcount-1],gKineAll.InfraredRkalmanfilterdata1+gKineAll.infraredcount);
				//if (gKineAll.infraredcount > 5){ 
				//gKineAll.smoothfilter(gKineAll.InfraredRdata1+gKineAll.infraredcount,gKineAll.InfraredRfilterdata1+gKineAll.infraredcount); 
				//}
				// 
				//gKineAll.kalmanfilter(gKineAll.InfraredRdata2[gKineAll.infraredcount],gKineAll.InfraredRkalmanfilterdata2[gKineAll.infraredcount-1],gKineAll.InfraredRkalmanfilterdata2+gKineAll.infraredcount);
				//if (gKineAll.infraredcount > 5){ 
				//gKineAll.smoothfilter(gKineAll.InfraredRdata2+gKineAll.infraredcount,gKineAll.InfraredRfilterdata2+gKineAll.infraredcount); 
				//}

				//gKineAll.kalmanfilter(gKineAll.InfraredRdata3[gKineAll.infraredcount],gKineAll.InfraredRkalmanfilterdata3[gKineAll.infraredcount-1],gKineAll.InfraredRkalmanfilterdata3+gKineAll.infraredcount); 
				//if (gKineAll.infraredcount > 5){ 
				//gKineAll.smoothfilter(gKineAll.InfraredRdata3+gKineAll.infraredcount,gKineAll.InfraredRfilterdata3+gKineAll.infraredcount); 
				//}

				//gKineAll.kalmanfilter(gKineAll.InfraredRdata4[gKineAll.infraredcount],gKineAll.InfraredRkalmanfilterdata4[gKineAll.infraredcount-1],gKineAll.InfraredRkalmanfilterdata4+gKineAll.infraredcount);
				//if (gKineAll.infraredcount > 5){ 
				//gKineAll.smoothfilter(gKineAll.InfraredRdata4+gKineAll.infraredcount,gKineAll.InfraredRfilterdata4+gKineAll.infraredcount); 
				//}
				//
				//gKineAll.InfaredSensorData(
				//	gKineAll.InfraredLfilterdata1[gKineAll.infraredcount-1],
				//	gKineAll.InfraredLfilterdata2[gKineAll.infraredcount-1],
				//	gKineAll.InfraredLfilterdata3[gKineAll.infraredcount-1],
				//	gKineAll.InfraredLfilterdata4[gKineAll.infraredcount-1],
				//	gKineAll.InfraredRfilterdata1[gKineAll.infraredcount-1],
				//	gKineAll.InfraredRfilterdata2[gKineAll.infraredcount-1],
				//	gKineAll.InfraredRfilterdata3[gKineAll.infraredcount-1],
				//	gKineAll.InfraredRfilterdata4[gKineAll.infraredcount-1]
				//);

				// //5,6電池
				//	gA_48Battery = (tR5*2.0)/1024.0/0.066;//i
				//	gV_48Battery = (tR4*2.5/1024+2.5)*12;

				////1,2	2號板
				//	gA_LLeg = (tR1*2.0)/1024.0/0.066;//i
				//	gV_LLeg = (tR8*2.5/1024+2.5)*12; 

				////3,4	1號板
				//	gA_RLeg = (tR3*2.0)/1024.0/0.066;//i
				//	gV_RLeg = (tR2*2.5/1024+2.5)*12;

				////force sensor A	LLeg Original
				//	 ForceXLLeg =  (-0.24085*tL1 + 0.01703*tL2 + -0.12106*tL3 + -33.74634*tL4 + 0.12308*tL5 + 33.77477*tL6)/0.430878160557695;
				//	 ForceYLLeg =  (0.31769*tL1 + 40.25149*tL2 + -0.33910*tL3 + -19.38004*tL4 + -0.03536*tL5 + -19.57769*tL6)/0.430878160557695 ;
				//	 ForceZLLeg =  (19.83786*tL1 + -0.07518*tL2 + 19.79808*tL3 + -0.03262*tL4 + 19.92480*tL5 + 0.08090*tL6)/0.161973783103153;
				//	 MomentXLLeg =  (0.19878*tL1 + -0.03675*tL2 + -34.22007*tL3 + 0.15798*tL4 + 34.17283*tL5 + 0.10072*tL6)/8.13428824249133;
				//	 MomentYLLeg =  (39.40601*tL1 + -0.24237*tL2 + -19.76348*tL3 + 0.03235*tL4 + -19.40749*tL5 + -0.00142*tL6)/8.13428824249133;
				//	 MomentZLLeg =  (-0.11503*tL1 + -20.84771*tL2 + -0.26432*tL3 + -20.31898*tL4 + 0.04156*tL5 + -20.38642*tL6)/7.26448104045155;
				
				//force sensor A	LLeg 並轉至機器人座標(前為x左為y) 20130311 WeiZh
				ForceYLLeg =  -(-0.24085*tL1 + 0.01703*tL2 + -0.12106*tL3 + -33.74634*tL4 + 0.12308*tL5 + 33.77477*tL6)/0.430878160557695;
				ForceXLLeg =  -(0.31769*tL1 + 40.25149*tL2 + -0.33910*tL3 + -19.38004*tL4 + -0.03536*tL5 + -19.57769*tL6)/0.430878160557695 ;
				ForceZLLeg =  -(19.83786*tL1 + -0.07518*tL2 + 19.79808*tL3 + -0.03262*tL4 + 19.92480*tL5 + 0.08090*tL6)/0.161973783103153;
				MomentYLLeg =  -(0.19878*tL1 + -0.03675*tL2 + -34.22007*tL3 + 0.15798*tL4 + 34.17283*tL5 + 0.10072*tL6)/8.13428824249133;
				MomentXLLeg =  -(39.40601*tL1 + -0.24237*tL2 + -19.76348*tL3 + 0.03235*tL4 + -19.40749*tL5 + -0.00142*tL6)/8.13428824249133;
				MomentZLLeg =  -(-0.11503*tL1 + -20.84771*tL2 + -0.26432*tL3 + -20.31898*tL4 + 0.04156*tL5 + -20.38642*tL6)/7.26448104045155;

				//gV_LArm = ForceXLLeg;
				//gA_LArm = ForceYLLeg;
				//gV_LLeg = ForceZLLeg;
				//gV_RArm = MomentXLLeg;
				//gA_RArm = MomentYLLeg;
				//gV_RLeg = MomentZLLeg;

				////force sensor B	RLeg Original
				// ForceXRLeg =  (-0.35087*tR1 + 0.05676*tR2 + 0.72867*tR3 + -33.71603*tR4 + 0.03244*tR5 + 34.12740*tR6)/0.430878160557695;
				// ForceYRLeg =  (0.63335*tR1 + 39.75563*tR2 + -0.06558*tR3 + -19.33992*tR4 + 0.08471*tR5 + -19.81289*tR6)/0.430878160557695 ;
				// ForceZRLeg =  (19.83950*tR1 + -0.75912*tR2 + 19.85607*tR3 + -0.57790*tR4 + 19.69887*tR5 + -0.48464*tR6)/0.161973783103153;
				// MomentXRLeg =  (0.07007*tR1 + -0.05137*tR2 + -34.07545*tR3 + 1.03387*tR4 + 34.03219*tR5 + -0.77628*tR6)/8.13428824249133;
				// MomentYRLeg =  (39.33715*tR1 + -1.47193*tR2 + -19.63957*tR3 + 0.53222*tR4 + -19.32459*tR5 + 0.46830*tR6)/8.13428824249133;
				// MomentZRLeg =  (-0.10867*tR1 + -20.10498*tR2 + 0.10060*tR3 + -19.77458*tR4 + 0.02340*tR5 + -20.16681*tR6)/7.26448104045155;

				//force sensor B	RLeg 並轉至機器人座標(前為x左為y) 20130311 WeiZh
				ForceYRLeg =  (-0.35087*tR1 + 0.05676*tR2 + 0.72867*tR3 + -33.71603*tR4 + 0.03244*tR5 + 34.12740*tR6)/0.430878160557695;
				ForceXRLeg =  (0.63335*tR1 + 39.75563*tR2 + -0.06558*tR3 + -19.33992*tR4 + 0.08471*tR5 + -19.81289*tR6)/0.430878160557695 ;
				ForceZRLeg =  -(19.83950*tR1 + -0.75912*tR2 + 19.85607*tR3 + -0.57790*tR4 + 19.69887*tR5 + -0.48464*tR6)/0.161973783103153;
				MomentYRLeg =  (0.07007*tR1 + -0.05137*tR2 + -34.07545*tR3 + 1.03387*tR4 + 34.03219*tR5 + -0.77628*tR6)/8.13428824249133;
				MomentXRLeg =  (39.33715*tR1 + -1.47193*tR2 + -19.63957*tR3 + 0.53222*tR4 + -19.32459*tR5 + 0.46830*tR6)/8.13428824249133;
				MomentZRLeg =  -(-0.10867*tR1 + -20.10498*tR2 + 0.10060*tR3 + -19.77458*tR4 + 0.02340*tR5 + -20.16681*tR6)/7.26448104045155;
				
				//gV_48Battery = ForceXRLeg;
				//gA_48Battery = ForceYRLeg;
				//gT_48Battery = ForceZRLeg;
				//gV_24Battery = MomentXRLeg;
				//gA_24Battery = MomentYRLeg;
				//gT_24Battery = MomentZRLeg;
				//gForceDataLLeg[gForceDataCount] = tR1;//ForceXLLeg;
				//gForceDataRLeg[gForceDataCount] = tL1;//ForceXRLeg;
				//gForceDataCount ++;
				//gForceDataLLeg[gForceDataCount] = tR2;//ForceYLLeg;
				//gForceDataRLeg[gForceDataCount] = tL2;//ForceYRLeg;
				//gForceDataCount ++;
				//gForceDataLLeg[gForceDataCount] = tR3;//ForceZLLeg;
				//gForceDataRLeg[gForceDataCount] = tL3;//ForceZRLeg;
				//gForceDataCount ++;
				//gForceDataLLeg[gForceDataCount] = tR4;//MomentXLLeg;
				//gForceDataRLeg[gForceDataCount] = tL4;//MomentXRLeg;
				//gForceDataCount ++;
				//gForceDataLLeg[gForceDataCount] = tR5;//MomentYLLeg;
				//gForceDataRLeg[gForceDataCount] = tL5;//MomentYRLeg;
				//gForceDataCount ++;
				//gForceDataLLeg[gForceDataCount] = tR6;//MomentZLLeg;
				//gForceDataRLeg[gForceDataCount] = tL6;//MomentZRLeg;
				//gForceDataCount ++;
										
			//直接平移力規值做校正 20130311 WeiZh
				if(LogEncCount==0)	// 傳值之前
				{
					if(gForceDataCount < 100)
					{
						SensorCaliBuf[0] += ForceXLLeg;
						SensorCaliBuf[1] += ForceXRLeg;
						SensorCaliBuf[2] += ForceYLLeg;
						SensorCaliBuf[3] += ForceYRLeg;
						SensorCaliBuf[4] += ForceZLLeg;
						SensorCaliBuf[5] += ForceZRLeg;
						SensorCaliBuf[6] += MomentXLLeg;
						SensorCaliBuf[7] += MomentXRLeg;
						SensorCaliBuf[8] += MomentYLLeg;
						SensorCaliBuf[9] += MomentYRLeg;
						SensorCaliBuf[10] += MomentZLLeg;
						SensorCaliBuf[11] += MomentZRLeg;
					}

					if(gForceDataCount == 100){
						for(int i = 0 ; i < 12 ; i++)
							SensorCaliBuf[i] /= 100; 
					}
				}
					
				if(gForceDataCount > 100 || LogEncCount > 0)	// 傳值之後
				{
					ForceXLLeg -= SensorCaliBuf[0];
					ForceXRLeg -= SensorCaliBuf[1];
					ForceYLLeg -= SensorCaliBuf[2];
					ForceYRLeg -= SensorCaliBuf[3];
					ForceZLLeg -= SensorCaliBuf[4];
					ForceZRLeg -= SensorCaliBuf[5];
					MomentXLLeg -= SensorCaliBuf[6];
					MomentXRLeg -= SensorCaliBuf[7];
					MomentYLLeg -= SensorCaliBuf[8];
					MomentYRLeg -= SensorCaliBuf[9];
					MomentZLLeg -= SensorCaliBuf[10];
					MomentZRLeg -= SensorCaliBuf[11];
				}								

					//tL7 = gKineAll.InfraredLdisdata1[gKineAll.infraredcount-1];//將紅外線濾波後的值傳回  20130329哲軒
					//tL8 = gKineAll.InfraredLdisdata2[gKineAll.infraredcount-1];
					//tL9 = gKineAll.InfraredLdisdata3[gKineAll.infraredcount-1];
					//tL10 = gKineAll.InfraredLdisdata4[gKineAll.infraredcount-1];
					//tLmean = (tL7+tL8+tL9+tL10)*0.25;
					//tLslope = atan( (abs(tL7 -tL9) / 18))  *180 / PI;

					//
					//tR7 = gKineAll.InfraredRdisdata1[gKineAll.infraredcount-1];//將紅外線濾波後的值傳回  20130329哲軒
					//tR8 = gKineAll.InfraredRdisdata2[gKineAll.infraredcount-1];
					//tR9 = gKineAll.InfraredRdisdata3[gKineAll.infraredcount-1];
					//tR10 = gKineAll.InfraredRdisdata4[gKineAll.infraredcount-1];
					//tRmean = (tR7+tR8+tR9+tR10)*0.25;
					//tRslope = atan( (abs(tR7 -tR9) / 18))  *180 / PI;


					// 請注意這樣的傳值方式在中間停下來的時候 或是最後結束時 還是會收值 而且會把值灌到最後一筆
					// 但這樣的好處是值的count會跟著傳出去的值跑
					gForceDataLLeg[(LogEncCount+gForceDataCount)*6] = ForceXLLeg;
					gForceDataRLeg[(LogEncCount+gForceDataCount)*6] = ForceXRLeg;

					gForceDataLLeg[(LogEncCount+gForceDataCount)*6+1] = ForceYLLeg;
					gForceDataRLeg[(LogEncCount+gForceDataCount)*6+1] = ForceYRLeg;

					gForceDataLLeg[(LogEncCount+gForceDataCount)*6+2] = ForceZLLeg;
					gForceDataRLeg[(LogEncCount+gForceDataCount)*6+2] = ForceZRLeg;

					gForceDataLLeg[(LogEncCount+gForceDataCount)*6+3] = MomentXLLeg;
					gForceDataRLeg[(LogEncCount+gForceDataCount)*6+3] = MomentXRLeg;

					gForceDataLLeg[(LogEncCount+gForceDataCount)*6+4] = MomentYLLeg;
					gForceDataRLeg[(LogEncCount+gForceDataCount)*6+4] = MomentYRLeg;

					gForceDataLLeg[(LogEncCount+gForceDataCount)*6+5] = MomentZLLeg;
					gForceDataRLeg[(LogEncCount+gForceDataCount)*6+5] = MomentZRLeg;

					gKineAll.ForceSensorData(gFlagSimulation, gFlagGoPass, gForceDataCount, LogEncCount, gForceDataLLeg, gForceDataRLeg);

									
					//if(gEndOfScript==false)	// 資料去尾
					//{
					////if(gFlagGoPass==0)	// 資料去頭
					//	if(LogEncCount>0)	//20130424 要再改!	
					//	{					
					//		 //紅外線存值

					//		gRangeDataLLeg[gRangeDataCount] = gKineAll.InfaredSensor_dataL[0] ;
					//		gRangeDataRLeg[gRangeDataCount] = gKineAll.InfaredSensor_dataR[0];
					//		gRangeDataCount++;

					//		gRangeDataLLeg[gRangeDataCount] = gKineAll.InfaredSensor_dataL[1] ;
					//		gRangeDataRLeg[gRangeDataCount] = gKineAll.InfaredSensor_dataR[1];
					//		gRangeDataCount++;
					//
					//		gRangeDataLLeg[gRangeDataCount] = gKineAll.InfaredSensor_dataL[2] ;
					//		gRangeDataRLeg[gRangeDataCount] = gKineAll.InfaredSensor_dataR[2];
					//		gRangeDataCount++;
					//
					//		gRangeDataLLeg[gRangeDataCount] = gKineAll.InfaredSensor_dataL[3] ;
					//		gRangeDataRLeg[gRangeDataCount] = gKineAll.InfaredSensor_dataR[3];
					//		gRangeDataCount++;

					//		gRangeDataLLeg[gRangeDataCount] = gKineAll.InfaredSensor_dataL[4] ;
					//		gRangeDataRLeg[gRangeDataCount] = gKineAll.InfaredSensor_dataR[4];
					//		gRangeDataCount++;
					//
					//		gRangeDataLLeg[gRangeDataCount] = gKineAll.InfaredSensor_dataL[5] ;
					//		gRangeDataRLeg[gRangeDataCount] = gKineAll.InfaredSensor_dataR[5];
					//		gRangeDataCount++;
					//	}
					//}
							/*printf("ttt1 = %f %f %f %f \n",gV_48Battery,gA_48Battery,gT_48Battery,gV_24Battery);
							printf("ttt2 = %f %f %f %f \n",gA_24Battery,gT_24Battery,t7,t8);
							printf("ttt3 = %f %f %f %f \n",t9,t10,t11,t12);
							printf("ttt4 = %f %f %f %f \n\n",t13,t14,t15,t16);*/

					 //gLQs.toc();// 計時								
			}//if (SysTime_Render >= TickNumber_Sensor*dt)	
///////////////////////////////////////////////////////////////////////////////////資料收值WZ///////////////////////////////////////////////////					

			if (gRenderKineWorking	== false)	//畫圖
			{
				gRenderPMSWorking = true;

				w_scale_x = gPMSSizeX;
				w_scale_y = gPMSSizeY;

				//QueryPerformanceCounter(&StartTime_Render);
				if (SysTime_Render >= TickNumber_Render/FPS)//unit:1/FPS ms() over a little or equal
				{
					TickNumber_Render+=1;

					#if PMSUpdate
///////////////////////////////////////////////////////////////////////////////////////////////////////畫圖
						IndexBuf = 0;
						IndexBufRev = 0;

						glutSetWindow(gGL_ID_PMS);

						glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

						//glClearColor(1.0f,1.0f,1.0f,1.0f);

						glLoadIdentity();
						glViewport(0,0,gPMSSizeX,gPMSSizeY);
						glMatrixMode(GL_PROJECTION);					
						glLoadIdentity();
						glOrtho(-w_scale_x,w_scale_x,-w_scale_y,w_scale_y,-640,640);

						//double MV48Voltage = 48; // max voltage 3.2*15
						//double mV48Voltage = 37.5; // min voltage 2.5*15
						//double MV24Voltage = 25.6; // max voltage 3.2*15
						//double mV24Voltage = 20; // min voltage 2.5*15

						sprintf_s(msg_title,"Force Measurement");//"Power / Battery Management System");
						glPushMatrix();
							glTranslatef(0.0,0.0,0);		
							glColor3f(1,1,1);

							glRasterPos2f(-310,640-40);
							for ( m = msg_title ; *m!= '\0';m++)
								glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18 ,*m);

						glPopMatrix();

						sprintf_s(time,"Time = %6.3f s ", SysTime_Render);
						glPushMatrix();
							glTranslatef(0.0,0.0,0);		
							glColor3f(1,1,1);

							glRasterPos2f(-310,640-90);
							for ( c = time ; *c!= '\0';c++)
								glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12 ,*c);

						glPopMatrix();

						//sprintf_s(msg_title,"L ForceXLeg  = %.2f ",ForceXLLeg); // 濾波前的值
						sprintf_s(msg_title,"L ForceXLeg  = %.2f ",gKineAll.ForceDataKFL[(LogEncCount+gForceDataCount)*6]);
						//sprintf_s(msg_title,"L Arm V = %.2f V",gV_LArm);
						glPushMatrix();
							glTranslatef(0.0,0.0,0);

							//if (gV_LArm >= MV24Voltage)
								glColor3f(1,1,1);
							//else if (gV_LArm <= mV24Voltage)
							//	glColor3f(1,0,0);
							//else
							//	glColor3f(1,(gV_LArm-mV24Voltage)/(MV24Voltage-mV24Voltage),(gV_LArm-mV24Voltage)/(MV24Voltage-mV24Voltage));

							glRasterPos2f(-310,640-175);
							for ( m = msg_title ; *m!= '\0';m++)
								glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12 ,*m);

						glPopMatrix();

						//glPopMatrix();
				
						//sprintf_s(msg_title,"R ForceXLeg  = %.2f ",ForceXRLeg); // 濾波前的值
						sprintf_s(msg_title,"R ForceXLeg  = %.2f ",gKineAll.ForceDataKFR[(LogEncCount+gForceDataCount)*6]);
						//sprintf_s(msg_title,"L Arm V = %.2f V",gV_LArm);
						glPushMatrix();
							glTranslatef(0.0,0.0,0);

							//if (gV_LArm >= MV24Voltage)
								glColor3f(1,1,1);
							//else if (gV_LArm <= mV24Voltage)
							//	glColor3f(1,0,0);
							//else
							//	glColor3f(1,(gV_LArm-mV24Voltage)/(MV24Voltage-mV24Voltage),(gV_LArm-mV24Voltage)/(MV24Voltage-mV24Voltage));

							glRasterPos2f(20,640-175);
							for ( m = msg_title ; *m!= '\0';m++)
								glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12 ,*m);

						glPopMatrix();

						//sprintf_s(msg_title,"L ForceYLeg  = %.2f ",ForceYLLeg); // 濾波前的值
						sprintf_s(msg_title,"L ForceYLeg  = %.2f ",gKineAll.ForceDataKFL[(LogEncCount+gForceDataCount)*6+1]);
						//sprintf_s(msg_title,"L Arm A = %.2f A",gA_LArm);
						glPushMatrix();
							glTranslatef(0.0,0.0,0);
							//if (gA_LArm >= MA24Current)
								glColor3f(1,1,1);
							//else if (gA_LArm <= mA24Current)
							//	glColor3f(1,0,0);
							//else
							//	glColor3f(1,(MA24Current-gA_LArm)/(MA24Current-mA24Current),(MA24Current-gA_LArm)/(MA24Current-mA24Current));

							glRasterPos2f(-310,640-210);
							for ( m = msg_title ; *m!= '\0';m++)
								glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12 ,*m);

						glPopMatrix();

						//sprintf_s(msg_title,"R ForceYLeg  = %.2f ",ForceYRLeg); // 濾波前的值
						sprintf_s(msg_title,"R ForceYLeg  = %.2f ",gKineAll.ForceDataKFR[(LogEncCount+gForceDataCount)*6+1]);
						//sprintf_s(msg_title,"L Arm A = %.2f A",gA_LArm);
						glPushMatrix();
							glTranslatef(0.0,0.0,0);
							//if (gA_LArm >= MA24Current)
								glColor3f(1,1,1);
							//else if (gA_LArm <= mA24Current)
							//	glColor3f(1,0,0);
							//else
							//	glColor3f(1,(MA24Current-gA_LArm)/(MA24Current-mA24Current),(MA24Current-gA_LArm)/(MA24Current-mA24Current));

							glRasterPos2f(20,640-210);
							for ( m = msg_title ; *m!= '\0';m++)
								glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12 ,*m);

						glPopMatrix();

						//sprintf_s(msg_title,"L ForceZLeg  = %.2f ",ForceZLLeg); // 濾波前的值
						sprintf_s(msg_title,"L ForceZLeg  = %.2f ",gKineAll.ForceDataKFL[(LogEncCount+gForceDataCount)*6+2]);
						//sprintf_s(msg_title,"R Arm V = %.2f V",gV_RArm);
						glPushMatrix();
							glTranslatef(0.0,0.0,0);		
							//if (gV_RArm >= MV24Voltage)
								glColor3f(1,1,1);
							//else if (gV_RArm <= mV24Voltage)
							//	glColor3f(1,0,0);
							//else
							//	glColor3f(1,(gV_RArm-mV24Voltage)/(MV24Voltage-mV24Voltage),(gV_RArm-mV24Voltage)/(MV24Voltage-mV24Voltage));

							glRasterPos2f(-310,640-245);
							//glRasterPos2f(0,640-175);
							for ( m = msg_title ; *m!= '\0';m++)
								glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12 ,*m);

						glPopMatrix();

						//sprintf_s(msg_title,"R ForceZLeg  = %.2f ",ForceZRLeg); // 濾波前的值
						sprintf_s(msg_title,"R ForceZLeg  = %.2f ",gKineAll.ForceDataKFR[(LogEncCount+gForceDataCount)*6+2]);
						//sprintf_s(msg_title,"R Arm V = %.2f V",gV_RArm);
						glPushMatrix();
							glTranslatef(0.0,0.0,0);		
							//if (gV_RArm >= MV24Voltage)
								glColor3f(1,1,1);
							//else if (gV_RArm <= mV24Voltage)
							//	glColor3f(1,0,0);
							//else
							//	glColor3f(1,(gV_RArm-mV24Voltage)/(MV24Voltage-mV24Voltage),(gV_RArm-mV24Voltage)/(MV24Voltage-mV24Voltage));

							glRasterPos2f(20,640-245);
							//glRasterPos2f(0,640-175);
							for ( m = msg_title ; *m!= '\0';m++)
								glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12 ,*m);

						glPopMatrix();

						//sprintf_s(msg_title,"L MomentXLeg  = %.2f ",MomentXLLeg); // 濾波前的值
						sprintf_s(msg_title,"L MomentXLeg  = %.2f ",gKineAll.ForceDataKFL[(LogEncCount+gForceDataCount)*6+3]);
						//sprintf_s(msg_title,"R Arm A = %.2f A",gA_RArm);
						glPushMatrix();
							glTranslatef(0.0,0.0,0);		
							//if (gA_RArm >= MA24Current)
								glColor3f(1,1,1);
							//else if (gA_RArm <= mA24Current)
							//	glColor3f(1,0,0);
							//else
							//	glColor3f(1,(MA24Current-gA_RArm)/(MA24Current-mA24Current),(MA24Current-gA_RArm)/(MA24Current-mA24Current));

							glRasterPos2f(-310,640-300);
							//glRasterPos2f(0,640-210);
							for ( m = msg_title ; *m!= '\0';m++)
								glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12 ,*m);

						glPopMatrix();

						//sprintf_s(msg_title,"R MomentXLeg  = %.2f ",MomentXRLeg); // 濾波前的值
						sprintf_s(msg_title,"R MomentXLeg  = %.2f ",gKineAll.ForceDataKFR[(LogEncCount+gForceDataCount)*6+3]);
						//sprintf_s(msg_title,"R Arm A = %.2f A",gA_RArm);
						glPushMatrix();
							glTranslatef(0.0,0.0,0);		
							//if (gA_RArm >= MA24Current)
								glColor3f(1,1,1);
							//else if (gA_RArm <= mA24Current)
							//	glColor3f(1,0,0);
							//else
							//	glColor3f(1,(MA24Current-gA_RArm)/(MA24Current-mA24Current),(MA24Current-gA_RArm)/(MA24Current-mA24Current));

							glRasterPos2f(20,640-300);
							//glRasterPos2f(0,640-210);
							for ( m = msg_title ; *m!= '\0';m++)
								glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12 ,*m);

						glPopMatrix();

						//sprintf_s(msg_title,"L MomentYLeg  = %.2f ",MomentYLLeg); // 濾波前的值
						sprintf_s(msg_title,"L MomentYLeg  = %.2f ",gKineAll.ForceDataKFL[(LogEncCount+gForceDataCount)*6+4]);
						//sprintf_s(msg_title,"L Leg V = %.2f V",gV_LLeg);
						glPushMatrix();
							glTranslatef(0.0,0.0,0);		
							//if (gV_LLeg >= MV48Voltage)
								glColor3f(1,1,1);
							//else if (gV_LLeg <= mV48Voltage)
							//	glColor3f(1,0,0);
							//else
							//	glColor3f(1,(gV_LLeg-mV48Voltage)/(MV48Voltage-mV48Voltage),(gV_LLeg-mV48Voltage)/(MV48Voltage-mV48Voltage));

							glRasterPos2f(-310,640-335);
							//glRasterPos2f(-310,640-260);
							for ( m = msg_title ; *m!= '\0';m++)
								glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12 ,*m);

						glPopMatrix();

						//sprintf_s(msg_title,"R MomentYLeg  = %.2f ",MomentYRLeg); // 濾波前的值
						sprintf_s(msg_title,"R MomentYLeg  = %.2f ",gKineAll.ForceDataKFR[(LogEncCount+gForceDataCount)*6+4]);
						//sprintf_s(msg_title,"L Leg V = %.2f V",gV_LLeg);
						glPushMatrix();
							glTranslatef(0.0,0.0,0);		
							//if (gV_LLeg >= MV48Voltage)
								glColor3f(1,1,1);
							//else if (gV_LLeg <= mV48Voltage)
							//	glColor3f(1,0,0);
							//else
							//	glColor3f(1,(gV_LLeg-mV48Voltage)/(MV48Voltage-mV48Voltage),(gV_LLeg-mV48Voltage)/(MV48Voltage-mV48Voltage));

							glRasterPos2f(20,640-335);
							//glRasterPos2f(-310,640-260);
							for ( m = msg_title ; *m!= '\0';m++)
								glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12 ,*m);

						glPopMatrix();

						//sprintf_s(msg_title,"L MomentZLeg  = %.2f ",MomentZLLeg); // 濾波前的值
						sprintf_s(msg_title,"L MomentZLeg  = %.2f ",gKineAll.ForceDataKFL[(LogEncCount+gForceDataCount)*6+5]);
						//sprintf_s(msg_title,"L Leg A = %.2f A",gA_LLeg);
						glPushMatrix();
							glTranslatef(0.0,0.0,0);		
							//if (gA_LLeg >= MA48Current)
								glColor3f(1,1,1);
							//else if (gA_LLeg <= mA48Current)
							//	glColor3f(1,0,0);
							//else
							//	glColor3f(1,(MA48Current-gA_LLeg)/(MA48Current-mA48Current),(MA48Current-gA_LLeg)/(MA48Current-mA48Current));

							glRasterPos2f(-310,640-370);
							//glRasterPos2f(-310,640-295);
							for ( m = msg_title ; *m!= '\0';m++)
								glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12 ,*m);

						glPopMatrix();

						//sprintf_s(msg_title,"R MomentZLeg  = %.2f ",MomentZRLeg); // 濾波前的值
						sprintf_s(msg_title,"R MomentZLeg  = %.2f ",gKineAll.ForceDataKFR[(LogEncCount+gForceDataCount)*6+5]);
						//sprintf_s(msg_title,"L Leg A = %.2f A",gA_LLeg);
						glPushMatrix();
							glTranslatef(0.0,0.0,0);		
							//if (gA_LLeg >= MA48Current)
								glColor3f(1,1,1);
							//else if (gA_LLeg <= mA48Current)
							//	glColor3f(1,0,0);
							//else
							//	glColor3f(1,(MA48Current-gA_LLeg)/(MA48Current-mA48Current),(MA48Current-gA_LLeg)/(MA48Current-mA48Current));

							glRasterPos2f(20,640-370);
							//glRasterPos2f(-310,640-295);
							for ( m = msg_title ; *m!= '\0';m++)
								glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12 ,*m);

						glPopMatrix();

						//sprintf_s(msg_title,"R Leg V = %.2f V",gV_RLeg);
						//glPushMatrix();
						//	glTranslatef(0.0,0.0,0);		
						//	if (gV_RLeg >= MV48Voltage)
						//		glColor3f(1,1,1);
						//	else if (gV_RLeg <= mV48Voltage)
						//		glColor3f(1,0,0);
						//	else
						//		glColor3f(1,(gV_RLeg-mV48Voltage)/(MV48Voltage-mV48Voltage),(gV_RLeg-mV48Voltage)/(MV48Voltage-mV48Voltage));


						//	glRasterPos2f(0,640-260);
						//	for ( m = msg_title ; *m!= '\0';m++)
						//		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12 ,*m);

						//glPopMatrix();

						//sprintf_s(msg_title,"R Leg A = %.2f A",gA_RLeg);
						//glPushMatrix();
						//	glTranslatef(0.0,0.0,0);		
						//	if (gA_RLeg >= MA48Current)
						//		glColor3f(1,1,1);
						//	else if (gA_RLeg <= mA48Current)
						//		glColor3f(1,0,0);
						//	else
						//		glColor3f(1,(MA48Current-gA_RLeg)/(MA48Current-mA48Current),(MA48Current-gA_RLeg)/(MA48Current-mA48Current));


						//	glRasterPos2f(0,640-295);
						//	for ( m = msg_title ; *m!= '\0';m++)
						//		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12 ,*m);

						//glPopMatrix();

						//sprintf_s(msg_title,"Head V = %.2f V",gV_Head);
						//glPushMatrix();
						//	glTranslatef(0.0,0.0,0);		
						//	glColor3f(1,1,1);

						//	glRasterPos2f(-310,640-345);
						//	for ( m = msg_title ; *m!= '\0';m++)
						//		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12 ,*m);

						//glPopMatrix();

						//sprintf_s(msg_title,"Head A = %.2f A",gA_Head);
						//glPushMatrix();
						//	glTranslatef(0.0,0.0,0);		
						//	glColor3f(1,1,1);

						//	glRasterPos2f(-310,640-380);
						//	for ( m = msg_title ; *m!= '\0';m++)
						//		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12 ,*m);

						//glPopMatrix();

						//////////////////////////// Green Frame
						glLineWidth(2.0f);
						glBegin(GL_LINE_STRIP);

							glColor3f(0.0f,1.0f,0.0f);
							glVertex3f(-315.0f,490.0f,0.0f);
							glVertex3f(315.0f,490.0f,0.0f);
							glVertex3f(315.0f,240.0f,0.0f);
							glVertex3f(-315.0f,240.0f,0.0f);
							glVertex3f(-315.0f,490.0f,0.0f);

						glEnd();
						glLineWidth(1.0f);

						//////////////////////////// 1st Title
						sprintf_s(msg_title,"Force Value");
						glPushMatrix();
							glTranslatef(0.0,0.0,0);		
							glColor3f(1,1,1);

							glRasterPos2f(-285,640-140);
							for ( m = msg_title ; *m!= '\0';m++)
								glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18 ,*m);

						glPopMatrix();


						//sprintf_s(msg_title,"48V Bat. V = %.2f V",gV_48Battery);
						//glPushMatrix();
						//	glTranslatef(0.0,0.0,0);		
						//	if (gV_48Battery >= MV48Voltage)
						//		glColor3f(1,1,1);
						//	else if (gV_48Battery <= mV48Voltage)
						//		glColor3f(1,0,0);
						//	else
						//		glColor3f(1,(gV_48Battery-mV48Voltage)/(MV48Voltage-mV48Voltage),(gV_48Battery-mV48Voltage)/(MV48Voltage-mV48Voltage));


						//	glRasterPos2f(-310,640-510);
						//	for ( m = msg_title ; *m!= '\0';m++)
						//		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12 ,*m);

						//glPopMatrix();

						//sprintf_s(msg_title,"48V Bat. A = %.2f A",gA_48Battery);
						//glPushMatrix();
						//	glTranslatef(0.0,0.0,0);		
						//	if (gA_48Battery >= MA48Current)
						//		glColor3f(1,1,1);
						//	else if (gA_48Battery <= mA48Current)
						//		glColor3f(1,0,0);
						//	else
						//		glColor3f(1,(MA48Current-gA_48Battery)/(MA48Current-mA48Current),(MA48Current-gA_48Battery)/(MA48Current-mA48Current));


						//	glRasterPos2f(-310,640-545);
						//	for ( m = msg_title ; *m!= '\0';m++)
						//		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12 ,*m);

						//glPopMatrix();

						//sprintf_s(msg_title,"48V Bat. T = %.2f C",gT_48Battery);
						//glPushMatrix();
						//	glTranslatef(0.0,0.0,0);		
						//	glColor3f(1,1,1);

						//	glRasterPos2f(-310,640-580);
						//	for ( m = msg_title ; *m!= '\0';m++)
						//		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12 ,*m);

						//glPopMatrix();

						//sprintf_s(msg_title,"24V Bat. V = %.2f V",gV_24Battery);
						//glPushMatrix();
						//	glTranslatef(0.0,0.0,0);		
						//	if (gV_24Battery >= MV24Voltage)
						//		glColor3f(1,1,1);
						//	else if (gV_24Battery <= mV24Voltage)
						//		glColor3f(1,0,0);
						//	else
						//		glColor3f(1,(gV_24Battery-mV24Voltage)/(MV24Voltage-mV24Voltage),(gV_24Battery-mV24Voltage)/(MV24Voltage-mV24Voltage));


						//	glRasterPos2f(0,640-510);
						//	for ( m = msg_title ; *m!= '\0';m++)
						//		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12 ,*m);

						//glPopMatrix();

						//sprintf_s(msg_title,"24V Bat. A = %.2f A",gA_24Battery);
						//glPushMatrix();
						//	glTranslatef(0.0,0.0,0);		
						//	if (gA_24Battery >= MA24Current)
						//		glColor3f(1,1,1);
						//	else if (gA_24Battery <= mA24Current)
						//		glColor3f(1,0,0);
						//	else
						//		glColor3f(1,(MA24Current-gA_24Battery)/(MA24Current-mA24Current),(MA24Current-gA_24Battery)/(MA24Current-mA24Current));


						//	glRasterPos2f(0,640-545);
						//	for ( m = msg_title ; *m!= '\0';m++)
						//		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12 ,*m);

						//glPopMatrix();

						//sprintf_s(msg_title,"24V Bat. T = %.2f C",	gT_24Battery);
						//glPushMatrix();
						//	glTranslatef(0.0,0.0,0);		
						//	glColor3f(1,1,1);

						//	glRasterPos2f(0,640-580);
						//	for ( m = msg_title ; *m!= '\0';m++)
						//		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12 ,*m);

						//glPopMatrix();


						//glLineWidth(2.0f);
						//glBegin(GL_LINE_STRIP);

						//	glColor3f(0.0f,1.0f,0.0f);
						//	glVertex3f(-315.0f,160.0f,0.0f);
						//	glVertex3f(315.0f,160.0f,0.0f);
						//	glVertex3f(315.0f,40.0f,0.0f);
						//	glVertex3f(-315.0f,40.0f,0.0f);
						//	glVertex3f(-315.0f,160.0f,0.0f);

						//glEnd();
						//glLineWidth(1.0f);

						//sprintf_s(msg_title,"BMS");
						//glPushMatrix();
						//	glTranslatef(0.0,0.0,0);		
						//	glColor3f(1,1,1);

						//	glRasterPos2f(-295,170);
						//	for ( m = msg_title ; *m!= '\0';m++)
						//		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18 ,*m);

						//glPopMatrix();

						/////20130307 WeiZh
						if(gFlagGoPass==0)	// Start按下 CrdAll開始有值
						{
							//左腳COP
							glPushMatrix();
									glTranslatef((float)-1*gKineAll.CoPL[LogEncCount*2+1],(float)(gKineAll.CoPL[LogEncCount*2] - gKineAll.CrdAll->data[21] - 214),0);
									glColor3f(1.0f,0.0f,0.0f);
									glutSolidSphere(5.0f,20,20);				
							glPopMatrix();

							//右腳COP
							glPushMatrix();
									glTranslatef((float)-1*gKineAll.CoPR[LogEncCount*2+1],(float)(gKineAll.CoPR[LogEncCount*2] - gKineAll.CrdAll->data[60] - 214),0);
									glColor3f(1.0f,0.0f,0.0f);
									glutSolidSphere(5.0f,20,20);				
							glPopMatrix();

							//ZMP
							glPushMatrix();
								if(gKineAll.FSensor_forcL[2] < 20000000)	//左腳離地 SSP ZMP不看左腳力規值  畫圖原點移到右腳
									glTranslatef((float)-1*gKineAll.FS_ZMP[LogEncCount*2+1],(float)gKineAll.FS_ZMP[LogEncCount*2] - gKineAll.CrdAll->data[60] - 214,0);
								else if(gKineAll.FSensor_forcR[2] < 20000000)	//右腳離地 SSP ZMP不看右腳力規值 畫圖原點移到左腳
									glTranslatef((float)-1*gKineAll.FS_ZMP[LogEncCount*2+1],(float)gKineAll.FS_ZMP[LogEncCount*2] - gKineAll.CrdAll->data[21] - 214,0);
								else	// DSP 以左腳為準
									glTranslatef((float)-1*gKineAll.FS_ZMP[LogEncCount*2+1],(float)gKineAll.FS_ZMP[LogEncCount*2] - gKineAll.CrdAll->data[21] - 214,0);

									glColor3f(1.0f,2.0f,0.0f);
									glutSolidSphere(5.0f,20,20);				
							glPopMatrix();
						}
						else	// Statr前 CrdAll無值 DSP
						{
							//左腳COP
							glPushMatrix();
									glTranslatef((float)-1*gKineAll.CoPL[1],(float)(gKineAll.CoPL[0] - 214),0);
									glColor3f(1.0f,0.0f,0.0f);
									glutSolidSphere(5.0f,20,20);				
							glPopMatrix();

							//右腳COP
							glPushMatrix();
									glTranslatef((float)-1*gKineAll.CoPR[1],(float)(gKineAll.CoPR[0] - 214),0);
									glColor3f(1.0f,0.0f,0.0f);
									glutSolidSphere(5.0f,20,20);				
							glPopMatrix();

							//ZMP
							glPushMatrix();
									glTranslatef((float)-1*gKineAll.FS_ZMP[1],(float)gKineAll.FS_ZMP[0]-214,0);

									glColor3f(1.0f,2.0f,0.0f);
									glutSolidSphere(5.0f,20,20);				
							glPopMatrix();							
						}

						//LeftLeg
						glLineWidth(2.0f);
						glBegin(GL_LINE_STRIP);

							glColor3f(0.0f,1.0f,0.0f);
							glVertex3f(-80.0f,-214.0f,0.0f);
							glVertex3f(-158.0f,-100.0f,0.0f);
							glVertex3f(-18.0f,-100.0f,0.0f);
							glVertex3f(-18.0f,-310.0f,0.0f);
							glVertex3f(-158.0f,-310.0f,0.0f);
							glVertex3f(-158.0f,-100.0f,0.0f);

						glEnd();
				
						//RightLeg
						glLineWidth(2.0f);
						glBegin(GL_LINE_STRIP);

							glColor3f(0.0f,1.0f,0.0f);
							glVertex3f(80.0f,-214.0f,0.0f);
							glVertex3f(158.0f,-100.0f,0.0f);
							glVertex3f(158.0f,-310.0f,0.0f);
							glVertex3f(18.0f,-310.0f,0.0f);
							glVertex3f(18.0f,-100.0f,0.0f);
							glVertex3f(158.0f,-100.0f,0.0f);

						glEnd();

						glLineWidth(2.0f);
						glBegin(GL_LINE_STRIP);

							glColor3f(0.0f,1.0f,0.0f);
							glVertex3f(-315.0f,-40.0f,0.0f);
							glVertex3f(315.0f,-40.0f,0.0f);
							glVertex3f(315.0f,-370.0f,0.0f);
							glVertex3f(-315.0f,-370.0f,0.0f);
							glVertex3f(-315.0f,-40.0f,0.0f);

						glEnd();
						glLineWidth(1.0f);

						sprintf_s(msg_title,"CoP");
						glPushMatrix();
							glTranslatef(0.0,0.0,0);		
							glColor3f(1,1,1);

							glRasterPos2f(-295,-30);
							for ( m = msg_title ; *m!= '\0';m++)
								glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18 ,*m);

						glPopMatrix();

						//////////////////////////////////// 紅外線 data //////////////////////////////

						   //////////////////////////////////// red Frame
							glLineWidth(2.0f);
							glBegin(GL_LINE_STRIP);
							glColor3f(1,0,0);
									
							glVertex3f(-315.0f,185.0f,0.0f);
							glVertex3f(315.0f,185.0f,0.0f);
							glVertex3f(315.0f,20.0f,0.0f);
							glVertex3f(-315.0f,20.0f,0.0f);
							glVertex3f(-315.0f,185.0f,0.0f);
					
						glEnd();
						glLineWidth(1.0f);
						///////IMU frame ////////

						glLineWidth(2.0f);
							glBegin(GL_LINE_STRIP);
							glColor3f(0,1,0);
									
						    glVertex3f(-315.0f,-420.0f,0.0f);
							glVertex3f(315.0f,-420.0f,0.0f);
							glVertex3f(315.0f,-640.0f,0.0f);
							glVertex3f(-315.0f,-640.0f,0.0f);
							glVertex3f(-315.0f,-420.0f,0.0f);
						glEnd();
						glLineWidth(1.0f);

						////////title /////
						sprintf_s(msg_title,"IMU  angle ");
						glPushMatrix();
							glTranslatef(0.0,0.0,0);		
							glColor3f(1,1,1);

							glRasterPos2f(-285,-407);
							for ( m = msg_title ; *m!= '\0';m++)
								glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18 ,*m);

						glPopMatrix();

                //roll
         
			   glcount = IMU1.count  ; 

			   if (IMUglplot==1){
				   if ( glcount>6000){   //openGL 圖面有限超過6000count重新繪製
				   glcount = IMU1.count-6000 ; 
				   }
			   
				   glLineWidth(2.0f);
				   glBegin(GL_LINE_STRIP);
				   glColor3f(1,0,0);
				   glVertex3f(-315.0f,-480.0f,0.0f);
				   glVertex3f(315.0f,-480.0f,0.0f);					    					
			   
				   glEnd();
				   glBegin(GL_LINE_STRIP);
				   glColor3f(1,0,0);
				   glVertex3f(-315.0f,-600.0f,0.0f);
				   glVertex3f(315.0f,-600.0f,0.0f);
				   glEnd();

				   for(int i=0;i<glcount;i++)
				   {
					   ///roll 			
					   glPushMatrix();
					   glTranslatef(i/10 -315  , IMU1.absanglex[i+6000*(IMU1.count/6000)]*1800/PI -480 ,0);   //將值放大10被顯示於openGL上
					   glColor3f(0.0f,1.0f,0.0f);
					   glutSolidSphere(1.0f,20,20);
					   glPopMatrix();			   
					   ///pitch			
					   glPushMatrix();
					   glTranslatef(i/10 -315  , IMU1.absangley[i+6000*(IMU1.count/6000)]*1800/PI -600 ,0);   
					   glColor3f(0.0f,1.0f,0.0f);
					   glutSolidSphere(1.0f,20,20);
					   glPopMatrix();		
					}
			   }
						////////////////////////////////////   2nd Title
						sprintf_s(msg_title,"Infrared Sensor Value");
						glPushMatrix();
							glTranslatef(0.0,0.0,0);		
							glColor3f(1,0,0);

							glRasterPos2f(-285,640-440);
							for ( m = msg_title ; *m!= '\0';m++)
								glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18 ,*m);

						glPopMatrix();

						 ////////////////////////////////////data

						sprintf_s(msg_title,"infaredL1  =  %.2f ",tL7);
				
						glPushMatrix();
						glTranslatef(0.0,0.0,0);
						glColor3f(1,1,1);
						glRasterPos2f(-310,640-475);
							for ( m = msg_title ; *m!= '\0';m++)
								glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12 ,*m);

						glPopMatrix();

				
						sprintf_s(msg_title,"infaredR1  =  %.2f ",tR7);
				
						glPushMatrix();
							glTranslatef(0.0,0.0,0);
							glColor3f(1,1,1);
							glRasterPos2f(20,640-475);
							for ( m = msg_title ; *m!= '\0';m++)
								glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12 ,*m);

						glPopMatrix();



						sprintf_s(msg_title,"infaredL2  =  %.2f ",tL8);
						glPushMatrix();
						glTranslatef(0.0,0.0,0);
						glColor3f(1,1,1);
						glRasterPos2f(-310,640-510);
							for ( m = msg_title ; *m!= '\0';m++)
								glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12 ,*m);

						glPopMatrix();

						sprintf_s(msg_title,"infaredR2  =  %.2f ",tR8);
						glPushMatrix();
						glTranslatef(0.0,0.0,0);
						glColor3f(1,1,1);
						glRasterPos2f(20,640-510);
							for ( m = msg_title ; *m!= '\0';m++)
								glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12 ,*m);

						glPopMatrix();


						sprintf_s(msg_title,"infaredL3  =  %.2f ",tL9);
						glPushMatrix();
						glTranslatef(0.0,0.0,0);		
						glColor3f(1,1,1);
						glRasterPos2f(-310,640-545);
			
							for ( m = msg_title ; *m!= '\0';m++)
								glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12 ,*m);

						glPopMatrix();


				
						sprintf_s(msg_title,"infaredR3  =  %.2f ",tR9);
						glPushMatrix();
						glTranslatef(0.0,0.0,0);		
						glColor3f(1,1,1);
						glRasterPos2f(20,640-545);
			
							for ( m = msg_title ; *m!= '\0';m++)
								glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12 ,*m);

						glPopMatrix();


						sprintf_s(msg_title,"infaredL4  =  %.2f ",tL10);
						glPushMatrix();
						glTranslatef(0.0,0.0,0);		
						glColor3f(1,1,1);
						glRasterPos2f(-310,640-580);
			
							for ( m = msg_title ; *m!= '\0';m++)
								glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12 ,*m);

						glPopMatrix();



						sprintf_s(msg_title,"infaredR4  =  %.2f ",tR10);
						glPushMatrix();
						glTranslatef(0.0,0.0,0);		
						glColor3f(1,1,1);
						glRasterPos2f(20,640-580);
			
							for ( m = msg_title ; *m!= '\0';m++)
								glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12 ,*m);

						glPopMatrix();


				
						sprintf_s(msg_title,"L mean    =  %.2f ",tLmean);
						glPushMatrix();
						glTranslatef(0.0,0.0,0);		
						glColor3f(1,1,1);
						glRasterPos2f(-310,640-615);
			
							for ( m = msg_title ; *m!= '\0';m++)
								glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12 ,*m);

						glPopMatrix();

				
						sprintf_s(msg_title,"R mean     =  %.2f ",tRmean);
						glPushMatrix();
						glTranslatef(0.0,0.0,0);		
						glColor3f(1,1,1);
						glRasterPos2f(20,640-615);
			
							for ( m = msg_title ; *m!= '\0';m++)
								glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12 ,*m);

						glPopMatrix();


				
						


						//glBegin(GL_LINES);

						//	glColor3f(0.0f,1.0f,0.0f);
						//	glVertex3f(0.0f,0.0f,0.0f);
						//	glVertex3f(50.0f,0.0f,0.0f);

						//	glColor3f(0.0f,0.0f,1.0f);
						//	glVertex3f(0.0f,0.0f,0.0f);
						//	glVertex3f(0.0f,50.0f,0.0f);

						//	glColor3f(1.0f,0.0f,0.0f);
						//	glVertex3f(0.0f,0.0f,0.0f);
						//	glVertex3f(0.0f,0.0f,50.0f);

						//glEnd();


						//glFlush();
						glutSwapBuffers();
		        
						//printf("Render time cost = %f \n",SysTime_Render);////
						
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////畫圖WZ// PMSUpdate
					
					#else
						gRenderKineWorking = false;
					#endif	//PMSUpdate

					gRenderPMSWorking = false;
				}
			}


		} //gRenderPMSLife
		else
		{
			break;
		}
	}
}

void CRobotAllDlg::OnBnClickedButton9() // PMS/BMS save
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 儲存電源管理，力規資料，紅外線資料之按鈕
	// 實驗結束後記得按下以儲存資料
	******************************************************************/

	fstream force_data;
	cout<<"Saving Data...";
	if (gFlagReadForceSensor == 1)
	{
		if(gFlagSimulation != CppSimu){	// 在CppSimu模式下點開力規checkbox要從濾波開始 所以就不存原始力規txt 
			force_data.open("force_data_LLeg.txt",ios::out | ios::trunc);
			for(int i = 0;i < (LogEncCount+gForceDataCount+1)*6 ; i++){
				force_data<<gForceDataLLeg[i]<<"\t";
				if(i%6 == 5){
					force_data<<endl;
				}
			}
			force_data.close();

			force_data.open("force_data_RLeg.txt",ios::out | ios::trunc);
			for(int i = 0;i < (LogEncCount+gForceDataCount+1)*6 ; i++){
				force_data<<gForceDataRLeg[i]<<"\t";
				if(i%6 == 5){
					force_data<<endl;
				}
			}
			force_data.close();
		}

		force_data.open("force_data_LLegKF.txt",ios::out | ios::trunc);
		for(int i = 0;i < (LogEncCount+gForceDataCount+1)*6 ; i++){
			force_data<<gKineAll.ForceDataKFL[i]<<"\t";
			if(i%6 == 5){
				force_data<<endl;
			}
		}
		force_data.close();

		force_data.open("force_data_RLegKF.txt",ios::out | ios::trunc);
		for(int i = 0;i < (LogEncCount+gForceDataCount+1)*6 ; i++){
			force_data<<gKineAll.ForceDataKFR[i]<<"\t";
			if(i%6 == 5){
				force_data<<endl;
			}
		}
		force_data.close();
		
		force_data.open("force_data_LLegCOP.txt",ios::out | ios::trunc);
		for(int i = 0;i < (LogEncCount+1)*2 ; i++){
			force_data<<gKineAll.CoPL[i]<<"\t";
			if(i%2 == 1){
				force_data<<endl;
			}
		}
		force_data.close();
		
		force_data.open("force_data_RLegCOP.txt",ios::out | ios::trunc);
		for(int i = 0;i < (LogEncCount+1)*2 ; i++){
			force_data<<gKineAll.CoPR[i]<<"\t";
			if(i%2 == 1){
				force_data<<endl;
			}
		}
		force_data.close();

		force_data.open("force_data_ZMP.txt",ios::out | ios::trunc);
		for(int i = 0;i < (LogEncCount+1)*2 ; i++){
			force_data<<gKineAll.FS_ZMP[i]<<"     ";
			if(i%2 == 1){
				force_data<<endl;
			}
		}
		force_data.close();	
	}

    /////////紀錄feedback 軌跡//////////////////////
	if (gFlagIMU == 1)
	{
		force_data.open("ALfeedbackroll.txt",ios::out);
		for(int i = 0;i < 10000 ; i++){
			force_data<<IMU1.Lfeedbackroll[i]<<endl;
		}
		force_data.close();

		force_data.open("ALfeedbackpitch.txt",ios::out);
		for(int i = 0;i < 10000 ; i++){
			force_data<<IMU1.Lfeedbackpitch[i]<<endl;	
		}
		force_data.close();

		force_data.open("ARfeedbackroll.txt",ios::out);
		for(int i = 0;i < 10000 ; i++){
			force_data<<IMU1.Rfeedbackroll[i]<<endl;
		}
		force_data.close();

		force_data.open("ARfeedbackpitch.txt",ios::out);
		for(int i = 0;i < 10000 ; i++){
			force_data<<IMU1.Rfeedbackpitch[i]<<endl;
		}
		force_data.close();		
	}

		//////////////////紀錄cogestimate data /////////////////////// 前面補0避免無法匯入matlab
	#if cogestimate
		force_data.close();
		force_data.open("ACogstatelateral.txt",ios::out);
		for (int i = 0 ;i<6 ;i++){
		force_data<<0<<endl;
		}
		
		for(int i = 6;i < COGestimate.Cogstatelateral.size()  ; i++){
			force_data<<COGestimate.Cogstatelateral[i]<<endl; 
		}	
		force_data.close();

		force_data.open("ADcogstatelateral.txt",ios::out);
		for (int i = 0 ;i<6 ;i++){
		force_data<<0<<endl;
		}
		
		for(int i = 6;i < COGestimate.Dcogstatelateral.size() ; i++){
			force_data<<COGestimate.Dcogstatelateral[i]<<endl;
		}
		force_data.close();
		

		force_data.open(" AZMPstatelateral.txt",ios::out);
		for (int i = 0 ;i<6 ;i++){
		force_data<<0<<endl;
		}
		
		for(int i = 6;i < COGestimate.zmpstatelateral.size() ; i++){
			force_data<<COGestimate.zmpstatelateral[i]<<endl;
		}
		force_data.close();

		force_data.open("ACogstatesaggital.txt",ios::out);
		for (int i = 0 ;i<6 ;i++){
		force_data<<0<<endl;
		}
		
		for(int i = 6;i < COGestimate.Cogstatesaggital.size()  ; i++){
			force_data<<COGestimate.Cogstatesaggital[i]<<endl; 
		}	
		force_data.close();

		force_data.open("ADcogstatesaggital.txt",ios::out);
		for (int i = 0 ;i<6 ;i++){
		force_data<<0<<endl;
		}
		
		for(int i = 6;i < COGestimate.Dcogstatesaggital.size() ; i++){
			force_data<<COGestimate.Dcogstatesaggital[i]<<endl;
		}
		force_data.close();
		
		force_data.open(" AZMPstatesaggital.txt",ios::out);
		for (int i = 0 ;i<6 ;i++){
			force_data<<0<<endl;
		}
		
		for(int i = 6;i < COGestimate.zmpstatesaggital.size() ; i++){
			force_data<<COGestimate.zmpstatesaggital[i]<<endl;
		}
		force_data.close();

		force_data.open(" AExternalforcesaggital.txt",ios::out);
		for (int i = 0 ;i<6 ;i++){
		force_data<<0<<endl;
		}
		
		for(int i = 6;i < COGestimate.externalforcesaggital.size() ; i++){
			force_data<<COGestimate.externalforcesaggital[i]<<endl;
		}
		force_data.close();

		
		force_data.open(" AExternalforcelateral.txt",ios::out);
		for (int i = 0 ;i<6 ;i++){
		force_data<<0<<endl;
		}
		
		for(int i = 6;i < COGestimate.externalforcelateral.size() ; i++){
			force_data<<COGestimate.externalforcelateral[i]<<endl;
		}
		force_data.close();
		
		force_data.open("A finalanglex.txt",ios::out);
		for(int i = 0;i < 10000  ; i++){
			force_data<<IMU1.finalanglex[i]<<endl; 
		}	
		force_data.close();
	
		force_data.open("A finalangley.txt",ios::out);
		for(int i = 0;i < 10000  ; i++){
			force_data<<IMU1.finalangley[i]<<endl; 
		}	
		force_data.close();
		
		force_data.open("A velx.txt",ios::out);
		for(int i = 0;i < 10000  ; i++){
			force_data<<IMU1.velx[i]<<endl; 
		}	
		force_data.close();
		
		force_data.open("A vely.txt",ios::out);
		for(int i = 0;i < 10000  ; i++){
			force_data<<IMU1.vely[i]<<endl; 
		}	
		force_data.close();
		
		force_data.open("A accelx.txt",ios::out);
		for(int i = 0;i < 10000  ; i++){
			force_data<<IMU1.accelx[i]<<endl; 
		}	
		force_data.close();
		
		force_data.open("A accely.txt",ios::out);
		for(int i = 0;i < 10000  ; i++){
			force_data<<IMU1.accely[i]<<endl; 
		}	
		force_data.close();
	#endif

		//force_data.open("position_data.txt",ios::out);
		//for(int i = 0;i < PosDataCount;i++)
		//{
		//	force_data<<PosCalZMP[i]<<"\t";
		//	if(i%6 == 5)
		//	{
		//		force_data<<endl;
		//	}
		//}
		//force_data.close();

		
		//fstream  Fx ;
		//Fx.open("infaredL_data.txt",ios::app);	
		//for(int i = 0 ; i < infraedbuffersize ; i++)	
		//	Fx <<  gKineAll.InfraredLdata1[i] <<"\t"<< gKineAll.InfraredLdata2[i]<<"\t"<<gKineAll.InfraredLdata3[i]<<"\t"<< gKineAll.InfraredLdata4[i]<<endl;
		//Fx.close();

		//Fx.open("infaredL_filterdata.txt",ios::app);	
		//for(int i = 0 ; i < infraedbuffersize ; i++)	
		//	Fx <<  gKineAll.InfraredLfilterdata1[i]<<"\t"<< gKineAll.InfraredLfilterdata2[i]<< "\t"<<gKineAll.InfraredLfilterdata3[i]<<"\t"<< gKineAll.InfraredLfilterdata4[i]<<endl;
		//Fx.close();

		//Fx.open("infaredR_filterdata.txt",ios::app);	
		//for(int i = 0 ; i < infraedbuffersize ; i++)	
		//	Fx <<  gKineAll.InfraredRfilterdata1[i]<<"\t"<< gKineAll.InfraredRfilterdata2[i]<< "\t"<<gKineAll.InfraredRfilterdata3[i]<<"\t"<< gKineAll.InfraredRfilterdata4[i]<<endl;
		//Fx.close();


		//Fx.open("infaredL_kalmanfilterdata.txt",ios::app);	
		//for(int i = 0 ; i < infraedbuffersize ; i++)	
		//	Fx <<  gKineAll.InfraredLkalmanfilterdata1[i]<<"\t"<< gKineAll.InfraredLfilterdata2[i]<< "\t"<<gKineAll.InfraredLfilterdata3[i]<<"\t"<< gKineAll.InfraredLfilterdata4[i]<<endl;
		//Fx.close();

		//Fx.open("infaredL_dis.txt",ios::app);	
		//for(int i = 0 ; i < gKineAll.infraredcount ; i++)	
		//	Fx <<  gKineAll.InfraredLdisdata1[i]<<"\t"<< gKineAll.InfraredLdisdata2[i]<<"\t"<<gKineAll.InfraredLdisdata3[i]<<"\t"<< gKineAll.InfraredLdisdata4[i]<<endl;
		//Fx.close();

		//Fx.open("infaredR_dis.txt",ios::app);	
		//for(int i = 0 ; i < gKineAll.infraredcount ; i++)	
		//	Fx <<  gKineAll.InfraredRdisdata1[i]<<"\t"<< gKineAll.InfraredRdisdata2[i]<<"\t"<<gKineAll.InfraredRdisdata3[i]<<"\t"<< gKineAll.InfraredRdisdata4[i]<<endl;
		//Fx.close();

		//force_data.open("range_data_LLeg.txt",ios::out);
		//for(int i = 0;i < gRangeDataCount;i++)
		//{
		//	force_data<<gRangeDataLLeg[i]<<"     ";
		//	if(i%4 == 3)
		//	{
		//		force_data<<endl;
		//	}
		//}
		//force_data.close();

		//force_data.open("range_data_RLeg.txt",ios::out);
		//for(int i = 0;i < gRangeDataCount;i++)
		//{
		//	force_data<<gRangeDataRLeg[i]<<"     ";
		//	if(i%4 == 3)
		//	{
		//		force_data<<endl;
		//	}
		//}
		//force_data.close();	

	cout<<"Save Complete!"<<"\n";
	if (gFlagSimulation == ADAMSSimu){
		OnBnClickedOk();	// end button 釋放Thread
	}
}


void CRobotAllDlg::OnBnClickedCheck1()
{
	/******************************************************************
	input: void
	output: void

	Note:

	// 切換到ADAMS模式  to CheckBox3
	// COM Part不會發送訊號
	// 本程式最開始Run預設為C++ 模擬模式 要利用這三個CheckBox來切換

	******************************************************************/
	CheckSimu.SetCheck(0);
	CheckExp.SetCheck(0);
	CheckADAMS.SetCheck(1);
	CheckSimu.EnableWindow(0);
	CheckExp.EnableWindow(0);
	CheckADAMS.EnableWindow(1);
	gFlagSimulation = 1; // ADAMS simu
	gFlagBoostSimu = 0; // Boost模式只有在 C++模式可以進行
	CheckEmergentStop.EnableWindow(1);

	// clear the status and flag of reading encoder, infrared and 6axis force sensor
	gFlagReadEncoder = 0;
	gFlagReadForceSensor = 0;
	//gFlagSkinModule = 0;
	gFlagInfrared = 0;
	gFlagIMU = 0;
	CheckEncoder.SetCheck(0);
	CheckForceSensor.SetCheck(0);
	CheckEncoder.EnableWindow(0);
	CheckForceSensor.EnableWindow(1);
	CheckInfrared.SetCheck(0);
	CheckInfrared.EnableWindow(0);
	CheckBoost.SetCheck(0);
	CheckBoost.EnableWindow(0);
	CheckIMU.SetCheck(0);
	CheckIMU.EnableWindow(0);
	mButton_Init_PBMS.EnableWindow(false);
	CheckManualMode.EnableWindow(false);
	CheckArmCtrl.EnableWindow(false);
	CheckHandCtrl.EnableWindow(false);
}


void CRobotAllDlg::OnBnClickedCheck2()
{
	/******************************************************************
	input: void
	output: void

	Note:

	// 切換到模擬模式  to CheckBox1
	// COM Part不會發送訊號
	// 本程式最開始Run預設為C++ 模擬模式 要利用這三個CheckBox來切換

	******************************************************************/


	CheckSimu.SetCheck(1);
	CheckExp.SetCheck(0);
	CheckADAMS.SetCheck(0);
	CheckSimu.EnableWindow(1);
	CheckExp.EnableWindow(0);
	CheckADAMS.EnableWindow(0);
	gFlagSimulation = 0;
	gFlagBoostSimu = 0; // Boost mode can only be used in C++ simu mode
	CheckEmergentStop.EnableWindow(0);

	// clear the status and flag of reading encoder, infrared and 6axis force sensor
	gFlagReadEncoder = 0;
	gFlagReadForceSensor = 0;
	//gFlagSkinModule = 0;
	gFlagInfrared = 0;
	gFlagIMU = 0;
	CheckEncoder.SetCheck(0);
	CheckForceSensor.SetCheck(0);
	CheckEncoder.EnableWindow(0);
	CheckForceSensor.EnableWindow(1);
	CheckInfrared.SetCheck(0);
	CheckInfrared.EnableWindow(0); 
	CheckBoost.SetCheck(0);
	CheckBoost.EnableWindow(1); // Boost mode can only be used in C++ simu mode
	CheckIMU.SetCheck(0);
	CheckIMU.EnableWindow(0);
	mButton_Init_PBMS.EnableWindow(false);
	CheckManualMode.EnableWindow(false);

	CheckArmCtrl.EnableWindow(false);
	CheckHandCtrl.EnableWindow(false);

}


void CRobotAllDlg::OnBnClickedCheck3()
{
	/******************************************************************
	input: void
	output: void

	Note:

	// 切換到實驗模式 to CheckBox2
	// COM Part會發送訊號
	// 本程式最開始Run預設為C++ 模擬模式 要利用這三個CheckBox來切換

	******************************************************************/

	CheckSimu.SetCheck(0);
	CheckExp.SetCheck(1);
	CheckADAMS.SetCheck(0);
	CheckSimu.EnableWindow(0);
	CheckExp.EnableWindow(1);
	CheckADAMS.EnableWindow(0);
	gFlagSimulation = 2;
	gFlagBoostSimu = 0;// Boost mode can only be used in C++ simu mode
	//______________________________Slongz
	gFlagReadEncoder = 1;
	CheckEncoder.SetCheck(1);
	CheckEncoder.EnableWindow(0);
	mButton_Init_PBMS.EnableWindow(true);
	
	CheckEmergentStop.EnableWindow(true);
	CheckManualMode.EnableWindow(true);
	CheckArmCtrl.EnableWindow(true);
	CheckHandCtrl.EnableWindow(true);
	//______________________________Slongz	
	//CheckEncoder.EnableWindow(1);
	CheckForceSensor.EnableWindow(1);
	CheckInfrared.EnableWindow(1);
	CheckBoost.SetCheck(0);
	CheckBoost.EnableWindow(0); // Boost mode can only be used in C++ simu mode

	CheckIMU.SetCheck(0);
	CheckIMU.EnableWindow(1);
	mButton_Init_PBMS.EnableWindow(true);

}


void gInitTurnLeft(void)
{
	/******************************************************************
	input: void
	output: void

	Note:

	// 初始化機器人行為為轉彎(左)
	// 腳步設定 地形設定 機器人步行參數 設定於此
	// 相似的函數有 gInitTurnLeft gInitWalkStraight gInitStepHere gInitStair gInitSquat gInitDownStair

	******************************************************************/

	gNumOfStep = 13; // 包含初始、轉換與preivew的總步數
	gCOGDown = 30; // 機器人要蹲的量
	gKineAll.FlagSumoMode = 0; // 非單腳站立模式

	double x_val_zmp = 80; // ZMP 在左右方向的位置
	bool PNx = 0;
	double y_step_zmp = 200; 
	double TurnRadius = 300; // 機器人迴轉半徑

	double distance2L = 200.0; // 機器人雙腳距離 轉彎的時候怕踩到自己
	
	gKineAll.StepHeight[0] = 0;
	for (int i = 1 ; i < gNumOfStep+30 ; i++)
		gKineAll.StepHeight[i] = 25;

	gKineAll.selSupport[0] = 2;
	for (int i = 1 ; i < gNumOfStep+30 ; i+=2)
	{
		gKineAll.selSupport[i] = 1;
		gKineAll.selSupport[i+1] = 0;
	}
	for (int i = gNumOfStep-4 ; i < gNumOfStep+30 ; i++)
		gKineAll.selSupport[i] = 2; // double support and prepare to stop

	double AngChange = 0.20;

	gLRotAngZ[0] = 0+gLAngZWorld;
	gLRotAngZ[1] = 0+gLAngZWorld;
	gLRotAngZ[2] = gAngChange*1+gLAngZWorld;
	gLRotAngZ[3] = gAngChange*1+gLAngZWorld;
	cout<<gLRotAngZ[0]<<endl;
	cout<<gLRotAngZ[1]<<endl;
	cout<<gLRotAngZ[2]<<endl;
	cout<<gLRotAngZ[3]<<endl;

	for (int i = 2 ; i < gNumOfStep+30 ; i++)
	{
		gLRotAngZ[i*2] = gAngChange*(2.0*i-1)+gLAngZWorld;
		gLRotAngZ[i*2+1] = gAngChange*(2.0*i-1)+gLAngZWorld;
		if(i<gNumOfStep)
		{
		cout<<gLRotAngZ[i*2]<<endl;
		cout<<gLRotAngZ[i*2+1]<<endl;
		}
	}

	cout<<endl;
	gRRotAngZ[0] = gRAngZWorld;
	for (int i = 0 ; i < gNumOfStep+30 ; i++)
	{
		gRRotAngZ[i*2+1] = 2.0*gAngChange*i+gRAngZWorld;
		gRRotAngZ[i*2+2] = 2.0*gAngChange*i+gRAngZWorld;
		if(i<gNumOfStep)
		{
		cout<<gRRotAngZ[i*2+1]<<endl;
		cout<<gRRotAngZ[i*2+2]<<endl;
		}
	}


	gLLInitZMPFB = -80*sin(gRRotAngZ[0]);
	gRLInitZMPFB = 80*sin(gRRotAngZ[0]);
	gLLInitZMPLR = 80*cos(gRRotAngZ[0]);
	gRLInitZMPLR = -80*cos(gRRotAngZ[0]);


	// 左右 
	//gFstpX[0] = 0;
	//gFstpX[1] = -x_val_zmp;
	gFstpX[0] = 0;
	gFstpX[1] = -x_val_zmp*cos(gRRotAngZ[1]);
	gFstpX[2] = (distance2L/2.0)*cos(gLRotAngZ[0]) - (TurnRadius-distance2L/2.0)*(cos(gLRotAngZ[2])-cos(gLRotAngZ[0]));
	for (int i = 2 ; i < gNumOfStep+30 ; i++)
	{
		gFstpX[i*2-1] = gFstpX[(i-1)*2-1]-(TurnRadius+distance2L/2.0)*(cos(gRRotAngZ[i*2-1])-cos(gRRotAngZ[(i-1)*2-1]));
		gFstpX[i*2] = gFstpX[(i-1)*2]-(TurnRadius-distance2L/2.0)*(cos(gLRotAngZ[i*2])-cos(gLRotAngZ[(i-1)*2]));
	}

	//gFstpY[0] = 0;
	//gFstpY[1] = 0;
	gFstpY[0] = 0;
	gFstpY[1] = distance2L/2.0*sin(gRRotAngZ[1]);
	gFstpY[2] = -(distance2L/2.0)*sin(gLRotAngZ[0]) + (TurnRadius-distance2L/2.0)*(sin(gLRotAngZ[2])-sin(gLRotAngZ[0]));
	for (int i = 2 ; i < gNumOfStep+30 ; i++)
	{
		//gFstpY[i*2] = (TurnRadius-distance2L/2.0)*sin(gLRotAngZ[i*2]);
		//gFstpY[i*2+1] = gFstpY[i*2]+distance2L*sin(gLRotAngZ[i*2]);
		gFstpY[i*2-1] = gFstpY[(i-1)*2-1]+(TurnRadius+distance2L/2.0)*(sin(gRRotAngZ[i*2-1])-sin(gRRotAngZ[(i-1)*2-1]));
		gFstpY[i*2] = gFstpY[(i-1)*2]+(TurnRadius-distance2L/2.0)*(sin(gLRotAngZ[i*2])-sin(gLRotAngZ[(i-1)*2]));
	}


	double X_Cen;
	double Y_Cen;
	if (gKineAll.selSupport[gNumOfStep-5] == 1) // right support
	{
		X_Cen = gFstpX[gNumOfStep-5] + 80.0*sin(gRRotAngZ[gNumOfStep-5]+3.1415926/2.0);
		Y_Cen = gFstpY[gNumOfStep-5] + 80.0*cos(gRRotAngZ[gNumOfStep-5]+3.1415926/2.0);
	}
	else if (gKineAll.selSupport[gNumOfStep-5] == 0) // left support
	{
		X_Cen = gFstpX[gNumOfStep-5] - 80.0*sin(gLRotAngZ[gNumOfStep-5]+3.1415926/2.0);
		Y_Cen = gFstpY[gNumOfStep-5] - 80.0*cos(gLRotAngZ[gNumOfStep-5]+3.1415926/2.0);
	}

	for (int i = gNumOfStep-4 ; i < gNumOfStep+30 ; i++)
	{
		gFstpX[i] = X_Cen;
		gFstpY[i] = Y_Cen;
	}


	for (int i = 0 ; i < gNumOfStep+30 ; i++)
		gGroundHeight[i] = 0.0;

}

void gInitTurnRight(int StepNum, double TurnRadius, double TurnAngle, double Dist2L)
{
	/******************************************************************
	input: StepNum, 總步數， TurnRadius 迴轉半徑， TurnAngle 旋轉角度， Dist2L 轉彎時雙腳距離
	output: void

	Note:

	// 初始化機器人行為為轉彎
	// 腳步設定 地形設定 機器人步行參數 設定於此
	// 相似的函數有 gInitTurnLeft gInitTurnRight gInitWalkStraight gInitStepHere gInitStair gInitSquat

	******************************************************************/

	gNumOfStep = StepNum; // 包含初始、轉換與preivew的總步數
	gCOGDown = 13; // 蹲的量 盡量少 膝蓋比較直 但假若機器人singular 就要調大一點

	gKineAll.FlagSumoMode = 0; // 單腳站立模式

	double x_val_zmp = 80;
	bool PNx = 0;
	double y_step_zmp = 200;

	double distance2L = Dist2L;

	gKineAll.StepHeight[0] = 0;
	for (int i = 1 ; i < gNumOfStep+30 ; i++)
		gKineAll.StepHeight[i] = 20;

	gKineAll.selSupport[0] = 2;
	for (int i = 1 ; i < gNumOfStep+30 ; i+=2)
	{
		gKineAll.selSupport[i] = 0;
		gKineAll.selSupport[i+1] = 1;
	}
	for (int i = gNumOfStep-4 ; i < gNumOfStep+30 ; i++)
		gKineAll.selSupport[i] = 2; // double support and prepare to stop

	double AngChange = (-TurnAngle)/double(StepNum-6)/180.0*3.1415926;
	gAngChange = AngChange;

	//double AngChange = -0.1844;

	gRRotAngZ[0] = 0+gRAngZWorld;
	gRRotAngZ[1] = 0+gRAngZWorld;
	gRRotAngZ[2] = AngChange*1+gRAngZWorld;
	gRRotAngZ[3] = AngChange*1+gRAngZWorld;
	for (int i = 2 ; i < gNumOfStep+30 ; i++)
	{
		gRRotAngZ[i*2] = AngChange*(2.0*i-1)+gRAngZWorld;
		gRRotAngZ[i*2+1] = AngChange*(2.0*i-1)+gRAngZWorld;
	}

	gLRotAngZ[0] = gLAngZWorld;
	for (int i = 0 ; i < gNumOfStep+30 ; i++)
	{
		gLRotAngZ[i*2+1] = 2.0*AngChange*i+gLAngZWorld;
		gLRotAngZ[i*2+2] = 2.0*AngChange*i+gLAngZWorld;
	}

	gLLInitZMPFB = -80*sin(gRRotAngZ[0]);
	gRLInitZMPFB = 80*sin(gRRotAngZ[0]);
	gLLInitZMPLR = 80*cos(gRRotAngZ[0]);
	gRLInitZMPLR = -80*cos(gRRotAngZ[0]);


	// 左右 
	//gFstpX[0] = 0;
	//gFstpX[1] = -x_val_zmp;
	gFstpX[0] = 0;
	gFstpX[1] = x_val_zmp*cos(gRRotAngZ[1]);
	gFstpX[2] = -(distance2L/2.0)*cos(gLRotAngZ[0]) + (TurnRadius-distance2L/2.0)*(cos(gLRotAngZ[2])-cos(gLRotAngZ[0]));
	for (int i = 2 ; i < gNumOfStep+30 ; i++)
	{
		gFstpX[i*2-1] = gFstpX[(i-1)*2-1]+(TurnRadius+distance2L/2.0)*(cos(gRRotAngZ[i*2-1])-cos(gRRotAngZ[(i-1)*2-1]));
		gFstpX[i*2] = gFstpX[(i-1)*2]+(TurnRadius-distance2L/2.0)*(cos(gLRotAngZ[i*2])-cos(gLRotAngZ[(i-1)*2]));
	}

	//gFstpY[0] = 0;
	//gFstpY[1] = 0;
	gFstpY[0] = 0;
	gFstpY[1] = -x_val_zmp*sin(gRRotAngZ[1]);
	gFstpY[2] = (distance2L/2.0)*sin(gLRotAngZ[0]) - (TurnRadius-distance2L/2.0)*(sin(gLRotAngZ[2])-sin(gLRotAngZ[0]));
	for (int i = 2 ; i < gNumOfStep+30 ; i++)
	{
		gFstpY[i*2-1] = gFstpY[(i-1)*2-1]-(TurnRadius+distance2L/2.0)*(sin(gRRotAngZ[i*2-1])-sin(gRRotAngZ[(i-1)*2-1]));
		gFstpY[i*2] = gFstpY[(i-1)*2]-(TurnRadius-distance2L/2.0)*(sin(gLRotAngZ[i*2])-sin(gLRotAngZ[(i-1)*2]));
	}


	double X_Cen;
	double Y_Cen;
	if (gKineAll.selSupport[gNumOfStep-5] == 1) // right support
	{
		X_Cen = gFstpX[gNumOfStep-5] + 80.0*sin(gRRotAngZ[gNumOfStep-5]+3.1415926/2.0);
		Y_Cen = gFstpY[gNumOfStep-5] + 80.0*cos(gRRotAngZ[gNumOfStep-5]+3.1415926/2.0);
	}
	else if (gKineAll.selSupport[gNumOfStep-5] == 0) // left support
	{
		X_Cen = gFstpX[gNumOfStep-5] - 80.0*sin(gLRotAngZ[gNumOfStep-5]+3.1415926/2.0);
		Y_Cen = gFstpY[gNumOfStep-5] - 80.0*cos(gLRotAngZ[gNumOfStep-5]+3.1415926/2.0);
	}

	for (int i = gNumOfStep-4 ; i < gNumOfStep+30 ; i++)
	{
		gFstpX[i] = X_Cen;
		gFstpY[i] = Y_Cen;
	}


	for (int i = 0 ; i < gNumOfStep+30 ; i++)
		gGroundHeight[i] = 0.0;

}
void gInitQCCDWalkStraight(int StepInput, double StepLength)
{
	/******************************************************************
	input: StepInput 總步數 因為preview以及去頭去尾 所以要減六 可以得到總跨步數, StepLength 步距，可正可負
	output: void

	Note:

	// 初始化機器人行為為直走
	// 腳步設定 地形設定 機器人步行參數 設定於此
	// 相似的函數有 gInitTurnLeft gInitWalkStraight gInitStepHere gInitStair gInitSquat

	******************************************************************/
	gNumOfStep = StepInput; // 包含初始、轉換與preivew的總步數
	gCOGUp =-2.00; // 愈少膝蓋愈直 比較像人 也比較省力  //gCOGUp =-3.00; // 愈少膝蓋愈直 比較像人 也比較省力
	#if ConstantCOGMode
		gCOGDown =13; // 愈少膝蓋愈直 比較像人 也比較省力
	#else
		gCOGDown =12; // 愈少膝蓋愈直 比較像人 也比較省力
	#endif

	gKineAll.FlagSumoMode = 0;

	double x_val_zmp = 80; // ZMP 左右方向位置
	bool PNx = 0;
	double y_step_zmp = StepLength;
	double TurnRadius = 300;


	double distance2L = 160.0;

	gKineAll.StepHeight[0] = 0;
	for (int i = 1 ; i < gNumOfStep+30 ; i++)
	{
		#if ConstantCOGMode
			gKineAll.StepHeight[i] = 25;
		#else
			gKineAll.StepHeight[i] = 30;//25
		#endif
	}
	gKineAll.selSupport[0] = 2;
	for (int i = 1 ; i < gNumOfStep+30 ; i+=2)
	{
		gKineAll.selSupport[i] = 1;
		gKineAll.selSupport[i+1] = 0;
	}
	for (int i = gNumOfStep-4 ; i < 4000 ; i++)
		gKineAll.selSupport[i] = 2; // double support and prepare to stop


	double AngChange = 0.0;

	for (int i = 0 ; i < gNumOfStep+30 ; i++)
	{
		gLRotAngZ[i*2] = AngChange*i + gLAngZWorld;
		gLRotAngZ[i*2+1] = AngChange*i + gLAngZWorld;
	}

	gRRotAngZ[0] = gRAngZWorld;
	for (int i = 0 ; i < gNumOfStep+30 ; i++)
	{
		gRRotAngZ[i*2+1] = AngChange*i + gRAngZWorld;
		gRRotAngZ[i*2+2] = AngChange*i + gRAngZWorld;
	}

	gLLInitZMPFB = -80*sin(gRRotAngZ[0]);
	gRLInitZMPFB = 80*sin(gRRotAngZ[0]);
	gLLInitZMPLR = 80*cos(gRRotAngZ[0]);
	gRLInitZMPLR = -80*cos(gRRotAngZ[0]);

	double BodyDir = (gLAngZWorld+gRAngZWorld)/2.0;
	double StrideX = y_step_zmp*sin(BodyDir);
	double StrideY = y_step_zmp*cos(BodyDir);


	// 左右 mn 
	gFstpX[0] = 0;
	gFstpX[1] = -x_val_zmp*cos(gRRotAngZ[1]);
	gFstpX[2] = x_val_zmp*cos(gRRotAngZ[0])+StrideX/2.0;
	for (int i = 2 ; i < gNumOfStep ; i++)
	{
		gFstpX[i*2-1] = gFstpX[(i-1)*2-1] + StrideX;
		gFstpX[i*2] = gFstpX[(i-1)*2] + StrideX;
	}

	if (gKineAll.selSupport[gNumOfStep-4] == 0) // right support
	{
		gFstpX[gNumOfStep-4] = gFstpX[gNumOfStep-5] + 160*cos((gLRotAngZ[gNumOfStep-4]+gRRotAngZ[gNumOfStep-4])/2.0);
	}
	else
	{
		gFstpX[gNumOfStep-4] = gFstpX[gNumOfStep-5] - 160*cos((gLRotAngZ[gNumOfStep-4]+gRRotAngZ[gNumOfStep-4])/2.0);
	}

	gFstpX[gNumOfStep-4] = (gFstpX[gNumOfStep-4]+gFstpX[gNumOfStep-5])/2.0;

	for (int i = gNumOfStep - 3 ; i < gNumOfStep+4 ; i++)
	{
		gFstpX[i] = gFstpX[gNumOfStep-4];
	}

	//gFstpY[0] = 0;
	//gFstpY[1] = distance2L/2.0*sin(gRRotAngZ[2]);
	//gFstpY[2] = -distance2L/2.0*sin(gLRotAngZ[0])+StrideY/2.0;
	//for (int i = 1 ; i < gNumOfStep ; i++)
	//{
	//	gFstpY[i*2+1] = gFstpY[i*2-1]+StrideY;
	//	gFstpY[i*2+2] = gFstpY[i*2]+StrideY;
	//}

	////////////////////////後退搭配"-300.00"
	//gFstpY[0] = 0;
	//gFstpY[1] = distance2L/2.0*sin(gRRotAngZ[2]);
	//gFstpY[2] = -distance2L/2.0*sin(gLRotAngZ[0])-287/2;//StrideY/2.0;//295/2;//
	//gFstpY[3] = gFstpY[1]-299.5;
	//gFstpY[4] = gFstpY[2]-300.00;
	//gFstpY[5] = gFstpY[3]-300.00;
	//gFstpY[6] = gFstpY[4]+StrideY;
	//for (int i = 2 ; i < gNumOfStep ; i++)
	//{
	//	gFstpY[i*2+1] = gFstpY[ i*2-1]+StrideY;
	//	gFstpY[i*2+2] = gFstpY[i*2]+StrideY;
	//}

	#if ConstantCOGMode
		gFstpY[0] = 0;  
		gFstpY[1] = distance2L/2.0*sin(gRRotAngZ[2]);  
		gFstpY[2] = -distance2L/2.0*sin(gLRotAngZ[0])+WalkRatio*299.5/2;//StrideY/2.0;//295/2;//
		gFstpY[3] = gFstpY[1]+WalkRatio*311.75;
		gFstpY[4] = gFstpY[2]+StrideY-0.5;

		for (int i = 2 ; i < gNumOfStep ; i++)
		{
			gFstpY[i*2+1] = gFstpY[ i*2-1]+StrideY;
			gFstpY[i*2+2] = gFstpY[i*2]+StrideY;
		}
	#else
		gFstpY[0] = 0;
		gFstpY[1] = distance2L/2.0*sin(gRRotAngZ[2]);
		gFstpY[2] = -distance2L/2.0*sin(gLRotAngZ[0])+WalkRatio*StrideY/2;//StrideY/2.0;//295/2;//
		gFstpY[3] = gFstpY[1]+WalkRatio*StrideY;
		gFstpY[4] = gFstpY[2]+WalkRatio*StrideY;
		gFstpY[5] = gFstpY[3]+WalkRatio*StrideY;
		gFstpY[6] = gFstpY[4]+WalkRatio*StrideY;

		for (int i = 3 ; i < gNumOfStep ; i++)
		{
			gFstpY[i*2+1] = gFstpY[ i*2-1]+StrideY;
			gFstpY[i*2+2] = gFstpY[i*2]+StrideY;
		}
		//gFstpY[0] = 0;
		//gFstpY[1] = distance2L/2.0*sin(gRRotAngZ[2]);
		//gFstpY[2] = -distance2L/2.0*sin(gLRotAngZ[0])+WalkRatio*299.5/2;//StrideY/2.0;//295/2;//
		//gFstpY[3] = gFstpY[1]+WalkRatio*315;
		//gFstpY[4] = gFstpY[2]+WalkRatio*322.75;
		//gFstpY[5] = gFstpY[3]+WalkRatio*322.5;
		//gFstpY[6] = gFstpY[4]+WalkRatio*StrideY;

		//for (int i = 3 ; i < gNumOfStep ; i++)
		//{
		//	gFstpY[i*2+1] = gFstpY[ i*2-1]+StrideY;
		//	gFstpY[i*2+2] = gFstpY[i*2]+StrideY;
		//}
	#endif

	if (gKineAll.selSupport[gNumOfStep-4] == 0) // right support
	{
		gFstpY[gNumOfStep-4] = gFstpY[gNumOfStep-5] - 160*sin((gLRotAngZ[gNumOfStep-4]+gRRotAngZ[gNumOfStep-4])/2.0);
	}
	else
	{
		gFstpY[gNumOfStep-4] = gFstpY[gNumOfStep-5] + 160*sin((gLRotAngZ[gNumOfStep-4]+gRRotAngZ[gNumOfStep-4])/2.0);
	}

	gFstpY[gNumOfStep-4] = (gFstpY[gNumOfStep-4]+gFstpY[gNumOfStep-5])/2.0;

	for (int i = gNumOfStep - 3 ; i < gNumOfStep+5 ; i++)
	{
		gFstpY[i] = gFstpY[gNumOfStep-4];
	}

	for (int i = 0 ; i < gNumOfStep+25 ; i++)
		gGroundHeight[i] = 0.0;
}

void gInitWalkStraight(int StepInput, double StepLength)
{
	/******************************************************************
	input: StepInput 總步數 因為preview以及去頭去尾 所以要減六 可以得到總跨步數, StepLength 步距，可正可負
	output: void

	Note:

	// 初始化機器人行為為直走
	// 腳步設定 地形設定 機器人步行參數 設定於此
	// 相似的函數有 gInitTurnLeft gInitWalkStraight gInitStepHere gInitStair gInitSquat gInitDownStair

	******************************************************************/
	gNumOfStep = StepInput; // 包含初始、轉換與preivew的總步數
	gCOGDown = 9;//36; // 愈少膝蓋愈直 比較像人 也比較省力
	if(gNumOfStep==7)//Specific Ver. Warn.
		checkonestep = 1;
	gKineAll.FlagSumoMode = 0;
	

	double x_val_zmp = 80; // ZMP 左右方向位置
	bool PNx = 0;
	double y_step_zmp =  StepLength;;
	double TurnRadius = 300;


	double distance2L = 160.0;

	gKineAll.StepHeight[0] = 0;
	for (int i = 1 ; i < gNumOfStep+30 ; i++)
		gKineAll.StepHeight[i] = 20;

	gKineAll.selSupport[0] = 2;
	for (int i = 1 ; i < gNumOfStep+30 ; i+=2)
	{
		gKineAll.selSupport[i] = 1;
		gKineAll.selSupport[i+1] = 0;
	}
	for (int i = gNumOfStep-4 ; i < 4000 ; i++)
		gKineAll.selSupport[i] = 2; // double support and prepare to stop


	double AngChange = 0.0;

	for (int i = 0 ; i < gNumOfStep+30 ; i++)
	{
		gLRotAngZ[i*2] = AngChange*i + gLAngZWorld;
		gLRotAngZ[i*2+1] = AngChange*i + gLAngZWorld;
	}

	gRRotAngZ[0] = gRAngZWorld;
	for (int i = 0 ; i < gNumOfStep+30 ; i++)
	{
		gRRotAngZ[i*2+1] = AngChange*i + gRAngZWorld;
		gRRotAngZ[i*2+2] = AngChange*i + gRAngZWorld;
	}

	gLLInitZMPFB = -80*sin(gRRotAngZ[0]); // ZMP Initial 
	gRLInitZMPFB = 80*sin(gRRotAngZ[0]);
	gLLInitZMPLR = 80*cos(gRRotAngZ[0]); // ZMP Initial Swing 軌跡會用到 換腳要改
	gRLInitZMPLR = -80*cos(gRRotAngZ[0]);

	double BodyDir = (gLAngZWorld+gRAngZWorld)/2.0;
	double StrideX = y_step_zmp*sin(BodyDir);
	double StrideY = y_step_zmp*cos(BodyDir);


	// 左右 
	gFstpX[0] = 0;
	gFstpX[1] = -x_val_zmp*cos(gRRotAngZ[1]); // ZMP Initial Swing 軌跡會用到 換腳要改
	gFstpX[2] = x_val_zmp*cos(gRRotAngZ[0])+StrideX/2.0; // ZMP Initial Swing 軌跡會用到 換腳要改
	for (int i = 2 ; i < gNumOfStep ; i++)
	{
		gFstpX[i*2-1] = gFstpX[(i-1)*2-1] + StrideX;
		gFstpX[i*2] = gFstpX[(i-1)*2] + StrideX;
	}

	if (gKineAll.selSupport[gNumOfStep-4] == 0) // right support
	{
		gFstpX[gNumOfStep-4] = gFstpX[gNumOfStep-5] + 160*cos((gLRotAngZ[gNumOfStep-4]+gRRotAngZ[gNumOfStep-4])/2.0);
	}
	else
	{
		gFstpX[gNumOfStep-4] = gFstpX[gNumOfStep-5] - 160*cos((gLRotAngZ[gNumOfStep-4]+gRRotAngZ[gNumOfStep-4])/2.0);
	}

	gFstpX[gNumOfStep-4] = (gFstpX[gNumOfStep-4]+gFstpX[gNumOfStep-5])/2.0;

	for (int i = gNumOfStep - 3 ; i < gNumOfStep+4 ; i++)
	{
		gFstpX[i] = gFstpX[gNumOfStep-4];
	}

	if(gNumOfStep ==7)
	{
      for (int i = gNumOfStep-4; i < 4000 ; i++)
		gFstpX[i] = 0.0;

	    gFstpX[3] =gFstpX[1];
	}
	//else
	//{
	//	for (int i = gNumOfStep-4; i < 4000 ; i++)
	//	gFstpX[i] = 0.0;
	//}



	gFstpY[0] = 0;
	gFstpY[1] = distance2L/2.0*sin(gRRotAngZ[2]);
	gFstpY[2] = -distance2L/2.0*sin(gLRotAngZ[0])+StrideY/2.0;
	//gFstpY[0] = 0;
	//gFstpY[1] = 0;
	//gFstpY[2] = 50;
	//gFstpY[3] = 260;
	//	gFstpY[4] = 310;
	//		gFstpY[5] = 520;
	//			gFstpY[6] = 570;
	//				gFstpY[7] = 570;
	for (int i = 1 ; i < gNumOfStep ; i++)
	{
		gFstpY[i*2+1] = gFstpY[i*2-1]+StrideY;
		gFstpY[i*2+2] = gFstpY[i*2]+StrideY;
	}

	if (gKineAll.selSupport[gNumOfStep-4] == 0) // right support
	{
		gFstpY[gNumOfStep-4] = gFstpY[gNumOfStep-5] - 160*sin((gLRotAngZ[gNumOfStep-4]+gRRotAngZ[gNumOfStep-4])/2.0);
	}
	else
	{
		gFstpY[gNumOfStep-4] = gFstpY[gNumOfStep-5] + 160*sin((gLRotAngZ[gNumOfStep-4]+gRRotAngZ[gNumOfStep-4])/2.0);
	}

	gFstpY[gNumOfStep-4] = (gFstpY[gNumOfStep-4]+gFstpY[gNumOfStep-5])/2.0;

	for (int i = gNumOfStep - 3 ; i < gNumOfStep+5 ; i++)
	{
		gFstpY[i] = gFstpY[gNumOfStep-4];
	}

	for (int i = 0 ; i < gNumOfStep+25 ; i++)
		gGroundHeight[i] = 0.0;
}

void gInitStayMode(double StayTime)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 機器人停在原地不動

	******************************************************************/
	//gNumOfStep = StepInput;
	//gCOGDown = 13;

	gNoMotionTime = StayTime;
	gKineAll.FlagStayMode = 1;
	gKineAll.FlagStayBreak = 0;
	//gKineAll.FlagSumoMode = 0;

	//double x_val_zmp = 80;
	//bool PNx = 0;
	//double y_step_zmp = StepLength;
	//double TurnRadius = 300;


	//double distance2L = 160.0;

	//gKineAll.StepHeight[0] = 0;
	//for (int i = 1 ; i < gNumOfStep+30 ; i++)
	//	gKineAll.StepHeight[i] = 25;

	//gKineAll.selSupport[0] = 2;
	//for (int i = 1 ; i < gNumOfStep+30 ; i+=2)
	//{
	//	gKineAll.selSupport[i] = 1;
	//	gKineAll.selSupport[i+1] = 0;
	//}
	//for (int i = gNumOfStep-4 ; i < 4000 ; i++)
	//	gKineAll.selSupport[i] = 2; // double support and prepare to stop


	//double AngChange = 0.0;

	//for (int i = 0 ; i < gNumOfStep+30 ; i++)
	//{
	//	gLRotAngZ[i*2] = AngChange*i + gLAngZWorld;
	//	gLRotAngZ[i*2+1] = AngChange*i + gLAngZWorld;
	//}

	//gRRotAngZ[0] = gRAngZWorld;
	//for (int i = 0 ; i < gNumOfStep+30 ; i++)
	//{
	//	gRRotAngZ[i*2+1] = AngChange*i + gRAngZWorld;
	//	gRRotAngZ[i*2+2] = AngChange*i + gRAngZWorld;
	//}

	//gLLInitZMPFB = -80*sin(gRRotAngZ[0]);
	//gRLInitZMPFB = 80*sin(gRRotAngZ[0]);
	//gLLInitZMPLR = 80*cos(gRRotAngZ[0]);
	//gRLInitZMPLR = -80*cos(gRRotAngZ[0]);

	//double BodyDir = (gLAngZWorld+gRAngZWorld)/2.0;
	//double StrideX = y_step_zmp*sin(BodyDir);
	//double StrideY = y_step_zmp*cos(BodyDir);


	//// 左右 
	//gFstpX[0] = 0;
	//gFstpX[1] = -x_val_zmp*cos(gRRotAngZ[1]);
	//gFstpX[2] = x_val_zmp*cos(gRRotAngZ[0])+StrideX/2.0;
	//for (int i = 2 ; i < gNumOfStep ; i++)
	//{
	//	gFstpX[i*2-1] = gFstpX[(i-1)*2-1] + StrideX;
	//	gFstpX[i*2] = gFstpX[(i-1)*2] + StrideX;
	//}

	//if (gKineAll.selSupport[gNumOfStep-4] == 0) // right support
	//{
	//	gFstpX[gNumOfStep-4] = gFstpX[gNumOfStep-5] + 160*cos((gLRotAngZ[gNumOfStep-4]+gRRotAngZ[gNumOfStep-4])/2.0);
	//}
	//else
	//{
	//	gFstpX[gNumOfStep-4] = gFstpX[gNumOfStep-5] - 160*cos((gLRotAngZ[gNumOfStep-4]+gRRotAngZ[gNumOfStep-4])/2.0);
	//}

	//gFstpX[gNumOfStep-4] = (gFstpX[gNumOfStep-4]+gFstpX[gNumOfStep-5])/2.0;

	//for (int i = gNumOfStep - 3 ; i < gNumOfStep+4 ; i++)
	//{
	//	gFstpX[i] = gFstpX[gNumOfStep-4];
	//}

	//gFstpY[0] = 0;
	//gFstpY[1] = distance2L/2.0*sin(gRRotAngZ[2]);
	//gFstpY[2] = -distance2L/2.0*sin(gLRotAngZ[0])+StrideY/2.0;
	//for (int i = 1 ; i < gNumOfStep ; i++)
	//{
	//	gFstpY[i*2+1] = gFstpY[i*2-1]+StrideY;
	//	gFstpY[i*2+2] = gFstpY[i*2]+StrideY;
	//}

	//if (gKineAll.selSupport[gNumOfStep-4] == 0) // right support
	//{
	//	gFstpY[gNumOfStep-4] = gFstpY[gNumOfStep-5] - 160*sin((gLRotAngZ[gNumOfStep-4]+gRRotAngZ[gNumOfStep-4])/2.0);
	//}
	//else
	//{
	//	gFstpY[gNumOfStep-4] = gFstpY[gNumOfStep-5] + 160*sin((gLRotAngZ[gNumOfStep-4]+gRRotAngZ[gNumOfStep-4])/2.0);
	//}

	//gFstpY[gNumOfStep-4] = (gFstpY[gNumOfStep-4]+gFstpY[gNumOfStep-5])/2.0;

	//for (int i = gNumOfStep - 3 ; i < gNumOfStep+5 ; i++)
	//{
	//	gFstpY[i] = gFstpY[gNumOfStep-4];
	//}

	//for (int i = 0 ; i < gNumOfStep+25 ; i++)
	//	gGroundHeight[i] = 0.0;

}
void gInitStaySquatMode(double StayTime) 
{
	/******************************************************************
	input: void
	output: void
	original variables: gKineAll.FlagStaySquatMode;
	                    gKineAll.FlagStayBreak;
	Editor: DORA

	Note:
	// 機器人停在原地不動，但會初始彎腳與歸HOME，motioncontrolthread中第一段第三段可照常使用，但可使用FlagStayBreak觸發進入第三段，主要有對第二段作修改
	// 採用讀取鍵盤緩衝區的方式處理，機器人執行時在console window按鍵p即可，以後可修改加入預先清除鍵盤緩衝區功能，讓觸發只在適當的scenario內啟動
	// 若要以鍵盤觸發為主，請將StayTime設定較長

	******************************************************************/
	gNoMotionTime = StayTime;
	gKineAll.FlagStayMode = 0;
	gKineAll.FlagStaySquatMode = 1;
	gKineAll.FlagStayBreak = 0;
}

void gInitStepHere(void)
{
	/******************************************************************
	input: void
	output: void

	Note:

	// 初始化機器人行為為原地踏步
	// 腳步設定 地形設定 機器人步行參數 設定於此
	// 相似的函數有 gInitTurnLeft gInitWalkStraight gInitStepHere gInitStair gInitSquat gInitDownStair

	******************************************************************/
	gNumOfStep = 8; // 包含初始、轉換與preivew的總步數
	gCOGDown = 10; // 膝蓋蠻直的
	//gCOGDown = 40; // 膝蓋小彎

	gKineAll.FlagSumoMode = 0;

	double x_val_zmp = 80;
	bool PNx = 0;
	double y_step_zmp = 0;
	double TurnRadius = 300;

	double distance2L = 160;

	gKineAll.StepHeight[0] = 0;
	for (int i = 1 ; i < gNumOfStep+10 ; i++)
		gKineAll.StepHeight[i] = 25;

	gKineAll.selSupport[0] = 2;
	for (int i = 1 ; i < gNumOfStep+10 ; i+=2)
	{
		gKineAll.selSupport[i] = 1;
		gKineAll.selSupport[i+1] = 0;
	}

	for (int i = gNumOfStep-4 ; i < gNumOfStep+20 ; i++)
		gKineAll.selSupport[i] = 2; // double support and prepare to stop


	double AngChange = 0.0;

	for (int i = 0 ; i < gNumOfStep+20 ; i++)
	{
		gLRotAngZ[i*2] = AngChange*i+gLAngZWorld;
		gLRotAngZ[i*2+1] = AngChange*i+gLAngZWorld;
	}

	gRRotAngZ[0] = 0.0 + (gLAngZWorld+gRAngZWorld)/2.0;
	for (int i = 0 ; i < gNumOfStep+20 ; i++)
	{
		gRRotAngZ[i*2+1] = AngChange*i+gRAngZWorld;
		gRRotAngZ[i*2+2] = AngChange*i+gRAngZWorld;
	}

	gLLInitZMPFB = -80*sin(gRRotAngZ[0]);
	gRLInitZMPFB = 80*sin(gRRotAngZ[0]);
	gLLInitZMPLR = 80*cos(gRRotAngZ[0]);
	gRLInitZMPLR = -80*cos(gRRotAngZ[0]);

	// 左右 
	gFstpX[0] = 0;
	gFstpX[1] = -x_val_zmp*cos(gRRotAngZ[1]);
	for (int i = 1 ; i < gNumOfStep+20 ; i++)
	{
		gFstpX[i*2] = distance2L/2.0*cos(gRRotAngZ[i*2]);
		gFstpX[i*2+1] = -distance2L/2.0*cos(gLRotAngZ[i*2+1]);
	}

	for (int i = gNumOfStep - 4 ; i < gNumOfStep+20 ; i++)
	{
		gFstpX[i] = 0.0;
	}

	gFstpY[0] = 0;
	gFstpY[1] = distance2L/2.0*sin(gRRotAngZ[2]);
	gFstpY[2] = -distance2L/2.0*sin(gLRotAngZ[2]);
	for (int i = 1 ; i < 30 ; i++)
	{
		gFstpY[i*2+1] = distance2L/2.0*sin(gRRotAngZ[i*2]);
		gFstpY[i*2+2] = -distance2L/2.0*sin(gLRotAngZ[i*2+1]);
	}

	for (int i = 0 ; i < gNumOfStep+20 ; i++)
		gGroundHeight[i] = 0.0;

}

void gInitSideWalk(int Direction)
{
	/******************************************************************
	input: void
	output: void

	Note:

	// 初始化機器人行為為原地踏步
	// 腳步設定 地形設定 機器人步行參數 設定於此
	// 相似的函數有 gInitTurnLeft gInitWalkStraight gInitStepHere gInitStair gInitSquat

	******************************************************************/
	gNumOfStep = 9; // 包含初始、轉換與preivew的總步數
	gCOGDown = 20; // 蹲下的量
	//gCOGDown = 40; // 膝蓋小彎

	gKineAll.FlagSumoMode = 0;

	double x_val_zmp = 80;
	bool PNx = 0;
	double y_step_zmp = 0;

	double SideWalkZMP = 45;

	double TurnRadius = 300;

	double distance2L = 160;

	gKineAll.StepHeight[0] = 0;
	for (int i = 1 ; i < gNumOfStep+10 ; i++)
		gKineAll.StepHeight[i] = 20;


	if (Direction == DirectionLeft)
	{
		gKineAll.selSupport[0] = 2;
		for (int i = 1 ; i < gNumOfStep+10 ; i+=2)
		{
			gKineAll.selSupport[i] = 1;
			gKineAll.selSupport[i+1] = 0;
		}
	}
	else if (Direction == DirectionRight)
	{
		gKineAll.selSupport[0] = 2;
		for (int i = 1 ; i < gNumOfStep+10 ; i+=2)
		{
			gKineAll.selSupport[i] = 0;
			gKineAll.selSupport[i+1] = 1;
		}
	}

	for (int i = gNumOfStep-4 ; i < gNumOfStep+20 ; i++)
		gKineAll.selSupport[i] = 2; // double support and prepare to stop


	double AngChange = 0.0;

	for (int i = 0 ; i < gNumOfStep+20 ; i++)
	{
		gLRotAngZ[i*2] = AngChange*i+gLAngZWorld;
		gLRotAngZ[i*2+1] = AngChange*i+gLAngZWorld;
	}

	gRRotAngZ[0] = 0.0 + (gLAngZWorld+gRAngZWorld)/2.0;
	for (int i = 0 ; i < gNumOfStep+20 ; i++)
	{
		gRRotAngZ[i*2+1] = AngChange*i+gRAngZWorld;
		gRRotAngZ[i*2+2] = AngChange*i+gRAngZWorld;
	}

	gLLInitZMPFB = -80*sin(gRRotAngZ[0]);
	gRLInitZMPFB = 80*sin(gRRotAngZ[0]);
	gLLInitZMPLR = 80*cos(gRRotAngZ[0]);
	gRLInitZMPLR = -80*cos(gRRotAngZ[0]);


	if (Direction == DirectionLeft)
	{
		// 左右 
		gFstpX[0] = 0;
		gFstpX[1] = -x_val_zmp*cos(gRRotAngZ[1]);

		gFstpX[2] = (distance2L/2.0+SideWalkZMP)*cos(gRRotAngZ[2]);
		gFstpX[3] = -(distance2L/2.0-SideWalkZMP)*cos(gRRotAngZ[3]);
		gFstpX[4] = (distance2L/2.0+SideWalkZMP*2)*cos(gRRotAngZ[4]);
		gFstpX[5] = SideWalkZMP*2*cos(gRRotAngZ[5]);

		for (int i = 3 ; i < gNumOfStep+20 ; i++)
		{
			gFstpX[i*2] = 2*SideWalkZMP*cos(gRRotAngZ[i*2]);
			gFstpX[i*2+1] = 2*SideWalkZMP*cos(gLRotAngZ[i*2+1]);
		}

		gFstpY[0] = 0;
		gFstpY[1] = (distance2L/2.0)*sin(gRRotAngZ[1]);
		gFstpY[2] = -(distance2L/2.0+SideWalkZMP)*sin(gLRotAngZ[2]);
		gFstpY[3] = (distance2L/2.0-SideWalkZMP)*sin(gRRotAngZ[3]);
		gFstpY[4] = -(distance2L/2.0+2*SideWalkZMP)*sin(gLRotAngZ[4]);
		gFstpY[5] = -2*SideWalkZMP*sin(gRRotAngZ[5]);


		for (int i = 3 ; i < 30 ; i++)
		{
			gFstpY[i*2] = -2*SideWalkZMP*sin(gRRotAngZ[i*2]);
			gFstpY[i*2+1] = -2*SideWalkZMP*sin(gLRotAngZ[i*2+1]);
		}
	}
	else if (Direction == DirectionRight)
	{
		// 左右 
		gFstpX[0] = 0;
		gFstpX[1] = x_val_zmp*cos(gRRotAngZ[1]);

		gFstpX[2] = -(distance2L/2.0+SideWalkZMP)*cos(gRRotAngZ[2]);
		gFstpX[3] = (distance2L/2.0-SideWalkZMP)*cos(gRRotAngZ[3]);
		gFstpX[4] = -(distance2L/2.0+SideWalkZMP*2)*cos(gRRotAngZ[4]);
		gFstpX[5] = -SideWalkZMP*2*cos(gRRotAngZ[5]);

		for (int i = 3 ; i < gNumOfStep+20 ; i++)
		{
			gFstpX[i*2] = -2*SideWalkZMP*cos(gRRotAngZ[i*2]);
			gFstpX[i*2+1] = -2*SideWalkZMP*cos(gLRotAngZ[i*2+1]);
		}

		gFstpY[0] = 0;
		gFstpY[1] = -distance2L/2.0*sin(gRRotAngZ[1]);
		gFstpY[2] = (distance2L/2.0+SideWalkZMP)*sin(gLRotAngZ[2]);
		gFstpY[3] = -(distance2L/2.0-SideWalkZMP)*sin(gRRotAngZ[3]);
		gFstpY[4] = (distance2L/2.0+2*SideWalkZMP)*sin(gLRotAngZ[4]);
		gFstpY[5] = 2*SideWalkZMP*sin(gRRotAngZ[5]);

		for (int i = 3 ; i < 30 ; i++)
		{
			gFstpY[i*2] = 2*SideWalkZMP*sin(gRRotAngZ[i*2]);
			gFstpY[i*2+1] = 2*SideWalkZMP*sin(gLRotAngZ[i*2+1]);
		}
	}


	for (int i = 0 ; i < gNumOfStep+20 ; i++)
		gGroundHeight[i] = 0.0;

}

void gInitSumoMotion(void) // 抬起單腳 類似相撲動作
{
	/******************************************************************
	input: void
	output: void

	Note:

	// 初始化機器人行為為相撲伸展
	// 腳步設定 地形設定 機器人步行參數 設定於此
	// 相似的函數有 gInitTurnLeft gInitWalkStraight gInitStepHere gInitStair gInitSquat

	******************************************************************/
	gNumOfStep = 11;
	gCOGDown = 13; // 
	//gCOGDown = 40; // 膝蓋小彎

	gKineAll.FlagSumoMode = 1;

	double x_val_zmp = 80;
	bool PNx = 0;
	double y_step_zmp = 0;

	double SideWalkZMP = 45;

	double TurnRadius = 300;

	double distance2L = 160;

	gKineAll.StepHeight[0] = 0;
	for (int i = 1 ; i < gNumOfStep+10 ; i++)
		gKineAll.StepHeight[i] = 25;

	gKineAll.selSupport[0] = 2;
	for (int i = 1 ; i < 4 ; i++)
	{
		gKineAll.selSupport[i] = 1;
	}
	for (int i = 4 ; i < gNumOfStep+10 ; i++)
	{
		gKineAll.selSupport[i] = 0;
	}
	for (int i = 8 ; i < gNumOfStep+10 ; i++)
	{
		gKineAll.selSupport[i] = 2;
	}




	for (int i = gNumOfStep-4 ; i < gNumOfStep+20 ; i++)
		gKineAll.selSupport[i] = 2; // double support and prepare to stop


	double AngChange = 0.0;

	for (int i = 0 ; i < gNumOfStep+20 ; i++)
	{
		gLRotAngZ[i*2] = AngChange*i+gLAngZWorld;
		gLRotAngZ[i*2+1] = AngChange*i+gLAngZWorld;
	}

	gRRotAngZ[0] = 0.0 + (gLAngZWorld+gRAngZWorld)/2.0;
	for (int i = 0 ; i < gNumOfStep+20 ; i++)
	{
		gRRotAngZ[i*2+1] = AngChange*i+gRAngZWorld;
		gRRotAngZ[i*2+2] = AngChange*i+gRAngZWorld;
	}

	gLLInitZMPFB = -80*sin(gRRotAngZ[0]);
	gRLInitZMPFB = 80*sin(gRRotAngZ[0]);
	gLLInitZMPLR = 80*cos(gRRotAngZ[0]);
	gRLInitZMPLR = -80*cos(gRRotAngZ[0]);


	// 左右 
	gFstpX[0] = 0;
	gFstpX[1] = -x_val_zmp*cos(gRRotAngZ[1]);

	gFstpX[2] = -(distance2L/2.0)*cos(gRRotAngZ[2]);
	gFstpX[3] = -(distance2L/2.0)*cos(gRRotAngZ[3]);
	gFstpX[4] = (distance2L/2.0)*cos(gRRotAngZ[4]);
	gFstpX[5] = (distance2L/2.0)*cos(gRRotAngZ[4]);
	gFstpX[6] = (distance2L/2.0)*cos(gRRotAngZ[4]);
	gFstpX[7] = 0;

	for (int i = 8 ; i < gNumOfStep+20 ; i++)
	{
		gFstpX[i] = 0;
	}

	//for (int i = 3 ; i < gNumOfStep+20 ; i++)
	//{
	//	gFstpX[i*2] = -2*SideWalkZMP*cos(gRRotAngZ[i*2]);
	//	gFstpX[i*2+1] = -2*SideWalkZMP*cos(gLRotAngZ[i*2+1]);
	//}

	gFstpY[0] = 0;
	gFstpY[1] = (distance2L/2.0)*sin(gRRotAngZ[1]);
	gFstpY[2] = (distance2L/2.0)*sin(gLRotAngZ[2]);
	gFstpY[3] = (distance2L/2.0)*sin(gRRotAngZ[3]);
	gFstpY[4] = -(distance2L/2.0)*sin(gLRotAngZ[4]);
	gFstpY[5] = -(distance2L/2.0)*sin(gLRotAngZ[4]);
	gFstpY[6] = -(distance2L/2.0)*sin(gLRotAngZ[4]);
	gFstpY[7] = 0;

	for (int i = 3 ; i < 30 ; i++)
	{
		gFstpY[i*2] = 0;
		gFstpY[i*2+1] = 0;
	}

	for (int i = 0 ; i < gNumOfStep+20 ; i++)
		gGroundHeight[i] = 0.0;

}

void gInitStair(void)
{
	/******************************************************************
	input: void
	output: void

	Note:

	// 初始化機器人行為為走樓梯
	// 腳步設定 地形設定 機器人步行參數 設定於此
	// 相似的函數有 gInitTurnLeft gInitWalkStraight gInitStepHere gInitStair gInitSquat gInitDownStair
	// Wei_Zh Lai 20121128
	******************************************************************/
	gNumOfStep = 7; //gNumOfStep = 8  上兩階 
	checkonestep = 1;  //用來判斷在gCalculateZMP()中的gPCenter[1][3] 要等於0 (這樣才zmp才不會在只跨一步的時候 會多左右擺動一次)

	gCOGDown = 54; // 蹲很多才跨得過
	mode1 = 1 ; 
	//#define gCOGDown		115
	double x_val_zmp = 80;
	bool PNx = 0;
	double y_step_zmp = 460;
	double TurnRadius = 300;


	double distance2L = 160.0;
	double stairheight = 40;//45
		
	//設定support情況
	
	for (int i = 0 ; i < 400 ; i++)
		gKineAll.selSupport[i] = 0.0;
	
	gKineAll.selSupport[0] = 2;
	
	for (int i = 1 ; i < gNumOfStep+30 ; i+=2)
	{
		gKineAll.selSupport[i] = 1;
		gKineAll.selSupport[i+1] = 0;
	}
	
	for (int i = gNumOfStep-4 ; i < 4000 ; i++)
		gKineAll.selSupport[i] = 2; // double support and prepare to stop	
	
	gKineAll.StepHeight[0] = 0;
	for (int i = 1 ; i < 4000 ; i++)
		gKineAll.StepHeight[i] = 20;

	for (int i = gNumOfStep-4 ; i < 4000 ; i++)
		gKineAll.StepHeight[i] = 0;

	gKineAll.StepHeight[1]=30;
	//rotate
	
	double AngChange = 0.0;

	for (int i = 0 ; i < 30 ; i++)
	{
		gLRotAngZ[i*2] = AngChange*i;
		gLRotAngZ[i*2+1] = AngChange*i;
	}

	gRRotAngZ[0] = 0.0 ;
	for (int i = 0 ; i < 30 ; i++)
	{
		gRRotAngZ[i*2+1] = AngChange*i;
		gRRotAngZ[i*2+2] = AngChange*i;
	}

	gLLInitZMPFB = -80*sin(gRRotAngZ[0]);
	gRLInitZMPFB = 80*sin(gRRotAngZ[0]);
	gLLInitZMPLR = 80*cos(gRRotAngZ[0]);
	gRLInitZMPLR = -80*cos(gRRotAngZ[0]);


	// 左右 x方向
	for (int i = 0 ; i < 400 ; i++){
		gFstpX[i] = 0.0;};
	       

    gFstpX[0] = 0;
	gFstpX[1] = -x_val_zmp;
	for (int i = 1 ; i < 2000 ; i++)
	{
		gFstpX[i*2] = distance2L/2.0;
		gFstpX[i*2+1] = -distance2L/2.0;
	}


	//只有踏一階
	if(checkonestep ==1)
	{
      for (int i = gNumOfStep-4; i < 4000 ; i++)
		gFstpX[i] = 0.0;
	    gFstpX[3] =gFstpX[1];
	}
	else
	{
		for (int i = gNumOfStep-4; i < 4000 ; i++)
		gFstpX[i] = 0.0;

	}
	





	for (int i = 0 ; i < 400 ; i++)
		gFstpY[i] = 0.0;

	gFstpY[0] = 0.0;
	gFstpY[1] = 0.0;
	

	for (int i = 1 ; i < gNumOfStep ; i+=2)	
	{	
		gFstpY[i+1] =  y_step_zmp*0.5*i;
		gFstpY[i+2] =  y_step_zmp*0.5*(i+1);   
	}

	for (int i = gNumOfStep-4 ; i < 200 ; i++)
		gFstpY[i] = gFstpY[gNumOfStep-5];
		
	
// Set height of ground
	
	for (int i = 0 ; i < 400 ; i++)
		gGroundHeight[i] = 0.0;
	

	gGroundHeight[0] = 0.0;
	gGroundHeight[1] = 0.0;
	gGroundHeight[2] = stairheight;
	//gGroundHeight[3] = 65;
	
	for (int i = 1 ; i < gNumOfStep-4 ; i++)		
		gGroundHeight[i+1] = stairheight *(i)  ;

	for (int i = gNumOfStep-4 ; i < 1000 ; i++)		
		gGroundHeight[i] = gGroundHeight[gNumOfStep-5]   ;


		
		//哲軒改20121127	
	/*for (int pk = 0 ; pk < 11 ; pk++)   //似乎是斜坡
	{
		gGroundHeight[pk] *= 130.0/40.0;
	}*/
}
void gInitStairMod(int StepNum, double Dist2L, double StairHeight)
{
	/******************************************************************
	input: void
	output: void

	Note:

	// 初始化機器人行為為走樓梯
	// 腳步設定 地形設定 機器人步行參數 設定於此
	// 相似的函數有 gInitTurnLeft gInitWalkStraight gInitStepHere gInitStair gInitSquat gInitDownStair
	// Wei_Zh Lai 20121128
	******************************************************************/
	gNumOfStep = StepNum; //gNumOfStep = 8  上兩階(Upstair uncompleted) =7上1階(Upstair completed)
	gCOGDown = 54; // 蹲很多才跨得過

	if(gNumOfStep==7)//Specific Ver. Warn.
		checkonestep = 1;  //用來判斷在gCalculateZMP()中的gPCenter[1][3] 要等於0 (這樣才zmp才不會在只跨一步的時候 會多左右擺動一次)
	else
		checkonestep=0;
	if(StairHeight>0)
		gUpStair=true;
	else
		gUpStair=false;

	mode1 = 1 ; // Warning!!! Please Make the Comment

	double x_val_zmp = 80;
	bool PNx = 0;
	double y_step_zmp = Dist2L;
	double TurnRadius = 300;

	double distance2L = 160.0;
	double stairheight = StairHeight;
		
	//設定support情況
	
	for (int i = 0 ; i < 400 ; i++)
		gKineAll.selSupport[i] = 0.0;
	
	gKineAll.selSupport[0] = 2;
	
	for (int i = 1 ; i < gNumOfStep+30 ; i+=2)
	{
		gKineAll.selSupport[i] = 1;
		gKineAll.selSupport[i+1] = 0;
	}
	
	for (int i = gNumOfStep-4 ; i < 4000 ; i++)
		gKineAll.selSupport[i] = 2; // double support and prepare to stop	
	
	gKineAll.StepHeight[0] = 0;
	for (int i = 1 ; i < 4000 ; i++)
		gKineAll.StepHeight[i] = 20;

	for (int i = gNumOfStep-4 ; i < 4000 ; i++)
		gKineAll.StepHeight[i] = 0;
	
	if(gNumOfStep==7)//Specific Ver. Warn.
		gKineAll.StepHeight[1]=30;
	//rotate
	
	double AngChange = 0.0;

	for (int i = 0 ; i < 30 ; i++)
	{
		gLRotAngZ[i*2] = AngChange*i;
		gLRotAngZ[i*2+1] = AngChange*i;
	}

	gRRotAngZ[0] = 0.0 ;
	for (int i = 0 ; i < 30 ; i++)
	{
		gRRotAngZ[i*2+1] = AngChange*i;
		gRRotAngZ[i*2+2] = AngChange*i;
	}

	gLLInitZMPFB = -80*sin(gRRotAngZ[0]);
	gRLInitZMPFB = 80*sin(gRRotAngZ[0]);
	gLLInitZMPLR = 80*cos(gRRotAngZ[0]);
	gRLInitZMPLR = -80*cos(gRRotAngZ[0]);


	// 左右 x方向
	for (int i = 0 ; i < 400 ; i++){
		gFstpX[i] = 0.0;};
	       

    gFstpX[0] = 0;
	gFstpX[1] = -x_val_zmp;
	for (int i = 1 ; i < 2000 ; i++)
	{
		gFstpX[i*2] = distance2L/2.0;
		gFstpX[i*2+1] = -distance2L/2.0;
	}

	//只有踏一階
	if(checkonestep ==1)
	{
      for (int i = gNumOfStep-4; i < 4000 ; i++)
		gFstpX[i] = 0.0;
	    gFstpX[3] =gFstpX[1];
	}
	else
	{
		for (int i = gNumOfStep-4; i < 4000 ; i++)
		gFstpX[i] = 0.0;
	}
	
	for (int i = 0 ; i < 400 ; i++)
		gFstpY[i] = 0.0;

	gFstpY[0] = 0.0;
	gFstpY[1] = 0.0;
	

	for (int i = 1 ; i < gNumOfStep ; i+=2)	
	{	
		gFstpY[i+1] =  y_step_zmp*0.5*i;
		gFstpY[i+2] =  y_step_zmp*0.5*(i+1);   
	}

	for (int i = gNumOfStep-4 ; i < 200 ; i++)
		gFstpY[i] = gFstpY[gNumOfStep-5];
		
	
// Set height of ground
	
	for (int i = 0 ; i < 400 ; i++)
		gGroundHeight[i] = 0.0;

	gGroundHeight[0] = 0.0;
	gGroundHeight[1] = 0.0;
	gGroundHeight[2] = stairheight;

	for (int i = 1 ; i < gNumOfStep-4 ; i++)		
		gGroundHeight[i+1] = stairheight *(i)  ;

	for (int i = gNumOfStep-4 ; i < 1000 ; i++)		
		gGroundHeight[i] = gGroundHeight[gNumOfStep-5]   ;
}
void gInitUpStairMod(int StepNum, double Dist2L, double StairHeight)
{
	/******************************************************************
	input: void
	output: void

	Note:

	// 初始化機器人行為為走樓梯
	// 腳步設定 地形設定 機器人步行參數 設定於此
	// 相似的函數有 gInitTurnLeft gInitWalkStraight gInitStepHere gInitStair gInitSquat gInitDownStair
	// Wei_Zh Lai 20121128
	******************************************************************/
	gNumOfStep = StepNum; //gNumOfStep = 8  上兩階(Upstair uncompleted) =7上1階(Upstair completed)
	gCOGDown = 60; // 蹲很多才跨得過
	gFlagStairSaid=true;
	if(gNumOfStep==7)//Specific Ver. Warn.
		checkonestep = 1;  //用來判斷在gCalculateZMP()中的gPCenter[1][3] 要等於0 (這樣才zmp才不會在只跨一步的時候 會多左右擺動一次)
	else
		checkonestep = 0;
	if(StairHeight>0)
		gUpStair=true;
	else
		gUpStair=false;

	mode1 = 1 ; // Warning!!! Please Make the Comment

	double x_val_zmp = 80;
	bool PNx = 0;
	double y_step_zmp = Dist2L;
	double TurnRadius = 300;

	double distance2L = 160.0;
	double stairheight = StairHeight;
		
	//設定support情況
	
	for (int i = 0 ; i < 400 ; i++)
		gKineAll.selSupport[i] = 0.0;
	
	gKineAll.selSupport[0] = 2;
	
	for (int i = 1 ; i < gNumOfStep+30 ; i+=2)
	{
		gKineAll.selSupport[i] = 1;
		gKineAll.selSupport[i+1] = 0;
	}
	
	for (int i = gNumOfStep-4 ; i < 4000 ; i++)
		gKineAll.selSupport[i] = 2; // double support and prepare to stop	
	
	gKineAll.StepHeight[0] = 0;
	for (int i = 1 ; i < 4000 ; i++)
		gKineAll.StepHeight[i] = 20;

	for (int i = gNumOfStep-4 ; i < 4000 ; i++)
		gKineAll.StepHeight[i] = 0;
	
	if(gNumOfStep==7)//Specific Ver. Warn.
		gKineAll.StepHeight[1]=30;
	//rotate
	
	double AngChange = 0.0;

	for (int i = 0 ; i < 30 ; i++)
	{
		gLRotAngZ[i*2] = AngChange*i;
		gLRotAngZ[i*2+1] = AngChange*i;
	}

	gRRotAngZ[0] = 0.0 ;
	for (int i = 0 ; i < 30 ; i++)
	{
		gRRotAngZ[i*2+1] = AngChange*i;
		gRRotAngZ[i*2+2] = AngChange*i;
	}

	gLLInitZMPFB = -80*sin(gRRotAngZ[0]);
	gRLInitZMPFB = 80*sin(gRRotAngZ[0]);
	gLLInitZMPLR = 80*cos(gRRotAngZ[0]);
	gRLInitZMPLR = -80*cos(gRRotAngZ[0]);


	// 左右 x方向
	for (int i = 0 ; i < 400 ; i++){
		gFstpX[i] = 0.0;};
	       

    gFstpX[0] = 0;
	gFstpX[1] = -x_val_zmp;
	for (int i = 1 ; i < 2000 ; i++)
	{
		gFstpX[i*2] = distance2L/2.0;
		gFstpX[i*2+1] = -distance2L/2.0;
	}

	//只有踏一階
	if(checkonestep ==1)
	{
      for (int i = gNumOfStep-4; i < 4000 ; i++)
		gFstpX[i] = 0.0;
	    gFstpX[3] =gFstpX[1];
	}
	else
	{
		for (int i = gNumOfStep-4; i < 4000 ; i++)
		gFstpX[i] = 0.0;
	}
	
	for (int i = 0 ; i < 400 ; i++)
		gFstpY[i] = 0.0;

	gFstpY[0] = 0.0;
	gFstpY[1] = 0.0;
	

	for (int i = 1 ; i < gNumOfStep ; i+=2)	
	{	
		gFstpY[i+1] =  y_step_zmp*0.5*((i-1)/2+1);
		gFstpY[i+2] =  y_step_zmp*0.5*((i-1)/2+1);   
	}

	for (int i = gNumOfStep-4 ; i < 200 ; i++)
		gFstpY[i] = gFstpY[gNumOfStep-5];
		
	
// Set height of ground
	
	for (int i = 0 ; i < 400 ; i++)
		gGroundHeight[i] = 0.0;

	gGroundHeight[0] = 0.0;
	gGroundHeight[1] = 0.0;
	gGroundHeight[2] = stairheight;

	for (int i = 1 ; i < gNumOfStep-4 ; i+=2)
	{
		gGroundHeight[i+1] = stairheight *((i-1)/2+1)  ;
		gGroundHeight[i+2] = stairheight *((i-1)/2+1)  ;
	}
	for (int i = gNumOfStep-4 ; i < 1000 ; i++)		
		gGroundHeight[i] = gGroundHeight[gNumOfStep-5]   ;
}
void gInitdownStair(void)
{
	/******************************************************************
	input: void
	output: void

	Note:

	// 初始化機器人行為為走樓梯
	// 腳步設定 地形設定 機器人步行參數 設定於此
	// 相似的函數有 gInitTurnLeft gInitWalkStraight gInitStepHere gInitStair gInitSquat

	******************************************************************/
	gNumOfStep = 7; //gNumOfStep = 8  上兩階    
	gCOGDown = 60; // 蹲很多才跨得過
	mode1= 1;
	//#define gCOGDown		115
	double x_val_zmp = 75;
	bool PNx = 0;
	double y_step_zmp = 500;
	double TurnRadius = 300;
	double distance2L = 160.0;
	double stairheight = 40;	
	
	//設定support情況
	
	for (int i = 0 ; i < 400 ; i++)
		gKineAll.selSupport[i] = 0.0;
	
	gKineAll.selSupport[0] = 2;
	
	for (int i = 1 ; i < gNumOfStep+30 ; i+=2)
	{
		gKineAll.selSupport[i] = 1;
		gKineAll.selSupport[i+1] = 0;
	}
	
	for (int i = gNumOfStep-4 ; i < 4000 ; i++)
		gKineAll.selSupport[i] = 2; // double support and prepare to stop

	gKineAll.StepHeight[0] = 0;
	
	for (int i = 1 ; i < 4000 ; i++)
		gKineAll.StepHeight[i] = 6;

	for (int i = gNumOfStep-4 ; i < 4000 ; i++)
		gKineAll.StepHeight[i] = 0;


	//rotate
	
	double AngChange = 0.0;

	for (int i = 0 ; i < 30 ; i++)
	{
		gLRotAngZ[i*2] = AngChange*i;
		gLRotAngZ[i*2+1] = AngChange*i;
	}

	gRRotAngZ[0] = 0.0 ;
	for (int i = 0 ; i < 30 ; i++)
	{
		gRRotAngZ[i*2+1] = AngChange*i;
		gRRotAngZ[i*2+2] = AngChange*i;
	}

	gLLInitZMPFB = -80*sin(gRRotAngZ[0]);
	gRLInitZMPFB = 80*sin(gRRotAngZ[0]);
	gLLInitZMPLR = 80*cos(gRRotAngZ[0]);
	gRLInitZMPLR = -80*cos(gRRotAngZ[0]);


	// 左右 x方向
	for (int i = 0 ; i < 400 ; i++){
		gFstpX[i] = 0.0;};
	       

    gFstpX[0] = 0;
	gFstpX[1] = -x_val_zmp;
	for (int i = 1 ; i < 2000 ; i++)
	{
		gFstpX[i*2] = distance2L/2.0;
		gFstpX[i*2+1] = -distance2L/2.0;
	}
	for (int i = gNumOfStep-4; i < 4000 ; i++)
		gFstpX[i] = 0.0;

	for (int i = 0 ; i < 400 ; i++)
		gFstpY[i] = 0.0;

	gFstpY[0] = 0.0;
	gFstpY[1] = 0.0;
	
	for (int i = 1 ; i < gNumOfStep ; i+=2)	
	{	
		gFstpY[i+1] =  y_step_zmp*0.5*i;
		gFstpY[i+2] =  y_step_zmp*0.5*(i+1);   
	}

	for (int i = gNumOfStep-4 ; i < 200 ; i++)
		gFstpY[i] = gFstpY[gNumOfStep-5];
		


// Set height of ground
	
	for (int i = 0 ; i < 400 ; i++)
		gGroundHeight[i] = 0.0;

	gGroundHeight[0] = 0.0;
	gGroundHeight[1] = 0.0;
	gGroundHeight[2] = - stairheight;
	

	for (int i = 1 ; i < gNumOfStep-4 ; i++)		
		gGroundHeight[i+1] = -stairheight *(i)   ;

	//gGroundHeight[2] = 40//- 55;

	for (int i = gNumOfStep-4 ; i < 1000 ; i++)		
		gGroundHeight[i] = gGroundHeight[gNumOfStep-5]   ;
	

		//哲軒改20121127
		
	/*for (int pk = 0 ; pk < 11 ; pk++)   //似乎是斜坡
	{
		gGroundHeight[pk] *= 130.0/40.0;
	}*/

}

void gInitSquat(void)
{
	/******************************************************************
	input: void
	output: void

	Note:

	// 初始化機器人行為為蹲站
	// 腳步設定 地形設定 機器人步行參數 設定於此
	// 相似的函數有 gInitTurnLeft gInitWalkStraight gInitStepHere gInitStair gInitSquat gInitDownStair

	******************************************************************/

	// // 請不要小於8!!!
	gNumOfStep = 9; // 請不要小於8!!!
	// 請不要小於8!!!
	gCOGDown = 0;
	gKineAll.FlagSumoMode = 0;

	for (int i = 0 ; i < gNumOfStep+30 ; i++)
		gKineAll.StepHeight[i] = 0;

	double AngChange = 0.0;

	for (int i = 0 ; i < gNumOfStep+30 ; i++)
	{
		gLRotAngZ[i*2] = AngChange*i+gLAngZWorld;
		gLRotAngZ[i*2+1] = AngChange*i+gLAngZWorld;
	}

	gRRotAngZ[0] = gRAngZWorld;
	for (int i = 0 ; i < gNumOfStep+30 ; i++)
	{
		gRRotAngZ[i*2+1] = AngChange*i+gRAngZWorld;
		gRRotAngZ[i*2+2] = AngChange*i+gRAngZWorld;
	}

	for (int i = 0 ; i < gNumOfStep+30 ; i++)
	{
		gKineAll.selSupport[i] = 2;	// 從頭到尾都是DSP
	}

	gLLInitZMPFB = -80*sin(gRRotAngZ[0]);
	gRLInitZMPFB = 80*sin(gRRotAngZ[0]);
	gLLInitZMPLR = 80*cos(gRRotAngZ[0]);
	gRLInitZMPLR = -80*cos(gRRotAngZ[0]);

	// 左右 
	for (int i = 1 ; i < gNumOfStep+30 ; i++)
	{
		gFstpX[i] = 0;
		gFstpY[i] = 0;
		gGroundHeight[i] = 0.0;
	}	
}

//泓逸start20120309
void gInitArmWaveWalkStraight(void)
{
	/******************************************************************
	input: void
	output: void

	Note:

	// 初始化手臂直走時的揮動
	// 手部設定 外面環近參數設定 設定於此
	// 腳步必須先設定好之後才能使用
	// 相似的函數有 gInitArmWaveWalkStraight gInitArmPushCartWalkStraight gInitRArmCarryWalkStraight gInitArmWaveTurn

	******************************************************************/
	//泓逸start120223

	gDemoArmTrajFlag = 0;//用來判別arm不同traj用的(demo用的) 0代表直走 1代表左轉
	gIKMethod = 0;
	//走路時擺手軌跡
	double RArmFrontX = 120;	//右手往前擺x
	double RArmFrontY = -0;	//右手往前擺y
	double RArmFrontZ = 80;		//右手往前擺z
	double LArmFrontX = 120;	//左手往前擺x
	double LArmFrontY = 0;	//左手往前擺y
	double LArmFrontZ = 80;		//左手往前擺z

	//double RArmFrontX = 200;	//右手往前擺x
	//double RArmFrontY = 0;	//右手往前擺y
	//double RArmFrontZ = 50;		//右手往前擺z
	//double LArmFrontX = 200;	//左手往前擺x
	//double LArmFrontY = 0;	//左手往前擺y
	//double LArmFrontZ = 50;		//左手往前擺z

	//double RArmFrontX = 0;	//右手往前擺x
	//double RArmFrontY = 0;	//右手往前擺y
	//double RArmFrontZ = 0;		//右手往前擺z
	//double LArmFrontX = 0;	//左手往前擺x
	//double LArmFrontY = 0;	//左手往前擺y
	//double LArmFrontZ = 0;		//左手往前擺z

	double RArmBackX = 0;		//右手往後擺x
	double RArmBackY = 0;		//右手往後擺y
	double RArmBackZ = 0;		//右手往後擺z
	double LArmBackX = 0;		//左手往後擺x
	double LArmBackY = 0;		//左手往後擺y
	double LArmBackZ = 0;		//左手往後擺z

	gRArmWalkRot[0] = 0;
	gRArmWalkRot[1] = 1;
	gRArmWalkRot[2] = 0;
	gRArmWalkRot[3] = 0;
	gRArmWalkRot[4] = 0;
	gRArmWalkRot[5] = -1;
	gRArmWalkRot[6] = -1;
	gRArmWalkRot[7] = 0;
	gRArmWalkRot[8] = 0;

	gLArmWalkRot[0] = 0;
	gLArmWalkRot[1] = 1;
	gLArmWalkRot[2] = 0;
	gLArmWalkRot[3] = 0;
	gLArmWalkRot[4] = 0;
	gLArmWalkRot[5] = -1;
	gLArmWalkRot[6] = -1;
	gLArmWalkRot[7] = 0;
	gLArmWalkRot[8] = 0;
	
	for(int i = 0;i < (gNumOfStep-2)*gStepSample ; i++ )
	{
		for(int j = 0;j<9;j++)
		{
			gRArmWalkRot[i*9+j] = gRArmWalkRot[j];
			gLArmWalkRot[i*9+j] = gLArmWalkRot[j];
		}
	}

	for (int i = 0 ; i < gNumOfStep-4 ; i++)
	{
		if (i == 0)
		{
			for(int j = 0 ; j < gStepSample ; j++)//蹲下時手不擺動
			{
				gRArmWalkX[j] = 0;
				gRArmWalkY[j] = 0;
				gRArmWalkZ[j] = 0;
				gLArmWalkX[j] = 0;
				gLArmWalkY[j] = 0;	
				gLArmWalkZ[j] = 0;
			}
		}
		else if(i == 1)//踏第一步時只有一隻手擺動
		{
			if( gKineAll.selSupport[1] == 0)//左腳support
			{
				gKineAll.GenSmoothZMPShift_ZeroJerk(LArmBackX,LArmFrontX,gStepSample,gLArmWalkX+gStepSample);//左手往前擺
				gKineAll.GenSmoothZMPShift_ZeroJerk(LArmBackY,LArmFrontY,gStepSample,gLArmWalkY+gStepSample);
				gKineAll.GenSmoothZMPShift_ZeroJerk(LArmBackZ,LArmFrontZ,gStepSample,gLArmWalkZ+gStepSample);
				for(int j = gStepSample;j<gStepSample*2;j++)
				{
					gRArmWalkX[j] = 0;
					gRArmWalkY[j] = 0;
					gRArmWalkZ[j] = 0;
				}
			}
			else//右腳support
			{
				gKineAll.GenSmoothZMPShift_ZeroJerk(RArmBackX,RArmFrontX,gStepSample,gRArmWalkX+gStepSample);//右手往前擺
				gKineAll.GenSmoothZMPShift_ZeroJerk(RArmBackY,RArmFrontY,gStepSample,gRArmWalkY+gStepSample);
				gKineAll.GenSmoothZMPShift_ZeroJerk(RArmBackZ,RArmFrontZ,gStepSample,gRArmWalkZ+gStepSample);
				for(int j = gStepSample;j<gStepSample*2;j++)
				{
					gLArmWalkX[j] = 0;
					gLArmWalkY[j] = 0;
					gLArmWalkZ[j] = 0;
				}
			}
		}
		else if(i < (gNumOfStep - 5))
		{
			if( gKineAll.selSupport[i] == 0)//左腳support
			{
				gKineAll.GenSmoothZMPShift_ZeroJerk(LArmBackX,LArmFrontX,gStepSample,gLArmWalkX+gStepSample*i);//左手往前擺
				gKineAll.GenSmoothZMPShift_ZeroJerk(LArmBackY,LArmFrontY,gStepSample,gLArmWalkY+gStepSample*i);
				gKineAll.GenSmoothZMPShift_ZeroJerk(LArmBackZ,LArmFrontZ,gStepSample,gLArmWalkZ+gStepSample*i);

				gKineAll.GenSmoothZMPShift_ZeroJerk(RArmFrontX,RArmBackX,gStepSample,gRArmWalkX+gStepSample*i);//右手往後擺
				gKineAll.GenSmoothZMPShift_ZeroJerk(RArmFrontY,RArmBackY,gStepSample,gRArmWalkY+gStepSample*i);
				gKineAll.GenSmoothZMPShift_ZeroJerk(RArmFrontZ,RArmBackZ,gStepSample,gRArmWalkZ+gStepSample*i);
			}
			else//右腳support
			{
				gKineAll.GenSmoothZMPShift_ZeroJerk(RArmBackX,RArmFrontX,gStepSample,gRArmWalkX+gStepSample*i);//右手往前擺
				gKineAll.GenSmoothZMPShift_ZeroJerk(RArmBackY,RArmFrontY,gStepSample,gRArmWalkY+gStepSample*i);
				gKineAll.GenSmoothZMPShift_ZeroJerk(RArmBackZ,RArmFrontZ,gStepSample,gRArmWalkZ+gStepSample*i);

				gKineAll.GenSmoothZMPShift_ZeroJerk(LArmFrontX,LArmBackX,gStepSample,gLArmWalkX+gStepSample*i);//左手往後擺
				gKineAll.GenSmoothZMPShift_ZeroJerk(LArmFrontY,LArmBackY,gStepSample,gLArmWalkY+gStepSample*i);
				gKineAll.GenSmoothZMPShift_ZeroJerk(LArmFrontZ,LArmBackZ,gStepSample,gLArmWalkZ+gStepSample*i);
			}
		}
		else
		{
			if( gKineAll.selSupport[i] == 0)//左腳support
			{
				gKineAll.GenSmoothZMPShift_ZeroJerk(RArmFrontX,RArmBackX,gStepSample,gRArmWalkX+gStepSample*i);//右手往後擺
				gKineAll.GenSmoothZMPShift_ZeroJerk(RArmFrontY,RArmBackY,gStepSample,gRArmWalkY+gStepSample*i);
				gKineAll.GenSmoothZMPShift_ZeroJerk(RArmFrontZ,RArmBackZ,gStepSample,gRArmWalkZ+gStepSample*i);
				for(int j = gStepSample*i;j<gStepSample*(i+1);j++)
				{
					gLArmWalkX[j] = 0;
					gLArmWalkY[j] = 0;
					gLArmWalkZ[j] = 0;
				}
			}
			else//右腳support
			{
				gKineAll.GenSmoothZMPShift_ZeroJerk(LArmFrontX,LArmBackX,gStepSample,gLArmWalkX+gStepSample*i);//左手往後擺
				gKineAll.GenSmoothZMPShift_ZeroJerk(LArmFrontY,LArmBackY,gStepSample,gLArmWalkY+gStepSample*i);
				gKineAll.GenSmoothZMPShift_ZeroJerk(LArmFrontZ,LArmBackZ,gStepSample,gLArmWalkZ+gStepSample*i);
				for(int j = gStepSample*i;j<gStepSample*(i+1);j++)
				{
					gRArmWalkX[j] = 0;
					gRArmWalkY[j] = 0;
					gRArmWalkZ[j] = 0;
				}
			}
			for(int j = gStepSample*(i+1);j<LQSIBufferSize;j++)
			{
				gRArmWalkX[j] = 0;
				gRArmWalkY[j] = 0;
				gRArmWalkZ[j] = 0;
				gLArmWalkX[j] = 0;
				gLArmWalkY[j] = 0;	
				gLArmWalkZ[j] = 0;
			
			}
		}
	}
	

	//for(int i = 0;i<20000;i++)
	//{
	//	TestDeg[TestPos] = gLArmWalkX[i];
	//	TestPos ++;
	//	TestDeg[TestPos] = gLArmWalkY[i];
	//	TestPos ++;
	//	TestDeg[TestPos] = gLArmWalkZ[i];
	//	TestPos ++;
	//	TestDeg[TestPos] = gRArmWalkX[i];
	//	TestPos ++;
	//	TestDeg[TestPos] = gRArmWalkY[i];
	//	TestPos ++;
	//	TestDeg[TestPos] = gRArmWalkZ[i];
	//	TestPos ++;
	//}

	//fstream ccc;
	//ccc.open("testpos1.txt",ios::out);
	//for(int i = 0;i < TestPos;i++)
	//{
	//	ccc<<TestDeg[i]<<"     ";
	//	if(i%6 == 5)
	//	{
	//		ccc<<endl;
	//	}
	//}
	//ccc.close();
	//TestPos = 0;
	//泓逸end120223


}

void gInitArmPushCartWalkStraight(void)
{

	/******************************************************************
	input: void
	output: void

	Note:

	// 初始化手臂推東西時的動作
	// 手部設定 外面環近參數設定 設定於此
	// 腳步必須先設定好之後才能使用
	// 相似的函數有 gInitArmWaveWalkStraight gInitArmPushCartWalkStraight gInitRArmCarryWalkStraight gInitArmWaveTurn

	******************************************************************/

	gDemoArmTrajFlag = 0;//用來判別arm不同traj用的(demo用的) 0代表直走 1代表左轉
	gIKMethod = 0;
	double RArmFrontX = 350;	//右手往前擺x
	double RArmFrontY = 0;		//右手往前擺y
	double RArmFrontZ = 350;	//右手往前擺z
	double LArmFrontX = 350;	//左手往前擺x
	double LArmFrontY = 0;		//左手往前擺y
	double LArmFrontZ = 350;	//左手往前擺z


	double RArmBackX = 0;		//右手往後擺x
	double RArmBackY = 0;		//右手往後擺y
	double RArmBackZ = 0;		//右手往後擺z
	double LArmBackX = 0;		//左手往後擺x
	double LArmBackY = 0;		//左手往後擺y
	double LArmBackZ = 0;		//左手往後擺z

	double RArmFinalX = 350;	//右手最後x
	double RArmFinalY = 0;		//右手最後y
	double RArmFinalZ = 335;	//右手最後z
	double LArmFinalX = 350;	//左手最後x
	double LArmFinalY = 0;		//左手最後y
	double LArmFinalZ = 335;	//左手最後z

	double RotStart[9];			//旋轉一開始
	double RotEnd[9];			//選轉結束
	double RotTemp[9];			//暫時的選轉矩陣
	double TempPos;				//算Quaternion暫時用的變數

	RotStart[0] = 0;
	RotStart[1] = 1;
	RotStart[2] = 0;
	RotStart[3] = 0;
	RotStart[4] = 0;
	RotStart[5] = -1;
	RotStart[6] = -1;
	RotStart[7] = 0;
	RotStart[8] = 0;

	RotEnd[0] = 1;
	RotEnd[1] = 0;
	RotEnd[2] = 0;
	RotEnd[3] = 0;
	RotEnd[4] = 0;
	RotEnd[5] = -1;
	RotEnd[6] = 0;
	RotEnd[7] = 1;
	RotEnd[8] = 0;

	for (int i = 0 ; i < gNumOfStep ; i++)
	{
		if (i == 0)//蹲下來的那一步
		{

			gKineAll.GenSmoothZMPShift_ZeroJerk(LArmBackX,LArmFrontX,gStepSample-300,gLArmWalkX);
			gKineAll.GenSmoothZMPShift_ZeroJerk(LArmBackY,LArmFrontY,gStepSample-300,gLArmWalkY);
			gKineAll.GenSmoothZMPShift_ZeroJerk(LArmBackZ,LArmFrontZ,gStepSample-300,gLArmWalkZ);
			gKineAll.GenSmoothZMPShift_ZeroJerk(RArmBackX,RArmFrontX,gStepSample-300,gRArmWalkX);
			gKineAll.GenSmoothZMPShift_ZeroJerk(RArmBackY,RArmFrontY,gStepSample-300,gRArmWalkY);
			gKineAll.GenSmoothZMPShift_ZeroJerk(RArmBackZ,RArmFrontZ,gStepSample-300,gRArmWalkZ);

			gKineAll.GenSmoothZMPShift_ZeroJerk(LArmFrontX,LArmFinalX,300,gLArmWalkX+900);
			gKineAll.GenSmoothZMPShift_ZeroJerk(LArmFrontY,LArmFinalY,300,gLArmWalkY+900);
			gKineAll.GenSmoothZMPShift_ZeroJerk(LArmFrontZ,LArmFinalZ,300,gLArmWalkZ+900);
			gKineAll.GenSmoothZMPShift_ZeroJerk(RArmFrontX,RArmFinalX,300,gRArmWalkX+900);
			gKineAll.GenSmoothZMPShift_ZeroJerk(RArmFrontY,RArmFinalY,300,gRArmWalkY+900);
			gKineAll.GenSmoothZMPShift_ZeroJerk(RArmFrontZ,RArmFinalZ,300,gRArmWalkZ+900);


			//KineAll.GenSmoothZMPShift_ZeroJerk(LArmBackX,LArmFrontX,StepSample-300,LArmWalkX);
			//KineAll.GenSmoothZMPShift_ZeroJerk(LArmBackY,LArmFrontY,StepSample-300,LArmWalkY);
			//KineAll.GenSmoothZMPShift_ZeroJerk(LArmBackZ,LArmFrontZ,StepSample-300,LArmWalkZ);
			//KineAll.GenSmoothZMPShift_ZeroJerk(RArmBackX,RArmFrontX,StepSample-300,RArmWalkX);
			//KineAll.GenSmoothZMPShift_ZeroJerk(RArmBackY,RArmFrontY,StepSample-300,RArmWalkY);
			//KineAll.GenSmoothZMPShift_ZeroJerk(RArmBackZ,RArmFrontZ,StepSample-300,RArmWalkZ);

			QuatInitMatrixSLERP(RotStart,RotEnd);
			TempPos = 1.0/(gStepSample-500);
			for(int j = 100 ; j < (gStepSample-400) ; j++)//蹲下時手不擺動
			{
				QuatMatrixSLERP(TempPos*(j+1-100),RotTemp);
				for(int k = 0 ; k < 9 ; k++)
				{
					gRArmWalkRot[j*9+k] = RotTemp[k];
					gLArmWalkRot[j*9+k] = RotTemp[k];
				}
			}
		}
		else if(i < (gNumOfStep-5))
		{
			for(int j = 0;j<1200;j++)
			{
				gLArmWalkX[gStepSample*i+j] = gLArmWalkX[gStepSample*i-1+j]+0.08;
				gLArmWalkY[gStepSample*i+j] = gLArmWalkY[gStepSample*i-1];
				gLArmWalkZ[gStepSample*i+j] = gLArmWalkZ[gStepSample*i-1];
				gRArmWalkX[gStepSample*i+j] = gRArmWalkX[gStepSample*i-1+j]+0.08;
				gRArmWalkY[gStepSample*i+j] = gRArmWalkY[gStepSample*i-1];
				gRArmWalkZ[gStepSample*i+j] = gRArmWalkZ[gStepSample*i-1];
			}
		}
		else
		{
			for(int j = 0;j<1200;j++)
			{
				gLArmWalkX[gStepSample*i+j] = gLArmWalkX[gStepSample*i-1+j];
				gLArmWalkY[gStepSample*i+j] = gLArmWalkY[gStepSample*i-1];
				gLArmWalkZ[gStepSample*i+j] = gLArmWalkZ[gStepSample*i-1];
				gRArmWalkX[gStepSample*i+j] = gRArmWalkX[gStepSample*i-1+j];
				gRArmWalkY[gStepSample*i+j] = gRArmWalkY[gStepSample*i-1];
				gRArmWalkZ[gStepSample*i+j] = gRArmWalkZ[gStepSample*i-1];
			}
		
		}

	}

}

void gInitRArmCarryWalkStraight(void)
{
	/******************************************************************
	input: void
	output: void

	Note:

	// 初始化用右手提東西之軌跡(左手還是會揮動)
	// 手部設定 外面環近參數設定 設定於此
	// 腳步必須先設定好之後才能使用
	// 相似的函數有 gInitArmWaveWalkStraight gInitArmPushCartWalkStraight gInitRArmCarryWalkStraight gInitArmWaveTurn

	******************************************************************/

	gDemoArmTrajFlag = 0;//用來判別arm不同traj用的(demo用的) 0代表直走 1代表左轉
	gIKMethod = 0;
	//走路時擺手軌跡
	double RArmFrontX = 0;	//右手往前擺x
	double RArmFrontY = -50;	//右手往前擺y
	double RArmFrontZ = 0;		//右手往前擺z
	double LArmFrontX = 200;	//左手往前擺x
	double LArmFrontY = -50;	//左手往前擺y
	double LArmFrontZ = 50;		//左手往前擺z



	double RArmBackX = 0;		//右手往後擺x
	double RArmBackY = 0;		//右手往後擺y
	double RArmBackZ = 0;		//右手往後擺z
	double LArmBackX = 0;		//左手往後擺x
	double LArmBackY = 0;		//左手往後擺y
	double LArmBackZ = 0;		//左手往後擺z

	double RArmFinalX = -30;		//右手拉起物品的x
	double RArmFinalY = -40;		//右手拉起物品的y
	double RArmFinalZ = 100;		//右手拉起物品的z

	for (int i = 0 ; i < gNumOfStep ; i++)
	{
		if (i == 0)
		{
			for(int j = 0 ; j < gStepSample ; j++)//蹲下時手不擺動
			{
				gLArmWalkX[j] = 0;
				gLArmWalkY[j] = 0;	
				gLArmWalkZ[j] = 0;
			}
			gKineAll.GenSmoothZMPShift_ZeroJerk(RArmBackX,RArmFrontX,600,gRArmWalkX);
			gKineAll.GenSmoothZMPShift_ZeroJerk(RArmBackY,RArmFrontY,600,gRArmWalkY);
			gKineAll.GenSmoothZMPShift_ZeroJerk(RArmBackZ,RArmFrontZ,600,gRArmWalkZ);

			gKineAll.GenSmoothZMPShift_ZeroJerk(RArmFrontX,RArmBackX,300,gRArmWalkX+600);
			gKineAll.GenSmoothZMPShift_ZeroJerk(RArmFrontY,15,300,gRArmWalkY+600);
			gKineAll.GenSmoothZMPShift_ZeroJerk(RArmFrontZ,-7,300,gRArmWalkZ+600);

			gKineAll.GenSmoothZMPShift_ZeroJerk(RArmBackX,RArmFinalX,300,gRArmWalkX+900);
			gKineAll.GenSmoothZMPShift_ZeroJerk(15,RArmFinalY,300,gRArmWalkY+900);
			gKineAll.GenSmoothZMPShift_ZeroJerk(-7,RArmFinalZ,300,gRArmWalkZ+900);


		}
		else if(i == 1)//踏第一步時只有一隻手擺動
		{
			if( gKineAll.selSupport[1] == 0)//左腳support
			{
				gKineAll.GenSmoothZMPShift_ZeroJerk(LArmBackX,LArmFrontX,gStepSample,gLArmWalkX+gStepSample);
				gKineAll.GenSmoothZMPShift_ZeroJerk(LArmBackY,LArmFrontY,gStepSample,gLArmWalkY+gStepSample);
				gKineAll.GenSmoothZMPShift_ZeroJerk(LArmBackZ,LArmFrontZ,gStepSample,gLArmWalkZ+gStepSample);
			}
			else//右腳support
			{

			}
		}
		else if(i < (gNumOfStep - 4))
		{
			if( gKineAll.selSupport[i] == 0)//左腳support
			{
				gKineAll.GenSmoothZMPShift_ZeroJerk(LArmBackX,LArmFrontX,gStepSample,gLArmWalkX+gStepSample*i);
				gKineAll.GenSmoothZMPShift_ZeroJerk(LArmBackY,LArmFrontY,gStepSample,gLArmWalkY+gStepSample*i);
				gKineAll.GenSmoothZMPShift_ZeroJerk(LArmBackZ,LArmFrontZ,gStepSample,gLArmWalkZ+gStepSample*i);


			}
			else//右腳support
			{


				gKineAll.GenSmoothZMPShift_ZeroJerk(LArmFrontX,LArmBackX,gStepSample,gLArmWalkX+gStepSample*i);
				gKineAll.GenSmoothZMPShift_ZeroJerk(LArmFrontY,LArmBackY,gStepSample,gLArmWalkY+gStepSample*i);
				gKineAll.GenSmoothZMPShift_ZeroJerk(LArmFrontZ,LArmBackZ,gStepSample,gLArmWalkZ+gStepSample*i);
			}
		}
		else
		{
			if( gKineAll.selSupport[i] == 0)//左腳support
			{

			}
			else//右腳support
			{
				gKineAll.GenSmoothZMPShift_ZeroJerk(LArmFrontX,LArmBackX,gStepSample,gLArmWalkX+gStepSample*i);
				gKineAll.GenSmoothZMPShift_ZeroJerk(LArmFrontY,LArmBackY,gStepSample,gLArmWalkY+gStepSample*i);
				gKineAll.GenSmoothZMPShift_ZeroJerk(LArmFrontZ,LArmBackZ,gStepSample,gLArmWalkZ+gStepSample*i);
			}
		}
		for(int j = 0;j < (gNumOfStep-1)*gStepSample;j++)
		{
			gRArmWalkX[gStepSample + j] = gRArmWalkX[gStepSample + j-1];
			gRArmWalkY[gStepSample + j] = gRArmWalkY[gStepSample + j-1];
			gRArmWalkZ[gStepSample + j] = gRArmWalkZ[gStepSample + j-1];
		}
	}



}

void gInitArmWaveTurn(int Direction)
{
	/******************************************************************
	input: void
	output: void

	Note:

	// 初始化轉彎時手部的揮動
	// 手部設定 外面環近參數設定 設定於此
	// 腳步必須先設定好之後才能使用
	// 相似的函數有 gInitArmWaveWalkStraight gInitArmPushCartWalkStraight gInitRArmCarryWalkStraight gInitArmWaveTurn

	******************************************************************/
	//泓逸start120223

	gTurnDirection = Direction;
	gDemoArmTrajFlag = 1;//用來判別arm不同traj用的(demo用的) 0代表直走 1代表左轉
	gIKMethod = 0;


	gLastAngRArm = gAngRArm;
	gLastAngLArm = gAngLArm;
	//走路時擺手軌跡
	double RArmFrontX = 150;	//右手往前擺x
	double RArmFrontY = 0;	//右手往前擺y
	double RArmFrontZ = 100;		//右手往前擺z
	double LArmFrontX = 150;	//左手往前擺x
	double LArmFrontY = 0;	//左手往前擺y
	double LArmFrontZ = 100;		//左手往前擺z

	//double RArmFrontX = 0;	//右手往前擺x
	//double RArmFrontY = 0;	//右手往前擺y
	//double RArmFrontZ = 0;		//右手往前擺z
	//double LArmFrontX = 0;	//左手往前擺x
	//double LArmFrontY = 0;	//左手往前擺y
	//double LArmFrontZ = 0;		//左手往前擺z

	double RArmBackX = 0;		//右手往後擺x
	double RArmBackY = 0;		//右手往後擺y
	double RArmBackZ = 0;		//右手往後擺z
	double LArmBackX = 0;		//左手往後擺x
	double LArmBackY = 0;		//左手往後擺y
	double LArmBackZ = 0;		//左手往後擺z

	gRArmWalkRot[0] = 0;
	gRArmWalkRot[1] = 1;
	gRArmWalkRot[2] = 0;
	gRArmWalkRot[3] = 0;
	gRArmWalkRot[4] = 0;
	gRArmWalkRot[5] = -1;
	gRArmWalkRot[6] = -1;
	gRArmWalkRot[7] = 0;
	gRArmWalkRot[8] = 0;

	gLArmWalkRot[0] = 0;
	gLArmWalkRot[1] = 1;
	gLArmWalkRot[2] = 0;
	gLArmWalkRot[3] = 0;
	gLArmWalkRot[4] = 0;
	gLArmWalkRot[5] = -1;
	gLArmWalkRot[6] = -1;
	gLArmWalkRot[7] = 0;
	gLArmWalkRot[8] = 0;

	for(int i = 0;i < (gNumOfStep-2)*gStepSample ; i++ )
	{
		for(int j = 0;j<9;j++)
		{
			gRArmWalkRot[i*9+j] = gRArmWalkRot[j];
			gLArmWalkRot[i*9+j] = gLArmWalkRot[j];
		}
	}

	for (int i = 0 ; i < gNumOfStep-4 ; i++)
	{
		if (i == 0)
		{
			for(int j = 0 ; j < gStepSample ; j++)//蹲下時手不擺動
			{
				gRArmWalkX[j] = 0;
				gRArmWalkY[j] = 0;
				gRArmWalkZ[j] = 0;
				gLArmWalkX[j] = 0;
				gLArmWalkY[j] = 0;	
				gLArmWalkZ[j] = 0;
			}
		}
		else if(i == 1)//踏第一步時只有一隻手擺動
		{
			if( gKineAll.selSupport[1] == 0)//左腳support
			{
				gKineAll.GenSmoothZMPShift_ZeroJerk(LArmBackX,LArmFrontX,gStepSample,gLArmWalkX+gStepSample);//左手往前擺
				gKineAll.GenSmoothZMPShift_ZeroJerk(LArmBackY,LArmFrontY,gStepSample,gLArmWalkY+gStepSample);
				gKineAll.GenSmoothZMPShift_ZeroJerk(LArmBackZ,LArmFrontZ,gStepSample,gLArmWalkZ+gStepSample);
				for(int j = gStepSample;j<gStepSample*2;j++)
				{
					gRArmWalkX[j] = 0;
					gRArmWalkY[j] = 0;
					gRArmWalkZ[j] = 0;
				}
			}
			else//右腳support
			{
				gKineAll.GenSmoothZMPShift_ZeroJerk(RArmBackX,RArmFrontX,gStepSample,gRArmWalkX+gStepSample);//右手往前擺
				gKineAll.GenSmoothZMPShift_ZeroJerk(RArmBackY,RArmFrontY,gStepSample,gRArmWalkY+gStepSample);
				gKineAll.GenSmoothZMPShift_ZeroJerk(RArmBackZ,RArmFrontZ,gStepSample,gRArmWalkZ+gStepSample);
				for(int j = gStepSample;j<gStepSample*2;j++)
				{
					gLArmWalkX[j] = 0;
					gLArmWalkY[j] = 0;
					gLArmWalkZ[j] = 0;
				}
			}
		}
		else if(i < (gNumOfStep - 5))
		{
			if( gKineAll.selSupport[i] == 0)//左腳support
			{
				gKineAll.GenSmoothZMPShift_ZeroJerk(LArmBackX,LArmFrontX,gStepSample,gLArmWalkX+gStepSample*i);//左手往前擺
				gKineAll.GenSmoothZMPShift_ZeroJerk(LArmBackY,LArmFrontY,gStepSample,gLArmWalkY+gStepSample*i);
				gKineAll.GenSmoothZMPShift_ZeroJerk(LArmBackZ,LArmFrontZ,gStepSample,gLArmWalkZ+gStepSample*i);

				gKineAll.GenSmoothZMPShift_ZeroJerk(RArmFrontX,RArmBackX,gStepSample,gRArmWalkX+gStepSample*i);//右手往後擺
				gKineAll.GenSmoothZMPShift_ZeroJerk(RArmFrontY,RArmBackY,gStepSample,gRArmWalkY+gStepSample*i);
				gKineAll.GenSmoothZMPShift_ZeroJerk(RArmFrontZ,RArmBackZ,gStepSample,gRArmWalkZ+gStepSample*i);
			}
			else//右腳support
			{
				gKineAll.GenSmoothZMPShift_ZeroJerk(RArmBackX,RArmFrontX,gStepSample,gRArmWalkX+gStepSample*i);//右手往前擺
				gKineAll.GenSmoothZMPShift_ZeroJerk(RArmBackY,RArmFrontY,gStepSample,gRArmWalkY+gStepSample*i);
				gKineAll.GenSmoothZMPShift_ZeroJerk(RArmBackZ,RArmFrontZ,gStepSample,gRArmWalkZ+gStepSample*i);

				gKineAll.GenSmoothZMPShift_ZeroJerk(LArmFrontX,LArmBackX,gStepSample,gLArmWalkX+gStepSample*i);//左手往後擺
				gKineAll.GenSmoothZMPShift_ZeroJerk(LArmFrontY,LArmBackY,gStepSample,gLArmWalkY+gStepSample*i);
				gKineAll.GenSmoothZMPShift_ZeroJerk(LArmFrontZ,LArmBackZ,gStepSample,gLArmWalkZ+gStepSample*i);
			}
		}
		else
		{
			if( gKineAll.selSupport[i] == 0)//左腳support
			{
				gKineAll.GenSmoothZMPShift_ZeroJerk(RArmFrontX,RArmBackX,gStepSample,gRArmWalkX+gStepSample*i);//右手往後擺
				gKineAll.GenSmoothZMPShift_ZeroJerk(RArmFrontY,RArmBackY,gStepSample,gRArmWalkY+gStepSample*i);
				gKineAll.GenSmoothZMPShift_ZeroJerk(RArmFrontZ,RArmBackZ,gStepSample,gRArmWalkZ+gStepSample*i);
				for(int j = gStepSample*i;j<gStepSample*(i+1);j++)
				{
					gLArmWalkX[j] = 0;
					gLArmWalkY[j] = 0;
					gLArmWalkZ[j] = 0;
				}
			}
			else//右腳support
			{
				gKineAll.GenSmoothZMPShift_ZeroJerk(LArmFrontX,LArmBackX,gStepSample,gLArmWalkX+gStepSample*i);//左手往後擺
				gKineAll.GenSmoothZMPShift_ZeroJerk(LArmFrontY,LArmBackY,gStepSample,gLArmWalkY+gStepSample*i);
				gKineAll.GenSmoothZMPShift_ZeroJerk(LArmFrontZ,LArmBackZ,gStepSample,gLArmWalkZ+gStepSample*i);
				for(int j = gStepSample*i;j<gStepSample*(i+1);j++)
				{
					gRArmWalkX[j] = 0;
					gRArmWalkY[j] = 0;
					gRArmWalkZ[j] = 0;
				}
			}
			for(int j = gStepSample*(i+1);j<LQSIBufferSize;j++)
			{
				gRArmWalkX[j] = 0;
				gRArmWalkY[j] = 0;
				gRArmWalkZ[j] = 0;
				gLArmWalkX[j] = 0;
				gLArmWalkY[j] = 0;	
				gLArmWalkZ[j] = 0;
			
			}
		}
	}
	

	//for(int i = 0;i<20000;i++)
	//{
	//	TestDeg[TestPos] = gLArmWalkX[i];
	//	TestPos ++;
	//	TestDeg[TestPos] = gLArmWalkY[i];
	//	TestPos ++;
	//	TestDeg[TestPos] = gLArmWalkZ[i];
	//	TestPos ++;
	//	TestDeg[TestPos] = gRArmWalkX[i];
	//	TestPos ++;
	//	TestDeg[TestPos] = gRArmWalkY[i];
	//	TestPos ++;
	//	TestDeg[TestPos] = gRArmWalkZ[i];
	//	TestPos ++;
	//}

	//fstream ccc;
	//ccc.open("testpos1.txt",ios::out);
	//for(int i = 0;i < TestPos;i++)
	//{
	//	ccc<<TestDeg[i]<<"     ";
	//	if(i%6 == 5)
	//	{
	//		ccc<<endl;
	//	}
	//}
	//ccc.close();
	//TestPos = 0;
	//泓逸end120223


}

void gInitArmSideWalkWave(int Direction)
{
	/******************************************************************
	input: Direction
	output: void

	Note:

	// 初始化側向行走時手部的揮動
	// 手部設定 外面環近參數設定 設定於此
	// 腳步必須先設定好之後才能使用
	// 相似的函數有 gInitArmWaveWalkStraight gInitArmPushCartWalkStraight gInitRArmCarryWalkStraight gInitArmWaveTurn

	******************************************************************/

	gDemoArmTrajFlag = 0;//用來判別arm不同traj用的(demo用的) 0代表直走 1代表左轉
	gIKMethod = 0;

	double RArmFrontX = 350;	//右手往前擺x
	double RArmFrontY = 0;		//右手往前擺y
	double RArmFrontZ = 300;	//右手往前擺z
	double LArmFrontX = 350;	//左手往前擺x
	double LArmFrontY = 0;		//左手往前擺y
	double LArmFrontZ = 300;	//左手往前擺z


	double RArmBackX = 0;		//右手往後擺x
	double RArmBackY = 0;		//右手往後擺y
	double RArmBackZ = 0;		//右手往後擺z
	double LArmBackX = 0;		//左手往後擺x
	double LArmBackY = 0;		//左手往後擺y
	double LArmBackZ = 0;		//左手往後擺z



	double RotStart[9];			//旋轉一開始
	double RotEnd[9];			//選轉結束
	double RotTemp[9];			//暫時的選轉矩陣
	double TempPos;				//算Quaternion暫時用的變數

	RotStart[0] = 0;
	RotStart[1] = 1;
	RotStart[2] = 0;
	RotStart[3] = 0;
	RotStart[4] = 0;
	RotStart[5] = -1;
	RotStart[6] = -1;
	RotStart[7] = 0;
	RotStart[8] = 0;

	RotEnd[0] = 1;
	RotEnd[1] = 0;
	RotEnd[2] = 0;
	RotEnd[3] = 0;
	RotEnd[4] = 0;
	RotEnd[5] = -1;
	RotEnd[6] = 0;
	RotEnd[7] = 1;
	RotEnd[8] = 0;

	for (int i = 0 ; i < gNumOfStep ; i++)
	{
		if (i == 0)//蹲下來的那一步
		{

			gKineAll.GenSmoothZMPShift_ZeroJerk(LArmBackX,LArmFrontX,gStepSample,gLArmWalkX);
			gKineAll.GenSmoothZMPShift_ZeroJerk(LArmBackY,LArmFrontY,gStepSample,gLArmWalkY);
			gKineAll.GenSmoothZMPShift_ZeroJerk(LArmBackZ,LArmFrontZ,gStepSample,gLArmWalkZ);
			gKineAll.GenSmoothZMPShift_ZeroJerk(RArmBackX,RArmFrontX,gStepSample,gRArmWalkX);
			gKineAll.GenSmoothZMPShift_ZeroJerk(RArmBackY,RArmFrontY,gStepSample,gRArmWalkY);
			gKineAll.GenSmoothZMPShift_ZeroJerk(RArmBackZ,RArmFrontZ,gStepSample,gRArmWalkZ);




			QuatInitMatrixSLERP(RotStart,RotEnd);
			TempPos = 1.0/(gStepSample-400);
			for(int j = 100 ; j < (gStepSample-300) ; j++)//蹲下時手不擺動(gStepSample-400)如果要修改下面也要跟著改
			{
				QuatMatrixSLERP(TempPos*(j+1-100),RotTemp);
				for(int k = 0 ; k < 9 ; k++)
				{
					gRArmWalkRot[j*9+k] = RotTemp[k];
					gLArmWalkRot[j*9+k] = RotTemp[k];
				}
			}
		}

		else if(i < (gNumOfStep-5))
		{
			for(int j = 0;j<1200;j++)
			{
				gLArmWalkX[gStepSample*i+j] = gLArmWalkX[gStepSample*i-1+j];
				gLArmWalkY[gStepSample*i+j] = gLArmWalkY[gStepSample*i-1];
				gLArmWalkZ[gStepSample*i+j] = gLArmWalkZ[gStepSample*i-1];
				gRArmWalkX[gStepSample*i+j] = gRArmWalkX[gStepSample*i-1+j];
				gRArmWalkY[gStepSample*i+j] = gRArmWalkY[gStepSample*i-1];
				gRArmWalkZ[gStepSample*i+j] = gRArmWalkZ[gStepSample*i-1];
			}
		}
		else
		{
			for(int j = 0;j<2400;j++)
			{
				gLArmWalkX[gStepSample*i+j] = gLArmWalkX[gStepSample*i-1+j];
				gLArmWalkY[gStepSample*i+j] = gLArmWalkY[gStepSample*i-1];
				gLArmWalkZ[gStepSample*i+j] = gLArmWalkZ[gStepSample*i-1];
				gRArmWalkX[gStepSample*i+j] = gRArmWalkX[gStepSample*i-1+j];
				gRArmWalkY[gStepSample*i+j] = gRArmWalkY[gStepSample*i-1];
				gRArmWalkZ[gStepSample*i+j] = gRArmWalkZ[gStepSample*i-1];
			}
		
		}

	}
	for(int i = (gStepSample-300);i<gStepSample*(gNumOfStep-2);i++)
	{
		for(int j = 0;j<9;j++)
		{
			gRArmWalkRot[i*9+j] = RotEnd[j];
			gLArmWalkRot[i*9+j] = RotEnd[j];
		}
	}



}

void gInitArmSumoMotion()
{
	/******************************************************************
	input: void
	output: void

	Note:

	// 初始化機器人行為為相撲伸展的手的動作
	// 手部動作 設定於此
	// 相似的函數有  gInitArmWaveWalkStraight gInitArmPushCartWalkStraight gInitRArmCarryWalkStraight gInitArmWaveTurn

	******************************************************************/

	gDemoArmTrajFlag = 0;//用來判別arm不同traj用的(demo用的) 0代表直走 1代表左轉
	gIKMethod = 1;
	double ArmDegOfflineStart[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
	//double ArmDegOfflineEndFirst[12] = {0,1.47,0,0,0,0,0,1.07,0,0,0,0};
	//double ArmDegOfflineEndSecond[12] = {0,1.07,0,0,0,0,0,1.47,0,0,0,0};
	double ArmDegOfflineEndFirst[12] = {0,1.1,0,0,0,0,0,1.1,0,0,0,0};
	double ArmDegOfflineEndSecond[12] = {0,1.0,0,0,0,0,0,1.1,0,0,0,0};
	double TempDataRArm[10000];
	double TempDataLArm[10000];
	

	for (int i = 0 ; i < gNumOfStep ; i++)
	{
		if (i == 0)//蹲下來的那一步
		{
			for(int j = 0;j < gStepSample*6;j++)
			{
				gRArmOfflineTraj[j] = 0;
				gLArmOfflineTraj[j] = 0;
			}

		}
		else if(i == 1)
		{
			for(int j = 0;j<6;j++)
			{
				gKineAll.GenSmoothZMPShift_ZeroJerk(ArmDegOfflineStart[j],ArmDegOfflineEndFirst[j],gStepSample,TempDataRArm);
				gKineAll.GenSmoothZMPShift_ZeroJerk(ArmDegOfflineStart[j+6],ArmDegOfflineEndFirst[j+6],gStepSample,TempDataLArm);
				for(int k = 0;k<gStepSample;k++)
				{
					gRArmOfflineTraj[i*6*gStepSample + k*6 + j] = TempDataRArm[k];
					gLArmOfflineTraj[i*6*gStepSample + k*6 + j] = TempDataLArm[k];
				}
			}
		}
		else if(i == 2 )
		{
			for(int j = 0;j<6*2*gStepSample;j++)
			{
				gRArmOfflineTraj[i*6*gStepSample + j] = gRArmOfflineTraj[i*6*gStepSample + j - 6];
				gLArmOfflineTraj[i*6*gStepSample + j] = gLArmOfflineTraj[i*6*gStepSample + j - 6];
			}
		}
		else if(i == 3)
		{			
			for(int j = 0;j<6;j++)
			{
				gKineAll.GenSmoothZMPShift_ZeroJerk(ArmDegOfflineEndFirst[j],ArmDegOfflineStart[j],gStepSample,TempDataRArm);
				gKineAll.GenSmoothZMPShift_ZeroJerk(ArmDegOfflineEndFirst[j+6],ArmDegOfflineStart[j+6],gStepSample,TempDataLArm);
				for(int k = 0;k<gStepSample;k++)
				{
					gRArmOfflineTraj[i*6*gStepSample + k*6 + j] = TempDataRArm[k];
					gLArmOfflineTraj[i*6*gStepSample + k*6 + j] = TempDataLArm[k];
				}
			}
		
		}
		else if(i == 4)
		{
			for(int j = 0;j<6;j++)
			{
				gKineAll.GenSmoothZMPShift_ZeroJerk(ArmDegOfflineStart[j],ArmDegOfflineEndSecond[j],gStepSample,TempDataRArm);
				gKineAll.GenSmoothZMPShift_ZeroJerk(ArmDegOfflineStart[j+6],ArmDegOfflineEndSecond[j+6],gStepSample,TempDataLArm);
				for(int k = 0;k<gStepSample;k++)
				{
					gRArmOfflineTraj[i*6*gStepSample + k*6 + j] = TempDataRArm[k];
					gLArmOfflineTraj[i*6*gStepSample + k*6 + j] = TempDataLArm[k];
				}
			}
		}
		else if(i == 5 )
		{
			for(int j = 0;j<6*2*gStepSample;j++)
			{
				gRArmOfflineTraj[i*6*gStepSample + j] = gRArmOfflineTraj[i*6*gStepSample + j - 6];
				gLArmOfflineTraj[i*6*gStepSample + j] = gLArmOfflineTraj[i*6*gStepSample + j - 6];
			}
		}
		else if(i == 6)
		{			
			for(int j = 0;j<6;j++)
			{
				gKineAll.GenSmoothZMPShift_ZeroJerk(ArmDegOfflineEndSecond[j],ArmDegOfflineStart[j],gStepSample,TempDataRArm);
				gKineAll.GenSmoothZMPShift_ZeroJerk(ArmDegOfflineEndSecond[j+6],ArmDegOfflineStart[j+6],gStepSample,TempDataLArm);
				for(int k = 0;k<gStepSample;k++)
				{
					gRArmOfflineTraj[i*6*gStepSample + k*6 + j] = TempDataRArm[k];
					gLArmOfflineTraj[i*6*gStepSample + k*6 + j] = TempDataLArm[k];
				}
			}
		
		}

	}

	for(int i = gNumOfStep*gStepSample;i>1;i--)
	{
		for(int j = 0;j<6 ;j++)
		{
			gRArmOfflineTraj[i*6 + j] -= gRArmOfflineTraj[i*6 + j -6];
			gLArmOfflineTraj[i*6 + j] -= gLArmOfflineTraj[i*6 + j -6];
		}
	}





}

void gInitArmIntroductionMode(double Time,int Mode,int Count)
{
	/******************************************************************
	input: time
	output: void

	Note:

	// 初始化機器人自我介紹的動作
	// 手部動作 設定於此
	// 相似的函數有  gInitArmWaveWalkStraight gInitArmPushCartWalkStraight gInitRArmCarryWalkStraight gInitArmWaveTurn

	******************************************************************/

	double position[10000];
	int gIKMethod = 1;
	int DataLength = 0;
	int DataPos = 0;

	int DataCount = Count;
	
	double TempDataRArm01[1000];
	double TempDataRArm02[1000];
	double TempDataRArm03[1000];
	double TempDataRArm04[1000];
	double TempDataRArm05[1000];
	double TempDataRArm06[1000];
	double TempDataLArm01[1000];
	double TempDataLArm02[1000];
	double TempDataLArm[1000];
	double TempDataRArm[30000];

	// 12組手臂歸零用暫存區
	//double TempDataRARM01[2000];
	//double TempDataRARM02[2000];
	//double TempDataRARM03[2000];
	//double TempDataRARM04[2000];
	//double TempDataRARM05[2000];
	//double TempDataRARM06[2000];
	//double TempDataLARM01[2000];
	//double TempDataLARM02[2000];
	//double TempDataLARM03[2000];
	//double TempDataLARM04[2000];
	//double TempDataLARM05[2000];
	//double TempDataLARM06[2000];
	// ****************************
	//至峻手掌軌跡開始 0528 
	gFlagHandStart=1;

	DataPos = 0;

	gArmOfflineTrajCount = 0;

	fstream OfflineData;
	fstream OfflineData2;
	//******ARM Traj 檔案輸入用
	int TrajLength = 38392;//38392;//軌跡檔SAMPLE數
	int InitLength = 1000;//初始化SAMPLE數
	int PNLHand[6] = {1,1,1,1,1,-1};//左手正負組態係數
	int PNRHand[6] = {-1,-1,-1,-1,-1,-1};//右手正負組態係數
	//******

	if(Mode == 1)	
		OfflineData.open("offline_data.txt",ios::in);
	else if(Mode == 2)
		OfflineData.open("offline_data2.txt",ios::in);
	else if(Mode == 3)
	{

		#if ArmDH

		if(gFlagGoBackPos==ShiftZeroHome ||gFlagGoBackPos==ZeroHome)
		{
				gKineAll.GenSmoothZMPShift_ZeroJerk(0,5,InitLength,TempDataLArm);
				gKineAll.GenSmoothZMPShift_ZeroJerk(0,-0.16,InitLength,TempDataRArm01);
				gKineAll.GenSmoothZMPShift_ZeroJerk(0,5,InitLength,TempDataRArm02);//5.04
				gKineAll.GenSmoothZMPShift_ZeroJerk(0,90,InitLength,TempDataRArm03); //左手90 右手90
				gKineAll.GenSmoothZMPShift_ZeroJerk(0,5.26,InitLength,TempDataRArm04);
				gKineAll.GenSmoothZMPShift_ZeroJerk(0,90,InitLength,TempDataRArm05); //左手-90 右手90
				gKineAll.GenSmoothZMPShift_ZeroJerk(0,0.09,InitLength,TempDataRArm06);
		}
		else if(gFlagGoBackPos==BentHome)
		{
				gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.FKLArm->theta[4]/3.1415926*180,0,InitLength,TempDataLArm01);
				gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.FKLArm->theta[5]/3.1415926*180,5,InitLength,TempDataLArm02);
				gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.FKLArm->theta[7]/3.1415926*180,5,InitLength,TempDataLArm);
				
				gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.FKRArm->theta[4]/3.1415926*180,0,InitLength,TempDataRArm01);
				gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.FKRArm->theta[5]/3.1415926*180,5,InitLength,TempDataRArm02);//5.04
				gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.FKRArm->theta[6]/3.1415926*180,90,InitLength,TempDataRArm03); //左手90 右手90
				gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.FKRArm->theta[7]/3.1415926*180,5.26,InitLength,TempDataRArm04);
				gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.FKRArm->theta[8]/3.1415926*180,90,InitLength,TempDataRArm05); //左手-90 右手90
				gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.FKRArm->theta[9]/3.1415926*180,0.09,InitLength,TempDataRArm06);	
			//for (int jp = 0 ; jp < 6 ; jp++)
			//{
			//	if(jp==0)
			//	{
			//		for(int i=0;i<InitLength;i++)
			//		{
			//		TempDataRArm01[i]=(TempDataRArm01[i]-5)*-1;
			//		}
			//	}
			//	else if(jp==1)
			//	{
			//		for(int i=0;i<InitLength;i++)
			//		{
			//			TempDataLArm[i] = gInitThetas[jp+14]/3.1415926*180;
			//			TempDataRArm02[i]=gInitThetas[jp+20]/3.1415926*180;
			//		}
			//	}
			//	else if(jp==3)
			//		{
			//			for(int i=0;i<InitLength;i++)
			//			{
			//			TempDataRArm04[i]=gInitThetas[jp+20]/3.1415926*180*-1;			
			//			}
			//		}
			//	else
			//		{
			//			for(int i=0;i<InitLength;i++)
			//			{
			//				//TempDataRArm01[i]=0;
			//				//TempDataRArm03[i]=90;
			//				//TempDataRArm05[i]=90;
			//				TempDataRArm06[i]=0;
			//			}
			//		}
			//}
		}
		#else
		gKineAll.GenSmoothZMPShift_ZeroJerk(0,5,InitLength,TempDataLArm);
		gKineAll.GenSmoothZMPShift_ZeroJerk(0,-0.16,InitLength,TempDataRArm01);
		gKineAll.GenSmoothZMPShift_ZeroJerk(0,5,InitLength,TempDataRArm02);//5.04
		gKineAll.GenSmoothZMPShift_ZeroJerk(0,-0.27,InitLength,TempDataRArm03);
		gKineAll.GenSmoothZMPShift_ZeroJerk(0,5.26,InitLength,TempDataRArm04);
		gKineAll.GenSmoothZMPShift_ZeroJerk(0,0.3,InitLength,TempDataRArm05);
		gKineAll.GenSmoothZMPShift_ZeroJerk(0,0.09,InitLength,TempDataRArm06);		

		#endif
		for(int i=0;i<InitLength;i++)//此段先插補5度的初始角度給手臂1000SAMPLE內
			for(int j=0;j<6;j++)
			{
				#if ArmDH
					if(j==1||j==3)
					{
						if(gFlagGoBackPos==ShiftZeroHome ||gFlagGoBackPos==ZeroHome)
						gLArmOfflineTraj[6*i+j]=TempDataLArm[i]*3.1415926/180;
						else if(gFlagGoBackPos==BentHome)
						{
							if(j==3)
							gLArmOfflineTraj[6*i+j]=TempDataLArm[i]*3.1415926/180;
							else
							{
							gLArmOfflineTraj[6*i+j]=TempDataLArm02[i]*3.1415926/180;	
							}
						
						}
					}
					else if (j==0 && gFlagGoBackPos==BentHome)
						gLArmOfflineTraj[6*i+j]=TempDataLArm01[i]*3.1415926/180;
					else if(j==2)
						gLArmOfflineTraj[6*i+j]=TempDataRArm03[i]*3.1415926/180;
					else if(j==4)
						gLArmOfflineTraj[6*i+j]=TempDataRArm05[i]*3.1415926/180*-1;
					else
						gLArmOfflineTraj[6*i+j]=0;
				#else
					if(j==1||j==3)
						gLArmOfflineTraj[6*i+j]=TempDataLArm[i]*3.1415926/180;
					else
						gLArmOfflineTraj[6*i+j]=0;
				#endif
			}
		for(int i=0;i<InitLength;i++)
			//for(int j=0;j<6;j++)
			{
				//if(j==1||j==3)
					gRArmOfflineTraj[6*i+0]=TempDataRArm01[i]*3.1415926/180*-1;
					gRArmOfflineTraj[6*i+1]=TempDataRArm02[i]*3.1415926/180*-1;
					gRArmOfflineTraj[6*i+2]=TempDataRArm03[i]*3.1415926/180*-1;
					gRArmOfflineTraj[6*i+3]=TempDataRArm04[i]*3.1415926/180*-1;
					gRArmOfflineTraj[6*i+4]=TempDataRArm05[i]*3.1415926/180;
					gRArmOfflineTraj[6*i+5]=TempDataRArm06[i]*3.1415926/180*-1;
				/*else
					gRArmOfflineTraj[6*i+j]=0;*/
			}
	}

	if(Mode == 3)
	{

		//OfflineData.open("mod_LAJointData.txt",ios::in);//open left arm file
		////OfflineData2.open("RightArmTrajTest.txt",ios::in);
		//for(int i = 0;i <TrajLength; i++)
		//{
		//	for(int j=0;j<6;j++)//22為手臂FILE資料格式 前6筆為手臂角度
		//	{
		//		OfflineData>>gHandTrajFileBuf[i][j];
		//	//OfflineData2>>temptraj2[i][j];
		//	}
		//}
		for(int i=0;i<TrajLength;i++)
		{
			for(int j=0;j<6;j++)
			{
				gLArmOfflineTraj[(2*i)*6+j+InitLength*6] = gLHandTrajFileBuf[i][j]/180*3.1415926 * PNLHand[j];
			}
		}
		for(int i=0;i<TrajLength;i++)
		{
			for(int j=0;j<6;j++)
			{
				if(i == (TrajLength-1))
				{
					gLArmOfflineTraj[(2*i+1)*6+j+InitLength*6] = gLArmOfflineTraj[(2*i)*6+j+InitLength*6];
				}
				else
				{
					gLArmOfflineTraj[(2*i+1)*6+j+InitLength*6] = (gLArmOfflineTraj[(2*i)*6+j+InitLength*6] + gLArmOfflineTraj[(2*i+2)*6+j+InitLength*6])/2;
				}
			}
		}
		//OfflineData.close();
		//OfflineData.open("mod_RAJointData.txt",ios::in);
		//for(int i = 0;i <TrajLength; i++)
		//{
		//	//for(int j=0;j<22;j++)
		//	for(int j=0;j<6;j++)
		//	{
		//		OfflineData>>gHandTrajFileBuf[i][j];		
		//	}
		//}
		for(int i=0;i<TrajLength;i++)
		{
			for(int j=0;j<6;j++)
			{
				gRArmOfflineTraj[(2*i)*6+j+InitLength*6] = gRHandTrajFileBuf[i][j]/180*3.1415926* PNRHand[j];
				//gRArmOfflineTraj[(2*i+1)*6+j+InitLength*6] = gHandTrajFileBuf[i][j]/180*3.1415926* PNRHand[j];
			}
		}	
		for(int i=0;i<TrajLength;i++)
		{
			for(int j=0;j<6;j++)
			{
				if(i == (TrajLength-1))
				{
					gRArmOfflineTraj[(2*i+1)*6+j+InitLength*6] = gRArmOfflineTraj[(2*i)*6+j+InitLength*6];
				}
				else
				{
					gRArmOfflineTraj[(2*i+1)*6+j+InitLength*6] = (gRArmOfflineTraj[(2*i)*6+j+InitLength*6] + gRArmOfflineTraj[(2*i+2)*6+j+InitLength*6])/2;
				}
			}
		}
		gKineAll.GenSmoothZMPShift_ZeroJerk(90,0,1000,TempDataRArm03); //左手90 右手90
		//gKineAll.GenSmoothZMPShift_ZeroJerk(0,5.26,InitLength,TempDataRArm04);
		gKineAll.GenSmoothZMPShift_ZeroJerk(90,0,1000,TempDataRArm05); //左手-90 右手90



		for(int i=0;i<1000;i++)
		{
			for(int j=0;j<6;j++)
			{
				if(j==2)
					gRArmOfflineTraj[TrajLength*2*6+InitLength*6+6*i+j] = TempDataRArm03[i]*3.1415926/180*-1;
				else if (j==4)
					gRArmOfflineTraj[TrajLength*2*6+InitLength*6+6*i+j] = TempDataRArm05[i]*3.1415926/180;	

				if(j==2)
					gLArmOfflineTraj[TrajLength*2*6+InitLength*6+6*i+j] = TempDataRArm03[i]*3.1415926/180;
				else if (j==4)
					gLArmOfflineTraj[TrajLength*2*6+InitLength*6+6*i+j] = TempDataRArm05[i]*3.1415926/180*-1;
			}		
		
		}
		//OfflineData.close();	
		// enforce to home position traj generator
		/*gKineAll.GenSmoothZMPShift_ZeroJerk(0,0,InitLength,TempDataLArm);
		gKineAll.GenSmoothZMPShift_ZeroJerk(0,0,InitLength,TempDataLArm);
		gKineAll.GenSmoothZMPShift_ZeroJerk(0,0,InitLength,TempDataLArm);
		gKineAll.GenSmoothZMPShift_ZeroJerk(0,0,InitLength,TempDataLArm);
		gKineAll.GenSmoothZMPShift_ZeroJerk(0,0,InitLength,TempDataLArm);
		gKineAll.GenSmoothZMPShift_ZeroJerk(0,0,InitLength,TempDataLArm);

		gKineAll.GenSmoothZMPShift_ZeroJerk(0,0,InitLength,TempDataLArm);
		gKineAll.GenSmoothZMPShift_ZeroJerk(0,0,InitLength,TempDataLArm);
		gKineAll.GenSmoothZMPShift_ZeroJerk(0,0,InitLength,TempDataLArm);
		gKineAll.GenSmoothZMPShift_ZeroJerk(0,0,InitLength,TempDataLArm);
		gKineAll.GenSmoothZMPShift_ZeroJerk(0,0,InitLength,TempDataLArm);
		gKineAll.GenSmoothZMPShift_ZeroJerk(0,0,InitLength,TempDataLArm);*/
	}
	else//mode 1 or 2
	{
		for(int i = 0;i < (DataCount*13+13);i++)
		{
			OfflineData>>position[i];
		}
		OfflineData.close();


	for(int i = 0;i<DataCount+1;i++)
	{
		for(int j = 0;j<12;j++)
		{
			position[i*13+j] = position[i*13+j]/180*3.1415926;
		}
	}
	


	for(int k = 0;k<DataCount;k++)
	{
		for(int i = 0;i<6;i++)
		{
			DataLength = (int)(position[k*13+13+12]);
			gKineAll.GenSmoothZMPShift_ZeroJerk(position[k*13+i],position[k*13+13+i],DataLength,TempDataLArm);
			gKineAll.GenSmoothZMPShift_ZeroJerk(position[k*13+i+6],position[k*13+13+i+6],DataLength,TempDataRArm);
			for(int j = 0;j<DataLength;j++)
			{
				gRArmOfflineTraj[DataPos*6 + j*6 + i] = TempDataRArm[j];
				gLArmOfflineTraj[DataPos*6 + j*6 + i] = TempDataLArm[j];

			}
		}
		DataPos += DataLength;
	}

	for(int i = 0;i<(Time*200-DataPos)*6+20000;i++)
	{
		gRArmOfflineTraj[DataPos*6+i] = 0;
		gLArmOfflineTraj[DataPos*6+i] = 0;
	}
	
	}
}
//泓逸end20120309


void gInitArmManualWalkStraight(int StepInput)
{
	/******************************************************************
	input:  int StepInput
	output: void

	Note:

	// 初始化機器人自我介紹的動作
	// 手部動作 設定於此, 目的在於嘗試讓手臂在手臂組Initial configuration下可以進行邊行走的擺動
	// 相似的函數有  gInitArmWaveWalkStraight gInitArmPushCartWalkStraight gInitRArmCarryWalkStraight gInitArmWaveTurn
	
	修改建議:

	// Ps1: 函數參數化程度不足(如只能配合六秒一步或固定步數) 且Manually輸入手臂軌跡時需注意相關Flag的開關
	// Ps2: 手臂雖可擺動 但動作自然程度仍須調整
	// Ps3: 複數以上的scenario若是用到同一buffer需注意軌跡初始化問題

	******************************************************************/

	//double position[10000];
	//gKineAll.FlagStayMode = 0;

	int fullstepnumber=StepInput-6;

	gIKMethod = 1;
	ArmOfflinMethod = 1 ;
	//int DataLength = 0;
	int DataPos = 0;

	//int DataCount = Count;
	//
	double TempDataRArm[2000];
	double TempDataLArm[2000];
	double TempDataRArm2[2000];
	double TempDataLArm2[2000];

	double AngleElbow=0;
	double AngleShPitch=15;

// count 0-1200 reserved
	DataPos=0;

	for(int i=0;i<1200;i++)
	{
		for(int j=0;j<6;j++)
		{

			gRArmOfflineTraj[DataPos*6+6*i+j]=gInitThetas[j+20]*-1;
			gLArmOfflineTraj[DataPos*6+6*i+j]=gInitThetas[j+14];

		}
	}

//count 0-1200 start!! (Right)

		DataPos=1200;
	for(int i=0;i<6;i++)
	{
		if(i==0)
		gKineAll.GenSmoothZMPShift_ZeroJerk(gInitThetas[i+20]*-1*180/3.1415926535,AngleShPitch*-1,1200,TempDataRArm);
		if(i==3)
		gKineAll.GenSmoothZMPShift_ZeroJerk(gInitThetas[i+20]*-1*180/3.1415926535,AngleElbow,1200,TempDataRArm2);
	}

	
	for(int i=0;i<1200;i++)
	{
		for(int j=0;j<6;j++)
		{
			if(j==0)
			gRArmOfflineTraj[DataPos*6+6*i+j]=TempDataRArm[i]/180*3.1415926535;
			else if(j==3)
			gRArmOfflineTraj[DataPos*6+6*i+j]=TempDataRArm2[i]/180*3.1415926535;
			else
			gRArmOfflineTraj[DataPos*6+6*i+j]=gInitThetas[j+20]*-1;//gKineAll.FKRArm->theta[j+4];
			gLArmOfflineTraj[DataPos*6+6*i+j]=gInitThetas[j+14];//gKineAll.FKLArm->theta[j+4];

		}
	}
	
//count 0-1200 2nd (Left)

		DataPos=2400;
	for(int i=0;i<6;i++)
	{
		if(i==0)
		{
		gKineAll.GenSmoothZMPShift_ZeroJerk(AngleShPitch*-1,gInitThetas[i+20]*-1*180/3.1415926535,1200,TempDataRArm);
		gKineAll.GenSmoothZMPShift_ZeroJerk(gInitThetas[i+14]*180/3.1415926535,AngleShPitch*-1,1200,TempDataLArm);
		}

		if(i==3)
		{
		gKineAll.GenSmoothZMPShift_ZeroJerk(AngleElbow,gInitThetas[i+20]*-1*180/3.1415926535,1200,TempDataRArm2);
		gKineAll.GenSmoothZMPShift_ZeroJerk(gInitThetas[i+14]*180/3.1415926535,AngleElbow,1200,TempDataLArm2);
		}

	}

	for(int i=0;i<1200;i++)
	{
		for(int j=0;j<6;j++)
		{
			if(j==0)
			{
			gRArmOfflineTraj[DataPos*6+6*i+j]=TempDataRArm[i]/180*3.1415926535;
			gLArmOfflineTraj[DataPos*6+6*i+j]=TempDataLArm[i]/180*3.1415926535;
			}
			else if(j==3)
			{
			gRArmOfflineTraj[DataPos*6+6*i+j]=TempDataRArm2[i]/180*3.1415926535;
			gLArmOfflineTraj[DataPos*6+6*i+j]=TempDataLArm2[i]/180*3.1415926535;
			}
			else
			{
			gRArmOfflineTraj[DataPos*6+6*i+j]=gInitThetas[j+20]*-1;//gKineAll.FKRArm->theta[j+4];
			gLArmOfflineTraj[DataPos*6+6*i+j]=gInitThetas[j+14];//gKineAll.FKLArm->theta[j+4];
			}
		}
	}

//count 0-1200 3rd 

		DataPos=3600;
	for(int i=0;i<6;i++)
	{
		if(i==0)
		{
		gKineAll.GenSmoothZMPShift_ZeroJerk(gInitThetas[i+20]*-1*180/3.1415926535,AngleShPitch*-1,1200,TempDataRArm);
		gKineAll.GenSmoothZMPShift_ZeroJerk(AngleShPitch*-1,gInitThetas[i+14]*180/3.1415926535,1200,TempDataLArm);
		}

		if(i==3)
		{
		gKineAll.GenSmoothZMPShift_ZeroJerk(gInitThetas[i+20]*-1*180/3.1415926535,AngleElbow,1200,TempDataRArm2);
		gKineAll.GenSmoothZMPShift_ZeroJerk(AngleElbow,gInitThetas[i+14]*180/3.1415926535,1200,TempDataLArm2);
		}

	}
	for(int i=0;i<1200;i++)
	{
		for(int j=0;j<6;j++)
		{
			if(j==0)
			{
			gRArmOfflineTraj[DataPos*6+6*i+j]=TempDataRArm[i]/180*3.1415926535;
			gLArmOfflineTraj[DataPos*6+6*i+j]=TempDataLArm[i]/180*3.1415926535;
			}
			else if(j==3)
			{
			gRArmOfflineTraj[DataPos*6+6*i+j]=TempDataRArm2[i]/180*3.1415926535;
			gLArmOfflineTraj[DataPos*6+6*i+j]=TempDataLArm2[i]/180*3.1415926535;
			}
			else
			{
			gRArmOfflineTraj[DataPos*6+6*i+j]=gInitThetas[j+20]*-1;//gKineAll.FKRArm->theta[j+4];
			gLArmOfflineTraj[DataPos*6+6*i+j]=gInitThetas[j+14];//gKineAll.FKLArm->theta[j+4];
			}
		}
	}

//count 0-1200 4th (Left)

		DataPos=4800;
	for(int i=0;i<6;i++)
	{
		if(i==0)
		{
		gKineAll.GenSmoothZMPShift_ZeroJerk(AngleShPitch*-1,gInitThetas[i+20]*-1*180/3.1415926535,1200,TempDataRArm);
		//gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.FKLArm->theta[i+4]*180/3.1415926535,-30,1200,TempDataLArm);
		}

		if(i==3)
		{
		gKineAll.GenSmoothZMPShift_ZeroJerk(AngleElbow,gInitThetas[i+20]*-1*180/3.1415926535,1200,TempDataRArm2);
		//gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.FKLArm->theta[i+4]*180/3.1415926535,30,1200,TempDataLArm2);		}
		}
	}

	for(int i=0;i<1200;i++)
	{
		for(int j=0;j<6;j++)
		{
			if(j==0)
			{
			gRArmOfflineTraj[DataPos*6+6*i+j]=TempDataRArm[i]/180*3.1415926535;
			gLArmOfflineTraj[DataPos*6+6*i+j]=gInitThetas[j+14];//gKineAll.FKLArm->theta[j+4];
			//gLArmOfflineTraj[DataPos*6+6*i+j]=TempDataLArm[i]/180*3.1415926535;
			}
			else if(j==3)
			{
			gRArmOfflineTraj[DataPos*6+6*i+j]=TempDataRArm2[i]/180*3.1415926535;
			gLArmOfflineTraj[DataPos*6+6*i+j]=gInitThetas[j+14];//gKineAll.FKLArm->theta[j+4];
			//gLArmOfflineTraj[DataPos*6+6*i+j]=TempDataLArm2[i]/180*3.1415926535;
			}
			else
			{
			gRArmOfflineTraj[DataPos*6+6*i+j]=gInitThetas[j+20]*-1;//gKineAll.FKRArm->theta[j+4];
			gLArmOfflineTraj[DataPos*6+6*i+j]=gInitThetas[j+14];//gKineAll.FKLArm->theta[j+4];
			}
		}
	}

	DataPos=6000;
	for(int i=0;i<2400;i++)
	{
		for(int j=0;j<6;j++)
		{

			gRArmOfflineTraj[DataPos*6+6*i+j]=gInitThetas[j+20]*-1;//gKineAll.FKRArm->theta[j+4];
			gLArmOfflineTraj[DataPos*6+6*i+j]=gInitThetas[j+14];//gKineAll.FKLArm->theta[j+4];
			
		}
	}
}





void CRobotAllDlg::OnBnClickedCheck4() // Checkbox --> Read Encoder
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 設定機器人要不要讀取encoder

	******************************************************************/
	if (gFlagReadEncoder == 0)
	{
		//gFlagReadEncoder = 1;
		CheckEncoder.SetCheck(0);
	}
	else if (gFlagReadEncoder == 1)
	{
		//gFlagReadEncoder = 0;
		CheckEncoder.SetCheck(1);
	}
}


void CRobotAllDlg::OnBnClickedCheck5() // Checkbox --> Read force sensor
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 設定機器人要不要讀取六軸力規

	******************************************************************/
	if (gFlagReadForceSensor == 0)
	{
		gFlagReadForceSensor = 1;
		CheckForceSensor.SetCheck(1);
	}
	else if (gFlagReadForceSensor == 1)
	{
		gFlagReadForceSensor = 0;
		CheckForceSensor.SetCheck(0);
	}
}


void Trans2Matlab(void)
{
	double CheckNum;
	fstream trans;
	for(int i = 0;i<6;i++)
		Trans2MatlabData[i] = gKineAll.FKLLeg->theta[i+1];
	for(int i = 0;i<6;i++)
		Trans2MatlabData[i+6] = gKineAll.FKRLeg->theta[i+1];
	for(int i = 0;i<6;i++)
		Trans2MatlabData[i+12] = gKineAll.FKLArm->theta[i+4];
	for(int i = 0;i<6;i++)
		Trans2MatlabData[i+18] = gKineAll.FKRArm->theta[i+4];
	for(int i = 0;i<2;i++)//在adams手臂是13~24軸腰是25,26軸
		Trans2MatlabData[i+24] = gKineAll.FKRArm->theta[i+1];
	
	for(int i = 0;i<26;i++)
		Trans2MatlabData[i] -= Trans2MatlabLastData[i];


	trans.open("R:/0228/data.txt",ios::out);
	for(int i=0;i<26;i++)
	{
		trans<<Trans2MatlabData[i];
		trans<<endl;
	}
	trans.close();

	trans.open("R:/0228/check.txt",ios::out);
	trans<<"1";
	trans.close();

	while(1)
	{
		trans.open("R:/0228/check.txt",ios::in);
		trans>>CheckNum;
		trans.close();

		if (CheckNum == 2)
		{
			for(int i = 0;i<26;i++)
				Trans2MatlabLastData[i] += Trans2MatlabData[i];
			break;
		}

		Sleep(1);
	}
}


void CRobotAllDlg::OnBnClickedCheck6()
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 設定GUI要不要繪製表示ZMP位置的小紅球

	******************************************************************/
	if (CheckZMPPlot.GetCheck())
		gFlagZMPPlot = 1;
	else
		gFlagZMPPlot = 0;
	
}


void CRobotAllDlg::OnBnClickedCheck7()
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 設定要不要讀取皮膚模組
	// 20140221 改成紅外線模組
	******************************************************************/
	//if (CheckSkinModule.GetCheck())
	//	gFlagSkinModule = 1;
	//else
	//	gFlagSkinModule = 0;

	if (CheckInfrared.GetCheck())
	{	
		gFlagInfrared = 1;
		CheckInfrared.SetCheck(1);
	}
	else
	{
		gFlagInfrared = 0;
		CheckInfrared.SetCheck(0);
	}
}

void CRobotAllDlg::OnBnClickedCheck8()
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 設定要不要開啟語音模組

	******************************************************************/
	if (CheckSpeech.GetCheck())
		gFlagSpeechModule = 1;
	else
		gFlagSpeechModule = 0;
}

//void gReadSkinSensor(void)
//{
//	/******************************************************************
//	input: void
//	output: void
//
//	Note:
//	// 設定要不要開啟皮膚模組
//
//	******************************************************************/
//
//	gpPortRSkin->_write(gCmdReadSkin,1); // 發送讀取命令
//	Sleep(2);
//	gpPortRSkin->read(gRArmSkin, 129,1,NULL); // 讀取129筆資料, 存到 buffer之中
//
//	for (int i = 0 ; i < 64 ; i++)
//	{
//		gRArmSkin[i] = gRArmSkin[i*2+1]*256+gRArmSkin[i*2+2];
//	}
//	for (int i = 0 ; i < 64 ; i++)
//	{
//		printf("%d ",gRSkinArray[i]);
//		if ((i+1)%8==0)
//			printf("\n");
//	}
//
//}

//void gReadLegEncoder(void)	//  改用 TCAT->EtherCATReadEncoder
//{
//	/******************************************************************
//	input: void
//	output: void
//
//	Note:
//	// 假若旗標 gFlagReadEncoder 為真 則會進來讀取腳的 encoder
//	//  改用 TCAT->EtherCATReadEncoder
//	******************************************************************/
//
//	//int index_read = 0;
//	int index_buf = 0;
//	const char reason = 0;
//
//	//// 紀錄encoder data 
//	//gpPortLL->read(gBufIn,96,1000,&reason);
//
//	for (int j = 0 ; j < 4 ; j++)
//	{
//		for (int i=0 ; i < 6;i++)
//		{
//			if (gBufIn[index_buf] > 128)
//			{	
//				gReadEnc[i][gEncoderReadIndex] = -(gBufIn[index_buf+1]*65536+gBufIn[index_buf+2]*256+gBufIn[index_buf+3]);
//			}
//			else
//			{
//				gReadEnc[i][gEncoderReadIndex] = gBufIn[index_buf+1]*65536+gBufIn[index_buf+2]*256+gBufIn[index_buf+3];
//			}
//			index_buf += 4;
//		}
//		gEncoderReadIndex += 1;
//	}
//
//	gEncoderReadIndex-= 4;
//	index_buf = 0;
//
//	//gpPortRL->read(gBufIn,96,1000,&reason);
//
//	for (int j = 0 ; j < 4 ; j++)
//	{
//		for (int i=6 ; i < 12;i++)
//		{
//			if (gBufIn[index_buf] > 128)
//			{
//				gReadEnc[i][gEncoderReadIndex] = -(gBufIn[index_buf+1]*65536+gBufIn[index_buf+2]*256+gBufIn[index_buf+3]);
//			}
//			else
//			{
//				gReadEnc[i][gEncoderReadIndex] = gBufIn[index_buf+1]*65536+gBufIn[index_buf+2]*256+gBufIn[index_buf+3];
//			}
//			index_buf += 4;
//		}
//		gEncoderReadIndex += 1;
//	}
//	index_buf = 0;
//}

void CRobotAllDlg::OnBnClickedCheck9()
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 切換是否進入boost 模式 此模式只有在C++模擬模式可以進行 快速檢查程式效果使用
	******************************************************************/

	if (CheckBoost.GetCheck())
		gFlagBoostSimu = true;
	else
		gFlagBoostSimu = false;
}


void gCalculateZMP(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 根據設定好的場景規劃ZMP軌跡
	******************************************************************/

	// 計算總軌跡點數
	gTrajLength	= (T_P/dt*gNumOfStep);
	gContTrajLen = gInterNum + gNumOfStep*StepSam;

	if (gKineAll.FlagStayMode == 1 || gKineAll.FlagStaySquatMode == 1) // 機器人是否進入停止不動模式
		gStopTrajLen = int(gNoMotionTime/dt);
	else
		gStopTrajLen = gContTrajLen - int(3.82*StepSam); // 少幾步 讓LQ多算一些

	gLQs.LQDataLen = gNumOfStep*StepSam; // 總長度 = 總步數*每步格數

	/////////////ZMP Traj Planning///////////////////////

	/////// Local Variables for ZMP generation /////////
	double DirFB_Now[3]; // 現在這步前後單位向量
	double DirLR_Now[3]; // 現在這步左右單位向量
	double LenFront=0; // ZMP在足底前進的量
	double LenBack=0; // ZMP在足底從後面多少開始的量
	double Front2Step[3] = {0,0,0}; // 記住下一步與這一步的位置差
	double Back2Step[3] = {0,0,0}; // 記住這一步與前一步的位置差
	double DistFBFront2P = 0;
	double DistFBBack2P = 0;
	double DistLRFront2P = 0;
	double VectorDSP[3] = {0,0,0}; // vector between the front point and the last point
	double LastVectorDSP[3] = {0,0,0}; // Last vector between the front point and the last point
	double DsitanceDSP = 0; // norm  of VectorDSP
	double LastDsitanceDSP = 0; // norm  of LastVectorDSP

	double VecFB_Last = 0; // 上一個在腳底往前後方向的ZMP移動長度
	double VecLR_Last = 0; // 上一個在腳底往左右方向的ZMP移動長度 
	double VecFB_Now = 0; // 在腳底往前後方向的ZMP移動長度
	double VecLR_Now = 0; // 在腳底往左右方向的ZMP移動長度 
	////////////////////////////////////////////////////

	/******************************************************************
	Flag
		gUpStair 爬樓梯使用 將ZMP在後腳離地前盡量往前腳腳踝投影靠近
	******************************************************************/		


	/******************************************************************
	ZMP 軌跡參數說明

	Note:
		以往舊版本的軌跡在Single support phase時ZMP的位置均在腳踝投影點,只有在Double support phase時移動
		但事實上人類行走的ZMP在SSP時是會移動的,而且適當的移動會有助於步伐的擴展
	
	Par:
		gLateralZMPShift  此參數為ZMP於SSP之起點終點之側向距離 (描述的昰離腳踝投影點的偏移量mm,方向朝內側為正)
		gSagittalZMPFront 此參數為ZMP於SSP之終點正向移動比例 (描述的昰離腳踝投影點的偏移程度 腳踝投影點到腳尖距離為1)			 
		gSagittalZMPBack  此參數為ZMP於SSP之起點正向移動比例 (描述的昰離腳踝投影點的偏移程度 腳踝投影點到腳跟距離為1)
		gDivideVel        此參數描述的昰SSP內軌跡銜接指定起始或終點速度的參數(請參照尋找所有參考) 參數設不好可能會造成overshoot的問題需要多加注意
	******************************************************************/	

	if (gKineAll.FlagSumoMode == 0)
	{
		#if ConstantCOGMode
			gLateralZMPShift = 6;//3;//6 // 側向ZMP移動量 for 自然ZMP
		#else
			gLateralZMPShift = 8; // 側向ZMP移動量 for 自然ZMP
		#endif

		if (gUpStair)
		{
			gLateralZMPShift = 3;
			gDivideVel=12;
		}

	    gSagittalZMPFront = 0.35; // 前方ZMP移動量 for 自然ZMP
	    gSagittalZMPBack = 0.2;//0.0;//0.2;  // 後方ZMP移動量 for 自然ZMP

		if (gUpStair)
		{
			gSagittalZMPBack = 0.1;
		}

		DistLRFront2P = gLateralZMPShift;
	}
	else
	{
		gLateralZMPShift = 0.0; 
	    gSagittalZMPFront = 0.0; 
	    gSagittalZMPBack = 0.0;
		DistLRFront2P = gLateralZMPShift;
	}


	// Calculate all foot and body vectors
	for (int i = 0 ; i < gNumOfStep ; i++)
	{
		gRotAngBody[i] = (gLRotAngZ[i]+gRRotAngZ[i])/2.0; // 先指定為兩腳平均角度 以後要做不一樣動作可以改
		gDirLFoot[0][i] = cos(gLRotAngZ[i]); // vector = [cos() sin() 0] 地面角度先設為0
		gDirLFoot[1][i] = sin(gLRotAngZ[i]);
		gDirLFoot[2][i] = 0;
		gDirRFoot[0][i] = cos(gRRotAngZ[i]); // vector = [cos() sin() 0] 地面角度先設為0
		gDirRFoot[1][i] = sin(gRRotAngZ[i]);
		gDirRFoot[2][i] = 0;
		gDirBody[0][i] = cos(gRotAngBody[i]); // vector = [cos() sin() 0] 地面角度先設為0
		gDirBody[1][i] = sin(gRotAngBody[i]);
		gDirBody[2][i] = 0;

		gDirLLateral[0][i] = sin(gLRotAngZ[i]); // vector = 順時鐘旋轉90度 [cos() sin() 0] 地面角度先設為0
		gDirLLateral[1][i] = -cos(gLRotAngZ[i]);
		gDirLLateral[2][i] = 0;
		gDirRLateral[0][i] = -sin(gRRotAngZ[i]); // vector = 逆時鐘旋轉90度[cos() sin() 0] 地面角度先設為0
		gDirRLateral[1][i] = cos(gRRotAngZ[i]);
		gDirRLateral[2][i] = 0;

		gPFront[0][i] = 0; // clear 
		gPFront[1][i] = 0; // clear 
		gPFront[2][i] = 0; // clear 
		gPCenter[0][i] = 0; // clear 
		gPCenter[1][i] = 0; // clear 
		gPCenter[2][i] = 0; // clear 
		gPBack[0][i] = 0; // clear 
		gPBack[1][i] = 0; // clear 
		gPBack[2][i] = 0; // clear 
	}
	
	for (int i = 0 ; i < gNumOfStep ; i++)
	{
		Back2Step[0] = Front2Step[0];
		Back2Step[1] = Front2Step[1];
		Back2Step[2] = Front2Step[2];

		Front2Step[0] = gFstpY[i+1]-gFstpY[i];
		Front2Step[1] = (gFstpX[i+1]-gFstpX[i])*ZMPRatio;
		Front2Step[2] = 0;

		gPCenter[0][i] = gFstpY[i];
		gPCenter[1][i] = gFstpX[i]*ZMPRatio;
		gPCenter[2][i] = 0;

		//用來測試是否只有跨一步////  請小叮噹整理註解
		if(checkonestep ==1)
		{
			gPCenter[1][2] = 80;
			gPCenter[0][2] = 40;
			new_gPCenter[1][3] = 0;
			gPCenter[1][3] = 0;
		}

		if(checkWalkOneStepHigh==1)
		{
			new_gPCenter[1][3] = 0;
		    gPCenter[1][3] = 0;
		}

		if(checkLeft_leg_up_and_down_OneStep)
		{
			gPCenter[1][2] = 72;
			gPCenter[0][2] = 217;
			gPCenter[1][3] = -80;
			new_gPCenter[1][3] = -80;
			gPCenter[1][4] = 0;
			new_gPCenter[1][4] = 0;
		}

//		if(checkLeft_leg_up_and_down_TwoStep)
//{
//	gPCenter[1][2] = 72;
//	gPCenter[0][2] = 212;
//	gPCenter[1][3] = 80;
//	new_gPCenter[1][3] = 80;
//	gPCenter[0][3] = 230;
//	gPCenter[0][4] = 230;
//	new_gPCenter[0][4] = 230;
//	gPCenter[1][5] = 0;
//	new_gPCenter[1][5] = 0;
//}

		if (gKineAll.selSupport[i] == RightSupport) // right support
		{
			LenFront = Front2Step[0]*gDirBody[0][i]+Front2Step[1]*gDirBody[1][i]+Front2Step[2]*gDirBody[2][i]; // dot product
            LenBack = Back2Step[0]*gDirBody[0][i]+Back2Step[1]*gDirBody[1][i]+Back2Step[2]*gDirBody[2][i]; // dot product

			if (gKineAll.selSupport[i-1] == DoubleSupport) // 上一步是DSP
			{
				gPFront[0][i] = gPCenter[0][i]+gDirRLateral[0][i]*gLateralZMPShift+gSagittalZMPFront*LenFront*gDirRFoot[0][i];
				gPFront[1][i] = gPCenter[1][i]+gDirRLateral[1][i]*gLateralZMPShift+gSagittalZMPFront*LenFront*gDirRFoot[1][i];
				gPFront[2][i] = gPCenter[2][i]+gDirRLateral[2][i]*gLateralZMPShift+gSagittalZMPFront*LenFront*gDirRFoot[2][i];
				gPBack[0][i] = gPCenter[0][i]+gDirRLateral[0][i]*gLateralZMPShift;
				gPBack[1][i] = gPCenter[1][i]+gDirRLateral[1][i]*gLateralZMPShift;
				gPBack[2][i] = gPCenter[2][i]+gDirRLateral[2][i]*gLateralZMPShift;
			}
			else if(gKineAll.selSupport[i+1] == DoubleSupport) // 下一步是DSP
			{
				// 這個還有下面兩個情況目前相同 都寫是為了以後或許可以增加功能讓這兩個不同
				gPFront[0][i] = gPCenter[0][i]+gDirRLateral[0][i]*gLateralZMPShift+gSagittalZMPFront*LenFront*gDirRFoot[0][i];
				gPFront[1][i] = gPCenter[1][i]+gDirRLateral[1][i]*gLateralZMPShift+gSagittalZMPFront*LenFront*gDirRFoot[1][i];
				gPFront[2][i] = gPCenter[2][i]+gDirRLateral[2][i]*gLateralZMPShift+gSagittalZMPFront*LenFront*gDirRFoot[2][i];
				gPBack[0][i] = gPCenter[0][i]+gDirRLateral[0][i]*gLateralZMPShift-gSagittalZMPBack*LenBack*gDirRFoot[0][i];
				gPBack[1][i] = gPCenter[1][i]+gDirRLateral[1][i]*gLateralZMPShift-gSagittalZMPBack*LenBack*gDirRFoot[1][i];
				gPBack[2][i] = gPCenter[2][i]+gDirRLateral[2][i]*gLateralZMPShift-gSagittalZMPBack*LenBack*gDirRFoot[2][i];
			}
			else // 普通行走中
			{
				gPFront[0][i] = gPCenter[0][i]+gDirRLateral[0][i]*gLateralZMPShift+gSagittalZMPFront*LenFront*gDirRFoot[0][i];
				gPFront[1][i] = gPCenter[1][i]+gDirRLateral[1][i]*gLateralZMPShift+gSagittalZMPFront*LenFront*gDirRFoot[1][i];
				gPFront[2][i] = gPCenter[2][i]+gDirRLateral[2][i]*gLateralZMPShift+gSagittalZMPFront*LenFront*gDirRFoot[2][i];
				gPBack[0][i] = gPCenter[0][i]+gDirRLateral[0][i]*gLateralZMPShift-gSagittalZMPBack*LenBack*gDirRFoot[0][i];
				gPBack[1][i] = gPCenter[1][i]+gDirRLateral[1][i]*gLateralZMPShift-gSagittalZMPBack*LenBack*gDirRFoot[1][i];
				gPBack[2][i] = gPCenter[2][i]+gDirRLateral[2][i]*gLateralZMPShift-gSagittalZMPBack*LenBack*gDirRFoot[2][i];				
			}

		}
		else if (gKineAll.selSupport[i] == LeftSupport) // left support
		{
			LenFront = Front2Step[0]*gDirBody[0][i]+Front2Step[1]*gDirBody[1][i]+Front2Step[2]*gDirBody[2][i]; // dot product
            LenBack = Back2Step[0]*gDirBody[0][i]+Back2Step[1]*gDirBody[1][i]+Back2Step[2]*gDirBody[2][i]; // dot product

			if (gKineAll.selSupport[i-1] == DoubleSupport) // 上一步是DSP
			{
				gPFront[0][i] = gPCenter[0][i]+gDirLLateral[0][i]*gLateralZMPShift+gSagittalZMPFront*LenFront*gDirLFoot[0][i];
				gPFront[1][i] = gPCenter[1][i]+gDirLLateral[1][i]*gLateralZMPShift+gSagittalZMPFront*LenFront*gDirLFoot[1][i];
				gPFront[2][i] = gPCenter[2][i]+gDirLLateral[2][i]*gLateralZMPShift+gSagittalZMPFront*LenFront*gDirLFoot[2][i];
				gPBack[0][i] = gPCenter[0][i]+gDirLLateral[0][i]*gLateralZMPShift;
				gPBack[1][i] = gPCenter[1][i]+gDirLLateral[1][i]*gLateralZMPShift;
				gPBack[2][i] = gPCenter[2][i]+gDirLLateral[2][i]*gLateralZMPShift;
			}
			else if(gKineAll.selSupport[i+1] == DoubleSupport) // 下一步是DSP
			{
				// 這個還有下面兩個情況目前相同 都寫是為了以後或許可以增加功能讓這兩個不同
				gPFront[0][i] = gPCenter[0][i]+gDirLLateral[0][i]*gLateralZMPShift+gSagittalZMPFront*LenFront*gDirLFoot[0][i];
				gPFront[1][i] = gPCenter[1][i]+gDirLLateral[1][i]*gLateralZMPShift+gSagittalZMPFront*LenFront*gDirLFoot[1][i];
				gPFront[2][i] = gPCenter[2][i]+gDirLLateral[2][i]*gLateralZMPShift+gSagittalZMPFront*LenFront*gDirLFoot[2][i];
				gPBack[0][i] = gPCenter[0][i]+gDirLLateral[0][i]*gLateralZMPShift-gSagittalZMPBack*LenBack*gDirLFoot[0][i];
				gPBack[1][i] = gPCenter[1][i]+gDirLLateral[1][i]*gLateralZMPShift-gSagittalZMPBack*LenBack*gDirLFoot[1][i];
				gPBack[2][i] = gPCenter[2][i]+gDirLLateral[2][i]*gLateralZMPShift-gSagittalZMPBack*LenBack*gDirLFoot[2][i];
			}
			else // 普通行走中
			{
				gPFront[0][i] = gPCenter[0][i]+gDirLLateral[0][i]*gLateralZMPShift+gSagittalZMPFront*LenFront*gDirLFoot[0][i];
				gPFront[1][i] = gPCenter[1][i]+gDirLLateral[1][i]*gLateralZMPShift+gSagittalZMPFront*LenFront*gDirLFoot[1][i];
				gPFront[2][i] = gPCenter[2][i]+gDirLLateral[2][i]*gLateralZMPShift+gSagittalZMPFront*LenFront*gDirLFoot[2][i];
				gPBack[0][i] = gPCenter[0][i]+gDirLLateral[0][i]*gLateralZMPShift-gSagittalZMPBack*LenBack*gDirLFoot[0][i];
				gPBack[1][i] = gPCenter[1][i]+gDirLLateral[1][i]*gLateralZMPShift-gSagittalZMPBack*LenBack*gDirLFoot[1][i];
				gPBack[2][i] = gPCenter[2][i]+gDirLLateral[2][i]*gLateralZMPShift-gSagittalZMPBack*LenBack*gDirLFoot[2][i];				
			}
		}
		else if (gKineAll.selSupport[i] == DoubleSupport)
		{
			LenFront = 0; // zero, remain the same position
            LenBack = 0; // zero, remain the same position
			gPFront[0][i] = gPCenter[0][i];
			gPFront[1][i] = gPCenter[1][i];
			gPFront[2][i] = gPCenter[2][i];
			gPBack[0][i] = gPCenter[0][i];
			gPBack[1][i] = gPCenter[1][i];
			gPBack[2][i] = gPCenter[2][i];				
		}
	}

	//用來測試是否只有跨一步//
		if(checkonestep ==1)
		{
		new_gPCenter[1][3] = 0;
		gPCenter[1][3] = 0;
		}

	// Copy all back center front points to a queue // 
	int index_copy = 0;
	for (int i = 0 ; i < gNumOfStep ; i++)
	{
		gPAll[0][index_copy] = gPBack[0][i];
		gPAll[1][index_copy] = gPBack[1][i];
		gPAll[2][index_copy] = gPBack[2][i];
		index_copy++;
		gPAll[0][index_copy] = gPCenter[0][i];
		gPAll[1][index_copy] = gPCenter[1][i];
		gPAll[2][index_copy] = gPCenter[2][i];
		index_copy++;
		gPAll[0][index_copy] = gPFront[0][i];
		gPAll[1][index_copy] = gPFront[1][i];
		gPAll[2][index_copy] = gPFront[2][i];
		index_copy++;
	}
	///////////////////////////////////////////////////

	////////////////// 算出ZMP連續軌跡 /////////////////

	// 開始都是零 0~gNza確定是零
	for (int j = 0 ; j < gNza ; j++)
	{
		gInpZMPy[j] = 0;
		gInpZMPx[j] = 0;
	}

	for (int i = 0 ; i < gNumOfStep ; i++)
	{
		index_copy = (i+1)*3;
		VectorDSP[0] = gPAll[0][index_copy]-gPAll[0][index_copy-1]; // 取出在DSP中所移動的ZMP距離(前後x) 下一步的gPBack-前一步的gPFront
		VectorDSP[1] = gPAll[1][index_copy]-gPAll[1][index_copy-1]; // 取出在DSP中所移動的ZMP距離(左右y)
		VectorDSP[2] = gPAll[2][index_copy]-gPAll[2][index_copy-1]; // 取出在DSP中所移動的ZMP距離(上下z)
		//DSPRatio[i] = VectorDSP[1];	// WZ Distributor 1015
		DsitanceDSP = gNorm1Pd(VectorDSP);
		if (DsitanceDSP == 0) // zero vector
		{
			VectorDSP[0] = 0; VectorDSP[1] = 0; VectorDSP[2] = 0;
		}
		else
		{
			VectorDSP[0] = VectorDSP[0]/DsitanceDSP;
			VectorDSP[1] = VectorDSP[1]/DsitanceDSP;
			VectorDSP[2] = VectorDSP[2]/DsitanceDSP;
		}

		if (gKineAll.selSupport[i] == RightSupport) // right support
		{
			DirFB_Now[0] = gDirRFoot[0][i];
			DirFB_Now[1] = gDirRFoot[1][i];
			DirFB_Now[2] = gDirRFoot[2][i];
			DirLR_Now[0] = -gDirRLateral[0][i];
			DirLR_Now[1] = -gDirRLateral[1][i];
			DirLR_Now[2] = -gDirRLateral[2][i];
		}
		else if (gKineAll.selSupport[i] == LeftSupport) // left support  
		{
			DirFB_Now[0] = gDirLFoot[0][i];
			DirFB_Now[1] = gDirLFoot[1][i];
			DirFB_Now[2] = gDirLFoot[2][i];
			DirLR_Now[0] = -gDirLLateral[0][i];
			DirLR_Now[1] = -gDirLLateral[1][i];
			DirLR_Now[2] = -gDirLLateral[2][i];
		}      
		else // double support
		{
			DirFB_Now[0] = gDirRFoot[0][i];
			DirFB_Now[1] = gDirRFoot[1][i];
			DirFB_Now[2] = gDirRFoot[2][i];
			DirLR_Now[0] = -gDirRLateral[0][i];
			DirLR_Now[1] = -gDirRLateral[1][i];
			DirLR_Now[2] = -gDirRLateral[2][i];
		}

		// 求出前中與中後點 之間的距離(投影在腳的方向上的距離)
		DistFBFront2P = (gPAll[0][index_copy-1]-gPAll[0][index_copy-2])*DirFB_Now[0]+(gPAll[1][index_copy-1]-gPAll[1][index_copy-2])*DirFB_Now[1]+(gPAll[2][index_copy-1]-gPAll[2][index_copy-2])*DirFB_Now[2];
		DistFBBack2P = (gPAll[0][index_copy-2]-gPAll[0][index_copy-3])*DirFB_Now[0]+(gPAll[1][index_copy-2]-gPAll[1][index_copy-3])*DirFB_Now[1]+(gPAll[2][index_copy-2]-gPAll[2][index_copy-3])*DirFB_Now[2];

		//DistLRFront2P = 20;

		VecFB_Now = DsitanceDSP*(DirFB_Now[0]*VectorDSP[0]+DirFB_Now[1]*VectorDSP[1]+DirFB_Now[2]*VectorDSP[2]);
		VecLR_Now = DsitanceDSP*(DirLR_Now[0]*VectorDSP[0]+DirLR_Now[1]*VectorDSP[1]+DirLR_Now[2]*VectorDSP[2]);    
  
		if (i == 0) // first step
		{
			VecFB_Last = LastDsitanceDSP*(DirFB_Now[0]*LastVectorDSP[0]+DirFB_Now[1]*LastVectorDSP[1]+DirFB_Now[2]*LastVectorDSP[2]);
			VecLR_Last = LastDsitanceDSP*(DirLR_Now[0]*LastVectorDSP[0]+DirLR_Now[1]*LastVectorDSP[1]+DirLR_Now[2]*LastVectorDSP[2]);    
        
			gKineAll.GenZMPFreeAssign(T_P*SSP/2.0,0,0,0,0,0,0,0,0,gNab/2.0+1,varSSP1FB);
			gKineAll.GenZMPFreeAssign(T_P*SSP/2.0,0,0,0,0,0,0,0,0,gNab/2.0+1,varSSP1LR);
     
			gKineAll.GenZMPFreeAssign(T_P*SSP/2.0,0,0,0,0,0,0,0,0,gNab/2.0+1,varSSP2FB);
			gKineAll.GenZMPFreeAssign(T_P*SSP/2.0,0,0,0,0,0,0,0,0,gNab/2.0+1,varSSP2LR);

            gKineAll.GenZMPFreeAssign(T_P*DSP,0,0,0,0,DsitanceDSP,DsitanceDSP/gDivideVel,0,0,gNza+gNzb+1,varDSP);
		}
		else 
		{
			if (gKineAll.selSupport[i] == DoubleSupport)
			{
				VecFB_Last = LastDsitanceDSP*(DirFB_Now[0]*LastVectorDSP[0]+DirFB_Now[1]*LastVectorDSP[1]+DirFB_Now[2]*LastVectorDSP[2]);
				VecLR_Last = LastDsitanceDSP*(DirLR_Now[0]*LastVectorDSP[0]+DirLR_Now[1]*LastVectorDSP[1]+DirLR_Now[2]*LastVectorDSP[2]);    

				gKineAll.GenZMPFreeAssign(T_P*SSP/2.0,0,0,0,0,0,0,0,0,gNab/2.0+1,varSSP1FB);
				gKineAll.GenZMPFreeAssign(T_P*SSP/2.0,0,0,0,0,0,0,0,0,gNab/2.0+1,varSSP1LR);
     
				gKineAll.GenZMPFreeAssign(T_P*SSP/2.0,0,0,0,0,0,0,0,0,gNab/2.0+1,varSSP2FB);
				gKineAll.GenZMPFreeAssign(T_P*SSP/2.0,0,0,0,0,0,0,0,0,gNab/2.0+1,varSSP2LR);

				gKineAll.GenZMPFreeAssign(T_P*DSP,0,DsitanceDSP/gDivideVel,0,0,DsitanceDSP,DsitanceDSP/gDivideVel,0,0,gNza+gNzb+1,varDSP);
			}
			else
			{
				if (gKineAll.selSupport[i-1] == DoubleSupport)
				{
					VecFB_Last = LastDsitanceDSP*(DirFB_Now[0]*LastVectorDSP[0]+DirFB_Now[1]*LastVectorDSP[1]+DirFB_Now[2]*LastVectorDSP[2]);
					VecLR_Last = LastDsitanceDSP*(DirLR_Now[0]*LastVectorDSP[0]+DirLR_Now[1]*LastVectorDSP[1]+DirLR_Now[2]*LastVectorDSP[2]);    

					gKineAll.GenZMPFreeAssign(T_P*SSP/2.0,0,VecFB_Last/gDivideVel,0,0,0,0,0,0,gNab/2.0+1,varSSP1FB);
					gKineAll.GenZMPFreeAssign(T_P*SSP/2.0,0,VecLR_Last/gDivideVel,0,0,DistLRFront2P,0,0,0,gNab/2.0+1,varSSP1LR);
     
					gKineAll.GenZMPFreeAssign(T_P*SSP/2.0,0,0,0,0,DistFBFront2P,VecFB_Now/gDivideVel,0,0,gNab/2.0+1,varSSP2FB);
					gKineAll.GenZMPFreeAssign(T_P*SSP/2.0,0,0,0,0,-DistLRFront2P,VecLR_Now/gDivideVel,0,0,gNab/2.0+1,varSSP2LR);

					gKineAll.GenZMPFreeAssign(T_P*DSP,0,DsitanceDSP/gDivideVel,0,0,DsitanceDSP,DsitanceDSP/gDivideVel,0,0,gNza+gNzb+1,varDSP);
				}
				else if (gKineAll.selSupport[i+1] == DoubleSupport)
				{
					VecFB_Last = LastDsitanceDSP*(DirFB_Now[0]*LastVectorDSP[0]+DirFB_Now[1]*LastVectorDSP[1]+DirFB_Now[2]*LastVectorDSP[2]);
					VecLR_Last = LastDsitanceDSP*(DirLR_Now[0]*LastVectorDSP[0]+DirLR_Now[1]*LastVectorDSP[1]+DirLR_Now[2]*LastVectorDSP[2]);    

					gKineAll.GenZMPFreeAssign(T_P*SSP/2.0,0,VecFB_Last/gDivideVel,0,0,DistFBBack2P,0,0,0,gNab/2.0+1,varSSP1FB);
					gKineAll.GenZMPFreeAssign(T_P*SSP/2.0,0,VecLR_Last/gDivideVel,0,0,DistLRFront2P,0,0,0,gNab/2.0+1,varSSP1LR);
						
					gKineAll.GenZMPFreeAssign(T_P*SSP/2.0,0,0,0,0,DistFBFront2P,VecFB_Now/gDivideVel,0,0,gNab/2.0+1,varSSP2FB);
					gKineAll.GenZMPFreeAssign(T_P*SSP/2.0,0,0,0,0,-DistLRFront2P,VecLR_Now/gDivideVel,0,0,gNab/2.0+1,varSSP2LR);

					gKineAll.GenZMPFreeAssign(T_P*DSP,0,DsitanceDSP/gDivideVel,0,0,DsitanceDSP,DsitanceDSP/gDivideVel,0,0,gNza+gNzb+1,varDSP);
				}
				else
				{
					VecFB_Last = LastDsitanceDSP*(DirFB_Now[0]*LastVectorDSP[0]+DirFB_Now[1]*LastVectorDSP[1]+DirFB_Now[2]*LastVectorDSP[2]);
					VecLR_Last = LastDsitanceDSP*(DirLR_Now[0]*LastVectorDSP[0]+DirLR_Now[1]*LastVectorDSP[1]+DirLR_Now[2]*LastVectorDSP[2]);    

					gKineAll.GenZMPFreeAssign(T_P*SSP/2.0,0,VecFB_Last/gDivideVel,0,0,DistFBBack2P,DistFBBack2P/(T_P*SSP/2.0),0,0,gNab/2.0+1,varSSP1FB);
					gKineAll.GenZMPFreeAssign(T_P*SSP/2.0,0,VecLR_Last/gDivideVel,0,0,DistLRFront2P,0,0,0,gNab/2.0+1,varSSP1LR);

					gKineAll.GenZMPFreeAssign(T_P*SSP/2.0,0,DistFBBack2P/(T_P*SSP/2.0),0,0,DistFBFront2P,VecFB_Now/gDivideVel,0,0,gNab/2.0+1,varSSP2FB);
					gKineAll.GenZMPFreeAssign(T_P*SSP/2.0,0,0,0,0,-DistLRFront2P,VecLR_Now/gDivideVel,0,0,gNab/2.0+1,varSSP2LR);

					gKineAll.GenZMPFreeAssign(T_P*DSP,0,DsitanceDSP/gDivideVel,0,0,DsitanceDSP,DsitanceDSP/gDivideVel,0,0,gNza+gNzb+1,varDSP);
				}
			}
		}

		index_copy = 1; // 捨棄第一筆 因為重複，又由於生的時候有多生一筆 所以有剛剛好數量的軌跡點
		for (int j = i*gStepSample+gNza ; j < i*gStepSample+gNza+gNab/2.0 ; j++)
		{
			gInpZMPy[j] = gPBack[0][i]+varSSP1FB[index_copy]*DirFB_Now[0]+varSSP1LR[index_copy]*DirLR_Now[0];
			gInpZMPx[j] = gPBack[1][i]+varSSP1FB[index_copy]*DirFB_Now[1]+varSSP1LR[index_copy]*DirLR_Now[1];
			index_copy++;
		}
		index_copy = 1; // 捨棄第一筆 因為重複，又由於生的時候有多生一筆 所以有剛剛好數量的軌跡點
		for (int j = i*gStepSample+gNza+gNab/2 ; j < i*gStepSample+gNza+gNab ; j++)
		{
			gInpZMPy[j] = gPCenter[0][i]+varSSP2FB[index_copy]*DirFB_Now[0]+varSSP2LR[index_copy]*DirLR_Now[0];
			gInpZMPx[j] = gPCenter[1][i]+varSSP2FB[index_copy]*DirFB_Now[1]+varSSP2LR[index_copy]*DirLR_Now[1];
			index_copy++;
		}
		index_copy = 1; // 捨棄第一筆 因為重複，又由於生的時候有多生一筆 所以有剛剛好數量的軌跡點
		for (int j = i*gStepSample+gNza+gNab ; j < (i+1)*gStepSample+gNza ; j++)
		{
			gInpZMPy[j] = gPFront[0][i]+varDSP[index_copy]*VectorDSP[0];
			gInpZMPx[j] = gPFront[1][i]+varDSP[index_copy]*VectorDSP[1];
			index_copy++;
		}

		LastDsitanceDSP = DsitanceDSP;
        LastVectorDSP[0] = VectorDSP[0];
        LastVectorDSP[1] = VectorDSP[1];
        LastVectorDSP[2] = VectorDSP[2];
	}
	///////////////////////////////////////////////////
}

void gCalculateCOG(double COGDown)
{
	/******************************************************************
	input: COGDown: 在這段軌跡中來機器人要蹲下的高度
	output: void

	Note:
	// 根據設定好的場景規劃COG軌跡
	******************************************************************/

		// 初始化
		for (int ii = 0 ; ii < gTrajLength+8 ; ii++)
		{
			gInpZMPHeight[ii] = 0.0;
			gInpCOG[ii] = 0.0;
		}

		// 初始COG 軌跡 平的 暫存在gInpInvPendulumHeight裡面
		gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.initCOG[2],gKineAll.initCOG[2]-COGDown/3.0,gStepSample+2,gInpInvPendulumHeight);
		gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.initCOG[2]-COGDown/3.0,gKineAll.initCOG[2]-COGDown,gStepSample+2,gInpInvPendulumHeight+gStepSample+2);

		for (int ii=0 ; ii< gTrajLength+8 ; ii++)
		{
			if (ii >= gStepSample*2+4)
			{
				//gInpCOG[ii] = gKineAll.initCOG[2]-ii*0.1; // testing cogz input
				gInpInvPendulumHeight[ii] = gKineAll.initCOG[2] - COGDown; // testing cogz input
			}
		}
		gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.initCOG[2]-COGDown,gKineAll.initCOG[2]-COGDown/3.0,gStepSample+2,gInpInvPendulumHeight+gStepSample*(gNumOfStep-5)+4);
		gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.initCOG[2]-COGDown/3.0,gKineAll.initCOG[2],gStepSample+2,gInpInvPendulumHeight+gStepSample*(gNumOfStep-4)+2);
		for (int ii=gStepSample*(gNumOfStep-3)+4 ; ii< gTrajLength+8 ; ii++)
		{
			if (ii >= gStepSample+4)
			{
				//gInpCOG[ii] = gKineAll.initCOG[2]-ii*0.1; // testing cogz input
				gInpInvPendulumHeight[ii] = gKineAll.initCOG[2]; // testing cogz input
			}
		}

		//哲軒改20101127
		
	if(checkLeft_leg_up_and_down_OneStep==0	)
	{
		// 零加上半階高度 先存在 gInpCOG 這是機器人在上樓梯時重心高度的波動
		for (int ii = 1; ii < gNumOfStep; ii++)
		{
			gKineAll.GenSmoothZMPShift_ZeroJerkshiftstair((gGroundHeight[ii-1]+gGroundHeight[ii])*0.5,(gGroundHeight[ii]+gGroundHeight[ii+1])*0.5,gStepSample,gInpCOG+4+gStepSample*ii, mode1);
			/*if(checkLeft_leg_up_and_down_OneStep ==1 && ii == 1)
			{
				gKineAll.GenSmoothZMPShift_ZeroJerkshiftstair(20,0,gStepSample,gInpCOG+4+gStepSample*ii, 0);
			}*/
		}
	}
	else //Dora單腳
	{
		// 零加上半階高度 先存在 gInpCOG 這是機器人在上樓梯時重心高度的波動
		for (int ii = 1; ii < gNumOfStep; ii++)
		{
			gKineAll.GenSmoothZMPShift_ZeroJerkshiftstair((gGroundHeight[ii-1]+gGroundHeight[ii])*0.5,(gGroundHeight[ii]+gGroundHeight[ii+1])*0.5,gStepSample,gInpCOG+4+gStepSample*ii, mode1);
			/*if(checkLeft_leg_up_and_down_OneStep ==1 && ii == 1)
			{
				gKineAll.GenSmoothZMPShift_ZeroJerkshiftstair(20,0,gStepSample,gInpCOG+4+gStepSample*ii, 0);
			}*/
		}	
	}
		for (int ii = 0 ; ii < gTrajLength+8 ; ii++)
		{
			gInpCOG[ii] = gInpCOG[ii] + gInpInvPendulumHeight[ii]; // 真正重心軌跡 現在 gInpInvPendulumHeight只是暫存 無物理意義
		}

		// 計算地面高度軌跡
		for (int ii = 1; ii < gNumOfStep; ii++)
		{
			gKineAll.GenSmoothZMPShift_ZeroJerkshiftstair(gGroundHeight[ii],gGroundHeight[ii+1],gStepSample,gInpZMPHeight+4+gStepSample*ii, mode1 );//更改部分
		}
		//哲軒改20101127

		////fstream LoadCOGz;

		////LoadCOGz.open("czTP6p0.txt",ios::in);

		////for (int i = 0 ; i < 20000 ; i++)
		////{
		////	  LoadCOGz >> gInpCOG[i+gStepSample+4];
		////	  gInpCOG[i+gStepSample+4] -= 504.56;
		////	  gInpCOG[i+gStepSample+4]*= 0.7;
		////	  gInpCOG[i+gStepSample+4] += (504.56-32.8+3.3);
		////}
		////LoadCOGz.close();

		////gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.initCOG[2],gInpCOG[gStepSample+4],gStepSample+4,gInpCOG);
		////LoadCOGz.open("resultCOG.txt",ios::out);
		////for (int i = 0 ; i < 20000+gStepSample+4 ; i++)
		////{
		////	   LoadCOGz << gInpCOG[i] << " ";
		////}
		////LoadCOGz.close();

		// 真正算出倒單擺高度 = COG真正軌跡 - 地面高度軌跡
		for (int ii = 0 ; ii < gTrajLength+8 ; ii++)
		{
			gInpInvPendulumHeight[ii] = gInpCOG[ii] - gInpZMPHeight[ii]; 
		}

		//fstream fCOGz;
		//	fCOGz.open("COGz.txt",ios::out| ios::app);
		//	//fCOGz<<123<<"\t";
		//	for(int i=0;i<gTrajLength+8;i++)
		//	{
		//		//fCOGz<<gInpInvPendulumHeight[i]<<"\t";
		//		fCOGz<<gInpCOG[i]<<"\t";

		//	}
		//	fCOGz<<"\n";
		//	fCOGz.close();

	    gLQs.Initval(gInpInvPendulumHeight);
		
		////DORADATA//
		//int size_all = 13200;

		//double **all;
		//all = new double *[size_all];
	 //   for(int i =0;i<size_all;i++)
		//all[i] = new double[3];

		//fstream ALL; 
		//ALL.open("all.txt",ios::in);
		//
		//for (int i=0 ; i< size_all ; i++)
		//{
		//	for(int j = 0;j<3;j++)
		//	{
		//		ALL >> all[i][j];
		//	}
		//	
		//}

		//ALL.close();
		//
		//for(int i = 0;i<size_all;i++)
		//{
		//	gLQs.YState[i].data[0] = all[i][0];
		//	gLQs.XState[i].data[0] = all[i][1];
		//}

		////DORADATA//

		gLQs.BackRiccati(gInpZMPx,gInpZMPy);
        gLQs.DummyControl();
}

void gCalculateFlatuatedCOG(double COGDown, double COGUp)
{
	/******************************************************************
	input: COGDown: 在這段軌跡中來機器人要蹲下的高度
	output: void

	Note:
	// 根據設定好的場景規劃COG軌跡
	******************************************************************/

		// 初始化
		for (int ii = 0 ; ii < gTrajLength+8 ; ii++)
		{
			gInpZMPHeight[ii] = 0.0;
			gInpCOG[ii] = 0.0;
		}

		// 初始COG 軌跡 平的 暫存在gInpInvPendulumHeight裡面
		gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.initCOG[2],gKineAll.initCOG[2]-COGDown,gStepSample+4,gInpInvPendulumHeight);
		//gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.initCOG[2]-COGDown/3,gKineAll.initCOG[2]-COGDown,gStepSample+2,gInpInvPendulumHeight+gStepSample+2);

	//int gNza; // 將一步切成三段 第一段格點數  gNza = gStepSample * DSP /2.0
	//int gNab; // 將一步切成三段 第二段格點數 gNab = gStepSample * SSP
	//int gNzb; // 將一步切成三段 第三段格點數  gNb = gStepSample - gNza - gNab

		for(int j=0;j<gStepSample;j++)
		gInpInvPendulumHeight[j+gStepSample+4]=gKineAll.initCOG[2]-COGDown;
	//gKineAll.GenSwingTrajMod(0,0,0.5,gKineAll.StepHeight[gKineAll.stepIndex]+gStrideZ,0,gStrideX,gStrideZ,0,0,gKineAll.Nab,0, gKineAll.SwingBufferx, gKineAll.SwingBufferz);
	//gKineAll.GenSwingTrajMod(0,0,0.5,gKineAll.StepHeight[gKineAll.stepIndex]+gStrideZ,0,gStrideY,gStrideZ,0,0,gKineAll.Nab,0, gKineAll.SwingBuffery, gKineAll.SwingBufferz);


		for (int ii=2 ; ii< (gNumOfStep-5) ; ii++)
		{

				double* tempCOGZ;
				double ccccc=2000;
				tempCOGZ=&ccccc;
				//gInpCOG[ii] = gKineAll.initCOG[2]-ii*0.1; // testing cogz input
				for(int j=0;j<gNza+0.1*gNab-0.25*gNza;j++)
					gInpInvPendulumHeight[j+gStepSample*ii+4]=gKineAll.initCOG[2]-COGDown;
				//gKineAll.GenCOGZTrajMod(0,0,gKineAll.initCOG[2]-COGDown,0.5,COGUp-COGDown,0,gStrideX,gStrideZ,0,0,gNab,0,tempCOGZ,gInpInvPendulumHeight+gStepSample*ii+gNza+4);
				gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.initCOG[2]-COGDown,gKineAll.initCOG[2]-COGUp,gStepSample*SSP/5*2,gInpInvPendulumHeight+gStepSample*ii+gNza+gNab/10+4-gNza/4);
				gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.initCOG[2]-COGUp,gKineAll.initCOG[2]-COGDown,gStepSample*SSP/5*2,gInpInvPendulumHeight+gStepSample*ii+gNza+gNab/2+4-gNza/4);
				//for(int j=0;j<gNab;j++)
				//	gInpInvPendulumHeight[j+gNza+gStepSample*ii+4]=gKineAll.initCOG[2]-COGDown;
				for(int j=0;j<gNzb+0.1*gNab+0.25*gNza;j++)
					gInpInvPendulumHeight[j+gStepSample*ii+4+gNza+gNab/10*9-gNza/4]=gKineAll.initCOG[2]-COGDown;
				//gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.initCOG[2]-COGDown,gKineAll.initCOG[2]-COGUp,gStepSample*DSP/2,gInpInvPendulumHeight+gStepSample*ii+gNza+gNab+4);
				//gInpInvPendulumHeight[ii] = gKineAll.initCOG[2] - COGUp; // testing cogz input

		}
		for(int j=0;j<gStepSample;j++)
		gInpInvPendulumHeight[j+gStepSample*(gNumOfStep-5)+4]=gKineAll.initCOG[2]-COGDown;
		gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.initCOG[2]-COGDown,gKineAll.initCOG[2],gStepSample+4,gInpInvPendulumHeight+gStepSample*(gNumOfStep-4)+4);

		//gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.initCOG[2]-COGDown/3,gKineAll.initCOG[2],gStepSample+2,gInpInvPendulumHeight+gStepSample*(gNumOfStep-4)+2);
		for (int ii=gStepSample*(gNumOfStep-3)+4 ; ii< gTrajLength+8 ; ii++)
		{
			if (ii >= gStepSample+4)
			{
				//gInpCOG[ii] = gKineAll.initCOG[2]-ii*0.1; // testing cogz input
				gInpInvPendulumHeight[ii] = gKineAll.initCOG[2]; // testing cogz input
			}
		}

		// 零加上半階高度 先存在 gInpCOG 這是機器人在上樓梯時重心高度的波動
		for (int ii = 1; ii < gNumOfStep; ii++)
		{
			gKineAll.GenSmoothZMPShift_ZeroJerk((gGroundHeight[ii-1]+gGroundHeight[ii])*0.5,(gGroundHeight[ii]+gGroundHeight[ii+1])*0.5,gStepSample,gInpCOG+4+gStepSample*ii);
		}

		for (int ii = 0 ; ii < gTrajLength+8 ; ii++)
		{
			gInpCOG[ii] = gInpCOG[ii] + gInpInvPendulumHeight[ii]; // 真正重心軌跡 現在 gInpInvPendulumHeight只是暫存 無物理意義
		}

		// 計算地面高度軌跡
		for (int ii = 1; ii < gNumOfStep; ii++)
		{
			gKineAll.GenSmoothZMPShift_ZeroJerk(gGroundHeight[ii],gGroundHeight[ii+1],gStepSample,gInpZMPHeight+4+gStepSample*ii);
		}

		////fstream LoadCOGz;

		////LoadCOGz.open("czTP6p0.txt",ios::in);

		////for (int i = 0 ; i < 20000 ; i++)
		////{
		////	  LoadCOGz >> gInpCOG[i+gStepSample+4];
		////	  gInpCOG[i+gStepSample+4] -= 504.56;
		////	  gInpCOG[i+gStepSample+4]*= 0.7;
		////	  gInpCOG[i+gStepSample+4] += (504.56-32.8+3.3);
		////}
		////LoadCOGz.close();

		////gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.initCOG[2],gInpCOG[gStepSample+4],gStepSample+4,gInpCOG);
		////LoadCOGz.open("resultCOG.txt",ios::out);
		////for (int i = 0 ; i < 20000+gStepSample+4 ; i++)
		////{
		////	   LoadCOGz << gInpCOG[i] << " ";
		////}
		////LoadCOGz.close();

		//// 真正算出倒單擺高度 = COG真正軌跡 - 地面高度軌跡
		//for (int ii = 0 ; ii < gTrajLength+8 ; ii++)
		//{
		//	gInpInvPendulumHeight[ii] = gInpCOG[ii] - gInpZMPHeight[ii]; 
		//}
		//fstream fCOGz;
		//	fCOGz.open("COGz.txt",ios::out| ios::app);
		//	//fCOGz<<123<<"\t";
		//	for(int i=0;i<gTrajLength+8;i++)
		//	{
		//		fCOGz<<gInpInvPendulumHeight[i]<<"\t";
		//	}
		//	fCOGz<<"\n";
		//	fCOGz.close();


	    gLQs.Initval(gInpInvPendulumHeight);
		gLQs.BackRiccati(gInpZMPx,gInpZMPy);
        gLQs.DummyControl();
}

void gCalculateSinCOG(double COGDown, double COGUp)
{
	/******************************************************************
	input: COGDown: 在這段軌跡中來機器人要蹲下的高度
	output: void

	Note:
	// 根據設定好的場景規劃COG軌跡
	******************************************************************/

		// 初始化
		for (int ii = 0 ; ii < gTrajLength+8 ; ii++)
		{
			gInpZMPHeight[ii] = 0.0;
			gInpCOG[ii] = 0.0;
		}

		// 初始COG 軌跡 平的 暫存在gInpInvPendulumHeight裡面
		gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.initCOG[2],gKineAll.initCOG[2]-COGDown,gStepSample+4,gInpInvPendulumHeight);
		//gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.initCOG[2]-COGDown/3,gKineAll.initCOG[2]-COGDown,gStepSample+2,gInpInvPendulumHeight+gStepSample+2);

	//int gNza; // 將一步切成三段 第一段格點數  gNza = gStepSample * DSP /2.0
	//int gNab; // 將一步切成三段 第二段格點數 gNab = gStepSample * SSP
	//int gNzb; // 將一步切成三段 第三段格點數  gNb = gStepSample - gNza - gNab

		for(int j=0;j<gStepSample;j++)
		gInpInvPendulumHeight[j+gStepSample+4]=gKineAll.initCOG[2]-COGDown;
	//gKineAll.GenSwingTrajMod(0,0,0.5,gKineAll.StepHeight[gKineAll.stepIndex]+gStrideZ,0,gStrideX,gStrideZ,0,0,gKineAll.Nab,0, gKineAll.SwingBufferx, gKineAll.SwingBufferz);
	//gKineAll.GenSwingTrajMod(0,0,0.5,gKineAll.StepHeight[gKineAll.stepIndex]+gStrideZ,0,gStrideY,gStrideZ,0,0,gKineAll.Nab,0, gKineAll.SwingBuffery, gKineAll.SwingBufferz);


		for (int ii=2 ; ii< (gNumOfStep-5) ; ii++)
		{
			for(int j=0;j<gStepSample;j++)
			//gInpInvPendulumHeight[ii*gStepSample+j]=gKineAll.initCOG[2]-COGDown;
			gInpInvPendulumHeight[j+ii*gStepSample]=(COGDown-COGUp)/2*sin(2*PI/(gStepSample)*j -0.5*PI) - (COGUp+COGDown)/2+gKineAll.initCOG[2];
				//double* tempCOGZ;
				//double ccccc=2000;
				//tempCOGZ=&ccccc;
				////gInpCOG[ii] = gKineAll.initCOG[2]-ii*0.1; // testing cogz input
				//for(int j=0;j<gNza+0.1*gNab-0.25*gNza;j++)
				//	gInpInvPendulumHeight[j+gStepSample*ii+4]=gKineAll.initCOG[2]-COGDown;
				////gKineAll.GenCOGZTrajMod(0,0,gKineAll.initCOG[2]-COGDown,0.5,COGUp-COGDown,0,gStrideX,gStrideZ,0,0,gNab,0,tempCOGZ,gInpInvPendulumHeight+gStepSample*ii+gNza+4);
				//gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.initCOG[2]-COGDown,gKineAll.initCOG[2]-COGUp,gStepSample*SSP/5*2,gInpInvPendulumHeight+gStepSample*ii+gNza+gNab/10+4-gNza/4);
				//gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.initCOG[2]-COGUp,gKineAll.initCOG[2]-COGDown,gStepSample*SSP/5*2,gInpInvPendulumHeight+gStepSample*ii+gNza+gNab/2+4-gNza/4);
				////for(int j=0;j<gNab;j++)
				////	gInpInvPendulumHeight[j+gNza+gStepSample*ii+4]=gKineAll.initCOG[2]-COGDown;
				//for(int j=0;j<gNzb+0.1*gNab+0.25*gNza;j++)
				//	gInpInvPendulumHeight[j+gStepSample*ii+4+gNza+gNab/10*9-gNza/4]=gKineAll.initCOG[2]-COGDown;
				////gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.initCOG[2]-COGDown,gKineAll.initCOG[2]-COGUp,gStepSample*DSP/2,gInpInvPendulumHeight+gStepSample*ii+gNza+gNab+4);
				////gInpInvPendulumHeight[ii] = gKineAll.initCOG[2] - COGUp; // testing cogz input

		}
		for(int j=0;j<gStepSample;j++)
		gInpInvPendulumHeight[j+gStepSample*(gNumOfStep-5)+4]=gKineAll.initCOG[2]-COGDown;
		gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.initCOG[2]-COGDown,gKineAll.initCOG[2],gStepSample+4,gInpInvPendulumHeight+gStepSample*(gNumOfStep-4)+4);

		//gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.initCOG[2]-COGDown/3,gKineAll.initCOG[2],gStepSample+2,gInpInvPendulumHeight+gStepSample*(gNumOfStep-4)+2);
		for (int ii=gStepSample*(gNumOfStep-3)+4 ; ii< gTrajLength+8 ; ii++)
		{
			if (ii >= gStepSample+4)
			{
				//gInpCOG[ii] = gKineAll.initCOG[2]-ii*0.1; // testing cogz input
				gInpInvPendulumHeight[ii] = gKineAll.initCOG[2]; // testing cogz input
			}
		}

		// 零加上半階高度 先存在 gInpCOG 這是機器人在上樓梯時重心高度的波動
		for (int ii = 1; ii < gNumOfStep; ii++)
		{
			gKineAll.GenSmoothZMPShift_ZeroJerk((gGroundHeight[ii-1]+gGroundHeight[ii])*0.5,(gGroundHeight[ii]+gGroundHeight[ii+1])*0.5,gStepSample,gInpCOG+4+gStepSample*ii);
		}

		for (int ii = 0 ; ii < gTrajLength+8 ; ii++)
		{
			gInpCOG[ii] = gInpCOG[ii] + gInpInvPendulumHeight[ii]; // 真正重心軌跡 現在 gInpInvPendulumHeight只是暫存 無物理意義
		}

		// 計算地面高度軌跡
		for (int ii = 1; ii < gNumOfStep; ii++)
		{
			gKineAll.GenSmoothZMPShift_ZeroJerk(gGroundHeight[ii],gGroundHeight[ii+1],gStepSample,gInpZMPHeight+4+gStepSample*ii);
		}

		////fstream LoadCOGz;

		////LoadCOGz.open("czTP6p0.txt",ios::in);

		////for (int i = 0 ; i < 20000 ; i++)
		////{
		////	  LoadCOGz >> gInpCOG[i+gStepSample+4];
		////	  gInpCOG[i+gStepSample+4] -= 504.56;
		////	  gInpCOG[i+gStepSample+4]*= 0.7;
		////	  gInpCOG[i+gStepSample+4] += (504.56-32.8+3.3);
		////}
		////LoadCOGz.close();

		////gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.initCOG[2],gInpCOG[gStepSample+4],gStepSample+4,gInpCOG);
		////LoadCOGz.open("resultCOG.txt",ios::out);
		////for (int i = 0 ; i < 20000+gStepSample+4 ; i++)
		////{
		////	   LoadCOGz << gInpCOG[i] << " ";
		////}
		////LoadCOGz.close();

		//// 真正算出倒單擺高度 = COG真正軌跡 - 地面高度軌跡
		//for (int ii = 0 ; ii < gTrajLength+8 ; ii++)
		//{
		//	gInpInvPendulumHeight[ii] = gInpCOG[ii] - gInpZMPHeight[ii]; 
		//}
		//fstream fCOGz;
		//	fCOGz.open("COGz.txt",ios::out| ios::app);
		//	//fCOGz<<123<<"\t";
		//	for(int i=0;i<gTrajLength+8;i++)
		//	{
		//		fCOGz<<gInpInvPendulumHeight[i]<<"\t";
		//	}
		//	fCOGz<<"\n";
		//	fCOGz.close();


	    gLQs.Initval(gInpInvPendulumHeight);
		gLQs.BackRiccati(gInpZMPx,gInpZMPy);
        gLQs.DummyControl();
}
//void gCalculateFlatuatedCOG(double COGDown, double COGUp)
//{
//	/******************************************************************
//	input: COGDown: 在這段軌跡中來機器人要蹲下的高度
//	output: void
//
//	Note:
//	// 根據設定好的場景規劃COG軌跡
//	******************************************************************/
//
//		// 初始化
//		for (int ii = 0 ; ii < gTrajLength+8 ; ii++)
//		{
//			gInpZMPHeight[ii] = 0.0;
//			gInpCOG[ii] = 0.0;
//		}
//
//		// 初始COG 軌跡 平的 暫存在gInpInvPendulumHeight裡面
//		gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.initCOG[2],gKineAll.initCOG[2]-COGDown/3,gStepSample+2,gInpInvPendulumHeight);
//		gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.initCOG[2]-COGDown/3,gKineAll.initCOG[2]-COGDown,gStepSample+2,gInpInvPendulumHeight+gStepSample+2);
//
//	//int gNza; // 將一步切成三段 第一段格點數  gNza = gStepSample * DSP /2.0
//	//int gNab; // 將一步切成三段 第二段格點數 gNab = gStepSample * SSP
//	//int gNzb; // 將一步切成三段 第三段格點數  gNb = gStepSample - gNza - gNab
//
//
//	//gKineAll.GenSwingTrajMod(0,0,0.5,gKineAll.StepHeight[gKineAll.stepIndex]+gStrideZ,0,gStrideX,gStrideZ,0,0,gKineAll.Nab,0, gKineAll.SwingBufferx, gKineAll.SwingBufferz);
//	//gKineAll.GenSwingTrajMod(0,0,0.5,gKineAll.StepHeight[gKineAll.stepIndex]+gStrideZ,0,gStrideY,gStrideZ,0,0,gKineAll.Nab,0, gKineAll.SwingBuffery, gKineAll.SwingBufferz);
//
//
//		for (int ii=2 ; ii< (gNumOfStep-5) ; ii++)
//		{
//				double* tempCOGZ;
//				double ccccc=2000;
//				tempCOGZ=&ccccc;
//				//gInpCOG[ii] = gKineAll.initCOG[2]-ii*0.1; // testing cogz input
//				for(int j=0;j<gNza+0.1*gNab;j++)
//					gInpInvPendulumHeight[j+gStepSample*ii+4]=gKineAll.initCOG[2]-COGDown;
//				//gKineAll.GenCOGZTrajMod(0,0,gKineAll.initCOG[2]-COGDown,0.5,COGUp-COGDown,0,gStrideX,gStrideZ,0,0,gNab,0,tempCOGZ,gInpInvPendulumHeight+gStepSample*ii+gNza+4);
//				gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.initCOG[2]-COGDown,gKineAll.initCOG[2]-COGUp,gStepSample*SSP/5*2,gInpInvPendulumHeight+gStepSample*ii+gNza+gNab/10+4);
//				gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.initCOG[2]-COGUp,gKineAll.initCOG[2]-COGDown,gStepSample*SSP/5*2,gInpInvPendulumHeight+gStepSample*ii+gNza+gNab/2+4);
//				//for(int j=0;j<gNab;j++)
//				//	gInpInvPendulumHeight[j+gNza+gStepSample*ii+4]=gKineAll.initCOG[2]-COGDown;
//				for(int j=0;j<gNzb+0.1*gNab;j++)
//					gInpInvPendulumHeight[j+gStepSample*ii+4+gNza+gNab/10*9]=gKineAll.initCOG[2]-COGDown;
//				//gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.initCOG[2]-COGDown,gKineAll.initCOG[2]-COGUp,gStepSample*DSP/2,gInpInvPendulumHeight+gStepSample*ii+gNza+gNab+4);
//				//gInpInvPendulumHeight[ii] = gKineAll.initCOG[2] - COGUp; // testing cogz input
//
//		}
//		gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.initCOG[2]-COGDown,gKineAll.initCOG[2]-COGDown/3,gStepSample+2,gInpInvPendulumHeight+gStepSample*(gNumOfStep-5)+4);
//		gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.initCOG[2]-COGDown/3,gKineAll.initCOG[2],gStepSample+2,gInpInvPendulumHeight+gStepSample*(gNumOfStep-4)+2);
//		for (int ii=gStepSample*(gNumOfStep-3)+4 ; ii< gTrajLength+8 ; ii++)
//		{
//			if (ii >= gStepSample+4)
//			{
//				//gInpCOG[ii] = gKineAll.initCOG[2]-ii*0.1; // testing cogz input
//				gInpInvPendulumHeight[ii] = gKineAll.initCOG[2]; // testing cogz input
//			}
//		}
//
//		// 零加上半階高度 先存在 gInpCOG 這是機器人在上樓梯時重心高度的波動
//		for (int ii = 1; ii < gNumOfStep; ii++)
//		{
//			gKineAll.GenSmoothZMPShift_ZeroJerk((gGroundHeight[ii-1]+gGroundHeight[ii])*0.5,(gGroundHeight[ii]+gGroundHeight[ii+1])*0.5,gStepSample,gInpCOG+4+gStepSample*ii);
//		}
//
//		for (int ii = 0 ; ii < gTrajLength+8 ; ii++)
//		{
//			gInpCOG[ii] = gInpCOG[ii] + gInpInvPendulumHeight[ii]; // 真正重心軌跡 現在 gInpInvPendulumHeight只是暫存 無物理意義
//		}
//
//		// 計算地面高度軌跡
//		for (int ii = 1; ii < gNumOfStep; ii++)
//		{
//			gKineAll.GenSmoothZMPShift_ZeroJerk(gGroundHeight[ii],gGroundHeight[ii+1],gStepSample,gInpZMPHeight+4+gStepSample*ii);
//		}
//
//		////fstream LoadCOGz;
//
//		////LoadCOGz.open("czTP6p0.txt",ios::in);
//
//		////for (int i = 0 ; i < 20000 ; i++)
//		////{
//		////	  LoadCOGz >> gInpCOG[i+gStepSample+4];
//		////	  gInpCOG[i+gStepSample+4] -= 504.56;
//		////	  gInpCOG[i+gStepSample+4]*= 0.7;
//		////	  gInpCOG[i+gStepSample+4] += (504.56-32.8+3.3);
//		////}
//		////LoadCOGz.close();
//
//		////gKineAll.GenSmoothZMPShift_ZeroJerk(gKineAll.initCOG[2],gInpCOG[gStepSample+4],gStepSample+4,gInpCOG);
//		////LoadCOGz.open("resultCOG.txt",ios::out);
//		////for (int i = 0 ; i < 20000+gStepSample+4 ; i++)
//		////{
//		////	   LoadCOGz << gInpCOG[i] << " ";
//		////}
//		////LoadCOGz.close();
//
//		//// 真正算出倒單擺高度 = COG真正軌跡 - 地面高度軌跡
//		//for (int ii = 0 ; ii < gTrajLength+8 ; ii++)
//		//{
//		//	gInpInvPendulumHeight[ii] = gInpCOG[ii] - gInpZMPHeight[ii]; 
//		//}
//		//fstream fCOGz;
//		//	fCOGz.open("COGz.txt",ios::out| ios::app);
//		//	//fCOGz<<123<<"\t";
//		//	for(int i=0;i<gTrajLength+8;i++)
//		//	{
//		//		fCOGz<<gInpInvPendulumHeight[i]<<"\t";
//		//	}
//		//	fCOGz<<"\n";
//		//	fCOGz.close();
//
//
//	    gLQs.Initval(gInpInvPendulumHeight);
//		gLQs.BackRiccati(gInpZMPx,gInpZMPy);
//        gLQs.DummyControl();
//}
void gRefreshControlSystem(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 銜接兩段動作中間的處理與初始化，記住機器人在上一段結束時的狀態
	******************************************************************/

	gKineAll.stepIndex = 0;
	gKineAll.selIK = gKineAll.selSupport[gKineAll.stepIndex];

	gKineAll.remLL[0] = gKineAll.CrdAll->data[18];
	gKineAll.remLL[1] = gKineAll.CrdAll->data[19];
	gKineAll.remLL[2] = gKineAll.CrdAll->data[20];
	gKineAll.shiftLL[0] = gKineAll.CrdAll->data[21];
	gKineAll.shiftLL[1] = gKineAll.CrdAll->data[22];
	gKineAll.shiftLL[2] = gKineAll.CrdAll->data[23];

	gKineAll.remRL[0] = gKineAll.CrdAll->data[57];
	gKineAll.remRL[1] = gKineAll.CrdAll->data[58];
	gKineAll.remRL[2] = gKineAll.CrdAll->data[59];
	gKineAll.shiftRL[0] = gKineAll.CrdAll->data[60];
	gKineAll.shiftRL[1] = gKineAll.CrdAll->data[61];
	gKineAll.shiftRL[2] = gKineAll.CrdAll->data[62];

	gKineAll.GetLegsCoords();
	gKineAll.LSwitchRMot[0] = gKineAll.LLegRotM[0];
	gKineAll.LSwitchRMot[1] = gKineAll.LLegRotM[1];
	gKineAll.LSwitchRMot[2] = gKineAll.LLegRotM[2];
	gKineAll.LSwitchRMot[4] = gKineAll.LLegRotM[3];
	gKineAll.LSwitchRMot[5] = gKineAll.LLegRotM[4];
	gKineAll.LSwitchRMot[6] = gKineAll.LLegRotM[5];
	gKineAll.LSwitchRMot[8] = gKineAll.LLegRotM[6];
	gKineAll.LSwitchRMot[9] = gKineAll.LLegRotM[7];
	gKineAll.LSwitchRMot[10] = gKineAll.LLegRotM[8];
	gKineAll.RSwitchRMot[0] = gKineAll.RLegRotM[0];
	gKineAll.RSwitchRMot[1] = gKineAll.RLegRotM[1];
	gKineAll.RSwitchRMot[2] = gKineAll.RLegRotM[2];
	gKineAll.RSwitchRMot[4] = gKineAll.RLegRotM[3];
	gKineAll.RSwitchRMot[5] = gKineAll.RLegRotM[4];
	gKineAll.RSwitchRMot[6] = gKineAll.RLegRotM[5];
	gKineAll.RSwitchRMot[8] = gKineAll.RLegRotM[6];
	gKineAll.RSwitchRMot[9] = gKineAll.RLegRotM[7];
	gKineAll.RSwitchRMot[10] = gKineAll.RLegRotM[8];

	for (int pp = 0 ; pp < gStepSample ; pp++)
	{
		gAngLBuf[pp] = gLAngZWorld;
		gAngRBuf[pp] = gRAngZWorld;
	}

	if (gFlagCurrentPos == 0) // 腳完全伸直
	{
		//gInitThetas
		for (int k = 0 ; k < 6 ; k++)
		{
			gKineAll.FKLLeg->theta[k+1] = gInitThetas[k]*gPNJoints[k];
			gKineAll.FKRLeg->theta[k+1] = gInitThetas[k+6]*gPNJoints[k+6];
		}
		gKineAll.FindFK();
		gKineAll.FindCOG();
		gKineAll.initCOG[0] = gKineAll.COG[0]; // 到達DH初始值 機器人不再singular 的 初始COG
		gKineAll.initCOG[1] = gKineAll.COG[1]; // 到達DH初始值 機器人不再singular 的 初始COG
		gKineAll.initCOG[2] = gKineAll.COG[2]; // 到達DH初始值 機器人不再singular 的 初始COG
		for (int k = 0 ; k < 6 ; k++)
		{
			gKineAll.FKLLeg->theta[k+1] = 0;
			gKineAll.FKRLeg->theta[k+1] = 0;
		}
		gKineAll.FindFK(); // 洗掉CrdAll的暫存值 這樣畫圖就不會出問題

	}
	else if (gFlagCurrentPos == 1) // 剛蹲完DH初始值
	{
		gKineAll.FindFK();
		gKineAll.FindCOG();
		gKineAll.initCOG[0] = gKineAll.COG[0]; // 到達DH初始值 機器人不再singular 的 初始COG
		gKineAll.initCOG[1] = gKineAll.COG[1]; // 到達DH初始值 機器人不再singular 的 初始COG
		gKineAll.initCOG[2] = gKineAll.COG[2]; // 到達DH初始值 機器人不再singular 的 初始COG
	}
	else if (gFlagCurrentPos == 2) // 自我介紹偷偷移動重心
	{
		gKineAll.FindFK();
		gKineAll.FindCOG();
		gKineAll.initCOG[0] = gKineAll.COG[0]; // 到達DH初始值 機器人不再singular 的 初始COG
		gKineAll.initCOG[1] = gKineAll.COG[1]; // 到達DH初始值 機器人不再singular 的 初始COG
		gKineAll.initCOG[2] = gKineAll.COG[2]; // 到達DH初始值 機器人不再singular 的 初始COG
	}
		
	gIthIK = 0;
	gSendContTraj = 1;
	gContTrajLock = 0;
	gStartTimeAcquired = 1;
}

void gPrepareScenarioScript(int SecNo)
{
	/******************************************************************
	input: SecNo: 選擇腳本號碼
	output: void

	Note:
	// 腳本選擇函式 本函式中紀錄了多個腳本，藉由輸入SecNo來選擇腳本
	// 假若使用gPrepareScenarioScript(Index++)方式來呼叫的話 可以照順序撥放腳本
	******************************************************************/

	// 本函式準備下一段動作要執行的任務
	// SecNo: 0 is the default section, defined in push button 1, cannot be edited here

    // 不允許單腳站立模式 每個腳本裡面也應該要設定 在這裡清除為零是因為怕以後的新腳本沒有加上清除


	/******************************************************************
	Initialization for every script  //0715

	Note:
		所有各個script內會切換的flag之initialization應該放置於此
	******************************************************************/

	gKineAll.FlagSumoMode = 0; // 先預設不是sumo mode 在下面要用才會改成1
	gKineAll.FlagStayMode = 0; // 先預設不是stay mode 在下面要用才會改成1 
	
	//暫時性措施, 讓手語軌跡之scenario後不會因為繼續讀取軌跡而亂動  //0715
	AfterSignLanguageFlag = 0; 
	//Preload Torque 讀檔使用Slongz 0528
	gSecNumb=SecNo;
	
	if (SecNo > 0) // 第一次不須執行
		gRefreshControlSystem(); // 重設系統

	//unsigned char comment[5]={255,0,8,0,255}; // for robot face LED array 現在在gIntLEDFace中宣告

	/******************************************************************
	Flag
		gIKMethod 1: IK解12軸 0:IK解24軸
		gFlagGoBackPos: Scenario結束時是否將腳伸直  ZeroHome; BentHome; ShiftZeroHome; SlopeHome;  Warning!!! 手語軌跡後的Zerohome / ShiftZerohome 之後之script的蹲下動作會有不連續的暴衝現象,需要debug 
		AfterSignLanguageFlag: 1: 限定手語軌跡後的scenario手臂保持不作動狀態 Warning!!!數值限定非完整state machine 需要進行整理Warning!!!
		gFlagSlSaid01: Demo 2nd 語音啟動使用(於motion control thread觸發)(一次限定)
		gFlagSlSaid02
		gFlagSlSaid03
	參數
		ZMPRatio:  Lateral distance的壓縮量 可能範圍:0.75~1.0 (視ZMP的狀況而定) Warning!!! ZMP Ratio 無法對旋轉動作使用Warning!!! 

	Note:
		此函式在SecNo為零時會先在各個theread尚未開啟前便執行,需注意SecNo 0中有無相關功能未初始化導致衝突
	******************************************************************/
	if (SecNo == 3) // 劇情結束
	{
        gEndOfScript = true;
	}

	if (gEndOfScript == false)
	{
		printf("Preparing Motions of Section %d ...",SecNo);
		// 設定新一段軌跡

		if (SecNo == 0) // upstair
		{
		
			#if QCCDMode
				gIKMethod = 1;
				#if ConstantCOGMode
					gInitQCCDWalkStraight(15,WalkRatio*325.0);
				#else
					gInitQCCDWalkStraight(15,WalkRatio*330);
				#endif
				//gInitTurnLeft();
				gSpeaker.LoadSoundFile("SoundFiles/InitialSpeak.wav");
			#else
				 gIKMethod = 1;	//  = 1 只解腳
				 //gInitStaySquatMode(10);
				 gFlagGoBackPos = BentHome;//BentHome;
				 //ZMPRatio=0.85;

			//////斜坡
				 ////gWalkSlopeDora1();
				 //gFlagGoBackPos = SlopeHome;
			//////樓梯測試
				 //gInitLeft_leg_up_and_down_OneStep(); // Demo 2nd 前的測試程式: 踏一階後腳抬起後前腳踏回
				// gInitLeft_leg_up_and_down_TwoStep(); // Demo 2nd 前的測試程式: (似乎是失敗函式需經確認)

			//Walk Straight  Arm請配合大金Config
				gInitWalkStraight(9,200);
				 
				//gInitArmWaveWalkStraight(); 
			
			////TrinRight  Arm請配合大金Config
				//gInitTurnRight(14,300,90,200);
				//gInitArmWaveTurn(DirectionRight);

			////3D  
				//Slongz 0517 上樓梯改
				//gInitUpStairMod(11,460,40);

				//Slongz 0517 下樓梯
				//gInitStairMod(9,468,-40);
	
				// Dora1
				//gInitdownStair(); //可能需要檢查

				// WZ FK COG
				 //gFlagGoBackPos = BentHome;
				 //gInitWalkStraight(9,200);
				 //gInitSquat();
	    
			
			////Self Intro.  Mode1,2 請配合大金Config Mode 3 請配合聖翔Config
				//gInitStayMode(400); // 包含一秒從 shift zero home 到 zero home	
				//gInitArmIntroductionMode(400,3,5);//自我介紹用
			
				//gSpeaker.PlaySoundByFile(gSpeaker.Sound1); 
				//gSpeaker.LoadSoundFile("SoundFiles/InitialSpeak.wav");

			#endif
		}
		else if (SecNo == 1) // down stair
		{
			#if QCCDMode
			gEndOfScript = true;
			#else
		
			/*gSpeaker.LoadSoundFile("SoundFiles/Intro1.wav");
			gSpeaker.PlaySoundByFile(gSpeaker.Sound1); */
		
			//Sign Lang  Arm請配合聖翔Config
				//
				//gSpeaker.LoadSoundFile("SoundFiles/N03.wav");//N:對於人形機器人來說平地行走要保持平衡是一個相當困難的問題，
				//											//N:而上下樓梯又是更加的不容易呢。
				//gSpeaker.PlaySoundByFile(gSpeaker.Sound1); 
			 //   gInitWalkStraight(7,40);

				//gInitStayMode(400); // 包含一秒從 shift zero home 到 zero home	
				//gInitArmIntroductionMode(400,3,5);//自我介紹用
			gIKMethod = 1;	//  = 1 只解腳
			gInitStaySquatMode(10);
			//gInitWalkStraight(9,200);
			gFlagGoBackPos = BentHome;//BentHome;
				//gInitArmManualWalkStraight(3);
				 //gIKMethod = 1;
				 //ZMPRatio=0.85;
				 //gFlagGoBackPos = BentHome;//BentHome;
				 //gInitStairMod(9,465,-40);
				//gInitStair();
				// gInitStairMod(7,460,40);
				
				//gInitWalkStraight(9,200);
			
				
			
				 //gWalkSlopeDora1();
				 //gFlagGoBackPos = SlopeHome;
		

			//gFlagGoBackPos = BentHome;; // 一定要放在自我介紹前面 不能亂放 機器人偷偷往後一點點 讓手臂舉起來更難摔倒
			#endif
		}
		else if (SecNo == 2) // turn right
		{
			gIKMethod = 1;
			//ZMPRatio=1.0;
			gInitWalkStraight(9,200);
			//gInitTurnRight(13,200,90,200);
			gFlagGoBackPos = BentHome;//ZeroHome;
			//#if SoundCtrl
			//gSpeaker.LoadSoundFile("SoundFiles/N04.wav");
			////gSpeaker.PlaySoundByFile(gSpeaker.Sound1);
			//#endif
			/*gFlagSlSaid01=true;
			gFlagSlSaid02= true;
			gFlagSlSaid03= true;*/
		}
		else if (SecNo == 3) // 機器人腳不動 手語
		{
		
			gIntLEDFace(1);		
			gIKMethod = 1;
			gInitStayMode(400); // 包含一秒從 shift zero home 到 zero home	
			gInitArmIntroductionMode(400,3,5);//自我介紹用
			gFlagGoBackPos = BentHome;
		}
		else if (SecNo == 4) // turn right
		{
			AfterSignLanguageFlag = 1;
			//gKineAll.FlagStayMode = 0;
			gIKMethod = 1;
			gStartTimeAcquired=0; //gSystime 重算 Warning!!! 意義不明 Warning!!!
			//ArmOfflinMethod=1;
			//for(int i =0;i<42000;i++)
			//{
			//	for(int j=0;j<6;j++)
			//	{
			//	gLArmOfflineTraj[6*i+j]=0;//gKineAll.FKLArm->theta[i+1]*3.1415926/180;
			//	gRArmOfflineTraj[6*i+j]=0;//gKineAll.FKRArm->theta[i+1]*3.1415926/180;		
			//	}
			//}
		
			ZMPRatio=1.0;
			gInitTurnRight(13,200,90,200);
		
		
			gFlagGoBackPos = BentHome;//ShiftZeroHome;
		}
		else if (SecNo == 5) // walk straight
		{
			AfterSignLanguageFlag = 1;
			gIKMethod = 1;
			ZMPRatio=1.0;
			//gInitWalkStraight(11,200);
			gInitWalkStraight(11,200);
			gFlagGoBackPos = BentHome;
		}
		//	gSpeaker.LoadSoundFile("SoundFiles/SayHiToBunny.wav");
		//	gSpeaker.PlaySoundByFile(gSpeaker.Sound1);
		//	Sleep(3500);

		//	//gSpeaker.LoadSoundFile("SoundFiles/Intro1.wav");
		//	//gSpeaker.PlaySoundByFile(gSpeaker.Sound1); 
		//	gFlagGoBackPos = ZeroHome; // 限定ZeroHome 腳要伸直 不給彎 要改的話就要配合改 MotionControlThread

		//	//gInitStayMode(36.5-8.0); // 包含一秒從 shift zero home 到 zero home	
		//	//gInitArmIntroductionMode(36.5-8.0,1,5);//自我介紹用

		//	// face command
		//	if(gFlagSimulation == 2)
		//	{
		//		//gpPortHead->_write(comment,5);
		//		//gpPortHead->_write(comment,5);
		//		//gpDP->TransENC(5,  ID_SetENC+(2)*CMD_SET);
		//		//gpPortHead->_write(gpDP->TxData,9);
		//	}
		//
		//}
		//else if (SecNo == 2) // 螃蟹行走 右方
		//{
		//	gInitSideWalk(DirectionRight);
		//	gInitArmSideWalkWave(DirectionRight);
		//	gFlagGoBackPos = BentHome;
		//	gSpeaker.LoadSoundFile("SoundFiles/Intro2.wav");
		//	gSpeaker.PlaySoundByFile(gSpeaker.Sound1);
		//	if(gFlagSimulation == 2)
		//	{
		//		//gpPortHead->_write(comment,5);
		//		//gpPortHead->_write(comment,5);
		//		//gpDP->TransENC(5,  ID_SetENC+(2)*CMD_SET);
		//		//gpPortHead->_write(gpDP->TxData,9);
		//	}
		//	
		//}
		//else if (SecNo == 3) // 螃蟹行走 左方
		//{
		//	 gEndOfScript = true;
		//	gSpeaker.LoadSoundFile("SoundFiles/Intro3.wav");
		//	gSpeaker.PlaySoundByFile(gSpeaker.Sound1); // 語音23秒
		//	gFlagGoBackPos = BentHome;
		//	gInitSideWalk(DirectionLeft);	 
		//	gInitArmSideWalkWave(DirectionLeft);
		//	if(gFlagSimulation == 2)
		//	{
		//		//gpPortHead->_write(comment,5);
		//		//gpPortHead->_write(comment,5);
		//		//gpDP->TransENC(5,  ID_SetENC+(2)*CMD_SET);
		//		//gpPortHead->_write(gpDP->TxData,9);
		//	}
		//	
		//}
		//////因為機器人SA57過熱 所以取消單腳站立 不然SA57會燒掉
		////else if (SecNo == 5)
		////{
		////	gSpeaker.LoadSoundFile("SoundFiles/Intro3.wav");
		////	gSpeaker.PlaySoundByFile(gSpeaker.Sound1); // 語音23秒
		////	//gKineAll.FlagSumoMode = 1; // 允許單腳站立模式
		////	gInitSumoMotion(); // 抬起單腳 類似相撲動作
		////	gInitArmSumoMotion();
		////	gFlagGoBackPos = BentHome;

		////}
		//else if (SecNo == 2) // 機器人後退 準備退場
		//{
		//	gSpeaker.LoadSoundFile("SoundFiles/ThankITRI.wav");
		//	gSpeaker.PlaySoundByFile(gSpeaker.Sound1); // 語音23秒

		//	gFlagGoBackPos = BentHome;//ShiftZeroHome;

		//	////Sign Lang  Arm請配合聖翔Config
		//		gInitWalkStraight(9,-200);
		//		gInitArmManualWalkStraight(3);

		//	if(gFlagSimulation == 2)
		//	{
		//		//gpPortHead->_write(comment,5);
		//		//gpPortHead->_write(comment,5);
		//		//gpDP->TransENC(5,  ID_SetENC+(6)*CMD_SET);
		//		//gpPortHead->_write(gpDP->TxData,9);
		//	}

		//	

		//}
		//else if (SecNo == 100) // 機器人揮手 說再見
		//{
		//	gSpeaker.LoadSoundFile("SoundFiles/SayGoodBye.wav");
		//	gSpeaker.PlaySoundByFile(gSpeaker.Sound1);

		//	gFlagGoBackPos = BentHome;

		//	gInitStayMode(19); // 包含一秒從 shift zero home 到 zero home	
		//	gInitArmIntroductionMode(19,2,6);//自我介紹用
		//	if(gFlagSimulation == 2)
		//	{
		//		gpPortHead->_write(comment,5);
		//		gpPortHead->_write(comment,5);
		//		gpDP->TransENC(5,  ID_SetENC+(6)*CMD_SET);
		//		gpPortHead->_write(gpDP->TxData,9);
		//	}
		//}
		//else if (SecNo == 5) // 機器人轉彎回家
		//{
		//	gFlagGoBackPos = BentHome;
		//	gInitTurnRight(13,200,90,200); 	
		//	gInitArmWaveTurn(DirectionRight);
		//	if(gFlagSimulation == 2)
		//	{
		//		gpPortHead->_write(comment,5);
		//		gpPortHead->_write(comment,5);
		//		gpDP->TransENC(5,  ID_SetENC+(1)*CMD_SET);
		//		gpPortHead->_write(gpDP->TxData,9);
		//	}
		//}
		//else if (SecNo == 6) // 機器人轉完彎 直走回家
		//{
		//	gFlagGoBackPos = BentHome;//ZeroHome;
		//	gInitWalkStraight(9,250); 
		//	gInitArmWaveWalkStraight();
		//	if(gFlagSimulation == 2)
		//	{
		//		gpPortHead->_write(comment,5);
		//		gpPortHead->_write(comment,5);
		//		gpDP->TransENC(5,  ID_SetENC+(1)*CMD_SET);
		//		gpPortHead->_write(gpDP->TxData,9);
		//	}
		//}
	
		gCalculateZMP(); // 重算ZMP軌跡
		//gCalculate_NEW_ZMP(); //新的ZMP軌跡  如果有需要用就關掉上面舊的 把這一個打開

		#if RunDynamics
			gKineAll.Distributor(gInpZMPx, gInpZMPy, gFstpY ,gFstpX, gNumOfStep, gInterNum, gStopTrajLen, gFlagGoBackPos);	// WZ
		#endif

		#if ConstantCOGMode
			gCalculateCOG(gCOGDown); // 重算COG軌跡
		#else
			gCalculateFlatuatedCOG(gCOGDown,gCOGUp);
		#endif

		printf("Done!!\n");	
	}

	QueryPerformanceCounter(&gStartTime); // 重新計時

	if (SecNo > 0) // 第一次不須執行 小段計時部分
	{
		QueryPerformanceCounter(&gGlobalCurrentTime);
		double GlobalTimePrint = (gGlobalCurrentTime.QuadPart - gGlobalTime.QuadPart)/gFreqT;
		printf("Global Time = %f\n",GlobalTimePrint);
	}
}

void CRobotAllDlg::OnBnClickedCheck10()
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 設定要不要開啟IMU模組

	******************************************************************/
	if (CheckIMU.GetCheck())
	{
		gFlagIMU = 1;
		CheckIMU.SetCheck(1);
	}
	else
	{
		gFlagIMU = 0;
		CheckIMU.SetCheck(0);
	}
}


void CRobotAllDlg::OnBnClickedCheck11()
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 設定要不要開啟OPENGL model

	******************************************************************/
	if (CheckOpenGL.GetCheck())
		gFlagGLMode = 1;//OPENGL model
	else
		gFlagGLMode = 0;//棒棒人
}


void CRobotAllDlg::OnBnClickedCheck12()
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 設定要不要開啟OPENGL model

	******************************************************************/
	// TODO: Add your control notification handler code here
	if (gFlagManualMode == 0)
	{
		gFlagManualMode = 1;
		CheckEncoder.SetCheck(1);
		SendIndex = 2;
		cout<<"Sending Mode is Changing to Manual Mode "<<endl;
		//cout<<"欲手動調整各軸角度 請鍵入 1 "<<endl;
		//cin>> CaliAngleMode;
	}
	else if (gFlagManualMode == 1)
	{
		gFlagManualMode = 0;
		CheckEncoder.SetCheck(0);
		SendIndex = 5;
		cout<<"Sending Mode is Changing to Traj Mode "<<endl;
		//CaliAngleMode = 0;
	}

}


void CRobotAllDlg::OnBnClickedCheck13()
{
	// TODO: Add your control notification handler code here
	if (CheckPE.GetCheck())
		gFlagPE = 1;//OPENGL model
	else
		gFlagPE = 0;//棒棒人
	//gFlagPE

}



void gCalculate_NEW_ZMP(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 根據設定好的場景規劃新的ZMP軌跡
	//新ZMP軌跡doratom//
	******************************************************************/
	// 計算總軌跡點數
	gTrajLength	= (T_P/dt*gNumOfStep);
	gContTrajLen = gInterNum + gNumOfStep*StepSam;

	if (gKineAll.FlagStayMode == 1 || gKineAll.FlagStaySquatMode == 1) // 機器人是否進入停止不動模式
		gStopTrajLen = int(gNoMotionTime/dt);
	else
		gStopTrajLen = gContTrajLen - int(3.82*StepSam); // 少幾步 讓LQ多算一些

	gLQs.LQDataLen = gNumOfStep*StepSam; // 總長度 = 總步數*每步格數


	/*************************************************************
	                   開始規劃新的ZMP軌跡
	**************************************************************/
	//NEW_ZMP的變數宣告 //
	double new_gPCentermoveX = 15.0;//輸入絕對值//
	double new_gPCentermoveY = 10.0;//"不用"輸入絕對值//
	double new_gPBackmoveX =1.0; //輸入絕對值//
	double new_gPBackmoveY =-36.0; //"不用"輸入絕對值//-36
	double newdistance = 0;
	double newratio_y ;
	double newratio_x ;
	double newratio_z ;
	double firststepy;//上一部是DSP用
	double finalstepx;//上一部是DSP用
	float lenfront = 114.0; //中心到腳尖
	float lenback = 96.0;   //腳後跟到中心
	float lenleft = 62.0;   //左邊到中心
	float lenright = 78.0;  //中心到右邊
	float ratio_lenfront ;  //中心到腳間的距離 要乘的比例
	float ratio_lenback ;   //腳後跟到中心的距離 要乘的比例
	//內插的時候的變數宣告
	//後點到中點
	double interpolationBC_Y_needtimeratio = 0.5; // 後中在Y方向 所需時間的比例
	double interpolationBC_Y_gDivideVel = 10.5;  //後中在Y方向 要算"起始"速度時候 要除的時間的比例
	double interpolationBC_Y_finalvelocityratio = 3.5;//後中在Y方向 要算"末"速度時候 要除的時間的比例
	double interpolationBC_X_needtimeratio = 0.5;//後中在X方向 所需時間的比例
	double interpolationBC_X_gDivideVel = 10.5;//後中在X方向 要算起使速度時候 要除的時間的比例
	double interpolationBC_X_finalvelocityratio = 3.0;//後中在X方向 要算"末"速度時候 要除的時間的比例
	//中點到前點
	double interpolationCF_Y_needtimeratio = 0.5;// 中前在Y方向 所需時間的比例
	double interpolationCF_Y_finalvelocityratio =10.5;//中前在Y方向 要算"末"速度時候 要除的時間的比例
	double interpolationCF_X_needtimeratio = 0.5;// 中前在X方向 所需時間的比例
	double interpolationCF_X_finalvelocityratio= 10.5;//中前在X方向 要算"末"速度時候 要除的時間的比例
	//DSP
	double interpolation_DSP_start_velocity = 8.0; // 要算DSP的"起始"速度時候 要除的時間的比例
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//原本的ZMP變數宣告//
	double DirFB_Now[3]; // 現在這步前後單位向量
	double DirLR_Now[3]; // 現在這步左右單位向量
	double LenFront=0; // ZMP在足底前進的量
	double LenBack=0; // ZMP在足底從後面多少開始的量
	double Front2Step[3] = {0,0,0}; // 記住下一步與這一步的位置差
	double Back2Step[3] = {0,0,0}; // 記住這一步與前一步的位置差
	double DistFBFront2P = 0;
	double DistFBBack2P = 0;
	double DistLRFront2P = 0;
	double DistLRBack2P = 0;

	double VectorDSP[3] = {0,0,0}; // vector between the front point and the last point
	double LastVectorDSP[3] = {0,0,0}; // Last vector between the front point and the last point
	double DsitanceDSP = 0; // norm  of VectorDSP
	double LastDsitanceDSP = 0; // norm  of LastVectorDSP

	double VecFB_Last = 0; // 上一個在腳底往前後方向的ZMP移動長度
	double VecLR_Last = 0; // 上一個在腳底往左右方向的ZMP移動長度 
	double VecFB_Now = 0; // 在腳底往前後方向的ZMP移動長度
	double VecLR_Now = 0; // 在腳底往左右方向的ZMP移動長度 
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	if (gKineAll.FlagSumoMode == 0)
	{
		#if ConstantCOGMode
			gLateralZMPShift = 6; // 側向ZMP移動量 for 自然ZMP
		#else
			gLateralZMPShift = 8; // 側向ZMP移動量 for 自然ZMP
		#endif
	    
		DistLRFront2P = gLateralZMPShift;
		ratio_lenfront = 0.35;  //中心到腳間的距離 要乘的比例
	    ratio_lenback = 0.25;   //腳後跟到中心的距離 要乘的比例
	    newratio_y = 1.5; //當我一但算出後中的向量長度(Y方向)之後  我要算前點的位置Y 可以用 前點位置Y = 中點位置Y +  newratio_y * 後中的向量長度(Y方向)   就可以得到new_gPFrontY
		newratio_x = 1.2; ////當我一但算出後中的向量長度(X方向)之後  我要算前點的位置X 可以用 前點位置X = 中點位置X +  newratio_x * 後中的向量長度(X方向) 就可以得到new_gPFrontX
		newratio_z = 0;
		firststepy = 90;//上一步是DSP用 
		finalstepx = 90;//上一步是DSP用	
	}
	else
	{
		gLateralZMPShift = 0.0; 
	    
		DistLRFront2P = gLateralZMPShift;
		ratio_lenfront = 0.0;  //中心到腳間的距離 要乘的比例
	    ratio_lenback = 0.0;
	}

	//20120925//
	if(checkWalkOneStepHigh ==1)//第一步就跨東西//
	{
		 new_gPCentermoveX = 10.0;//輸入絕對值//
		 new_gPCentermoveY = 5.0;//"不用"輸入絕對值//
		 new_gPBackmoveX = 5.0; //輸入絕對值//
		 new_gPBackmoveY = 3.0; //"不用"輸入絕對值//
		 newratio_y = 1.86; //當我一但算出後中的向量長度(Y方向)之後  我要算前點的位置Y 可以用 前點位置Y = 中點位置Y +  newratio_y * 後中的向量長度(Y方向)   就可以得到new_gPFrontY
		 newratio_x = 0.44; ////當我一但算出後中的向量長度(X方向)之後  我要算前點的位置X 可以用 前點位置X = 中點位置X +  newratio_x * 後中的向量長度(X方向) 就可以得到new_gPFrontX
	}
	//20120925//

	//20120926//
	if(this_step ==1) //只有這一腳踩上障礙物
	{
		 new_gPCentermoveX = 10.0;//輸入絕對值//
		 new_gPCentermoveY = 5.0;//"不用"輸入絕對值//
		 new_gPBackmoveX = 8.0; //輸入絕對值//
		 new_gPBackmoveY = 3.0; //"不用"輸入絕對值//
		 newratio_y = 2.5; //當我一但算出後中的向量長度(Y方向)之後  我要算前點的位置Y 可以用 前點位置Y = 中點位置Y +  newratio_y * 後中的向量長度(Y方向)   就可以得到new_gPFrontY
		 newratio_x = 1.0; ////當我一但算出後中的向量長度(X方向)之後  我要算前點的位置X 可以用 前點位置X = 中點位置X +  newratio_x * 後中的向量長度(X方向) 就可以得到new_gPFrontX
	}
	//20120926//

	//走斜坡doratom//
	if(check_slopeangle ==1) //走斜坡
	{
		 new_gPCentermoveX = 15.0;//輸入絕對值//
		 new_gPCentermoveY = 5.0;//"不用"輸入絕對值//
		 new_gPBackmoveX = 10.0; //輸入絕對值//
		 new_gPBackmoveY = -5.0; //"不用"輸入絕對值//
		 newratio_y = 3.2; //當我一但算出後中的向量長度(Y方向)之後  我要算前點的位置Y 可以用 前點位置Y = 中點位置Y +  newratio_y * 後中的向量長度(Y方向)   就可以得到new_gPFrontY
		 newratio_x = 3.2; ////當我一但算出後中的向量長度(X方向)之後  我要算前點的位置X 可以用 前點位置X = 中點位置X +  newratio_x * 後中的向量長度(X方向) 就可以得到new_gPFrontX
	}

	//結束變數宣告//

	//計算現在機器人的旋轉方向 是用在機器人轉彎的時候會用到的//
	// Calculate all foot and body vectors
	for (int i = 0 ; i < gNumOfStep ; i++)
	{
		gRotAngBody[i] = (gLRotAngZ[i]+gRRotAngZ[i])/2.0; // 先指定為兩腳平均角度 以後要做不一樣動作可以改
		gDirLFoot[0][i] = cos(gLRotAngZ[i]); // vector = [cos() sin() 0] 地面角度先設為0
		gDirLFoot[1][i] = sin(gLRotAngZ[i]);
		gDirLFoot[2][i] = 0;
		gDirRFoot[0][i] = cos(gRRotAngZ[i]); // vector = [cos() sin() 0] 地面角度先設為0
		gDirRFoot[1][i] = sin(gRRotAngZ[i]);
		gDirRFoot[2][i] = 0;
		gDirBody[0][i] = cos(gRotAngBody[i]); // vector = [cos() sin() 0] 地面角度先設為0
		gDirBody[1][i] = sin(gRotAngBody[i]);
		gDirBody[2][i] = 0;

		gDirLLateral[0][i] = sin(gLRotAngZ[i]); // vector = 順時鐘旋轉90度 [cos() sin() 0] 地面角度先設為0
		gDirLLateral[1][i] = -cos(gLRotAngZ[i]);
		gDirLLateral[2][i] = 0;
		gDirRLateral[0][i] = -sin(gRRotAngZ[i]); // vector = 逆時鐘旋轉90度[cos() sin() 0] 地面角度先設為0
		gDirRLateral[1][i] = cos(gRRotAngZ[i]);
		gDirRLateral[2][i] = 0;

		gPFront[0][i] = 0; // clear 
		gPFront[1][i] = 0; // clear 
		gPFront[2][i] = 0; // clear 
		gPCenter[0][i] = 0; // clear 
		gPCenter[1][i] = 0; // clear 
		gPCenter[2][i] = 0; // clear 
		gPBack[0][i] = 0; // clear 
		gPBack[1][i] = 0; // clear 
		gPBack[2][i] = 0; // clear 
	}

	//重要!!!開始處理所需要的點的的位置 gPFront gPCenter gPBack new_gPFront new_gPCenter new_gPBack//
	for (int i = 0 ; i < gNumOfStep ; i++)
	{
		//舊的中點
		gPCenter[0][i] = gFstpY[i];
		gPCenter[1][i] = gFstpX[i];
		gPCenter[2][i] = 0;

		Back2Step[0] = Front2Step[0];
		Back2Step[1] = Front2Step[1];
		Back2Step[2] = Front2Step[2];

		Front2Step[0] = gFstpY[i+1]-gFstpY[i];
		Front2Step[1] = gFstpX[i+1]-gFstpX[i];
		Front2Step[2] = 0;

		//新的中點 new_gPCenter
		new_gPCenter[0][i] = gFstpY[i];
		new_gPCenter[1][i] = gFstpX[i];
		new_gPCenter[2][i] = 0;

		//用來測試是否只有跨一步//
		if(checkonestep ==1)
		{
		new_gPCenter[1][3] = 0;
		gPCenter[1][3] = 0;
		}

		////////////////////////////處理new_gPCenter///////////////////////////////////
		if (gKineAll.selSupport[i] == RightSupport) // right support
		{
			if (gKineAll.selSupport[i-1] == DoubleSupport) // 上一步是DSP
			{
			new_gPCenter[0][i] = gFstpY[i]; 
			new_gPCenter[1][i] = gFstpX[i]; 
			new_gPCenter[2][i] = new_gPCenter[2][i];
			}
			else if(gKineAll.selSupport[i+1] == DoubleSupport) // 下一步是DSP
			{
			new_gPCenter[0][i] = gFstpY[i]; 
			new_gPCenter[1][i] = gFstpX[i]+ new_gPCentermoveX; 
			new_gPCenter[2][i] = new_gPCenter[2][i];
			}
			else
			{
			new_gPCenter[0][i] = gFstpY[i]+ new_gPCentermoveY;
			new_gPCenter[1][i] = gFstpX[i]+ new_gPCentermoveX; //因為+x方向為向左邊
			new_gPCenter[2][i] = new_gPCenter[2][i];

			}
		}
		else if (gKineAll.selSupport[i] == LeftSupport) // left support
		{
			if (gKineAll.selSupport[i-1] == DoubleSupport) // 上一步是DSP
			{
			new_gPCenter[0][i] = gFstpY[i]; 
			new_gPCenter[1][i] = gFstpX[i]; 
			new_gPCenter[2][i] = new_gPCenter[2][i];
			}
			else if(gKineAll.selSupport[i+1] == DoubleSupport) // 下一步是DSP
			{
			new_gPCenter[0][i] = gFstpY[i]; 
			new_gPCenter[1][i] = gFstpX[i]- new_gPCentermoveX; 
			new_gPCenter[2][i] = new_gPCenter[2][i];
			}
			else
			{
			new_gPCenter[0][i] = gFstpY[i]+ new_gPCentermoveY;
			new_gPCenter[1][i] = gFstpX[i]- new_gPCentermoveX; //因為+x方向為向左邊
			new_gPCenter[2][i] = new_gPCenter[2][i];
			}
		}
		else if(gKineAll.selSupport[i] == DoubleSupport)	//DSP
		{
			new_gPCenter[0][i] = gFstpY[i] ;
			new_gPCenter[1][i] = gFstpX[i] ; //因為+x方向為向左邊
			new_gPCenter[2][i] = new_gPCenter[2][i];
		}
		////用來測試是否只有跨一步//
 		if(checkonestep ==1){
		new_gPCenter[1][3] = 0;
		gPCenter[1][3] = 0;
		}


		///////////////////////////////////////開始處理new_gPBack 和 new_gPFront/////////////////////////////////////////
		//小心new_gPBack[1][i] 是由gPCenter[1][i]加一個數字決定的//
		if (gKineAll.selSupport[i] == RightSupport) // right support
		{
			lenfront = 114.0; //中心到腳尖
	        lenback = 96.0;   //腳後跟到中心
			if (gKineAll.selSupport[i-1] == DoubleSupport) // 上一步是DSP
			{	
				//指定出新的new_gPBack點
				new_gPBack[0][i] = new_gPCenter[0][i]+gDirRLateral[0][i]*gLateralZMPShift;
				new_gPBack[1][i] = new_gPCenter[1][i]+gDirRLateral[1][i]*gLateralZMPShift;
				new_gPBack[2][i] = new_gPCenter[2][i]+gDirRLateral[2][i]*gLateralZMPShift;

				//算出new_gPCenter和new_gPBack的向量
				newvector[0][i] = (new_gPCenter[0][i]-new_gPBack[0][i]);
				newvector[1][i] = (new_gPCenter[1][i]-new_gPBack[1][i]);
				newvector[2][i] = (new_gPCenter[2][i]-new_gPBack[2][i]);

				//利用上面算出來的向量 再乘上一個比例值 newratio 再加上(剪掉)new_gPCenter 之後就可以得到新的new_gPFront
				new_gPFront[0][i] = new_gPCenter[0][i]+newratio_y*newvector[0][i] + firststepy;  
				// 在這裡加上一個firststepy是因為我的前一步是DSP 如果我不加上這個值這一步的三個數值變成new_gPBack(-74,0,0)  new_gPCenter(-80,0,0)  new_gPFront(-71,0,0)
				//這樣一來ZMP只會在X=0的這一條直線上面跑過去又跑回來而已
				new_gPFront[1][i] = new_gPBack[1][i]-newratio_x*newvector[1][i];//這裡用"-" newratio_x*newvector[1][i] 而不是用 "+" newratio_x*newvector[1][i] 是因為我想要讓我的ZMP越靠近內側越好 而不是遠離內側
				new_gPFront[2][i] = new_gPCenter[2][i]+newratio_z*newvector[2][i];
			}
			else if(gKineAll.selSupport[i+1] == DoubleSupport) // 下一步是DSP
			{	
				//指定出新的new_gPBack點
				new_gPBack[0][i] = new_gPCenter[0][i]+gDirRLateral[0][i]*gLateralZMPShift-ratio_lenback*lenback*gDirRFoot[0][i] + new_gPBackmoveY;				
				if(new_gPCenter[1][i]<0)
				{new_gPBack[1][i] = -(new_gPCenter[1][i-1] - new_gPBackmoveX);}
				if(new_gPCenter[1][i]>0)
				{new_gPBack[1][i] = -(new_gPCenter[1][i-1] + new_gPBackmoveX);}
				
				new_gPBack[2][i] = new_gPCenter[2][i]+gDirRLateral[2][i]*gLateralZMPShift-ratio_lenback*lenback*gDirRFoot[2][i];

				//算出new_gPCenter和new_gPBack的向量
				newvector[0][i] = (new_gPCenter[0][i]-new_gPBack[0][i]);
				newvector[1][i] = (new_gPCenter[1][i]-new_gPBack[1][i]);
				newvector[2][i] = (new_gPCenter[2][i]-new_gPBack[2][i]);
				
				new_gPFront[0][i] = new_gPCenter[0][i];
				new_gPFront[1][i] = new_gPCenter[1][i]+finalstepx;
				if(finalstepx < new_gPCenter[1][i])
				{
					new_gPFront[1][i] = 0.0;
				}
				new_gPFront[2][i] = new_gPCenter[2][i]+newratio_z*newvector[2][i];
			}
			else // 普通行走中
			{
				new_gPBack[0][i] = new_gPCenter[0][i]+gDirRLateral[0][i]*gLateralZMPShift-  ratio_lenback*lenback*gDirRFoot[0][i] + new_gPBackmoveY;
				
				if(new_gPCenter[1][i]>0)
				{new_gPBack[1][i] = new_gPCenter[1][i]+ new_gPBackmoveX;}
				if(new_gPCenter[1][i]<0)
				{new_gPBack[1][i] = new_gPCenter[1][i]-new_gPBackmoveX;}
					
				new_gPBack[2][i] = new_gPCenter[2][i]+gDirRLateral[2][i]*gLateralZMPShift- ratio_lenback*lenback*gDirRFoot[2][i] ;

				newvector[0][i] = (new_gPCenter[0][i]-new_gPBack[0][i]);
				newvector[1][i] = (new_gPCenter[1][i]-new_gPBack[1][i]);
				newvector[2][i] = (new_gPCenter[2][i]-new_gPBack[2][i]);
				
				new_gPFront[0][i] = new_gPCenter[0][i]+newratio_y*newvector[0][i];
				new_gPFront[1][i] = new_gPCenter[1][i]+newratio_x*newvector[1][i];
				new_gPFront[2][i] = new_gPCenter[2][i]+newratio_z*newvector[2][i];
			}
		}
		else if (gKineAll.selSupport[i] == LeftSupport) // left support
		{
			lenfront = 114.0; //中心到腳尖
	        lenback = 96.0;   //腳後跟到中心
			
			if (gKineAll.selSupport[i-1] == DoubleSupport) // 上一步是DSP
			{
				new_gPBack[0][i] = new_gPCenter[0][i]+gDirLLateral[0][i]*gLateralZMPShift;
				new_gPBack[1][i] = new_gPCenter[1][i]+gDirLLateral[1][i]*gLateralZMPShift;
				new_gPBack[2][i] = new_gPCenter[2][i]+gDirLLateral[2][i]*gLateralZMPShift;

				newvector[0][i] = (new_gPCenter[0][i]-new_gPBack[0][i]);
				newvector[1][i] = (new_gPCenter[1][i]-new_gPBack[1][i]);
				newvector[2][i] = (new_gPCenter[2][i]-new_gPBack[2][i]);

				new_gPFront[0][i] = new_gPCenter[0][i]+newratio_y*newvector[0][i]+ firststepy;
				new_gPFront[1][i] = new_gPBack[1][i]+newratio_x*newvector[1][i];
				new_gPFront[2][i] = new_gPCenter[2][i]+newratio_z*newvector[2][i];
			}
			else if(gKineAll.selSupport[i+1] == DoubleSupport) // 下一步是DSP
			{
				new_gPBack[0][i] = new_gPCenter[0][i]+gDirLLateral[0][i]*gLateralZMPShift- ratio_lenback*lenback*gDirLFoot[0][i]+ new_gPBackmoveY;
				if(new_gPCenter[1][i]>0)
				{new_gPBack[1][i] = -(new_gPCenter[1][i-1]- new_gPBackmoveX);}
				if(new_gPCenter[1][i]<0)
				{new_gPBack[1][i] = -(new_gPCenter[1][i-1]+ new_gPBackmoveX);}
				new_gPBack[2][i] = new_gPCenter[2][i]+gDirLLateral[2][i]*gLateralZMPShift-  ratio_lenback*lenback*gDirLFoot[2][i] ;

				newvector[0][i] = (new_gPCenter[0][i]-new_gPBack[0][i]);
				newvector[1][i] = (new_gPCenter[1][i]-new_gPBack[1][i]);
				newvector[2][i] = (new_gPCenter[2][i]-new_gPBack[2][i]);

				new_gPFront[0][i] = new_gPCenter[0][i];
				new_gPFront[1][i] = new_gPCenter[1][i]-finalstepx;
				if(finalstepx > new_gPCenter[1][i])
				{
					new_gPFront[1][i] = 0.0;
				}
				new_gPFront[2][i] = new_gPCenter[2][i]+newratio_z*newvector[2][i];
			}
			else // 普通行走中
			{	
				new_gPBack[0][i] = new_gPCenter[0][i]+gDirLLateral[0][i]*gLateralZMPShift-  ratio_lenback*lenback*gDirLFoot[0][i]+ new_gPBackmoveY;
				
				//new_gPBack[0][i] = new_gPCenter[0][i]+gDirLLateral[0][i]*gLateralZMPShift-  gSagittalZMPBack*lenback*gDirLFoot[0][i] + new_gPBackmoveY;
					
				if(new_gPCenter[1][i]>0)
				{new_gPBack[1][i] = new_gPCenter[1][i]+ new_gPBackmoveX;}
				if(new_gPCenter[1][i]<0)
				{new_gPBack[1][i] = new_gPCenter[1][i]- new_gPBackmoveX;}
					
				new_gPBack[2][i] = new_gPCenter[2][i]+gDirLLateral[2][i]*gLateralZMPShift- ratio_lenback*lenback*gDirLFoot[2][i] ;

				newvector[0][i] = (new_gPCenter[0][i]-new_gPBack[0][i]);
				newvector[1][i] = (new_gPCenter[1][i]-new_gPBack[1][i]);
				newvector[2][i] = (new_gPCenter[2][i]-new_gPBack[2][i]);
				
				new_gPFront[0][i] = new_gPCenter[0][i]+newratio_y*newvector[0][i];
				new_gPFront[1][i] = new_gPCenter[1][i]+newratio_x*newvector[1][i];
				new_gPFront[2][i] = new_gPCenter[2][i]+newratio_z*newvector[2][i];
			}
		}
		else if (gKineAll.selSupport[i] == DoubleSupport)
		{
			lenfront = 0; // zero, remain the same position
            lenback = 0; // zero, remain the same position
			new_gPFront[0][i] = new_gPCenter[0][i];
			new_gPFront[1][i] = new_gPCenter[1][i];
			new_gPFront[2][i] = new_gPCenter[2][i];

			new_gPBack[0][i] = new_gPCenter[0][i];
			new_gPBack[1][i] = new_gPCenter[1][i];
			new_gPBack[2][i] = new_gPCenter[2][i];	
		}
	}

	//把上面算出的new_gPFront new_gPCenter new_gPBack放在一個大矩陣new_gPAll中//
	int index_copy = 0;
	for (int i = 0 ; i < gNumOfStep ; i++)
	{
		new_gPAll[0][index_copy] = new_gPBack[0][i];
		new_gPAll[1][index_copy] = new_gPBack[1][i];
		new_gPAll[2][index_copy] = new_gPBack[2][i];
		index_copy++;

		new_gPAll[0][index_copy] = new_gPCenter[0][i];
		new_gPAll[1][index_copy] = new_gPCenter[1][i];
		new_gPAll[2][index_copy] = new_gPCenter[2][i];
		index_copy++;

		new_gPAll[0][index_copy] = new_gPFront[0][i];
		new_gPAll[1][index_copy] = new_gPFront[1][i];
		new_gPAll[2][index_copy] = new_gPFront[2][i];
		index_copy++;
	}


	//開始算ZMP連續軌跡//
	for (int j = 0 ; j < gNza ; j++)
	{
		gInpZMPy[j] = 0;
		gInpZMPx[j] = 0;
	}

	//處理旋轉角度//
	for (int i = 0 ; i < gNumOfStep ; i++)
	{
		index_copy = (i+1)*3;
		VectorDSP[0] = new_gPAll[0][index_copy]-new_gPAll[0][index_copy-1]; // 取出在DSP中所移動的ZMP距離
		VectorDSP[1] = new_gPAll[1][index_copy]-new_gPAll[1][index_copy-1]; // 取出在DSP中所移動的ZMP距離
		VectorDSP[2] = new_gPAll[2][index_copy]-new_gPAll[2][index_copy-1]; // 取出在DSP中所移動的ZMP距離
		DsitanceDSP = gNorm1Pd(VectorDSP);
		if (DsitanceDSP == 0) // zero vector
		{
			VectorDSP[0] = 0; VectorDSP[1] = 0; VectorDSP[2] = 0;
		}
		else
		{
			VectorDSP[0] = VectorDSP[0]/DsitanceDSP;
			VectorDSP[1] = VectorDSP[1]/DsitanceDSP;
			VectorDSP[2] = VectorDSP[2]/DsitanceDSP;
		}

		if (gKineAll.selSupport[i] == RightSupport) // right support
		{
			DirFB_Now[0] = gDirRFoot[0][i];
			DirFB_Now[1] = gDirRFoot[1][i];
			DirFB_Now[2] = gDirRFoot[2][i];
			DirLR_Now[0] = -gDirRLateral[0][i];
			DirLR_Now[1] = -gDirRLateral[1][i];
			DirLR_Now[2] = -gDirRLateral[2][i];
		}
		else if (gKineAll.selSupport[i] == LeftSupport) // left support  
		{
			DirFB_Now[0] = gDirLFoot[0][i];
			DirFB_Now[1] = gDirLFoot[1][i];
			DirFB_Now[2] = gDirLFoot[2][i];
			DirLR_Now[0] = -gDirLLateral[0][i];
			DirLR_Now[1] = -gDirLLateral[1][i];
			DirLR_Now[2] = -gDirLLateral[2][i];
		}      
		else // double support
		{
			DirFB_Now[0] = gDirRFoot[0][i];
			DirFB_Now[1] = gDirRFoot[1][i];
			DirFB_Now[2] = gDirRFoot[2][i];
			DirLR_Now[0] = -gDirRLateral[0][i];
			DirLR_Now[1] = -gDirRLateral[1][i];
			DirLR_Now[2] = -gDirRLateral[2][i];
		}


		// 求出前中與中後點 之間的距離(投影在腳的方向上的距離)
		//Y方向
		DistFBFront2P = (new_gPAll[0][index_copy-1]-new_gPAll[0][index_copy-2])*DirFB_Now[0]+(new_gPAll[1][index_copy-1]-new_gPAll[1][index_copy-2])*DirFB_Now[1]+(new_gPAll[2][index_copy-1]-new_gPAll[2][index_copy-2])*DirFB_Now[2];
		DistFBBack2P = (new_gPAll[0][index_copy-2]-new_gPAll[0][index_copy-3])*DirFB_Now[0]+(new_gPAll[1][index_copy-2]-new_gPAll[1][index_copy-3])*DirFB_Now[1]+(new_gPAll[2][index_copy-2]-new_gPAll[2][index_copy-3])*DirFB_Now[2];
		
		//X方向
		DistLRFront2P = (new_gPAll[0][index_copy-1]-new_gPAll[0][index_copy-2])*DirLR_Now[0]+(new_gPAll[1][index_copy-1]-new_gPAll[1][index_copy-2])*DirLR_Now[1]+(new_gPAll[2][index_copy-1]-new_gPAll[2][index_copy-2])*DirLR_Now[2];		
		DistLRBack2P = (new_gPAll[0][index_copy-2]-new_gPAll[0][index_copy-3])*DirLR_Now[0]+(new_gPAll[1][index_copy-2]-new_gPAll[1][index_copy-3])*DirLR_Now[1]+(new_gPAll[2][index_copy-2]-new_gPAll[2][index_copy-3])*DirLR_Now[2];
		if(DistLRFront2P < 0)
		{DistLRFront2P =-( (new_gPAll[0][index_copy-1]-new_gPAll[0][index_copy-2])*DirLR_Now[0]+(new_gPAll[1][index_copy-1]-new_gPAll[1][index_copy-2])*DirLR_Now[1]+(new_gPAll[2][index_copy-1]-new_gPAll[2][index_copy-2])*DirLR_Now[2]);}
		if(DistLRBack2P < 0)
		{DistLRBack2P =-( (new_gPAll[0][index_copy-2]-new_gPAll[0][index_copy-3])*DirLR_Now[0]+(new_gPAll[1][index_copy-2]-new_gPAll[1][index_copy-3])*DirLR_Now[1]+(new_gPAll[2][index_copy-2]-new_gPAll[2][index_copy-3])*DirLR_Now[2]);}

		VecFB_Now = DsitanceDSP*(DirFB_Now[0]*VectorDSP[0]+DirFB_Now[1]*VectorDSP[1]+DirFB_Now[2]*VectorDSP[2]);
		VecLR_Now = DsitanceDSP*(DirLR_Now[0]*VectorDSP[0]+DirLR_Now[1]*VectorDSP[1]+DirLR_Now[2]*VectorDSP[2]);


		//開始算連續軌跡//
		if (i == 0) // first step
		{
			VecFB_Last = LastDsitanceDSP*(DirFB_Now[0]*LastVectorDSP[0]+DirFB_Now[1]*LastVectorDSP[1]+DirFB_Now[2]*LastVectorDSP[2]);
			VecLR_Last = LastDsitanceDSP*(DirLR_Now[0]*LastVectorDSP[0]+DirLR_Now[1]*LastVectorDSP[1]+DirLR_Now[2]*LastVectorDSP[2]);    
        
			gKineAll.GenZMPFreeAssign(T_P*SSP*interpolationBC_Y_needtimeratio,0,0,0,0,0,0,0,0,gNab/2.0+1,varSSP1FB);
			gKineAll.GenZMPFreeAssign(T_P*SSP*interpolationBC_X_needtimeratio,0,0,0,0,0,0,0,0,gNab/2.0+1,varSSP1LR);
     
			gKineAll.GenZMPFreeAssign(T_P*SSP*interpolationCF_Y_needtimeratio,0,0,0,0,0,0,0,0,gNab/2.0+1,varSSP2FB);
			gKineAll.GenZMPFreeAssign(T_P*SSP*interpolationBC_X_needtimeratio,0,0,0,0,0,0,0,0,gNab/2.0+1,varSSP2LR);

           gKineAll.GenZMPFreeAssign(T_P*DSP,0,DsitanceDSP/(gDivideVel*interpolation_DSP_start_velocity),0,0,DsitanceDSP,DsitanceDSP /(gDivideVel*interpolation_DSP_start_velocity),0,0,gNza+gNzb+1,varDSP);
		}
		else 
		{
			if (gKineAll.selSupport[i] == DoubleSupport)
			{
				VecFB_Last = LastDsitanceDSP*(DirFB_Now[0]*LastVectorDSP[0]+DirFB_Now[1]*LastVectorDSP[1]+DirFB_Now[2]*LastVectorDSP[2]);
				VecLR_Last = LastDsitanceDSP*(DirLR_Now[0]*LastVectorDSP[0]+DirLR_Now[1]*LastVectorDSP[1]+DirLR_Now[2]*LastVectorDSP[2]);    

				gKineAll.GenZMPFreeAssign(T_P*SSP*interpolationBC_Y_needtimeratio,0,0,0,0,0,0,0,0,gNab/2.0+1,varSSP1FB);
				gKineAll.GenZMPFreeAssign(T_P*SSP*interpolationBC_X_needtimeratio,0,0,0,0,0,0,0,0,gNab/2.0+1,varSSP1LR);
     
				gKineAll.GenZMPFreeAssign(T_P*SSP*interpolationCF_Y_needtimeratio,0,0,0,0,0,0,0,0,gNab/2.0+1,varSSP2FB);
				gKineAll.GenZMPFreeAssign(T_P*SSP*interpolationCF_X_needtimeratio,0,0,0,0,0,0,0,0,gNab/2.0+1,varSSP2LR);

				gKineAll.GenZMPFreeAssign(T_P*DSP,0,DsitanceDSP/(gDivideVel*interpolation_DSP_start_velocity),0,0,DsitanceDSP,DsitanceDSP /(gDivideVel*interpolation_DSP_start_velocity),0,0,gNza+gNzb+1,varDSP);					
			}
			else
			{
				if (gKineAll.selSupport[i-1] == DoubleSupport)
				{
					VecFB_Last = LastDsitanceDSP*(DirFB_Now[0]*LastVectorDSP[0]+DirFB_Now[1]*LastVectorDSP[1]+DirFB_Now[2]*LastVectorDSP[2]);
					VecLR_Last = LastDsitanceDSP*(DirLR_Now[0]*LastVectorDSP[0]+DirLR_Now[1]*LastVectorDSP[1]+DirLR_Now[2]*LastVectorDSP[2]);

					gKineAll.GenZMPFreeAssign(T_P*SSP*interpolationBC_Y_needtimeratio,0,VecFB_Last/(gDivideVel*interpolationBC_Y_gDivideVel),0,0,DistFBBack2P,0,0,0,gNab/2.0+1,varSSP1FB);
					gKineAll.GenZMPFreeAssign(T_P*SSP*interpolationBC_X_needtimeratio,0,VecLR_Last/(gDivideVel*interpolationBC_X_gDivideVel),0,0,DistLRBack2P,0,0,0,gNab/2.0+1,varSSP1LR);

					gKineAll.GenZMPFreeAssign(T_P*SSP*interpolationCF_Y_needtimeratio,0,0,0,0,DistFBFront2P,-VecFB_Now/(gDivideVel*interpolationCF_Y_finalvelocityratio),0,0,gNab/2.0+1,varSSP2FB);
					gKineAll.GenZMPFreeAssign(T_P*SSP*interpolationCF_X_needtimeratio,0,0,0,0,-DistLRFront2P,VecLR_Now/(gDivideVel*interpolationCF_X_finalvelocityratio),0,0,gNab/2.0+1,varSSP2LR);

					gKineAll.GenZMPFreeAssign(T_P*DSP,0,DsitanceDSP/(gDivideVel*interpolation_DSP_start_velocity),0,0,DsitanceDSP,DsitanceDSP /(gDivideVel*interpolation_DSP_start_velocity),0,0,gNza+gNzb+1,varDSP);
					
				}
				else if (gKineAll.selSupport[i+1] == DoubleSupport)
				{
					VecFB_Last = LastDsitanceDSP*(DirFB_Now[0]*LastVectorDSP[0]+DirFB_Now[1]*LastVectorDSP[1]+DirFB_Now[2]*LastVectorDSP[2]);
					VecLR_Last = LastDsitanceDSP*(DirLR_Now[0]*LastVectorDSP[0]+DirLR_Now[1]*LastVectorDSP[1]+DirLR_Now[2]*LastVectorDSP[2]);    

					gKineAll.GenZMPFreeAssign(T_P*SSP*interpolationBC_Y_needtimeratio,0,VecFB_Last/(gDivideVel*interpolationBC_Y_gDivideVel),0,0,DistFBBack2P,0,0,0,gNab/2.0+1,varSSP1FB);
					gKineAll.GenZMPFreeAssign(T_P*SSP*interpolationBC_X_needtimeratio,0,VecLR_Last/(gDivideVel*interpolationBC_X_gDivideVel),0,0,-DistLRBack2P,0,0,0,gNab/2.0+1,varSSP1LR);

										
					gKineAll.GenZMPFreeAssign(T_P*SSP*interpolationCF_Y_needtimeratio,0,0,0,0,DistFBFront2P,-VecFB_Now/(gDivideVel*interpolationCF_Y_finalvelocityratio),0,0,gNab/2.0+1,varSSP2FB);
					gKineAll.GenZMPFreeAssign(T_P*SSP*interpolationCF_X_needtimeratio,0,0,0,0,-DistLRFront2P,VecLR_Now/(gDivideVel*interpolationCF_X_finalvelocityratio),0,0,gNab/2.0+1,varSSP2LR);

					gKineAll.GenZMPFreeAssign(T_P*DSP,0,DsitanceDSP/(gDivideVel*interpolation_DSP_start_velocity),0,0,DsitanceDSP,DsitanceDSP /(gDivideVel*interpolation_DSP_start_velocity),0,0,gNza+gNzb+1,varDSP);
				}
				else
				{
					VecFB_Last = LastDsitanceDSP*(DirFB_Now[0]*LastVectorDSP[0]+DirFB_Now[1]*LastVectorDSP[1]+DirFB_Now[2]*LastVectorDSP[2]);
					VecLR_Last = LastDsitanceDSP*(DirLR_Now[0]*LastVectorDSP[0]+DirLR_Now[1]*LastVectorDSP[1]+DirLR_Now[2]*LastVectorDSP[2]);    

					gKineAll.GenZMPFreeAssign(T_P*SSP*interpolationBC_Y_needtimeratio,0,VecFB_Last/(gDivideVel*interpolationBC_Y_gDivideVel),0,0,DistFBBack2P,DistFBBack2P/(T_P*SSP*interpolationBC_Y_finalvelocityratio),0,0,gNab/2.0+1,varSSP1FB);
					gKineAll.GenZMPFreeAssign(T_P*SSP*interpolationBC_X_needtimeratio,0,VecLR_Last/(gDivideVel*interpolationBC_X_gDivideVel),0,0,-DistLRBack2P,DistLRBack2P/(T_P*SSP*interpolationBC_X_finalvelocityratio),0,0,gNab/2.0+1,varSSP1LR);

					gKineAll.GenZMPFreeAssign(T_P*SSP*interpolationCF_Y_needtimeratio,0,DistFBBack2P/(T_P*SSP*interpolationBC_Y_finalvelocityratio),0,0,DistFBFront2P,VecFB_Now/(gDivideVel*interpolationCF_Y_finalvelocityratio),0,0,gNab/2.0+1,varSSP2FB); //CF的出速度要和BC的末速度一樣
					gKineAll.GenZMPFreeAssign(T_P*SSP*interpolationCF_X_needtimeratio,0,DistLRBack2P/(T_P*SSP*interpolationBC_X_finalvelocityratio),0,0,-DistLRFront2P,VecLR_Now/(gDivideVel*interpolationCF_X_finalvelocityratio),0,0,gNab/2.0+1,varSSP2LR);//CF的出速度要和BC的末速度一樣

					gKineAll.GenZMPFreeAssign(T_P*DSP,0,DsitanceDSP/(gDivideVel*interpolation_DSP_start_velocity),0,0,DsitanceDSP,DsitanceDSP /(gDivideVel*interpolation_DSP_start_velocity),0,0,gNza+gNzb+1,varDSP); //初速和末速相等
				}
			}
		}
		index_copy = 1; // 捨棄第一筆 因為重複，又由於生的時候有多生一筆 所以有剛剛好數量的軌跡點 SSP的開始到一半
		for (int j = i*gStepSample+gNza ; j < i*gStepSample+gNza+gNab/2.0 ; j++)
		{
			gInpZMPy[j] = new_gPBack[0][i]+varSSP1FB[index_copy]*DirFB_Now[0]+varSSP1LR[index_copy]*DirLR_Now[0];
			gInpZMPx[j] = new_gPBack[1][i]+varSSP1FB[index_copy]*DirFB_Now[1]+varSSP1LR[index_copy]*DirLR_Now[1];
			index_copy++;
		}
		index_copy = 1; // 捨棄第一筆 因為重複，又由於生的時候有多生一筆 所以有剛剛好數量的軌跡點 SSP的一半到結束
		for (int j = i*gStepSample+gNza+gNab/2 ; j < i*gStepSample+gNza+gNab ; j++)
		{
			gInpZMPy[j] = new_gPCenter[0][i]+varSSP2FB[index_copy]*DirFB_Now[0]+varSSP2LR[index_copy]*DirLR_Now[0];
			gInpZMPx[j] = new_gPCenter[1][i]+varSSP2FB[index_copy]*DirFB_Now[1]+varSSP2LR[index_copy]*DirLR_Now[1];
			index_copy++;
		}
		index_copy = 1; // 捨棄第一筆 因為重複，又由於生的時候有多生一筆 所以有剛剛好數量的軌跡點 後段DSP到下一步前段DSP
		for (int j = i*gStepSample+gNza+gNab ; j < (i+1)*gStepSample+gNza ; j++)
		{
			gInpZMPy[j] = new_gPFront[0][i]+varDSP[index_copy]*VectorDSP[0]; //+ varDSPLR[index_copy]*VectorDSP[0];
			gInpZMPx[j] = new_gPFront[1][i]+varDSP[index_copy]*VectorDSP[1]; //+ varDSPLR[index_copy]*VectorDSP[1];
			index_copy++;
		}

		LastDsitanceDSP = DsitanceDSP;
        LastVectorDSP[0] = VectorDSP[0];
        LastVectorDSP[1] = VectorDSP[1];
        LastVectorDSP[2] = VectorDSP[2];
	}
}



void gStepWhileWalk(void)	//邊走邊跨軌跡doratom//
{
	gNumOfStep = 11; // 包含初始、轉換與preivew的總步數
	gCOGDown = 28; // 愈少膝蓋愈直 比較像人 也比較省力

	gKineAll.FlagSumoMode = 0; //// 是否機器人要長時間單腳站立

	double x_val_zmp = 80; // ZMP 左右方向位置
	bool PNx = 0;
	double y_step_zmp =  250;
	double TurnRadius = 300;


	double distance2L = 160.0;

	gKineAll.StepHeight[0] = 0;
	for (int i = 1 ; i < gNumOfStep+30 ; i++)
		gKineAll.StepHeight[i] = 20;

    gKineAll.StepHeight[3] = 110;
	//gKineAll.StepHeight[4] = 20;

	gKineAll.selSupport[0] = 2;
	for (int i = 1 ; i < gNumOfStep+30 ; i+=2)
	{
		gKineAll.selSupport[i] = 1;
		gKineAll.selSupport[i+1] = 0;
	}
	for (int i = gNumOfStep-4 ; i < 4000 ; i++)
		gKineAll.selSupport[i] = 2; // double support and prepare to stop

 /////////////////////要轉的角度//////////////////////////////
	double AngChange = 0.0;

	for (int i = 0 ; i < gNumOfStep+30 ; i++)
	{
		gLRotAngZ[i*2] = AngChange*i + gLAngZWorld;
		gLRotAngZ[i*2+1] = AngChange*i + gLAngZWorld;
	}

	gRRotAngZ[0] = gRAngZWorld;
	for (int i = 0 ; i < gNumOfStep+30 ; i++)
	{
		gRRotAngZ[i*2+1] = AngChange*i + gRAngZWorld;
		gRRotAngZ[i*2+2] = AngChange*i + gRAngZWorld;
	}

	gLLInitZMPFB = -80*sin(gRRotAngZ[0]);
	gRLInitZMPFB = 80*sin(gRRotAngZ[0]);
	gLLInitZMPLR = 80*cos(gRRotAngZ[0]);
	gRLInitZMPLR = -80*cos(gRRotAngZ[0]);

///////////////////////要轉的角度///////////////////////////////

	double BodyDir = (gLAngZWorld+gRAngZWorld)/2.0;
	double StrideX = y_step_zmp*sin(BodyDir);
	double StrideY = y_step_zmp*cos(BodyDir);


	// 左右 
	gFstpX[0] = 0;
	gFstpX[1] = -x_val_zmp*cos(gRRotAngZ[1]);
	gFstpX[2] = x_val_zmp*cos(gRRotAngZ[0])+StrideX/2.0;
	for (int i = 2 ; i < gNumOfStep ; i++)
	{
		gFstpX[i*2-1] = gFstpX[(i-1)*2-1] + StrideX;
		gFstpX[i*2] = gFstpX[(i-1)*2] + StrideX;
	}
	


	//////////////////////////////////用來處理我如果要右轉的話 右腳要先出去 左轉地的時候 左腳要先出去///////////
	if (gKineAll.selSupport[gNumOfStep-4] == 0) // right support
	{
		gFstpX[gNumOfStep-4] = gFstpX[gNumOfStep-5] + 160*cos((gLRotAngZ[gNumOfStep-4]+gRRotAngZ[gNumOfStep-4])/2.0);
	}
	else
	{
		gFstpX[gNumOfStep-4] = gFstpX[gNumOfStep-5] - 160*cos((gLRotAngZ[gNumOfStep-4]+gRRotAngZ[gNumOfStep-4])/2.0);
	}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
	gFstpX[gNumOfStep-4] = (gFstpX[gNumOfStep-4]+gFstpX[gNumOfStep-5])/2.0;  //最後一步要走半部(和第一步一樣)

	for (int i = gNumOfStep - 3 ; i < gNumOfStep+4 ; i++)
	{
		gFstpX[i] = gFstpX[gNumOfStep-4];
	}

	
	    
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//前後
	gFstpY[0] = 0;
	gFstpY[1] = distance2L/2.0*sin(gRRotAngZ[2]);
	gFstpY[2] = -distance2L/2.0*sin(gLRotAngZ[0])+StrideY/2.0;
	for (int i = 1 ; i < gNumOfStep ; i++)
	{
		gFstpY[i*2+1] = gFstpY[i*2-1]+StrideY;
		gFstpY[i*2+2] = gFstpY[i*2]+StrideY;
	}
	gFstpY[4] = gFstpY[2]+StrideY+100.0; //為什麼是由第3部加呢? 因為第4部是跨左腳 第3部才是右腳
   // gFstpY[5] = gFstpY[3]+StrideY;
	for(int i = 2;i<gNumOfStep;i++)
	{
		gFstpY[i*2+1] = gFstpY[i*2-1]+StrideY+50.0;
		
	}
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////用來處理我如果要右轉的話 右腳要先出去 左轉地的時候 左腳要先出去///////////
	if (gKineAll.selSupport[gNumOfStep-4] == 0) // right support
	{
		gFstpY[gNumOfStep-4] = gFstpY[gNumOfStep-5] - 160*sin((gLRotAngZ[gNumOfStep-4]+gRRotAngZ[gNumOfStep-4])/2.0);
	}
	else
	{
		gFstpY[gNumOfStep-4] = gFstpY[gNumOfStep-5] + 160*sin((gLRotAngZ[gNumOfStep-4]+gRRotAngZ[gNumOfStep-4])/2.0);
	}
///////////////////////////////////////////////////////////////////////////////////////////////////////
	gFstpY[gNumOfStep-4] = (gFstpY[gNumOfStep-4]+gFstpY[gNumOfStep-5])/2.0; //最後一步要走半步(和第一步一樣)

	for (int i = gNumOfStep - 3 ; i < gNumOfStep+5 ; i++)
	{
		gFstpY[i] = gFstpY[gNumOfStep-4];
	}
	
	for (int i = 0 ; i < gNumOfStep+25 ; i++)
		gGroundHeight[i] = 0.0;
	

} 


void gInitWalkOneStep(void)
{
	/******************************************************************
	input: StepInput 總步數 因為preview以及去頭去尾 所以要減六 可以得到總跨步數, StepLength 步距，可正可負
	output: void

	Note:

	// 初始化機器人行為為直走
	// 腳步設定 地形設定 機器人步行參數 設定於此
	// 相似的函數有 gInitTurnLeft gInitWalkStraight gInitStepHere gInitStair gInitSquat
	//跨一步doratom//
	******************************************************************/
	gNumOfStep = 9; // 包含初始、轉換與preivew的總步數
	gCOGDown = 15; // 愈少膝蓋愈直 比較像人 也比較省力

	gKineAll.FlagSumoMode = 0;

	double x_val_zmp = 80; // ZMP 左右方向位置
	bool PNx = 0;
	double y_step_zmp =  270;
	double TurnRadius = 300;


	double distance2L = 160.0;

	 checkonestep = 1;  //用來判斷在gCalculateZMP()中的gPCenter[1][3] 要等於0 (這樣才zmp才不會在只跨一步的時候 會多左右擺動一次)

	gKineAll.StepHeight[0] = 0;
	for (int i = 1 ; i < gNumOfStep+30 ; i++)
		gKineAll.StepHeight[i] = 50;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	gKineAll.selSupport[0] = 2;
	for (int i = 1 ; i < gNumOfStep+30 ; i+=2)
	{
		gKineAll.selSupport[i] = 1;
		gKineAll.selSupport[i+1] = 0;
	}
	//gKineAll.selSupport[3] = 0;
	for (int i = gNumOfStep-4-2 ; i < 4000 ; i++)
		gKineAll.selSupport[i] = 2; // double support and prepare to stop
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	double AngChange = 0.0;

	for (int i = 0 ; i < gNumOfStep+30 ; i++)
	{
		gLRotAngZ[i*2] = AngChange*i + gLAngZWorld;
		gLRotAngZ[i*2+1] = AngChange*i + gLAngZWorld;
	}

	gRRotAngZ[0] = gRAngZWorld;
	for (int i = 0 ; i < gNumOfStep+30 ; i++)
	{
		gRRotAngZ[i*2+1] = AngChange*i + gRAngZWorld;
		gRRotAngZ[i*2+2] = AngChange*i + gRAngZWorld;
	}

	gLLInitZMPFB = -80*sin(gRRotAngZ[0]); // ZMP Initial 
	gRLInitZMPFB = 80*sin(gRRotAngZ[0]);
	gLLInitZMPLR = 80*cos(gRRotAngZ[0]); // ZMP Initial Swing 軌跡會用到 換腳要改
	gRLInitZMPLR = -80*cos(gRRotAngZ[0]);

	double BodyDir = (gLAngZWorld+gRAngZWorld)/2.0;
	double StrideX = y_step_zmp*sin(BodyDir);  //BodyDir = 0;
	double StrideY = y_step_zmp*cos(BodyDir);


	// 左右 
	gFstpX[0] = 0;
	gFstpX[1] = -x_val_zmp*cos(gRRotAngZ[1]); // ZMP Initial Swing 軌跡會用到 換腳要改
	gFstpX[2] = x_val_zmp*cos(gRRotAngZ[0])+StrideX/2.0; // ZMP Initial Swing 軌跡會用到 換腳要改
	
	for (int i = 2 ; i < gNumOfStep ; i++)
	{
		gFstpX[i*2-1] = gFstpX[(i-1)*2-1] + StrideX;
		gFstpX[i*2] = gFstpX[(i-1)*2] + StrideX;
	}

	if (gKineAll.selSupport[gNumOfStep-4-2] == 0) // right support
	{
		gFstpX[gNumOfStep-4-2] = gFstpX[gNumOfStep-5-2] - 160*cos((gLRotAngZ[gNumOfStep-4]+gRRotAngZ[gNumOfStep-4])/2.0);
	}
	else
	{
		
		gFstpX[gNumOfStep-4-2] = gFstpX[gNumOfStep-5-2]- 160*cos((gLRotAngZ[gNumOfStep-4]+gRRotAngZ[gNumOfStep-4])/2.0);
		
	}
	
	gFstpX[gNumOfStep-4-1] = (gFstpX[gNumOfStep-4-2]+gFstpX[gNumOfStep-5-2])/2.0;
	
	for (int i = gNumOfStep - 3 -2; i < gNumOfStep+8 ; i++)
	{
		gFstpX[i] = gFstpX[gNumOfStep-4-1];
	}

	

	gFstpY[0] = 0;
	gFstpY[1] = distance2L/2.0*sin(gRRotAngZ[2]);
	gFstpY[2] = -distance2L/2.0*sin(gLRotAngZ[0])+StrideY/2.0;
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////
	for (int i = 1 ; i < gNumOfStep ; i++)
	{
		gFstpY[i*2+1] = gFstpY[i*2-1]+StrideY;
		gFstpY[i*2+2] = gFstpY[i*2]+StrideY;
	}
	/////////////////////////////////////////////////////////////////////////////////////////////////////////

	if (gKineAll.selSupport[gNumOfStep-4-2] == 0) // right support
	{
		gFstpY[gNumOfStep-4-2] = gFstpY[gNumOfStep-5-2] + 160*sin((gLRotAngZ[gNumOfStep-4]+gRRotAngZ[gNumOfStep-4])/2.0);
	}
	else
	{
		gFstpY[gNumOfStep-4-2] = gFstpY[gNumOfStep-5-2]+ 160*sin((gLRotAngZ[gNumOfStep-4]+gRRotAngZ[gNumOfStep-4])/2.0);
	}

	gFstpY[gNumOfStep-4-2] = (gFstpY[gNumOfStep-4-2]+gFstpY[gNumOfStep-5-2])/2.0;

	for (int i = gNumOfStep - 3-2 ; i < gNumOfStep+5 ; i++)
	{

		gFstpY[i] = gFstpY[gNumOfStep-4-2];
	}

	for (int i = 0 ; i < gNumOfStep+25 ; i++)
		gGroundHeight[i] = 0.0;
}

//只有一腳踩上障礙物doratom//
void gInitWalkOneStepHigh(void)
{
	/******************************************************************
	input: StepInput 總步數 因為preview以及去頭去尾 所以要減六 可以得到總跨步數, StepLength 步距，可正可負
	output: void

	Note:

	// 初始化機器人行為為直走
	// 腳步設定 地形設定 機器人步行參數 設定於此
	// 相似的函數有 gInitTurnLeft gInitWalkStraight gInitStepHere gInitStair gInitSquat

	******************************************************************/
	gNumOfStep = 11; // 包含初始、轉換與preivew的總步數
	gCOGDown = 30; // 愈少膝蓋愈直 比較像人 也比較省力

	
	//20120925//
	checkWalkOneStepHigh = 1;
	//20120925//


	gKineAll.FlagSumoMode = 0;

	double x_val_zmp = 80; // ZMP 左右方向位置
	bool PNx = 0;
	double y_step_zmp =  200;
	double TurnRadius = 300;


	double distance2L = 160.0;

	gKineAll.StepHeight[0] = 0;
	for (int i = 1 ; i < gNumOfStep+30 ; i++)
		gKineAll.StepHeight[i] = 20;

	gKineAll.selSupport[0] = 2;
	for (int i = 1 ; i < gNumOfStep+30 ; i+=2)
	{
		gKineAll.selSupport[i] = 1;
		gKineAll.selSupport[i+1] = 0;
	}
	for (int i = gNumOfStep-4 ; i < 4000 ; i++)
		gKineAll.selSupport[i] = 2; // double support and prepare to stop


	double AngChange = 0.0;

	for (int i = 0 ; i < gNumOfStep+30 ; i++)
	{
		gLRotAngZ[i*2] = AngChange*i + gLAngZWorld;
		gLRotAngZ[i*2+1] = AngChange*i + gLAngZWorld;
	}

	gRRotAngZ[0] = gRAngZWorld;
	for (int i = 0 ; i < gNumOfStep+30 ; i++)
	{
		gRRotAngZ[i*2+1] = AngChange*i + gRAngZWorld;
		gRRotAngZ[i*2+2] = AngChange*i + gRAngZWorld;
	}

	gLLInitZMPFB = -80*sin(gRRotAngZ[0]); // ZMP Initial 
	gRLInitZMPFB = 80*sin(gRRotAngZ[0]);
	gLLInitZMPLR = 80*cos(gRRotAngZ[0]); // ZMP Initial Swing 軌跡會用到 換腳要改
	gRLInitZMPLR = -80*cos(gRRotAngZ[0]);

	double BodyDir = (gLAngZWorld+gRAngZWorld)/2.0;
	double StrideX = y_step_zmp*sin(BodyDir);
	double StrideY = y_step_zmp*cos(BodyDir);


	// 左右 
	gFstpX[0] = 0;
	gFstpX[1] = -x_val_zmp*cos(gRRotAngZ[1]); // ZMP Initial Swing 軌跡會用到 換腳要改
	gFstpX[2] = x_val_zmp*cos(gRRotAngZ[0])+StrideX/2.0; // ZMP Initial Swing 軌跡會用到 換腳要改
	for (int i = 2 ; i < gNumOfStep ; i++)
	{
		gFstpX[i*2-1] = gFstpX[(i-1)*2-1] + StrideX;
		gFstpX[i*2] = gFstpX[(i-1)*2] + StrideX;
	}

	if (gKineAll.selSupport[gNumOfStep-4] == 0) // right support
	{
		gFstpX[gNumOfStep-4] = gFstpX[gNumOfStep-5] + 160*cos((gLRotAngZ[gNumOfStep-4]+gRRotAngZ[gNumOfStep-4])/2.0);
	}
	else
	{
		gFstpX[gNumOfStep-4] = gFstpX[gNumOfStep-5] - 160*cos((gLRotAngZ[gNumOfStep-4]+gRRotAngZ[gNumOfStep-4])/2.0);
	}

	gFstpX[gNumOfStep-4] = (gFstpX[gNumOfStep-4]+gFstpX[gNumOfStep-5])/2.0;

	for (int i = gNumOfStep - 3 ; i < gNumOfStep+4 ; i++)
	{
		gFstpX[i] = gFstpX[gNumOfStep-4];
	}

	gFstpY[0] = 0;
	gFstpY[1] = distance2L/2.0*sin(gRRotAngZ[2]);
	gFstpY[2] = -distance2L/2.0*sin(gLRotAngZ[0])+StrideY/2.0;
	for (int i = 1 ; i < gNumOfStep ; i++)
	{
		gFstpY[i*2+1] = gFstpY[i*2-1]+StrideY;
		gFstpY[i*2+2] = gFstpY[i*2]+StrideY;
	}

	//20120925//
	this_step =1; //為了要調整newzmp軌跡中的參數
	gFstpY[4] = gFstpY[2]+StrideY;
	gFstpY[5] = gFstpY[4]+100.0;//gFstpY[3]+StrideY-180;
	this_step =0;//調整完之後就恢復成原本newzmp軌跡中的參數
	for(int i = 2;i<gNumOfStep;i++)
	{
		gFstpY[i*2+2] = gFstpY[i*2]+StrideY;
		gFstpY[i*2+3] = gFstpY[i*2+1]+StrideY;
		
		
	}
	
	//20120925//


	if (gKineAll.selSupport[gNumOfStep-4] == 0) // right support
	{
		gFstpY[gNumOfStep-4] = gFstpY[gNumOfStep-5] - 160*sin((gLRotAngZ[gNumOfStep-4]+gRRotAngZ[gNumOfStep-4])/2.0);
	}
	else
	{
		gFstpY[gNumOfStep-4] = gFstpY[gNumOfStep-5] + 160*sin((gLRotAngZ[gNumOfStep-4]+gRRotAngZ[gNumOfStep-4])/2.0);
	}

	gFstpY[gNumOfStep-4] = (gFstpY[gNumOfStep-4]+gFstpY[gNumOfStep-5])/2.0;

	for (int i = gNumOfStep - 3 ; i < gNumOfStep+5 ; i++)
	{
		gFstpY[i] = gFstpY[gNumOfStep-4];
	}

	for (int i = 0 ; i < gNumOfStep+25 ; i++)
	{gGroundHeight[i] = 0.0;}


	gGroundHeight[4] = 40.0;

}


void gWalkSlopeDora(void)
{
	/******************************************************************
	input: StepInput 總步數 因為preview以及去頭去尾 所以要減六 可以得到總跨步數, StepLength 步距，可正可負
	output: void

	Note:
	// 初始化機器人行為為直走
	// 腳步設定 地形設定 機器人步行參數 設定於此
	// 相似的函數有 gInitTurnLeft gInitWalkStraight gInitStepHere gInitStair gInitSquat
	// 走斜坡 在地上先2步才踏上斜坡 不是第一步就踏上斜坡
	// 20121214 doratom
	******************************************************************/
	gNumOfStep = 9; // 包含初始、轉換與preivew的總步數
	gCOGDown = 30; // 愈少膝蓋愈直 比較像人 也比較省力

	slopeangle = -5.15*3.1415926/180; //斜坡角度小心!!!要記得加上負號
	check_slopeangle = 1;
	rotate_pitch_time_ratio = 0.94;

	gKineAll.FlagSumoMode = 0;

	double x_val_zmp = 80; // ZMP 左右方向位置
	bool PNx = 0;
	double y_step_zmp =  200;
	double TurnRadius = 300;
	double distance2L = 160.0;

	gKineAll.StepHeight[0] = 0;
	
	for (int i = 1 ; i < gNumOfStep+30 ; i++)
		gKineAll.StepHeight[i] = 11.5;

	gKineAll.selSupport[0] = 2;
	for (int i = 1 ; i < gNumOfStep+30 ; i+=2)
	{
		gKineAll.selSupport[i] = 1;
		gKineAll.selSupport[i+1] = 0;
	}
	for (int i = gNumOfStep-4 ; i < 4000 ; i++)
		gKineAll.selSupport[i] = 2; // double support and prepare to stop

	double AngChange = 0.0;

	for (int i = 0 ; i < gNumOfStep+30 ; i++)
	{
		gLRotAngZ[i*2] = AngChange*i + gLAngZWorld;
		gLRotAngZ[i*2+1] = AngChange*i + gLAngZWorld;
	}

	gRRotAngZ[0] = gRAngZWorld;
	for (int i = 0 ; i < gNumOfStep+30 ; i++)
	{
		gRRotAngZ[i*2+1] = AngChange*i + gRAngZWorld;
		gRRotAngZ[i*2+2] = AngChange*i + gRAngZWorld;
	}

	gLLInitZMPFB = -80*sin(gRRotAngZ[0]); // ZMP Initial 
	gRLInitZMPFB = 80*sin(gRRotAngZ[0]);
	gLLInitZMPLR = 80*cos(gRRotAngZ[0]); // ZMP Initial Swing 軌跡會用到 換腳要改
	gRLInitZMPLR = -80*cos(gRRotAngZ[0]);

	double BodyDir = (gLAngZWorld+gRAngZWorld)/2.0;
	double StrideX = y_step_zmp*sin(BodyDir);
	double StrideY = y_step_zmp*cos(BodyDir);

	//R
     gRRotAngPitch[0] = 0.0;
	 gRRotAngPitch[1] = 0.0;
	 gRRotAngPitch[2] = 0.0;

	 for(int i = 3;i< gNumOfStep-4;i++)
      gRRotAngPitch[i] = slopeangle;

	 for(int i = gNumOfStep-4;i< gNumOfStep+30;i++)
		 gRRotAngPitch[i] = 0.0;

	 //L
	 gLRotAngPitch[0] = 0.0;
	 gLRotAngPitch[1] = 0.0;
	 gLRotAngPitch[2] = slopeangle;
	 gLRotAngPitch[3] = slopeangle;
	 
	 for(int i = 4;i< gNumOfStep-5;i++)
      gLRotAngPitch[i] = slopeangle;

	 for(int i = gNumOfStep-5;i< gNumOfStep+30;i++)
		 gLRotAngPitch[i] = 0.0;


	// 左右 
	gFstpX[0] = 0;
	gFstpX[1] = -x_val_zmp*cos(gRRotAngZ[1]); // ZMP Initial Swing 軌跡會用到 換腳要改
	gFstpX[2] = x_val_zmp*cos(gRRotAngZ[0])+StrideX/2.0; // ZMP Initial Swing 軌跡會用到 換腳要改
	for (int i = 2 ; i < gNumOfStep ; i++)
	{
		gFstpX[i*2-1] = gFstpX[(i-1)*2-1] + StrideX;
		gFstpX[i*2] = gFstpX[(i-1)*2] + StrideX;
	}

	if (gKineAll.selSupport[gNumOfStep-4] == 0) // right support
	{
		gFstpX[gNumOfStep-4] = gFstpX[gNumOfStep-5] + 160*cos((gLRotAngZ[gNumOfStep-4]+gRRotAngZ[gNumOfStep-4])/2.0);
	}
	else
	{
		gFstpX[gNumOfStep-4] = gFstpX[gNumOfStep-5] - 160*cos((gLRotAngZ[gNumOfStep-4]+gRRotAngZ[gNumOfStep-4])/2.0);
	}

	gFstpX[gNumOfStep-4] = (gFstpX[gNumOfStep-4]+gFstpX[gNumOfStep-5])/2.0;

	for (int i = gNumOfStep - 3 ; i < gNumOfStep+4 ; i++)
	{
		gFstpX[i] = gFstpX[gNumOfStep-4];
	}

	//gFstpY[0] = 0;
	//gFstpY[1] = distance2L/2.0*sin(gRRotAngZ[2]);
	//gFstpY[2] = -distance2L/2.0*sin(gLRotAngZ[0])+StrideY/2.0;
	//for (int i = 1 ; i < gNumOfStep ; i++)
	//{
	//	gFstpY[i*2+1] = gFstpY[i*2-1]+StrideY;
	//	gFstpY[i*2+2] = gFstpY[i*2]+StrideY;
	//}

	//if (gKineAll.selSupport[gNumOfStep-4] == 0) // right support
	//{
	//	gFstpY[gNumOfStep-4] = gFstpY[gNumOfStep-5] - 160*sin((gLRotAngZ[gNumOfStep-4]+gRRotAngZ[gNumOfStep-4])/2.0);
	//}
	//else
	//{
	//	gFstpY[gNumOfStep-4] = gFstpY[gNumOfStep-5] + 160*sin((gLRotAngZ[gNumOfStep-4]+gRRotAngZ[gNumOfStep-4])/2.0);
	//}

	//gFstpY[gNumOfStep-4] = (gFstpY[gNumOfStep-4]+gFstpY[gNumOfStep-5])/2.0;
	
	//20121217 處理完第一步和最後一步
	gFstpY[0] = 0;
	gFstpY[1] = 0;
	gFstpY[2] = 210;
	gFstpY[3] = 300;
	gFstpY[4] = 400;
	gFstpY[5] = 500;
	gFstpY[6] = 500;
	gFstpY[7] = 500;
	
	
	//gFstpY[6] = 495+210;
	//gFstpY[7] = 495+210;


	for (int i = gNumOfStep - 3 ; i < gNumOfStep+5 ; i++)
	{
		gFstpY[i] = gFstpY[gNumOfStep-4];
	}

	//以上都跟直走一模一樣//
	  
	//設定要踏高的高度//
	gGroundHeight[0] = 0.0;
	gGroundHeight[1] = 0.0;
	gGroundHeight[2] = (gFstpY[2]-114)*tan(-slopeangle);
	gGroundHeight[3] = (gFstpY[3]-114)*tan(-slopeangle);
	gGroundHeight[4] = (gFstpY[4]-114)*tan(-slopeangle);
	gGroundHeight[5] = (gFstpY[5]-114)*tan(-slopeangle);
	gGroundHeight[6] = (gFstpY[5])*tan(-slopeangle);
	
	/*for (int i = 3 ; i < gNumOfStep-4 ; i++)
		gGroundHeight[i] = gGroundHeight[i-1]+ (StrideY/2)*tan(-slopeangle);*/

	for (int i = gNumOfStep-4 ; i < gNumOfStep+25 ; i++)
		gGroundHeight[i] = gGroundHeight[i-1];

}


void gWalkSlopeDora1(void)
{
	/******************************************************************
	input: StepInput 總步數 因為preview以及去頭去尾 所以要減六 可以得到總跨步數, StepLength 步距，可正可負
	output: void

	Note:
	// 初始化機器人行為為直走
	// 腳步設定 地形設定 機器人步行參數 設定於此
	// 相似的函數有 gInitTurnLeft gInitWalkStraight gInitStepHere gInitStair gInitSquat
	// 20121214 doratom
	******************************************************************/
	gNumOfStep = 9; // 包含初始、轉換與preivew的總步數
	gCOGDown = 50; // 愈少膝蓋愈直 比較像人 也比較省力

	slopeangle = -6.5*3.1415926/180; //斜坡角度小心!!!要記得加上負號
	check_slopeangle = 1;
	rotate_pitch_time_ratio = 0.94;

	gKineAll.FlagSumoMode = 0;

	double x_val_zmp = 80; // ZMP 左右方向位置
	bool PNx = 0;
	double y_step_zmp =  200;
	double TurnRadius = 300;
	double distance2L = 160.0;
	double AngChange = 0.0;	// 直走

	gKineAll.StepHeight[0] = 0;
	
	for (int i = 1 ; i < gNumOfStep+30 ; i++)
		gKineAll.StepHeight[i] = 30;

	gKineAll.selSupport[0] = 2;
	for (int i = 1 ; i < gNumOfStep+30 ; i+=2)
	{
		gKineAll.selSupport[i] = 1;
		gKineAll.selSupport[i+1] = 0;
	}
	for (int i = gNumOfStep-4 ; i < 4000 ; i++)
		gKineAll.selSupport[i] = 2; // double support and prepare to stop

	for (int i = 0 ; i < gNumOfStep+30 ; i++)
	{
		gLRotAngZ[i*2] = AngChange*i + gLAngZWorld;
		gLRotAngZ[i*2+1] = AngChange*i + gLAngZWorld;
	}

	gRRotAngZ[0] = gRAngZWorld;
	for (int i = 0 ; i < gNumOfStep+30 ; i++)
	{
		gRRotAngZ[i*2+1] = AngChange*i + gRAngZWorld;
		gRRotAngZ[i*2+2] = AngChange*i + gRAngZWorld;
	}

	gLLInitZMPFB = -80*sin(gRRotAngZ[0]); // ZMP Initial 
	gRLInitZMPFB = 80*sin(gRRotAngZ[0]);
	gLLInitZMPLR = 80*cos(gRRotAngZ[0]); // ZMP Initial Swing 軌跡會用到 換腳要改
	gRLInitZMPLR = -80*cos(gRRotAngZ[0]);

	double BodyDir = (gLAngZWorld+gRAngZWorld)/2.0;
	double StrideX = y_step_zmp*sin(BodyDir);
	double StrideY = y_step_zmp*cos(BodyDir);

	//R
     gRRotAngPitch[0] = 0.0;
	 gRRotAngPitch[1] = 0.0;
	 gRRotAngPitch[2] = 0.0;

	 for(int i = 3;i< gNumOfStep+30;i++)
      gRRotAngPitch[i] = slopeangle;

	//for(int i = gNumOfStep-4;i< gNumOfStep+30;i++)
	//	 gRRotAngPitch[i] = 0.0;

	 //L
	 gLRotAngPitch[0] = 0.0;
	 gLRotAngPitch[1] = 0.0;
	 gLRotAngPitch[2] = slopeangle;
	 gLRotAngPitch[3] = slopeangle;
	 
	 for(int i = 4;i< gNumOfStep+30;i++)
      gLRotAngPitch[i] = slopeangle;

	 //for(int i = gNumOfStep-5;i< gNumOfStep+30;i++)
		// gLRotAngPitch[i] = 0.0;

	// 左右 
	gFstpX[0] = 0;
	gFstpX[1] = -x_val_zmp*cos(gRRotAngZ[1]); // ZMP Initial Swing 軌跡會用到 換腳要改
	gFstpX[2] = x_val_zmp*cos(gRRotAngZ[0])+StrideX/2.0; // ZMP Initial Swing 軌跡會用到 換腳要改
	for (int i = 2 ; i < gNumOfStep ; i++)
	{
		gFstpX[i*2-1] = gFstpX[(i-1)*2-1] + StrideX;
		gFstpX[i*2] = gFstpX[(i-1)*2] + StrideX;
	}

	if (gKineAll.selSupport[gNumOfStep-4] == 0) // right support
	{
		gFstpX[gNumOfStep-4] = gFstpX[gNumOfStep-5] + 160*cos((gLRotAngZ[gNumOfStep-4]+gRRotAngZ[gNumOfStep-4])/2.0);
	}
	else
	{
		gFstpX[gNumOfStep-4] = gFstpX[gNumOfStep-5] - 160*cos((gLRotAngZ[gNumOfStep-4]+gRRotAngZ[gNumOfStep-4])/2.0);
	}

	gFstpX[gNumOfStep-4] = (gFstpX[gNumOfStep-4]+gFstpX[gNumOfStep-5])/2.0;

	for (int i = gNumOfStep - 3 ; i < gNumOfStep+4 ; i++)
	{
		gFstpX[i] = gFstpX[gNumOfStep-4];
	}

	//gFstpY[0] = 0;
	//gFstpY[1] = distance2L/2.0*sin(gRRotAngZ[2]);
	//gFstpY[2] = -distance2L/2.0*sin(gLRotAngZ[0])+StrideY/2.0;
	//for (int i = 1 ; i < gNumOfStep ; i++)
	//{
	//	gFstpY[i*2+1] = gFstpY[i*2-1]+StrideY;
	//	gFstpY[i*2+2] = gFstpY[i*2]+StrideY;
	//}

	//if (gKineAll.selSupport[gNumOfStep-4] == 0) // right support
	//{
	//	gFstpY[gNumOfStep-4] = gFstpY[gNumOfStep-5] - 160*sin((gLRotAngZ[gNumOfStep-4]+gRRotAngZ[gNumOfStep-4])/2.0);
	//}
	//else
	//{
	//	gFstpY[gNumOfStep-4] = gFstpY[gNumOfStep-5] + 160*sin((gLRotAngZ[gNumOfStep-4]+gRRotAngZ[gNumOfStep-4])/2.0);
	//}

	//gFstpY[gNumOfStep-4] = (gFstpY[gNumOfStep-4]+gFstpY[gNumOfStep-5])/2.0;
	
	//20121217 處理完第一步和最後一步
	gFstpY[0] = 0;
	gFstpY[1] = 0;
	gFstpY[2] = 215;
	gFstpY[3] = 305;
	gFstpY[4] = 405;
	gFstpY[5] = 405;
	gFstpY[6] = 510+230;
	gFstpY[7] = 510+230;
	
	
	//gFstpY[6] = 495+210;
	//gFstpY[7] = 495+210;


	for (int i = gNumOfStep - 3 ; i < gNumOfStep+5 ; i++)
	{
		gFstpY[i] = gFstpY[gNumOfStep-4];
	}

	//以上都跟直走一模一樣//
	  
	//設定要踏高的高度//
	gGroundHeight[0] = 0.0;
	gGroundHeight[1] = 0.0;
	gGroundHeight[2] = (gFstpY[2]-114-5)*tan(-slopeangle);
	gGroundHeight[3] = (gFstpY[3]-114-5)*tan(-slopeangle);
	gGroundHeight[4] = (gFstpY[4]-114-5)*tan(-slopeangle);
	gGroundHeight[5] = (gFstpY[4]-114-5)*tan(-slopeangle);
	gGroundHeight[6] = 40;//(gFstpY[4]-114-5)*tan(-slopeangle);
	//gGroundHeight[7] = (gFstpY[4]-114-5)*tan(-slopeangle);
	//gGroundHeight[8] = (gFstpY[4]-114-5)*tan(-slopeangle);
	
	/*for (int i = 3 ; i < gNumOfStep-4 ; i++)
		gGroundHeight[i] = gGroundHeight[i-1]+ (StrideY/2)*tan(-slopeangle);*/

	for (int i = gNumOfStep-4 ; i < gNumOfStep+25 ; i++)
		gGroundHeight[i] = gGroundHeight[i-1];
}



void gInitneck(int mode) 
{	
	/******************************************************************
	input: 希望的腳本
	output: void
	
	Note://給encoder的值必須是整數
	// mode0 (initial 位置)
	// mode1 (搖頭)
	// mode2 (點頭)
	// mode3  (轉頭) 需要更改RS232丟值方法
	******************************************************************/

	if (gFlagSimulation == RealExp)
	{	
		int i = 0;
		gneck = new SerialPort();
		try{
			gneck->open("\\\\.\\COM12",NORMAL_RS232);
			gneck->_set_baudrate(115200);
		}
		catch(...){
			cout<<"COM open faied"<<endl;
		}

		if (mode == 0)
		{	for (int j = 0 ; j <2 ; j++){
				switch(i){
					case 0 :
						neck_omega = 300;
						thetastart = 1500;
						thetafinal = 1650;
						neck_point = 150;
						neck_motor = 1;
						Sleep(1000);
						i ++ ;
						break;

					case 1 :
						neck_omega = 300;   
						thetastart = 1400;
						thetafinal = 1400;
						neck_point = 10 ;
						neck_motor = 0;
						Sleep(1000);
						i = 0 ;
						break;

					case 2 :
						neck_omega = 400;
						thetastart = 1800;
						thetafinal = 1580;
						neck_point = 1;
						neck_motor = 1;
						i ++ ;
						break;

					case 3 :
						neck_omega = 400;
						thetastart = 800;
						thetafinal = 2000;
						neck_point = 1;
						neck_motor = 0;  	
						i++ ;
						break;

					case 4 :
						neck_omega = 400; 
						thetastart = 2000;
						thetafinal = 1400;
						neck_point = 1;
						neck_motor = 0;
						i = 0 ;
						break;
				}
				neck1.setvalue(thetastart, thetafinal, neck_point, neck_motor, neck_omega);
			}
		}

		if (mode == 1)
		{	
			for (int j = 0 ; j <6 ; j++){
				switch(i){
					case 0 :
						neck_omega = 300;
						thetastart = 1500 ;
						thetafinal = 1650;
						neck_point = 150 ;
						neck_motor = 1;
						i ++ ;
						break;

					case 1 :
						neck_omega = 300;   
						thetastart = 1400 ;
						thetafinal = 1400;
						neck_point = 10 ;
						neck_motor = 0;
						i ++ ;
						break;

					case 2 :
						neck_omega = 300;
						thetastart = 1400;
						thetafinal = 1000;
						neck_point = 50 ;
						neck_motor = 0;
						i ++ ;
						break;

					 case 3 :
						neck_omega = 300;
						thetastart = 1000;
						thetafinal = 1800;
						neck_point = 100 ;
						neck_motor = 0 ;
						i++ ;
						break;
   
					 case 4 :
						neck_omega = 300; 
						thetastart = 1800 ;
						thetafinal = 1000;
						neck_point = 100 ;
						neck_motor = 0;
						i ++ ;
						break;

					 case 5 :
						neck_omega = 300; 
						thetastart = 1000 ;
						thetafinal = 1400;
						neck_point = 50 ;
						neck_motor = 0;
						i = 0 ;
						break;
					}
			neck1.setvalue(thetastart, thetafinal, neck_point, neck_motor, neck_omega);
			}
		}

		if (mode == 2)
		{	
			for (int j = 0 ; j <7 ; j++){
				switch(i){
					case 0 :
						neck_omega = 300;
						thetastart = 1500 ;
						thetafinal = 1650;
						neck_point = 150 ;
						neck_motor = 1;
  						i ++ ;
  						break;

					case 1 :
						neck_omega = 300;   
						thetastart = 1400 ;
						thetafinal = 1400;
						neck_point = 10 ;
						neck_motor = 0;
						//Sleep(1000);
						system("pause"); 
						i ++ ;
						break;

					case 2 :
						neck_omega = 300;
						thetastart = 1650;
						thetafinal = 1750;
						neck_point = 100 ;
						neck_motor = 1;
						i ++ ;
						break;
 
					case 3 :
						neck_omega = 300;
						thetastart = 1750;
						thetafinal = 1500;
						neck_point = 250 ;
						neck_motor = 1 ;
						i++ ;
						break;
   
					 case 4 :
						neck_omega = 300; 
						thetastart = 1500 ;
						thetafinal = 1750;
						neck_point = 250 ;
						neck_motor = 1;
						i ++ ;
						break;

					 case 5 :
						neck_omega = 300; 
						thetastart = 1750 ;
						thetafinal = 1500;
						neck_point = 250 ;
						neck_motor = 0;
						i ++ ;
						break;

					case 6 :
						neck_omega = 300; 
						thetastart = 1500 ;
						thetafinal = 1650;
						neck_point = 150 ;
						neck_motor = 0;
						i = 0 ;
						break;
				}
				neck1.setvalue(thetastart, thetafinal ,neck_point, neck_motor, neck_omega);
			}
		}
	}
}


void CRobotAllDlg::OnBnClickedButton10()
{
	// TODO: 在此加入控制項告知處理常式程式碼
		gFlagEmergentStop=1;

		if (gFlagSimulation == ADAMSSimu){
			OnBnClickedButton9();	// Save 鈕
		}
}


void CRobotAllDlg::OnBnClickedCheck14()
{
	// TODO: 在此加入控制項告知處理常式程式碼
	if (CheckArmCtrl.GetCheck())
		gFlagArmCtrl = 1;
	else
		gFlagArmCtrl = 0;

}


void CRobotAllDlg::OnBnClickedCheck15()
{
	// TODO: Add your control notification handler code here
	if (CheckHandCtrl.GetCheck())
		gFlagHandCtrl = 1;
	else
		gFlagHandCtrl = 0;
}




void gInitLeft_leg_up_and_down_OneStep(void)
{
		/******************************************************************
	input: void
	output: void

	Note:

	// 初始化機器人行為為走樓梯
	// 腳步設定 地形設定 機器人步行參數 設定於此
	// 相似的函數有 gInitTurnLeft gInitWalkStraight gInitStepHere gInitStair gInitSquat gInitDownStair
	// Wei_Zh Lai 20121128
	******************************************************************/
	gNumOfStep = 9; //gNumOfStep = 8  上兩階  //-4
	checkonestep = 0;  //用來判斷在gCalculateZMP()中的gPCenter[1][3] 要等於0 (這樣才zmp才不會在只跨一步的時候 會多左右擺動一次)

	checkWalkOneStepHigh = 1;
	checkLeft_leg_up_and_down_OneStep = 1;


	gCOGDown = 72; // 蹲很多才跨得過
	mode1 = 1 ; 
	//#define gCOGDown		115
	double x_val_zmp = 80;
	bool PNx = 0;
	double y_step_zmp = 500;
	double TurnRadius = 300;


	double distance2L = 160.0;
	double stairheight = 40;
		
	//設定support情況
	
	for (int i = 0 ; i < 400 ; i++)
	{
		gKineAll.selSupport[i] = 0.0;
	}
	gKineAll.selSupport[0] = 2;
	gKineAll.selSupport[1] = 1;
	gKineAll.selSupport[2] = 0;
	gKineAll.selSupport[3] = 1;
	gKineAll.selSupport[4] = 0;
	gKineAll.selSupport[5] = 2;
	gKineAll.selSupport[6] = 2;

	for (int i = 7 ; i < 4000 ; i++)
	{
		gKineAll.selSupport[i] = 2; // double support and prepare to stop
	}
	//	//Slongz 20130425
	for (int i=0;i<1000;i++)
	{
		gKineAll.SupportPhs[i]=-100;

	}
	//gKineAll.PhsIndex((gNumOfStep),(1200),(0),(0),100);
	//for (int i=0;i<gNumOfStep-4;i++)
	//{
	//	for(int j=0;j<1200;j++)
	//	{
	//		if(gKineAll.selSupport[i]==2||gKineAll.selSupport[i]==1)
	//		gKineAll.SupportPhs[i*1200+1000+j]=100;
	//		else
	//		gKineAll.SupportPhs[i*1200+1000+j]=-100;
	//	}
	//}
	//fstream Phs;
	//Phs.open("EncPhs.txt",ios::app);
	//for(int i=0;i<(gNumOfStep-4)*1200;i++)
	//{						  
	//	Phs<<gKineAll.SupportPhs[i]<<"\t";
	////LogEncData<<endl;
	//}
	//Phs.close();

	//	//Slongz 20130425

	//for (int i = 1 ; i < gNumOfStep+30 ; i+=2)
	//{
	//	gKineAll.selSupport[i] = 1;
	//	gKineAll.selSupport[i+1] = 0;
	//}
	//
	for (int i = gNumOfStep-4 ; i < 4000 ; i++)
		gKineAll.selSupport[i] = 2; // double support and prepare to stop	
	
	gKineAll.StepHeight[0] = 0;

	for (int i = 1 ; i < 4000 ; i++)
		gKineAll.StepHeight[i] = 30;

	for (int i = 3 ; i < 4000 ; i++)
		gKineAll.StepHeight[i] = 0;


	gKineAll.StepHeight[2]=50;
	//rotate
	
	double AngChange = 0.0;

	for (int i = 0 ; i < 30 ; i++)
	{
		gLRotAngZ[i*2] = AngChange*i;
		gLRotAngZ[i*2+1] = AngChange*i;
	}

	gRRotAngZ[0] = 0.0 ;
	for (int i = 0 ; i < 30 ; i++)
	{
		gRRotAngZ[i*2+1] = AngChange*i;
		gRRotAngZ[i*2+2] = AngChange*i;
	}

	gLLInitZMPFB = -80*sin(gRRotAngZ[0]);
	gRLInitZMPFB = 80*sin(gRRotAngZ[0]);
	gLLInitZMPLR = 80*cos(gRRotAngZ[0]);
	gRLInitZMPLR = -80*cos(gRRotAngZ[0]);


	// 左右 x方向
	for (int i = 0 ; i < 400 ; i++){
		gFstpX[i] = 0.0;}
	       
        gFstpX[0] = 0;
	    gFstpX[1] =-80;
		gFstpX[2] = 80;
		gFstpX[3] = -80;

		gFstpX[4] = 80;
		gFstpX[5] = 0 ;

		gFstpX[6] = 0;




    /*gFstpX[0] = 0;
	gFstpX[1] = -x_val_zmp;
	for (int i = 1 ; i < 2000 ; i++)
	{
		gFstpX[i*2] = distance2L/2.0;
		gFstpX[i*2+1] = -distance2L/2.0;
	}
	
	;*/


	//只有踏一階
	/*if(checkonestep ==1)
	{
      for (int i = gNumOfStep-4; i < 4000 ; i++)
		gFstpX[i] = 0.0;
	    gFstpX[3] =gFstpX[1];
	}
	else
	{
		for (int i = gNumOfStep-4; i < 4000 ; i++)
		gFstpX[i] = 0.0;

	}*/
	





	for (int i = 0 ; i < 400 ; i++)
		gFstpY[i] = 0.0;

	gFstpY[0] = 0.0;
	gFstpY[1] = 0.0;
	gFstpY[2] = 230.0;
    gFstpY[3] = 0.0;
	gFstpY[4] = 0.0;
	gFstpY[5] = 0.0;
	gFstpY[6] = 0.0;

	

	/*for (int i = 1 ; i < gNumOfStep ; i+=2)	
	{	
		gFstpY[i+1] =  y_step_zmp*0.5*i;
		gFstpY[i+2] =  y_step_zmp*0.5*(i+1);   
	}

	for (int i = gNumOfStep-4 ; i < 200 ; i++)
		gFstpY[i] = gFstpY[gNumOfStep-5];
		*/
	
// Set height of ground
	
	for (int i = 0 ; i < 400 ; i++)
		gGroundHeight[i] = 0.0;
	

	gGroundHeight[0] = 0.0;
	gGroundHeight[1] = 0.0;
	gGroundHeight[2] = 45;
	gGroundHeight[3] = 0;
	gGroundHeight[4] = 0;



	//for (int i = 1 ; i < gNumOfStep-4 ; i++)		
	//	gGroundHeight[i+1] = stairheight *(i)  ;

	//for (int i = gNumOfStep-4 ; i < 1000 ; i++)		
	//	gGroundHeight[i] = gGroundHeight[gNumOfStep-5]   ;


		
		//哲軒改20121127	
	/*for (int pk = 0 ; pk < 11 ; pk++)   //似乎是斜坡
	{
		gGroundHeight[pk] *= 130.0/40.0;
	}*/
}

void gInitLeft_leg_up_and_down_TwoStep(void)
{
	/******************************************************************
	input: void
	output: void

	Note:

	// 初始化機器人行為為走樓梯
	// 腳步設定 地形設定 機器人步行參數 設定於此
	// 相似的函數有 gInitTurnLeft gInitWalkStraight gInitStepHere gInitStair gInitSquat gInitDownStair
	// Wei_Zh Lai 20121128
	******************************************************************/
	gNumOfStep = 9; //gNumOfStep = 8  上兩階  //-4
	checkonestep = 0;  //用來判斷在gCalculateZMP()中的gPCenter[1][3] 要等於0 (這樣才zmp才不會在只跨一步的時候 會多左右擺動一次)

	checkWalkOneStepHigh = 1;
	checkLeft_leg_up_and_down_TwoStep = 1;


	gCOGDown = 72; // 蹲很多才跨得過
	mode1 = 1 ; 
	//#define gCOGDown		115
	double x_val_zmp = 80;
	bool PNx = 0;
	double y_step_zmp = 500;
	double TurnRadius = 300;


	double distance2L = 160.0;
	double stairheight = 40;
		
	//設定support情況
	
	for (int i = 0 ; i < 400 ; i++)
	{
		gKineAll.selSupport[i] = 2.0;
	}
	gKineAll.selSupport[0] = 2;
	gKineAll.selSupport[1] = 1;
	gKineAll.selSupport[2] = 0;
	gKineAll.selSupport[3] = 0;
	gKineAll.selSupport[4] = 2;
	gKineAll.selSupport[5] = 2;
	gKineAll.selSupport[6] = 2;
	gKineAll.selSupport[7] = 2;
	//gKineAll.selSupport[8] = 2;
	//gKineAll.selSupport[6] = 2;

	for (int i = 7 ; i < 4000 ; i++)
	{
		gKineAll.selSupport[i] = 2; // double support and prepare to stop
	}
	//	//Slongz 20130425
	for (int i=0;i<1000;i++)
	{
		gKineAll.SupportPhs[i]=-100;

	}
	//gKineAll.PhsIndex((gNumOfStep),(1200),(0),(0),100);
	//for (int i=0;i<gNumOfStep-4;i++)
	//{
	//	for(int j=0;j<1200;j++)
	//	{
	//		if(gKineAll.selSupport[i]==2||gKineAll.selSupport[i]==1)
	//		gKineAll.SupportPhs[i*1200+1000+j]=100;
	//		else
	//		gKineAll.SupportPhs[i*1200+1000+j]=-100;
	//	}
	//}
	//fstream Phs;
	//Phs.open("EncPhs.txt",ios::app);
	//for(int i=0;i<(gNumOfStep-4)*1200;i++)
	//{						  
	//	Phs<<gKineAll.SupportPhs[i]<<"\t";
	////LogEncData<<endl;
	//}
	//Phs.close();

	//	//Slongz 20130425

	//for (int i = 1 ; i < gNumOfStep+30 ; i+=2)
	//{
	//	gKineAll.selSupport[i] = 1;
	//	gKineAll.selSupport[i+1] = 0;
	//}
	//
	for (int i = gNumOfStep-4 ; i < 4000 ; i++)
		gKineAll.selSupport[i] = 2; // double support and prepare to stop	
	
	gKineAll.StepHeight[0] = 0;

	for (int i = 1 ; i < 4000 ; i++)
		gKineAll.StepHeight[i] = 30;

	for (int i = 3 ; i < 4000 ; i++)
		gKineAll.StepHeight[i] = 0;


	//gKineAll.StepHeight[2]=60;
	//rotate
	
	double AngChange = 0.0;

	for (int i = 0 ; i < 30 ; i++)
	{
		gLRotAngZ[i*2] = AngChange*i;
		gLRotAngZ[i*2+1] = AngChange*i;
	}

	gRRotAngZ[0] = 0.0 ;
	for (int i = 0 ; i < 30 ; i++)
	{
		gRRotAngZ[i*2+1] = AngChange*i;
		gRRotAngZ[i*2+2] = AngChange*i;
	}

	gLLInitZMPFB = -80*sin(gRRotAngZ[0]);
	gRLInitZMPFB = 80*sin(gRRotAngZ[0]);
	gLLInitZMPLR = 80*cos(gRRotAngZ[0]);
	gRLInitZMPLR = -80*cos(gRRotAngZ[0]);


	// 左右 x方向
	for (int i = 0 ; i < 400 ; i++){
		gFstpX[i] = 0.0;}
	       
        gFstpX[0] = 0;
	    gFstpX[1] =-80;
		gFstpX[2] = 80;
		gFstpX[3] = -80;

		gFstpX[4] = 0;
		gFstpX[5] = 0 ;

		gFstpX[6] = 0;




    /*gFstpX[0] = 0;
	gFstpX[1] = -x_val_zmp;
	for (int i = 1 ; i < 2000 ; i++)
	{
		gFstpX[i*2] = distance2L/2.0;
		gFstpX[i*2+1] = -distance2L/2.0;
	}
	
	;*/


	//只有踏一階
	/*if(checkonestep ==1)
	{
      for (int i = gNumOfStep-4; i < 4000 ; i++)
		gFstpX[i] = 0.0;
	    gFstpX[3] =gFstpX[1];
	}
	else
	{
		for (int i = gNumOfStep-4; i < 4000 ; i++)
		gFstpX[i] = 0.0;

	}*/
	





	for (int i = 0 ; i < 400 ; i++)
		gFstpY[i] = 0.0;

	gFstpY[0] = 0.0;
	gFstpY[1] = 0.0;
	gFstpY[2] = 230.0;
    gFstpY[3] = 230.0;
	gFstpY[4] = 230.0;
	gFstpY[5] = 0.0;
	gFstpY[6] = 0.0;

	

	/*for (int i = 1 ; i < gNumOfStep ; i+=2)	
	{	
		gFstpY[i+1] =  y_step_zmp*0.5*i;
		gFstpY[i+2] =  y_step_zmp*0.5*(i+1);   
	}

	for (int i = gNumOfStep-4 ; i < 200 ; i++)
		gFstpY[i] = gFstpY[gNumOfStep-5];
		*/
	
// Set height of ground
	
	for (int i = 0 ; i < 400 ; i++)
		gGroundHeight[i] = 0.0;
	

	gGroundHeight[0] = 0.0;
	gGroundHeight[1] = 0.0;
	gGroundHeight[2] = 45;
	gGroundHeight[3] = 45;
	gGroundHeight[4] = 45;



	//for (int i = 1 ; i < gNumOfStep-4 ; i++)		
	//	gGroundHeight[i+1] = stairheight *(i)  ;

	//for (int i = gNumOfStep-4 ; i < 1000 ; i++)		
	//	gGroundHeight[i] = gGroundHeight[gNumOfStep-5]   ;


		
		//哲軒改20121127	
	/*for (int pk = 0 ; pk < 11 ; pk++)   //似乎是斜坡
	{
		gGroundHeight[pk] *= 130.0/40.0;
	}*/
}

void CRobotAllDlg::OnBnClickedButton11()
{
	// TODO: 在此加入控制項告知處理常式程式碼
	#if TwinCAT_Mode
	
	for(int i=0;i<12;i++)
	{
		if(EposError[i]==1)
			TCAT->EtherCATFaultReset(i);
	}

	#endif// TwinCAT_Mode

}


void gDeleteLogFile(void)
{
	/******************************************************************
	input: void
	output: void

	Note:

	// 於程式開始前刪除所有DATA記錄檔
	// 已由ios::out | ios::trunc 代替
	******************************************************************/	
	//system("del EncTimeLogData.txt");	// EncTimeLogData為 gLQs.LogSysTime 實驗timer 計時  
	//system("del EncLogData.txt");
	//system("del EncVelLogData.txt");
	//system("del EncTorqueLogData.txt");

	//#if BangBangControl
	//system("del EncMovAveVelLogData.txt");
	//system("del EncLogSdTorqueOffset.txt");
	//system("del EncDesireVelLogData.txt");
	//system("del EncVelDiffLogData.txt");
	//system("del EncDiffLogData.txt");
	//system("del EncLogSdSFunction.txt");
	//#endif

	if (gFlagInfrared == 1)
	{	
		system("del infaredR_dis.txt");
		system("del infaredL_dis.txt");
	}
	
	if (gFlagIMU == 1)
	{	
		system("del EncLogIMUAngVel.txt");
		system("del EncLogIMUAngVelMA.txt");
		system("del IMUvel.txt");
		system("del IMUaccel.txt");
		system("del IMUangle.txt");
	}
}
void gRecordLogFile(void)
{
	/******************************************************************
	input: void
	output: void

	Note:

	// 寫出所有DATA記錄檔
	******************************************************************/	

	//TWIN_CAT DataRecord
		//Record Enc
	fstream LogEncData;
	LogEncData.open("EncLogData.txt",ios::out | ios::trunc);
		for(int i=0;i<LogEncCount;i++){						  
			for(int j=0;j<12;j++)
				LogEncData<<LogEnc[j+12*i]<<"\t";
		LogEncData<<endl;
		}
	LogEncData.close();

	//Record Torque
	LogEncData.open("EncTorqueLogData.txt",ios::out | ios::trunc);
		for(int i=0;i<LogEncCount;i++){						  
			for(int j=0;j<12;j++)
				LogEncData<<LogTorque[j+12*i]<<"\t";
		LogEncData<<endl;
		}
	LogEncData.close();

	//Record Systime
	LogEncData.open("EncTimeLogData.txt",ios::out | ios::trunc);
		for(int i=0;i<LogEncCount;i++){						  
			LogEncData<<gLQs.LogSysTime[i]<<endl;
		}
	LogEncData.close();

	//Record Vel
	LogEncData.open("EncVelLogData.txt",ios::out | ios::trunc);
		for(int i=0;i<LogEncCount;i++){						  
			for(int j=0;j<12;j++)
				LogEncData<<LogVel[j+12*i]<<"\t";
		LogEncData<<endl;
		}
	LogEncData.close();

	LogEncData.open("Enc_FKCOG.txt",ios::out | ios::trunc);	// 130925 WZ線上收COG
	for(int i = 0 ; i < LogEncCount ; i++){
		for(int j = 0 ; j < 3 ; j++)
			LogEncData<<gEnc_FKCOG[j+3*i]<<"\t";
		LogEncData<<endl;
	}

	#if BangBangControl
	//Record Vel Moving Average
	LogEncData.open("EncMovAveVelLogData.txt",ios::out | ios::trunc);
	for(int i=0;i<LogEncCount;i++){						  
		for(int j=0;j<12;j++)
			LogEncData<<LogMovAveVel[j+12*i]<<"\t";
		LogEncData<<endl;
	}
	LogEncData.close();			
	
	//Record EncDiff
	LogEncData.open("EncDiffLogData.txt",ios::out | ios::trunc);
		for(int i=0;i<LogEncCount;i++){						  
			for(int j=0;j<12;j++)
				LogEncData<<LogEncDiff[j+12*i]<<"\t";
		LogEncData<<endl;
		}
	LogEncData.close();	
	
	//Record Desire Vel
	LogEncData.open("EncDesireVelLogData.txt",ios::out | ios::trunc);
		for(int i=0;i<LogEncCount;i++){						  
			for(int j=0;j<12;j++)
				LogEncData<<LogDesireVel[j+12*i]<<"\t";
		LogEncData<<endl;
		}
	LogEncData.close();	
	
	//Record Vel Diff
	LogEncData.open("EncVelDiffLogData.txt",ios::out | ios::trunc);
		for(int i=0;i<LogEncCount;i++){						  
			for(int j=0;j<12;j++)
				LogEncData<<LogVelDiff[j+12*i]<<"\t";
		LogEncData<<endl;
		}
	LogEncData.close();	
	
	//Record Sliding Torque Offset						
	LogEncData.open("EncLogSdTorqueOffset.txt",ios::out | ios::trunc);
		for(int i=0;i<LogEncCount;i++){						  
			for(int j=0;j<12;j++)
				LogEncData<<LogTorqueOffset[j+12*i]<<"\t";
		LogEncData<<endl;
		}
	LogEncData.close();
	
	LogEncData.open("EncLogSdSFunction.txt",ios::out | ios::trunc);
		for(int i=0;i<LogEncCount;i++){						  
			for(int j=0;j<12;j++)
				LogEncData<<LogSFunction[j+12*i]<<"\t";
		LogEncData<<endl;
		}
	LogEncData.close();
	#endif

	if (gFlagIMU == 1)
	{
		LogEncData.open("EncLogIMUAngVel.txt",ios::app);
		for(int i=0;i<LogEncCount;i++){						  
			for(int j=0;j<3;j++)
				LogEncData<<LogIMUAngVel[j+3*i]<<"\t";
			LogEncData<<endl;
		}
		LogEncData.close();

		LogEncData.open("EncLogIMUAngVelMA.txt",ios::app);
			for(int i=0;i<LogEncCount;i++){ 
				for(int j=0;j<3;j++)
					LogEncData<<LogIMUAngVelMA[j+3*i]<<"\t";
				LogEncData<<endl;
			}
		LogEncData.close();	
	}

//TWIN_CAT DataRecord
}

void gWarningJointLimit(long * buf)
{
	/******************************************************************
	input: void
	output: void

	Note:自動確認當下動作是否接近EPOS3 joint limit
	******************************************************************/

	for (int i = 0 ; i < 12 ; i++)
	{
		if (*(buf+i) > gJointUpLimit[i]-4000)
		{
			printf("\n警告!!! 第%d軸角度%d count接近上限%d count\n",i+1,*(buf+i),gJointUpLimit[i]);
		}
		if (*(buf+i) < gJointLoLimit[i]+4000)
		{
			printf("\n警告!!! 第%d軸角度%d count接近下限%d count\n",i+1,*(buf+i),gJointLoLimit[i]);
		}
	}
}

void gReadPreloadTorque(void)
{
	/******************************************************************
	input: void
	output: void

	Note:

	// 於程式開始前預先讀入需要的Torque offset (Estimated Torque from Inverse dynamics)
	******************************************************************/	
  	fstream gTorqueOffsetFile;
#if ILC_UpStair
	gTorqueOffsetFile.open("TorqueOffset_up3_201305211635_ILC06.txt",ios::in);//FilterEncTorqueLogDataQQ.txt
	for (int j=0;j<gDataTotal;j++)
	{
		for (int i=0;i<12;i++)
		{
			//TempENC[i][j] = 0;
			gTorqueOffsetFile>>gTorqueOffsetFileBuf1[j*12+i];
			gShortTorqueOffsetFileBuf1[j*12+i]=short(gTorqueOffsetFileBuf1[j*12+i]);
		}
	}
	gTorqueOffsetFile.close();	
#endif
	//for (int j=0;j<gDataTotal;j++)
	//{
	//	for (int i=0;i<12;i++)
	//	{
	//		//TempENC[i][j] = 0;
	//		//gTorqueOffsetFile>>gTorqueOffsetFileBuf1[j*12+i];
	//		gShortTorqueOffsetFileBuf2[j*12+i]=0;//short(gTorqueOffsetFileBuf1[j*12+i]);
	//	}
	//}			
#if ILC_DownStair
	gTorqueOffsetFile.open("TorqueOffset_down3_201305171730_ILC07.txt",ios::in);//FilterEncTorqueLogDataQQ.txt
	for (int j=0;j<gDataTotal;j++)
	{
		for (int i=0;i<12;i++)
		{
			//TempENC[i][j] = 0;
			gTorqueOffsetFile>>gTorqueOffsetFileBuf2[j*12+i];
			gShortTorqueOffsetFileBuf2[j*12+i]=short(gTorqueOffsetFileBuf2[j*12+i]);
		}
	}
	gTorqueOffsetFile.close();	
#endif
}

void gReadArmTraj(void)
{
	fstream OfflineData;
	//******ARM Traj 檔案輸入用
	int TrajLength = 38392;//38392;//軌跡檔SAMPLE數
	int InitLength = 1000;//初始化SAMPLE數
	int PNLHand[6] = {1,1,1,1,1,-1};//左手正負組態係數
	int PNRHand[6] = {-1,-1,-1,-1,-1,-1};//右手正負組態係數
	//******
			OfflineData.open("Final_LAJointData.txt",ios::in);//open left arm file
		//OfflineData2.open("RightArmTrajTest.txt",ios::in);
		for(int i = 0;i <TrajLength; i++)
		{
			for(int j=0;j<6;j++)//22為手臂FILE資料格式 前6筆為手臂角度
			{
				OfflineData>>gLHandTrajFileBuf[i][j];
				# if ArmDH
				if(j==2)
					gLHandTrajFileBuf[i][j]+=90;
				else if (j==4)
					gLHandTrajFileBuf[i][j]-=90;
				#endif
			//OfflineData2>>temptraj2[i][j];
			}
		}
		//for(int i=0;i<TrajLength;i++)
		//{
		//	for(int j=0;j<6;j++)
		//	{
		//		gLArmOfflineTraj[(2*i)*6+j+InitLength*6] = gHandTrajFileBuf[i][j]/180*3.1415926 * PNLHand[j];
		//	}
		//}
		//for(int i=0;i<TrajLength;i++)
		//{
		//	for(int j=0;j<6;j++)
		//	{
		//		if(i == (TrajLength-1))
		//		{
		//			gLArmOfflineTraj[(2*i+1)*6+j+InitLength*6] = gLArmOfflineTraj[(2*i)*6+j+InitLength*6];
		//		}
		//		else
		//		{
		//			gLArmOfflineTraj[(2*i+1)*6+j+InitLength*6] = (gLArmOfflineTraj[(2*i)*6+j+InitLength*6] + gLArmOfflineTraj[(2*i+2)*6+j+InitLength*6])/2;
		//		}
		//	}
		//}
		OfflineData.close();
		OfflineData.open("Final_RAJointData.txt",ios::in);
		for(int i = 0;i <TrajLength; i++)
		{
			//for(int j=0;j<22;j++)
			for(int j=0;j<6;j++)
			{
				OfflineData>>gRHandTrajFileBuf[i][j];		
				#if ArmDH
				if(j==2)
					gRHandTrajFileBuf[i][j]+=90;
				else if (j==4)
					gRHandTrajFileBuf[i][j]-=90;
				#endif
			}
		}
		//for(int i=0;i<TrajLength;i++)
		//{
		//	for(int j=0;j<6;j++)
		//	{
		//		gRArmOfflineTraj[(2*i)*6+j+InitLength*6] = gHandTrajFileBuf[i][j]/180*3.1415926* PNRHand[j];
		//		//gRArmOfflineTraj[(2*i+1)*6+j+InitLength*6] = gHandTrajFileBuf[i][j]/180*3.1415926* PNRHand[j];
		//	}
		//}	
		//for(int i=0;i<TrajLength;i++)
		//{
		//	for(int j=0;j<6;j++)
		//	{
		//		if(i == (TrajLength-1))
		//		{
		//			gRArmOfflineTraj[(2*i+1)*6+j+InitLength*6] = gRArmOfflineTraj[(2*i)*6+j+InitLength*6];
		//		}
		//		else
		//		{
		//			gRArmOfflineTraj[(2*i+1)*6+j+InitLength*6] = (gRArmOfflineTraj[(2*i)*6+j+InitLength*6] + gRArmOfflineTraj[(2*i+2)*6+j+InitLength*6])/2;
		//		}
		//	}
		//}
		OfflineData.close();	
}


void gIntLEDFace (int mode)
{
	/******************************************************************
	input: int mode 
	output: void

	Note:
	// 臉部表情 只有實驗模式可開啟
	// 用 mode 控制表情
	// 泓逸
	******************************************************************/

	unsigned char comment[5]={255,0,8,0,255}; // for robot face LED array
			
	if (gFlagSimulation == RealExp)	// 只能在實驗模式開啟
	{
		gpPortHead->_write(comment,5);
		gpPortHead->_write(comment,5);
		gpDP->TransENC(5, ID_SetENC+(mode)*CMD_SET);
		gpPortHead->_write(gpDP->TxData,9);
	}
}

void gEncFeedback(void)
{
	/******************************************************************
	input: void 
	output: void

	Note:
	// 線上讀encoder 或離線讀txt測試
	// 由FinfFK FindCOG 找到實際的COG	
	// 角度與角速度皆為 弧度  並且皆為機器人方向
	// 放的位置要注意 會影響到gKineAll.FindEncCOG(COGtheta) 的步數計算
	// 此函數只能用在Prepare Scenario 有的(不能用Stay Mode) 因為要調整fix frame
	// 不會在OpenGL看到
	// 20140120 WZ
	******************************************************************/
	double temp_enc[12];	// motor theta
	for (int i= 0; i < 12 ; i++){
		if (gFlagSimulation == RealExp){
			if (LogEncCount == 0){
				temp_enc[i] = LogEnc[LogEncCount*12+i];	// 線上回收ENC (不會在OpenGL看到) 
			}
			else{
				temp_enc[i] = LogEnc[(LogEncCount-1)*12+i];	// 線上回收ENC (不會在OpenGL看到) 
			}
		} 
		else if(gFlagSimulation == CppSimu || gFlagSimulation == ADAMSSimu){
			temp_enc[i] = gKineAll.COG_ENCload[TickNumber*12+i];	// 非實驗的話輸入txt測試
		}

		if (i == 0 || i ==6) // yaw axis
			temp_enc[i] = (temp_enc[i] / 78353.2027529); // *2000*160*40/26/2pi 減速比
		else // other axes
			temp_enc[i] = (temp_enc[i] / 74896.4438101); // *2000*160*50/34/2pi 減速比

		temp_enc[i] = temp_enc[i]*gPNJoints[i];	// DH theta
	}		

	gKineAll.FindEncCOG(temp_enc, gEnc_FKCOG+LogEncCount*3);	// 找COG
}

void gSendOfflineTraj(void)
{
	/******************************************************************
	input: void 
	output: void

	Note:
	// 加入旗標OfflineTraj 線上直接丟入離線ENC (已乘過gPNJoints 可以直接輸入給EPOS的那種軌跡)
	// txt請排成 n筆*12軸
	// 在OpenGL可看到軌跡
	// 20140220 WZ
	******************************************************************/
	#if OfflineTraj
		double feed_enc[12];
		for (int i= 0; i < 12 ; i++){
			feed_enc[i] = gKineAll.COG_ENCload[TickNumber*12+i];	// motor theta
		
			if (i == 0 || i ==6) // yaw axis
				feed_enc[i] = (feed_enc[i] / 78353.2027529); // *2000*160*40/26/2pi 減速比
			else // other axes
				feed_enc[i] = (feed_enc[i] / 74896.4438101); // *2000*160*50/34/2pi 減速比

			feed_enc[i] = feed_enc[i]*gPNJoints[i];	// DH theta
		}

		for (int h = 0 ; h < 6 ; ++h)
		{
			gKineAll.FKLLeg->theta[h+1] = feed_enc[h];	// 左腳
			gKineAll.FKRLeg->theta[h+1] = feed_enc[h+6];	// 左腳
		}
	#endif
}

void gSpeakSignLanguage(int IndexCount)
{
	switch(IndexCount)
	{
	case 600:
		gSpeaker.PlaySoundByFile(gSpeaker.SignLanq[0]);
			 
		break;
	case 1009:
		
		gSpeaker.PlaySoundByFile(gSpeaker.SignLanq[1]);
		break;
	case 1209:
		
		gSpeaker.PlaySoundByFile(gSpeaker.SignLanq[2]);
		break;
	case 1750:
		
		gSpeaker.PlaySoundByFile(gSpeaker.SignLanq[3]);
		break;
	case 2400:
		
		gSpeaker.PlaySoundByFile(gSpeaker.SignLanq[4]);
		break;
	case 3000:
		
		gSpeaker.PlaySoundByFile(gSpeaker.SignLanq[5]);
		break;
	case 3500:
		
		gSpeaker.PlaySoundByFile(gSpeaker.SignLanq[6]);
		break;
	case 3941:
		
		gSpeaker.PlaySoundByFile(gSpeaker.SignLanq[7]);
		break;
	case 4431:
		
		gSpeaker.PlaySoundByFile(gSpeaker.SignLanq[8]);
		break;
	case 5200:
		
		gSpeaker.PlaySoundByFile(gSpeaker.SignLanq[9]);
		break;
	case 5700:
		
		gSpeaker.PlaySoundByFile(gSpeaker.SignLanq[10]);
		break;
	case 6400:
		
		gSpeaker.PlaySoundByFile(gSpeaker.SignLanq[11]);
		break;
	case 7000:
		
		gSpeaker.PlaySoundByFile(gSpeaker.SignLanq[12]);
		break;
	case 7500:
		
		gSpeaker.PlaySoundByFile(gSpeaker.SignLanq[13]);
		break;
	case 8100:
		
		gSpeaker.PlaySoundByFile(gSpeaker.SignLanq[14]);
		break;
	case 8800:
		
		gSpeaker.PlaySoundByFile(gSpeaker.SignLanq[15]);
		break;
	case 8991:
		
		gSpeaker.PlaySoundByFile(gSpeaker.SignLanq[16]);
		break;
	case 9156:
		
		gSpeaker.PlaySoundByFile(gSpeaker.SignLanq[17]);
		break;
	case 9600:
		
		gSpeaker.PlaySoundByFile(gSpeaker.SignLanq[18]);
		break;
	case 10300:
		
		gSpeaker.PlaySoundByFile(gSpeaker.SignLanq[19]);
		break;
	case 10700:
		
		gSpeaker.PlaySoundByFile(gSpeaker.SignLanq[20]);
		break;
	case 11100:
		
		gSpeaker.PlaySoundByFile(gSpeaker.SignLanq[21]);
		break;
	case 11600:
		
		gSpeaker.PlaySoundByFile(gSpeaker.SignLanq[22]);
		break;
	case 11677:
		
		gSpeaker.PlaySoundByFile(gSpeaker.SignLanq[23]);
		break;
	case 12050:
		
		gSpeaker.PlaySoundByFile(gSpeaker.SignLanq[24]);
		break;
	case 12900:
		
		gSpeaker.PlaySoundByFile(gSpeaker.SignLanq[25]);
		break;
	case 13300:
		
		gSpeaker.PlaySoundByFile(gSpeaker.SignLanq[26]);
		break;
	case 13900:
		
		gSpeaker.PlaySoundByFile(gSpeaker.SignLanq[27]);
		break;
	case 14500:
		
		gSpeaker.PlaySoundByFile(gSpeaker.SignLanq[28]);
		break;
	case 14850:
		
		gSpeaker.PlaySoundByFile(gSpeaker.SignLanq[29]);
		break;
	case 15300:
		
		gSpeaker.PlaySoundByFile(gSpeaker.SignLanq[30]);
		break;
	case 15650:
		
		gSpeaker.PlaySoundByFile(gSpeaker.SignLanq[31]);
		break;
	case 16400:
		
		gSpeaker.PlaySoundByFile(gSpeaker.SignLanq[32]);
		break;
	case 16800:
		
		gSpeaker.PlaySoundByFile(gSpeaker.SignLanq[33]);
		break;
	case 17600:
		
		gSpeaker.PlaySoundByFile(gSpeaker.SignLanq[34]);
		break;
	case 18100:
		
		gSpeaker.PlaySoundByFile(gSpeaker.SignLanq[35]);
		break;
	
   case 18550:
		gIntLEDFace(2); //LEDFace change  
	    break;

	
	case 18650:
		
		gSpeaker.PlaySoundByFile(gSpeaker.SignLanq[36]);

		break;
	default:
		break;

	}
}

void uprightcontrol(float *IMUroll , float  *IMUpitch ,double *Ldeltatheta , double *Rdeltatheta , int count )
{ 	    
	//單位rad
	double rollerror =  0 ;
	double rollerrorbefore = 0   ;
	double pitcherror  = 0 ; 
	double pitcherrorbefore = 0  ; 
		
	double Pgainroll = 0.6  ;
	double Dgainroll = 0.01 ;
				
	double Pgainpitch = 0.5  ;
	double Igainpitch = 0 ;
	double Dgainpitch = 0.1 ;
				
	double Hiproll = 0 ;
	double Hippitch = 0 ;

	double anglemax  =  0 ;	

    //沒有control 的要給 0 
	for (int i = 0 ; i<6 ; i++)
	{	
		Ldeltatheta[i] = 0;
		Rdeltatheta[i] = 0;	
	}
				
	if (count <10){
		rollerrorbefore =  0 ;
		rollerror =  0 ;

		pitcherrorbefore = 0 ;
		pitcherror = 0 ;
	} 
	else{
		rollerrorbefore =  0 - *(IMUroll+count-1) ;
		rollerror =  0 - *(IMUroll+count) ;
				
		pitcherrorbefore = 0 - *(IMUpitch+count-1);
		pitcherror =  0 - *(IMUpitch+count);
	}

	//adams mode 

	//PD control  desired = 0 
   
	Hiproll  = Pgainroll *rollerror +Dgainroll*(rollerror - rollerrorbefore)/0.05 ;		
	Hippitch = Pgainpitch *pitcherror +  Igainpitch*(pitcherror - pitcherrorbefore)*0.05 +  Dgainpitch*(pitcherror - pitcherrorbefore)/0.05 ; 		
	if(gKineAll.selIK==1 /*&& gKineAll.FSensor_forcR[2] < 20000000*/)//  右腳support  與 力規值搭配
	{
		Rdeltatheta[1] =  -Hiproll;  
		Rdeltatheta[2] =  Hippitch;  //右腳Z軸 與adams pitch同方向  直接將error加入
	}
    else if(gKineAll.selIK==0 /*&& gKineAll.FSensor_forcL[2] < 20000000*/) //左腳support  index 1,2 為 HIP _ joint
	{		
		Ldeltatheta[1] =  -Hiproll;
		Ldeltatheta[2] =  -Hippitch;	
	}
	else{		
		Ldeltatheta[1] =  Hiproll/2;	
		Ldeltatheta[2] = -Hippitch;        //左右腳不同號			
		Rdeltatheta[1] =  -Hiproll/2;			
		Rdeltatheta[2] =  Hippitch;			       				
	}

	//// 設定每次角度補償上限
	if(abs(Ldeltatheta[1]) > anglemax)
	{if (Ldeltatheta[1]>0)
		Ldeltatheta[1] = anglemax ; //考慮正負號
		else
        Ldeltatheta[1] = -anglemax;
	}

	if(abs(Ldeltatheta[2]) > anglemax)
	{if (Ldeltatheta[2]>0)
		Ldeltatheta[2] = anglemax ; //考慮正負號
		else
        Ldeltatheta[2] = -anglemax;
	}

    if(abs(Rdeltatheta[1]) > anglemax)
	{if (Rdeltatheta[1]>0)
		Rdeltatheta[1] = anglemax ; //考慮正負號
		else
        Rdeltatheta[1] = -anglemax;
	}

    if(abs(Rdeltatheta[2]) > anglemax)
	{if (Rdeltatheta[2]>0)
		Rdeltatheta[2] = anglemax ; //考慮正負號
		else
        Rdeltatheta[2] = -anglemax;
	}

} 
