/**************************************************************************************************
Copyright, 2010-2012, Robotics Lab., Dept. of M.E., National Taiwan University
File Name: GlobalConst.h

Author: Jiu-Lou Yan
Version: 1.5
Date: 2012/02/01

Functions:None

Classes: None

Description:
     本標頭檔定義了關於步行的基礎物理量

Note: None
***************************************************************************************************/

// 模擬與實驗相關
// 步行相關
#define GravityConst	9810 // 重力常數
#define dt				0.005 // control sampling time
#define dtArm           0.005
#define dtUSB2C32       0.02
#define PI				3.1415926

#define T_P				6.0 // 每步所花時間
#define StepSam			(T_P/dt) // 每步的sample數

// 時機控制參數，步行軌跡參數
// DSP SSP代表行走中，ZMP切換的雙腳支撐時間百分比
// DSP = 0.4; // double support phase
// SSP = 0.6 // single support phase
// DSP + SSP = 1.0
#define DSP  0.3
#define	SSP  0.7

#define LQSIBufferSize 120000 // 為了LQSI控制運算的預留記憶體長度
#define SensorBufferSize 180000 // 所有sensor feedback buffer size

// 設定 Timer來源
enum TimerSource{MControl,Thread}; //Mcontrol: 使用Motion Control Thread計時， Thread: 另開一個Timer專用 Thread計時(試作)



// 機器人側向移動旗標定義
enum SideWalkDirection{DirectionLeft, DirectionRight};
// ZeroHome 回歸完全伸直 BentHome 1 回歸時回歸到蹲好 ShiftZeroHome 2 // 只有機器人自我介紹會用 偷偷將重心往後移一點 SlopeHome 3 斜坡專用 一直丟最後一筆
enum HomeType{ZeroHome, BentHome, ShiftZeroHome, SlopeHome};
// 機器人停止劇本時的狀態機
enum HomeSteps{HomeInitial, Homing, HomeFinished};
// 機器人support腳的描述
enum SupportPhases{LeftSupport, RightSupport, DoubleSupport};
// 機器人操作模式 包含 C++模擬, ADAMS模擬, 真實實驗
enum RobotExeMode{CppSimu, ADAMSSimu, RealExp};
// 設定致能或者禁用
enum EnableFlags{DisableFlag, EnableFlag};

// 設定 gPreProcessData 參數，這是用來跟機器人溝通要設定機器人參數用的function
enum RobotCommunication{SetHome,SetOnePID,Manual,SetIdle,Reserved1,Traj,SetAllPID,SetPWMLims,SetCali,SetEncReadFlag};

//===================================
// Others
//===================================
//Plot and Data 
#define CheckingMode 0 //1: 存各軸軌跡(弧度) 劃軌跡格線&顯示ZMP COG軌跡 & 格線 0: 功能關閉

//ZMP Feedback
//#define ZMPFeedbackMode 0 //1: Matlab量測之ZMP state feedback回LQSI 0: 功能關閉 (Warning: May possibly failed in current version)

//Knee-stretch Motion
#define SupportCCD 0 // =1: Support腳用IKCCDSolve解IK
#define QCCDMode 0 // =1: IK Solver 替換成QCCD Mode
#define QCCDSwAngle 10 // (Degree) for IK-solver swithcing
#define FootpadRotation 0// =0: 關閉 =1: 啟動Toe-off and Heel-contact Motions 
#define ConstantCOGMode 1// =0: Flu =1 Constant COG
#define ErrorRatio 0.05// QCCD Error Bound Modification

//Save Data
#define WriteInitZMPtxt 0 // 將Initial時候的規劃好的ZMP 以及COG寫入txt file
#define SaveSwingTraj 1 // 設定Kine物件之下 GenSwingTraj() GenSwingTrajMod() 要不要寫入文字檔案部分 (擺動腳運動軌跡)
#define SaveLQSIStates 0 // 設定LQSISolver物件之下 DummyControl() 要不要寫入文字檔案部分 (倒單擺計算結果)

//Simulation on Adams
#define AdamsSMode 1 // =0: 請配合泓逸Matlab程式 =1: 請配合Slongz Matlab程式

//Arm DH Settings
#define ArmDH 1 //		1:大金(1st Demo及腳組預設手臂初始方向設定)		0: 聖翔(手語軌跡手臂初始方向設定)

//Stair-Climbing
//#define StairClimbing 1 //	 //0516 Modified: 取消定義形態 (宣告在RobotAllDlg.cpp)

//===================================
// Sensors
//===================================
#define LaserCatch 0 //1: 開啟雷射 0: 關閉
#define PMSUpdate 1	//是否開啟PMS畫圖 

//#define TwinCAT_Mode 1	位置移至:MainLoops.h
//關掉此項 可以離線測試 不須與TwinCAT溝通 
//但注意需要把 方案總管-RobotAll(右鍵)-屬性-連結器-輸入-其他相依性 刪掉TCatIoDrv.lib 才可以離線測試

#define RunDynamics 0	// 開啟動力學計算 = 1
#define OfflineTraj 0	// 離線軌跡版本

//===================================
// Controllers
//===================================
#define BangBangControl 0; //	1: 開啟Sliding Mode 補償項     0:關閉
#define PreloadTorque 0;
#define ILC_UpStair 0;//ILC up offset buf1
#define ILC_DownStair 0;// buf2

#define SoundCtrl 0;// 0:取消第一段前置語音

////哲軒20140218
#define uprightfeedback 0  //開啟上身姿態補償 = 1 
#define cogestimate 0	// 開啟估測COG (FK_COG) = 1 


///////// Define 整理規劃

//Define太多不容易整理..

//重要項目且不會因模式或參術切換改變屬性者用
//次要項目請改成flag(如Exp Simu ADAMS 須自動切換不同模式者), 方便微調或在UI定義修改
//
//盡量改成常駐模式 (在同一個mode盡量維持全開或全關)
/////  Debug or 檢查少用者盡量不使用define來開開關
//請大家保持良好習慣，自定義的功能開關以不影響其他使用者或原本程式運行之設定為主