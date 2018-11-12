/**************************************************************************************************
Copyright, 2010-2012, Robotics Lab., Dept. of M.E., National Taiwan University
File Name: Kine.cpp

Author: Jiu-Lou Yan
Version: 1.0
Date: 2010/07/20

Functions:
	Kine() ~Kine() InitKineTrains() FindFK() ComputeJacobians()
	CalCoord() CCDCOGSolve() CCDIKSolve() CCDInit() CheckJointLimit()
	ComputeEulerAng() ComputeFixJacobians() ComputeJacobians() 
	Cross2Vd() Cross2Vf() FDAccelCOM() FDAccelJ() FDAlphaL() FDForceJ()
	FDOmegaL() FDTorqueJ() FDVelCOM() FDVelJ() FindCOG() FindD() FindDIni()
	FindFixQFK() FindFK() FindInertia2LocalCOM() FindPseudoJ()
	FindSwingQFK() FindWLN() ForceSensorData() Gen10DegPoly() Gen7DegPoly() Gen9DegPoly()
	GenCOGZTrajMod() GenSmoothZMPShift() GenSmoothZMPShift_ZeroJerk() 
	GenSwingTraj() GenSwingTrajMod() GenSwingTrajMod2() GenZMPFreeAssign() 
	GetCOGFixJacobian() GetCOGJacobian() GetLegsCoords() GetRotPartRn()
	IKCCDSolve() IKFixSolve() IKSolve() InitInertia() InitKineTrains()
	InitQKineTrains() InitWLN() InvWithSingTest() NormVecArray() NormXYZD() 
	NumDiff() SetIdentity() UpdateDrawingBuffer()

Classes: Kine

Description:
     本程式主要用在計算所有運動學功能，包含forward kinematics, inverse kinematics, 
	 Jacobian matrix, joint limit avoidance, singularity avoidance, dynamics 等主要功能
	 各函式與變數之說明請詳見下方宣告與定義處之說明

Note: None
***************************************************************************************************/


#include "stdafx.h"
#include <iostream>
#include <fstream>
#include "Kine.h"
#include <math.h>


using namespace std;

//#define PI 3.1415926  // 在Global Constant 設定
#define OldRobot 0 //Kine.cpp r_com mass_com 參數指定
	

extern unsigned int gIthIK; // 記憶IK解到第幾格
extern int gStepSample; // 記憶走到第幾步
extern int gInitCount; // 計算到第幾格
extern int check_slopeangle;
extern int gFlagSimulation;
extern bool gUpStair;

Kine::Kine(void) // not used
{ 
	/******************************************************************
	input: void
	output: void

	Note:
	// Class constructor  初始化所有需要用到的變數與矩陣
	******************************************************************/
	LegDHLen = 13;
	//泓逸start111213
	ArmDHLen = 10;
	//ArmDHLen = 4;
	//泓逸end111213

	FKLLeg = new FwdKine(13);
	FKRLeg = new FwdKine(13);
	//泓逸start111213
//	FK_LArm = new FwdKine(4); // 左手
//	FK_RArm = new FwdKine(4); // 右手
	FKLArm = new FwdKine(10); // 左手
	FKRArm = new FwdKine(10); // 右手
	//泓逸end111213

	CrdAll = new YMatLite(LegDHLen*2+ArmDHLen*2,3);
	ZAxisAll= new YMatLite(LegDHLen*2+ArmDHLen*2,3);

	EndEffDiff = new double[36];

	dth = new double[24]; // solved angular vel in each IK loop
	dx = new double[24];

	selIK = 2; // double support phase
	// selIK = 0 : left foot is the supporter
    // selIK = 1 : right foot is the supporter

	//泓逸start111220
	JaMRow = 24;
	JaNCol = 24;
	Ja = new YMatLite(JaMRow,JaNCol);

	tempJ = new double[Ja->MSize];
	tempJT = new double[Ja->MSize];
	tempJiWJT = new double[Ja->MRow*Ja->MRow];
	tempInv = new double[Ja->MRow];

	// Singularity avoidance
	work_clapack = new double[Ja->MRow*Ja->MRow];  // the same size as tempJiWJT
	ipiv_clapack = new long[Ja->MRow*Ja->MRow]; // the same size as tempJiWJT
	tempJJT_for_inv = new double[Ja->MRow*Ja->MRow];
	//泓逸start111227

	PseudoInv =  new double[Ja->MRow*Ja->MRow];

	for (int i = 0 ; i < Ja->MRow*Ja->MRow ; i++)
	{
		tempJJT_for_inv[i] = 0;
		PseudoInv[i] = 0;
	}

	//r_com = new double[10];
	//mass_com = new double[10];

	//pv_stack = new double[6*3+2*3+2*3];
	//temp_Norm = new double[6*1+2+2];

	//泓逸start111214
	r_com = new double[16];
	mass_com = new double[16];
	pv_stack = new double[6*3+8*3+2*3];
	temp_Norm = new double[6*1+8+2];
	//一隻腳三塊 一隻手四塊 腰兩塊
	UpBodyUpVec[0] = 0;
	UpBodyUpVec[1] = 0;
	UpBodyUpVec[2] = 0;
	//泓逸end111214
	COG[0] = 0;
	COG[1] = 0;
	COG[2] = 0;

	BodyUpVec[0] = 0;
	BodyUpVec[1] = 0;
	BodyUpVec[2] = 0;

	BodyRightVec[0] = 0;
	BodyRightVec[1] = 0;
	BodyRightVec[2] = 0;
	
	BodyFaceVec[0] = 0;
	BodyFaceVec[1] = 0;
	BodyFaceVec[2] = 0;

	DHOrigin[0] = 0;
	DHOrigin[1] = 0;
	DHOrigin[2] = 0;

	IndexBuf = 0; // clear index
	IndexBufRev = 0; // clear index

	if (selIK == LeftSupport)
	{
		RobotFixPoint[0] = 0;
		RobotFixPoint[1] = 80;
		RobotFixPoint[2] = 139.7;
	}
	else if (selIK == RightSupport || selIK == DoubleSupport)
	{
		RobotFixPoint[0] = 0;
		RobotFixPoint[1] = -80;
		RobotFixPoint[2] = 139.7;
	}

// from Matlab

//mass_com = [5200 2284 2743 ... %LLeg    Hip ㄇ字型部分以及上面的SHD25跟馬達，規腰下半部加算 620(ㄇ字型) + 910 (SHD25) + 480 (150W motor)
//           5200 2284 2743 ... %RLeg
//           5  ...             %LArm
//           5  ...             %RArm
//           3678+(620+910+480)*2 6432];                  %Body下半部 = 7698 上半部 = body value 14000  , 6423是是去掉電池的重量
//r_com = [109.88 134.74 56.95 ... %LLeg
//         109.88 134.74 56.95 ... %RLeg
//         206.16 ...                          %LArm
//         206.16 ...                          %RArm
//         45.6178 85];                    %Body 有加電池的時候，最後一項 109.25

// from Matlab

	//泓逸start111214
	r_com[0] = 109.88;  	r_com[1] = 134.74;  	r_com[2] = 56.95;   // LL
	r_com[3] = 109.88;  	r_com[4] = 134.74;  	r_com[5] = 56.95;   // RL
	r_com[6] = 136.41;  	r_com[7] = 130.98;		r_com[8] = 76.53;		r_com[9] = 5;// LArm
	r_com[10] = 136.41;     	r_com[11] = 130.98;		r_com[12] = 76.53;		r_com[13] = 5;// RArm
	r_com[14] = 45.6178;		r_com[15] = 109.25;								// Body 有加電池的時候，最後一項 109.25 // 只有空腰的時候85

	mass_com[0] = 5200;  	mass_com[1] = 2284;  	mass_com[2] = 2743;	// LL
	mass_com[3] = 5200;  	mass_com[4] = 2284;  	mass_com[5] = 2743;	// RL
	mass_com[6] = 3091;     mass_com[7] = 2751;  	mass_com[8] = 1508;		mass_com[9] = 5;	// LArm
	mass_com[10] = 3091;     mass_com[11] = 2751;  	mass_com[12] = 1508;		mass_com[13] = 5;// RArm
	mass_com[14] = 7698;		mass_com[15] = 14000;							
	// Body下半部 = 7698(左右兩腳0-1,1-2軸的LINK)  
	// 上半部 = body value 14000(電池+鋁矩形)  , 6423是是去掉電池的重量
	// 左右手總重14710
	// 2800 是配重
	// 總重56862

	//#if RunDynamics 130925 移出define
	// 以新量到的重量為主 必放在原本的後面 r_com則不用動 大致相等(軸的中點) 加電池
	mass_com[0] = 6300;  	mass_com[1] = 2800;  	mass_com[2] = 3800;	// LL
	mass_com[3] = 6300;  	mass_com[4] = 2800;  	mass_com[5] = 3800;	// RL 左右腳總重25800
	mass_com[14] =10698;	mass_com[15] = 20000; 
	//#endif


	#if  OldRobot
	//r_com = [109.88 134.74 56.95 ... %LLeg
	//         109.88 134.74 56.95 ... %RLeg
	//         206.16 ...                          %LArm
	//         206.16 ...                          %RArm
	//         28.894 160.771];                       %Body (從身體原點14.416向下) (93.352從身體下面pitch軸向上)

	r_com[8] = 28.894;		r_com[9] = 160.771;								// Body 有加電池的時候，最後一項 109.25 // 只有空腰的時候85

	//mass_com = [3117 1837 2730 ... %LLeg
	//            3117 1837 2730 ... %RLeg
	//            2434  ...                %LArm
	//            2434  ...                %RArm
	//            4367 22492];                  %Body下半部 = 493 上半部 = body value 770.37 
	mass_com[0] = 3117;  	mass_com[1] = 1837;  	mass_com[2] = 2730;	// LL
	mass_com[3] = 3117;  	mass_com[4] = 1837;  	mass_com[5] = 2730;	// RL
	mass_com[6] = 2434;  													// LArm
	mass_com[7] = 2434;  													// RArm
	mass_com[8] = 4367;		mass_com[9] = 22492;							// %Body下半部 = 7698 上半部 = body value 14000  , 6423是是去掉電池的重量
	#endif

	SumMass = 0;
	for (int i = 0 ; i < 16 ; i++)
	{
		SumMass += mass_com[i];
	}
	//泓逸end111214

 // 多項式，用以計算swing腳軌跡
 //A1 = [0       0       0      0      0      1; % x0 position eqn
 //      x1^5    x1^4    x1^3   x1^2   x1     1; % x1 position eqn
 //      0       0       0      0      1      0; % x0 velocity eqn
 //      5*x1^4  4*x1^3  3*x1^2 2*x1   1      0; % x1 velocity eqn
 //      0       0       0      2      0      0; % x0 acceleration eqn
 //      20*x2^3 12*x2^2 6*x2   2      0      0];% x1 acceleration eqn
 // 初始化的時候，把不會變的先設好了
	A1[0] = 0;  A1[1] = 0;  A1[2] = 0;  A1[3] = 0;  A1[4] = 0;  A1[5]  = 1;
	                                                            A1[11] = 1;
	A1[12] = 0; A1[13] = 0; A1[14] = 0; A1[15] = 0; A1[16] = 1; A1[17] = 0;
                                                    A1[22] = 1; A1[23] = 0;
	A1[24] = 0; A1[25] = 0; A1[26] = 0; A1[27] = 2; A1[28] = 0; A1[29] = 0;
	                                    A1[33] = 2; A1[34] = 0; A1[35] = 0;


 //A2 = [x1^5    x1^4    x1^3    x1^2    x1      1; % x1 position
 //      x2^5    x2^4    x2^3    x2^2    x2      1; % x2 position
 //      5*x1^4  4*x1^3  3*x1^2  2*x1    1       0; % x1 velocity
 //      5*x2^4  4*x2^3  3*x2^2  2*x2    1       0; % x2 velocity
 //      0       0       0       2       0       0; % x1 acceleration
 //      20*x2^3 12*x2^2 6*x2    2       0       0];% x2 acceleration

																A2[5]  = 1;
	                                                            A2[11] = 1;
													A2[16] = 1; A2[17] = 0;
                                                    A2[22] = 1; A2[23] = 0;
	A2[24] = 0; A2[25] = 0; A2[26] = 0; A2[27] = 2; A2[28] = 0; A2[29] = 0;
	                                    A2[33] = 2; A2[34] = 0; A2[35] = 0;
							
	//  r1 = [ 0 ; y1 ; v0 ; 0 ; a0 ; 0 ];
    //  %      x0  x1   v0  v1  a0  a1
	r1[0] = 0; r1[3] = 0; r1[5] = 0;

   //   r2 = [ y1 ; y2 ; 0 ; 0 ; 0 ; 0 ];
   //   %      x1   x2   v1  v2  a1  a2
	r2[2] = 0; r2[3] = 0; r2[4] = 0; r2[5] = 0;
 // 多項式，用以計算swing腳軌跡

	N_step = T_P/dt;
	Nab = N_step*SSP;

	Nza = (N_step-Nab)/2;
	Nzb = N_step-Nab-Nza;

    SwErrLim = 0.1; // swing 解IK 誤差值 mm 0.1
	COGErrLim = 0.01; // COG 解IK 誤差值 mm 0.01
	AngleErrLim = 0.001; // radian 0.001

    MaxSwingError = 1.5; // maximum input position trajectory difference 防止機構衝太快
	// 1.5 * 200 = 300mm/s 最快每秒30cm
	MaxCOGError = 1.5; //maximum input position trajectory difference 防止機構衝太快
	// 1.5 * 200 = 300mm/s 最快每秒30cm
	MaxRotationError = 0.00655; // maximum input angle trajectory difference 防止機構衝太快
	// 0.00655 * 200 * 57 每秒最快75度
	MaxJointAngleChange = 0.007;  // 每秒最快90度

	cntIK = 0;
	stepIndex = 0;

	FirstFKFound = false;

	SingularJudge = 0.0000000000001; // 10^-13
	SingularAdd = 0.1;

	// Footpad outline size for Euler angle coordinate construction
	LenEdgeXYZ[0] = 210.0;
	LenEdgeXYZ[1] = 140.0;
	LenEdgeXYZ[2] = 139.7;

	// WLN
	InitWLN();

	//// 換腳時候腳底板旋轉矩陣 初始化成單位矩陣
	//LSwitchRMot[0] = 1;	LSwitchRMot[1] = 0;	LSwitchRMot[2] = 0;
	//LSwitchRMot[3] = 0;	LSwitchRMot[4] = 1;	LSwitchRMot[5] = 0;
	//LSwitchRMot[6] = 0;	LSwitchRMot[7] = 0;	LSwitchRMot[8] = 1;

	//// 換腳時候腳底板旋轉矩陣 初始化成單位矩陣
	//RSwitchRMot[0] = 1;	RSwitchRMot[1] = 0;	RSwitchRMot[2] = 0;
	//RSwitchRMot[3] = 0;	RSwitchRMot[4] = 1;	RSwitchRMot[5] = 0;
	//RSwitchRMot[6] = 0;	RSwitchRMot[7] = 0;	RSwitchRMot[8] = 1;

	// 換腳時候腳底板旋轉矩陣 初始化成單位矩陣
	LSwitchRMot[0] = 1;	    LSwitchRMot[1] = 0;  	LSwitchRMot[2] = 0;   LSwitchRMot[3] = 0;
	LSwitchRMot[4] = 0;	    LSwitchRMot[5] = 1;	    LSwitchRMot[6] = 0;   LSwitchRMot[7] = 0;
	LSwitchRMot[8] = 0;	    LSwitchRMot[9] = 0;	    LSwitchRMot[10]= 1;   LSwitchRMot[11]= 0;
	LSwitchRMot[12]= 0;	    LSwitchRMot[13]= 0; 	LSwitchRMot[14]= 0;   LSwitchRMot[15]= 1;

	// 換腳時候腳底板旋轉矩陣 初始化成單位矩陣
	RSwitchRMot[0] = 1;	    RSwitchRMot[1] = 0;  	RSwitchRMot[2] = 0;   RSwitchRMot[3] = 0;
	RSwitchRMot[4] = 0;	    RSwitchRMot[5] = 1;	    RSwitchRMot[6] = 0;   RSwitchRMot[7] = 0;
	RSwitchRMot[8] = 0;	    RSwitchRMot[9] = 0;	    RSwitchRMot[10]= 1;   RSwitchRMot[11]= 0;
	RSwitchRMot[12]= 0;	    RSwitchRMot[13]= 0; 	RSwitchRMot[14]= 0;   RSwitchRMot[15]= 1;

	// 剛算完FK的Fix Leg 腳底板坐標系 初始化成單位矩陣
	FixEndEffRMot[0] = 1;	    FixEndEffRMot[1] = 0;  	FixEndEffRMot[2] = 0;   FixEndEffRMot[3] = 0;
	FixEndEffRMot[4] = 0;	    FixEndEffRMot[5] = 1;	FixEndEffRMot[6] = 0;   FixEndEffRMot[7] = 0;
	FixEndEffRMot[8] = 0;	    FixEndEffRMot[9] = 0;	FixEndEffRMot[10]= 1;   FixEndEffRMot[11]= 0;
	FixEndEffRMot[12]= 0;	    FixEndEffRMot[13]= 0; 	FixEndEffRMot[14]= 0;   FixEndEffRMot[15]= 1;

	// clear and initialize 每步抬高
	for (int i = 0 ; i < 4000 ; i++)
		StepHeight[i] = 0.0;


	// 設定機構上下限角度
	//      yaw                        roll                       pitch                      pitch                     pitch                     roll
	JointUpLimitLL[0] = 45.0;  JointUpLimitLL[1] = 38.0;   JointUpLimitLL[2] = 13.77;   JointUpLimitLL[3] = 95.0; JointUpLimitLL[4] = 40.545; JointUpLimitLL[5] = 38.0; // 左腳上限
	JointLoLimitLL[0] = -22.0;  JointLoLimitLL[1] = -16.45;   JointLoLimitLL[2] = -74.9;   JointLoLimitLL[3] = -10.0; JointLoLimitLL[4] = -63.4; JointLoLimitLL[5] = -29.0; // 左腳下限
	JointUpLimitRL[0] = 22.0;  JointUpLimitRL[1] = 16.45;   JointUpLimitRL[2] = 74.9;   JointUpLimitRL[3] = 10.0; JointUpLimitRL[4] = 63.4; JointUpLimitRL[5] = 29.0; // 左腳上限
	JointLoLimitRL[0] = -45.0;  JointLoLimitRL[1] = -38.0;   JointLoLimitRL[2] = -13.77;   JointLoLimitRL[3] = -95.0; JointLoLimitRL[4] = -40.545; JointLoLimitRL[5] = -38.0; // 左腳下限

	for (int i = 0 ; i < 6 ; i++)
	{  
		JointUpLimitLL[i] /= (180.0/3.1415926);
		JointUpLimitRL[i] /= (180.0/3.1415926);
		JointLoLimitLL[i] /= (180.0/3.1415926);
		JointLoLimitRL[i] /= (180.0/3.1415926);
	}

	//_______________________________________QCCD (Slongz)_______________________________________
	int FixJaMRow = 6;
	int FixJaNCol = 6;
	FixJa = new YMatLite(FixJaMRow,FixJaNCol);
	CCDInit();
	//_______________________________________QCCD (Slongz)_______________________________________

	#if RunDynamics
 	//_________________________實際模擬時關閉以下for loop 因為以下是讀已存下力規txt WZ______________________
		//KineFile.open("TestAdamsSimOriginalStairFilt.txt",ios::in);	// 上樓梯
		//KineFile.open("TestAdamsSimOriginalFilt.txt",ios::in);	// 走路(9,200)
		//KineFile.open("FSMerge04271800.txt",ios::in);	// EXP走路(9,200)
		//KineFile.open("FSMerge310.txt",ios::in); // 實際量回的力規值
		//AdamsMotionFile.open("TestAda\msMotionT.txt",ios::in);	//20130115 直接輸入Adams量到的MotionTorque (失敗)
		//KineFile.open("EncLogData.txt",ios::in);	// 讀回ENC變成COG 20130525 WZ
		//for (int i = 0 ; i < 12*8816+1 ; i++)
		//{
		//	KineFile >> COG_ENCload[i];	// 每次從已經在Kine打開的文件中收12筆資料讀ENC
		//}
	//_________________________實際模擬時關閉以下for loop 因為以下是讀以存下力規txt______________________
	
	// Rated Torque 單位 = N*m*10^-6
	// Original
	//RatedTorque[0] = 1.74*53.8;	RatedTorque[1] = 3.17*60.3;	RatedTorque[2] = 1.67*62.2; 
	//RatedTorque[3] = 3.17*60.3;	RatedTorque[4] = 1.67*62.2;	RatedTorque[5] = 1.67*62.2;
	//RatedTorque[6] = 1.74*53.8;	RatedTorque[7] = 3.17*60.3;	RatedTorque[8] = 1.67*62.2;
	//RatedTorque[9] = 3.17*60.3;	RatedTorque[10] = 1.67*62.2;	RatedTorque[11] = 1.67*62.2; 
	//
	RatedTorque[0] = 1.74*53.8;	RatedTorque[1] = 4.5*60.3;	RatedTorque[2] = 1.67*62.2; 
	RatedTorque[3] = 6*60.3;	RatedTorque[4] = 2.5*62.2;	RatedTorque[5] = 2.5*62.2;
	RatedTorque[6] = 1.74*53.8;	RatedTorque[7] = 4.5*60.3;	RatedTorque[8] = 1.67*62.2;
	RatedTorque[9] = 6*60.3;	RatedTorque[10] = 2.5*62.2;	RatedTorque[11] = 2.5*62.2; 

	// RatedTorque單位調整+TCAT中的馬達單位為千分之一的Rated Torque
	for(int i = 0 ; i < 12 ; i++)
		RatedTorque[i] *=0.000001;

	// 每個馬達自己的減速比
	GearRatio[0] = 160*40/26 ; GearRatio[1] = 160*50/34 ; GearRatio[2] = 160*50/34 ;
	GearRatio[3] = 160*50/34 ; GearRatio[4] = 160*50/34 ; GearRatio[5] = 160*50/34 ;
	GearRatio[6] = 160*40/26 ; GearRatio[7] = 160*50/34 ; GearRatio[8] = 160*50/34 ;
	GearRatio[9] = 160*50/34 ; GearRatio[10] = 160*50/34 ; GearRatio[11] = 160*50/34 ;

	// 各軸的摩擦力 先Viscous 再 Coulomb Friction
	FrictionJ[0] = 65;	FrictionJ[1] = 85;	FrictionJ[2] = 50;	FrictionJ[3] = 85;
	FrictionJ[4] = 50;	FrictionJ[5] = 50;	FrictionJ[6] = 80;	FrictionJ[7] = 80;
	FrictionJ[8] = 40;	FrictionJ[9] = 85;	FrictionJ[10] = 50;	FrictionJ[11] = 40;	

	for(int i = 12 ; i < 24 ; i++)	// Coulomb Friction 皆令為1
		FrictionJ[i] = 1;

	system("del Testerrorframe.txt");	// WZ131123
	system("del TestJoint.txt");	// WZ131123
	system("del TestdeltaKF.txt");	// WZ131123
	check_slopeangle = 1;	// 利用斜坡的IK矩陣
	#endif

	#if OfflineTraj
		OfflineNum = 12400;	// 手動輸入離線的軌跡筆數 給馬達的encoder值 已乘過PNjoint
		KineFile.open("test0001.txt",ios::in);	// 讀回ENC變成COG 20130525 WZ
		for (int i = 0 ; i < 12*OfflineNum+1 ; i++){
			KineFile >> COG_ENCload[i];	// 每次從已經在Kine打開的文件中收12筆資料讀ENC
		}
		KineFile.close();
	#endif

	#if cogestimate
		if (gFlagSimulation == CppSimu || gFlagSimulation == ADAMSSimu)
		{
			OfflineNum = 8816;	// 手動輸入離線的軌跡筆數
			KineFile.open("EncLogData.txt",ios::in);	// 讀回ENC變成COG 20130525 WZ
			for (int i = 0 ; i < 12*OfflineNum+1 ; i++){
				KineFile >> COG_ENCload[i];	// 每次從已經在Kine打開的文件中收12筆資料讀ENC
			}
			KineFile.close();
		}
	#endif

	FirstEncCOG = false;
	Enc_stepIndex = 0;
	
	// Enc Feedback換腳時候腳底板旋轉矩陣 初始化成單位矩陣
	Enc_LSwitchRMot[0] = 1;	    Enc_LSwitchRMot[1] = 0;  	Enc_LSwitchRMot[2] = 0;   Enc_LSwitchRMot[3] = 0;
	Enc_LSwitchRMot[4] = 0;	    Enc_LSwitchRMot[5] = 1;	    Enc_LSwitchRMot[6] = 0;   Enc_LSwitchRMot[7] = 0;
	Enc_LSwitchRMot[8] = 0;	    Enc_LSwitchRMot[9] = 0;	    Enc_LSwitchRMot[10]= 1;   Enc_LSwitchRMot[11]= 0;
	Enc_LSwitchRMot[12]= 0;	    Enc_LSwitchRMot[13]= 0; 	Enc_LSwitchRMot[14]= 0;   Enc_LSwitchRMot[15]= 1;

	// Enc Feedback換腳時候腳底板旋轉矩陣 初始化成單位矩陣
	Enc_RSwitchRMot[0] = 1;	    Enc_RSwitchRMot[1] = 0;  	Enc_RSwitchRMot[2] = 0;   Enc_RSwitchRMot[3] = 0;
	Enc_RSwitchRMot[4] = 0;	    Enc_RSwitchRMot[5] = 1;	    Enc_RSwitchRMot[6] = 0;   Enc_RSwitchRMot[7] = 0;
	Enc_RSwitchRMot[8] = 0;	    Enc_RSwitchRMot[9] = 0;	    Enc_RSwitchRMot[10]= 1;   Enc_RSwitchRMot[11]= 0;
	Enc_RSwitchRMot[12]= 0;	    Enc_RSwitchRMot[13]= 0; 	Enc_RSwitchRMot[14]= 0;   Enc_RSwitchRMot[15]= 1;
	// WZ 201309005


	// KF Initial
	for (int i = 0 ; i < 12 ; i ++){
		Q_KF[i] = 0.05;	// Q為model的coverence 越小越相信
		R_KF[i] = 3;	// R為measuremant的coverence 越小越相信	
		x_est_last[i] = 0;
		P_last[i] = 0;
		//x_est_lastMotor[i] = 0;
		//P_lastMotor[i] = 0;
		//QMotor[i] = 0.1;	// Q為model的coverence 越小越相信
		//RMotor[i] = 1;	// R為measuremant的coverence 越小越相信
	}

	RotPhaseFlag=0;
	for (int i=0;i<1200;i++){
		SwingFreeCoef[i]=1.0;
	}
	qTarSwing = new Quaternion();
	qShankSwing = new Quaternion();
	qChangedSwing = new Quaternion();
	for (int i=0;i<100;i++){
		RotFlagSeq[i]=0;
	}
	RotFlagSeq[2]=1;
	RotFlagSeq[3]=0;
	//RotFlagSeq[4]=1;
	//RotFlagSeq[5]=1;
	//RotFlagSeq[6]=1;
	//RotFlagSeq[7]=1;
	PhsFlag=0;
}

Kine::~Kine(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// Class destructor  釋放所有動態記憶體
	******************************************************************/

	delete FKLLeg;
	delete FKRLeg;
	delete FKLArm;
	delete FKRArm;
	delete CrdAll;
	delete ZAxisAll;
	delete Ja;

	delete[] r_com;
	delete[] mass_com;

	delete[] pv_stack;
	delete[] temp_Norm;

	delete[] EndEffDiff;
	delete[] dth;
	delete[] dx;

	delete[] tempJ;
	delete[] tempJT;
	delete[] tempJiWJT;
	delete[] tempInv;

	delete[] work_clapack;
	delete[] ipiv_clapack;
	delete[] tempJJT_for_inv;

	delete[] PseudoInv;
	delete FixJa;
	delete QFKLLeg;
	delete QFKRLeg;
	delete qTarSwing;
	delete qShankSwing;
	delete qChangedSwing;
	delete[] ArmDrawingBuffer;
	delete[] LegDrawingBuffer;
}


void Kine::InitKineTrains(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 讀取文字檔案得到所有kinematics train的 DH parameters
	******************************************************************/
		fstream LLeg_DH;

		LLeg_DH.open("LLeg.txt",ios::in);

		for(int i=0;i<LegDHLen;i++)
			LLeg_DH>>FKLLeg->d[i] ;

		for(int i=0;i<LegDHLen;i++)
			LLeg_DH>>FKLLeg->a[i] ;

		for(int i=0;i<LegDHLen;i++)
			LLeg_DH>>FKLLeg->theta[i];

		for(int i=0;i<LegDHLen;i++)
			LLeg_DH>>FKLLeg->theta_home[i];

		for(int i=0;i<LegDHLen;i++)
			LLeg_DH>>FKLLeg->alpha[i];

		for(int i=0;i<LegDHLen;i++)
		{
			FKLLeg->theta[i]=FKLLeg->theta[i]*PI/180.0;
			FKLLeg->alpha[i]=FKLLeg->alpha[i]*PI/180.0;
			FKLLeg->theta_home[i]=FKLLeg->theta_home[i]*PI/180.0;
		}
		LLeg_DH.close();

	//------------< Right Leg>------------
		//FwdKine		FKRLeg(LegDHLen);

		fstream RLeg_DH;
		RLeg_DH.open("RLeg.txt",ios::in);
		for(int i=0;i<LegDHLen;i++)
			RLeg_DH>>FKRLeg->d[i] ;

		for(int i=0;i<LegDHLen;i++)
			RLeg_DH>>FKRLeg->a[i] ;

		for(int i=0;i<LegDHLen;i++)
			RLeg_DH>>FKRLeg->theta[i];

		for(int i=0;i<LegDHLen;i++)
			RLeg_DH>>FKRLeg->theta_home[i];

		for(int i=0;i<LegDHLen;i++)
			RLeg_DH>>FKRLeg->alpha[i];

		for(int i=0;i<LegDHLen;i++)
		{
			FKRLeg->theta[i]=FKRLeg->theta[i]*PI/180.0;
			FKRLeg->alpha[i]=FKRLeg->alpha[i]*PI/180.0;
			FKRLeg->theta_home[i]=FKRLeg->theta_home[i]*PI/180.0;
		}
		RLeg_DH.close();

	// left arm
		fstream LArm_DH;
#if (ArmDH )
		LArm_DH.open("LArmDaiKin.txt",ios::in);
#else
		LArm_DH.open("LArm.txt",ios::in);
#endif

		for(int i=0;i<ArmDHLen;i++)
			LArm_DH>>FKLArm->d[i] ;

		for(int i=0;i<ArmDHLen;i++)
			LArm_DH>>FKLArm->a[i] ;

		for(int i=0;i<ArmDHLen;i++)
			LArm_DH>>FKLArm->theta[i];

		for(int i=0;i<ArmDHLen;i++)
			LArm_DH>>FKLArm->theta_home[i];

		for(int i=0;i<ArmDHLen;i++)
			LArm_DH>>FKLArm->alpha[i];

		for(int i=0;i<ArmDHLen;i++)
		{
			FKLArm->theta[i]=FKLArm->theta[i]*PI/180.0;
			FKLArm->alpha[i]=FKLArm->alpha[i]*PI/180.0;
			FKLArm->theta_home[i]=FKLArm->theta_home[i]*PI/180.0;
		}
		LArm_DH.close();

	// right arm
		fstream RArm_DH;

#if (ArmDH )
		RArm_DH.open("RArmDaiKin.txt",ios::in);
#else
		RArm_DH.open("RArm.txt",ios::in);
#endif

		for(int i=0;i<ArmDHLen;i++)
			RArm_DH>>FKRArm->d[i] ;

		for(int i=0;i<ArmDHLen;i++)
			RArm_DH>>FKRArm->a[i] ;

		for(int i=0;i<ArmDHLen;i++)
			RArm_DH>>FKRArm->theta[i];

		for(int i=0;i<ArmDHLen;i++)
			RArm_DH>>FKRArm->theta_home[i];

		for(int i=0;i<ArmDHLen;i++)
			RArm_DH>>FKRArm->alpha[i];

		for(int i=0;i<ArmDHLen;i++)
		{
			FKRArm->theta[i]=FKRArm->theta[i]*PI/180.0;
			FKRArm->alpha[i]=FKRArm->alpha[i]*PI/180.0;
			FKRArm->theta_home[i]=FKRArm->theta_home[i]*PI/180.0;
		}
		RArm_DH.close();
}

void Kine::FindFK(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 求出所有kinematics train 的 forward kinematics
	******************************************************************/

	// forward kinematics
	FKLLeg->DHConstruct();
	FKRLeg->DHConstruct();
	FKLArm->DHConstruct();
	FKRArm->DHConstruct();

	// 取出腳底板旋轉矩陣 並且將整隻機器人旋轉至適當方向
	if (selIK == LeftSupport)
	{
		// x axis -> joint 9 - joint 10
		FixEndEffRMot[0] = (FKLLeg->Rn[9].data[3]-FKLLeg->Rn[10].data[3])/LenEdgeXYZ[0];
		FixEndEffRMot[4] = (FKLLeg->Rn[9].data[7]-FKLLeg->Rn[10].data[7])/LenEdgeXYZ[0];
		FixEndEffRMot[8] = (FKLLeg->Rn[9].data[11]-FKLLeg->Rn[10].data[11])/LenEdgeXYZ[0];

		// y axis -> joint 11 - joint 10
		FixEndEffRMot[1] = (FKLLeg->Rn[11].data[3]-FKLLeg->Rn[10].data[3])/LenEdgeXYZ[1];
		FixEndEffRMot[5] = (FKLLeg->Rn[11].data[7]-FKLLeg->Rn[10].data[7])/LenEdgeXYZ[1];
		FixEndEffRMot[9] = (FKLLeg->Rn[11].data[11]-FKLLeg->Rn[10].data[11])/LenEdgeXYZ[1];

		// z axis -> joint 6 - joint 7
		FixEndEffRMot[2] = (FKLLeg->Rn[6].data[3]-FKLLeg->Rn[7].data[3])/LenEdgeXYZ[2];
		FixEndEffRMot[6] = (FKLLeg->Rn[6].data[7]-FKLLeg->Rn[7].data[7])/LenEdgeXYZ[2];
		FixEndEffRMot[10] = (FKLLeg->Rn[6].data[11]-FKLLeg->Rn[7].data[11])/LenEdgeXYZ[2];

		for (int i = 0 ; i < LegDHLen; i++)
		{
			MatMulAtB(FixEndEffRMot,4,4,FKLLeg->Rn[i].data,4,4,temp44MatMul);
			MatMulAB(LSwitchRMot,4,4,temp44MatMul,4,4,FKLLeg->Rn[i].data);
		}
		for (int i = 0 ; i < LegDHLen; i++)
		{
			MatMulAtB(FixEndEffRMot,4,4,FKRLeg->Rn[i].data,4,4,temp44MatMul);
			MatMulAB(LSwitchRMot,4,4,temp44MatMul,4,4,FKRLeg->Rn[i].data);
		}
		for (int i = 0 ; i < ArmDHLen; i++)
		{
			MatMulAtB(FixEndEffRMot,4,4,FKLArm->Rn[i].data,4,4,temp44MatMul);
			MatMulAB(LSwitchRMot,4,4,temp44MatMul,4,4,FKLArm->Rn[i].data);
		}
		for (int i = 0 ; i < ArmDHLen; i++)
		{
			MatMulAtB(FixEndEffRMot,4,4,FKRArm->Rn[i].data,4,4,temp44MatMul);
			MatMulAB(LSwitchRMot,4,4,temp44MatMul,4,4,FKRArm->Rn[i].data);

		}
	}
	else if (selIK == RightSupport || selIK == DoubleSupport)
	{

		// x axis -> joint 8 - joint 11
		FixEndEffRMot[0] = (FKRLeg->Rn[8].data[3]-FKRLeg->Rn[11].data[3])/LenEdgeXYZ[0];
		FixEndEffRMot[4] = (FKRLeg->Rn[8].data[7]-FKRLeg->Rn[11].data[7])/LenEdgeXYZ[0];
		FixEndEffRMot[8] = (FKRLeg->Rn[8].data[11]-FKRLeg->Rn[11].data[11])/LenEdgeXYZ[0];

		// y axis -> joint 10 - joint 11
		FixEndEffRMot[1] = (FKRLeg->Rn[10].data[3]-FKRLeg->Rn[11].data[3])/LenEdgeXYZ[1];
		FixEndEffRMot[5] = (FKRLeg->Rn[10].data[7]-FKRLeg->Rn[11].data[7])/LenEdgeXYZ[1];
		FixEndEffRMot[9] = (FKRLeg->Rn[10].data[11]-FKRLeg->Rn[11].data[11])/LenEdgeXYZ[1];

		// z axis -> joint 6 - joint 7
		FixEndEffRMot[2] = (FKRLeg->Rn[6].data[3]-FKRLeg->Rn[7].data[3])/LenEdgeXYZ[2];
		FixEndEffRMot[6] = (FKRLeg->Rn[6].data[7]-FKRLeg->Rn[7].data[7])/LenEdgeXYZ[2];
		FixEndEffRMot[10] = (FKRLeg->Rn[6].data[11]-FKRLeg->Rn[7].data[11])/LenEdgeXYZ[2];

		for (int i = 0 ; i < LegDHLen; i++)
		{
			MatMulAtB(FixEndEffRMot,4,4,FKLLeg->Rn[i].data,4,4,temp44MatMul);
			MatMulAB(RSwitchRMot,4,4,temp44MatMul,4,4,FKLLeg->Rn[i].data);
		}
		for (int i = 0 ; i < LegDHLen; i++)
		{
			MatMulAtB(FixEndEffRMot,4,4,FKRLeg->Rn[i].data,4,4,temp44MatMul);
			MatMulAB(RSwitchRMot,4,4,temp44MatMul,4,4,FKRLeg->Rn[i].data);
		}
		for (int i = 0 ; i < ArmDHLen; i++)
		{
			MatMulAtB(FixEndEffRMot,4,4,FKLArm->Rn[i].data,4,4,temp44MatMul);
			MatMulAB(RSwitchRMot,4,4,temp44MatMul,4,4,FKLArm->Rn[i].data);
		}
		for (int i = 0 ; i < ArmDHLen; i++)
		{
			MatMulAtB(FixEndEffRMot,4,4,FKRArm->Rn[i].data,4,4,temp44MatMul);
			MatMulAB(RSwitchRMot,4,4,temp44MatMul,4,4,FKRArm->Rn[i].data);
		}
	}

	// 旋轉完後取出所有軸點座標
	int WriteIndex = 0;
	for (int i = 0 ; i < LegDHLen; i++)
	{
		CrdAll->data[WriteIndex] = FKLLeg->Rn[i].data[3];
		CrdAll->data[WriteIndex+1] = FKLLeg->Rn[i].data[7];
		CrdAll->data[WriteIndex+2] = FKLLeg->Rn[i].data[11];
		WriteIndex += 3;
	}
	for (int i = 0 ; i < LegDHLen; i++)
	{
		CrdAll->data[WriteIndex] = FKRLeg->Rn[i].data[3];
		CrdAll->data[WriteIndex+1] = FKRLeg->Rn[i].data[7];
		CrdAll->data[WriteIndex+2] = FKRLeg->Rn[i].data[11];
		WriteIndex += 3;
	}
	for (int i = 0 ; i < ArmDHLen; i++)
	{
		CrdAll->data[WriteIndex] = FKLArm->Rn[i].data[3];
		CrdAll->data[WriteIndex+1] = FKLArm->Rn[i].data[7];
		CrdAll->data[WriteIndex+2] = FKLArm->Rn[i].data[11];
		WriteIndex += 3;
	}
	for (int i = 0 ; i < ArmDHLen; i++)
	{
		CrdAll->data[WriteIndex] = FKRArm->Rn[i].data[3];
		CrdAll->data[WriteIndex+1] = FKRArm->Rn[i].data[7];
		CrdAll->data[WriteIndex+2] = FKRArm->Rn[i].data[11];
		WriteIndex += 3;
	}

	// Move to the fixed point
	// RobotFixPoint  RobotFixVector
	// selIK = 0 : left foot is the supporter
    // selIK = 1 : right foot is the supporter

	// 平移所有機器人軸的位置到世界中正常位置
	if (FirstFKFound == true) // 防止初始化時取錯值
	{
		//if (selIK == 0)
		//{
		//	RobotFixPoint[0] = CrdAll->data[18];
		//	RobotFixPoint[1] = CrdAll->data[19];
		//	RobotFixPoint[2] = CrdAll->data[20];
		//}
		//else if (selIK == 1 || selIK == 2)
		//{
		//	RobotFixPoint[0] = CrdAll->data[57];
		//	RobotFixPoint[1] = CrdAll->data[58];
		//	RobotFixPoint[2] = CrdAll->data[59];
		//}

		WriteIndex = 0;
		if (selIK == LeftSupport)
		{
			RobotFixVector[0] = shiftLL[0]-CrdAll->data[21];
			RobotFixVector[1] = shiftLL[1]-CrdAll->data[22];
			RobotFixVector[2] = shiftLL[2]-CrdAll->data[23];
		}
		else if (selIK == RightSupport || selIK == DoubleSupport)
		{
			RobotFixVector[0] = shiftRL[0]-CrdAll->data[60];
			RobotFixVector[1] = shiftRL[1]-CrdAll->data[61];
			RobotFixVector[2] = shiftRL[2]-CrdAll->data[62];
		}

		for (int i = 0 ; i < LegDHLen*2+ArmDHLen*2; i++)
		{
			CrdAll->data[WriteIndex] += RobotFixVector[0];
			CrdAll->data[WriteIndex+1] += RobotFixVector[1];
			CrdAll->data[WriteIndex+2] += RobotFixVector[2];
			WriteIndex += 3;
		}

	}
	else
	{			
		WriteIndex = 0;
		if (OfflineTraj)
		{
			RobotFixVector[0] = 0-CrdAll->data[60];
			RobotFixVector[1] = -0.5*(CrdAll->data[13]-CrdAll->data[52])-CrdAll->data[61];
			RobotFixVector[2] = 0-(-720);//CrdAll->data[62];	

			for (int i = 0 ; i < LegDHLen*2+ArmDHLen*2; i++)
			{
				CrdAll->data[WriteIndex] += RobotFixVector[0];
				CrdAll->data[WriteIndex+1] += RobotFixVector[1];
				CrdAll->data[WriteIndex+2] += RobotFixVector[2];
				WriteIndex += 3;
			}
		} 
		else
		{
			FirstFKFound = true;
			if (selIK == LeftSupport)
			{
				RobotFixVector[0] = RobotFixPoint[0]-CrdAll->data[18];
				RobotFixVector[1] = RobotFixPoint[1]-CrdAll->data[19];
				RobotFixVector[2] = RobotFixPoint[2]-CrdAll->data[20];
			}
			else if (selIK == RightSupport || selIK == DoubleSupport)
			{
				RobotFixVector[0] = RobotFixPoint[0]-CrdAll->data[51];
				RobotFixVector[1] = RobotFixPoint[1]-CrdAll->data[52];
				RobotFixVector[2] = RobotFixPoint[2]-CrdAll->data[53];
			}

			for (int i = 0 ; i < LegDHLen*2+ArmDHLen*2; i++)
			{
				CrdAll->data[WriteIndex] += RobotFixVector[0];
				CrdAll->data[WriteIndex+1] += RobotFixVector[1];
				CrdAll->data[WriteIndex+2] += RobotFixVector[2];
				WriteIndex += 3;
			}

			remLL[0] = CrdAll->data[18];
			remLL[1] = CrdAll->data[19];
			remLL[2] = CrdAll->data[20];
			shiftLL[0] = CrdAll->data[21];
			shiftLL[1] = CrdAll->data[22];
			shiftLL[2] = CrdAll->data[23];

			remRL[0] = CrdAll->data[57];
			remRL[1] = CrdAll->data[58];
			remRL[2] = CrdAll->data[59];
			shiftRL[0] = CrdAll->data[60];
			shiftRL[1] = CrdAll->data[61];
			shiftRL[2] = CrdAll->data[62];

			//泓逸start1112
			remLA[0] = CrdAll->data[105];
			remLA[1] = CrdAll->data[106];
			remLA[2] = CrdAll->data[107];
			//左手第十軸 
			shiftLA[0] = CrdAll->data[105];
			shiftLA[1] = CrdAll->data[106];
			shiftLA[2] = CrdAll->data[107];
			//左手第十軸 

			remRA[0] = CrdAll->data[135];
			remRA[1] = CrdAll->data[136];
			remRA[2] = CrdAll->data[137];
			//右手第九軸
			shiftRA[0] = CrdAll->data[135];
			shiftRA[1] = CrdAll->data[136];
			shiftRA[2] = CrdAll->data[137];
			//右手第十軸
			//泓逸end111227
		}
	}


	// 平移後機器人原點位置
	DHOrigin[0] = RobotFixVector[0];
	DHOrigin[1] = RobotFixVector[1];
	DHOrigin[2] = RobotFixVector[2];

	// 取出所有Z軸方向 以供之後利用
	WriteIndex = 0;
	for (int i = 0 ; i < LegDHLen; i++)
	{
		ZAxisAll->data[WriteIndex] = FKLLeg->Rn[i].data[2];
		ZAxisAll->data[WriteIndex+1] = FKLLeg->Rn[i].data[6];
		ZAxisAll->data[WriteIndex+2] = FKLLeg->Rn[i].data[10];
		WriteIndex += 3;
	}
	for (int i = 0 ; i < LegDHLen; i++)
	{
		ZAxisAll->data[WriteIndex] = FKRLeg->Rn[i].data[2];
		ZAxisAll->data[WriteIndex+1] = FKRLeg->Rn[i].data[6];
		ZAxisAll->data[WriteIndex+2] = FKRLeg->Rn[i].data[10];
		WriteIndex += 3;
	}
	for (int i = 0 ; i < ArmDHLen; i++)
	{
		ZAxisAll->data[WriteIndex] = FKLArm->Rn[i].data[2];
		ZAxisAll->data[WriteIndex+1] = FKLArm->Rn[i].data[6];
		ZAxisAll->data[WriteIndex+2] = FKLArm->Rn[i].data[10];
		WriteIndex += 3;
	}
	for (int i = 0 ; i < ArmDHLen; i++)
	{
		ZAxisAll->data[WriteIndex] = FKRArm->Rn[i].data[2];
		ZAxisAll->data[WriteIndex+1] = FKRArm->Rn[i].data[6];
		ZAxisAll->data[WriteIndex+2] = FKRArm->Rn[i].data[10];
		WriteIndex += 3;
	}

	UpdateDrawingBuffer(); // refresh the drawing buffer
}

void Kine::CCDInit(void)
{
	QFKLLeg =  new QFwdKine();

	for (int i=0;i<6;i++)
		QFKLLeg->Qth[i]=0;
	for (int i=0;i<6;i++)
		QFKLLeg->QthHome[i]=0;
	//QFKLLeg->QthHome[4]=PI/6;
	QFKLLeg->QthHome[1]=PI/2;	


	QFKRLeg =  new QFwdKine();

	for (int i=0;i<6;i++)
		QFKRLeg->Qth[i]=0;
	for (int i=0;i<6;i++)
		QFKRLeg->QthHome[i]=0;
	//QFKRLeg->QthHome[4]=PI/6;
	QFKRLeg->QthHome[1]=PI/2;	

	k1=0;
	k2=0;
	k3=0; 
	K1=&k1;
	K2=&k2;
	K3=&k3;

	for (int i=0;i<6;i++)
		CCDWei[i]=1.9;//1.63;
	CCDWei[5]=1;
	CCDWei[4]=1;
	CCDWei[0]=1;
}

void Kine::InitQKineTrains(void)
{
		fstream LLeg_QParameter;
			double PatameterTemp[36];
			LLeg_QParameter.open("LLegQ.txt",ios::in);
			for(int i=0;i<36;i++)
				LLeg_QParameter>>PatameterTemp[i] ;
			LLeg_QParameter.close();

		for(int i=0;i<6;i++) // QLLRotate ini : the rotation in the parent coordinate
		{
			QFKLLeg->QRotate[i]->SetAxis(PatameterTemp[3*i+18],PatameterTemp[3*i+1+18],PatameterTemp[3*i+2+18]);
			QFKLLeg->QRotate[i]->SetAngle(0);
		}

		for(int i=0;i<6;i++) // QLLJointPos ini : the initial position in the world
			QFKLLeg->QJointPos[i]->SetData(0,PatameterTemp[3*i],PatameterTemp[3*i+1],PatameterTemp[3*i+2]);

		for(int i=0;i<6;i++) // QLLJointZaxis ini :the initial axis position in the parent coordinate
			QFKLLeg->QJointZaxis[i]->SetData(0,PatameterTemp[3*i+18],PatameterTemp[3*i+1+18],PatameterTemp[3*i+2+18]);

			////Check Point
			//for(int i=0;i<6;i++) 
			//	QFKLLeg->QRotate[i]->ShowQuaternion();
			//for(int i=0;i<6;i++) 
			//	QFKLLeg->QJointPos[i]->ShowQuaternion();
			//for(int i=0;i<6;i++) 
			//	QFKLLeg->QJointZaxis[i]->ShowQuaternion();
		FindSwingQFK(QFKLLeg,1);
		//for(int i=0;i<9;i++) 
		//	TarLLRot[i]=LLRot[i];
		QFKLLeg->TarRot[0]=1; QFKLLeg->TarRot[1]=0; QFKLLeg->TarRot[2]=0; 
		QFKLLeg->TarRot[3]=0; QFKLLeg->TarRot[4]=0; QFKLLeg->TarRot[5]=1;
		QFKLLeg->TarRot[6]=0; QFKLLeg->TarRot[7]=-1; QFKLLeg->TarRot[8]=0;

		//QFKLLeg->TarRot[0]=0.866; QFKLLeg->TarRot[1]=0.5; QFKLLeg->TarRot[2]=0; 
		//QFKLLeg->TarRot[3]=0; QFKLLeg->TarRot[4]=0; QFKLLeg->TarRot[5]=1;
		//QFKLLeg->TarRot[6]=0.5; QFKLLeg->TarRot[7]=-0.866; QFKLLeg->TarRot[8]=0;

////////////////////////////////////////////////////////////////////////////////////////////////
		fstream RLeg_QParameter;
			//double PatameterTemp[36];
			RLeg_QParameter.open("RLegQ.txt",ios::in);
			for(int i=0;i<36;i++)
				RLeg_QParameter>>PatameterTemp[i] ;
			RLeg_QParameter.close();

		for(int i=0;i<6;i++) // QLLRotate ini : the rotation in the parent coordinate
		{
			QFKRLeg->QRotate[i]->SetAxis(PatameterTemp[3*i+18],PatameterTemp[3*i+1+18],PatameterTemp[3*i+2+18]);
			QFKRLeg->QRotate[i]->SetAngle(0);
		}

		for(int i=0;i<6;i++) // QLLJointPos ini : the initial position in the world
			QFKRLeg->QJointPos[i]->SetData(0,PatameterTemp[3*i],PatameterTemp[3*i+1],PatameterTemp[3*i+2]);

		for(int i=0;i<6;i++) // QLLJointZaxis ini :the initial axis position in the parent coordinate
			QFKRLeg->QJointZaxis[i]->SetData(0,PatameterTemp[3*i+18],PatameterTemp[3*i+1+18],PatameterTemp[3*i+2+18]);

			////Check Point
			//for(int i=0;i<6;i++) 
			//	QFKRLeg->QRotate[i]->ShowQuaternion();
			//for(int i=0;i<6;i++) 
			//	QFKRLeg->QJointPos[i]->ShowQuaternion();
			//for(int i=0;i<6;i++) 
			//	QFKRLeg->QJointZaxis[i]->ShowQuaternion();
		FindSwingQFK(QFKRLeg,0);

		//for(int i=0;i<9;i++) 
		//	TarLLRot[i]=LLRot[i];
		QFKRLeg->TarRot[0]=1; QFKRLeg->TarRot[1]=0; QFKRLeg->TarRot[2]=0; 
		QFKRLeg->TarRot[3]=0; QFKRLeg->TarRot[4]=0; QFKRLeg->TarRot[5]=1;
		QFKRLeg->TarRot[6]=0; QFKRLeg->TarRot[7]=-1; QFKRLeg->TarRot[8]=0;
}
inline void Kine::FindSwingQFK(QFwdKine* QLeg, bool LLeg)
{
	// th update
	//for(int i=0;i<6;i++)
	//	QLLth[i]=FKLLeg->theta[i+1];
	// th update
	int IndexShift=0;
	if (LLeg==0)
		IndexShift=3*13;
	else
		IndexShift=0;


	double ShiftBody[3];
	for(int i=0;i<3;i++)
	{
		ShiftBody[i]=CrdAll->data[i+6+IndexShift]-QLeg->QJointPos[0]->data[i+1];
		QLeg->QJointPos[0]->data[i+1]+=ShiftBody[i];
	}
	
	for(int i=0;i<6;i++) // Set th
		QLeg->QRotate[i]->SetAngle(QLeg->Qth[i]+QLeg->QthHome[i]);

	QuatMul(QLeg->QRotate[0],QLeg->QRotate[1],QLeg->Q1n[0]); // QLLQ1n update
	for(int i=1;i<5;i++) 
		QuatMul(QLeg->Q1n[i-1],QLeg->QRotate[i+1],QLeg->Q1n[i]);
	//Check Point
		//for(int i=0;i<5;i++) 
		//	QLeg->Q1n[i]->ShowQuaternion();

	QuatMulPureQ(QLeg->QRotate[0],QLeg->QJointZaxis[0],QLeg->QZaxis[0]); // Zaxis update
	QuatMulConjQ(QLeg->QZaxis[0],QLeg->QRotate[0],QLeg->QZaxis[0]);
	for(int i=0;i<5;i++) 
	{
		QuatMulPureQ(QLeg->Q1n[i],QLeg->QJointZaxis[i+1],QLeg->QZaxis[i+1]);
		QuatMulConjQ(QLeg->QZaxis[i+1],QLeg->Q1n[i],QLeg->QZaxis[i+1]);
	}		
	////Check Point
		//for(int i=0;i<6;i++) 
		//	QLeg->QZaxis[i]->ShowQuaternion();

	QuatMulPureQ(QLeg->QRotate[0],QLeg->QJointPos[1],QLeg->QEndeffector[1]); // Zaxis update
	QuatMulConjQ(QLeg->QEndeffector[1],QLeg->QRotate[0],QLeg->QEndeffector[1]);
	QuatAdd(QLeg->QEndeffector[1],QLeg->QJointPos[0],QLeg->QEndeffector[1]);

	QuatAdd(QLeg->QEndeffector[1],QLeg->QJointPos[1],QLeg->QEndeffector[0]);
	for(int i=0;i<4;i++) 
	{
		QuatMulPureQ(QLeg->Q1n[i],QLeg->QJointPos[i+2],QLeg->QEndeffector[i+2]); //!!!!!!!!!!!!!!!!!!!!!
		QuatMulConjQ(QLeg->QEndeffector[i+2],QLeg->Q1n[i],QLeg->QEndeffector[i+2]);
		QuatAdd(QLeg->QEndeffector[i+2],QLeg->QEndeffector[i+1],QLeg->QEndeffector[i+2]);
	}	
	////Check Point
		//for(int i=0;i<6;i++) 
		//	QLeg->QEndeffector[i]->ShowQuaternion();

	// end effector rotation update
	QuatToMatT(QLeg->Q1n[4],QLeg->Rot);
	//for(int i=0;i<9;i++)
	//	cout<<QLeg->Rot[i]<<" "<<endl;
	//	cout<<endl;
	////Check Point
			//QLeg->Q1n[4]->ShowQuaternion();

	// end effector rotation update
}

inline void Kine::FindFixQFK(QFwdKine* QLeg, bool LLeg)
{
	// th update
	//for(int i=0;i<6;i++)
	//	QLLth[i]=FKLLeg->theta[i+1];
	// th update
	int IndexShift=0;
	if (LLeg==0)
		IndexShift=3*13;
	else
		IndexShift=0;


	//double ShiftBody[3];
	//for(int i=0;i<3;i++)
	//{
	//	ShiftBody[i]=CrdAll->data[i+6+IndexShift]-QLeg->QJointPos[0]->data[i+1];
	//	QLeg->QJointPos[0]->data[i+1]+=ShiftBody[i];
	//}
	
	for(int i=0;i<6;i++) // Set th
		QLeg->QRotate[i]->SetAngle(QLeg->Qth[i]+QLeg->QthHome[i]);

	QuatMul(QLeg->QRotate[0],QLeg->QRotate[1],QLeg->Q1n[0]); // QLLQ1n update
	for(int i=1;i<5;i++) 
		QuatMul(QLeg->Q1n[i-1],QLeg->QRotate[i+1],QLeg->Q1n[i]);
	//Check Point
		//for(int i=0;i<5;i++) 
		//	QLeg->Q1n[i]->ShowQuaternion();

	QuatMulPureQ(QLeg->QRotate[0],QLeg->QJointZaxis[0],QLeg->QZaxis[0]); // Zaxis update
	QuatMulConjQ(QLeg->QZaxis[0],QLeg->QRotate[0],QLeg->QZaxis[0]);
	for(int i=0;i<5;i++) 
	{
		QuatMulPureQ(QLeg->Q1n[i],QLeg->QJointZaxis[i+1],QLeg->QZaxis[i+1]);
		QuatMulConjQ(QLeg->QZaxis[i+1],QLeg->Q1n[i],QLeg->QZaxis[i+1]);
	}		
	////Check Point
	//	for(int i=0;i<6;i++) 
	//		QLeg->QZaxis[i]->ShowQuaternion();

	QuatMulPureQ(QLeg->QRotate[0],QLeg->QJointPos[1],QLeg->QEndeffector[1]); // Zaxis update
	QuatMulConjQ(QLeg->QEndeffector[1],QLeg->QRotate[0],QLeg->QEndeffector[1]);
	QuatAdd(QLeg->QEndeffector[1],QLeg->QJointPos[0],QLeg->QEndeffector[1]);

	QuatAdd(QLeg->QEndeffector[1],QLeg->QJointPos[1],QLeg->QEndeffector[0]);
	for(int i=0;i<4;i++) 
	{
		QuatMulPureQ(QLeg->Q1n[i],QLeg->QJointPos[i+2],QLeg->QEndeffector[i+2]); //!!!!!!!!!!!!!!!!!!!!!
		QuatMulConjQ(QLeg->QEndeffector[i+2],QLeg->Q1n[i],QLeg->QEndeffector[i+2]);
		QuatAdd(QLeg->QEndeffector[i+2],QLeg->QEndeffector[i+1],QLeg->QEndeffector[i+2]);
	}	

	double ShiftBody[3];
	for(int i=0;i<3;i++)
		ShiftBody[i]=CrdAll->data[i+12+IndexShift]-QLeg->QEndeffector[5]->data[i+1];
	for(int j=0;j<6;j++)		
		for(int i=0;i<3;i++)
			QLeg->QEndeffector[j]->data[i+1]+=ShiftBody[i];

	////Check Point
		//for(int i=0;i<6;i++) 
		//	QLeg->QEndeffector[i]->ShowQuaternion();

	// end effector rotation update
	QuatToMatT(QLeg->Q1n[4],QLeg->Rot);
	////Check Point
			//QLeg->Q1n[4]->ShowQuaternion();

	// end effector rotation update
}

//void Kine::CCDIKSolve(void)//double* tBody ,double* tSwing, double* tRSwing, int* status
void Kine::CCDIKSolve(QFwdKine* QLeg,FwdKine* FLeg,double* tSwing, double* tRSwing,bool LLeg, int* status)//
{
	////Check Point
	//	for(int i=0;i<6;i++) 
	//		QLeg->QEndeffector[i]->ShowQuaternion();
	double TarSwing[3];
	TarSwing[0] =tSwing[0];//
	TarSwing[1] =tSwing[1];//
	TarSwing[2] =tSwing[2];//
	
	CCDcntIK=0;
	double VecTemp1[3];
	double VecTemp2[3];

	while(1)
	{
		for(int i=5;i>=0;i--) 
		{
			if (i==5||i==4||i==0)
			{
				*K1=0;
				*K2=0;
				*K3=0;
				for (int j=0;j<3;j++)
				{
					MatMulAB(QLeg->Rot+3*j,1,3,QLeg->QZaxis[i]->data+1,3,1,VecTemp1);
					MatMulAB(QLeg->TarRot+3*j,1,3,QLeg->QZaxis[i]->data+1,3,1,VecTemp2);	
					*K1+=*K1+VecTemp1[0]*VecTemp2[0];
					MatMulAB(QLeg->Rot+3*j,1,3,QLeg->TarRot+3*j,3,1,VecTemp1);
					*K2+=*K2+VecTemp1[0];
					Cross2Vec(QLeg->Rot+3*j,QLeg->TarRot+3*j,VecTemp1);
					MatMulAB(QLeg->QZaxis[i]->data+1,1,3,VecTemp1,3,1,VecTemp2);
					*K3+=*K3+VecTemp2[0];
				}

				if ((*K2-*K1)!=0)
				QCCDdth[i]=atan(*K3/(*K2-*K1))  * CCDWei[i] ;
				else
				QCCDdth[i]=0;

			}
			//else if (i==0)
			//{
			//	*K1=0;
			//	*K2=0;
			//	*K3=0;
			//	for (int j=0;j<3;j++)
			//	{
			//		MatMulAB(QLeg->Rot+3*j,1,3,QLeg->QZaxis[i]->data+1,3,1,VecTemp1);
			//		MatMulAB(QLeg->TarRot+3*j,1,3,QLeg->QZaxis[i]->data+1,3,1,VecTemp2);	
			//		*K1+=*K1+VecTemp1[0]*VecTemp2[0];
			//		MatMulAB(QLeg->Rot+3*j,1,3,QLeg->TarRot+3*j,3,1,VecTemp1);
			//		*K2+=*K2+VecTemp1[0];
			//		Cross2Vec(QLeg->Rot+3*j,QLeg->TarRot+3*j,VecTemp1);
			//		MatMulAB(QLeg->QZaxis[i]->data+1,1,3,VecTemp1,3,1,VecTemp2);
			//		*K3+=*K3+VecTemp2[0];
			//	}

			//	if ((*K2-*K1)!=0)
			//	QCCDdth[i]=atan(*K3/(*K2-*K1))  * CCDWei[i] ;
			//	else
			//	QCCDdth[i]=0;

			//}
			else  /// Posi Error : 
			{
				*K1=0;
				*K2=0;
				*K3=0;

				for (int j=0;j<3;j++) 
					Pid[j]=TarSwing[j]-QLeg->QEndeffector[i]->data[j+1];
				for (int j=0;j<3;j++) 
					Pic[j]=QLeg->QEndeffector[5]->data[j+1]-QLeg->QEndeffector[i]->data[j+1];	

				//K1=
				MatMulAB(Pid,1,3,QLeg->QZaxis[i]->data+1,3,1,VecTemp1);
				MatMulAB(Pic,1,3,QLeg->QZaxis[i]->data+1,3,1,VecTemp2);
				*K1= VecTemp1[0]*VecTemp2[0];
				//K2=
				MatMulAB(Pid,1,3,Pic,3,1,K2);
				//K3=
				Cross2Vec(Pic,Pid,VecTemp1);
				MatMulAB(QLeg->QZaxis[i]->data+1,1,3,VecTemp1,3,1,K3);

				//cout<<"K1="<<*K1<<endl;
				//cout<<"K2="<<*K2<<endl;
				//cout<<"K3="<<*K3<<endl;
				if ((*K2-*K1)!=0)
				QCCDdth[i]=atan(*K3/(*K2-*K1))  * CCDWei[i] ;
				else
				QCCDdth[i]=0;
				//cout<<"dth = "<< QCCDdth[i]<<endl;
			}	
			QLeg->Qth[i]+=QCCDdth[i];
			FindSwingQFK(QLeg,LLeg);
		} //For

		//Norm Error
			for (int i=0;i<3;i++) 
				CCDErr[i]=TarSwing[i]-QLeg->QEndeffector[5]->data[i+1];	

		 MatMiuAB(QLeg-> Rot,QLeg->TarRot,  DiffRotMatSw,9);
		temp_vec2[0] = DiffRotMatSw[0];
		temp_vec2[1] = DiffRotMatSw[1];
		temp_vec2[2] = DiffRotMatSw[2];
		//temp_vec2[0] = DiffRotMatSw[3];
		//temp_vec2[1] =  DiffRotMatSw[4];
		//temp_vec2[2] = DiffRotMatSw[5];
		//temp_vec2[0] = DiffRotMatSw[6];
		//temp_vec2[1] = DiffRotMatSw[7];
		//temp_vec2[2] = DiffRotMatSw[8];

		//NormXYZD(temp_vec2)
		
		//Norm Error

		//abs(res-AnklePitchRef[gIthIK%gStepSample])
		//if (QLeg->Rot[0]>0.99999 && abs(QLeg->Rot[1])<0.00001 && QLeg->Rot[5]>0.99999 && abs(QLeg->Rot[4])<0.00001 && sqrt(CCDErr[0]*CCDErr[0]+CCDErr[1]*CCDErr[1]+CCDErr[2]*CCDErr[2])< 0.1)// 
		if (0< AngleErrLim && sqrt(CCDErr[0]*CCDErr[0]+CCDErr[1]*CCDErr[1]+CCDErr[2]*CCDErr[2])< SwErrLim*ErrorRatio)// 0.05 for Switching Scheme
		{
			//cout<<"Succeed"<<"\n";			
			//cout<<"Iteration Number = "<<CCDcntIK<<"\n";



			for(int i=0;i<6;i++) 
				FLeg->theta[i+1]=QLeg->Qth[i];



			CCDcntIK=0;
			//for(int i=0;i<9;i++) 
			//cout<<QLeg->Rot[i]<<"\t";
			//cout<<endl;
			//cout<<endl;
			//for(int i=0;i<9;i++) 
			//cout<<tRSwing[i]<<"\t";
			//cout<<endl;
			//for(int i=0;i<6;i++) 
			//cout<<QLeg->Qth[i]<<endl;
			//cout<<res-AnklePitchRef[gIthIK%gStepSample]<<" QQ3"<<endl;

			break;		
		}
		else if (CCDcntIK==10000)
		{
			cout<<"Failed"<<"\n";
			//Check Point
				for(int i=0;i<6;i++) 
					QLeg->QEndeffector[i]->ShowQuaternion();
			//for(int i=0;i<9;i++) 
			//cout<<LLRot[i]<<"\t";
			//cout<<endl;
			double Long[3];
			for(int i=0;i<3;i++) 
			Long[i]=TarSwing[i]-QLeg->QEndeffector[1]->data[i+1];

			cout<<sqrt(Long[0]*Long[0]+Long[1]*Long[1]+Long[2]*Long[2])<<endl;

			//cout<<NormXYZD(temp_vec2);
			//for(int i=0;i<3;i++) 
			//cout<<tSwing[i]<<"\t";
			//cout<<endl;
			//cout<<res<<" QQ1"<<endl;
			//cout<<AnklePitchRef[gIthIK%gStepSample]<<" QQ2"<<endl;
			//cout<<res-AnklePitchRef[gIthIK%gStepSample]<<" QQ3"<<endl;
			//for(int i=0;i<9;i++) 
			//cout<< QLeg->Rot[i]<<" "<<endl;
			//cout<< endl;
			//for(int i=0;i<9;i++) 
			//cout<< QLeg->TarRot[i]<<" "<<endl;
			//cout<< endl;

			system("pause");
			break;
		}
		CCDcntIK++;
	} //While
}

void Kine::CCDCOGSolve(QFwdKine* QLeg,FwdKine* FLeg,double* tCOG, double* tRFixd,bool LLeg, int* status)//
{
	////Check Point
	//	for(int i=0;i<6;i++) 
	//		QLeg->QEndeffector[i]->ShowQuaternion();
	double TarCOG[3];
	TarCOG[0] =tCOG[0];//
	TarCOG[1] =tCOG[1];//
	TarCOG[2] =tCOG[2];//
	
	double TarCOGErr[3];
	//TarCOGErr[0] =tCOG[0]-COG[0];//
	//TarCOGErr[1] =tCOG[1]-COG[1];//
	//TarCOGErr[2] =tCOG[2]-COG[2];//

	//cout<<TarCOGErr[0]<<"  "<<TarCOGErr[1]<<"  "<<TarCOGErr[2]<<"  \n";

	CCDcntIK=0;
	double VecTemp1[3];
	double VecTemp2[3];

	while(1)
	{
		for(int i=0;i<6;i++) 
		{
			if (i==1||i==2||i==0)
			{
				if (selIK ==1 ||selIK == 2)
				{
					*K1=0;
					*K2=0;
					*K3=0;
					for (int j=0;j<3;j++)
					{
						MatMulAB(BodyRotM+3*j,1,3, ZAxisAll->data+39+3*i,3,1,VecTemp1);
						MatMulAB(tRFixd+3*j,1,3,ZAxisAll->data+39+3*i,3,1,VecTemp2);	
						*K1+=*K1+VecTemp1[0]*VecTemp2[0];
						MatMulAB(BodyRotM+3*j,1,3,tRFixd+3*j,3,1,VecTemp1);
						*K2+=*K2+VecTemp1[0];
						Cross2Vec(BodyRotM+3*j,tRFixd+3*j,VecTemp1);
						MatMulAB(ZAxisAll->data+39+3*i,1,3,VecTemp1,3,1,VecTemp2);
						*K3+=*K3+VecTemp2[0];
					}

					if ((*K2-*K1)!=0)
					QCCDdth[i]=atan(*K3/(*K2-*K1))  * 1;//CCDWei[i] ;
					else
					QCCDdth[i]=0;
				}
				else
				{
					*K1=0;
					*K2=0;
					*K3=0;
					for (int j=0;j<3;j++)
					{
						MatMulAB(BodyRotM+3*j,1,3, ZAxisAll->data+3*i,3,1,VecTemp1);
						MatMulAB(tRFixd+3*j,1,3,ZAxisAll->data+3*i,3,1,VecTemp2);	
						*K1+=*K1+VecTemp1[0]*VecTemp2[0];
						MatMulAB(BodyRotM+3*j,1,3,tRFixd+3*j,3,1,VecTemp1);
						*K2+=*K2+VecTemp1[0];
						Cross2Vec(BodyRotM+3*j,tRFixd+3*j,VecTemp1);
						MatMulAB(ZAxisAll->data+3*i,1,3,VecTemp1,3,1,VecTemp2);
						*K3+=*K3+VecTemp2[0];
					}

					if ((*K2-*K1)!=0)
					QCCDdth[i]=atan(*K3/(*K2-*K1))  * 1;//CCDWei[i] ;
					else
					QCCDdth[i]=0;				
				}
			}
			//else if (i==0)
			//{
			//	*K1=0;
			//	*K2=0;
			//	*K3=0;
			//	for (int j=0;j<3;j++)
			//	{
			//		MatMulAB(QLeg->Rot+3*j,1,3,ZAxisAll->data+39+i*3+j,3,1,VecTemp1);
			//		MatMulAB(QLeg->TarRot+3*j,1,3,ZAxisAll->data+39+i*3+j,3,1,VecTemp2);	
			//		*K1+=*K1+VecTemp1[0]*VecTemp2[0];
			//		MatMulAB(QLeg->Rot+3*j,1,3,QLeg->TarRot+3*j,3,1,VecTemp1);
			//		*K2+=*K2+VecTemp1[0];
			//		Cross2Vec(QLeg->Rot+3*j,QLeg->TarRot+3*j,VecTemp1);
			//		MatMulAB(QLeg->QZaxis[i]->data+1,1,3,VecTemp1,3,1,VecTemp2);
			//		*K3+=*K3+VecTemp2[0];
			//	}

			//	if ((*K2-*K1)!=0)
			//	QCCDdth[i]=atan(*K3/(*K2-*K1)) * 1;//CCDWei[i] ;
			//	else
			//	QCCDdth[i]=0;

			//}
			else  /// Posi Error : 
			{
				if (selIK ==1 ||selIK == 2)
				{
					*K1=0;
					*K2=0;
					*K3=0;

					for (int j=0;j<3;j++) 
						Pid[j]=TarCOG[j]-CrdAll->data[39+i*3+j];
					for (int j=0;j<3;j++) 
						Pic[j]=COG[j]   -CrdAll->data[39+i*3+j];	

					//K1=
					MatMulAB(Pid,1,3,ZAxisAll->data+39+i*3,3,1,VecTemp1);
					MatMulAB(Pic,1,3,ZAxisAll->data+39+i*3,3,1,VecTemp2);
					*K1= VecTemp1[0]*VecTemp2[0];
					//K2=
					MatMulAB(Pid,1,3,Pic,3,1,K2);
					//K3=
					Cross2Vec(Pic,Pid,VecTemp1);
					MatMulAB(ZAxisAll->data+39+i*3,1,3,VecTemp1,3,1,K3);

				//cout<<"K1="<<*K1<<endl;
				//cout<<"K2="<<*K2<<endl;
				//cout<<"K3="<<*K3<<endl;
				if ((*K2-*K1)!=0)
				QCCDdth[i]=atan(*K3/(*K2-*K1))  *(-1);// CCDWei[i] *-1 ;
				else
				QCCDdth[i]=0;
				//cout<<"dth = "<< QCCDdth[i]<<endl;
				}
				else
				{
					*K1=0;
					*K2=0;
					*K3=0;

					for (int j=0;j<3;j++) 
						Pid[j]=TarCOG[j]-CrdAll->data[i*3+j];
					for (int j=0;j<3;j++) 
						Pic[j]=COG[j]   -CrdAll->data[i*3+j];	

					//K1=
					MatMulAB(Pid,1,3,ZAxisAll->data+i*3,3,1,VecTemp1);
					MatMulAB(Pic,1,3,ZAxisAll->data+i*3,3,1,VecTemp2);
					*K1= VecTemp1[0]*VecTemp2[0];
					//K2=
					MatMulAB(Pid,1,3,Pic,3,1,K2);
					//K3=
					Cross2Vec(Pic,Pid,VecTemp1);
					MatMulAB(ZAxisAll->data+i*3,1,3,VecTemp1,3,1,K3);

				//cout<<"K1="<<*K1<<endl;
				//cout<<"K2="<<*K2<<endl;
				//cout<<"K3="<<*K3<<endl;
				if ((*K2-*K1)!=0)
				QCCDdth[i]=atan(*K3/(*K2-*K1))  *(-1);// CCDWei[i] *-1 ;
				else
				QCCDdth[i]=0;
				//cout<<"dth = "<< QCCDdth[i]<<endl;				
				}
			}	
			//QLeg->Qth[i]+=QCCDdth[i];
			//FindFixQFK(QLeg,LLeg);
			FLeg->theta[i+1]+=QCCDdth[i];
			FindFK();
			FindCOG();
			GetLegsCoords();   

		} //For

		//Norm Error
			for (int i=0;i<3;i++) 
				CCDErr[i]=TarCOG[i]-COG[i];	

		 MatMiuAB(QLeg-> Rot,QLeg->TarRot,  DiffRotMatSw,9);
		temp_vec2[0] = DiffRotMatSw[0];
		temp_vec2[1] = DiffRotMatSw[1];
		temp_vec2[2] = DiffRotMatSw[2];
		//temp_vec2[0] = DiffRotMatSw[3];
		//temp_vec2[1] =  DiffRotMatSw[4];
		//temp_vec2[2] = DiffRotMatSw[5];
		//temp_vec2[0] = DiffRotMatSw[6];
		//temp_vec2[1] = DiffRotMatSw[7];
		//temp_vec2[2] = DiffRotMatSw[8];

		//NormXYZD(temp_vec2)

		//Norm Error
		//if (QLeg->Rot[0]>0.99999 && abs(QLeg->Rot[1])<0.00001 && QLeg->Rot[5]>0.99999 && abs(QLeg->Rot[4])<0.00001 && sqrt(CCDErr[0]*CCDErr[0]+CCDErr[1]*CCDErr[1]+CCDErr[2]*CCDErr[2])< 0.1)// 
		if (0 < AngleErrLim && sqrt(CCDErr[0]*CCDErr[0]+CCDErr[1]*CCDErr[1]+CCDErr[2]*CCDErr[2])< 0.01)// 
		{
			//cout<<"COG Succeed"<<"\n";			
			//cout<<"Iteration Number = "<<CCDcntIK<<"\n";
			//for(int i=0;i<6;i++) 
			//	FLeg->theta[i+1]=QLeg->Qth[i];

			CCDcntIK=0;
			//for(int i=0;i<9;i++) 
			//cout<<QLeg->Rot[i]<<"\t";
			//cout<<endl;
			//cout<<endl;
			//for(int i=0;i<9;i++) 
			//cout<<tRSwing[i]<<"\t";
			//cout<<endl;
			//for(int i=0;i<6;i++) 
			//cout<<QLeg->Qth[i]<<endl;
			//cout<<QLeg->Qth[4];

			break;		
		}
		else if (CCDcntIK==1000)
		{
			cout<<"COG Failed"<<"\n";

				TarCOGErr[0] =tCOG[0]-COG[0];//
				TarCOGErr[1] =tCOG[1]-COG[1];//
				TarCOGErr[2] =tCOG[2]-COG[2];//

				cout<<TarCOGErr[0]<<"  "<<TarCOGErr[1]<<"  "<<TarCOGErr[2]<<"  \n";
			//Check Point
				//for(int i=0;i<6;i++) 
				//	QLeg->QEndeffector[i]->ShowQuaternion();
				//cout<<"QQ";
				//for(int i=0;i<6;i++) 
				//{
				//	for(int j=0;j<3;j++) 
				//	cout<<CrdAll->data[39+i*3+j]<<"  ";
				//	cout<<endl;
				//}
				
			//for(int i=0;i<9;i++) 
			//cout<<LLRot[i]<<"\t";
			//cout<<endl;
			double Long[3];
			for(int i=0;i<3;i++) 
			Long[i]=TarCOG[i]-COG[i];

			cout<<sqrt(Long[0]*Long[0]+Long[1]*Long[1]+Long[2]*Long[2])<<endl;

			cout<<NormXYZD(temp_vec2);
			//for(int i=0;i<3;i++) 
			//cout<<tSwing[i]<<"\t";
			//cout<<endl;
			system("pause");
			break;
		}
		CCDcntIK++;
	} //While
}


void Kine::ComputeJacobians(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 求出機器人整體的Jacobian matrix 
	******************************************************************/
	// dth => [左腳6軸 ; 右腳6軸 ; 左手6軸 ; 右手6軸]
	// selIK = 0 : left foot is the supporter
    // selIK = 1 : right foot is the supporter

	// when selIK = 0,   dx = [sw_x sw_y sw_z sw_tx sw_ty sw_tz fix_tx fix_ty fix_tz COGx COGy COGz ]
	// when selIK = 1,2, dx = [sw_x sw_y sw_z sw_tx sw_ty sw_tz fix_tx fix_ty fix_tz COGx COGy COGz ]
	// 被排成相同的!!
	// when selIK = 0,   dx = [sw_x sw_y sw_z sw_tx sw_ty sw_tz fix_tx fix_ty fix_tz LA RA COGx COGy COGz ]
	// when selIK = 1,2, dx = [sw_x sw_y sw_z sw_tx sw_ty sw_tz fix_tx fix_ty fix_tz LA RA COGx COGy COGz ]

	if (selIK == LeftSupport) // support leg = left
	{
		// 第 4 5 6 rol
		ind_x = JaNCol*3;
		ind_y = ind_x + JaNCol;
		ind_z = ind_y + JaNCol; 
		ind_source = 0;

		// swing theta x y z, 3 rows, 12 cols
		// 填寫角速度 Jacobian  fixed leg to swing leg
		for (int i = 0 ; i < 6 ; i++) // 先跑6次 1~6 column  4~6 row(1)
		{
			Ja->data[ind_x] = -ZAxisAll->data[ind_source];
			Ja->data[ind_y] = -ZAxisAll->data[ind_source+1];
			Ja->data[ind_z] = -ZAxisAll->data[ind_source+2];
			ind_source += 3;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		// 填寫角速度 Jacobian  swing leg to swing leg
		ind_source = 39;
		for (int i = 0 ; i < 6 ; i++) // 再跑6次 7~12 column 4~6 row(2)
		{
			Ja->data[ind_x] = ZAxisAll->data[ind_source];
			Ja->data[ind_y] = ZAxisAll->data[ind_source+1];
			Ja->data[ind_z] = ZAxisAll->data[ind_source+2];
			ind_source += 3;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		// 填寫角速度 Jacobian  left arm to swing leg,right arm to swing leg
		for(int i = 0; i<12 ; i++)// 跑12次 13~24 column 4~6 row(3)
		{
			Ja->data[ind_x] = 0;
			Ja->data[ind_y] = 0;
			Ja->data[ind_z] = 0;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		// 第 7 8 9 rol
		ind_x = JaNCol*6;
		ind_y = ind_x + JaNCol;
		ind_z = ind_y + JaNCol; 

		// fixed theta x y z, 3 rows, 12 cols
		// 填寫角速度 Jacobian  fixed leg to fixed leg
		ind_source = 0;
		for (int i = 0 ; i < 6 ; i++)//1~6 column 7~9 row(4)
		{
			Ja->data[ind_x] = ZAxisAll->data[ind_source];
			Ja->data[ind_y] = ZAxisAll->data[ind_source+1];
			Ja->data[ind_z] = ZAxisAll->data[ind_source+2];
			ind_source += 3;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		// 填寫角速度 Jacobian  swing leg to fixed leg,left arm to fixed leg,right arm to fixed leg(5)
		for (int i = 0 ; i < 18 ; i++)
		{
			Ja->data[ind_x] = 0;
			Ja->data[ind_y] = 0;
			Ja->data[ind_z] = 0;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		// 填寫方向速度 Jacobian 所有軸，對於swing腳的endeffector
		ind_source = 0;
		ind_dest = 0;
		EndEff[0] = CrdAll->data[51];
		EndEff[1] = CrdAll->data[52];
		EndEff[2] = CrdAll->data[53];

		for (int i = 0 ; i <6 ; i++)//fixed腳每一軸到swing腳endefector的距離
		{
			EndEffDiff[ind_dest] =  CrdAll->data[ind_source] - EndEff[0];
			EndEffDiff[ind_dest+1] = CrdAll->data[ind_source+1] - EndEff[1];
			EndEffDiff[ind_dest+2] = CrdAll->data[ind_source+2] - EndEff[2];
			ind_source += 3;
			ind_dest += 3;
		}

		ind_source = 39;
		//int_dest 繼續加算
		for (int i = 6 ; i <12 ; i++)//swing腳每一軸到swing腳endefector的距離
		{
			EndEffDiff[ind_dest] = EndEff[0] - CrdAll->data[ind_source];
			EndEffDiff[ind_dest+1] = EndEff[1] - CrdAll->data[ind_source+1];
			EndEffDiff[ind_dest+2] = EndEff[2] - CrdAll->data[ind_source+2];
			ind_source += 3;
			ind_dest += 3;
		}

		ind_x = 0;
		ind_y = JaNCol;
		ind_z = ind_y + JaNCol; 
		ind_source = 0;
		// 填寫方向速度 Jacobian fixed leg to swing leg
		for (int i = 0 ; i<6 ; i++)//1~6 column 1~3 row(6)
		{
			Cross2Vd(&ZAxisAll->data[ind_source],&EndEffDiff[ind_source],temp_cross);
			Ja->data[ind_x] = temp_cross[0];
			Ja->data[ind_y] = temp_cross[1];
			Ja->data[ind_z] = temp_cross[2];
			ind_source += 3;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		ind_source = 39;
		ind_dest = 18; // 借這個變數來用，ind_dest原本的意思是 要存到的地方之index
		// 填寫方向速度 Jacobian swing leg to swing leg
		for (int i = 6 ; i<12 ; i++)//7~12 column 1~3 row(7)
		{
			Cross2Vd(&ZAxisAll->data[ind_source],&EndEffDiff[ind_dest],temp_cross);
			Ja->data[ind_x] = temp_cross[0];
			Ja->data[ind_y] = temp_cross[1];
			Ja->data[ind_z] = temp_cross[2];
			ind_source += 3;
			ind_dest += 3;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		// 填寫方向速度 Jacobian  left arm to swing leg,right arm to swing leg
		for (int i = 0 ; i < 12 ; i++)//13~24 column 1~3 row(8)
		{
			Ja->data[ind_x] = 0;
			Ja->data[ind_y] = 0;
			Ja->data[ind_z] = 0;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		// 第 13 14 15 rol
		ind_x = JaNCol*12;
		ind_y = ind_x + JaNCol;
		ind_z = ind_y + JaNCol; 
		ind_source = 0;

		// 填寫角速度 Jacobian  fixed leg to left arm
		for (int i = 0 ; i < 6 ; i++) // 先跑6次 1~6 column  13~15 row(9)
		{
			Ja->data[ind_x] = -ZAxisAll->data[ind_source];
			Ja->data[ind_y] = -ZAxisAll->data[ind_source+1];
			Ja->data[ind_z] = -ZAxisAll->data[ind_source+2];
			ind_source += 3;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		// 填寫角速度 Jacobian  swing leg to left arm
		for (int i = 0 ; i < 6 ; i++) // 先跑6次 7~12 column  13~15 row(10)
		{
			Ja->data[ind_x] = 0;
			Ja->data[ind_y] = 0;
			Ja->data[ind_z] = 0;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		ind_source = 87;
		// 填寫角速度 Jacobian  left arm to left arm
		for (int i = 0 ; i < 6 ; i++) // 先跑6次 13~18 column  13~15 row(11)
		{
			Ja->data[ind_x] = ZAxisAll->data[ind_source];
			Ja->data[ind_y] = ZAxisAll->data[ind_source+1];
			Ja->data[ind_z] = ZAxisAll->data[ind_source+2];
			ind_source += 3;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		// 填寫角速度 Jacobian  right arm to left arm
		for (int i = 0 ; i < 6 ; i++) // 先跑6次 19~24 column  13~15 row(12)
		{
			Ja->data[ind_x] = 0;
			Ja->data[ind_y] = 0;
			Ja->data[ind_z] = 0;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		// 第 19 20 21 rol
		ind_x = JaNCol*18;
		ind_y = ind_x + JaNCol;
		ind_z = ind_y + JaNCol; 
		ind_source = 0;

		// 填寫角速度 Jacobian  fixed leg to right arm
		for (int i = 0 ; i < 6 ; i++) // 先跑6次 1~6 column  19~21 row(13)
		{
			Ja->data[ind_x] = -ZAxisAll->data[ind_source];
			Ja->data[ind_y] = -ZAxisAll->data[ind_source+1];
			Ja->data[ind_z] = -ZAxisAll->data[ind_source+2];
			ind_source += 3;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		// 填寫角速度 Jacobian  swing leg to right arm,left arm to right arm
		for (int i = 0 ; i < 12 ; i++) // 7~18 column  19~21 row(14)
		{
			Ja->data[ind_x] = 0;
			Ja->data[ind_y] = 0;
			Ja->data[ind_z] = 0;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		ind_source = 117;
		// 填寫角速度 Jacobian  right arm to right arm
		for (int i = 0 ; i < 6 ; i++) // 先跑6次 19~24 column  19~21 row(15)
		{
			Ja->data[ind_x] = ZAxisAll->data[ind_source];
			Ja->data[ind_y] = ZAxisAll->data[ind_source+1];
			Ja->data[ind_z] = ZAxisAll->data[ind_source+2];
			ind_source += 3;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		// 填寫方向速度 Jacobian 所有軸，對於left arm的endeffector
		ind_source = 0;
		ind_dest = 0;
		EndEff[0] = CrdAll->data[102];
		EndEff[1] = CrdAll->data[103];
		EndEff[2] = CrdAll->data[104];

		for (int i = 0 ; i <6 ; i++)//fixed腳每一軸到left arm endeffector的距離
		{
			EndEffDiff[ind_dest] =  CrdAll->data[ind_source] - EndEff[0];
			EndEffDiff[ind_dest+1] = CrdAll->data[ind_source+1] - EndEff[1];
			EndEffDiff[ind_dest+2] = CrdAll->data[ind_source+2] - EndEff[2];
			ind_source += 3;
			ind_dest += 3;
		}

		ind_source = 87;
		//int_dest 繼續加算
		for (int i = 6 ; i <12 ; i++)//left arm每一軸到left arm endeffector的距離
		{
			EndEffDiff[ind_dest] = EndEff[0] - CrdAll->data[ind_source];
			EndEffDiff[ind_dest+1] = EndEff[1] - CrdAll->data[ind_source+1];
			EndEffDiff[ind_dest+2] = EndEff[2] - CrdAll->data[ind_source+2];
			ind_source += 3;
			ind_dest += 3;
		}

		//第10 11 12 row
		ind_x = JaNCol*9;
		ind_y = ind_x + JaNCol;
		ind_z = ind_y + JaNCol; 
		ind_source = 0;
		// 填寫方向速度 Jacobian fixed leg to left arm
		for (int i = 0 ; i<6 ; i++)//1~6 column 10~12 row(16)
		{
			Cross2Vd(&ZAxisAll->data[ind_source],&EndEffDiff[ind_source],temp_cross);
			Ja->data[ind_x] = temp_cross[0];
			Ja->data[ind_y] = temp_cross[1];
			Ja->data[ind_z] = temp_cross[2];
			ind_source += 3;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		// 填寫方向速度 Jacobian  swing leg to left arm
		for (int i = 0 ; i < 6 ; i++)//7~12 column 10~12 row(17)
		{
			Ja->data[ind_x] = 0;
			Ja->data[ind_y] = 0;
			Ja->data[ind_z] = 0;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		ind_source = 87;
		ind_dest = 18; // 借這個變數來用，ind_dest原本的意思是 要存到的地方之index
		// 填寫方向速度 Jacobian left arm to left arm
		for (int i = 6 ; i<12 ; i++)//13~18 column 10~12 row(18)
		{
			Cross2Vd(&ZAxisAll->data[ind_source],&EndEffDiff[ind_dest],temp_cross);
			Ja->data[ind_x] = temp_cross[0];
			Ja->data[ind_y] = temp_cross[1];
			Ja->data[ind_z] = temp_cross[2];
			ind_source += 3;
			ind_dest += 3;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		// 填寫方向速度 Jacobian  right arm to left arm
		for (int i = 0 ; i < 6 ; i++)//19~24 column 10~12 row(19)
		{
			Ja->data[ind_x] = 0;
			Ja->data[ind_y] = 0;
			Ja->data[ind_z] = 0;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		// 填寫方向速度 Jacobian 所有軸，對於right arm的endeffector
		ind_source = 0;
		ind_dest = 0;
		EndEff[0] = CrdAll->data[132];
		EndEff[1] = CrdAll->data[133];
		EndEff[2] = CrdAll->data[134];

		for (int i = 0 ; i <6 ; i++)//fixed腳每一軸到right arm endeffector的距離
		{
			EndEffDiff[ind_dest] =  CrdAll->data[ind_source] - EndEff[0];
			EndEffDiff[ind_dest+1] = CrdAll->data[ind_source+1] - EndEff[1];
			EndEffDiff[ind_dest+2] = CrdAll->data[ind_source+2] - EndEff[2];
			ind_source += 3;
			ind_dest += 3;
		}

		ind_source = 117;
		//int_dest 繼續加算
		for (int i = 6 ; i <12 ; i++)//right arm每一軸到right arm endeffector的距離
		{
			EndEffDiff[ind_dest] = EndEff[0] - CrdAll->data[ind_source];
			EndEffDiff[ind_dest+1] = EndEff[1] - CrdAll->data[ind_source+1];
			EndEffDiff[ind_dest+2] = EndEff[2] - CrdAll->data[ind_source+2];
			ind_source += 3;
			ind_dest += 3;
		}

		//第16 17 18 row
		ind_x = JaNCol*15;
		ind_y = ind_x + JaNCol;
		ind_z = ind_y + JaNCol; 
		ind_source = 0;
		// 填寫方向速度 Jacobian fixed leg to right arm
		for (int i = 0 ; i<6 ; i++)//1~6 column 16~18 row(20)
		{
			Cross2Vd(&ZAxisAll->data[ind_source],&EndEffDiff[ind_source],temp_cross);
			Ja->data[ind_x] = temp_cross[0];
			Ja->data[ind_y] = temp_cross[1];
			Ja->data[ind_z] = temp_cross[2];
			ind_source += 3;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}
		// 填寫方向速度 Jacobian  swing leg to right arm,left arm to right arm
		for (int i = 0 ; i < 12 ; i++)//7~18 column 16~18 row(21)
		{
			Ja->data[ind_x] = 0;
			Ja->data[ind_y] = 0;
			Ja->data[ind_z] = 0;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		ind_source = 117;
		ind_dest = 18; // 借這個變數來用，ind_dest原本的意思是 要存到的地方之index
		// 填寫方向速度 Jacobian right arm to right arm
		for (int i = 6 ; i<12 ; i++)//19~24 column 16~18 row(22)
		{
			Cross2Vd(&ZAxisAll->data[ind_source],&EndEffDiff[ind_dest],temp_cross);
			Ja->data[ind_x] = temp_cross[0];
			Ja->data[ind_y] = temp_cross[1];
			Ja->data[ind_z] = temp_cross[2];
			ind_source += 3;
			ind_dest += 3;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}
	}
	else if (selIK == 1 || selIK == 2) // support leg = right
	{	
		//row 4 5 6
		ind_x = JaNCol*3;
		ind_y = ind_x + JaNCol;
		ind_z = ind_y + JaNCol; 
		ind_source = 0;

		// swing theta x y z, 3 rows, 12 cols
		// 填寫角速度 Jacobian  swing leg to swing leg     swing = left
		for (int i = 0 ; i < 6 ; i++) // 先跑6次  column 1~6 row 4~6(1)
		{
			Ja->data[ind_x] = ZAxisAll->data[ind_source];
			Ja->data[ind_y] = ZAxisAll->data[ind_source+1];
			Ja->data[ind_z] = ZAxisAll->data[ind_source+2];
			ind_source += 3;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		// 填寫角速度 Jacobian  fixed leg to swing leg     swing = left
		ind_source = 39;
		for (int i = 0 ; i < 6 ; i++) // 再跑6次 column 7~12 row 4~6(2)
		{
			Ja->data[ind_x] = -ZAxisAll->data[ind_source];
			Ja->data[ind_y] = -ZAxisAll->data[ind_source+1];
			Ja->data[ind_z] = -ZAxisAll->data[ind_source+2];
			ind_source += 3;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		// 填寫角速度 Jacobian  left arm to swing leg, right arm to swing leg
		for (int i = 0 ; i < 12 ; i++)//column 13~24 row 4~6(3)
		{
			Ja->data[ind_x] = 0;
			Ja->data[ind_y] = 0;
			Ja->data[ind_z] = 0;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}


		//row 7 8 9
		ind_x = JaNCol*6;
		ind_y = ind_x + JaNCol;
		ind_z = ind_y + JaNCol; 

		// fixed theta x y z, 3 rows, 12 cols
		// 填寫角速度 Jacobian  swing leg to fixed leg (all zeros)
		//ind_source = 0;
		for (int i = 0 ; i < 6 ; i++)//column 1~6 row 7~9(4)
		{
			Ja->data[ind_x] = 0;
			Ja->data[ind_y] = 0;
			Ja->data[ind_z] = 0;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		// 填寫角速度 Jacobian  fix leg to fixed leg
		ind_source = 39;
		for (int i = 0 ; i < 6 ; i++)//column 7~12 row 7~9(5)
		{
			Ja->data[ind_x] = ZAxisAll->data[ind_source];
			Ja->data[ind_y] = ZAxisAll->data[ind_source+1];
			Ja->data[ind_z] = ZAxisAll->data[ind_source+2];
			ind_source += 3;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}
		// 填寫角速度 Jacobian  left arm to fixed leg ,right arm to fixed leg(all zeros)
		//ind_source = 0;
		for (int i = 0 ; i < 12 ; i++)//column 13~24 row 7~9(6)
		{
			Ja->data[ind_x] = 0;
			Ja->data[ind_y] = 0;
			Ja->data[ind_z] = 0;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}


		// 填寫方向速度 Jacobian 所有軸，對於endeffector

		EndEff[0] = CrdAll->data[12];
		EndEff[1] = CrdAll->data[13];
		EndEff[2] = CrdAll->data[14];

		ind_source = 0;
		ind_dest = 0;
		for (int i = 0 ; i <6 ; i++)//swing腳每一軸到swing腳endeffector的距離
		{
			EndEffDiff[ind_dest] =  EndEff[0] - CrdAll->data[ind_source];
			EndEffDiff[ind_dest+1] = EndEff[1] - CrdAll->data[ind_source+1];
			EndEffDiff[ind_dest+2] = EndEff[2] - CrdAll->data[ind_source+2];
			ind_source += 3;
			ind_dest += 3;
		}

		ind_source = 39;
		//ind_dest 繼續加算
		for (int i = 6 ; i <12 ; i++)//fixed腳每一軸到swing腳endeffector的距離
		{
			EndEffDiff[ind_dest] =  CrdAll->data[ind_source] - EndEff[0];
			EndEffDiff[ind_dest+1] = CrdAll->data[ind_source+1] - EndEff[1];
			EndEffDiff[ind_dest+2] = CrdAll->data[ind_source+2] - EndEff[2];
			ind_source += 3;
			ind_dest += 3;
		}

		ind_x = 0;
		ind_y = JaNCol;
		ind_z = ind_y + JaNCol; 
		ind_source = 0;
		// 填寫方向速度 Jacobian swing leg to swing leg
		for (int i = 0 ; i<6 ; i++)//column 1~6 row 1~3(7)
		{
			Cross2Vd(&ZAxisAll->data[ind_source],&EndEffDiff[ind_source],temp_cross);
			Ja->data[ind_x] = temp_cross[0];
			Ja->data[ind_y] = temp_cross[1];
			Ja->data[ind_z] = temp_cross[2];
			ind_source += 3;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		ind_source = 39;
		ind_dest = 18; // 借這個變數來用，ind_dest原本的意思是 要存到的地方之index
		// 填寫方向速度 Jacobian fixed leg to swing leg
		for (int i = 6 ; i<12 ; i++)//column 7~12 row 1~3(8)
		{
			Cross2Vd(&ZAxisAll->data[ind_source],&EndEffDiff[ind_dest],temp_cross);
			Ja->data[ind_x] = temp_cross[0];
			Ja->data[ind_y] = temp_cross[1];
			Ja->data[ind_z] = temp_cross[2];
			ind_source += 3;
			ind_dest += 3;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}
		// 填寫方向速度 Jacobian  left arm to swing leg ,right arm to swing leg(all zeros)
		//ind_source = 0;
		for (int i = 0 ; i < 12 ; i++)//column 13~24 row 1~3(9)
		{
			Ja->data[ind_x] = 0;
			Ja->data[ind_y] = 0;
			Ja->data[ind_z] = 0;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		// 第 13 14 15 rol
		ind_x = JaNCol*12;
		ind_y = ind_x + JaNCol;
		ind_z = ind_y + JaNCol;

		// 填寫角速度 Jacobian  swing leg to left arm
		for (int i = 0 ; i < 6 ; i++) // 先跑6次 1~6 column  13~15 row(10)
		{
			Ja->data[ind_x] = 0;
			Ja->data[ind_y] = 0;
			Ja->data[ind_z] = 0;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}
		ind_source = 39;
		// 填寫角速度 Jacobian  fixed leg to left arm
		for (int i = 0 ; i < 6 ; i++) // 先跑6次 7~12 column  13~15 row(11)
		{
			Ja->data[ind_x] = -ZAxisAll->data[ind_source];
			Ja->data[ind_y] = -ZAxisAll->data[ind_source+1];
			Ja->data[ind_z] = -ZAxisAll->data[ind_source+2];
			ind_source += 3;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}
		ind_source = 87;
		// 填寫角速度 Jacobian  left arm to left arm
		for (int i = 0 ; i < 6 ; i++) // 先跑6次 13~18 column  13~15 row(12)
		{
			Ja->data[ind_x] = ZAxisAll->data[ind_source];
			Ja->data[ind_y] = ZAxisAll->data[ind_source+1];
			Ja->data[ind_z] = ZAxisAll->data[ind_source+2];
			ind_source += 3;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}
		// 填寫角速度 Jacobian  right arm to left arm
		for (int i = 0 ; i < 6 ; i++) // 先跑6次 19~24 column  13~15 row(13)
		{
			Ja->data[ind_x] = 0;
			Ja->data[ind_y] = 0;
			Ja->data[ind_z] = 0;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		// 第 19 20 21 rol
		ind_x = JaNCol*18;
		ind_y = ind_x + JaNCol;
		ind_z = ind_y + JaNCol; 


		// 填寫角速度 Jacobian  swing leg to right arm
		for (int i = 0 ; i < 6 ; i++) // 1~6 column  19~21 row(14)
		{
			Ja->data[ind_x] = 0;
			Ja->data[ind_y] = 0;
			Ja->data[ind_z] = 0;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		ind_source = 39;
		// 填寫角速度 Jacobian  fixed leg to right arm
		for (int i = 0 ; i < 6 ; i++) // 先跑6次 7~12 column  19~21 row(15)
		{
			Ja->data[ind_x] = -ZAxisAll->data[ind_source];
			Ja->data[ind_y] = -ZAxisAll->data[ind_source+1];
			Ja->data[ind_z] = -ZAxisAll->data[ind_source+2];
			ind_source += 3;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		// 填寫角速度 Jacobian  left arm to right arm
		for (int i = 0 ; i < 6 ; i++) // 13~18 column  19~21 row(16)
		{
			Ja->data[ind_x] = 0;
			Ja->data[ind_y] = 0;
			Ja->data[ind_z] = 0;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		ind_source = 117;
		// 填寫角速度 Jacobian  right arm to right arm
		for (int i = 0 ; i < 6 ; i++) // 先跑6次 19~24 column  19~21 row(17)
		{
			Ja->data[ind_x] = ZAxisAll->data[ind_source];
			Ja->data[ind_y] = ZAxisAll->data[ind_source+1];
			Ja->data[ind_z] = ZAxisAll->data[ind_source+2];
			ind_source += 3;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		// 填寫方向速度 Jacobian 所有軸，對於left arm的endeffector
		ind_source = 39;
		ind_dest = 0;
		EndEff[0] = CrdAll->data[102];
		EndEff[1] = CrdAll->data[103];
		EndEff[2] = CrdAll->data[104] ;

		for (int i = 0 ; i <6 ; i++)//fixed腳每一軸到left arm endeffector的距離
		{
			EndEffDiff[ind_dest] =  CrdAll->data[ind_source] - EndEff[0];
			EndEffDiff[ind_dest+1] = CrdAll->data[ind_source+1] - EndEff[1];
			EndEffDiff[ind_dest+2] = CrdAll->data[ind_source+2] - EndEff[2];
			ind_source += 3;
			ind_dest += 3;
		}

		ind_source = 87;
		//int_dest 繼續加算
		for (int i = 6 ; i <12 ; i++)//left arm每一軸到left arm endeffector的距離
		{
			EndEffDiff[ind_dest] = EndEff[0] - CrdAll->data[ind_source];
			EndEffDiff[ind_dest+1] = EndEff[1] - CrdAll->data[ind_source+1];
			EndEffDiff[ind_dest+2] = EndEff[2] - CrdAll->data[ind_source+2];
			ind_source += 3;
			ind_dest += 3;
		}

		//第10 11 12 row
		ind_x = JaNCol*9;
		ind_y = ind_x + JaNCol;
		ind_z = ind_y + JaNCol; 
		

		// 填寫方向速度 Jacobian  swing leg to left arm
		for (int i = 0 ; i < 6 ; i++)//1~6 column 10~12 row(18)
		{
			Ja->data[ind_x] = 0;
			Ja->data[ind_y] = 0;
			Ja->data[ind_z] = 0;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		ind_source = 39;
		ind_dest = 0; // 借這個變數來用，ind_dest原本的意思是 要存到的地方之index
		// 填寫方向速度 Jacobian fixed leg to left arm
		for (int i = 0 ; i<6 ; i++)//7~12 column 10~12 row(19)
		{
			Cross2Vd(&ZAxisAll->data[ind_source],&EndEffDiff[ind_dest],temp_cross);
			Ja->data[ind_x] = temp_cross[0];
			Ja->data[ind_y] = temp_cross[1];
			Ja->data[ind_z] = temp_cross[2];
			ind_source += 3;
			ind_dest +=3;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		ind_source = 87;
		ind_dest = 18; // 借這個變數來用，ind_dest原本的意思是 要存到的地方之index
		// 填寫方向速度 Jacobian left arm to left arm
		for (int i = 6 ; i<12 ; i++)//13~18 column 10~12 row(20)
		{
			Cross2Vd(&ZAxisAll->data[ind_source],&EndEffDiff[ind_dest],temp_cross);
			Ja->data[ind_x] = temp_cross[0];
			Ja->data[ind_y] = temp_cross[1];
			Ja->data[ind_z] = temp_cross[2];
			ind_source += 3;
			ind_dest += 3;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		// 填寫方向速度 Jacobian  right arm to left arm
		for (int i = 0 ; i < 6 ; i++)//19~24 column 10~12 row(21)
		{
			Ja->data[ind_x] = 0;
			Ja->data[ind_y] = 0;
			Ja->data[ind_z] = 0;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}



		// 填寫方向速度 Jacobian 所有軸，對於right arm的endeffector
		ind_source = 39;
		ind_dest = 0;
		EndEff[0] = CrdAll->data[132];
		EndEff[1] = CrdAll->data[133];
		EndEff[2] = CrdAll->data[134];


		for (int i = 0 ; i <6 ; i++)//fixed腳每一軸到right arm endeffector的距離
		{
			EndEffDiff[ind_dest] =  CrdAll->data[ind_source] - EndEff[0];
			EndEffDiff[ind_dest+1] = CrdAll->data[ind_source+1] - EndEff[1];
			EndEffDiff[ind_dest+2] = CrdAll->data[ind_source+2] - EndEff[2];
			ind_source += 3;
			ind_dest += 3;
		}

		ind_source = 117;
		//int_dest 繼續加算
		for (int i = 6 ; i <12 ; i++)//right arm每一軸到right arm endeffector的距離
		{
			EndEffDiff[ind_dest] = EndEff[0] - CrdAll->data[ind_source];
			EndEffDiff[ind_dest+1] = EndEff[1] - CrdAll->data[ind_source+1];
			EndEffDiff[ind_dest+2] = EndEff[2] - CrdAll->data[ind_source+2];
			ind_source += 3;
			ind_dest += 3;
		}

		//第16 17 18 row
		ind_x = JaNCol*15;
		ind_y = ind_x + JaNCol;
		ind_z = ind_y + JaNCol; 

		// 填寫方向速度 Jacobian  swing leg to right arm
		for (int i = 0 ; i < 6 ; i++)//1~6 column 16~18 row(22)
		{
			Ja->data[ind_x] = 0;
			Ja->data[ind_y] = 0;
			Ja->data[ind_z] = 0;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		ind_source = 39;
		ind_dest = 0; // 借這個變數來用，ind_dest原本的意思是 要存到的地方之index
		// 填寫方向速度 Jacobian fixed leg to right arm
		for (int i = 0 ; i<6 ; i++)//6~12 column 16~18 row(23)
		{
			Cross2Vd(&ZAxisAll->data[ind_source],&EndEffDiff[ind_dest],temp_cross);
			Ja->data[ind_x] = temp_cross[0];
			Ja->data[ind_y] = temp_cross[1];
			Ja->data[ind_z] = temp_cross[2];
			ind_source += 3;
			ind_dest += 3;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		// 填寫方向速度 Jacobian  left arm to right arm
		for (int i = 0 ; i < 6 ; i++)//13~18 column 16~18 row(24)
		{
			Ja->data[ind_x] = 0;
			Ja->data[ind_y] = 0;
			Ja->data[ind_z] = 0;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		ind_source = 117;
		ind_dest = 18; // 借這個變數來用，ind_dest原本的意思是 要存到的地方之index
		// 填寫方向速度 Jacobian right arm to right arm
		for (int i = 6 ; i<12 ; i++)//19~24 column 16~18 row(25)
		{
			Cross2Vd(&ZAxisAll->data[ind_source],&EndEffDiff[ind_dest],temp_cross);
			Ja->data[ind_x] = temp_cross[0];
			Ja->data[ind_y] = temp_cross[1];
			Ja->data[ind_z] = temp_cross[2];
			ind_source += 3;
			ind_dest += 3;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		} 
	}

	// 計算機器人COG Jacobian
	GetCOGJacobian();

}

void Kine::ComputeFixJacobians(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 求出機器人整體的Jacobian matrix 
	******************************************************************/
	// dth => [左腳6軸 ; 右腳6軸]
	// selIK = 0 : left foot is the supporter
    // selIK = 1 : right foot is the supporter

	// when selIK = 0,   dx = [sw_x sw_y sw_z sw_tx sw_ty sw_tz fix_tx fix_ty fix_tz COGx COGy COGz ]
	// when selIK = 1,2, dx = [sw_x sw_y sw_z sw_tx sw_ty sw_tz fix_tx fix_ty fix_tz COGx COGy COGz ]
	// 被排成相同的!!

	if (selIK == 0) // support leg = left  // 還沒改
	{
		//// 第 4 5 6 rol
		//ind_x = JaNCol*3;
		//ind_y = ind_x + JaNCol;
		//ind_z = ind_y + JaNCol; 
		//ind_source = 0;

		//// swing theta x y z, 3 rows, 12 cols
		//// 填寫角速度 Jacobian  fixed leg to swing leg
		//for (int i = 0 ; i < 6 ; i++) // 先跑6次 1~6 column
		//{
		//	Ja->data[ind_x] = -ZAxisAll->data[ind_source];
		//	Ja->data[ind_y] = -ZAxisAll->data[ind_source+1];
		//	Ja->data[ind_z] = -ZAxisAll->data[ind_source+2];
		//	ind_source += 3;
		//	ind_x += 1;
		//	ind_y += 1;
		//	ind_z += 1;
		//}

		//// 填寫角速度 Jacobian  swing leg to swing leg
		//ind_source = 39;
		//for (int i = 0 ; i < 6 ; i++) // 再跑6次 7~12 column
		//{
		//	Ja->data[ind_x] = ZAxisAll->data[ind_source];
		//	Ja->data[ind_y] = ZAxisAll->data[ind_source+1];
		//	Ja->data[ind_z] = ZAxisAll->data[ind_source+2];
		//	ind_source += 3;
		//	ind_x += 1;
		//	ind_y += 1;
		//	ind_z += 1;
		//}

		//// 第 7 8 9 rol
		//ind_x = JaNCol*6;
		//ind_y = ind_x + JaNCol;
		//ind_z = ind_y + JaNCol; 
		int JaNColF=6;

		ind_x = 0;
		ind_y = ind_x + JaNColF;
		ind_z = ind_y + JaNColF; 

		// fixed theta x y z, 3 rows, 12 cols
		// 填寫角速度 Jacobian  fixed leg to fixed leg
		ind_source = 0;
		for (int i = 0 ; i < 6 ; i++)
		{
			FixJa->data[ind_x] = ZAxisAll->data[ind_source];
			FixJa->data[ind_y] = ZAxisAll->data[ind_source+1];
			FixJa->data[ind_z] = ZAxisAll->data[ind_source+2];
			ind_source += 3;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		//// 填寫角速度 Jacobian  swing leg to fixed leg
		//for (int i = 0 ; i < 6 ; i++)
		//{
		//	Ja->data[ind_x] = 0;
		//	Ja->data[ind_y] = 0;
		//	Ja->data[ind_z] = 0;
		//	ind_x += 1;
		//	ind_y += 1;
		//	ind_z += 1;
		//}

		//// 填寫方向速度 Jacobian 所有軸，對於swing腳的endeffector
		//ind_source = 0;
		//ind_dest = 0;
		//EndEff[0] = CrdAll->data[51];
		//EndEff[1] = CrdAll->data[52];
		//EndEff[2] = CrdAll->data[53];

		//for (int i = 0 ; i <6 ; i++)
		//{
		//	EndEffDiff[ind_dest] =  CrdAll->data[ind_source] - EndEff[0];
		//	EndEffDiff[ind_dest+1] = CrdAll->data[ind_source+1] - EndEff[1];
		//	EndEffDiff[ind_dest+2] = CrdAll->data[ind_source+2] - EndEff[2];
		//	ind_source += 3;
		//	ind_dest += 3;
		//}

		//ind_source = 39;
		////int_dest 繼續加算
		//for (int i = 6 ; i <12 ; i++)
		//{
		//	EndEffDiff[ind_dest] = EndEff[0] - CrdAll->data[ind_source];
		//	EndEffDiff[ind_dest+1] = EndEff[1] - CrdAll->data[ind_source+1];
		//	EndEffDiff[ind_dest+2] = EndEff[2] - CrdAll->data[ind_source+2];
		//	ind_source += 3;
		//	ind_dest += 3;
		//}

		//ind_x = 0;
		//ind_y = JaNCol;
		//ind_z = ind_y + JaNCol; 
		//ind_source = 0;
		//for (int i = 0 ; i<6 ; i++)
		//{
		//	Cross2Vd(&ZAxisAll->data[ind_source],&EndEffDiff[ind_source],temp_cross);
		//	Ja->data[ind_x] = temp_cross[0];
		//	Ja->data[ind_y] = temp_cross[1];
		//	Ja->data[ind_z] = temp_cross[2];
		//	ind_source += 3;
		//	ind_x += 1;
		//	ind_y += 1;
		//	ind_z += 1;
		//}

		//ind_source = 39;
		//ind_dest = 18; // 借這個變數來用，ind_dest原本的意思是 要存到的地方之index
		//for (int i = 6 ; i<12 ; i++)
		//{
		//	Cross2Vd(&ZAxisAll->data[ind_source],&EndEffDiff[ind_dest],temp_cross);
		//	Ja->data[ind_x] = temp_cross[0];
		//	Ja->data[ind_y] = temp_cross[1];
		//	Ja->data[ind_z] = temp_cross[2];
		//	ind_source += 3;
		//	ind_dest += 3;
		//	ind_x += 1;
		//	ind_y += 1;
		//	ind_z += 1;
		//}
	}
	else if (selIK == 1 || selIK == 2) // support leg = right
	{	

		//ind_x = JaNCol*3; //36
		//ind_y = ind_x + JaNCol;
		//ind_z = ind_y + JaNCol; 
		//ind_source = 0;

		//// swing theta x y z, 3 rows, 12 cols
		//// 填寫角速度 Jacobian  swing leg to swing leg     swing = left
		//for (int i = 0 ; i < 6 ; i++) // 先跑6次
		//{
		//	Ja->data[ind_x] = ZAxisAll->data[ind_source];
		//	Ja->data[ind_y] = ZAxisAll->data[ind_source+1];
		//	Ja->data[ind_z] = ZAxisAll->data[ind_source+2];
		//	ind_source += 3;
		//	ind_x += 1;
		//	ind_y += 1;
		//	ind_z += 1;
		//}

		//// 填寫角速度 Jacobian  fixed leg to swing leg     swing = left
		//ind_source = 39;
		//for (int i = 0 ; i < 6 ; i++) // 再跑6次
		//{
		//	Ja->data[ind_x] = -ZAxisAll->data[ind_source];
		//	Ja->data[ind_y] = -ZAxisAll->data[ind_source+1];
		//	Ja->data[ind_z] = -ZAxisAll->data[ind_source+2];
		//	ind_source += 3;
		//	ind_x += 1;
		//	ind_y += 1;
		//	ind_z += 1;
		//}
		int JaNColF=6;

		ind_x = 0;
		ind_y = ind_x + JaNColF;
		ind_z = ind_y + JaNColF; 

		// 填寫角速度 Jacobian  fix leg to fixed leg
		ind_source = 39;
		for (int i = 0 ; i < 6 ; i++)
		{
			FixJa->data[ind_x] = ZAxisAll->data[ind_source];
			FixJa->data[ind_y] = ZAxisAll->data[ind_source+1];
			FixJa->data[ind_z] = ZAxisAll->data[ind_source+2];
			ind_source += 3;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		//// 填寫方向速度 Jacobian 所有軸，對於endeffector

		//EndEff[0] = CrdAll->data[12];
		//EndEff[1] = CrdAll->data[13];
		//EndEff[2] = CrdAll->data[14];

		//ind_source = 0;
		//ind_dest = 0;
		//for (int i = 0 ; i <6 ; i++)
		//{
		//	EndEffDiff[ind_dest] =  EndEff[0] - CrdAll->data[ind_source];
		//	EndEffDiff[ind_dest+1] = EndEff[1] - CrdAll->data[ind_source+1];
		//	EndEffDiff[ind_dest+2] = EndEff[2] - CrdAll->data[ind_source+2];
		//	ind_source += 3;
		//	ind_dest += 3;
		//}

		//ind_source = 39;
		////ind_dest 繼續加算
		//for (int i = 6 ; i <12 ; i++)
		//{
		//	EndEffDiff[ind_dest] =  CrdAll->data[ind_source] - EndEff[0];
		//	EndEffDiff[ind_dest+1] = CrdAll->data[ind_source+1] - EndEff[1];
		//	EndEffDiff[ind_dest+2] = CrdAll->data[ind_source+2] - EndEff[2];
		//	ind_source += 3;
		//	ind_dest += 3;
		//}

		//ind_x = 0;
		//ind_y = JaNCol;
		//ind_z = ind_y + JaNCol; 
		//ind_source = 0;
		//for (int i = 0 ; i<6 ; i++)
		//{
		//	Cross2Vd(&ZAxisAll->data[ind_source],&EndEffDiff[ind_source],temp_cross);
		//	Ja->data[ind_x] = temp_cross[0];
		//	Ja->data[ind_y] = temp_cross[1];
		//	Ja->data[ind_z] = temp_cross[2];
		//	ind_source += 3;
		//	ind_x += 1;
		//	ind_y += 1;
		//	ind_z += 1;
		//}

		//ind_source = 39;
		//ind_dest = 18; // 借這個變數來用，ind_dest原本的意思是 要存到的地方之index
		//for (int i = 6 ; i<12 ; i++)
		//{
		//	Cross2Vd(&ZAxisAll->data[ind_source],&EndEffDiff[ind_dest],temp_cross);
		//	Ja->data[ind_x] = temp_cross[0];
		//	Ja->data[ind_y] = temp_cross[1];
		//	Ja->data[ind_z] = temp_cross[2];
		//	ind_source += 3;
		//	ind_dest += 3;
		//	ind_x += 1;
		//	ind_y += 1;
		//	ind_z += 1;
		//}
	}

	// 計算機器人COG Jacobian
	GetCOGFixJacobian();
	//double ORZ[36];
	//		for (int i = 0 ; i < 36 ; i++)
	//	{
	//		ORZ[i]=FixJa->data[i] ;
	//		}
}



void Kine::NormVecArray(double* VecArray, double* Norm, int Len)
{
	/******************************************************************
	input: VecArray 輸入之 vector array 三個三個一組 排成整個array, Len 代表輸入矩陣的長度
	output: Norm 算完的所有norm值之輸出

	Note:
	// 將輸入矩陣的值 三個三個求得norm之後輸出
	******************************************************************/

	int Index = 0;
	for (int i = 0 ; i < Len ; i++)
	{		
		Norm[i] = sqrt(VecArray[Index]*VecArray[Index] + VecArray[Index+1]*VecArray[Index+1] + VecArray[Index+2]*VecArray[Index+2]);
		Index += 3;
	}

}

void Kine::GetCOGJacobian(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 計算COG jacobian
	******************************************************************/

	// dth => [左腳6軸 ; 右腳6軸]
	// selIK = 0 : left foot is the supporter
    // selIK = 1 : right foot is the supporter

	// when selIK = 0, dx = [sw_x sw_y sw_z sw_tx sw_ty sw_tz fix_tx fix_ty fix_tz COGx COGy COGz]'
	// when selIK = 0, dx = [sw_x sw_y sw_z sw_tx sw_ty sw_tz fix_tx fix_ty fix_tz LA_x LA_y LA_z LA_tx LA_ty LA_tz RA_x RA_y RA_z RA_tx RA_ty RA_tz COGx COGy COGz]'

	// 算出各個linkage的重量乘上其在世界中的位置
	// 求出後 直接利用會影響當前累積質量乘上位置之值 計算COG Jacobian
	// 詳細公式與推導請參見論文
	//泓逸start111223
	if (selIK == LeftSupport) // selIK = 0 : left foot is the supporter
	{

		// axis 12 RL ankle roll
		temp_r_ef[0] = pv_stack[15]-CrdAll->data[54];
		temp_r_ef[1] = pv_stack[16]-CrdAll->data[55];
		temp_r_ef[2] = pv_stack[17]-CrdAll->data[56];

		temp_scale = mass_com[5]/SumMass;
		Cross2Vd(&ZAxisAll->data[54],temp_r_ef,temp_cross);
		Ja->ValSet(22,12,temp_cross[0]*temp_scale); //(1)
		Ja->ValSet(23,12,temp_cross[1]*temp_scale); 
		Ja->ValSet(24,12,temp_cross[2]*temp_scale); 

		// axis 11 RL ankle pitch
		Cross2Vd(&ZAxisAll->data[51],temp_r_ef,temp_cross);
		Ja->ValSet(22,11,temp_cross[0]*temp_scale);//(2) 
		Ja->ValSet(23,11,temp_cross[1]*temp_scale); 
		Ja->ValSet(24,11,temp_cross[2]*temp_scale); 

		// axis 10 RL knee pitch
		temp_scale += mass_com[4]/SumMass;
		temp_mass_sum = mass_com[4] + mass_com[5];
		local_COG[0] = (pv_stack[12]*mass_com[4]+pv_stack[15]*mass_com[5])/temp_mass_sum;
		local_COG[1] = (pv_stack[13]*mass_com[4]+pv_stack[16]*mass_com[5])/temp_mass_sum;
		local_COG[2] = (pv_stack[14]*mass_com[4]+pv_stack[17]*mass_com[5])/temp_mass_sum;

		temp_r_ef[0] = local_COG[0]-CrdAll->data[48];
		temp_r_ef[1] = local_COG[1]-CrdAll->data[49];
		temp_r_ef[2] = local_COG[2]-CrdAll->data[50];
		Cross2Vd(&ZAxisAll->data[48],temp_r_ef,temp_cross);
		Ja->ValSet(22,10,temp_cross[0]*temp_scale);//(3) 
		Ja->ValSet(23,10,temp_cross[1]*temp_scale); 
		Ja->ValSet(24,10,temp_cross[2]*temp_scale); 

		// axis 9 RL hip pitch
		temp_scale += mass_com[3]/SumMass;
		temp_mass_sum_next = temp_mass_sum + mass_com[3];
		local_COG[0] = (local_COG[0]*temp_mass_sum + pv_stack[9]*mass_com[3])/temp_mass_sum_next;
		local_COG[1] = (local_COG[1]*temp_mass_sum + pv_stack[10]*mass_com[3])/temp_mass_sum_next;
		local_COG[2] = (local_COG[2]*temp_mass_sum + pv_stack[11]*mass_com[3])/temp_mass_sum_next;
		temp_mass_sum = temp_mass_sum_next;
		
		temp_r_ef[0] = local_COG[0]-CrdAll->data[45];
		temp_r_ef[1] = local_COG[1]-CrdAll->data[46];
		temp_r_ef[2] = local_COG[2]-CrdAll->data[47];
		Cross2Vd(&ZAxisAll->data[45],temp_r_ef,temp_cross);
		Ja->ValSet(22,9,temp_cross[0]*temp_scale); //(4)
		Ja->ValSet(23,9,temp_cross[1]*temp_scale); 
		Ja->ValSet(24,9,temp_cross[2]*temp_scale); 

		// axis 8 RL hip roll
		Cross2Vd(&ZAxisAll->data[42],temp_r_ef,temp_cross);
		Ja->ValSet(22,8,temp_cross[0]*temp_scale);//(5) 
		Ja->ValSet(23,8,temp_cross[1]*temp_scale); 
		Ja->ValSet(24,8,temp_cross[2]*temp_scale); 

		// axis 7 RL hip yaw
		Cross2Vd(&ZAxisAll->data[39],temp_r_ef,temp_cross);
		Ja->ValSet(22,7,temp_cross[0]*temp_scale); //(6)
		Ja->ValSet(23,7,temp_cross[1]*temp_scale); 
		Ja->ValSet(24,7,temp_cross[2]*temp_scale); 


		//left arm COG Jacobian




		// LA axis6
		temp_r_ef[0] = pv_stack[27]-CrdAll->data[102];
		temp_r_ef[1] = pv_stack[28]-CrdAll->data[103];
		temp_r_ef[2] = pv_stack[29]-CrdAll->data[104];

		temp_scale_LA = mass_com[9]/SumMass;
		Cross2Vd(&ZAxisAll->data[102],temp_r_ef,temp_cross);
		Ja->ValSet(22,18,temp_cross[0]*temp_scale_LA); //(7)
		Ja->ValSet(23,18,temp_cross[1]*temp_scale_LA); 
		Ja->ValSet(24,18,temp_cross[2]*temp_scale_LA); 

		// LA axis5
		temp_scale_LA += mass_com[8]/SumMass;
		temp_mass_sum_LA = mass_com[8] + mass_com[9];
		local_COG_LA[0] = (pv_stack[24]*mass_com[8]+pv_stack[27]*mass_com[9])/temp_mass_sum_LA;
		local_COG_LA[1] = (pv_stack[25]*mass_com[8]+pv_stack[28]*mass_com[9])/temp_mass_sum_LA;
		local_COG_LA[2] = (pv_stack[26]*mass_com[8]+pv_stack[29]*mass_com[9])/temp_mass_sum_LA;

		temp_r_ef[0] = local_COG_LA[0]-CrdAll->data[99];
		temp_r_ef[1] = local_COG_LA[1]-CrdAll->data[100];
		temp_r_ef[2] = local_COG_LA[2]-CrdAll->data[101];
		Cross2Vd(&ZAxisAll->data[99],temp_r_ef,temp_cross);
		Ja->ValSet(22,17,temp_cross[0]*temp_scale_LA);//(8) 
		Ja->ValSet(23,17,temp_cross[1]*temp_scale_LA); 
		Ja->ValSet(24,17,temp_cross[2]*temp_scale_LA); 

		// LA axis4
		Cross2Vd(&ZAxisAll->data[96],temp_r_ef,temp_cross);
		Ja->ValSet(22,16,temp_cross[0]*temp_scale_LA);//(9) 
		Ja->ValSet(23,16,temp_cross[1]*temp_scale_LA); 
		Ja->ValSet(24,16,temp_cross[2]*temp_scale_LA);

		// LA axis3
		temp_scale_LA += mass_com[7]/SumMass;
		temp_mass_sum_next_LA = temp_mass_sum_LA + mass_com[7];
		local_COG_LA[0] = (local_COG_LA[0]*temp_mass_sum_LA + pv_stack[21]*mass_com[7])/temp_mass_sum_next_LA;
		local_COG_LA[1] = (local_COG_LA[1]*temp_mass_sum_LA + pv_stack[22]*mass_com[7])/temp_mass_sum_next_LA;
		local_COG_LA[2] = (local_COG_LA[2]*temp_mass_sum_LA + pv_stack[23]*mass_com[7])/temp_mass_sum_next_LA;
		temp_mass_sum_LA = temp_mass_sum_next_LA;

		temp_r_ef[0] = local_COG_LA[0]-CrdAll->data[93];
		temp_r_ef[1] = local_COG_LA[1]-CrdAll->data[94];
		temp_r_ef[2] = local_COG_LA[2]-CrdAll->data[95];
		Cross2Vd(&ZAxisAll->data[93],temp_r_ef,temp_cross);
		Ja->ValSet(22,15,temp_cross[0]*temp_scale_LA); //(10)
		Ja->ValSet(23,15,temp_cross[1]*temp_scale_LA); 
		Ja->ValSet(24,15,temp_cross[2]*temp_scale_LA); 

		// LA axis 2
		Cross2Vd(&ZAxisAll->data[90],temp_r_ef,temp_cross);
		Ja->ValSet(22,14,temp_cross[0]*temp_scale_LA);//(11) 
		Ja->ValSet(23,14,temp_cross[1]*temp_scale_LA); 
		Ja->ValSet(24,14,temp_cross[2]*temp_scale_LA); 

		// LA axis 1
		temp_scale_LA += mass_com[6]/SumMass;
		temp_mass_sum_next_LA = temp_mass_sum_LA + mass_com[6];
		local_COG_LA[0] = (local_COG_LA[0]*temp_mass_sum_LA + pv_stack[18]*mass_com[6])/temp_mass_sum_next_LA;
		local_COG_LA[1] = (local_COG_LA[1]*temp_mass_sum_LA + pv_stack[19]*mass_com[6])/temp_mass_sum_next_LA;
		local_COG_LA[2] = (local_COG_LA[2]*temp_mass_sum_LA + pv_stack[20]*mass_com[6])/temp_mass_sum_next_LA;
		temp_mass_sum_LA = temp_mass_sum_next_LA;

		temp_r_ef[0] = local_COG_LA[0]-CrdAll->data[87];
		temp_r_ef[1] = local_COG_LA[1]-CrdAll->data[88];
		temp_r_ef[2] = local_COG_LA[2]-CrdAll->data[89];
		Cross2Vd(&ZAxisAll->data[87],temp_r_ef,temp_cross);
		Ja->ValSet(22,13,temp_cross[0]*temp_scale_LA); //(12)
		Ja->ValSet(23,13,temp_cross[1]*temp_scale_LA); 
		Ja->ValSet(24,13,temp_cross[2]*temp_scale_LA);




		//right arm COG Jacobian

		// RA axis6
		temp_r_ef[0] = pv_stack[39]-CrdAll->data[132];
		temp_r_ef[1] = pv_stack[40]-CrdAll->data[133];
		temp_r_ef[2] = pv_stack[41]-CrdAll->data[134];

		temp_scale_RA = mass_com[13]/SumMass;
		Cross2Vd(&ZAxisAll->data[132],temp_r_ef,temp_cross);
		Ja->ValSet(22,24,temp_cross[0]*temp_scale_RA); //(13)
		Ja->ValSet(23,24,temp_cross[1]*temp_scale_RA); 
		Ja->ValSet(24,24,temp_cross[2]*temp_scale_RA); 

		// RA axis5
		temp_scale_RA += mass_com[12]/SumMass;
		temp_mass_sum_RA = mass_com[12] + mass_com[13];
		local_COG_RA[0] = (pv_stack[39]*mass_com[13]+pv_stack[36]*mass_com[12])/temp_mass_sum_RA;
		local_COG_RA[1] = (pv_stack[40]*mass_com[13]+pv_stack[37]*mass_com[12])/temp_mass_sum_RA;
		local_COG_RA[2] = (pv_stack[41]*mass_com[13]+pv_stack[38]*mass_com[12])/temp_mass_sum_RA;

		temp_r_ef[0] = local_COG_RA[0]-CrdAll->data[129];
		temp_r_ef[1] = local_COG_RA[1]-CrdAll->data[130];
		temp_r_ef[2] = local_COG_RA[2]-CrdAll->data[131];
		Cross2Vd(&ZAxisAll->data[129],temp_r_ef,temp_cross);
		Ja->ValSet(22,23,temp_cross[0]*temp_scale_RA);//(14) 
		Ja->ValSet(23,23,temp_cross[1]*temp_scale_RA); 
		Ja->ValSet(24,23,temp_cross[2]*temp_scale_RA); 

		// RA axis4
		Cross2Vd(&ZAxisAll->data[126],temp_r_ef,temp_cross);
		Ja->ValSet(22,22,temp_cross[0]*temp_scale_RA);//(15) 
		Ja->ValSet(23,22,temp_cross[1]*temp_scale_RA); 
		Ja->ValSet(24,22,temp_cross[2]*temp_scale_RA);

		// RA axis3
		temp_scale_RA += mass_com[11]/SumMass;
		temp_mass_sum_next_RA = temp_mass_sum_RA + mass_com[11];
		local_COG_RA[0] = (local_COG_RA[0]*temp_mass_sum_RA + pv_stack[33]*mass_com[11])/temp_mass_sum_next_RA;
		local_COG_RA[1] = (local_COG_RA[1]*temp_mass_sum_RA + pv_stack[34]*mass_com[11])/temp_mass_sum_next_RA;
		local_COG_RA[2] = (local_COG_RA[2]*temp_mass_sum_RA + pv_stack[35]*mass_com[11])/temp_mass_sum_next_RA;
		temp_mass_sum_RA = temp_mass_sum_next_RA;

		temp_r_ef[0] = local_COG_RA[0]-CrdAll->data[123];
		temp_r_ef[1] = local_COG_RA[1]-CrdAll->data[124];
		temp_r_ef[2] = local_COG_RA[2]-CrdAll->data[125];
		Cross2Vd(&ZAxisAll->data[123],temp_r_ef,temp_cross);
		Ja->ValSet(22,21,temp_cross[0]*temp_scale_RA); //(16)
		Ja->ValSet(23,21,temp_cross[1]*temp_scale_RA); 
		Ja->ValSet(24,21,temp_cross[2]*temp_scale_RA); 

		// RA axis 2
		Cross2Vd(&ZAxisAll->data[120],temp_r_ef,temp_cross);
		Ja->ValSet(22,20,temp_cross[0]*temp_scale_RA);//(17) 
		Ja->ValSet(23,20,temp_cross[1]*temp_scale_RA); 
		Ja->ValSet(24,20,temp_cross[2]*temp_scale_RA); 

		// RA axis 1
		temp_scale_RA += mass_com[10]/SumMass;
		temp_mass_sum_next_RA = temp_mass_sum_RA + mass_com[10];
		local_COG_RA[0] = (local_COG_RA[0]*temp_mass_sum_RA + pv_stack[30]*mass_com[10])/temp_mass_sum_next_RA;
		local_COG_RA[1] = (local_COG_RA[1]*temp_mass_sum_RA + pv_stack[31]*mass_com[10])/temp_mass_sum_next_RA;
		local_COG_RA[2] = (local_COG_RA[2]*temp_mass_sum_RA + pv_stack[32]*mass_com[10])/temp_mass_sum_next_RA;
		temp_mass_sum_RA = temp_mass_sum_next_RA;

		temp_r_ef[0] = local_COG_RA[0]-CrdAll->data[117];
		temp_r_ef[1] = local_COG_RA[1]-CrdAll->data[118];
		temp_r_ef[2] = local_COG_RA[2]-CrdAll->data[119];
		Cross2Vd(&ZAxisAll->data[117],temp_r_ef,temp_cross);
		Ja->ValSet(22,19,temp_cross[0]*temp_scale_RA); //(18)
		Ja->ValSet(23,19,temp_cross[1]*temp_scale_RA); 
		Ja->ValSet(24,19,temp_cross[2]*temp_scale_RA);









		// fixed leg Jacobian : axis 1~6
		// axis 1 LL hip yaw include body and arms
		//////temp_scale += mass_com[8]/SumMass + mass_com[9]/SumMass;
		//////temp_mass_sum_next += mass_com[8] + mass_com[9];
		//////local_COG[0] = (local_COG[0]*temp_mass_sum + pv_stack[24]*mass_com[8]+ pv_stack[27]*mass_com[9])/temp_mass_sum_next;
		//////local_COG[1] = (local_COG[1]*temp_mass_sum + pv_stack[25]*mass_com[8]+ pv_stack[28]*mass_com[9])/temp_mass_sum_next;
		//////local_COG[2] = (local_COG[2]*temp_mass_sum + pv_stack[26]*mass_com[8]+ pv_stack[29]*mass_com[9])/temp_mass_sum_next;
		//////temp_mass_sum = temp_mass_sum_next;

		temp_scale += temp_scale_LA + temp_scale_RA + (mass_com[14] + mass_com[15])/SumMass;
		temp_mass_sum_next += temp_mass_sum_LA + temp_mass_sum_RA + mass_com[14] + mass_com[15];
		local_COG[0] = (local_COG[0]*temp_mass_sum + local_COG_LA[0]*temp_mass_sum_LA + local_COG_RA[0]*temp_mass_sum_RA + pv_stack[42]*mass_com[14] + pv_stack[45]*mass_com[15])/temp_mass_sum_next;
		local_COG[1] = (local_COG[1]*temp_mass_sum + local_COG_LA[1]*temp_mass_sum_LA + local_COG_RA[1]*temp_mass_sum_RA + pv_stack[43]*mass_com[14] + pv_stack[46]*mass_com[15])/temp_mass_sum_next;
		local_COG[2] = (local_COG[2]*temp_mass_sum + local_COG_LA[2]*temp_mass_sum_LA + local_COG_RA[2]*temp_mass_sum_RA + pv_stack[44]*mass_com[14] + pv_stack[47]*mass_com[15])/temp_mass_sum_next;
		temp_mass_sum = temp_mass_sum_next;


		temp_r_ef[0] = local_COG[0]-CrdAll->data[0];
		temp_r_ef[1] = local_COG[1]-CrdAll->data[1];
		temp_r_ef[2] = local_COG[2]-CrdAll->data[2];

		Cross2Vd(&ZAxisAll->data[0],temp_r_ef,temp_cross);
		Ja->ValSet(22,1,-temp_cross[0]*temp_scale); //(19)
		Ja->ValSet(23,1,-temp_cross[1]*temp_scale); 
		Ja->ValSet(24,1,-temp_cross[2]*temp_scale); 

		// axis 2 LL hip roll
		Cross2Vd(&ZAxisAll->data[3],temp_r_ef,temp_cross);
		Ja->ValSet(22,2,-temp_cross[0]*temp_scale); //(20) 
		Ja->ValSet(23,2,-temp_cross[1]*temp_scale); 
		Ja->ValSet(24,2,-temp_cross[2]*temp_scale); 	

		// axis 3 LL hip pitch
		Cross2Vd(&ZAxisAll->data[6],temp_r_ef,temp_cross);
		Ja->ValSet(22,3,-temp_cross[0]*temp_scale); //(21)
		Ja->ValSet(23,3,-temp_cross[1]*temp_scale); 
		Ja->ValSet(24,3,-temp_cross[2]*temp_scale); 	

		// axis 4 LL knee pitch
		temp_scale += mass_com[0]/SumMass;
		temp_mass_sum_next += mass_com[0];
		local_COG[0] = (local_COG[0]*temp_mass_sum + pv_stack[0]*mass_com[0])/temp_mass_sum_next;
		local_COG[1] = (local_COG[1]*temp_mass_sum + pv_stack[1]*mass_com[0])/temp_mass_sum_next;
		local_COG[2] = (local_COG[2]*temp_mass_sum + pv_stack[2]*mass_com[0])/temp_mass_sum_next;

		temp_mass_sum = temp_mass_sum_next;

		temp_r_ef[0] = local_COG[0]-CrdAll->data[9];
		temp_r_ef[1] = local_COG[1]-CrdAll->data[10];
		temp_r_ef[2] = local_COG[2]-CrdAll->data[11];

		Cross2Vd(&ZAxisAll->data[9],temp_r_ef,temp_cross);
		Ja->ValSet(22,4,-temp_cross[0]*temp_scale); //(22) 
		Ja->ValSet(23,4,-temp_cross[1]*temp_scale); 
		Ja->ValSet(24,4,-temp_cross[2]*temp_scale); 

		// axis 5 LL ankle pitch
		temp_scale += mass_com[1]/SumMass;
		temp_mass_sum_next += mass_com[1];
		local_COG[0] = (local_COG[0]*temp_mass_sum + pv_stack[3]*mass_com[1])/temp_mass_sum_next;
		local_COG[1] = (local_COG[1]*temp_mass_sum + pv_stack[4]*mass_com[1])/temp_mass_sum_next;
		local_COG[2] = (local_COG[2]*temp_mass_sum + pv_stack[5]*mass_com[1])/temp_mass_sum_next;

		temp_mass_sum = temp_mass_sum_next;

		temp_r_ef[0] = local_COG[0]-CrdAll->data[12];
		temp_r_ef[1] = local_COG[1]-CrdAll->data[13];
		temp_r_ef[2] = local_COG[2]-CrdAll->data[14];

		Cross2Vd(&ZAxisAll->data[12],temp_r_ef,temp_cross);
		Ja->ValSet(22,5,-temp_cross[0]*temp_scale); //(23)
		Ja->ValSet(23,5,-temp_cross[1]*temp_scale); 
		Ja->ValSet(24,5,-temp_cross[2]*temp_scale); 

		// axis 6 LL ankle roll

		Cross2Vd(&ZAxisAll->data[15],temp_r_ef,temp_cross);
		Ja->ValSet(22,6,-temp_cross[0]*temp_scale); //(24)
		Ja->ValSet(23,6,-temp_cross[1]*temp_scale); 
		Ja->ValSet(24,6,-temp_cross[2]*temp_scale); 

		////泓逸start111228
		////讓手動不影響重心
		//for(int i = 12;i<24;i++)
		//{
		//	Ja->ValSet(22,i,0);
		//	Ja->ValSet(23,i,0);
		//	Ja->ValSet(24,i,0);
		//}
		////泓逸end111228



	}
	else if (selIK == RightSupport || selIK == DoubleSupport) // right leg support and double support
	{
		// axis 6 LL ankle roll
		temp_r_ef[0] = pv_stack[6]-CrdAll->data[15];
		temp_r_ef[1] = pv_stack[7]-CrdAll->data[16];
		temp_r_ef[2] = pv_stack[8]-CrdAll->data[17];

		temp_scale = mass_com[2]/SumMass;
		Cross2Vd(&ZAxisAll->data[15],temp_r_ef,temp_cross);
		Ja->ValSet(22,6,temp_cross[0]*temp_scale); //(1) 
		Ja->ValSet(23,6,temp_cross[1]*temp_scale); 
		Ja->ValSet(24,6,temp_cross[2]*temp_scale); 

		// axis 5 LL ankle pitch
		Cross2Vd(&ZAxisAll->data[12],temp_r_ef,temp_cross);
		Ja->ValSet(22,5,temp_cross[0]*temp_scale); //(2) 
		Ja->ValSet(23,5,temp_cross[1]*temp_scale); 
		Ja->ValSet(24,5,temp_cross[2]*temp_scale); 

		// axis 4 LL knee pitch
		temp_scale += mass_com[1]/SumMass;
		temp_mass_sum = mass_com[1] + mass_com[2];
		local_COG[0] = (pv_stack[3]*mass_com[1]+pv_stack[6]*mass_com[2])/temp_mass_sum;
		local_COG[1] = (pv_stack[4]*mass_com[1]+pv_stack[7]*mass_com[2])/temp_mass_sum;
		local_COG[2] = (pv_stack[5]*mass_com[1]+pv_stack[8]*mass_com[2])/temp_mass_sum;

		temp_r_ef[0] = local_COG[0]-CrdAll->data[9];
		temp_r_ef[1] = local_COG[1]-CrdAll->data[10];
		temp_r_ef[2] = local_COG[2]-CrdAll->data[11];
		Cross2Vd(&ZAxisAll->data[9],temp_r_ef,temp_cross);
		Ja->ValSet(22,4,temp_cross[0]*temp_scale); //(3)
		Ja->ValSet(23,4,temp_cross[1]*temp_scale); 
		Ja->ValSet(24,4,temp_cross[2]*temp_scale); 

		// axis 3 LL hip pitch
		temp_scale += mass_com[0]/SumMass;
		temp_mass_sum_next = temp_mass_sum + mass_com[0];
		local_COG[0] = (local_COG[0]*temp_mass_sum + pv_stack[0]*mass_com[0])/temp_mass_sum_next;
		local_COG[1] = (local_COG[1]*temp_mass_sum + pv_stack[1]*mass_com[0])/temp_mass_sum_next;
		local_COG[2] = (local_COG[2]*temp_mass_sum + pv_stack[2]*mass_com[0])/temp_mass_sum_next;
		temp_mass_sum = temp_mass_sum_next;
		
		temp_r_ef[0] = local_COG[0]-CrdAll->data[6];
		temp_r_ef[1] = local_COG[1]-CrdAll->data[7];
		temp_r_ef[2] = local_COG[2]-CrdAll->data[8];
		Cross2Vd(&ZAxisAll->data[6],temp_r_ef,temp_cross);
		Ja->ValSet(22,3,temp_cross[0]*temp_scale); //(4) 
		Ja->ValSet(23,3,temp_cross[1]*temp_scale); 
		Ja->ValSet(24,3,temp_cross[2]*temp_scale); 

		// axis 2 LL hip roll
		Cross2Vd(&ZAxisAll->data[3],temp_r_ef,temp_cross);
		Ja->ValSet(22,2,temp_cross[0]*temp_scale); //(5)
		Ja->ValSet(23,2,temp_cross[1]*temp_scale); 
		Ja->ValSet(24,2,temp_cross[2]*temp_scale); 

		// axis 1 LL hip yaw
		Cross2Vd(&ZAxisAll->data[0],temp_r_ef,temp_cross);
		Ja->ValSet(22,1,temp_cross[0]*temp_scale); //(6)
		Ja->ValSet(23,1,temp_cross[1]*temp_scale); 
		Ja->ValSet(24,1,temp_cross[2]*temp_scale); 


		//left arm COG Jacobian
		
		// LA axis6
		temp_r_ef[0] = pv_stack[27]-CrdAll->data[102];
		temp_r_ef[1] = pv_stack[28]-CrdAll->data[103];
		temp_r_ef[2] = pv_stack[29]-CrdAll->data[104];

		temp_scale_LA = mass_com[9]/SumMass;
		Cross2Vd(&ZAxisAll->data[102],temp_r_ef,temp_cross);
		Ja->ValSet(22,18,temp_cross[0]*temp_scale_LA); //(7)
		Ja->ValSet(23,18,temp_cross[1]*temp_scale_LA); 
		Ja->ValSet(24,18,temp_cross[2]*temp_scale_LA); 

		// LA axis5
		temp_scale_LA += mass_com[8]/SumMass;
		temp_mass_sum_LA = mass_com[8] + mass_com[9];
		local_COG_LA[0] = (pv_stack[24]*mass_com[8]+pv_stack[27]*mass_com[9])/temp_mass_sum_LA;
		local_COG_LA[1] = (pv_stack[25]*mass_com[8]+pv_stack[28]*mass_com[9])/temp_mass_sum_LA;
		local_COG_LA[2] = (pv_stack[26]*mass_com[8]+pv_stack[29]*mass_com[9])/temp_mass_sum_LA;

		temp_r_ef[0] = local_COG_LA[0]-CrdAll->data[99];
		temp_r_ef[1] = local_COG_LA[1]-CrdAll->data[100];
		temp_r_ef[2] = local_COG_LA[2]-CrdAll->data[101];
		Cross2Vd(&ZAxisAll->data[99],temp_r_ef,temp_cross);
		Ja->ValSet(22,17,temp_cross[0]*temp_scale_LA);//(8) 
		Ja->ValSet(23,17,temp_cross[1]*temp_scale_LA); 
		Ja->ValSet(24,17,temp_cross[2]*temp_scale_LA); 

		// LA axis4
		Cross2Vd(&ZAxisAll->data[96],temp_r_ef,temp_cross);
		Ja->ValSet(22,16,temp_cross[0]*temp_scale_LA);//(9) 
		Ja->ValSet(23,16,temp_cross[1]*temp_scale_LA); 
		Ja->ValSet(24,16,temp_cross[2]*temp_scale_LA);

		// LA axis3
		temp_scale_LA += mass_com[7]/SumMass;
		temp_mass_sum_next_LA = temp_mass_sum_LA + mass_com[7];
		local_COG_LA[0] = (local_COG_LA[0]*temp_mass_sum_LA + pv_stack[21]*mass_com[7])/temp_mass_sum_next_LA;
		local_COG_LA[1] = (local_COG_LA[1]*temp_mass_sum_LA + pv_stack[22]*mass_com[7])/temp_mass_sum_next_LA;
		local_COG_LA[2] = (local_COG_LA[2]*temp_mass_sum_LA + pv_stack[23]*mass_com[7])/temp_mass_sum_next_LA;
		temp_mass_sum_LA = temp_mass_sum_next_LA;

		temp_r_ef[0] = local_COG_LA[0]-CrdAll->data[93];
		temp_r_ef[1] = local_COG_LA[1]-CrdAll->data[94];
		temp_r_ef[2] = local_COG_LA[2]-CrdAll->data[95];
		Cross2Vd(&ZAxisAll->data[93],temp_r_ef,temp_cross);
		Ja->ValSet(22,15,temp_cross[0]*temp_scale_LA); //(10)
		Ja->ValSet(23,15,temp_cross[1]*temp_scale_LA); 
		Ja->ValSet(24,15,temp_cross[2]*temp_scale_LA); 

		// LA axis 2
		Cross2Vd(&ZAxisAll->data[90],temp_r_ef,temp_cross);
		Ja->ValSet(22,14,temp_cross[0]*temp_scale_LA);//(11) 
		Ja->ValSet(23,14,temp_cross[1]*temp_scale_LA); 
		Ja->ValSet(24,14,temp_cross[2]*temp_scale_LA); 

		// LA axis 1
		temp_scale_LA += mass_com[6]/SumMass;
		temp_mass_sum_next_LA = temp_mass_sum_LA + mass_com[6];
		local_COG_LA[0] = (local_COG_LA[0]*temp_mass_sum_LA + pv_stack[18]*mass_com[6])/temp_mass_sum_next_LA;
		local_COG_LA[1] = (local_COG_LA[1]*temp_mass_sum_LA + pv_stack[19]*mass_com[6])/temp_mass_sum_next_LA;
		local_COG_LA[2] = (local_COG_LA[2]*temp_mass_sum_LA + pv_stack[20]*mass_com[6])/temp_mass_sum_next_LA;
		temp_mass_sum_LA = temp_mass_sum_next_LA;

		temp_r_ef[0] = local_COG_LA[0]-CrdAll->data[87];
		temp_r_ef[1] = local_COG_LA[1]-CrdAll->data[88];
		temp_r_ef[2] = local_COG_LA[2]-CrdAll->data[89];
		Cross2Vd(&ZAxisAll->data[87],temp_r_ef,temp_cross);
		Ja->ValSet(22,13,temp_cross[0]*temp_scale_LA); //(12)
		Ja->ValSet(23,13,temp_cross[1]*temp_scale_LA); 
		Ja->ValSet(24,13,temp_cross[2]*temp_scale_LA);




		//right arm COG Jacobian

		// RA axis6
		temp_r_ef[0] = pv_stack[39]-CrdAll->data[132];
		temp_r_ef[1] = pv_stack[40]-CrdAll->data[133];
		temp_r_ef[2] = pv_stack[41]-CrdAll->data[134];

		temp_scale_RA = mass_com[13]/SumMass;
		Cross2Vd(&ZAxisAll->data[132],temp_r_ef,temp_cross);
		Ja->ValSet(22,24,temp_cross[0]*temp_scale_RA); //(13)
		Ja->ValSet(23,24,temp_cross[1]*temp_scale_RA); 
		Ja->ValSet(24,24,temp_cross[2]*temp_scale_RA); 

		// RA axis5
		temp_scale_RA += mass_com[12]/SumMass;
		temp_mass_sum_RA = mass_com[12] + mass_com[13];
		local_COG_RA[0] = (pv_stack[39]*mass_com[13]+pv_stack[36]*mass_com[12])/temp_mass_sum_RA;
		local_COG_RA[1] = (pv_stack[40]*mass_com[13]+pv_stack[37]*mass_com[12])/temp_mass_sum_RA;
		local_COG_RA[2] = (pv_stack[41]*mass_com[13]+pv_stack[38]*mass_com[12])/temp_mass_sum_RA;

		temp_r_ef[0] = local_COG_RA[0]-CrdAll->data[129];
		temp_r_ef[1] = local_COG_RA[1]-CrdAll->data[130];
		temp_r_ef[2] = local_COG_RA[2]-CrdAll->data[131];
		Cross2Vd(&ZAxisAll->data[129],temp_r_ef,temp_cross);
		Ja->ValSet(22,23,temp_cross[0]*temp_scale_RA);//(14) 
		Ja->ValSet(23,23,temp_cross[1]*temp_scale_RA); 
		Ja->ValSet(24,23,temp_cross[2]*temp_scale_RA); 

		// RA axis4
		Cross2Vd(&ZAxisAll->data[126],temp_r_ef,temp_cross);
		Ja->ValSet(22,22,temp_cross[0]*temp_scale_RA);//(15) 
		Ja->ValSet(23,22,temp_cross[1]*temp_scale_RA); 
		Ja->ValSet(24,22,temp_cross[2]*temp_scale_RA);

		// RA axis3
		temp_scale_RA += mass_com[11]/SumMass;
		temp_mass_sum_next_RA = temp_mass_sum_RA + mass_com[11];
		local_COG_RA[0] = (local_COG_RA[0]*temp_mass_sum_RA + pv_stack[33]*mass_com[11])/temp_mass_sum_next_RA;
		local_COG_RA[1] = (local_COG_RA[1]*temp_mass_sum_RA + pv_stack[34]*mass_com[11])/temp_mass_sum_next_RA;
		local_COG_RA[2] = (local_COG_RA[2]*temp_mass_sum_RA + pv_stack[35]*mass_com[11])/temp_mass_sum_next_RA;
		temp_mass_sum_RA = temp_mass_sum_next_RA;

		temp_r_ef[0] = local_COG_RA[0]-CrdAll->data[123];
		temp_r_ef[1] = local_COG_RA[1]-CrdAll->data[124];
		temp_r_ef[2] = local_COG_RA[2]-CrdAll->data[125];
		Cross2Vd(&ZAxisAll->data[123],temp_r_ef,temp_cross);
		Ja->ValSet(22,21,temp_cross[0]*temp_scale_RA); //(16)
		Ja->ValSet(23,21,temp_cross[1]*temp_scale_RA); 
		Ja->ValSet(24,21,temp_cross[2]*temp_scale_RA); 

		// RA axis 2
		Cross2Vd(&ZAxisAll->data[120],temp_r_ef,temp_cross);
		Ja->ValSet(22,20,temp_cross[0]*temp_scale_RA);//(17) 
		Ja->ValSet(23,20,temp_cross[1]*temp_scale_RA); 
		Ja->ValSet(24,20,temp_cross[2]*temp_scale_RA); 

		// RA axis 1
		temp_scale_RA += mass_com[10]/SumMass;
		temp_mass_sum_next_RA = temp_mass_sum_RA + mass_com[10];
		local_COG_RA[0] = (local_COG_RA[0]*temp_mass_sum_RA + pv_stack[30]*mass_com[10])/temp_mass_sum_next_RA;
		local_COG_RA[1] = (local_COG_RA[1]*temp_mass_sum_RA + pv_stack[31]*mass_com[10])/temp_mass_sum_next_RA;
		local_COG_RA[2] = (local_COG_RA[2]*temp_mass_sum_RA + pv_stack[32]*mass_com[10])/temp_mass_sum_next_RA;
		temp_mass_sum_RA = temp_mass_sum_next_RA;

		temp_r_ef[0] = local_COG_RA[0]-CrdAll->data[117];
		temp_r_ef[1] = local_COG_RA[1]-CrdAll->data[118];
		temp_r_ef[2] = local_COG_RA[2]-CrdAll->data[119];
		Cross2Vd(&ZAxisAll->data[117],temp_r_ef,temp_cross);
		Ja->ValSet(22,19,temp_cross[0]*temp_scale_RA); //(18)
		Ja->ValSet(23,19,temp_cross[1]*temp_scale_RA); 
		Ja->ValSet(24,19,temp_cross[2]*temp_scale_RA);








		// fixed leg Jacobian : axis 7~12
		// axis 7 RL hip yaw include body, not including arms
		temp_scale += temp_scale_LA + temp_scale_RA +( mass_com[14] + mass_com[15])/SumMass;
		temp_mass_sum_next += (temp_mass_sum_LA + temp_mass_sum_RA + mass_com[14] + mass_com[15]);
		local_COG[0] = (local_COG[0]*temp_mass_sum + local_COG_LA[0]*temp_mass_sum_LA + local_COG_RA[0]*temp_mass_sum_RA + pv_stack[42]*mass_com[14] + pv_stack[45]*mass_com[15])/temp_mass_sum_next;
		local_COG[1] = (local_COG[1]*temp_mass_sum + local_COG_LA[1]*temp_mass_sum_LA + local_COG_RA[1]*temp_mass_sum_RA + pv_stack[43]*mass_com[14] + pv_stack[46]*mass_com[15])/temp_mass_sum_next;
		local_COG[2] = (local_COG[2]*temp_mass_sum + local_COG_LA[2]*temp_mass_sum_LA + local_COG_RA[2]*temp_mass_sum_RA + pv_stack[44]*mass_com[14] + pv_stack[47]*mass_com[15])/temp_mass_sum_next;
		temp_mass_sum = temp_mass_sum_next;

		temp_r_ef[0] = local_COG[0]-CrdAll->data[39];
		temp_r_ef[1] = local_COG[1]-CrdAll->data[40];
		temp_r_ef[2] = local_COG[2]-CrdAll->data[41];

		Cross2Vd(&ZAxisAll->data[39],temp_r_ef,temp_cross);
		Ja->ValSet(22,7,-temp_cross[0]*temp_scale); //(19) 
		Ja->ValSet(23,7,-temp_cross[1]*temp_scale); 
		Ja->ValSet(24,7,-temp_cross[2]*temp_scale); 

		// axis 8 RL hip roll
		Cross2Vd(&ZAxisAll->data[42],temp_r_ef,temp_cross);
		Ja->ValSet(22,8,-temp_cross[0]*temp_scale); //(20)
		Ja->ValSet(23,8,-temp_cross[1]*temp_scale); 
		Ja->ValSet(24,8,-temp_cross[2]*temp_scale); 	

		// axis 9 RL hip pitch
		Cross2Vd(&ZAxisAll->data[45],temp_r_ef,temp_cross);
		Ja->ValSet(22,9,-temp_cross[0]*temp_scale); //(21)
		Ja->ValSet(23,9,-temp_cross[1]*temp_scale); 
		Ja->ValSet(24,9,-temp_cross[2]*temp_scale); 	

		// axis 10 RL knee pitch
		temp_scale += mass_com[3]/SumMass;
		temp_mass_sum_next += mass_com[3];
		local_COG[0] = (local_COG[0]*temp_mass_sum + pv_stack[9]*mass_com[3])/temp_mass_sum_next;
		local_COG[1] = (local_COG[1]*temp_mass_sum + pv_stack[10]*mass_com[3])/temp_mass_sum_next;
		local_COG[2] = (local_COG[2]*temp_mass_sum + pv_stack[11]*mass_com[3])/temp_mass_sum_next;

		temp_mass_sum = temp_mass_sum_next;

		temp_r_ef[0] = local_COG[0]-CrdAll->data[48];
		temp_r_ef[1] = local_COG[1]-CrdAll->data[49];
		temp_r_ef[2] = local_COG[2]-CrdAll->data[50];

		Cross2Vd(&ZAxisAll->data[48],temp_r_ef,temp_cross);
		Ja->ValSet(22,10,-temp_cross[0]*temp_scale); //(22) 
		Ja->ValSet(23,10,-temp_cross[1]*temp_scale); 
		Ja->ValSet(24,10,-temp_cross[2]*temp_scale); 

		// axis 11 LL ankle pitch
		temp_scale += mass_com[4]/SumMass;
		temp_mass_sum_next += mass_com[4];
		local_COG[0] = (local_COG[0]*temp_mass_sum + pv_stack[12]*mass_com[4])/temp_mass_sum_next;
		local_COG[1] = (local_COG[1]*temp_mass_sum + pv_stack[13]*mass_com[4])/temp_mass_sum_next;
		local_COG[2] = (local_COG[2]*temp_mass_sum + pv_stack[14]*mass_com[4])/temp_mass_sum_next;

		temp_mass_sum = temp_mass_sum_next;

		temp_r_ef[0] = local_COG[0]-CrdAll->data[51];
		temp_r_ef[1] = local_COG[1]-CrdAll->data[52];
		temp_r_ef[2] = local_COG[2]-CrdAll->data[53];

		Cross2Vd(&ZAxisAll->data[51],temp_r_ef,temp_cross);
		Ja->ValSet(22,11,-temp_cross[0]*temp_scale); //(23)
		Ja->ValSet(23,11,-temp_cross[1]*temp_scale); 
		Ja->ValSet(24,11,-temp_cross[2]*temp_scale); 

		// axis 6 LL ankle roll

		Cross2Vd(&ZAxisAll->data[54],temp_r_ef,temp_cross);
		Ja->ValSet(22,12,-temp_cross[0]*temp_scale); //(24) 
		Ja->ValSet(23,12,-temp_cross[1]*temp_scale); 
		Ja->ValSet(24,12,-temp_cross[2]*temp_scale); 


		//泓逸start111228
		//讓手動不影響重心
		/*for(int i = 13;i<25;i++)
		{
			Ja->ValSet(22,i,0);
			Ja->ValSet(23,i,0);
			Ja->ValSet(24,i,0);
		}*/
		//泓逸end111228


	}
		//泓逸end11122

}

void Kine::GetCOGFixJacobian(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 計算COG jacobian
	******************************************************************/

	// dth => [左腳6軸 ; 右腳6軸]
	// selIK = 0 : left foot is the supporter
    // selIK = 1 : right foot is the supporter

	// when selIK = 0, dx = [sw_x sw_y sw_z sw_tx sw_ty sw_tz fix_tx fix_ty fix_tz COGx COGy COGz]'

	// 算出各個linkage的重量乘上其在世界中的位置
	// 求出後 直接利用會影響當前累積質量乘上位置之值 計算COG Jacobian
	// 詳細公式與推導請參見論文
	if (selIK == 0) // selIK = 0 : left foot is the supporter  
	{
		//// axis 12 RL ankle roll
		//temp_r_ef[0] = pv_stack[15]-CrdAll->data[54];
		//temp_r_ef[1] = pv_stack[16]-CrdAll->data[55];
		//temp_r_ef[2] = pv_stack[17]-CrdAll->data[56];

		temp_scale = mass_com[5]/SumMass;
		//Cross2Vd(&ZAxisAll->data[54],temp_r_ef,temp_cross);
		//Ja->ValSet(10,12,temp_cross[0]*temp_scale); 
		//Ja->ValSet(11,12,temp_cross[1]*temp_scale); 
		//Ja->ValSet(12,12,temp_cross[2]*temp_scale); 

		//// axis 11 RL ankle pitch
		//Cross2Vd(&ZAxisAll->data[51],temp_r_ef,temp_cross);
		//Ja->ValSet(10,11,temp_cross[0]*temp_scale); 
		//Ja->ValSet(11,11,temp_cross[1]*temp_scale); 
		//Ja->ValSet(12,11,temp_cross[2]*temp_scale); 

		//// axis 10 RL knee pitch
		temp_scale += mass_com[4]/SumMass;
		temp_mass_sum = mass_com[4] + mass_com[5];
		//local_COG[0] = (pv_stack[12]*mass_com[4]+pv_stack[15]*mass_com[5])/temp_mass_sum;
		//local_COG[1] = (pv_stack[13]*mass_com[4]+pv_stack[16]*mass_com[5])/temp_mass_sum;
		//local_COG[2] = (pv_stack[14]*mass_com[4]+pv_stack[17]*mass_com[5])/temp_mass_sum;

		//temp_r_ef[0] = local_COG[0]-CrdAll->data[48];
		//temp_r_ef[1] = local_COG[1]-CrdAll->data[49];
		//temp_r_ef[2] = local_COG[2]-CrdAll->data[50];
		//Cross2Vd(&ZAxisAll->data[48],temp_r_ef,temp_cross);
		//Ja->ValSet(10,10,temp_cross[0]*temp_scale); 
		//Ja->ValSet(11,10,temp_cross[1]*temp_scale); 
		//Ja->ValSet(12,10,temp_cross[2]*temp_scale); 

		//// axis 9 RL hip pitch
		temp_scale += mass_com[3]/SumMass;
		temp_mass_sum_next = temp_mass_sum + mass_com[3];
		//local_COG[0] = (local_COG[0]*temp_mass_sum + pv_stack[9]*mass_com[3])/temp_mass_sum_next;
		//local_COG[1] = (local_COG[1]*temp_mass_sum + pv_stack[10]*mass_com[3])/temp_mass_sum_next;
		//local_COG[2] = (local_COG[2]*temp_mass_sum + pv_stack[11]*mass_com[3])/temp_mass_sum_next;
		temp_mass_sum = temp_mass_sum_next;
		//
		//temp_r_ef[0] = local_COG[0]-CrdAll->data[45];
		//temp_r_ef[1] = local_COG[1]-CrdAll->data[46];
		//temp_r_ef[2] = local_COG[2]-CrdAll->data[47];
		//Cross2Vd(&ZAxisAll->data[45],temp_r_ef,temp_cross);
		//Ja->ValSet(10,9,temp_cross[0]*temp_scale); 
		//Ja->ValSet(11,9,temp_cross[1]*temp_scale); 
		//Ja->ValSet(12,9,temp_cross[2]*temp_scale); 

		//// axis 8 RL hip roll
		//Cross2Vd(&ZAxisAll->data[42],temp_r_ef,temp_cross);
		//Ja->ValSet(10,8,temp_cross[0]*temp_scale); 
		//Ja->ValSet(11,8,temp_cross[1]*temp_scale); 
		//Ja->ValSet(12,8,temp_cross[2]*temp_scale); 

		//// axis 7 RL hip yaw
		//Cross2Vd(&ZAxisAll->data[39],temp_r_ef,temp_cross);
		//Ja->ValSet(10,7,temp_cross[0]*temp_scale); 
		//Ja->ValSet(11,7,temp_cross[1]*temp_scale); 
		//Ja->ValSet(12,7,temp_cross[2]*temp_scale); 


		// fixed leg Jacobian : axis 1~6
		// axis 1 LL hip yaw include body and arms
		//////temp_scale += mass_com[8]/SumMass + mass_com[9]/SumMass;
		//////temp_mass_sum_next += mass_com[8] + mass_com[9];
		//////local_COG[0] = (local_COG[0]*temp_mass_sum + pv_stack[24]*mass_com[8]+ pv_stack[27]*mass_com[9])/temp_mass_sum_next;
		//////local_COG[1] = (local_COG[1]*temp_mass_sum + pv_stack[25]*mass_com[8]+ pv_stack[28]*mass_com[9])/temp_mass_sum_next;
		//////local_COG[2] = (local_COG[2]*temp_mass_sum + pv_stack[26]*mass_com[8]+ pv_stack[29]*mass_com[9])/temp_mass_sum_next;
		//////temp_mass_sum = temp_mass_sum_next;

		// LA axis6
		//temp_r_ef[0] = pv_stack[27]-CrdAll->data[102];
		//temp_r_ef[1] = pv_stack[28]-CrdAll->data[103];
		//temp_r_ef[2] = pv_stack[29]-CrdAll->data[104];

		temp_scale_LA = mass_com[9]/SumMass;
		//Cross2Vd(&ZAxisAll->data[102],temp_r_ef,temp_cross);
		//Ja->ValSet(22,18,temp_cross[0]*temp_scale_LA); //(7)
		//Ja->ValSet(23,18,temp_cross[1]*temp_scale_LA); 
		//Ja->ValSet(24,18,temp_cross[2]*temp_scale_LA); 

		// LA axis5
		temp_scale_LA += mass_com[8]/SumMass;
		temp_mass_sum_LA = mass_com[8] + mass_com[9];
		//local_COG_LA[0] = (pv_stack[24]*mass_com[8]+pv_stack[27]*mass_com[9])/temp_mass_sum_LA;
		//local_COG_LA[1] = (pv_stack[25]*mass_com[8]+pv_stack[28]*mass_com[9])/temp_mass_sum_LA;
		//local_COG_LA[2] = (pv_stack[26]*mass_com[8]+pv_stack[29]*mass_com[9])/temp_mass_sum_LA;

		//temp_r_ef[0] = local_COG_LA[0]-CrdAll->data[99];
		//temp_r_ef[1] = local_COG_LA[1]-CrdAll->data[100];
		//temp_r_ef[2] = local_COG_LA[2]-CrdAll->data[101];
		//Cross2Vd(&ZAxisAll->data[99],temp_r_ef,temp_cross);
		//Ja->ValSet(22,17,temp_cross[0]*temp_scale_LA);//(8) 
		//Ja->ValSet(23,17,temp_cross[1]*temp_scale_LA); 
		//Ja->ValSet(24,17,temp_cross[2]*temp_scale_LA); 

		// LA axis4
		//Cross2Vd(&ZAxisAll->data[96],temp_r_ef,temp_cross);
		//Ja->ValSet(22,16,temp_cross[0]*temp_scale_LA);//(9) 
		//Ja->ValSet(23,16,temp_cross[1]*temp_scale_LA); 
		//Ja->ValSet(24,16,temp_cross[2]*temp_scale_LA);

		// LA axis3
		temp_scale_LA += mass_com[7]/SumMass;
		temp_mass_sum_next_LA = temp_mass_sum_LA + mass_com[7];
		//local_COG_LA[0] = (local_COG_LA[0]*temp_mass_sum_LA + pv_stack[21]*mass_com[7])/temp_mass_sum_next_LA;
		//local_COG_LA[1] = (local_COG_LA[1]*temp_mass_sum_LA + pv_stack[22]*mass_com[7])/temp_mass_sum_next_LA;
		//local_COG_LA[2] = (local_COG_LA[2]*temp_mass_sum_LA + pv_stack[23]*mass_com[7])/temp_mass_sum_next_LA;
		temp_mass_sum_LA = temp_mass_sum_next_LA;

		//temp_r_ef[0] = local_COG_LA[0]-CrdAll->data[93];
		//temp_r_ef[1] = local_COG_LA[1]-CrdAll->data[94];
		//temp_r_ef[2] = local_COG_LA[2]-CrdAll->data[95];
		//Cross2Vd(&ZAxisAll->data[93],temp_r_ef,temp_cross);
		//Ja->ValSet(22,15,temp_cross[0]*temp_scale_LA); //(10)
		//Ja->ValSet(23,15,temp_cross[1]*temp_scale_LA); 
		//Ja->ValSet(24,15,temp_cross[2]*temp_scale_LA); 

		// LA axis 2
		//Cross2Vd(&ZAxisAll->data[90],temp_r_ef,temp_cross);
		//Ja->ValSet(22,14,temp_cross[0]*temp_scale_LA);//(11) 
		//Ja->ValSet(23,14,temp_cross[1]*temp_scale_LA); 
		//Ja->ValSet(24,14,temp_cross[2]*temp_scale_LA); 

		// LA axis 1
		temp_scale_LA += mass_com[6]/SumMass;
		temp_mass_sum_next_LA = temp_mass_sum_LA + mass_com[6];
		//local_COG_LA[0] = (local_COG_LA[0]*temp_mass_sum_LA + pv_stack[18]*mass_com[6])/temp_mass_sum_next_LA;
		//local_COG_LA[1] = (local_COG_LA[1]*temp_mass_sum_LA + pv_stack[19]*mass_com[6])/temp_mass_sum_next_LA;
		//local_COG_LA[2] = (local_COG_LA[2]*temp_mass_sum_LA + pv_stack[20]*mass_com[6])/temp_mass_sum_next_LA;
		temp_mass_sum_LA = temp_mass_sum_next_LA;

		//temp_r_ef[0] = local_COG_LA[0]-CrdAll->data[87];
		//temp_r_ef[1] = local_COG_LA[1]-CrdAll->data[88];
		//temp_r_ef[2] = local_COG_LA[2]-CrdAll->data[89];
		//Cross2Vd(&ZAxisAll->data[87],temp_r_ef,temp_cross);
		//Ja->ValSet(22,13,temp_cross[0]*temp_scale_LA); //(12)
		//Ja->ValSet(23,13,temp_cross[1]*temp_scale_LA); 
		//Ja->ValSet(24,13,temp_cross[2]*temp_scale_LA);




		//right arm COG Jacobian

		// RA axis6
		//temp_r_ef[0] = pv_stack[39]-CrdAll->data[132];
		//temp_r_ef[1] = pv_stack[40]-CrdAll->data[133];
		//temp_r_ef[2] = pv_stack[41]-CrdAll->data[134];

		temp_scale_RA = mass_com[13]/SumMass;
		//Cross2Vd(&ZAxisAll->data[132],temp_r_ef,temp_cross);
		//Ja->ValSet(22,24,temp_cross[0]*temp_scale_RA); //(13)
		//Ja->ValSet(23,24,temp_cross[1]*temp_scale_RA); 
		//Ja->ValSet(24,24,temp_cross[2]*temp_scale_RA); 

		// RA axis5
		temp_scale_RA += mass_com[12]/SumMass;
		temp_mass_sum_RA = mass_com[12] + mass_com[13];
		//local_COG_RA[0] = (pv_stack[39]*mass_com[13]+pv_stack[36]*mass_com[12])/temp_mass_sum_RA;
		//local_COG_RA[1] = (pv_stack[40]*mass_com[13]+pv_stack[37]*mass_com[12])/temp_mass_sum_RA;
		//local_COG_RA[2] = (pv_stack[41]*mass_com[13]+pv_stack[38]*mass_com[12])/temp_mass_sum_RA;

		//temp_r_ef[0] = local_COG_RA[0]-CrdAll->data[129];
		//temp_r_ef[1] = local_COG_RA[1]-CrdAll->data[130];
		//temp_r_ef[2] = local_COG_RA[2]-CrdAll->data[131];
		//Cross2Vd(&ZAxisAll->data[129],temp_r_ef,temp_cross);
		//Ja->ValSet(22,23,temp_cross[0]*temp_scale_RA);//(14) 
		//Ja->ValSet(23,23,temp_cross[1]*temp_scale_RA); 
		//Ja->ValSet(24,23,temp_cross[2]*temp_scale_RA); 

		// RA axis4
		//Cross2Vd(&ZAxisAll->data[126],temp_r_ef,temp_cross);
		//Ja->ValSet(22,22,temp_cross[0]*temp_scale_RA);//(15) 
		//Ja->ValSet(23,22,temp_cross[1]*temp_scale_RA); 
		//Ja->ValSet(24,22,temp_cross[2]*temp_scale_RA);

		// RA axis3
		temp_scale_RA += mass_com[11]/SumMass;
		temp_mass_sum_next_RA = temp_mass_sum_RA + mass_com[11];
		//local_COG_RA[0] = (local_COG_RA[0]*temp_mass_sum_RA + pv_stack[33]*mass_com[11])/temp_mass_sum_next_RA;
		//local_COG_RA[1] = (local_COG_RA[1]*temp_mass_sum_RA + pv_stack[34]*mass_com[11])/temp_mass_sum_next_RA;
		//local_COG_RA[2] = (local_COG_RA[2]*temp_mass_sum_RA + pv_stack[35]*mass_com[11])/temp_mass_sum_next_RA;
		temp_mass_sum_RA = temp_mass_sum_next_RA;

		//temp_r_ef[0] = local_COG_RA[0]-CrdAll->data[123];
		//temp_r_ef[1] = local_COG_RA[1]-CrdAll->data[124];
		//temp_r_ef[2] = local_COG_RA[2]-CrdAll->data[125];
		//Cross2Vd(&ZAxisAll->data[123],temp_r_ef,temp_cross);
		//Ja->ValSet(22,21,temp_cross[0]*temp_scale_RA); //(16)
		//Ja->ValSet(23,21,temp_cross[1]*temp_scale_RA); 
		//Ja->ValSet(24,21,temp_cross[2]*temp_scale_RA); 

		// RA axis 2
		//Cross2Vd(&ZAxisAll->data[120],temp_r_ef,temp_cross);
		//Ja->ValSet(22,20,temp_cross[0]*temp_scale_RA);//(17) 
		//Ja->ValSet(23,20,temp_cross[1]*temp_scale_RA); 
		//Ja->ValSet(24,20,temp_cross[2]*temp_scale_RA); 

		// RA axis 1
		temp_scale_RA += mass_com[10]/SumMass;
		temp_mass_sum_next_RA = temp_mass_sum_RA + mass_com[10];
		//local_COG_RA[0] = (local_COG_RA[0]*temp_mass_sum_RA + pv_stack[30]*mass_com[10])/temp_mass_sum_next_RA;
		//local_COG_RA[1] = (local_COG_RA[1]*temp_mass_sum_RA + pv_stack[31]*mass_com[10])/temp_mass_sum_next_RA;
		//local_COG_RA[2] = (local_COG_RA[2]*temp_mass_sum_RA + pv_stack[32]*mass_com[10])/temp_mass_sum_next_RA;
		temp_mass_sum_RA = temp_mass_sum_next_RA;

		//temp_r_ef[0] = local_COG_RA[0]-CrdAll->data[117];
		//temp_r_ef[1] = local_COG_RA[1]-CrdAll->data[118];
		//temp_r_ef[2] = local_COG_RA[2]-CrdAll->data[119];
		//Cross2Vd(&ZAxisAll->data[117],temp_r_ef,temp_cross);
		//Ja->ValSet(22,19,temp_cross[0]*temp_scale_RA); //(18)
		//Ja->ValSet(23,19,temp_cross[1]*temp_scale_RA); 
		//Ja->ValSet(24,19,temp_cross[2]*temp_scale_RA);


		// fixed leg Jacobian : axis 1~6
		// axis 1 LL hip yaw include body and arms
		//////temp_scale += mass_com[8]/SumMass + mass_com[9]/SumMass;
		//////temp_mass_sum_next += mass_com[8] + mass_com[9];
		//////local_COG[0] = (local_COG[0]*temp_mass_sum + pv_stack[24]*mass_com[8]+ pv_stack[27]*mass_com[9])/temp_mass_sum_next;
		//////local_COG[1] = (local_COG[1]*temp_mass_sum + pv_stack[25]*mass_com[8]+ pv_stack[28]*mass_com[9])/temp_mass_sum_next;
		//////local_COG[2] = (local_COG[2]*temp_mass_sum + pv_stack[26]*mass_com[8]+ pv_stack[29]*mass_com[9])/temp_mass_sum_next;
		//////temp_mass_sum = temp_mass_sum_next;

		temp_scale += temp_scale_LA + temp_scale_RA +( mass_com[14] + mass_com[15])/SumMass;
		temp_mass_sum_next += temp_mass_sum_LA + temp_mass_sum_RA + mass_com[14] + mass_com[15];
		local_COG[0] = (local_COG[0]*temp_mass_sum + local_COG_LA[0]*temp_mass_sum_LA + local_COG_RA[0]*temp_mass_sum_RA + pv_stack[42]*mass_com[14] + pv_stack[45]*mass_com[15])/temp_mass_sum_next;
		local_COG[1] = (local_COG[1]*temp_mass_sum + local_COG_LA[1]*temp_mass_sum_LA + local_COG_RA[1]*temp_mass_sum_RA + pv_stack[43]*mass_com[14] + pv_stack[46]*mass_com[15])/temp_mass_sum_next;
		local_COG[2] = (local_COG[2]*temp_mass_sum + local_COG_LA[2]*temp_mass_sum_LA + local_COG_RA[2]*temp_mass_sum_RA + pv_stack[44]*mass_com[14] + pv_stack[47]*mass_com[15])/temp_mass_sum_next;
		temp_mass_sum = temp_mass_sum_next;


		temp_r_ef[0] = local_COG[0]-CrdAll->data[0];
		temp_r_ef[1] = local_COG[1]-CrdAll->data[1];
		temp_r_ef[2] = local_COG[2]-CrdAll->data[2];

		//Cross2Vd(&ZAxisAll->data[0],temp_r_ef,temp_cross);
		//Ja->ValSet(22,1,-temp_cross[0]*temp_scale); //(19)
		//Ja->ValSet(23,1,-temp_cross[1]*temp_scale); 
		//Ja->ValSet(24,1,-temp_cross[2]*temp_scale); 




		//temp_scale += (mass_com[6] + mass_com[7] + mass_com[8] + mass_com[9])/SumMass;
		//temp_mass_sum_next += mass_com[6] + mass_com[7] + mass_com[8] + mass_com[9];
		//local_COG[0] = (local_COG[0]*temp_mass_sum + pv_stack[18]*mass_com[6] + pv_stack[21]*mass_com[7] + pv_stack[24]*mass_com[8] + pv_stack[27]*mass_com[9])/temp_mass_sum_next;
		//local_COG[1] = (local_COG[1]*temp_mass_sum + pv_stack[19]*mass_com[6] + pv_stack[22]*mass_com[7] + pv_stack[25]*mass_com[8] + pv_stack[28]*mass_com[9])/temp_mass_sum_next;
		//local_COG[2] = (local_COG[2]*temp_mass_sum + pv_stack[20]*mass_com[6] + pv_stack[23]*mass_com[7] + pv_stack[26]*mass_com[8] + pv_stack[29]*mass_com[9])/temp_mass_sum_next;
		//temp_mass_sum = temp_mass_sum_next;

		//temp_r_ef[0] = local_COG[0]-CrdAll->data[0];
		//temp_r_ef[1] = local_COG[1]-CrdAll->data[1];
		//temp_r_ef[2] = local_COG[2]-CrdAll->data[2];

		Cross2Vd(&ZAxisAll->data[0],temp_r_ef,temp_cross);
		FixJa->ValSet(4,1,-temp_cross[0]*temp_scale); 
		FixJa->ValSet(5,1,-temp_cross[1]*temp_scale); 
		FixJa->ValSet(6,1,-temp_cross[2]*temp_scale); 

		// axis 2 LL hip roll
		Cross2Vd(&ZAxisAll->data[3],temp_r_ef,temp_cross);
		FixJa->ValSet(4,2,-temp_cross[0]*temp_scale); 
		FixJa->ValSet(5,2,-temp_cross[1]*temp_scale); 
		FixJa->ValSet(6,2,-temp_cross[2]*temp_scale); 	

		// axis 3 LL hip pitch
		Cross2Vd(&ZAxisAll->data[6],temp_r_ef,temp_cross);
		FixJa->ValSet(4,3,-temp_cross[0]*temp_scale); 
		FixJa->ValSet(5,3,-temp_cross[1]*temp_scale); 
		FixJa->ValSet(6,3,-temp_cross[2]*temp_scale); 	

		// axis 4 LL knee pitch
		temp_scale += mass_com[0]/SumMass;
		temp_mass_sum_next += mass_com[0];
		local_COG[0] = (local_COG[0]*temp_mass_sum + pv_stack[0]*mass_com[0])/temp_mass_sum_next;
		local_COG[1] = (local_COG[1]*temp_mass_sum + pv_stack[1]*mass_com[0])/temp_mass_sum_next;
		local_COG[2] = (local_COG[2]*temp_mass_sum + pv_stack[2]*mass_com[0])/temp_mass_sum_next;

		temp_mass_sum = temp_mass_sum_next;

		temp_r_ef[0] = local_COG[0]-CrdAll->data[9];
		temp_r_ef[1] = local_COG[1]-CrdAll->data[10];
		temp_r_ef[2] = local_COG[2]-CrdAll->data[11];

		Cross2Vd(&ZAxisAll->data[9],temp_r_ef,temp_cross);
		FixJa->ValSet(4,4,-temp_cross[0]*temp_scale); 
		FixJa->ValSet(5,4,-temp_cross[1]*temp_scale); 
		FixJa->ValSet(6,4,-temp_cross[2]*temp_scale); 

		// axis 5 LL ankle pitch
		temp_scale += mass_com[1]/SumMass;
		temp_mass_sum_next += mass_com[1];
		local_COG[0] = (local_COG[0]*temp_mass_sum + pv_stack[3]*mass_com[1])/temp_mass_sum_next;
		local_COG[1] = (local_COG[1]*temp_mass_sum + pv_stack[4]*mass_com[1])/temp_mass_sum_next;
		local_COG[2] = (local_COG[2]*temp_mass_sum + pv_stack[5]*mass_com[1])/temp_mass_sum_next;

		temp_mass_sum = temp_mass_sum_next;

		temp_r_ef[0] = local_COG[0]-CrdAll->data[12];
		temp_r_ef[1] = local_COG[1]-CrdAll->data[13];
		temp_r_ef[2] = local_COG[2]-CrdAll->data[14];

		Cross2Vd(&ZAxisAll->data[12],temp_r_ef,temp_cross);
		FixJa->ValSet(4,5,-temp_cross[0]*temp_scale); 
		FixJa->ValSet(5,5,-temp_cross[1]*temp_scale); 
		FixJa->ValSet(6,5,-temp_cross[2]*temp_scale); 

		// axis 6 LL ankle roll

		Cross2Vd(&ZAxisAll->data[15],temp_r_ef,temp_cross);
		FixJa->ValSet(4,6,-temp_cross[0]*temp_scale); 
		FixJa->ValSet(5,6,-temp_cross[1]*temp_scale); 
		FixJa->ValSet(6,6,-temp_cross[2]*temp_scale); 

	}
	else if (selIK == 1 || selIK == 2) // right leg support and double support
	{
		// axis 6 LL ankle roll
		//temp_r_ef[0] = pv_stack[6]-CrdAll->data[15];
		//temp_r_ef[1] = pv_stack[7]-CrdAll->data[16];
		//temp_r_ef[2] = pv_stack[8]-CrdAll->data[17];

		temp_scale = mass_com[2]/SumMass;
/*		Cross2Vd(&ZAxisAll->data[15],temp_r_ef,temp_cross);
		Ja->ValSet(10,6,temp_cross[0]*temp_scale); 
		Ja->ValSet(11,6,temp_cross[1]*temp_scale); 
		Ja->ValSet(12,6,temp_cross[2]*temp_scale);*/ 

		// axis 5 LL ankle pitch
		//Cross2Vd(&ZAxisAll->data[12],temp_r_ef,temp_cross);
		//Ja->ValSet(10,5,temp_cross[0]*temp_scale); 
		//Ja->ValSet(11,5,temp_cross[1]*temp_scale); 
		//Ja->ValSet(12,5,temp_cross[2]*temp_scale); 

		// axis 4 LL knee pitch
		temp_scale += mass_com[1]/SumMass;
		temp_mass_sum = mass_com[1] + mass_com[2];
		//local_COG[0] = (pv_stack[3]*mass_com[1]+pv_stack[6]*mass_com[2])/temp_mass_sum;
		//local_COG[1] = (pv_stack[4]*mass_com[1]+pv_stack[7]*mass_com[2])/temp_mass_sum;
		//local_COG[2] = (pv_stack[5]*mass_com[1]+pv_stack[8]*mass_com[2])/temp_mass_sum;

		//temp_r_ef[0] = local_COG[0]-CrdAll->data[9];
		//temp_r_ef[1] = local_COG[1]-CrdAll->data[10];
		//temp_r_ef[2] = local_COG[2]-CrdAll->data[11];
		//Cross2Vd(&ZAxisAll->data[9],temp_r_ef,temp_cross);
		//Ja->ValSet(10,4,temp_cross[0]*temp_scale); 
		//Ja->ValSet(11,4,temp_cross[1]*temp_scale); 
		//Ja->ValSet(12,4,temp_cross[2]*temp_scale); 

		// axis 3 LL hip pitch
		temp_scale += mass_com[0]/SumMass;
		temp_mass_sum_next = temp_mass_sum + mass_com[0];
		//local_COG[0] = (local_COG[0]*temp_mass_sum + pv_stack[0]*mass_com[0])/temp_mass_sum_next;
		//local_COG[1] = (local_COG[1]*temp_mass_sum + pv_stack[1]*mass_com[0])/temp_mass_sum_next;
		//local_COG[2] = (local_COG[2]*temp_mass_sum + pv_stack[2]*mass_com[0])/temp_mass_sum_next;
		temp_mass_sum = temp_mass_sum_next;
		
		//temp_r_ef[0] = local_COG[0]-CrdAll->data[6];
		//temp_r_ef[1] = local_COG[1]-CrdAll->data[7];
		//temp_r_ef[2] = local_COG[2]-CrdAll->data[8];
		//Cross2Vd(&ZAxisAll->data[6],temp_r_ef,temp_cross);
		//Ja->ValSet(10,3,temp_cross[0]*temp_scale); 
		//Ja->ValSet(11,3,temp_cross[1]*temp_scale); 
		//Ja->ValSet(12,3,temp_cross[2]*temp_scale); 

		// axis 2 LL hip roll
		//Cross2Vd(&ZAxisAll->data[3],temp_r_ef,temp_cross);
		//Ja->ValSet(10,2,temp_cross[0]*temp_scale); 
		//Ja->ValSet(11,2,temp_cross[1]*temp_scale); 
		//Ja->ValSet(12,2,temp_cross[2]*temp_scale); 

		// axis 1 LL hip yaw
		//Cross2Vd(&ZAxisAll->data[0],temp_r_ef,temp_cross);
		//Ja->ValSet(10,1,temp_cross[0]*temp_scale); 
		//Ja->ValSet(11,1,temp_cross[1]*temp_scale); 
		//Ja->ValSet(12,1,temp_cross[2]*temp_scale); 

		//left arm COG Jacobian
		
		// LA axis6
		//temp_r_ef[0] = pv_stack[27]-CrdAll->data[102];
		//temp_r_ef[1] = pv_stack[28]-CrdAll->data[103];
		//temp_r_ef[2] = pv_stack[29]-CrdAll->data[104];

		temp_scale_LA = mass_com[9]/SumMass;
		//Cross2Vd(&ZAxisAll->data[102],temp_r_ef,temp_cross);
		//Ja->ValSet(22,18,temp_cross[0]*temp_scale_LA); //(7)
		//Ja->ValSet(23,18,temp_cross[1]*temp_scale_LA); 
		//Ja->ValSet(24,18,temp_cross[2]*temp_scale_LA); 

		// LA axis5
		temp_scale_LA += mass_com[8]/SumMass;
		temp_mass_sum_LA = mass_com[8] + mass_com[9];
		//local_COG_LA[0] = (pv_stack[24]*mass_com[8]+pv_stack[27]*mass_com[9])/temp_mass_sum_LA;
		//local_COG_LA[1] = (pv_stack[25]*mass_com[8]+pv_stack[28]*mass_com[9])/temp_mass_sum_LA;
		//local_COG_LA[2] = (pv_stack[26]*mass_com[8]+pv_stack[29]*mass_com[9])/temp_mass_sum_LA;

		//temp_r_ef[0] = local_COG_LA[0]-CrdAll->data[99];
		//temp_r_ef[1] = local_COG_LA[1]-CrdAll->data[100];
		//temp_r_ef[2] = local_COG_LA[2]-CrdAll->data[101];
		//Cross2Vd(&ZAxisAll->data[99],temp_r_ef,temp_cross);
		//Ja->ValSet(22,17,temp_cross[0]*temp_scale_LA);//(8) 
		//Ja->ValSet(23,17,temp_cross[1]*temp_scale_LA); 
		//Ja->ValSet(24,17,temp_cross[2]*temp_scale_LA); 

		// LA axis4
		//Cross2Vd(&ZAxisAll->data[96],temp_r_ef,temp_cross);
		//Ja->ValSet(22,16,temp_cross[0]*temp_scale_LA);//(9) 
		//Ja->ValSet(23,16,temp_cross[1]*temp_scale_LA); 
		//Ja->ValSet(24,16,temp_cross[2]*temp_scale_LA);

		// LA axis3
		temp_scale_LA += mass_com[7]/SumMass;
		temp_mass_sum_next_LA = temp_mass_sum_LA + mass_com[7];
		//local_COG_LA[0] = (local_COG_LA[0]*temp_mass_sum_LA + pv_stack[21]*mass_com[7])/temp_mass_sum_next_LA;
		//local_COG_LA[1] = (local_COG_LA[1]*temp_mass_sum_LA + pv_stack[22]*mass_com[7])/temp_mass_sum_next_LA;
		//local_COG_LA[2] = (local_COG_LA[2]*temp_mass_sum_LA + pv_stack[23]*mass_com[7])/temp_mass_sum_next_LA;
		temp_mass_sum_LA = temp_mass_sum_next_LA;

		//temp_r_ef[0] = local_COG_LA[0]-CrdAll->data[93];
		//temp_r_ef[1] = local_COG_LA[1]-CrdAll->data[94];
		//temp_r_ef[2] = local_COG_LA[2]-CrdAll->data[95];
		//Cross2Vd(&ZAxisAll->data[93],temp_r_ef,temp_cross);
		//Ja->ValSet(22,15,temp_cross[0]*temp_scale_LA); //(10)
		//Ja->ValSet(23,15,temp_cross[1]*temp_scale_LA); 
		//Ja->ValSet(24,15,temp_cross[2]*temp_scale_LA); 

		// LA axis 2
		//Cross2Vd(&ZAxisAll->data[90],temp_r_ef,temp_cross);
		//Ja->ValSet(22,14,temp_cross[0]*temp_scale_LA);//(11) 
		//Ja->ValSet(23,14,temp_cross[1]*temp_scale_LA); 
		//Ja->ValSet(24,14,temp_cross[2]*temp_scale_LA); 

		// LA axis 1
		temp_scale_LA += mass_com[6]/SumMass;
		temp_mass_sum_next_LA = temp_mass_sum_LA + mass_com[6];
		//local_COG_LA[0] = (local_COG_LA[0]*temp_mass_sum_LA + pv_stack[18]*mass_com[6])/temp_mass_sum_next_LA;
		//local_COG_LA[1] = (local_COG_LA[1]*temp_mass_sum_LA + pv_stack[19]*mass_com[6])/temp_mass_sum_next_LA;
		//local_COG_LA[2] = (local_COG_LA[2]*temp_mass_sum_LA + pv_stack[20]*mass_com[6])/temp_mass_sum_next_LA;
		temp_mass_sum_LA = temp_mass_sum_next_LA;

		//temp_r_ef[0] = local_COG_LA[0]-CrdAll->data[87];
		//temp_r_ef[1] = local_COG_LA[1]-CrdAll->data[88];
		//temp_r_ef[2] = local_COG_LA[2]-CrdAll->data[89];
		//Cross2Vd(&ZAxisAll->data[87],temp_r_ef,temp_cross);
		//Ja->ValSet(22,13,temp_cross[0]*temp_scale_LA); //(12)
		//Ja->ValSet(23,13,temp_cross[1]*temp_scale_LA); 
		//Ja->ValSet(24,13,temp_cross[2]*temp_scale_LA);




		//right arm COG Jacobian

		// RA axis6
		//temp_r_ef[0] = pv_stack[39]-CrdAll->data[132];
		//temp_r_ef[1] = pv_stack[40]-CrdAll->data[133];
		//temp_r_ef[2] = pv_stack[41]-CrdAll->data[134];

		temp_scale_RA = mass_com[13]/SumMass;
		//Cross2Vd(&ZAxisAll->data[132],temp_r_ef,temp_cross);
		//Ja->ValSet(22,24,temp_cross[0]*temp_scale_RA); //(13)
		//Ja->ValSet(23,24,temp_cross[1]*temp_scale_RA); 
		//Ja->ValSet(24,24,temp_cross[2]*temp_scale_RA); 

		// RA axis5
		temp_scale_RA += mass_com[12]/SumMass;
		temp_mass_sum_RA = mass_com[12] + mass_com[13];
		//local_COG_RA[0] = (pv_stack[39]*mass_com[13]+pv_stack[36]*mass_com[12])/temp_mass_sum_RA;
		//local_COG_RA[1] = (pv_stack[40]*mass_com[13]+pv_stack[37]*mass_com[12])/temp_mass_sum_RA;
		//local_COG_RA[2] = (pv_stack[41]*mass_com[13]+pv_stack[38]*mass_com[12])/temp_mass_sum_RA;

		//temp_r_ef[0] = local_COG_RA[0]-CrdAll->data[129];
		//temp_r_ef[1] = local_COG_RA[1]-CrdAll->data[130];
		//temp_r_ef[2] = local_COG_RA[2]-CrdAll->data[131];
		//Cross2Vd(&ZAxisAll->data[129],temp_r_ef,temp_cross);
		//Ja->ValSet(22,23,temp_cross[0]*temp_scale_RA);//(14) 
		//Ja->ValSet(23,23,temp_cross[1]*temp_scale_RA); 
		//Ja->ValSet(24,23,temp_cross[2]*temp_scale_RA); 

		// RA axis4
		//Cross2Vd(&ZAxisAll->data[126],temp_r_ef,temp_cross);
		//Ja->ValSet(22,22,temp_cross[0]*temp_scale_RA);//(15) 
		//Ja->ValSet(23,22,temp_cross[1]*temp_scale_RA); 
		//Ja->ValSet(24,22,temp_cross[2]*temp_scale_RA);

		// RA axis3
		temp_scale_RA += mass_com[11]/SumMass;
		temp_mass_sum_next_RA = temp_mass_sum_RA + mass_com[11];
		//local_COG_RA[0] = (local_COG_RA[0]*temp_mass_sum_RA + pv_stack[33]*mass_com[11])/temp_mass_sum_next_RA;
		//local_COG_RA[1] = (local_COG_RA[1]*temp_mass_sum_RA + pv_stack[34]*mass_com[11])/temp_mass_sum_next_RA;
		//local_COG_RA[2] = (local_COG_RA[2]*temp_mass_sum_RA + pv_stack[35]*mass_com[11])/temp_mass_sum_next_RA;
		temp_mass_sum_RA = temp_mass_sum_next_RA;

		//temp_r_ef[0] = local_COG_RA[0]-CrdAll->data[123];
		//temp_r_ef[1] = local_COG_RA[1]-CrdAll->data[124];
		//temp_r_ef[2] = local_COG_RA[2]-CrdAll->data[125];
		//Cross2Vd(&ZAxisAll->data[123],temp_r_ef,temp_cross);
		//Ja->ValSet(22,21,temp_cross[0]*temp_scale_RA); //(16)
		//Ja->ValSet(23,21,temp_cross[1]*temp_scale_RA); 
		//Ja->ValSet(24,21,temp_cross[2]*temp_scale_RA); 

		// RA axis 2
		//Cross2Vd(&ZAxisAll->data[120],temp_r_ef,temp_cross);
		//Ja->ValSet(22,20,temp_cross[0]*temp_scale_RA);//(17) 
		//Ja->ValSet(23,20,temp_cross[1]*temp_scale_RA); 
		//Ja->ValSet(24,20,temp_cross[2]*temp_scale_RA); 

		// RA axis 1
		temp_scale_RA += mass_com[10]/SumMass;
		temp_mass_sum_next_RA = temp_mass_sum_RA + mass_com[10];
		//local_COG_RA[0] = (local_COG_RA[0]*temp_mass_sum_RA + pv_stack[30]*mass_com[10])/temp_mass_sum_next_RA;
		//local_COG_RA[1] = (local_COG_RA[1]*temp_mass_sum_RA + pv_stack[31]*mass_com[10])/temp_mass_sum_next_RA;
		//local_COG_RA[2] = (local_COG_RA[2]*temp_mass_sum_RA + pv_stack[32]*mass_com[10])/temp_mass_sum_next_RA;
		temp_mass_sum_RA = temp_mass_sum_next_RA;

		//temp_r_ef[0] = local_COG_RA[0]-CrdAll->data[117];
		//temp_r_ef[1] = local_COG_RA[1]-CrdAll->data[118];
		//temp_r_ef[2] = local_COG_RA[2]-CrdAll->data[119];
		//Cross2Vd(&ZAxisAll->data[117],temp_r_ef,temp_cross);
		//Ja->ValSet(22,19,temp_cross[0]*temp_scale_RA); //(18)
		//Ja->ValSet(23,19,temp_cross[1]*temp_scale_RA); 
		//Ja->ValSet(24,19,temp_cross[2]*temp_scale_RA);








		// fixed leg Jacobian : axis 7~12
		// axis 7 RL hip yaw include body, not including arms
		temp_scale += temp_scale_LA + temp_scale_RA +( mass_com[14] + mass_com[15])/SumMass;
		temp_mass_sum_next += (temp_mass_sum_LA + temp_mass_sum_RA + mass_com[14] + mass_com[15]);
		local_COG[0] = (local_COG[0]*temp_mass_sum + local_COG_LA[0]*temp_mass_sum_LA + local_COG_RA[0]*temp_mass_sum_RA + pv_stack[42]*mass_com[14] + pv_stack[45]*mass_com[15])/temp_mass_sum_next;
		local_COG[1] = (local_COG[1]*temp_mass_sum + local_COG_LA[1]*temp_mass_sum_LA + local_COG_RA[1]*temp_mass_sum_RA + pv_stack[43]*mass_com[14] + pv_stack[46]*mass_com[15])/temp_mass_sum_next;
		local_COG[2] = (local_COG[2]*temp_mass_sum + local_COG_LA[2]*temp_mass_sum_LA + local_COG_RA[2]*temp_mass_sum_RA + pv_stack[44]*mass_com[14] + pv_stack[47]*mass_com[15])/temp_mass_sum_next;
		temp_mass_sum = temp_mass_sum_next;

		temp_r_ef[0] = local_COG[0]-CrdAll->data[39];
		temp_r_ef[1] = local_COG[1]-CrdAll->data[40];
		temp_r_ef[2] = local_COG[2]-CrdAll->data[41];

		//Cross2Vd(&ZAxisAll->data[39],temp_r_ef,temp_cross);
		//Ja->ValSet(22,7,-temp_cross[0]*temp_scale); //(19) 
		//Ja->ValSet(23,7,-temp_cross[1]*temp_scale); 
		//Ja->ValSet(24,7,-temp_cross[2]*temp_scale); 

		//// fixed leg Jacobian : axis 7~12
		//// axis 7 RL hip yaw include body, not including arms
		//temp_scale += (mass_com[6]+mass_com[7]+mass_com[8]+mass_com[9])/SumMass;
		//temp_mass_sum_next += mass_com[6] + mass_com[7] + mass_com[8] + mass_com[9];
		//local_COG[0] = (local_COG[0]*temp_mass_sum + pv_stack[18]*mass_com[6]+pv_stack[21]*mass_com[7]+pv_stack[24]*mass_com[8]+ pv_stack[27]*mass_com[9])/temp_mass_sum_next;
		//local_COG[1] = (local_COG[1]*temp_mass_sum + pv_stack[19]*mass_com[6]+pv_stack[22]*mass_com[7]+pv_stack[25]*mass_com[8]+ pv_stack[28]*mass_com[9])/temp_mass_sum_next;
		//local_COG[2] = (local_COG[2]*temp_mass_sum + pv_stack[20]*mass_com[6]+pv_stack[23]*mass_com[7]+pv_stack[26]*mass_com[8]+ pv_stack[29]*mass_com[9])/temp_mass_sum_next;
		//temp_mass_sum = temp_mass_sum_next;

		//temp_r_ef[0] = local_COG[0]-CrdAll->data[39];
		//temp_r_ef[1] = local_COG[1]-CrdAll->data[40];
		//temp_r_ef[2] = local_COG[2]-CrdAll->data[41];

		Cross2Vd(&ZAxisAll->data[39],temp_r_ef,temp_cross);
		FixJa->ValSet(4,1,-temp_cross[0]*temp_scale); 
		FixJa->ValSet(5,1,-temp_cross[1]*temp_scale); 
		FixJa->ValSet(6,1,-temp_cross[2]*temp_scale); 

		// axis 8 RL hip roll
		Cross2Vd(&ZAxisAll->data[42],temp_r_ef,temp_cross);
		FixJa->ValSet(4,2,-temp_cross[0]*temp_scale); 
		FixJa->ValSet(5,2,-temp_cross[1]*temp_scale); 
		FixJa->ValSet(6,2,-temp_cross[2]*temp_scale); 	

		// axis 9 RL hip pitch
		Cross2Vd(&ZAxisAll->data[45],temp_r_ef,temp_cross);
		FixJa->ValSet(4,3,-temp_cross[0]*temp_scale); 
		FixJa->ValSet(5,3,-temp_cross[1]*temp_scale); 
		FixJa->ValSet(6,3,-temp_cross[2]*temp_scale); 	

		// axis 10 RL knee pitch
		temp_scale += mass_com[3]/SumMass;
		temp_mass_sum_next += mass_com[3];
		local_COG[0] = (local_COG[0]*temp_mass_sum + pv_stack[9]*mass_com[3])/temp_mass_sum_next;
		local_COG[1] = (local_COG[1]*temp_mass_sum + pv_stack[10]*mass_com[3])/temp_mass_sum_next;
		local_COG[2] = (local_COG[2]*temp_mass_sum + pv_stack[11]*mass_com[3])/temp_mass_sum_next;

		temp_mass_sum = temp_mass_sum_next;

		temp_r_ef[0] = local_COG[0]-CrdAll->data[48];
		temp_r_ef[1] = local_COG[1]-CrdAll->data[49];
		temp_r_ef[2] = local_COG[2]-CrdAll->data[50];

		Cross2Vd(&ZAxisAll->data[48],temp_r_ef,temp_cross);
		FixJa->ValSet(4,4,-temp_cross[0]*temp_scale); 
		FixJa->ValSet(5,4,-temp_cross[1]*temp_scale); 
		FixJa->ValSet(6,4,-temp_cross[2]*temp_scale); 

		// axis 11 LL ankle pitch
		temp_scale += mass_com[4]/SumMass;
		temp_mass_sum_next += mass_com[4];
		local_COG[0] = (local_COG[0]*temp_mass_sum + pv_stack[12]*mass_com[4])/temp_mass_sum_next;
		local_COG[1] = (local_COG[1]*temp_mass_sum + pv_stack[13]*mass_com[4])/temp_mass_sum_next;
		local_COG[2] = (local_COG[2]*temp_mass_sum + pv_stack[14]*mass_com[4])/temp_mass_sum_next;

		temp_mass_sum = temp_mass_sum_next;

		temp_r_ef[0] = local_COG[0]-CrdAll->data[51];
		temp_r_ef[1] = local_COG[1]-CrdAll->data[52];
		temp_r_ef[2] = local_COG[2]-CrdAll->data[53];

		Cross2Vd(&ZAxisAll->data[51],temp_r_ef,temp_cross);
		FixJa->ValSet(4,5,-temp_cross[0]*temp_scale); 
		FixJa->ValSet(5,5,-temp_cross[1]*temp_scale); 
		FixJa->ValSet(6,5,-temp_cross[2]*temp_scale); 

		// axis 6 LL ankle roll

		Cross2Vd(&ZAxisAll->data[54],temp_r_ef,temp_cross);
		FixJa->ValSet(4,6,-temp_cross[0]*temp_scale); 
		FixJa->ValSet(5,6,-temp_cross[1]*temp_scale); 
		FixJa->ValSet(6,6,-temp_cross[2]*temp_scale); 

	}

}

void Kine::FindCOG(void)
{

	/******************************************************************
	input: void
	output: void

	Note:
	// 利用桿件重量乘上其在空間中之位置 全部加起來 加權平均 可得到COG 位置
	******************************************************************/

	//  left leg 質量乘上位置 前處理 位置部分
	pv_stack[0] = CrdAll->data[9]-CrdAll->data[6];
	pv_stack[1] = CrdAll->data[10]-CrdAll->data[7];
	pv_stack[2] = CrdAll->data[11]-CrdAll->data[8]; // 軸3 knee 減 軸2 hip

	pv_stack[3] = CrdAll->data[12]-CrdAll->data[9];
	pv_stack[4] = CrdAll->data[13]-CrdAll->data[10];
	pv_stack[5] = CrdAll->data[14]-CrdAll->data[11]; // 軸4 ankle 減 軸3 knee

	pv_stack[6] = CrdAll->data[21]-CrdAll->data[12];
	pv_stack[7] = CrdAll->data[22]-CrdAll->data[13];
	pv_stack[8] = CrdAll->data[23]-CrdAll->data[14]; // 軸7 foot 減 軸4 ankle


	//  right leg 質量乘上位置 前處理 位置部分
	pv_stack[9] = CrdAll->data[48]-CrdAll->data[45];
	pv_stack[10] = CrdAll->data[49]-CrdAll->data[46];
	pv_stack[11] = CrdAll->data[50]-CrdAll->data[47]; // 軸16 knee 減 軸15 hip

	pv_stack[12] = CrdAll->data[51]-CrdAll->data[48];
	pv_stack[13] = CrdAll->data[52]-CrdAll->data[49];
	pv_stack[14] = CrdAll->data[53]-CrdAll->data[50]; // 軸17 ankle 減 軸16 knee

	pv_stack[15] = CrdAll->data[60]-CrdAll->data[51];
	pv_stack[16] = CrdAll->data[61]-CrdAll->data[52];
	pv_stack[17] = CrdAll->data[62]-CrdAll->data[53]; // 軸20 foot 減 軸17 ankle

	//泓逸start111214


	//  left and right arms
	/*
	// L Arm
	pv_stack[18] = CrdAll->data[87]-CrdAll->data[84];
	pv_stack[19] = CrdAll->data[88]-CrdAll->data[85];
	pv_stack[20] = CrdAll->data[89]-CrdAll->data[86]; 

	// R Arm
	pv_stack[21] = CrdAll->data[99]-CrdAll->data[96];
	pv_stack[22] = CrdAll->data[100]-CrdAll->data[97];
	pv_stack[23] = CrdAll->data[101]-CrdAll->data[98]; 
	*/
	
		// L Arm
	pv_stack[18] = CrdAll->data[90]-CrdAll->data[84];//5-3
	pv_stack[19] = CrdAll->data[91]-CrdAll->data[85];
	pv_stack[20] = CrdAll->data[92]-CrdAll->data[86]; 

	pv_stack[21] = CrdAll->data[96]-CrdAll->data[90];//7-5
	pv_stack[22] = CrdAll->data[97]-CrdAll->data[91];
	pv_stack[23] = CrdAll->data[98]-CrdAll->data[92];

	pv_stack[24] = CrdAll->data[102]-CrdAll->data[99];//9-8
	pv_stack[25] = CrdAll->data[103]-CrdAll->data[100];
	pv_stack[26] = CrdAll->data[104]-CrdAll->data[101];

	pv_stack[27] = CrdAll->data[105]-CrdAll->data[102];//10-9
	pv_stack[28] = CrdAll->data[106]-CrdAll->data[103];
	pv_stack[29] = CrdAll->data[107]-CrdAll->data[104];

	// R Arm
	pv_stack[30] = CrdAll->data[120]-CrdAll->data[114];//5-3
	pv_stack[31] = CrdAll->data[121]-CrdAll->data[115];
	pv_stack[32] = CrdAll->data[122]-CrdAll->data[116];
	

	pv_stack[33] = CrdAll->data[126]-CrdAll->data[120];//7-5
	pv_stack[34] = CrdAll->data[127]-CrdAll->data[121];
	pv_stack[35] = CrdAll->data[128]-CrdAll->data[122];

	pv_stack[36] = CrdAll->data[132]-CrdAll->data[129];//9-8
	pv_stack[37] = CrdAll->data[133]-CrdAll->data[130];
	pv_stack[38] = CrdAll->data[134]-CrdAll->data[131];

	pv_stack[39] = CrdAll->data[135]-CrdAll->data[132];//10-9
	pv_stack[40] = CrdAll->data[136]-CrdAll->data[133];
	pv_stack[41] = CrdAll->data[137]-CrdAll->data[134];
	


	//泓逸end111214  

	// body   up = 向上之向量 Right = 向右  Face = 面向  質量乘上位置 前處理 位置部分
	BodyUpVec[0] = DHOrigin[0]-(CrdAll->data[0]+CrdAll->data[39])/2.0;
	BodyUpVec[1] = DHOrigin[1]-(CrdAll->data[1]+CrdAll->data[40])/2.0;
	BodyUpVec[2] = DHOrigin[2]-(CrdAll->data[2]+CrdAll->data[41])/2.0;

	temp_double_1 = sqrt(BodyUpVec[0]*BodyUpVec[0]+BodyUpVec[1]*BodyUpVec[1]+BodyUpVec[2]*BodyUpVec[2]);
	BodyUpVec[0] = BodyUpVec[0]/temp_double_1;
	BodyUpVec[1] = BodyUpVec[1]/temp_double_1;
	BodyUpVec[2] = BodyUpVec[2]/temp_double_1;

	BodyRightVec[0] = CrdAll->data[39]-CrdAll->data[0]; // 右減左
	BodyRightVec[1] = CrdAll->data[40]-CrdAll->data[1];
	BodyRightVec[2] = CrdAll->data[41]-CrdAll->data[2];

	temp_double_1 = sqrt(BodyRightVec[0]*BodyRightVec[0]+BodyRightVec[1]*BodyRightVec[1]+BodyRightVec[2]*BodyRightVec[2]);
	BodyRightVec[0] = BodyRightVec[0]/temp_double_1;
	BodyRightVec[1] = BodyRightVec[1]/temp_double_1;
	BodyRightVec[2] = BodyRightVec[2]/temp_double_1;

	Cross2Vd(BodyUpVec, BodyRightVec, BodyFaceVec);

//泓逸start111214

	UpBodyUpVec[0] = CrdAll->data[84]-CrdAll->data[78];
	UpBodyUpVec[1] = CrdAll->data[85]-CrdAll->data[79];
	UpBodyUpVec[2] = CrdAll->data[86]-CrdAll->data[80];

	temp_double_1 = sqrt(UpBodyUpVec[0]*UpBodyUpVec[0]+UpBodyUpVec[1]*UpBodyUpVec[1]+UpBodyUpVec[2]*UpBodyUpVec[2]);
	UpBodyUpVec[0] = UpBodyUpVec[0]/temp_double_1;
	UpBodyUpVec[1] = UpBodyUpVec[1]/temp_double_1;
	UpBodyUpVec[2] = UpBodyUpVec[2]/temp_double_1;

	// body block down
	pv_stack[42] = -40.689*BodyUpVec[0]-20.625*BodyFaceVec[0];
	pv_stack[43] = -40.689*BodyUpVec[1]-20.625*BodyFaceVec[1];
	pv_stack[44] = -40.689*BodyUpVec[2]-20.625*BodyFaceVec[2];//下半部

	
	/*// body block up
	pv_stack[45] = BodyUpVec[0];
	pv_stack[46] = BodyUpVec[1];
	pv_stack[47] = BodyUpVec[2];//上半部*/

	// body block up
	pv_stack[45] = UpBodyUpVec[0];
	pv_stack[46] = UpBodyUpVec[1];
	pv_stack[47] = UpBodyUpVec[2];//上半部


	//DH_Origin[0] = 0;
	//DH_Origin[1] = 0;
	//DH_Origin[2] = 0;

	NormVecArray(pv_stack,temp_Norm,16); 

	int Index = 0;
	double Scale;
	for (int i = 0 ; i < 16 ; i++)
	{
		if (temp_Norm[i] < 0.000001)
			Scale = 0;
		else
			Scale = r_com[i]/temp_Norm[i];

		pv_stack[Index] *= Scale;
		pv_stack[Index+1] *= Scale;
		pv_stack[Index+2] *= Scale;
		Index += 3;
	}

	//  left leg
	pv_stack[0] += CrdAll->data[6];
	pv_stack[1] += CrdAll->data[7];
	pv_stack[2] += CrdAll->data[8]; // 軸3 knee 減 軸2 hip

	pv_stack[3] += CrdAll->data[9];
	pv_stack[4] += CrdAll->data[10];
	pv_stack[5] += CrdAll->data[11]; // 軸4 ankle 減 軸3 knee

	pv_stack[6] += CrdAll->data[12];
	pv_stack[7] += CrdAll->data[13];
	pv_stack[8] += CrdAll->data[14]; // 軸7 foot 減 軸4 ankle


	//  right leg
	pv_stack[9] += CrdAll->data[45];
	pv_stack[10] += CrdAll->data[46];
	pv_stack[11] += CrdAll->data[47]; // 軸16 knee 減 軸15 hip

	pv_stack[12] += CrdAll->data[48];
	pv_stack[13] += CrdAll->data[49];
	pv_stack[14] += CrdAll->data[50]; // 軸17 ankle 減 軸16 knee

	pv_stack[15] += CrdAll->data[51];
	pv_stack[16] += CrdAll->data[52];
	pv_stack[17] += CrdAll->data[53]; // 軸20 foot 減 軸17 ankle

	//泓逸start111214
	// arms
	/*// L Arm
	pv_stack[18] += CrdAll->data[84];
	pv_stack[19] += CrdAll->data[85];
	pv_stack[20] += CrdAll->data[86]; 

	// R Arm
	pv_stack[21] += CrdAll->data[96];
	pv_stack[22] += CrdAll->data[97];
	pv_stack[23] += CrdAll->data[98]; */


	// L Arm
	pv_stack[18] += CrdAll->data[84];//5-3
	pv_stack[19] += CrdAll->data[85];
	pv_stack[20] += CrdAll->data[86]; 

	pv_stack[21] += CrdAll->data[90];//7-5
	pv_stack[22] += CrdAll->data[91];
	pv_stack[23] += CrdAll->data[92];

	pv_stack[24] += CrdAll->data[99];//9-8
	pv_stack[25] += CrdAll->data[100];
	pv_stack[26] += CrdAll->data[101];

	pv_stack[27] += CrdAll->data[102];//10-9
	pv_stack[28] += CrdAll->data[103];
	pv_stack[29] += CrdAll->data[104];

	// R Arm
	pv_stack[30] += CrdAll->data[114];//5-3
	pv_stack[31] += CrdAll->data[115];
	pv_stack[32] += CrdAll->data[116];
	
	pv_stack[33] += CrdAll->data[120];//7-5
	pv_stack[34] += CrdAll->data[121];
	pv_stack[35] += CrdAll->data[122];

	pv_stack[36] += CrdAll->data[129];//9-8
	pv_stack[37] += CrdAll->data[130];
	pv_stack[38] += CrdAll->data[131];

	pv_stack[39] += CrdAll->data[132];//10-9
	pv_stack[40] += CrdAll->data[133];
	pv_stack[41] += CrdAll->data[134];


	// body
	// body block down
	pv_stack[42] += DHOrigin[0];
	pv_stack[43] += DHOrigin[1];
	pv_stack[44] += DHOrigin[2];

	// body block up
	pv_stack[45] += DHOrigin[0]+99*UpBodyUpVec[0];
	pv_stack[46] += DHOrigin[1]+99*UpBodyUpVec[1];
	pv_stack[47] += DHOrigin[2]+99*UpBodyUpVec[2];


	COG[0] = 0;
	COG[1] = 0;
	COG[2] = 0;

	Index = 0;
	for (int i = 0 ; i < 16 ; i++)
	{
		COG[0] += pv_stack[Index]*mass_com[i];
		COG[1] += pv_stack[Index+1]*mass_com[i];
		COG[2] += pv_stack[Index+2]*mass_com[i];
		Index += 3;
	}

	COG[0] = COG[0]/SumMass;
	COG[1] = COG[1]/SumMass;
	COG[2] = COG[2]/SumMass;

}


void Kine::Cross2Vf(float* v1, float* v2, float* v3)
{
	/******************************************************************
	input: v1 x v2 ( = v3 output)
	output: v3

	Note:
	// 向量外積
	******************************************************************/
	v3[0] = v1[1]*v2[2]-v1[2]*v2[1];
	v3[1] = v1[2]*v2[0]-v1[0]*v2[2];
	v3[2] = v1[0]*v2[1]-v1[1]*v2[0];
}

void Kine::Cross2Vd(double* v1, double* v2, double* v3)
{
	/******************************************************************
	input: v1 x v2 ( = v3 output)
	output: v3

	Note:
	// 向量外積
	******************************************************************/
	v3[0] = v1[1]*v2[2]-v1[2]*v2[1];
	v3[1] = v1[2]*v2[0]-v1[0]*v2[2];
	v3[2] = v1[0]*v2[1]-v1[1]*v2[0];
}

double Kine::NormXYZD(double* Vec)
{
	/******************************************************************
	input: vec 要求出norm的向量
	output: 回傳norm

	Note:
	// 向量外積
	******************************************************************/
	return sqrt(Vec[0]*Vec[0]+Vec[1]*Vec[1]+Vec[2]*Vec[2]);
}


void Kine::GenSwingTraj(double v0, double a0, double x1_percentage, double y1, double v1, double x2, double y2, double v2, double a2, int Np, int KeepPosCount, double* resultXY, double* resultZ)
{
	/******************************************************************
	input: v0 a0 y1 v1 x2 y2 v2 a2, 0代表初始點 1代表中間點 2代表結束點 x代表橫軸 y代表縱軸值 v a 代表縱軸速度與加速度
	       x1_percentage 代表中間點位置百分比,  Np 代表內差點數, KeepPosCount 代表維持常數的點數, resultXY 儲存x軸結果, resultZ 儲存y軸結果
	output: void

	Note:
	// 計算出swing腳軌跡 --> 每部要跨出去的軌跡
	// KeepPosCount 縮短軌跡到達時間 主要用在上樓梯 其他先給零 (平地可以永遠給零)
	// x1_percentage代表 x1在 0~x2多少百分比的地方

	******************************************************************/


	// 參考開始位置當做0
	// x0 = y0 = 0;

	// Na = Np 也就是SSP總共點數的前半
	Na = (int)(x1_percentage*Np);
	// Na = Np 也就是SSP總共點數的後半
	Nb = Np-Na; // 包含x1的下一格到x2

	for (int i =0 ; i<Nza ; i++)
	{
		resultXY[i] = 0;
		resultZ[i] = 0;
	}

	// 水平方向 借用平滑ZMP生成函數 開始與結尾速度 加速度 jerk = 0
	GenSmoothZMPShift_ZeroJerk(0,x2,Np-KeepPosCount,resultXY+Nza); 
	// 垂直方向 利用九階多項式生成
	Gen9DegPoly(x1_percentage, y1, y2, Np-KeepPosCount, resultZ+Nza);
	
	x_p = x2;

	for (int i =Nza+Np-KeepPosCount ; i< N_step ; i++)
	{
		resultXY[i] = x_p;
		resultZ[i] = y2;
	}

	#if SaveSwingTraj
 	// 要存成檔案就打開下面程式
		fstream Fx;
		
		Fx.open("swingx.txt",ios::out);
		Fx.precision(10);
		
		for (int i=0 ; i< N_step ; i++)
		{
			Fx << SwingBufferx[i] <<  endl;
		}
		
		Fx.close();
			
		Fx.open("swingz.txt",ios::out| ios::trunc);  // 須改回
		Fx.precision(10);
		
		for (int i=0 ; i< N_step ; i++)
		{
			Fx << SwingBufferz[i] <<  endl;
		}
		
		Fx.close();

	#endif

}

void Kine::GenSwingTrajMod(double v0, double a0, double x1_percentage, double z1, double v1, double xy2, double z2, double v2, double a2, int Np, int KeepPosCount, double* resultXY, double* resultZ)
{

	/******************************************************************
	input: v0 a0 y1 v1 xy2 z2 v2 a2, 0代表初始點 1代表中間點 2代表結束點 xy代表橫軸(gStrideX gStrideY) z代表縱軸值(gStrideZ) v a 代表縱軸速度與加速度
	       x1_percentage 代表中間點位置百分比,  Np 代表內差點數, KeepPosCount 代表維持常數的點數, resultXY 儲存x y軸結果, resultZ 儲存z軸結果
	output: void

	Note:
	// 計算出swing腳軌跡 --> 每部要跨出去的軌跡
	// KeepPosCount 縮短軌跡到達時間 主要用在上樓梯 其他先給零 (平地可以永遠給零)
	// x1_percentage代表 x1在 0~x2多少百分比的地方
	// Mod版本讓swing腳提早抬起 這樣就可以讓行為更像人 以及 更不容易在前進時讓後腳達到joint limit讓行走更順利
	******************************************************************/

	// Na是SSP總共點數的前半
	Na = (int)(x1_percentage*Np);
	// Nb是SSP總共點數的後半 
	Nb = Np-Na; // 包含x1的下一格到x2

	int NWait = int(Nza*0.25); // 25% of Nza, Nza is not used now
	if (gUpStair){
		NWait=Nza*0.30;
	}
	/******************************************************************
	Flag
		gUpStair 爬樓梯使用 針對爬階將腳底板與階梯的距離最大化(調整時間相關參數)
	******************************************************************/		

	for (int i =0 ; i<NWait ; i++)
	{
		resultXY[i] = 0;
		resultZ[i] = 0;
	}
	//// 水平方向 借用平滑ZMP生成函數 開始與結尾速度 加速度 jerk = 0
	if (gUpStair)
	{
		int LiftShifting = 300;
		GenSmoothZMPShift_ZeroJerk(0,xy2,Np-KeepPosCount+Nza-NWait-LiftShifting,resultXY+NWait); 	
		for(int i = 0 ; i < LiftShifting ; i++)
		{
			resultXY[Np-KeepPosCount+Nza-LiftShifting+i]=xy2; 
		}
	}
	else
	{
		GenSmoothZMPShift_ZeroJerk(0,xy2,Np-KeepPosCount+Nza-NWait,resultXY+NWait); 
	}	

	//GenSmoothZMPShift_ZeroJerk(0,20,50,resultXY+NWait); 	
	//GenSmoothZMPShift_ZeroJerk(0,x2,Np-KeepPosCount+Nza-NWait-100,resultXY+NWait+100); 
	//GenSmoothZMPShift_ZeroJerk(1,0,(Np-KeepPosCount+Nza-NWait-880),SwingFreeCoef+NWait); 
	//for(int i=0;i<800;i++)
	//{
	//SwingFreeCoef[NWait+80+i]=0;
	//}
	//////////////////////////////////////_________________________________________________________________Slongz
	//GenSmoothZMPShift_ZeroJerk(0,1,(Np-KeepPosCount+Nza-NWait-880),SwingFreeCoef+NWait+880); 
	GenSmoothZMPShift_ZeroJerk(1,0,100,SwingFreeCoef+NWait+100); 
	for(int i = 0 ; i < 100 ; i++)
	{
		SwingFreeCoef[NWait+200+i]=0;
	}

	GenSmoothZMPShift_ZeroJerk(0,1,500,SwingFreeCoef+NWait+300); 
	//////////////////////////////////////_________________________________________________________________Slongz


	// 垂直方向 利用九階多項式生成
	if(z2 >= 15 || z2 <= -15 )
	{
		//GenZMPFreeAssign(1.5,0,0,0,0,y2/2+30,0.30,0,0,300,resultZ+NWait);
		//GenZMPFreeAssign(1.5,y2/2+30,0.30,0,0,y2+20,0.1,0,0, 300,resultZ+NWait+300);
		//GenZMPFreeAssign(2.0,y2+20,0.1,0,0,y2,0,0,0,400,resultZ+NWait+600);
		//GenZMPFreeAssign(1.0,y2,0,0,0,y2,0,0,0,200,resultZ+NWait+1000);
			// for 單階slongz
		//GenZMPFreeAssign(3,0,0,0,0,y2+30,0.30,0,0,400-100,resultZ+NWait+100);
		////GenZMPFreeAssign(1.5,y2/2+30,0.30,0,0,y2+20,0.1,0,0, 300,resultZ+NWait+300);
		////GenZMPFreeAssign(2.0,y2+20,0.1,0,0,y2,0,0,0,400,resultZ+NWait+600);
		//GenZMPFreeAssign(3.0,y2+30,0,0,0,y2,0,0,0,700,resultZ+NWait+400);
		//GenZMPFreeAssign(3.0,y2,0,0,0,y2,0,0,0,100,resultZ+NWait+1100);
			// for 單階slongz
		Gen9DegPolyStair(x1_percentage, z1, z2, Np-KeepPosCount+Nza-NWait, resultZ+NWait);
	}
	else	// 平地
	{
		Gen9DegPoly(x1_percentage, z1, z2, Np-KeepPosCount+Nza-NWait, resultZ+NWait);
	}
 	//Gen10DegPoly(x1_percentage, y1,0.25,105,0.75,-105, y2, Np-KeepPosCount+Nza-NWait, resultZ+NWait);
	x_p = xy2;

	for (int i =Nza+Np-KeepPosCount ; i< N_step ; i++)	// 填Nzb 為後段DSP 
	{
		resultXY[i] = x_p;
		resultZ[i] = z2;
	}

	for (int i=0 ; i< N_step ; i++)
		AnklePitchRef[i]=0;

	#if SaveSwingTraj
	// 要存成檔案就打開下面程式
		fstream Fx;

		Fx.open("swingx.txt",ios::out);
		Fx.precision(10);
		for (int i=0 ; i< N_step ; i++)
		{
			Fx << SwingBufferx[i] <<  endl;
		}
		Fx.close();


		Fx.open("swingz.txt",ios::out);
		Fx.precision(10);
		for (int i=0 ; i< N_step ; i++)
		{
			Fx << SwingBufferz[i] <<  endl;
		}
		Fx.close();

		Fx.open("swingxcoef.txt",ios::out);
		Fx.precision(10);
		for (int i=0 ; i< N_step ; i++)
		{
			Fx << SwingFreeCoef[i] <<  endl;
		}
		Fx.close();
	#endif

}
void Kine::GenSwingTrajMod2(double v0, double a0, double x1_percentage, double y1, double v1, double x2, double y2, double v2, double a2, int Np, int KeepPosCount, double* resultXY, double* resultZ)
{

	/******************************************************************
	input: v0 a0 y1 v1 x2 y2 v2 a2, 0代表初始點 1代表中間點 2代表結束點 x代表橫軸 y代表縱軸值 v a 代表縱軸速度與加速度
	       x1_percentage 代表中間點位置百分比,  Np 代表內差點數, KeepPosCount 代表維持常數的點數, resultXY 儲存x軸結果, resultZ 儲存y軸結果
	output: void

	Note:
	// 計算出swing腳軌跡 --> 每部要跨出去的軌跡
	// KeepPosCount 縮短軌跡到達時間 主要用在上樓梯 其他先給零 (平地可以永遠給零)
	// x1_percentage代表 x1在 0~x2多少百分比的地方
	// Mod版本讓swing腳提早抬起 這樣就可以讓行為更像人 以及 更不容易在前進時讓後腳達到joint limit
	// 讓行走更順利
	//Mod2增加腳踝起伏，讓行走更自然
	//但有可能更容易碰觸joint limit 慎用!!
	******************************************************************/

	// 參考開始位置當做0
	// x0 = y0 = 0;

	// Na是SSP總共點數的前半
	Na = (int)(x1_percentage*Np);
	// Nb是SSP總共點數的後半
	Nb = Np-Na; // 包含x1的下一格到x2

	int NWait = int(Nza*0.25); // 25% of Nza, Nza is not used now,

	for (int i =0 ; i<NWait ; i++)
	{
		resultXY[i] = 0;
		resultZ[i] = 0;
	}

	// 水平方向 借用平滑ZMP生成函數 開始與結尾速度 加速度 jerk = 0
	GenSmoothZMPShift_ZeroJerk(0,x2,Np-KeepPosCount+Nza-NWait,resultXY+NWait); 
	// 垂直方向 利用九階多項式生成
	Gen9DegPoly(x1_percentage, y1, y2, Np-KeepPosCount+Nza-NWait, resultZ+NWait);
	
	x_p = x2;

	for (int i =Nza+Np-KeepPosCount ; i< N_step ; i++)
	{
		resultXY[i] = x_p;
		resultZ[i] = y2;
	}


	// Ankle Flustrtation
	double FlipRatio = 0.32;
	double FlipRatio2 = 0.50;

	double Ankle2Toe=180.31;
	double Ankle2Heel=172;
	//StepHeight[stepIndex]
	for (int i =0 ; i<N_step ; i++)
	{
		if (SwingBufferz[i+1]-SwingBufferz[i]>=0)
		{
			if (SwingBufferz[i]<=FlipRatio*30) //Rising Finger off
				AnklePitchRef[i]= asin((SwingBufferz[i]+139.70)/Ankle2Toe)-asin(139.70/Ankle2Toe);
			else 
				AnklePitchRef[i]=1* asin(((30-SwingBufferz[i])*FlipRatio/(1-FlipRatio)+139.70)/Ankle2Toe)-asin(139.70/Ankle2Toe);//175.25
		}
		else
		{
			if (SwingBufferz[i]<=FlipRatio2*30)
				AnklePitchRef[i]= -1* asin((SwingBufferz[i]+139.70)/Ankle2Heel)+asin(139.70/Ankle2Heel);//169.5054
			else 
				AnklePitchRef[i]= -asin(((30-SwingBufferz[i])*FlipRatio2/(1-FlipRatio2)+139.70)/Ankle2Heel)+asin(139.70/Ankle2Heel); // /PI*360
		}   
	}
	//for (int i =0 ; i<N_step ; i++)
	//	if (SwingBufferz[i+1]-SwingBufferz[i]>=0)
	//	{
	//		if (SwingBufferz[i]<=FlipRatio*30) //Rising Finger off
	//			AnklePitchRef[i]= asin((SwingBufferz[i]+122.2)/134.2129)-asin(122.2/134.2129);
	//		else 
	//			AnklePitchRef[i]=1* asin(((30-SwingBufferz[i])*FlipRatio/(1-FlipRatio)+122.2)/134.2129)-asin(122.2/134.2129);
	//	}
	//	else
	//	{
	//		if (SwingBufferz[i]<=FlipRatio2*30)
	//			AnklePitchRef[i]= -1* asin((SwingBufferz[i]+139.70)/163.54)+asin(139.70/163.54);//169.5054
	//		else 
	//			AnklePitchRef[i]= -asin(((30-SwingBufferz[i])*FlipRatio2/(1-FlipRatio2)+139.70)/163.54)+asin(139.70/163.54); // /PI*360
	//	}   

	#if SaveSwingTraj
	// 要存成檔案就打開下面程式
		fstream Fx;

		Fx.open("swingx.txt",ios::out);
		Fx.precision(10);

		for (int i=0 ; i< N_step ; i++)
		{
			Fx << SwingBufferx[i] <<  endl;
		}

		Fx.close();

		//fstream Fx;

		Fx.open("swingz.txt",ios::out);
		Fx.precision(10);

		for (int i=0 ; i< N_step ; i++)
		{
			Fx << SwingBufferz[i] <<  endl;
		}

		Fx.close();

		Fx.open("swinga.txt",ios::out);
		////Fx.precision(10);

		for (int i=0 ; i< N_step ; i++)
		{
			Fx << AnklePitchRef[i] <<  endl;
		}

		Fx.close();

	#endif
}

void Kine::GenCOGZTrajMod(double v0, double a0,double y0, double x1_percentage, double y1, double v1, double x2, double y2, double v2, double a2, int Np, int KeepPosCount, double* resultXY, double* resultZ)
{

	/******************************************************************
	input: v0 a0 y1 v1 x2 y2 v2 a2, 0代表初始點 1代表中間點 2代表結束點 x代表橫軸 y代表縱軸值 v a 代表縱軸速度與加速度
	       x1_percentage 代表中間點位置百分比,  Np 代表內差點數, KeepPosCount 代表維持常數的點數, resultXY 儲存x軸結果, resultZ 儲存y軸結果
	output: void

	Note:
	// 計算出swing腳軌跡 --> 每部要跨出去的軌跡
	// KeepPosCount 縮短軌跡到達時間 主要用在上樓梯 其他先給零 (平地可以永遠給零)
	// x1_percentage代表 x1在 0~x2多少百分比的地方
	// Mod版本讓swing腳提早抬起 這樣就可以讓行為更像人 以及 更不容易在前進時讓後腳達到joint limit
	// 讓行走更順利
	//Mod2增加腳踝起伏，讓行走更自然//但有可能更容易碰觸joint limit 慎用!!//????????????? 是上面那個函式吧
	******************************************************************/

	// 參考開始位置當做0
	// x0 = y0 = 0;

	// Na是SSP總共點數的前半
	Na = (int)(x1_percentage*Np);
	// Nb是SSP總共點數的後半
	Nb = Np-Na; // 包含x1的下一格到x2

	//int NWait = int(Nza*0.25); // 25% of Nza, Nza is not used now,

	//for (int i =0 ; i<NWait ; i++)
	//{
	//	resultXY[i] = 0;
	//	resultZ[i] = 0;
	//}

	// 水平方向 借用平滑ZMP生成函數 開始與結尾速度 加速度 jerk = 0
	GenSmoothZMPShift_ZeroJerk(0,x2,Np-KeepPosCount+Nza,resultXY); 
	// 垂直方向 利用九階多項式生成
	Gen9DegPoly(x1_percentage, y1, y2, Np-KeepPosCount+Nza, resultZ);
	
	//x_p = x2;

	//for (int i =Np-KeepPosCount+Nza ; i< Np ; i++)
	//{
	//	resultXY[i] = x_p;
	//	resultZ[i] = y2;
	//}

// 要存成檔案就打開下面程式
//	fstream Fx;
//
//	Fx.open("swingx.txt",ios::out);
//	Fx.precision(10);
//
//	for (int i=0 ; i< N_step ; i++)
//	{
//		Fx << SwingBufferx[i] <<  endl;
//	}
//
//	Fx.close();
//
////	fstream Fx;
//
//	Fx.open("swingz.txt",ios::out);
//	Fx.precision(10);
//
//	for (int i=0 ; i< N_step ; i++)
//	{
//		Fx << SwingBufferz[i] <<  endl;
//	}
//
//	Fx.close();


}


void Kine::Gen7DegPoly(double x1, double y1, double y2, int Np, double* result)
{
	/******************************************************************
	input: x1 y1 y2, 1代表初始點 2代表結束點 x代表橫軸 y代表縱軸值
	       Np 代表內差點數, result儲存結果
	output: void

	Note:
	// 計算七階多項式軌跡內插
	// x1 輸入請調在 0.43~0.57 之間
	******************************************************************/

	double A[25];
	A[0] = x1*x1*x1; A[1] = A[0]*x1; A[2] = A[1]*x1; A[3] = A[2]*x1; A[4] = A[3]*x1;
	A[5] = 3*x1*x1;  A[6] = 4*A[0];  A[7] = 5*A[1];  A[8] = 6*A[2];  A[9] = 7*A[3];
	A[10] = 1;		 A[11] = 1;		 A[12] = 1;		 A[13] = 1;		 A[14] = 1;
	A[15] = 3;		 A[16] = 4;		 A[17] = 5;		 A[18] = 6;		 A[19] = 7;
	A[20] = 6;		 A[21] = 12;	 A[22] = 20;	 A[23] = 30;	 A[24] = 42;

//A = [x1^3 x1^4 x1^5 x1^6 x1^7 ;
//     3*x1^2 4*x1^3 5*x1^4 6*x1^5 7*x1^6  ;
//     x2^3 x2^4 x2^5 x2^6 x2^7 ;
//     3*x2^2 4*x2^3 5*x2^4 6*x2^5 7*x2^6 ;
//     6*x2 12*x2^2 20*x2^3 30*x2^4 42*x2^5];

	double r_in[5];
	r_in[0] = y1; r_in[1] = 0; r_in[2] = y2; r_in[3] = 0; r_in[4] = 0;
	//r = [y1;v1;y2;v2;a2];

	double Poly[8];
	Poly[0] = 0; // a0
	Poly[1] = 0; // a1
	Poly[2] = 0; // a2

	InvSqMat(A,5);
	MatMulAB(A,5,5,r_in,5,1,Poly+3);

	double Step = 1.0/double(Np-1);

	double xn[8];
	xn[0] = 1;
	xn[1] = 0;

	result[0] = 0;

	for (int i = 1 ; i < Np ; i++)
	{
		xn[1] += Step; // x
		xn[2] = xn[1]*xn[1]; // x^2
		xn[3] = xn[1]*xn[2]; // x^3
		xn[4] = xn[1]*xn[3]; // x^4
		xn[5] = xn[1]*xn[4]; // x^5
		xn[6] = xn[1]*xn[5]; // x^6
		xn[7] = xn[1]*xn[6]; // x^7

		result[i] = Poly[3]*xn[3]+Poly[4]*xn[4]+Poly[5]*xn[5]+Poly[6]*xn[6]+Poly[7]*xn[7];

	}


}

void Kine::Gen9DegPoly(double x1, double y1, double y2, int Np, double* result)
{
	/******************************************************************
	input: x1 y1 y2, 1代表初始點 2代表結束點 x代表橫軸 y代表縱軸值
	       Np 代表內差點數, result儲存結果
	output: void

	Note:
	// 計算九階多項式軌跡內插
	// x1 輸入請調在 0.44~0.56 之間
	******************************************************************/

	double A[36];
	A[0] = x1*x1*x1*x1;		A[1] = A[0]*x1;		A[2] = A[1]*x1;		A[3] = A[2]*x1;		A[4] = A[3]*x1;		A[5] = A[4]*x1;
	A[6] = 4*x1*x1*x1;		A[7] = 5*A[0];		A[8] = 6*A[1];		A[9] = 7*A[2];		A[10] = 8*A[3];		A[11] = 9*A[4];
	A[12] = 1;				A[13] = 1;			A[14] = 1;			A[15] = 1;			A[16] = 1;			A[17] = 1;
	A[18] = 4;				A[19] = 5;			A[20] = 6;			A[21] = 7;			A[22] = 8;			A[23] = 9;
	A[24] = 12;				A[25] = 20;			A[26] = 30;			A[27] = 42;			A[28] = 56;			A[29] = 72;
	A[30] = 24;				A[31] = 60;			A[32] = 120;		A[33] = 210;		A[34] = 336;		A[35] = 504;

//A = [ x1^4 x1^5 x1^6 x1^7 x1^8 x1^9;
//      4*x1^3 5*x1^4 6*x1^5 7*x1^6 8*x1^7 9*x1^8 ;
//      x2^4 x2^5 x2^6 x2^7 x2^8 x2^9;
//     4*x2^3 5*x2^4 6*x2^5 7*x2^6 8*x2^7 9*x2^8;
//     12*x2^2 20*x2^3 30*x2^4 42*x2^5 56*x2^6 72*x2^7
//     24*x2 60*x2^2 120*x2^3 210*x2^4 336*x2^5 504*x2^6];

	double r_in[6];
	r_in[0] = y1; r_in[1] = 0; r_in[2] = y2; r_in[3] = 0; r_in[4] = 0; r_in[5] = 0;
	//r = [y1;v1;y2;v2;a2;j2];

	double Poly[10];
	Poly[0] = 0; // a0
	Poly[1] = 0; // a1
	Poly[2] = 0; // a2
	Poly[3] = 0; // a3

	InvSqMat(A,6);
	MatMulAB(A,6,6,r_in,6,1,Poly+4);

	double Step = 1.0/double(Np-1);

	double xn[10];
	xn[0] = 1;
	xn[1] = 0;

	result[0] = 0;

	for (int i = 1 ; i < Np ; i++)
	{
		xn[1] += Step; // x
		xn[2] = xn[1]*xn[1]; // x^2
		xn[3] = xn[1]*xn[2]; // x^3
		xn[4] = xn[1]*xn[3]; // x^4
		xn[5] = xn[1]*xn[4]; // x^5
		xn[6] = xn[1]*xn[5]; // x^6
		xn[7] = xn[1]*xn[6]; // x^7
		xn[8] = xn[1]*xn[7]; // x^8
		xn[9] = xn[1]*xn[8]; // x^9

		result[i] = Poly[4]*xn[4]+Poly[5]*xn[5]+Poly[6]*xn[6]+Poly[7]*xn[7]+Poly[8]*xn[8]+Poly[9]*xn[9];
	}

}

void Kine::Gen9DegPolyStair(double x1, double y1, double y2, int Np, double* result)
{ 
	/******************************************************************
	input: x1 y1 y2, 1代表初始點 2代表結束點 x代表橫軸 y代表縱軸值
	       Np 代表內差點數, result儲存結果
	output: void
	
	等更改部分

	Note:
	// x1請調0.35~0.26
	// 計算上樓梯Z方向 將兩段由九階多項式內插所得的軌跡在最高點(StepHeight+GroundHeight[i+1])
	// 在此是將50%的九階內插在最高點與%數較少的接合
	// 將50%的內差點數放大為原本(1- x1)的兩倍 然後取後面一半與%數較少的銜接 須注意填入result格數int double整除問題 小則軌跡不連續 大則程式crash
	// x1 輸入越少(%)越快抬腳跨過樓梯

	// 在下樓梯的規劃裡 目標把輸入的y1 y2 當作上樓梯來計算
	// 再利用resultTemp把上樓梯的軌跡存下來 反序丟進下樓梯的軌跡中
	// Wei-Zh Lai 20121123
	******************************************************************/
//若要銜接兩次內插 那就在算一次 但是Z的距離(y1)是StepHeight(y1-y2)
	
	int DownStairHeight = 0 ;
	double resultTemp[2000];
	if(y2<0)
	{
		DownStairHeight = y2;
		y2 = -y2;
		y1 = y1+y2+10;//30
	}
		
	
	double xMod =0.5;
	double A[36];
	A[0] = xMod*xMod*xMod*xMod;		A[1] = A[0]*xMod;		A[2] = A[1]*xMod;		A[3] = A[2]*xMod;		A[4] = A[3]*xMod;		A[5] = A[4]*xMod;
	A[6] = 4*xMod*xMod*xMod;		A[7] = 5*A[0];		A[8] = 6*A[1];		A[9] = 7*A[2];		A[10] = 8*A[3];		A[11] = 9*A[4];
	A[12] = 1;				A[13] = 1;			A[14] = 1;			A[15] = 1;			A[16] = 1;			A[17] = 1;
	A[18] = 4;				A[19] = 5;			A[20] = 6;			A[21] = 7;			A[22] = 8;			A[23] = 9;
	A[24] = 12;				A[25] = 20;			A[26] = 30;			A[27] = 42;			A[28] = 56;			A[29] = 72;
	A[30] = 24;				A[31] = 60;			A[32] = 120;		A[33] = 210;		A[34] = 336;		A[35] = 504;


	double r_in[6];
	r_in[0] = y1-y2; r_in[1] = 0; r_in[2] = 0; r_in[3] = 0; r_in[4] = 0; r_in[5] = 0;
	//r = [y1;v1;y2;v2;a2;j2];

	double Poly[10];
	Poly[0] = 0; // a0
	Poly[1] = 0; // a1
	Poly[2] = 0; // a2
	Poly[3] = 0; // a3

	InvSqMat(A,6);
	MatMulAB(A,6,6,r_in,6,1,Poly+4);

	double Step = 1.0/double((1-x1)*2*Np-1);

	double xn[10];
	xn[0] = 1;
	xn[1] = 0;

	result[0] = 0;
	int a  = Np*(2*x1-1)-1;
	for (int i = 1 ; i < (1-x1)*2*Np+1 ; i++)
	{
		xn[1] += Step; // x
		xn[2] = xn[1]*xn[1]; // x^2
		xn[3] = xn[1]*xn[2]; // x^3
		xn[4] = xn[1]*xn[3]; // x^4
		xn[5] = xn[1]*xn[4]; // x^5
		xn[6] = xn[1]*xn[5]; // x^6
		xn[7] = xn[1]*xn[6]; // x^7
		xn[8] = xn[1]*xn[7]; // x^8
		xn[9] = xn[1]*xn[8]; // x^9
		
		if(i>=(1-x1)*Np)
		result[i+a] = y2+Poly[4]*xn[4]+Poly[5]*xn[5]+Poly[6]*xn[6]+Poly[7]*xn[7]+Poly[8]*xn[8]+Poly[9]*xn[9];
	}
	

	//將後半0.5的部分 存入後半的index中  




	A[0] = x1*x1*x1*x1;		A[1] = A[0]*x1;		A[2] = A[1]*x1;		A[3] = A[2]*x1;		A[4] = A[3]*x1;		A[5] = A[4]*x1;
	A[6] = 4*x1*x1*x1;		A[7] = 5*A[0];		A[8] = 6*A[1];		A[9] = 7*A[2];		A[10] = 8*A[3];		A[11] = 9*A[4];
	A[12] = 1;				A[13] = 1;			A[14] = 1;			A[15] = 1;			A[16] = 1;			A[17] = 1;
	A[18] = 4;				A[19] = 5;			A[20] = 6;			A[21] = 7;			A[22] = 8;			A[23] = 9;
	A[24] = 12;				A[25] = 20;			A[26] = 30;			A[27] = 42;			A[28] = 56;			A[29] = 72;
	A[30] = 24;				A[31] = 60;			A[32] = 120;		A[33] = 210;		A[34] = 336;		A[35] = 504;

	r_in[0] = y1; r_in[1] = 0; r_in[2] = y2; r_in[3] = 0; r_in[4] = 0; r_in[5] = 0;
	//r = [y1;v1;y2;v2;a2;j2];

	Poly[0] = 0; // a0
	Poly[1] = 0; // a1
	Poly[2] = 0; // a2
	Poly[3] = 0; // a3

	InvSqMat(A,6);
	MatMulAB(A,6,6,r_in,6,1,Poly+4);

	Step = 1.0/double(Np-1);

	xn[0] = 1;
	xn[1] = 0;

	result[0] = 0;
	for (int i = 1 ; i <x1*Np ; i++)  // 前半區域的點數
	{
		xn[1] += Step; // x
		xn[2] = xn[1]*xn[1]; // x^2
		xn[3] = xn[1]*xn[2]; // x^3
		xn[4] = xn[1]*xn[3]; // x^4
		xn[5] = xn[1]*xn[4]; // x^5
		xn[6] = xn[1]*xn[5]; // x^6
		xn[7] = xn[1]*xn[6]; // x^7
		xn[8] = xn[1]*xn[7]; // x^8
		xn[9] = xn[1]*xn[8]; // x^9

		result[i] = Poly[4]*xn[4]+Poly[5]*xn[5]+Poly[6]*xn[6]+Poly[7]*xn[7]+Poly[8]*xn[8]+Poly[9]*xn[9];
	}

	if(DownStairHeight < 0)
	{
		for(int i = 1 ; i < Np ; i++)
		{
		resultTemp[i] = result[i];
		}
	
		for(int i = 1; i< Np ; i++)
		{
		result[i] = resultTemp[Np-i]+DownStairHeight;
		}
	}		
}




void Kine::Gen10DegPoly(double x1, double y1,double x2, double v2,double x3, double v3, double yend, int Np, double* result)
{
	/******************************************************************
	input: x1 y1 y2, 1代表初始點 2代表結束點 x代表橫軸 y代表縱軸值
	       Np 代表內差點數, result儲存結果
	output: void

	Note:
	// 計算十階多項式軌跡內插
	// x1 輸入請調在 0.44~0.56 之間
	******************************************************************/
	double PolyX2[9];
	double PolyX3[9];
	PolyX2[0]=x2;
	PolyX3[0]=x3;
	for (int i = 1 ; i < 9 ; i++)
	{
		PolyX2[i]=PolyX2[i-1]*x2;
		PolyX3[i]=PolyX3[i-1]*x3;
	}
	double A[64];
	A[0] = x1*x1*x1;		A[1] = A[0]*x1;			A[2] = A[1]*x1;			A[3] = A[2]*x1;			A[4] = A[3]*x1;			A[5] = A[4]*x1;			A[6] = A[5]*x1;			A[7] = A[6]*x1;
	A[8] = 3*x1*x1; 		A[9] = 4*A[0];			A[10] = 5*A[1];			A[11] = 6*A[2];			A[12] = 7*A[3];			A[13] = 8*A[4];			A[14] = 9*A[5];			A[15] = 10*A[6];
	A[16] = 1;		    	A[17] = 1;				A[18] = 1;				A[19] = 1;				A[20] = 1;				A[21] = 1;				A[22] = 1;				A[23] = 1;		
	A[24] = 3;				A[25] = 4;				A[26] = 5;				A[27] = 6;				A[28] = 7;				A[29] = 8;				A[30] = 9;				A[31] = 10;
	A[32] = 6;				A[33] = 12;				A[34] = 20;				A[35] = 30;				A[36] = 42;				A[37] = 56;				A[38] = 72;				A[39] = 90;				
	A[40] = 6;				A[41] = 24;				A[42] = 60;				A[43] = 120;			A[44] = 210;			A[45] = 336;			A[46] = 504;			A[47] = 720;				
	A[48] = 3*PolyX2[1];	A[49] = 4*PolyX2[2];	A[50] = 5*PolyX2[3];	A[51] = 6*PolyX2[4];	A[52] = 7*PolyX2[5];	A[53] = 8*PolyX2[6];	A[54] = 9*PolyX2[7];	A[55] = 10*PolyX2[8];		
	A[56] = 3*PolyX3[1];	A[57] = 4*PolyX3[2];	A[58] = 5*PolyX3[3];	A[59] = 6*PolyX3[4];	A[60] = 7*PolyX3[5];	A[61] = 8*PolyX3[6];	A[62] = 9*PolyX3[7];	A[63] = 10*PolyX3[8];		
//A = [ x1^4 x1^5 x1^6 x1^7 x1^8 x1^9;
//      4*x1^3 5*x1^4 6*x1^5 7*x1^6 8*x1^7 9*x1^8 ;
//      x2^4 x2^5 x2^6 x2^7 x2^8 x2^9;
//     4*x2^3 5*x2^4 6*x2^5 7*x2^6 8*x2^7 9*x2^8;
//     12*x2^2 20*x2^3 30*x2^4 42*x2^5 56*x2^6 72*x2^7
//     24*x2 60*x2^2 120*x2^3 210*x2^4 336*x2^5 504*x2^6];

	double r_in[8];
	r_in[0] = y1; r_in[1] = 0; r_in[2] = yend; r_in[3] = 0; r_in[4] = 0; r_in[5] = 0;  r_in[6] = v2;  r_in[7] = v3; 
	//r = [y1;v1;y2;v2;a2;j2];

	double Poly[11];
	Poly[0] = 0; // a0
	Poly[1] = 0; // a1
	Poly[2] = 0; // a2
	
	InvSqMat(A,8);
	MatMulAB(A,8,8,r_in,8,1,Poly+3);

	double Step = 1.0/double(Np-1);

	double xn[11];
	xn[0] = 1;
	xn[1] = 0;

	result[0] = 0;

	for (int i = 1 ; i < Np ; i++)
	{
		xn[1] += Step; // x
		xn[2] = xn[1]*xn[1]; // x^2
		xn[3] = xn[1]*xn[2]; // x^3
		xn[4] = xn[1]*xn[3]; // x^4
		xn[5] = xn[1]*xn[4]; // x^5
		xn[6] = xn[1]*xn[5]; // x^6
		xn[7] = xn[1]*xn[6]; // x^7
		xn[8] = xn[1]*xn[7]; // x^8
		xn[9] = xn[1]*xn[8]; // x^9
		xn[10] = xn[1]*xn[9]; // x^9

		result[i] =Poly[3]*xn[3]+ Poly[4]*xn[4]+Poly[5]*xn[5]+Poly[6]*xn[6]+Poly[7]*xn[7]+Poly[8]*xn[8]+Poly[9]*xn[9]+Poly[10]*xn[10];

	}

}

void Kine::Gen7DegPolyMod(double y0, double y1, int Np, double* result)
{
	/******************************************************************
	input: y0 y1, 0代表初始點 1代表結束點 x代表橫軸 y代表縱軸值
	       Np 代表內差點數, result儲存結果
	output: void

	Note:
	// 計算七階多項式軌跡內插
	// 生成的多項式非鐘形 x由0~Np y由y0~y1
	******************************************************************/

	double A[16];
	A[0] = 1;	A[1] = 1;	A[2] = 1;	A[3] = 1; 
	A[4] = 4;	A[5] = 5;	A[6] = 6;	A[7] = 7;
	A[8] = 12;	A[9] = 20;	A[10] = 30;	A[11] = 42;
	A[12] = 24;	A[13] = 60;	A[14] = 120;A[15] = 210;		 

//A = [x1^3 x1^4 x1^5 x1^6 x1^7 ;
//     3*x1^2 4*x1^3 5*x1^4 6*x1^5 7*x1^6  ;
//     x2^3 x2^4 x2^5 x2^6 x2^7 ;
//     3*x2^2 4*x2^3 5*x2^4 6*x2^5 7*x2^6 ;
//     6*x2 12*x2^2 20*x2^3 30*x2^4 42*x2^5];

	double r_in[4];
	r_in[0] = y1-y0; r_in[1] = 0; r_in[2] = 0; r_in[3] = 0;
	//r = [y1;v1;a1;j1];

	double Poly[8];

	Poly[0] = y0; // y0 
	Poly[1] = 0; // v0
	Poly[2] = 0; // a0
	Poly[3] = 0; // j0

	InvSqMat(A,4);
	MatMulAB(A,4,4,r_in,4,1,Poly+4);

	double Step = 1.0/double(Np-1);	// 減掉第一步

	double xn[9];
	xn[0] = 1;
	xn[1] = 0;

	result[0] = y0;

	for (int i = 1 ; i < Np ; i++)
	{
		xn[1] += Step; // x
		xn[2] = xn[1]*xn[1]; // x^2
		xn[3] = xn[1]*xn[2]; // x^3
		xn[4] = xn[1]*xn[3]; // x^4
		xn[5] = xn[1]*xn[4]; // x^5
		xn[6] = xn[1]*xn[5]; // x^6
		xn[7] = xn[1]*xn[6]; // x^7

		result[i] = Poly[0]+Poly[3]*xn[3]+Poly[4]*xn[4]+Poly[5]*xn[5]+Poly[6]*xn[6]+Poly[7]*xn[7];
	}
}

void Kine::IKSolve(double* tCOG, double* tSwing, double* tRSwing, double* tRFixed,double* tLArm, double* tRArm,double* tRLArm, double* tRRArm,int IKMode, int* status)
{
	/******************************************************************
	input: tCOG 代表 target COG, tSwing 代表 target Swing, tRSwing 代表 target Swing rotation matrix,
	       tRFixed 代表 target Fixed rotation matrix, status 回傳IK是否成功

	output: void

	Note:
	// 解IK讓各個end-effector達到目標位置與角度

	// 關於旋轉矩陣之解說
		// 下面解說求出不同坐標系之間角速度 並且丟到IK input之方法
		// 假設有兩相近坐標系(R1 R2 都是3x3矩陣)
		// 要將R1的三個軸 都轉到R2上， 必須左乘R到R1上
		// R是微小旋轉量之旋轉矩陣， 不是右乘 因為坐標軸是向量 是被旋轉的目標
		// 所以 R x R1 = R2, R = R2 x R1' 如同下面式子
		// 而且 假設 R = RxRyRz 並且是可交換的微小旋轉
		// RxRyRz = [1 -z y ; z 1 -x ; -y x 1]; 
		// 所以可以取出DiffRotMatSw DiffRotMatBody 在世界下的旋轉軸
		// 直接將此旋轉軸的向量輸入IK 就可以讓end-effector旋轉了!!
	******************************************************************/

	// selIK = 0 : left foot is the supporter
    // selIK = 1 : right foot is the supporter

	//SwErrLim = 0.1; // swing 解IK 誤差值
	//COGErrLim = 0.1; // COG 解IK 誤差值
	// Slongz 0516
		//////int min_a=-5;
		//////double a=(rand() % (min_a*(-2)  )   )+min_a; 

	int MatIndex = 0;

	cntIK = 0;

	bool RotFlag=1;
	
	double TempMat[9];	// Slongz 20130122
	double TempMat2[9]={1,0,0,0,1,0,0,0,1};	// slope
	while (1)
	{
		// distance error
		if (selIK == LeftSupport)
		{
			SwingErr[0] = tSwing[0] - CrdAll->data[57];
			SwingErr[1] = tSwing[1] - CrdAll->data[58];
			SwingErr[2] = tSwing[2] - CrdAll->data[59];
		}
		else if (selIK == RightSupport || selIK == DoubleSupport)
		{
		// solve IK
			//double QQ=FKLLeg->theta[4]*180/3.1415926;
			//if(QQ<19)
			//flagQQ=0;

			//if(flagQQ)
			//{
			//SwingErr[0] = tSwing[0] - CrdAll->data[18];
			//SwingErr[1] = tSwing[1] - CrdAll->data[19]; 
			//SwingErr[2] = tSwing[2] - CrdAll->data[20];
			//}
			//else
			//{
			//SwingErr[0] = 0;
			//SwingErr[1] = 0; 
			//SwingErr[2] = 0;
			//}
			SwingErr[0] = tSwing[0] - CrdAll->data[18];
			SwingErr[1] = tSwing[1] - CrdAll->data[19];
			SwingErr[2] = tSwing[2] - CrdAll->data[20];
		}
		//////Slongz 20130122
		//
		//	if(selIK!=2 && RotFlagSeq[int(gIthIK/(gStepSample))]) //&&gIthIK%(gStepSample/2)==0 
		//	{
		//		//if(gIthIK%(gStepSample)==0)
		//		//RotPhaseFlag=1;
		//		//else
		//		//RotPhaseFlag= RotPhaseFlag*-1+1;

		//		if(selIK == RightSupport  &&  gIthIK%gStepSample==100    )
		//		{
		//			ThetaInital=FKLLeg->theta[5];
		//			RotPhaseFlag= 1;
		//		}
		//		else if(selIK == LeftSupport&&  gIthIK%gStepSample==100     )
		//		{
		//			ThetaInital=FKRLeg->theta[5];	
		//			RotPhaseFlag= 1;
		//		}
		//		//QuatFromMat(tRSwing,qTarSwing);
		//		//GetShankCoords();
		//		if(selIK == RightSupport   &&  gIthIK%gStepSample==300      )
		//		{
		//			RotPhaseFlag= 3;
		//		}
		//		if(selIK == LeftSupport   &&  gIthIK%gStepSample==300     )
		//		{
		//			RotPhaseFlag= 3;
		//		}
		//		
		//		if (RotPhaseFlag==3)
		//		{
		//			double eye[9]={1,0,0,0,1,0,0,0,1};
		//			GetLegsCoords();

		//			if(selIK == RightSupport)
		//				for(int i=0;i<9;i++)
		//				TempMat[i]=LLegRotM[i];					
		//			else
		//				for(int i=0;i<9;i++)
		//				TempMat[i]=RLegRotM[i];		

		//			QuatFromMat(TempMat,qTarSwing);
		//			QuatFromMat(eye,qShankSwing);
		//			QuatInitSLERP(qTarSwing,qShankSwing);
		//			RotPhaseFlag=2;
		//		}
		//		if(gIthIK%gStepSample==1119      )
		//		{
		//			RotPhaseFlag= 0;
		//		}
		//		
		//	}
		//	if(selIK!=2 && RotFlagSeq[int(gIthIK/(gStepSample))])
		//	{
		//		if (RotPhaseFlag==1)
		//		{
		//			if(selIK == RightSupport)
		//			{
		//				ThetaTemp=FKLLeg->theta[5];
		//				FKLLeg->theta[5]=ThetaInital -1.0*ThetaInital*(1-SwingFreeCoef[gIthIK%gStepSample]);//2/180*3.1415926*(1-SwingFreeCoef[gIthIK%gStepSample]);
		//				FindFK();
		//				GetLegsCoords(); 
		//				for(int i=0;i<9;i++)
		//				tRSwing[i]=LLegRotM[i];
		//				FKLLeg->theta[5]=ThetaTemp;
		//				FindFK();
		//			}
		//			else
		//			{
		//				ThetaTemp=FKRLeg->theta[5];
		//				FKRLeg->theta[5]=ThetaInital-1.0*ThetaInital*(1-SwingFreeCoef[gIthIK%gStepSample]);//*SwingFreeCoef[gIthIK%gStepSample];
		//				FindFK();
		//				GetLegsCoords(); 
		//				for(int i=0;i<9;i++)
		//				tRSwing[i]=RLegRotM[i];
		//				FKRLeg->theta[5]=ThetaTemp;
		//				FindFK();
		//			}
		//		}
		//		else if(RotPhaseFlag==2)
		//		{
		//				QuatSLERP((SwingFreeCoef[gIthIK%gStepSample]),qChangedSwing);
		//				QuatToMat(qChangedSwing,tRSwing);
		//				//for(int i=0;i<3;i++)
		//				//{
		//				//	for(int j=0;j<3;j++)
		//				//		cout<<tRSwing[3*i+j]<<"\t";
		//				//	cout<<endl;
		//				//}
		//				//system("pause");
		//				//cout<<"QQ"<<endl;
		//		}

		//		
		//	}
		//////Slongz 20130122

		//泓逸start111227
		LArmErr[0] = tLArm[0] - CrdAll->data[105];
		LArmErr[1] = tLArm[1] - CrdAll->data[106];
		LArmErr[2] = tLArm[2] - CrdAll->data[107];
		RArmErr[0] = tRArm[0] - CrdAll->data[135];
		RArmErr[1] = tRArm[1] - CrdAll->data[136];
		RArmErr[2] = tRArm[2] - CrdAll->data[137];
		//泓逸end111227

		// COG error
		COGErr[0] = tCOG[0] - COG[0];
		COGErr[1] = tCOG[1] - COG[1];
		COGErr[2] = tCOG[2] - COG[2];

		// angle error
		GetLegsCoords(); // Also compute body coordinates

		for (int i = 0 ; i < 9 ; i++)
		{
			if(check_slopeangle == 1)
			TarRotMBody[i] =TempMat2[i];//
			else if(gUpStair)
				TarRotMBody[i]=	tRFixed[i];
			else
				TarRotMBody[i]=	(tRSwing[i]+tRFixed[i])/2.0;
		}
		// normalize
		temp_scale = sqrt(TarRotMBody[0]*TarRotMBody[0]+TarRotMBody[3]*TarRotMBody[3]+TarRotMBody[6]*TarRotMBody[6]);
		TarRotMBody[0]/=temp_scale;
		TarRotMBody[3]/=temp_scale;
		TarRotMBody[6]/=temp_scale;

		temp_scale = sqrt(TarRotMBody[1]*TarRotMBody[1]+TarRotMBody[4]*TarRotMBody[4]+TarRotMBody[7]*TarRotMBody[7]);
		TarRotMBody[1]/=temp_scale;
		TarRotMBody[4]/=temp_scale;
		TarRotMBody[7]/=temp_scale;

		temp_vec1[0] = TarRotMBody[0]; temp_vec1[1] = TarRotMBody[3]; temp_vec1[2] = TarRotMBody[6]; 
		temp_vec2[0] = TarRotMBody[1]; temp_vec2[1] = TarRotMBody[4]; temp_vec2[2] = TarRotMBody[7]; 
		Cross2Vd(temp_vec1, temp_vec2, temp_vec3);	// TarRotMBody新的Z軸 由XY外積而來

		TarRotMBody[2] = temp_vec3[0];
		TarRotMBody[5] = temp_vec3[1]; 
		TarRotMBody[8] = temp_vec3[2]; 
		temp_scale = sqrt(TarRotMBody[2]*TarRotMBody[2]+TarRotMBody[5]*TarRotMBody[5]+TarRotMBody[8]*TarRotMBody[8]);	// normalize
		TarRotMBody[2]/=temp_scale;
		TarRotMBody[5]/=temp_scale;
		TarRotMBody[8]/=temp_scale;

		temp_vec3[0] = TarRotMBody[2]; temp_vec3[1] = TarRotMBody[5]; temp_vec3[2] = TarRotMBody[8];
		Cross2Vd(temp_vec3, temp_vec1, temp_vec2);
		TarRotMBody[1] =temp_vec2[0];
		TarRotMBody[4] =temp_vec2[1];
		TarRotMBody[7] =temp_vec2[2];

		if (selIK == 0)
			MatMulABt(tRSwing ,3,3,RLegRotM, 3,3, DiffRotMatSw);
		else
			MatMulABt(tRSwing ,3,3,LLegRotM , 3,3, DiffRotMatSw);

		MatMulABt(TarRotMBody ,3,3,BodyRotM , 3,3, DiffRotMatBody);

		temp_vec1[0] = DiffRotMatBody[7];//*SwingFreeCoef[gIthIK%gStepSample];
		temp_vec1[1] = DiffRotMatBody[2];
		temp_vec1[2] = DiffRotMatBody[3];
		temp_vec2[0] = DiffRotMatSw[7];
		temp_vec2[1] = DiffRotMatSw[2];
		temp_vec2[2] = DiffRotMatSw[3];
	
			//for(int i = 0;i<9;i++)
			//	cout<<tRSwing[i]<<endl;

		MatMulABt(tRLArm ,3,3,LArmRotM , 3,3, DiffRotMatLA);
		MatMulABt(tRRArm ,3,3,RArmRotM , 3,3, DiffRotMatRA);

		temp_vec3[0] = DiffRotMatLA[7];
		temp_vec3[1] = DiffRotMatLA[2];
		temp_vec3[2] = DiffRotMatLA[3];
		temp_vec4[0] = DiffRotMatRA[7];
		temp_vec4[1] = DiffRotMatRA[2];
		temp_vec4[2] = DiffRotMatRA[3];


		// check whether the input trajectories continuous or not

		//MaxSwingError = 0.5; // maximum input position trajectory difference 防止機構衝太快
	    // 0.5 * 200 = 100mm/s 最快每秒10cm
	    //MaxCOGError = 0.25; //maximum input position trajectory difference 防止機構衝太快
	    // 0.25 * 200 = 50mm/s 最快每秒5cm
	    //MaxRotationError = 0.0035; // maximum input angle trajectory difference 防止機構衝太快
    	// 0.0035 * 200 * 57 每秒最快40度
		if (NormXYZD(SwingErr) >=  MaxSwingError)
		{
			printf("\n警告!! 輸入 Swing 軌跡不連續 或者 IK 爆掉!!\n");
			printf("error = %f mm\n",NormXYZD(SwingErr));
		//	system("pause");
		}
		if (NormXYZD(COGErr) >=  MaxCOGError)
		{
			printf("\n警告!! 輸入 COG 軌跡不連續 或者 IK 爆掉!!\n");
			printf("error = %f mm\n",NormXYZD(COGErr));
		//	system("pause");
		}
		if (NormXYZD(temp_vec2) >=  MaxRotationError)
		{
			printf("\n警告!! 輸入 Swing旋轉 軌跡不連續 或者 IK 爆掉!!\n");
			printf("rotation speed = %f X 200/s\n",NormXYZD(temp_vec2));
		//	system("pause");
		}
		if (NormXYZD(temp_vec1) >=  MaxRotationError)
		{
			printf("\n警告!! 輸入 Stance Foot 旋轉 軌跡不連續 或者 IK 爆掉!!\n");
			printf("rotation speed = %f X 200/s\n",NormXYZD(temp_vec1));
		//	system("pause");
		}
		if(IKMode == 0)
		{
			if (NormXYZD(temp_vec3) >=  MaxRotationError)
			{
				printf("\n警告!! 輸入 LeftArm 旋轉 軌跡不連續 或者 IK 爆掉!!\n");
				printf("rotation speed = %f X 200/s\n",NormXYZD(temp_vec3));
			//	system("pause");
			}
			if (NormXYZD(temp_vec4) >=  MaxRotationError)
			{
				printf("\n警告!! 輸入 RightArm 旋轉 軌跡不連續 或者 IK 爆掉!!\n");
				printf("rotation speed = %f X 200/s\n",NormXYZD(temp_vec4));
			//	system("pause");
			}
			if (NormXYZD(LArmErr) >=  MaxSwingError)
			{
				printf("\n警告!! 輸入 LArmErr 軌跡不連續 或者 IK 爆掉!!\n");
				printf("error = %f mm\n",NormXYZD(LArmErr));
			//	system("pause");
			}
			if (NormXYZD(RArmErr) >=  MaxSwingError)
			{
				printf("\n警告!! 輸入 RArmErr 軌跡不連續 或者 IK 爆掉!!\n");
				printf("error = %f mm\n",NormXYZD(RArmErr));
			//	system("pause");
			}
		}

		if(IKMode == 0)
		{
			if (NormXYZD(SwingErr) < SwErrLim && NormXYZD(COGErr) < COGErrLim && NormXYZD(temp_vec2) < AngleErrLim && NormXYZD(temp_vec1) < AngleErrLim && NormXYZD(LArmErr) < SwErrLim && NormXYZD(RArmErr) < SwErrLim  && NormXYZD(temp_vec3) < AngleErrLim && NormXYZD(temp_vec4) < AngleErrLim)
			{
				//printf("CNT = %d \n",cntIK);
				cntIK = 0;
				*status = 1; // IK succeed!!
				break;
			}
		}
		else if(IKMode == 1)	// 只解腳
		{
			if (NormXYZD(SwingErr) < SwErrLim && NormXYZD(COGErr) < COGErrLim && NormXYZD(temp_vec2) < AngleErrLim && NormXYZD(temp_vec1) < AngleErrLim )
			{
				//printf("CNT = %d \n",cntIK);
				cntIK = 0;
				*status = 1; // IK succeed!!
				RotFlag=1;
				//RotPhaseFlag=1;
				break;
			}
		
		}

		if (cntIK >= 30)
		{
			//cntIK = 0;
			*status = 0; // IK failed!!
			printf("Too many IK iterations \n");
			system("pause");
			break;
		}

		ComputeJacobians();	// 在30次以內 沒有解成功
		cntIK += 1;


	// when selIK = 0,   dx = [sw_x sw_y sw_z sw_tx sw_ty sw_tz fix_tx fix_ty fix_tz COGx COGy COGz ]
	// when selIK = 1,2, dx = [sw_x sw_y sw_z sw_tx sw_ty sw_tz fix_tx fix_ty fix_tz COGx COGy COGz ]
	// 被排成相同的!!

	// dth => [左腳6軸 ; 右腳6軸]
	// selIK = 0 : left foot is the supporter
    // selIK = 1 : right foot is the supporter

		if(IKMode == 0)
		{
			dx[0] = SwingErr[0];
			dx[1] = SwingErr[1];
			dx[2] = SwingErr[2];
			dx[3] = DiffRotMatSw[7];
			dx[4] = DiffRotMatSw[2];
			dx[5] = DiffRotMatSw[3];
			dx[6] = -DiffRotMatBody[7];
			dx[7] = -DiffRotMatBody[2];
			dx[8] = -DiffRotMatBody[3];
			dx[9] = LArmErr[0];
			dx[10] = LArmErr[1];
			dx[11] = LArmErr[2];
			dx[12] = DiffRotMatLA[7];
			dx[13] = DiffRotMatLA[2];
			dx[14] = DiffRotMatLA[3];
			dx[15] = RArmErr[0];
			dx[16] = RArmErr[1];
			dx[17] = RArmErr[2];
			dx[18] = DiffRotMatRA[7];
			dx[19] = DiffRotMatRA[2];
			dx[20] = DiffRotMatRA[3];
			dx[21] = COGErr[0];
			dx[22] = COGErr[1];
			dx[23] = COGErr[2];
		}
		else if(IKMode == 1)
		{
			dxArmOffline[0] = SwingErr[0];
			dxArmOffline[1] = SwingErr[1];
			dxArmOffline[2] = SwingErr[2];
			dxArmOffline[3] = DiffRotMatSw[7];
			dxArmOffline[4] = DiffRotMatSw[2];
			dxArmOffline[5] = DiffRotMatSw[3];
			dxArmOffline[6] = -DiffRotMatBody[7];
			dxArmOffline[7] = -DiffRotMatBody[2];
			dxArmOffline[8] = -DiffRotMatBody[3];
			dxArmOffline[9] = COGErr[0];
			dxArmOffline[10] = COGErr[1];
			dxArmOffline[11] = COGErr[2];
		}

		//FindWLN();
		//// disable WLN
		//for (int p = 0 ; p < 12 ; p++)
		//	invWLNMat[p] = 1;

		////// 不包含WLN 的IK
		////InvSqMat(Ja->data,12);
		////MatMulAB(Ja->data,12,12,dx,12,1,dth);
		////// 不包含WLN 的IK


		//// 包含WLN 的IK
		//MatIndex = 0;
		//// compute tempJT = J*inv(W)'
		//for (int i =0 ; i< Ja->MRow ; i++) 
		//{
		//	for (int j = 0 ; j< Ja->NCol ; j++)
		//	{
		//		tempJT[MatIndex]=Ja->data[MatIndex]*invWLNMat[j];
		//		MatIndex += 1;
		//	}
		//}

		if(IKMode == 0)
		{
			for (int i = 0 ; i < Ja->MSize ; i++) // copy J
			{

				tempJ[i] = Ja->data[i];
			}

			InvSqMat(tempJ,Ja->MRow);
			MatMulAB(tempJ,Ja->MRow,Ja->MRow,dx,Ja->MRow,1,dth);
			for (int i = 0 ; i < 12 ; i++)
			{
				if (fabs(dth[i]) > MaxJointAngleChange)
				{
					//cout<<dth[i]/3.1415926*180*200<<endl;
					printf("\n警告!! 解出之第%d軸角速度過快 達到 每秒%f度\n",i+1,dth[i]/3.1415926*180*200);
					printf("可能是輸入軌跡不連續或者IK爆掉\n");
					//system("pause");
				}
			}
			for(int i = 0;i<12;i++)
			{
				if (fabs(dth[i+12]) > MaxJointAngleChange)
				{
					//cout<<dth[i]/3.1415926*180*200<<endl;
					printf("\n警告!! 解出之第%d軸角速度過快 達到 每秒%f度\n",i+1+12,dth[i+12]/3.1415926*180*200);
					printf("可能是輸入軌跡不連續或者IK爆掉\n");
					//system("pause");
				}
			}

			for (int i = 0 ; i < 6 ; i++)
			{
				FKLLeg->theta[i+1] += dth[i];
				FKRLeg->theta[i+1] += dth[i+6];
				FKLArm->theta[i+4] += dth[i+12];
				FKRArm->theta[i+4] += dth[i+18];
			}

		}
		else if(IKMode == 1)
		{
			for(int i = 0;i<12;i++)
			{
				for(int j = 0;j<9;j++)
				{
					tempJArmOffline[i+j*12] = Ja->data[i+j*24];
				}
				for(int j = 0;j<3;j++)
				{
					tempJArmOffline[i + j*12 + 9*12] = Ja->data[i + j*24 + 21*24];
				}
			}

			InvSqMat(tempJArmOffline,12);
			MatMulAB(tempJArmOffline,12,12,dxArmOffline,12,1,dthArmOffline);
			for (int i = 0 ; i < 12 ; i++)
			{
				if (fabs(dthArmOffline[i]) > MaxJointAngleChange)
				{
					//cout<<dthArmOffline[i]/3.1415926*180*200<<endl;
					printf("\n警告!! 解出之第%d軸角速度過快 達到 每秒%f度\n",i+1,dthArmOffline[i]/3.1415926*180*200);
					printf("可能是輸入軌跡不連續或者IK爆掉\n");
					//system("pause");
				}
			}

			for (int i = 0 ; i < 6 ; i++)
			{
				FKLLeg->theta[i+1] += dthArmOffline[i];
				FKRLeg->theta[i+1] += dthArmOffline[i+6];
			}
		}
		//// compute J*inv(W)*J'
		//MatMulABt(tempJ,Ja->MRow,Ja->NCol,tempJT,Ja->MRow,Ja->NCol,tempJiWJT);

		//// compute inv(J*inv(W)*J')
		////InvSqMat(tempJiWJT,Ja->MRow);

		////SingularJudge = 0.0000000000001; // 10^-13
		////SingularAdd = 0.1;
		//InvWithSingTest(tempJiWJT, tempJJT_for_inv, Ja->MRow, ipiv_clapack, work_clapack, SingularJudge, SingularAdd);

		//// compute inv(J*inv(W)*J')*dx
		////MatMulAB(tempJiWJT,Ja->MRow,Ja->MRow,dx,Ja->MRow,1,tempInv);
		//MatMulAB(tempJJT_for_inv,Ja->MRow,Ja->MRow,dx,Ja->MRow,1,tempInv);

		//// compute dth = inv(W)*J'*inv(J*inv(W)*J')*dx
		//MatMulAtB(tempJT,Ja->MRow,Ja->NCol,tempInv,Ja->MRow,1,dth);

		//// 包含WLN 的IK

		//// PseudoInv
		//MatMulAtB(tempJT,Ja->MRow,Ja->NCol,tempJJT_for_inv,Ja->MRow,Ja->MRow,PseudoInv);
		
		//printf("\n\n\n");
		//for(int i = 0;i<24;i++)
		//	printf("%f   ",dx[i]);
		//printf("\n\n\n");
		//for(int i = 0;i<24;i++)
		//	printf("%f   ",dth[i]);
		//printf("\n\n\n");

		FindFK();
		FindCOG();
	}

	// 檢查機器人是否超過角度極限 若超過 則停止求解 程式暫停
	CheckJointLimit();

}

void Kine::IKFixSolve(double* tCOG, double* tSwing, double* tRSwing, double* tRFixed, int* status)
{
	/******************************************************************
	input: tCOG 代表 target COG, tSwing 代表 target Swing, tRSwing 代表 target Swing rotation matrix,
	       tRFixed 代表 target Fixed rotation matrix, status 回傳IK是否成功

	output: void

	Note:
	// 解IK讓各個end-effector達到目標位置與角度

	// 關於旋轉矩陣之解說
		// 下面解說求出不同坐標系之間角速度 並且丟到IK input之方法
		// 假設有兩相近坐標系(R1 R2 都是3x3矩陣)
		// 要將R1的三個軸 都轉到R2上， 必須左乘R到R1上
		// R是微小旋轉量之旋轉矩陣， 不是右乘 因為坐標軸是向量 是被旋轉的目標
		// 所以 R x R1 = R2, R = R2 x R1' 如同下面式子
		// 而且 假設 R = RxRyRz 並且是可交換的微小旋轉
		// RxRyRz = [1 -z y ; z 1 -x ; -y x 1]; 
		// 所以可以取出DiffRotMatSw DiffRotMatBody 在世界下的旋轉軸
		// 直接將此旋轉軸的向量輸入IK 就可以讓end-effector旋轉了!!
	******************************************************************/

	// selIK = 0 : left foot is the supporter
    // selIK = 1 : right foot is the supporter

	//SwErrLim = 0.1; // swing 解IK 誤差值
	//COGErrLim = 0.1; // COG 解IK 誤差值
	double TempCOG[3];
	int MatIndex = 0;

	cntIK = 0;
				//bool flagQQ=1;
	while (1)
	{
		// distance error
		if (selIK == 0)
		{

		}
		else if (selIK ==1 ||selIK == 2)
		{
 
		}

		// COG error
 		COGErr[0] = tCOG[0] - COG[0];
		COGErr[1] = tCOG[1] - COG[1];
		COGErr[2] = tCOG[2] - COG[2];

		// angle error
		GetLegsCoords(); // Also compute body coordinates

		for (int i = 0 ; i < 9 ; i++)
			TarRotMBody[i] = (tRSwing[i]+tRFixed[i])/2.0;
		
		// normalize
		temp_scale = sqrt(TarRotMBody[0]*TarRotMBody[0]+TarRotMBody[3]*TarRotMBody[3]+TarRotMBody[6]*TarRotMBody[6]);
		TarRotMBody[0]/=temp_scale;
		TarRotMBody[3]/=temp_scale;
		TarRotMBody[6]/=temp_scale;

		temp_scale = sqrt(TarRotMBody[1]*TarRotMBody[1]+TarRotMBody[4]*TarRotMBody[4]+TarRotMBody[7]*TarRotMBody[7]);
		TarRotMBody[1]/=temp_scale;
		TarRotMBody[4]/=temp_scale;
		TarRotMBody[7]/=temp_scale;

		temp_vec1[0] = TarRotMBody[0]; temp_vec1[1] = TarRotMBody[3]; temp_vec1[2] = TarRotMBody[6]; 
		temp_vec2[0] = TarRotMBody[1]; temp_vec2[1] = TarRotMBody[4]; temp_vec2[2] = TarRotMBody[7]; 
		Cross2Vd(temp_vec1, temp_vec2, temp_vec3);

		TarRotMBody[2] = temp_vec3[0];
		TarRotMBody[5] = temp_vec3[1]; 
		TarRotMBody[8] = temp_vec3[2]; 
		temp_scale = sqrt(TarRotMBody[2]*TarRotMBody[2]+TarRotMBody[5]*TarRotMBody[5]+TarRotMBody[8]*TarRotMBody[8]);
		TarRotMBody[2]/=temp_scale;
		TarRotMBody[5]/=temp_scale;
		TarRotMBody[8]/=temp_scale;

		temp_vec3[0] = TarRotMBody[2]; temp_vec3[1] = TarRotMBody[5]; temp_vec3[2] = TarRotMBody[8];
		Cross2Vd(temp_vec3, temp_vec1, temp_vec2);
		TarRotMBody[1] =temp_vec2[0];
		TarRotMBody[4] =temp_vec2[1];
		TarRotMBody[7] =temp_vec2[2];

		if (selIK == LeftSupport)
			MatMulABt(tRSwing ,3,3,RLegRotM, 3,3, DiffRotMatSw);
		else
			MatMulABt(tRSwing ,3,3,LLegRotM , 3,3, DiffRotMatSw);

		MatMulABt(TarRotMBody ,3,3,BodyRotM , 3,3, DiffRotMatBody);

		temp_vec1[0] = DiffRotMatBody[7];
		temp_vec1[1] = DiffRotMatBody[2];
		temp_vec1[2] = DiffRotMatBody[3];
		temp_vec2[0] = DiffRotMatSw[7];
		temp_vec2[1] = DiffRotMatSw[2];
		temp_vec2[2] = DiffRotMatSw[3];

		// check whether the input trajectories continuous or not

		//MaxSwingError = 0.5; // maximum input position trajectory difference 防止機構衝太快
	    // 0.5 * 200 = 100mm/s 最快每秒10cm
	    //MaxCOGError = 0.25; //maximum input position trajectory difference 防止機構衝太快
	    // 0.25 * 200 = 50mm/s 最快每秒5cm
	    //MaxRotationError = 0.0035; // maximum input angle trajectory difference 防止機構衝太快
    	// 0.0035 * 200 * 57 每秒最快40度
		//if (NormXYZD(SwingErr) >=  MaxSwingError)
		//{
		//	printf("\n警告!! 輸入 Swing 軌跡不連續 或者 IK 爆掉!!\n");
		//	printf("error = %f mm\n",NormXYZD(SwingErr));
		//	system("pause");
		//}
		if (NormXYZD(COGErr) >=  MaxCOGError)
		{
			printf("\n警告!! 輸入 COG 軌跡不連續 或者 IK 爆掉!!\n");
			printf("error = %f mm\n",NormXYZD(COGErr));
			system("pause");
		}
		//if (NormXYZD(temp_vec2) >=  MaxRotationError)
		//{
		//	printf("\n警告!! 輸入 Swing旋轉 軌跡不連續 或者 IK 爆掉!!\n");
		//	printf("rotation speed = %f X 200/s\n",NormXYZD(temp_vec2));
		//	system("pause");
		//}


		if (NormXYZD(temp_vec1) >=  MaxRotationError)
		{
			printf("\n警告!! 輸入 Stance Foot 旋轉 軌跡不連續 或者 IK 爆掉!!\n");
			printf("rotation speed = %f X 200/s\n",NormXYZD(temp_vec1));
			system("pause");
		}
					//printf("Knee Angle = %f \n",FKLLeg->theta[4]*180/3.1415926);

		if ( NormXYZD(COGErr) < COGErrLim &&  NormXYZD(temp_vec1) < AngleErrLim)
		{
			//printf("CNT = %d \n",cntIK);
			cntIK = 0;
			*status = 1; // IK succeed!!
			if(selIK==1 ||selIK==2)
			{
				for(int i=0;i<6;i++)
					QFKRLeg->Qth[i]=FKRLeg->theta[i+1];
				FindFixQFK(QFKRLeg,0);

				for(int i=0;i<6;i++)
					QFKLLeg->Qth[i]=FKLLeg->theta[i+1];
				FindSwingQFK(QFKLLeg,1);
				CCDIKSolve(QFKLLeg,FKLLeg,tSwing,QFKLLeg->TarRot,1, status);
			}
			else
			{
				for(int i=0;i<6;i++)
					QFKLLeg->Qth[i]=FKLLeg->theta[i+1];
				FindSwingQFK(QFKLLeg,1);

				for(int i=0;i<6;i++)
					QFKRLeg->Qth[i]=FKRLeg->theta[i+1];
				FindSwingQFK(QFKRLeg,0);
				CCDIKSolve(QFKRLeg,FKRLeg,tSwing,QFKRLeg->TarRot,0, status);
			}
			break;
		}

		if (cntIK >= 50)
		{
			//cntIK = 0;
			*status = 0; // IK failed!!
			printf("Too many IK iterations \n");
			system("pause");
			break;
		}

		ComputeFixJacobians();
		cntIK += 1;


	// when selIK = 0,   dx = [sw_x sw_y sw_z sw_tx sw_ty sw_tz fix_tx fix_ty fix_tz COGx COGy COGz ]
	// when selIK = 1,2, dx = [sw_x sw_y sw_z sw_tx sw_ty sw_tz fix_tx fix_ty fix_tz COGx COGy COGz ]
	// 被排成相同的!!

	// dth => [左腳6軸 ; 右腳6軸]
	// selIK = 0 : left foot is the supporter
    // selIK = 1 : right foot is the supporter

 	//	dx[0] = SwingErr[0];
		//dx[1] = SwingErr[1];
		//dx[2] = SwingErr[2];
		//dx[3] = DiffRotMatSw[7];
		//dx[4] = DiffRotMatSw[2];
		//dx[5] = DiffRotMatSw[3];
		dx[0] = -DiffRotMatBody[7];
		dx[1] = -DiffRotMatBody[2];
		dx[2] = -DiffRotMatBody[3];
		dx[3] = COGErr[0];
		dx[4] = COGErr[1];
		dx[5] = COGErr[2];

		FindWLN();
		//// disable WLN
		//for (int p = 0 ; p < 12 ; p++)
		//	invWLNMat[p] = 1;

		////// 不包含WLN 的IK
		InvSqMat(FixJa->data,6);
		MatMulAB(FixJa->data,6,6,dx,6,1,dth);
		////// 不包含WLN 的IK
		double GG[36];
 		for (int i =0 ; i< 36 ; i++) 
		{
		GG[i]=FixJa->data[i];
		}
		double GGdth[6];
		for (int i =0 ; i< 6 ; i++) 
		{
		GGdth[i]=dth[i];
		}



		//// 包含WLN 的IK
		//MatIndex = 0;
		//// compute tempJT = J*inv(W)'
		//for (int i =0 ; i< Ja->MRow ; i++) 
		//{
		//	for (int j = 0 ; j< Ja->NCol ; j++)
		//	{
		//		tempJT[MatIndex]=Ja->data[MatIndex]*invWLNMat[j];
		//		MatIndex += 1;
		//	}
		//}

		//for (int i = 0 ; i < Ja->MSize ; i++) // copy J
		//{
		//	tempJ[i] = Ja->data[i];
		//}

		//// compute J*inv(W)*J'
		//MatMulABt(tempJ,Ja->MRow,Ja->NCol,tempJT,Ja->MRow,Ja->NCol,tempJiWJT);

		//// compute inv(J*inv(W)*J')
		////InvSqMat(tempJiWJT,Ja->MRow);

		////SingularJudge = 0.0000000000001; // 10^-13
		////SingularAdd = 0.1;
		//InvWithSingTest(tempJiWJT, tempJJT_for_inv, Ja->MRow, ipiv_clapack, work_clapack, SingularJudge, SingularAdd);

		//// compute inv(J*inv(W)*J')*dx
		////MatMulAB(tempJiWJT,Ja->MRow,Ja->MRow,dx,Ja->MRow,1,tempInv);
		//MatMulAB(tempJJT_for_inv,Ja->MRow,Ja->MRow,dx,Ja->MRow,1,tempInv);

		//// compute dth = inv(W)*J'*inv(J*inv(W)*J')*dx
		//MatMulAtB(tempJT,Ja->MRow,Ja->NCol,tempInv,Ja->MRow,1,dth);

		//// 包含WLN 的IK

		//// PseudoInv
		//MatMulAtB(tempJT,Ja->MRow,Ja->NCol,tempJJT_for_inv,Ja->MRow,Ja->MRow,PseudoInv);


 		for (int i = 0 ; i < 6 ; i++)
		{
			if (fabs(dth[i]) > MaxJointAngleChange)
			{
				printf("\n警告!! 解出之第%d軸角速度過快 達到 每秒%f度\n",i+1,dth[i]/3.1415926*180*200);
				printf("可能是輸入軌跡不連續或者IK爆掉\n");
				system("pause");
			}
		}

		if (selIK == 1 || selIK == 2)
			for (int i = 0 ; i < 6 ; i++)
			{
				FKLLeg->theta[i+1] += 0;
				FKRLeg->theta[i+1] += dth[i];
			}
		else
			for (int i = 0 ; i < 6 ; i++)
			{
				FKLLeg->theta[i+1] += dth[i];
				FKRLeg->theta[i+1] += 0;
			}


		FindFK();
		FindCOG();

		for(int i=0;i<3;i++)
			COGDev[i]=TempCOG[i]-COG[i];
	}

	// 檢查機器人是否超過角度極限 若超過 則停止求解 程式暫停
	//CheckJointLimit();

}

void Kine::IKCCDSolve(double* tCOG, double* tSwing, double* tRSwing, double* tRFixed, int* status)
{
	/******************************************************************
	input: tCOG 代表 target COG, tSwing 代表 target Swing, tRSwing 代表 target Swing rotation matrix,
	       tRFixed 代表 target Fixed rotation matrix, status 回傳IK是否成功

	output: void

	Note:
	// 解IK讓各個end-effector達到目標位置與角度

	// 關於旋轉矩陣之解說
		// 下面解說求出不同坐標系之間角速度 並且丟到IK input之方法
		// 假設有兩相近坐標系(R1 R2 都是3x3矩陣)
		// 要將R1的三個軸 都轉到R2上， 必須左乘R到R1上
		// R是微小旋轉量之旋轉矩陣， 不是右乘 因為坐標軸是向量 是被旋轉的目標
		// 所以 R x R1 = R2, R = R2 x R1' 如同下面式子
		// 而且 假設 R = RxRyRz 並且是可交換的微小旋轉
		// RxRyRz = [1 -z y ; z 1 -x ; -y x 1]; 
		// 所以可以取出DiffRotMatSw DiffRotMatBody 在世界下的旋轉軸
		// 直接將此旋轉軸的向量輸入IK 就可以讓end-effector旋轉了!!
	******************************************************************/

	// selIK = 0 : left foot is the supporter
    // selIK = 1 : right foot is the supporter

	//SwErrLim = 0.1; // swing 解IK 誤差值
	//COGErrLim = 0.1; // COG 解IK 誤差值

	int MatIndex = 0;

	//cout<<"selIK= "<<selIK<<endl;
	//cntIK = 0;
				//bool flagQQ=1;
	while (1)
	{
		// distance error
		if (selIK == 0)
		{

		}
		else if (selIK ==1 ||selIK == 2)
		{
 
		}

		// COG error
 		COGErr[0] = tCOG[0] - COG[0];
		COGErr[1] = tCOG[1] - COG[1];
		COGErr[2] = tCOG[2] - COG[2];

		// angle error
		GetLegsCoords(); // Also compute body coordinates

		for (int i = 0 ; i < 9 ; i++)
			TarRotMBody[i] = (tRSwing[i]+tRFixed[i])/2.0;
		
		// normalize
		temp_scale = sqrt(TarRotMBody[0]*TarRotMBody[0]+TarRotMBody[3]*TarRotMBody[3]+TarRotMBody[6]*TarRotMBody[6]);
		TarRotMBody[0]/=temp_scale;
		TarRotMBody[3]/=temp_scale;
		TarRotMBody[6]/=temp_scale;

		temp_scale = sqrt(TarRotMBody[1]*TarRotMBody[1]+TarRotMBody[4]*TarRotMBody[4]+TarRotMBody[7]*TarRotMBody[7]);
		TarRotMBody[1]/=temp_scale;
		TarRotMBody[4]/=temp_scale;
		TarRotMBody[7]/=temp_scale;

		temp_vec1[0] = TarRotMBody[0]; temp_vec1[1] = TarRotMBody[3]; temp_vec1[2] = TarRotMBody[6]; 
		temp_vec2[0] = TarRotMBody[1]; temp_vec2[1] = TarRotMBody[4]; temp_vec2[2] = TarRotMBody[7]; 
		Cross2Vd(temp_vec1, temp_vec2, temp_vec3);

		TarRotMBody[2] = temp_vec3[0];
		TarRotMBody[5] = temp_vec3[1]; 
		TarRotMBody[8] = temp_vec3[2]; 
		temp_scale = sqrt(TarRotMBody[2]*TarRotMBody[2]+TarRotMBody[5]*TarRotMBody[5]+TarRotMBody[8]*TarRotMBody[8]);
		TarRotMBody[2]/=temp_scale;
		TarRotMBody[5]/=temp_scale;
		TarRotMBody[8]/=temp_scale;

		temp_vec3[0] = TarRotMBody[2]; temp_vec3[1] = TarRotMBody[5]; temp_vec3[2] = TarRotMBody[8];
		Cross2Vd(temp_vec3, temp_vec1, temp_vec2);
		TarRotMBody[1] =temp_vec2[0];
		TarRotMBody[4] =temp_vec2[1];
		TarRotMBody[7] =temp_vec2[2];

		if (selIK == 0)
			MatMulABt(tRSwing ,3,3,RLegRotM, 3,3, DiffRotMatSw);
		else
			MatMulABt(tRSwing ,3,3,LLegRotM , 3,3, DiffRotMatSw);

		MatMulABt(TarRotMBody ,3,3,BodyRotM , 3,3, DiffRotMatBody);

		temp_vec1[0] = DiffRotMatBody[7];
		temp_vec1[1] = DiffRotMatBody[2];
		temp_vec1[2] = DiffRotMatBody[3];
		temp_vec2[0] = DiffRotMatSw[7];
		temp_vec2[1] = DiffRotMatSw[2];
		temp_vec2[2] = DiffRotMatSw[3];

		// check whether the input trajectories continuous or not

		//MaxSwingError = 0.5; // maximum input position trajectory difference 防止機構衝太快
	    // 0.5 * 200 = 100mm/s 最快每秒10cm
	    //MaxCOGError = 0.25; //maximum input position trajectory difference 防止機構衝太快
	    // 0.25 * 200 = 50mm/s 最快每秒5cm
	    //MaxRotationError = 0.0035; // maximum input angle trajectory difference 防止機構衝太快
    	// 0.0035 * 200 * 57 每秒最快40度
		//if (NormXYZD(SwingErr) >=  MaxSwingError)
		//{
		//	printf("\n警告!! 輸入 Swing 軌跡不連續 或者 IK 爆掉!!\n");
		//	printf("error = %f mm\n",NormXYZD(SwingErr));
		//	system("pause");
		//}
		if (NormXYZD(COGErr) >=  MaxCOGError)
		{
			printf("\n警告!! 輸入 COG 軌跡不連續 或者 IK 爆掉!!\n");
			printf("error = %f mm\n",NormXYZD(COGErr));
			system("pause");
		}
		//if (NormXYZD(temp_vec2) >=  MaxRotationError)
		//{
		//	printf("\n警告!! 輸入 Swing旋轉 軌跡不連續 或者 IK 爆掉!!\n");
		//	printf("rotation speed = %f X 200/s\n",NormXYZD(temp_vec2));
		//	system("pause");
		//}


		if (NormXYZD(temp_vec1) >=  MaxRotationError)
		{
			printf("\n警告!! 輸入 Stance Foot 旋轉 軌跡不連續 或者 IK 爆掉!!\n");
			printf("rotation speed = %f X 200/s\n",NormXYZD(temp_vec1));
			system("pause");
		}
					//printf("Knee Angle = %f \n",FKLLeg->theta[4]*180/3.1415926);

		if ( NormXYZD(COGErr) < COGErrLim )//&&  NormXYZD(temp_vec1) < AngleErrLim)
		{
			//printf("CNT = %d \n",cntIK);
			cntIK = 0;
			*status = 1; // IK succeed!!
			if(selIK==1 ||selIK==2)
			{
				for(int i=0;i<6;i++)
					QFKLLeg->Qth[i]=FKLLeg->theta[i+1];
				FindSwingQFK(QFKLLeg,1);
				CCDIKSolve(QFKLLeg,FKLLeg,tSwing,QFKLLeg->TarRot,1, status);
			}
			else
			{
				for(int i=0;i<6;i++)
					QFKRLeg->Qth[i]=FKRLeg->theta[i+1];
				FindSwingQFK(QFKRLeg,0);
				CCDIKSolve(QFKRLeg,FKRLeg,tSwing,QFKRLeg->TarRot,0, status);
			}
			break;
		}

		//if (cntIK >= 50)
		//{
		//	//cntIK = 0;
		//	*status = 0; // IK failed!!
		//	printf("Too many IK iterations \n");
		//	system("pause");
		//	break;
		//}

		//ComputeFixJacobians();
		cntIK += 1;


	// when selIK = 0,   dx = [sw_x sw_y sw_z sw_tx sw_ty sw_tz fix_tx fix_ty fix_tz COGx COGy COGz ]
	// when selIK = 1,2, dx = [sw_x sw_y sw_z sw_tx sw_ty sw_tz fix_tx fix_ty fix_tz COGx COGy COGz ]
	// 被排成相同的!!

	// dth => [左腳6軸 ; 右腳6軸]
	// selIK = 0 : left foot is the supporter
    // selIK = 1 : right foot is the supporter

 	//	dx[0] = SwingErr[0];
		//dx[1] = SwingErr[1];
		//dx[2] = SwingErr[2];
		//dx[3] = DiffRotMatSw[7];
		//dx[4] = DiffRotMatSw[2];
		//dx[5] = DiffRotMatSw[3];
		dx[0] = -DiffRotMatBody[7];
		dx[1] = -DiffRotMatBody[2];
		dx[2] = -DiffRotMatBody[3];
		dx[3] = COGErr[0];
		dx[4] = COGErr[1];
		dx[5] = COGErr[2];

		//FindWLN();
		//// disable WLN
		//for (int p = 0 ; p < 12 ; p++)
		//	invWLNMat[p] = 1;

		////// 不包含WLN 的IK
		//__________________________________________________S
		if(selIK==1 ||selIK==2)
		{
			for(int i=0;i<6;i++)
				dth[i]=0;
			//	QFKRLeg->Qth[i]=FKRLeg->theta[i+1];
			//FindSwingQFK(QFKRLeg,0);
			CCDCOGSolve(QFKRLeg,FKRLeg,tCOG,DiffRotMatBody,0, status);
		} 
		else
		{
			for(int i=0;i<6;i++)
				dth[i]=0;
			//	QFKRLeg->Qth[i]=FKRLeg->theta[i+1];
			//FindSwingQFK(QFKRLeg,0);
			CCDCOGSolve(QFKLLeg,FKLLeg,tCOG,DiffRotMatBody,1, status);

			//for(int i=0;i<6;i++)
			//	QFKRLeg->Qth[i]=FKRLeg->theta[i+1];
			//FindQFK(QFKRLeg,0);
			//CCDCOGSolve(QFKRLeg,FKRLeg,tCOG,tRFixed,0, status);
		}
		//__________________________________________________S
		//InvSqMat(FixJa->data,6);
		//MatMulAB(FixJa->data,6,6,dx,6,1,dth);
		////// 不包含WLN 的IK



		//// 包含WLN 的IK
		//MatIndex = 0;
		//// compute tempJT = J*inv(W)'
		//for (int i =0 ; i< Ja->MRow ; i++) 
		//{
		//	for (int j = 0 ; j< Ja->NCol ; j++)
		//	{
		//		tempJT[MatIndex]=Ja->data[MatIndex]*invWLNMat[j];
		//		MatIndex += 1;
		//	}
		//}

		//for (int i = 0 ; i < Ja->MSize ; i++) // copy J
		//{
		//	tempJ[i] = Ja->data[i];
		//}

		//// compute J*inv(W)*J'
		//MatMulABt(tempJ,Ja->MRow,Ja->NCol,tempJT,Ja->MRow,Ja->NCol,tempJiWJT);

		//// compute inv(J*inv(W)*J')
		////InvSqMat(tempJiWJT,Ja->MRow);

		////SingularJudge = 0.0000000000001; // 10^-13
		////SingularAdd = 0.1;
		//InvWithSingTest(tempJiWJT, tempJJT_for_inv, Ja->MRow, ipiv_clapack, work_clapack, SingularJudge, SingularAdd);

		//// compute inv(J*inv(W)*J')*dx
		////MatMulAB(tempJiWJT,Ja->MRow,Ja->MRow,dx,Ja->MRow,1,tempInv);
		//MatMulAB(tempJJT_for_inv,Ja->MRow,Ja->MRow,dx,Ja->MRow,1,tempInv);

		//// compute dth = inv(W)*J'*inv(J*inv(W)*J')*dx
		//MatMulAtB(tempJT,Ja->MRow,Ja->NCol,tempInv,Ja->MRow,1,dth);

		//// 包含WLN 的IK

		//// PseudoInv
		//MatMulAtB(tempJT,Ja->MRow,Ja->NCol,tempJJT_for_inv,Ja->MRow,Ja->MRow,PseudoInv);


 		for (int i = 0 ; i < 6 ; i++)
		{
			if (fabs(dth[i]) > MaxJointAngleChange)
			{
				printf("\n警告!! QQ解出之第%d軸角速度過快 達到 每秒%f度\n",i+1,dth[i]/3.1415926*180*200);
				printf("可能是輸入軌跡不連續或者IK爆掉\n");
				system("pause");
			}
		}

		//if (selIK == 1 || selIK == 2)
		//	for (int i = 0 ; i < 6 ; i++)
		//	{
		//		//FKLLeg->theta[i+1] += 0;
		//		//FKRLeg->theta[i+1] += dth[i];
		//	}
		//else
		//	for (int i = 0 ; i < 6 ; i++)
		//	{
		//		//FKLLeg->theta[i+1] += dth[i];
		//		//FKRLeg->theta[i+1] += 0;
		//	}


		FindFK();
		FindCOG();
	}

	// 檢查機器人是否超過角度極限 若超過 則停止求解 程式暫停
	//CheckJointLimit();

}

void Kine::InitWLN(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	   初始化WLN joint limit avoidance 所要的變數
	******************************************************************/

 // // H Yaw			H Roll			H Pitch			K Pitch			A Pitch			 A Roll	
	//MaxLL[0] = 45;  MaxLL[1] = 37; 	MaxLL[2] = 15;  MaxLL[3] = 80;  MaxLL[4] = 38;   MaxLL[5] = 25;
	//MinLL[0] = -45;	MinLL[1] = -12; MinLL[2] = -53;	MinLL[3] = -10; MinLL[4] = -38;  MinLL[5] = -40;

	//MaxRL[0] = 45;  MaxRL[1] = 12; 	MaxRL[2] = 53;  MaxRL[3] = 10;  MaxRL[4] = 38;   MaxRL[5] = 40;
	//MinRL[0] = -45;	MinRL[1] = -37; MinRL[2] = -15;	MinRL[3] = -80; MinRL[4] = -38;  MinRL[5] = -25;

  // H Yaw			H Roll			H Pitch			K Pitch			A Pitch			 A Roll	
	MaxLL[0] = 55;  MaxLL[1] = 55; 	MaxLL[2] = 55;  MaxLL[3] = 90;  MaxLL[4] = 55;   MaxLL[5] = 55;
	MinLL[0] = -55;	MinLL[1] = -55; MinLL[2] = -90;	MinLL[3] = -20; MinLL[4] = -55;  MinLL[5] = -55;

	MaxRL[0] = 55;  MaxRL[1] = 55; 	MaxRL[2] = 90;  MaxRL[3] = 20;  MaxRL[4] = 55;   MaxRL[5] = 55;
	MinRL[0] = -55;	MinRL[1] = -55; MinRL[2] = -55;	MinRL[3] = -90; MinRL[4] = -55;  MinRL[5] = -55;

  // H Yaw					H Roll					H Pitch					K Pitch					A Pitch					A Roll	
	MaxLLBound[0] = 50;	MaxLLBound[1] = 50; 	MaxLLBound[2] = 50;	MaxLLBound[3] = 85;	MaxLLBound[4] = 50;    MaxLLBound[5] = 50;
	MinLLBound[0] = -50;	MinLLBound[1] = -50;	MinLLBound[2] = -85;	MinLLBound[3] = 5;		MinLLBound[4] = -50;   MinLLBound[5] = -50;

	MaxRLBound[0] = 50;	 MaxRLBound[1] = 50; 	MaxRLBound[2] = 85;	MaxRLBound[3] = -5;	MaxRLBound[4] = 50;	MaxRLBound[5] = 50;
	MinRLBound[0] = -50;	MinRLBound[1] = -50;	MinRLBound[2] = -50;	MinRLBound[3] = -85;	MinRLBound[4] = -50;	MinRLBound[5] = -50;


	for (int i = 0 ; i < 6 ; i++)
	{
		MaxLL[i] = MaxLL[i]/180.0*3.1415926;
		MaxRL[i] = MaxRL[i]/180.0*3.1415926;
		MinLL[i] = MinLL[i]/180.0*3.1415926;
		MinRL[i] = MinRL[i]/180.0*3.1415926;
	}

	// 用來限制IK解完超出不該有的邊界
	for (int i = 0 ; i < 6 ; i++)
	{
		MaxLLBound[i] = MaxLLBound[i]/180.0*3.1415926;
		MinLLBound[i] = MinLLBound[i]/180.0*3.1415926;
		MaxRLBound[i] = MaxRLBound[i]/180.0*3.1415926;
		MinRLBound[i] = MinRLBound[i]/180.0*3.1415926;
	}

	//double A_PC_LL[6]; // pre calculated para A for LL
	//double A_PC_RL[6]; // pre calculated para A for RL
	//double B_PC_LL[6]; // pre calculated para B for LL
	//double B_PC_RL[6]; // pre calculated para B for RL

	for (int i = 0 ; i < 6 ; i++)
	{
		A_PC_LL[i] = 0.25*(MaxLL[i]-MinLL[i])*(MaxLL[i]-MinLL[i]);
		A_PC_RL[i] = 0.25*(MaxRL[i]-MinRL[i])*(MaxRL[i]-MinRL[i]);
		B_PC_LL[i] = MaxLL[i]+MinLL[i];
		B_PC_RL[i] = MaxRL[i]+MinRL[i];
	}

	for (int i =0 ; i< 12 ; i++) // initialize it
	{
		invWLNMat[i] = 1;
		LastInvWLN_Mat[i] = 1;
		IfWLNEqualOne[i] = true;
	}

}

void Kine::FindWLN(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	    計算WLN joint limit avoid 權重矩陣
	******************************************************************/

	// dth => [左腳6軸 ; 右腳6軸]
	// selIK = 0 : left foot is the supporter
    // selIK = 1 : right foot is the supporter

	// find wi
	for (int i = 0 ; i < 6 ; i++)
	{
		TempTh = FKLLeg->theta[i+1];
		TempSquareVal = (MaxLL[i]-TempTh)*(MinLL[i]-TempTh);
		TempSquareVal = TempSquareVal*TempSquareVal;
		if (TempSquareVal < 0.000000001)
			TempSquareVal = 0.000000001;
		invWLNMat[i] = A_PC_LL[i]*(2.0*TempTh-B_PC_LL[i])/TempSquareVal;
	}

	for (int i = 0 ; i < 6 ; i++)
	{
		TempTh = FKRLeg->theta[i+1];
		TempSquareVal = (MaxRL[i]-TempTh)*(MinRL[i]-TempTh);
		TempSquareVal = TempSquareVal*TempSquareVal;
		if (TempSquareVal < 0.00000000001)
			TempSquareVal = 0.00000000001;

		invWLNMat[i+6] = A_PC_RL[i]*(2.0*TempTh-B_PC_RL[i])/TempSquareVal;
	}

	// inv_WLN_Mat_[i] = 1/(1+abs(wi))
	for (int i = 0 ; i < 12 ; i++)
	{
		if (invWLNMat[i] >= 0 )
		{
			invWLNMat[i] = 1.0/(invWLNMat[i] + 1.0);
		}
		else
		{
			invWLNMat[i] = 1.0/(1.0 - invWLNMat[i]);
		}

		if (invWLNMat[i] > LastInvWLN_Mat[i]) // 往相反於joint limit方向前進 那麼就不需要限制
		{
			invWLNMat[i] = 1.0;
			IfWLNEqualOne[i] = true;
		} // 反之 則需要限制，保留算出之權重值
		else
		{
			IfWLNEqualOne[i] = false;
		}

		LastInvWLN_Mat[i] = invWLNMat[i];
	}
	// 現在的 invWLNMat 是 論文中 W 矩陣的對角線元素，並且每個都是倒數 也就是 1/wi

}

void Kine::InvWithSingTest(double* JJT, double* tempJJT_inv, int MRowJJT, long* _ipiv, double* _work, double MinDet, double alph)
{
	/******************************************************************
	input: JJT 輸入J*J', tempJJT_inv 輸入暫存區位置,  MRowJJT 輸入JJT size, 
	       _ipiv _work 是CLAPACK的暫存區， MinDet是最小的允許deteriment值,  alph是singular時候要加上得對角線值
	output: void

	Note:
	    計算是否為singular configuration

	// JJT 是 被驗證是否 singular 的矩陣
	// tempJJT_inv 最後會存放 inverse結果
	// 假若小於 MinDet則被認為 singular
	// 被認為singular以後 對角線每格都會被加上 alph 當作 singularity avoidance
	// JJT 是方陣，MRowJJT 代表其 長與寬之值
	******************************************************************/


	doublereal* work = _work;
	integer* ipiv = _ipiv;
	integer INFO;
	integer Dim = MRowJJT;

	double detValue = 1;
	int write_index = 0;

	// LU decomposition

	for (int i = 0; i< MRowJJT*MRowJJT ; i++) // copy JJT to tempJJT
	{
		tempJJT_inv[i] = JJT[i];
	}

	dgetrf_(&Dim,&Dim,tempJJT_inv,&Dim,ipiv,&INFO);
	if (INFO != 0)
		cout << "Fail to Calculate LU decomposition" << endl;
	

	// detValue = U矩陣的對角線連乘
	for (int j = 0; j < Dim; j++)
		detValue *= tempJJT_inv[j*Dim+j];

	if (detValue < 0)
		detValue = -detValue;

	//printf("DET = %f \n",detValue);

	if (detValue < MinDet)
	{
		printf("Singular Pose");
		system("pause");

		for (int i = 0; i< MRowJJT*MRowJJT ; i++) //reload JJT to tempJJT
		{
			tempJJT_inv[i] = JJT[i];
		}

		for (int i = 0 ; i<Dim ; i++)
		{
			write_index += i*Dim+1;
			tempJJT_inv[write_index] += alph;
		}

		dgetrf_(&Dim,&Dim,tempJJT_inv,&Dim,ipiv,&INFO);

		if (INFO != 0)
			cout << "Fail to Calculate LU decomposition after singularity avoidance" << endl;

	}

	// matrix inversion result is stored in tempJJT
    dgetri_(&Dim,tempJJT_inv,&Dim,ipiv,work,&Dim,&INFO);

	if (INFO != 0)
	{
		cout << "Fail to Calculate Matrix Inversion" << endl;
		//system("pause");
	}
}

void Kine::ComputeEulerAng(double* RotM, double* EulerAng)
{
	/******************************************************************
	input: RotM 輸入旋轉矩陣 拿來求尤拉角, EulerAng 輸出之尤拉角
	       
	output: void

	Note:

	//從旋轉矩陣計算尤拉角
	// Matlab sample code
	// Euler angle -- Matlab code
	//function [thetaXYZ] = FindAngXYZ(R)

	//% please see reference
	//% http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToEuler/index.htm

	//% R = [xc yc zc] in the world  xc yc zc are all 3x1 column 

	//if (R(2,1) > 0.998) % singularity at north pole
	//	thetaXYZ(1,1) = 0;
	//	thetaXYZ(2,1) = atan2(R(1,3),R(3,3));
	//	thetaXYZ(3,1) = pi/2;
	//elseif (R(2,1) < -0.998) % singularity at south pole
	//	thetaXYZ(1,1) = 0; 
	//	thetaXYZ(2,1) = atan2(R(1,3),R(3,3));		
	//	thetaXYZ(3,1) = -pi/2; 
	//else
	//	thetaXYZ(1,1) = atan2(-R(2,3),R(2,2));
	//	thetaXYZ(2,1) = atan2(-R(3,1),R(1,1));
	//	thetaXYZ(3,1) = asin(R(2,1));
	//end     
	******************************************************************/
	if (RotM[3] > 0.9999) // singularity at north pole
	{
		EulerAng[0] = 0;
		EulerAng[1] = atan2(RotM[2],RotM[8]);
		EulerAng[2] = 1.5707963; // 3.1415926/2.0
	}
	else if (RotM[3] < -0.9999) // singularity at south pole
	{
		EulerAng[0] = 0;
		EulerAng[1] = atan2(RotM[2],RotM[8]);
		EulerAng[2] = -1.5707963; // 3.1415926/2.0
	}
	else
	{
		EulerAng[0] = atan2(-RotM[5],RotM[4]);
		EulerAng[1] = atan2(-RotM[6],RotM[0]);
		EulerAng[2] = asin(RotM[3]); 
	}
	
}

void Kine::GetLegsCoords(void)  // Also compute body coordinates
{
	/******************************************************************
	input: void
	output: void

	Note: 取出腳以及腰的坐標系旋轉矩陣
	******************************************************************/
	// x axis -> joint 9 - joint 10
	LLegRotM[0] = (CrdAll->data[27]-CrdAll->data[30])/LenEdgeXYZ[0];
	LLegRotM[3] = (CrdAll->data[28]-CrdAll->data[31])/LenEdgeXYZ[0];
	LLegRotM[6] = (CrdAll->data[29]-CrdAll->data[32])/LenEdgeXYZ[0];

	// y axis -> joint 11 - joint 10
	LLegRotM[1] = (CrdAll->data[33]-CrdAll->data[30])/LenEdgeXYZ[1];
	LLegRotM[4] = (CrdAll->data[34]-CrdAll->data[31])/LenEdgeXYZ[1];
	LLegRotM[7] = (CrdAll->data[35]-CrdAll->data[32])/LenEdgeXYZ[1];

	// z axis -> joint 6 - joint 7
	LLegRotM[2] = (CrdAll->data[18]-CrdAll->data[21])/LenEdgeXYZ[2];
	LLegRotM[5] = (CrdAll->data[19]-CrdAll->data[22])/LenEdgeXYZ[2];
	LLegRotM[8] = (CrdAll->data[20]-CrdAll->data[23])/LenEdgeXYZ[2];

	// x axis -> joint 21 - joint 24
	RLegRotM[0] = (CrdAll->data[63]-CrdAll->data[72])/LenEdgeXYZ[0];
	RLegRotM[3] = (CrdAll->data[64]-CrdAll->data[73])/LenEdgeXYZ[0];
	RLegRotM[6] = (CrdAll->data[65]-CrdAll->data[74])/LenEdgeXYZ[0];

	// y axis -> joint 23 - joint 24
	RLegRotM[1] = (CrdAll->data[69]-CrdAll->data[72])/LenEdgeXYZ[1];
	RLegRotM[4] = (CrdAll->data[70]-CrdAll->data[73])/LenEdgeXYZ[1];
	RLegRotM[7] = (CrdAll->data[71]-CrdAll->data[74])/LenEdgeXYZ[1];

	// z axis -> joint 19 - joint 20
	RLegRotM[2] = (CrdAll->data[57]-CrdAll->data[60])/LenEdgeXYZ[2];
	RLegRotM[5] = (CrdAll->data[58]-CrdAll->data[61])/LenEdgeXYZ[2];
	RLegRotM[8] = (CrdAll->data[59]-CrdAll->data[62])/LenEdgeXYZ[2];

	// y axis
	BodyRotM[1] = (CrdAll->data[0]-CrdAll->data[39])/160.0;
	BodyRotM[4] = (CrdAll->data[1]-CrdAll->data[40])/160.0;
	BodyRotM[7] = (CrdAll->data[2]-CrdAll->data[41])/160.0;

	// z axis
	BodyRotM[2] = (-CrdAll->data[81]+(CrdAll->data[90]+CrdAll->data[120])/2.0)/271.75;
	BodyRotM[5] = (-CrdAll->data[82]+(CrdAll->data[91]+CrdAll->data[121])/2.0)/271.75;
	BodyRotM[8] = (-CrdAll->data[83]+(CrdAll->data[92]+CrdAll->data[122])/2.0)/271.75;

	// x axis (x = y cross z)
	BodyRotM[0] = BodyRotM[4]*BodyRotM[8] - BodyRotM[7]*BodyRotM[5];
	BodyRotM[3] = BodyRotM[7]*BodyRotM[2] - BodyRotM[1]*BodyRotM[8];
	BodyRotM[6] = BodyRotM[1]*BodyRotM[5] - BodyRotM[4]*BodyRotM[2];

	//泓逸start111227
	//x-axis
	LArmRotM[0] = FKLArm->Rn[9].data[0];
	LArmRotM[3] = FKLArm->Rn[9].data[4];
	LArmRotM[6] = FKLArm->Rn[9].data[8];
	//y-axis
	LArmRotM[1] = FKLArm->Rn[9].data[1];
	LArmRotM[4] = FKLArm->Rn[9].data[5];
	LArmRotM[7] = FKLArm->Rn[9].data[9];
	//x-axis
	LArmRotM[2] = FKLArm->Rn[9].data[2];
	LArmRotM[5] = FKLArm->Rn[9].data[6];
	LArmRotM[8] = FKLArm->Rn[9].data[10];

	//for(int i = 0;i<16;i++)
	//{
	//	printf("%f   ",FK_LArm->Rn[9].data[i]);
	//}

	//x-axis
	RArmRotM[0] = FKRArm->Rn[9].data[0];
	RArmRotM[3] = FKRArm->Rn[9].data[4];
	RArmRotM[6] = FKRArm->Rn[9].data[8];
	//y-axis
	RArmRotM[1] = FKRArm->Rn[9].data[1];
	RArmRotM[4] = FKRArm->Rn[9].data[5];
	RArmRotM[7] = FKRArm->Rn[9].data[9];
	//z-axis
	RArmRotM[2] = FKRArm->Rn[9].data[2];
	RArmRotM[5] = FKRArm->Rn[9].data[6];
	RArmRotM[8] = FKRArm->Rn[9].data[10];

	//泓逸end111227
}
void Kine::GetShankCoords(void)
{
	double tempVec1[3];
	double tempVec2[3];
	double tempVec3[3];
	//MatMulABt(FKLLeg->RotMats[6].data,3,3,FKLLeg->RotMats[4].data,3,3,tempRotMat);
	//for(int i=0;i<9;i++)
	//	LShankRotM[i]=FKLLeg->RotMats[4].data[i];//tempRotMat[i];
	
	//y-axis
	LShankRotM[1] = -FKLLeg->RotMats[5].data[2];
	LShankRotM[4] = -FKLLeg->RotMats[5].data[5];
	LShankRotM[7] = -FKLLeg->RotMats[5].data[8];
	tempVec1[0] = -FKLLeg->RotMats[5].data[2];
	tempVec1[1] = -FKLLeg->RotMats[5].data[5];
	tempVec1[2] = -FKLLeg->RotMats[5].data[8];
	//z-axis
	LShankRotM[2] = FKLLeg->RotMats[5].data[6];
	LShankRotM[5] = FKLLeg->RotMats[5].data[3];
	LShankRotM[8] = FKLLeg->RotMats[5].data[0];
	tempVec2[0] = FKLLeg->RotMats[5].data[6];
	tempVec2[1] = FKLLeg->RotMats[5].data[3];
	tempVec2[2] = FKLLeg->RotMats[5].data[0];
	//x-axis	
	Cross2Vd(tempVec1,tempVec2,tempVec3);
	LShankRotM[0] = tempVec3[0];
	LShankRotM[3] = tempVec3[1];
	LShankRotM[6] = tempVec3[2];

	//y-axis
	RShankRotM[1] = FKRLeg->RotMats[5].data[2];
	RShankRotM[4] = FKRLeg->RotMats[5].data[5];
	RShankRotM[7] = FKRLeg->RotMats[5].data[8];
	tempVec1[0] = FKRLeg->RotMats[5].data[2];
	tempVec1[1] = FKRLeg->RotMats[5].data[5];
	tempVec1[2] = FKRLeg->RotMats[5].data[8];
	//z-axis
	RShankRotM[2] = FKRLeg->RotMats[5].data[6];
	RShankRotM[5] = FKRLeg->RotMats[5].data[3];
	RShankRotM[8] = FKRLeg->RotMats[5].data[0];
	tempVec2[0] = FKRLeg->RotMats[5].data[6];
	tempVec2[1] = FKRLeg->RotMats[5].data[3];
	tempVec2[2] = FKRLeg->RotMats[5].data[0];
	//x-axis	
	Cross2Vd(tempVec1,tempVec2,tempVec3);
	RShankRotM[0] = tempVec3[0];
	RShankRotM[3] = tempVec3[1];
	RShankRotM[6] = tempVec3[2];

}
void Kine::InitInertia(void)
{
	/******************************************************************
	input: void
	output: void

	Note: 初始化inertia matrices
	******************************************************************/	

	// catia and solidworks 量到的每個桿件在世界下的 inertia matrix
	double IR_FPad[9] = {6000, -550.1, 2000, -550.1, 9000, 98.94, 2000, 98.94, 7000}; 
	double IL_FPad[9] = {6000, 546, 2000, 546, 9000, -97.5, 2000, -97.5, 7000}; 
	double IR_KneeDown[9] = {17000, 106.1, -383, 106.1, 16000, 1000, -383, 1000, 3000};
	double IL_KneeDown[9] = {17000, -132.1, -450.3, -132.1, 16000, -1000, -450.3, -1000, 3000};
	double IR_KneeUp[9] = {33000, -200.2, -191.1, -200.2, 28000, -889.7, -191.1, -889.7, 8000};
	double IL_KneeUp[9] = {33000, 196.2, -201.5, 196.2, 28000, 877.5, -201.5, 877.5, 8000};

	// IB1 初始剛好就在世界下，所以不用旋轉
	IB1[0] = 76000;  IB1[1] = 75;	  IB1[2] = -10000; 
	IB1[3] = 75;	 IB1[4] = 55000;  IB1[5] = 71;
	IB1[6] = -10000; IB1[7] = 71;	  IB1[8] = 57000;

	double IBody[9] = {71000, 84, 94, 84, 87000, 3000, 94, 3000, 85000};  

	//double IR_Arm = {42000, -18.45, -134.8, -18.45, 42000, 54.74, -134.8, 54.74, 2000};
	double IR_Arm[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}; // 先disable 手臂，故意弄小
	//double IL_Arm = {42000, -19.39, -136.3, -19.39, 42000, -76.87, -136.3, -76.87, 2000};
	double IL_Arm[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}; // 先disable 手臂，故意弄小

	// 放大1000倍 單位變成 g.mm^2 (catia 裡面是 kg.m^2 乘上10^9 就會變成 g.mm^2)
	

                    
//clear IL_Arm IR_Arm IBody;
//clear A_RL0 A_LL0 A_RA0 A_LA0;         

//////////////////////IRL = [IR_Foot IR_Shank IR_Thigh]*1000;
//////////////////////ILL = [IL_Foot IL_Shank IL_Thigh]*1000;
//////////////////////IB =  [IB1 IB2].*1000;
//////////////////////ILA=ILA.*1000;
//////////////////////IRA=IRA.*1000;
//////////////////////clear IB1 IB2 IR_Foot IL_Foot IR_Shank IL_Shank IR_Thigh IL_Thigh
//////////////////////
//////////////////////warndlg('\n\n請小心!! momentum再遇到轉彎的時候(機器人不是面對前方的時候)，會有問題，座標也要轉!!\n\n')
//////////////////////



	double* tempThLL;
	tempThLL = new double[FKLLeg->NumJoint];

	double* tempThRL;
	tempThRL = new double[FKRLeg->NumJoint];

	double* tempThLA;
	tempThLA = new double[FKLArm->NumJoint];

	double* tempThRA;
	tempThRA = new double[FKRArm->NumJoint];

	for (int i = 0 ; i < FKLLeg->NumJoint ; i++)
	{
		tempThLL[i] = FKLLeg->theta[i];
		FKLLeg->theta[i] = 0;
	}

	for (int i = 0 ; i < FKRLeg->NumJoint ; i++)
	{
		tempThRL[i] = FKRLeg->theta[i];
		FKRLeg->theta[i] = 0;
	}

	for (int i = 0 ; i < FKRArm->NumJoint ; i++)
	{
		tempThRA[i] = FKRArm->theta[i];
		FKRArm->theta[i] = 0;
	}

	for (int i = 0 ; i < FKLArm->NumJoint ; i++)
	{
		tempThLA[i] = FKLArm->theta[i];
		FKLArm->theta[i] = 0;
	}

	double temp_compute[9];
	double Rot_Part_Rn[9];
	double Rot_Part_Rn2[9];
	double temp_Rn[9];

	// 先將theta設為0，取完DH之後 復原theta值，照常執行程式
	FKLLeg->DHConstruct();
	FKRLeg->DHConstruct();
	FKLArm->DHConstruct();
	FKRArm->DHConstruct();

	////// IL_Foot = A_LL0(1:3,25:27)'*IL_FPad*A_LL0(1:3,25:27); // 第6軸 Rn5
	GetRotPartRn(&FKLLeg->Rn[5],Rot_Part_Rn);
	MatMulAB(IL_FPad,3,3,Rot_Part_Rn,3,3,temp_compute);
	MatMulAtB(Rot_Part_Rn,3,3,temp_compute,3,3,IL_Foot);

	////// IL_Shank = A_LL0(1:3,17:19)'*IL_KneeDown*A_LL0(1:3,17:19); // 第4軸 Rn3
	GetRotPartRn(&FKLLeg->Rn[3],Rot_Part_Rn);
	MatMulAB(IL_KneeDown,3,3,Rot_Part_Rn,3,3,temp_compute);
	MatMulAtB(Rot_Part_Rn,3,3,temp_compute,3,3,IL_Shank);

	//////IL_Thigh =  A_LL0(1:3,13:15)'*IL_KneeUp*A_LL0(1:3,13:15); // 第3軸 Rn2
	GetRotPartRn(&FKLLeg->Rn[2],Rot_Part_Rn);
	MatMulAB(IL_KneeUp,3,3,Rot_Part_Rn,3,3,temp_compute);
	MatMulAtB(Rot_Part_Rn,3,3,temp_compute,3,3,IL_Thigh);

	////// IR_Foot = A_RL0(1:3,25:27)'*IR_FPad*A_RL0(1:3,25:27); // 第6軸 Rn5
	GetRotPartRn(&FKRLeg->Rn[5],Rot_Part_Rn);
	MatMulAB(IR_FPad,3,3,Rot_Part_Rn,3,3,temp_compute);
	MatMulAtB(Rot_Part_Rn,3,3,temp_compute,3,3,IR_Foot);

	////// IR_Shank = A_RL0(1:3,17:19)'*IR_KneeDown*A_RL0(1:3,17:19); // 第4軸 Rn3
	GetRotPartRn(&FKRLeg->Rn[3],Rot_Part_Rn);
	MatMulAB(IR_KneeDown,3,3,Rot_Part_Rn,3,3,temp_compute);
	MatMulAtB(Rot_Part_Rn,3,3,temp_compute,3,3,IR_Shank);

	////// IR_Thigh = A_RL0(1:3,13:15)'*IR_KneeUp*A_RL0(1:3,13:15); // 第3軸 Rn2
	GetRotPartRn(&FKRLeg->Rn[2],Rot_Part_Rn);
	MatMulAB(IR_KneeUp,3,3,Rot_Part_Rn,3,3,temp_compute);
	MatMulAtB(Rot_Part_Rn,3,3,temp_compute,3,3,IR_Thigh);

	// 手臂DH順序: 腰yaw->腰pitch->手臂pitch->手臂end-effector
	////// IB2 = A_RA0(1:3,5:7)'*IBody*A_RA0(1:3,5:7);   // 第2軸Rn1  % 腰的座標系應該取腰旋轉以後會受到影響的所以取5~7
	GetRotPartRn(&FKRArm->Rn[1],Rot_Part_Rn);
	MatMulAB(IBody,3,3,Rot_Part_Rn,3,3,temp_compute);
	MatMulAtB(Rot_Part_Rn,3,3,temp_compute,3,3,IB2);

	//////% 整隻右手臂  
	//////IRA = A_RA0(1:3,9:11)'*IR_Arm*A_RA0(1:3,9:11);  // 第3軸 Rn2
	GetRotPartRn(&FKRArm->Rn[2],Rot_Part_Rn);
	MatMulAB(IR_Arm,3,3,Rot_Part_Rn,3,3,temp_compute);
	MatMulAtB(Rot_Part_Rn,3,3,temp_compute,3,3,IRA);
	
	//////% 整隻左手臂 
	//////ILA = A_LA0(1:3,9:11)'*IL_Arm*A_LA0(1:3,9:11);  // 第3軸 Rn2
	GetRotPartRn(&FKLArm->Rn[2],Rot_Part_Rn);
	MatMulAB(IL_Arm,3,3,Rot_Part_Rn,3,3,temp_compute);
	MatMulAtB(Rot_Part_Rn,3,3,temp_compute,3,3,ILA);

	for (int i = 0 ; i < 9 ; i++)
	{
		IL_Foot[i] *= 1000;
		IL_Shank[i] *= 1000;
		IL_Thigh[i] *= 1000;
		IR_Foot[i] *= 1000;
		IR_Shank[i] *= 1000;
		IR_Thigh[i] *= 1000;
		IB1[i] *= 1000;
		IB2[i] *= 1000;
		IRA[i] *= 1000;
		ILA[i] *= 1000;
	}

	// 依照support腳不一樣 inertia的原始點也不同
	for (int i = 0 ; i < 9 ; i++)
	{
		IL_Foot_sup_R[i] = IL_Foot[i];
		IL_Shank_sup_R[i] = IL_Shank[i];
		IL_Thigh_sup_R[i] = IL_Thigh[i];
		IR_Foot_sup_L[i] = IR_Foot[i];
		IR_Shank_sup_L[i] = IR_Shank[i];
		IR_Thigh_sup_L[i] = IR_Thigh[i];

		// support foot 不用被微分，因為不會轉
		IR_Foot_sup_R[i] = IR_Foot[i];
		IL_Foot_sup_L[i] = IL_Foot[i];
	}

	// 依照support腳不一樣 inertia的原始點也不同
	////// IL_Shank = A_LL0(1:3,17:19)'*IL_KneeDown*A_LL0(1:3,17:19); // 第4軸 Rn3
	GetRotPartRn(&FKLLeg->Rn[3],Rot_Part_Rn);
	GetRotPartRn(&FKLLeg->Rn[7],Rot_Part_Rn2);
	MatMulAtB(Rot_Part_Rn2,3,3,Rot_Part_Rn,3,3,temp_Rn); // 留下 A7'*A6'*A5'*A4' (膝蓋以下)
	MatMulAB(IL_KneeDown,3,3,temp_Rn,3,3,temp_compute);
	MatMulAtB(temp_Rn,3,3,temp_compute,3,3,IL_Shank_sup_L);

	//////IL_Thigh =  A_LL0(1:3,13:15)'*IL_KneeUp*A_LL0(1:3,13:15); // 第3軸 Rn2
	GetRotPartRn(&FKLLeg->Rn[2],Rot_Part_Rn);
	MatMulAtB(Rot_Part_Rn2,3,3,Rot_Part_Rn,3,3,temp_Rn); // 留下 A7'*A6'*A5'*A4'*A3' (Hip以下)
	MatMulAB(IL_KneeUp,3,3,temp_Rn,3,3,temp_compute);
	MatMulAtB(temp_Rn,3,3,temp_compute,3,3,IL_Thigh_sup_L);

	////// IR_Shank = A_RL0(1:3,17:19)'*IR_KneeDown*A_RL0(1:3,17:19); // 第4軸 Rn3
	GetRotPartRn(&FKRLeg->Rn[3],Rot_Part_Rn);
	GetRotPartRn(&FKRLeg->Rn[7],Rot_Part_Rn2);
	MatMulAtB(Rot_Part_Rn2,3,3,Rot_Part_Rn,3,3,temp_Rn); // 留下 A7'*A6'*A5'*A4' (膝蓋以下)
	MatMulAB(IR_KneeDown,3,3,temp_Rn,3,3,temp_compute);
	MatMulAtB(temp_Rn,3,3,temp_compute,3,3,IR_Shank_sup_R);

	//////IR_Thigh =  A_RL0(1:3,13:15)'*IR_KneeUp*A_RL0(1:3,13:15); // 第3軸 Rn2
	GetRotPartRn(&FKRLeg->Rn[2],Rot_Part_Rn);
	MatMulAtB(Rot_Part_Rn2,3,3,Rot_Part_Rn,3,3,temp_Rn); // 留下 A7'*A6'*A5'*A4'*A3' (Hip以下)
	MatMulAB(IR_KneeUp,3,3,temp_Rn,3,3,temp_compute);
	MatMulAtB(temp_Rn,3,3,temp_compute,3,3,IR_Thigh_sup_R);


	for (int i = 0 ; i < 9 ; i++)
	{
		IL_Shank_sup_L[i] *= 1000;
		IL_Thigh_sup_L[i] *= 1000;
		IR_Shank_sup_R[i] *= 1000;
		IR_Thigh_sup_R[i] *= 1000;
	}

	for (int i = 0 ; i < FKLLeg->NumJoint ; i++)
	{
		FKLLeg->theta[i] = tempThLL[i];
	}
	delete[] tempThLL;

	for (int i = 0 ; i < FKRLeg->NumJoint ; i++)
	{
		FKRLeg->theta[i] = tempThRL[i];
	}
	delete[] tempThRL;

	for (int i = 0 ; i < FKLArm->NumJoint ; i++)
	{
		FKLArm->theta[i] = tempThLA[i];
	}
	delete[] tempThLA;

	for (int i = 0 ; i < FKRArm->NumJoint ; i++)
	{
		FKRArm->theta[i] = tempThRA[i];
	}
	delete[] tempThRA;

}

void Kine::GetRotPartRn(YMatLite* HomoMat, double* RotPart)
{
	/******************************************************************
	input: HomoMat輸入之homogeneous matrix, RotPart 輸出之旋轉矩陣
	output: void

	Note: 從homogeneous matrix 取出旋轉矩陣
	******************************************************************/
	// 取出 4*4 矩陣的 旋轉部份
	RotPart[0] = HomoMat->data[0];
	RotPart[1] = HomoMat->data[1];
	RotPart[2] = HomoMat->data[2];
	RotPart[3] = HomoMat->data[4];
	RotPart[4] = HomoMat->data[5];
	RotPart[5] = HomoMat->data[6];
	RotPart[6] = HomoMat->data[8];
	RotPart[7] = HomoMat->data[9];
	RotPart[8] = HomoMat->data[10];
}

void Kine::GenSmoothZMPShift(double y_start, double y_end, int Np, double* result)
{
	/******************************************************************
	input: y_start 初始位置, y_end 結束位置, Np 總內差點數, result 結果存在 result
	output: void

	Note: 內插平滑的ZMP曲線 Jerk不可指定 速度 加速度為0
	******************************************************************/

	// 參考開始位置當做0
	// x0 = y0 = 0;

	double y1 = y_end-y_start;
	double a[6]; // 5次多項式 共有六項 輸入0~1給此多項式可以得到開始與結尾速度與加速度都為0的曲線 (開始在y_start 結束在 y_end)
	a[0] = 0; a[1] = 0; a[2] = 0; a[3] = 10*y1; a[4] = -15*y1; a[5] = 6*y1;

    double step_int = 1.0/double(Np-1);


	double* x;
	x = new double[Np];

	x[0] = 0;
	for (int i = 1 ; i < Np-1; i++)
	{
		x[i] = x[i-1] + step_int;
	}
	x[Np-1] = 1.0;


	double x_acc[6];

	result[0] = y_start;
	for (int i = 1 ; i < Np-1; i++)
	{
		x_acc[0] = 1;
		for (int j = 1 ; j < 6 ; j++)
		{
			x_acc[j] = x_acc[j-1]*x[i];		
		}

		result[i] = y_start + a[0]*x_acc[0]+a[1]*x_acc[1]+a[2]*x_acc[2]+a[3]*x_acc[3]+a[4]*x_acc[4]+a[5]*x_acc[5];
	}
	result[Np-1] = y_end;	

	delete[] x;

}


void Kine::GenSmoothZMPShift_ZeroJerk(double y_start, double y_end, int Np, double* result)
{
	/******************************************************************
	input: y_start 初始位置, y_end 結束位置, Np 總內差點數, result 結果存在 result
	output: void

	Note: 內插平滑的ZMP曲線 Jerk 加速度 速度為0
	******************************************************************/

	// 參考開始位置當做0
	// x0 = y0 = 0;

	double y1 = y_end-y_start;
	double a[8]; // 7次多項式 共有8項 輸入0~1給此多項式可以得到開始與結尾速度與加速度都為0的曲線 Jerk也為0 (開始在y_start 結束在 y_end)
	a[0] = 0; a[1] = 0; a[2] = 0; a[3] = 0; a[4] = 35*y1; a[5] = -84*y1; a[6] = 70*y1; a[7] = -20*y1;

    double step_int = 1.0/double(Np-1);


	double* x;
	x = new double[Np];

	x[0] = 0;
	for (int i = 1 ; i < Np-1; i++)
	{
		x[i] = x[i-1] + step_int;
	}
	x[Np-1] = 1.0;


	double x_acc[8];

	result[0] = y_start;
	for (int i = 1 ; i < Np-1; i++)
	{
		x_acc[0] = 1;
		for (int j = 1 ; j < 8 ; j++)
		{
			x_acc[j] = x_acc[j-1]*x[i];		
		}

		result[i] = y_start + a[0]*x_acc[0]+a[1]*x_acc[1]+a[2]*x_acc[2]+a[3]*x_acc[3]+a[4]*x_acc[4]+a[5]*x_acc[5]+a[6]*x_acc[6]+a[7]*x_acc[7];
	}
	result[Np-1] = y_end;	

	delete[] x;

}



void Kine::GenSmoothZMPShift_ZeroJerkshiftstair(double y_start, double y_end, int Np, double* result,int mode1)
{
	/******************************************************************
	input: y_start 初始位置, y_end 結束位置, Np 總內差點數, result 結果存在 result
	output: void

	Note: 內插平滑的ZMP曲線 Jerk 加速度 速度為0
	******************************************************************/
	// 參考開始位置當做0
	// x0 = y0 = 0;
	//原本是內插1200個點  改為內插300個點
	// mode1 == 0 (上樓梯)   mode ==  1 (下樓梯)   

	double y1 = y_end-y_start;
	double a[8]; // 7次多項式 共有8項 輸入0~1給此多項式可以得到開始與結尾速度與加速度都為0的曲線 Jerk也為0 (開始在y_start 結束在 y_end)
	a[0] = 0; a[1] = 0; a[2] = 0; a[3] = 0; a[4] = 35*y1; a[5] = -84*y1; a[6] = 70*y1; a[7] = -20*y1;

    double step_int = 1.0/300;

	double* x;
	x = new double[Np];
	
	if (mode1 == 0) {
		x[0] = 0;
		for (int i = 1 ; i < 301; i++){
			x[i] = x[i-1] + step_int;
		}
		x[300] = 1.0;
		double x_acc[8];

		for (int i = 1 ; i < 301; i++){
			x_acc[0] = 1;
			for (int j = 1 ; j < 8 ; j++){
				x_acc[j] = x_acc[j-1]*x[i];		
			}
			result[i+ 839] = y_start + a[0]*x_acc[0]+a[1]*x_acc[1]+a[2]*x_acc[2]+a[3]*x_acc[3]+a[4]*x_acc[4]+a[5]*x_acc[5]+a[6]*x_acc[6]+a[7]*x_acc[7];
		}
	
		result[0] = y_start;
		for (int i = 1 ; i < 840; i++){
		result[i] = y_start;	
		}
		for (int i = 1140 ; i < Np; i++){
		result[i] = y_end;	
		}

		delete[] x;
	}

	if (mode1 == 1){
		x[0] = 0;
		for (int i = 1 ; i < 301; i++){
			x[i] = x[i-1] + step_int;
		}
		x[300] = 1.0;

		double x_acc[8];
		for (int i = 1 ; i < 301; i++){
			x_acc[0] = 1;
			for (int j = 1 ; j < 8 ; j++){
				x_acc[j] = x_acc[j-1]*x[i];		
			}
		
			result[i+ 639] = y_start + a[0]*x_acc[0]+a[1]*x_acc[1]+a[2]*x_acc[2]+a[3]*x_acc[3]+a[4]*x_acc[4]+a[5]*x_acc[5]+a[6]*x_acc[6]+a[7]*x_acc[7];
		}
	
		result[0] = y_start;
		for (int i = 1 ; i < 640; i++){
		result[i] = y_start;
		}
		for (int i = 940 ; i < Np; i++){
		result[i] = y_end;	
		}

		delete[] x;
	}
}


void Kine::GenZMPFreeAssign(double x2, double y0, double v0, double a0, double j0, double y2, double v2, double a2, double j2, int Np, double* result)
{
	/******************************************************************
	input: x2, y0, v0, a0, j0, y2, v2, a2,j2, 為第一點跟結束點位置 速度 加速度 jerk 都可以自由輸入
	       Np 總內插點數，結果存在result
	output: void

	Note: 內插平滑的ZMP曲線 位置速度加速度jerk全部都可以自由指定
	// A = [1 1 1 1 ; 4 5 6 7 ; 12 20 30 42 ; 24 60 120 210]; // 多項式排列係數成矩陣
	// 七階多項式 自由輸入 位置 速度 加速度 jerk 共八個變數
	// x 軸 x1 = 0  x2 = input 請依照真實秒數輸入 請注意x2單位是時間	
	******************************************************************/

	double xpoly[8];
	xpoly[0] = 1;
	xpoly[1] = x2;
	for (int i = 2;i<8;i++)
		xpoly[i] = xpoly[i-1]*x2;

	// 先填入沒有inverse的值 等等就要做inverse 
	double invA[16] = {xpoly[4],xpoly[5],xpoly[6],xpoly[7],4*xpoly[3],5*xpoly[4],6*xpoly[5],7*xpoly[6],12*xpoly[2], 20*xpoly[3], 30*xpoly[4], 42*xpoly[5], 24*xpoly[1], 60*xpoly[2], 120*xpoly[3], 210*xpoly[4]};
	InvSqMat(invA,4);

	double p[8]; // 7次多項式 共有8項  (開始在y1 結束在 y2)
	p[0] = y0; p[1] = v0; p[2] = 0.5*a0; p[3] = j0/6.0;

	// 除了 a4~a7 以外的部分 要用 invA*r 求出 a4~a7
	double r[4] = {y2-y0-v0*xpoly[1]-0.5*a0*xpoly[2]-j0*xpoly[3]/6.0, v2-v0-a0*xpoly[1]-0.5*j0*xpoly[2], a2-a0-j0*xpoly[1], j2-j0};
	MatMulAB(invA,4,4,r,4,1,p+4);

	// 線性內插 x軸 0~1
    double step_int = x2/double(Np-1);

	double* x;
	x = new double[Np];

	x[0] = 0;
	for (int i = 1 ; i < Np-1; i++)
	{
		x[i] = x[i-1] + step_int;
	}
	x[Np-1] = 1.0;


	double x_acc[8]; // 記住各個x 在 1~7次方

	// 算出七次多項式結果
	result[0] = y0;
	for (int i = 1 ; i < Np-1; i++)
	{
		x_acc[0] = 1;
		x_acc[1] = x[i];
		for (int j = 2 ; j < 8 ; j++)
		{
			x_acc[j] = x_acc[j-1]*x[i];		
		}
		result[i] = p[0]+p[1]*x_acc[1]+p[2]*x_acc[2]+p[3]*x_acc[3]+p[4]*x_acc[4]+p[5]*x_acc[5]+p[6]*x_acc[6]+p[7]*x_acc[7];
	}
	result[Np-1] = y2;	

	// 清除動態記憶體
	delete[] x;

}



void Kine::FindPseudoJ(void)
{
	/******************************************************************
	input: void
	output: void

	Note: 計算Pseudo Jacobian matrix
	      直接重複使用FK的函式來取得J+
	******************************************************************/

	int MatIndex;
	ComputeJacobians();
	FindWLN();

	// 包含WLN 的IK
	MatIndex = 0;
	// compute tempJT = J*inv(W)'
	for (int i =0 ; i< Ja->MRow ; i++) 
	{
		for (int j = 0 ; j< Ja->NCol ; j++)
		{
			tempJT[MatIndex]=Ja->data[MatIndex]*invWLNMat[j];
			MatIndex += 1;
		}
	}

	for (int i = 0 ; i < Ja->MSize ; i++) // copy J
	{
		tempJ[i] = Ja->data[i];
	}

	// compute J*inv(W)*J'
	MatMulABt(tempJ,Ja->MRow,Ja->NCol,tempJT,Ja->MRow,Ja->NCol,tempJiWJT);

	// compute inv(J*inv(W)*J')
	//InvSqMat(tempJiWJT,Ja->MRow);

	//SingularJudge = 0.0000000000001; // 10^-13
	//SingularAdd = 0.1;
	InvWithSingTest(tempJiWJT, tempJJT_for_inv, Ja->MRow, ipiv_clapack, work_clapack, SingularJudge, SingularAdd);

	// compute inv(J*inv(W)*J')*dx
	//MatMulAB(tempJiWJT,Ja->MRow,Ja->MRow,dx,Ja->MRow,1,tempInv);
	MatMulAB(tempJJT_for_inv,Ja->MRow,Ja->MRow,dx,Ja->MRow,1,tempInv);

	// compute dth = inv(W)*J'*inv(J*inv(W)*J')*dx
	MatMulAtB(tempJT,Ja->MRow,Ja->NCol,tempInv,Ja->MRow,1,dth);

	// 包含WLN 的IK

	// PseudoInv
	MatMulAtB(tempJT,Ja->MRow,Ja->NCol,tempJJT_for_inv,Ja->MRow,Ja->MRow,PseudoInv);

}

void Kine::SetIdentity(double* RotationMatrix)
{
	/******************************************************************
	input: 輸入 RotationMatrix，並且將其設為identity matrix
	output: void

	Note: 輸入 RotationMatrix，並且將其設為identity matrix
	******************************************************************/
	RotationMatrix[0] = 1;
	RotationMatrix[1] = 0;
	RotationMatrix[2] = 0;
	RotationMatrix[3] = 0;
	RotationMatrix[4] = 1;
	RotationMatrix[5] = 0;
	RotationMatrix[6] = 0;
	RotationMatrix[7] = 0;
	RotationMatrix[8] = 1;
}

void Kine::CheckJointLimit(void)
{
	/******************************************************************
	input: void
	output: void

	Note:自動確認當下動作是否超過機構極限
	******************************************************************/

	for (int i = 0 ; i < 6 ; i++)
	{
		if (FKLLeg->theta[i+1] > JointUpLimitLL[i])
		{
			printf("\n警告!!! 左腳第%d軸角度%f度超過上限%f度\n",i+1,FKLLeg->theta[i+1]/3.1415926*180,JointUpLimitLL[i]/3.1415926*180);
			system("pause");
		}
		if (FKRLeg->theta[i+1] > JointUpLimitRL[i])
		{
			printf("\n警告!!! 右腳第%d軸角度%f度超過上限%f度\n",i+1,FKRLeg->theta[i+1]/3.1415926*180,JointUpLimitRL[i]/3.1415926*180);
			system("pause");
		}
		if (FKLLeg->theta[i+1] < JointLoLimitLL[i])
		{
			printf("\n警告!!! 左腳第%d軸角度%f度小於下限%f度\n",i+1,FKLLeg->theta[i+1]/3.1415926*180,JointLoLimitLL[i]/3.1415926*180);
			system("pause");
		}
		if (FKRLeg->theta[i+1] < JointLoLimitRL[i])
		{
			printf("\n警告!!! 右腳第%d軸角度%f度小於下限%f度\n",i+1,FKRLeg->theta[i+1]/3.1415926*180,JointLoLimitRL[i]/3.1415926*180);
			system("pause");
		}
	}
}

void Kine::UpdateDrawingBuffer(void) // 更新繪圖，只有在FK結束時執行
{
	/******************************************************************
	input: void
	output: void

	Note:FK結束後 填入新的座標值
	******************************************************************/
	IndexBuf = 0;
	IndexBufRev = (LegDHLen-1)*3;

	for (int j = LegDHLen-1 ; j>=0 ; j--)
	{
		LegDrawingBuffer[IndexBuf+2] = CrdAll->data[IndexBufRev];
		LegDrawingBuffer[IndexBuf] = CrdAll->data[IndexBufRev+1];
		LegDrawingBuffer[IndexBuf+1] = CrdAll->data[IndexBufRev+2];
		IndexBuf += 3;
		IndexBufRev -= 3;
	}

	LegDrawingBuffer[IndexBuf+2] = DHOrigin[0];
	LegDrawingBuffer[IndexBuf] = DHOrigin[1];
	LegDrawingBuffer[IndexBuf+1] = DHOrigin[2];
	IndexBuf += 3;
	IndexBufRev = IndexBuf-3;
	for (int j = 0 ; j<LegDHLen ; j++)
	{
		LegDrawingBuffer[IndexBuf+2] = CrdAll->data[IndexBufRev];
		LegDrawingBuffer[IndexBuf] = CrdAll->data[IndexBufRev+1];
		LegDrawingBuffer[IndexBuf+1] = CrdAll->data[IndexBufRev+2];
		IndexBuf += 3;
		IndexBufRev += 3;
	}

	// 手臂
	IndexBufRev = (ArmDHLen-1+2*LegDHLen)*3;
	IndexBuf = 0;

	for (int j = ArmDHLen-1 ; j>=0 ; j--)
	{
		ArmDrawingBuffer[IndexBuf+2] = CrdAll->data[IndexBufRev];
		ArmDrawingBuffer[IndexBuf] = CrdAll->data[IndexBufRev+1];
		ArmDrawingBuffer[IndexBuf+1] = CrdAll->data[IndexBufRev+2];
		IndexBuf += 3;
		IndexBufRev -= 3;
	}

	ArmDrawingBuffer[IndexBuf+2] = DHOrigin[0];
	ArmDrawingBuffer[IndexBuf] = DHOrigin[1];
	ArmDrawingBuffer[IndexBuf+1] = DHOrigin[2];
	IndexBuf += 3;

	IndexBufRev = (ArmDHLen+2*LegDHLen)*3;
	for (int j = 0 ; j<ArmDHLen ; j++)
	{
		ArmDrawingBuffer[IndexBuf+2] = CrdAll->data[IndexBufRev];
		ArmDrawingBuffer[IndexBuf] = CrdAll->data[IndexBufRev+1];
		ArmDrawingBuffer[IndexBuf+1] = CrdAll->data[IndexBufRev+2];
		IndexBuf += 3;
		IndexBufRev += 3;
	}
				
}

void Kine::FindD(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 運算所有Dynamics函式 目的為得到各軸Torque
	// 2012 Slongz Start
	// 20121220 WeiZh Lai Start
	******************************************************************/
		//CalCoord();
		//NumDiff();
		//FDOmegaL();
		//FDVelJ();
		//FDAlphaL();
		//FDAccelJ();
		//FDVelCOM();
		//FDAccelCOM();
		//FDForceJ();
		//FDTorqueJ();
}

void Kine::FindDIni(void)
{   
	/******************************************************************
	input: void
	output: void

	Note:
	// 初始化所有Dynamics變數 包括力規值歸零
	// 目前放在Start鍵啟動時呼叫本函式
	// FindDIni請務必放在FindCOG之後 因為在計算上半身轉動慣量時會用到
	// 2012 Slongz Start
	// 20121210 WeiZh Lai Start
	******************************************************************/
	AxisJump = 3*6;
	DHJump = 3*LegDHLen;
	StackJump = 3*3;

	//for(int i = 0 ; i < 3 ; i++){	//力規值歸零
	//FSensor_forcL[i] = 0 ;
	//FSensor_forcR[i] = 0 ;
	//FSensor_TorqL[i] = 0 ;
	//FSensor_TorqR[i] = 0 ;
	//}
	
	// 在此訂定世界下的重力向量 g*[0 0 -1]
		GravityZaxi[0]=0;
		GravityZaxi[1]=0;
		GravityZaxi[2]=-1*GravityConst;

	// 計算上半身重量
	LArmW = 0;	RArmW = 0;	BodyW = 0;
	for (int i = 7 ; i < 10 ; i++)
		LArmW += mass_com[i];
	RArmW = LArmW ;

	for (int i = 6 ; i < 16 ; i++){
		BodyW += mass_com[i];
	}
	
	for(int i =0 ; i < 3 ; i++){
	LArmCOM[i] = 0;	
	RArmCOM[i] = 0;	
	BodyRCOM[i] = 0;
	}
	
	// 計算左手臂重心位置(扣除肩膀)
	int Index = 18;
	for (int i = 7 ; i < 10 ; i++){
	LArmCOM[0] += pv_stack[Index]*mass_com[i];
	LArmCOM[1] += pv_stack[Index+1]*mass_com[i];
	LArmCOM[2] += pv_stack[Index+2]*mass_com[i];
	Index += 3;
	}

	// 計算右手臂重心位置(扣除肩膀)
	Index = 30;
	for (int i = 11 ; i < 14 ; i++){
	RArmCOM[0] += pv_stack[Index]*mass_com[i];
	RArmCOM[1] += pv_stack[Index+1]*mass_com[i];
	RArmCOM[2] += pv_stack[Index+2]*mass_com[i];
	Index += 3;
	}

	// 計算上半身重心位置 在這裡把CalCoord中的式子拿來這裡計算 因為Initial只做一次
	Index = 18;
	for (int i = 6 ; i < 16 ; i++){
	BodyRCOM[0] += pv_stack[Index]*mass_com[i];
	BodyRCOM[1] += pv_stack[Index+1]*mass_com[i];
	BodyRCOM[2] += pv_stack[Index+2]*mass_com[i];
	Index += 3;
	}

	for(int i = 0 ; i < 3 ; i++){
	LArmCOM[i] = LArmCOM[i]/LArmW;
	RArmCOM[i] = RArmCOM[i]/RArmW;
	BodyRCOM[i] = BodyRCOM[i]/BodyW;
	}

	// 以各桿件質心座標描述該桿件轉動慣量 為一定值 故在此計算
	FindInertia2LocalCOM();
	
	//for(int i = 0 ; i < 12 ; i++)
	//	MotorTorq[i] = 0;

	CountMotor = 0;
			
	for(int i = 0 ; i < 36 ; i++){
	ForceJ[i];	
	TorqueJ[i];	
	}

	//// initialize Kalman Filter
	//	// 校正KF initial值時打開
	//	for (int i = 0 ; i < 12 ; i ++)
	//	{
	//		x_est_last[i] = 0;
	//		x_est_lastMotor[i] = 0;
	//	//}

	//	//第一筆值直接從Adams裡頭站直時的力規值匯入
	//	//x_est_last[0] = -0.01037;
	//	//x_est_last[1] = -0.0002592;
	//	//x_est_last[2] = -278.5;
	//	//x_est_last[3] = -1532;
	//	//x_est_last[4] = 2151;
	//	//x_est_last[5] = 0.04620;
	//	//x_est_last[6] = 0;
	//	//x_est_last[7] = 0;
	//	//x_est_last[8] = 282;
	//	//x_est_last[9] = -2580;
	//	//x_est_last[10] = -1000;
	//	//x_est_last[11] = 0;
	//	
	//	//// 第一筆Motor值直接從Adams裡頭站直時的Motor值匯入
	//	//	x_est_lastMotor[0] = 0;
	//	//	x_est_lastMotor[1] = -1560;
	//	//	x_est_lastMotor[2] = 3320;
	//	//	x_est_lastMotor[3] = 3320;
	//	//	x_est_lastMotor[4] = 3320;
	//	//	x_est_lastMotor[5] = -1409;
	//	//	x_est_lastMotor[6] = 0;
	//	//	x_est_lastMotor[7] = 0;
	//	//	x_est_lastMotor[8] = 975;
	//	//	x_est_lastMotor[9] = 975;
	//	//	x_est_lastMotor[10] = 975;
	//	//	x_est_lastMotor[11] = 0;
	//	//
	//	//	for(int i = 0 ; i < 12 ; i++ )
	//	//		x_est_lastMotor[i] *= 1000000;
	//			//	for(int i = 0 ; i < 12 ; i++ )
	//	//		x_est_lastMotor[i] *= 1000000;

	//	//for(int i = 0 ; i < 12 ; i++)
	//	//{
	//	P_last[i] = 0;
	//	Q_KF[i] = 0.05;	// Q為model的coverence 越小越相信
	//	R_KF[i] = 3;	// R為measuremant的coverence 越小越相信

	//	P_lastMotor[i] = 0;
	//	QMotor[i] = 0.1;	// Q為model的coverence 越小越相信
	//	RMotor[i] = 1;	// R為measuremant的coverence 越小越相信
	//	}


	//// 利用控制KF中的 Q跟R 來達到一開始脫離Singular的狀態
	//	for (int i = 0 ; i < 12 ; i++)
	//	{
	//		P_last[i] = 0;
	//	//the noise in the system
	//		Q[i] = 0.1;	// Q為model的coveriance 越小越相信
	//		R[i] = 0.5;	// R為measuremant的coveriance 越小越相信

	//		P_lastMotor[i] = 0;
	//	//the noise in the system
	//		QMotor[i] = 0.1;	// Q為model的coveriance 越小越相信
	//		RMotor[i] = 0.07;	// R為measuremant的coveriance 越小越相信		
	//	}	
	//
	// 平移力規位置 原本力規r是在腳底板 往上移80
	// 但質量沒變 移了是否正確? 是否計算比較精確?
		SensorOffset[0] = 0;
		SensorOffset[1] = 0;
		SensorOffset[2] = -60;
	////////////////////////SlongZ 用以在DSP時不用力規而用ZMP分配力量/////////////////////////
	//int tempsize=T_P/dt;
	//int tempsize2=tempsize*DSP*0.5;
	//int tempsize3=tempsize-tempsize*DSP*0.5;

	//if(selIK==2)
	//	DSPFlag=1;
	//else
	//{
	//	if(( gIthIK%tempsize)<tempsize2 || ( gIthIK%tempsize)>tempsize3)
	//	{
	//		DSPFlag=1;
	//	}
	//	else
	//		DSPFlag=0;
	//}
	////cout<<"DSPFlag"<<DSPFlag<<"\n";

	//if(GLindex==1&&selIK==2)
	//{
	//	for(int i=0;i<3;i++)
	//	{
	//	//LP[3*i]=CrdAll->data[3*i];
	//	//LP[3*i+1]=CrdAll->data[3*i+1];
	//	//LP[3*i+2]=CrdAll->data[3*i+2];

	//	//RP[3*i]=CrdAll->data[3*i+39];
	//	//RP[3*i+1]=CrdAll->data[3*i+40];
	//	//RP[3*i+2]=CrdAll->data[3*i+41];

	//	LP[3*i]=		pv_stack[3*i+0];
	//	LP[3*i+1]=		pv_stack[3*i+1];
	//	LP[3*i+2]=		pv_stack[3*i+2];

	//	RP[3*i]=		pv_stack[3*i+0+9];
	//	RP[3*i+1]=		pv_stack[3*i+1+9];
	//	RP[3*i+2]=		pv_stack[3*i+2+9];
	//	}	

	//	LP[9]=		BodyRCOM[0];
	//	LP[9+1]=	BodyRCOM[1];
	//	LP[9+2]=	BodyRCOM[2];

	//	RP[9]=		BodyRCOM[0];
	//	RP[9+1]=	BodyRCOM[1];
	//	RP[9+2]=	BodyRCOM[2];

	//	CP[0]=COG[0];
	//	CP[1]=COG[1];
	//	CP[2]=COG[2];

	//	for (int i = 0 ; i < 36 ; i++)
	//		VelJOld[i]=0;
	//	
	//	DSPFlag=1;
	//	GLindex=0;
	//}
	////////////////////////SlongZ 用以在DSP時不用力規而用ZMP分配力量/////////////////////////
}

void Kine::CalCoord(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 運算Dynamics會用到的所有位置向量
	// 2012 Slongz Start
	// 20121221 WeiZh Lai Start
	******************************************************************/
	// 計算左右腳各軸在世界中的位置向量
	//	left leg
	Rstack[0] = CrdAll->data[9]-CrdAll->data[6];
	Rstack[1] = CrdAll->data[10]-CrdAll->data[7];
	Rstack[2] = CrdAll->data[11]-CrdAll->data[8]; // 軸3 knee 減 軸2 hip

	Rstack[3] = CrdAll->data[12]-CrdAll->data[9];
	Rstack[4] = CrdAll->data[13]-CrdAll->data[10];
	Rstack[5] = CrdAll->data[14]-CrdAll->data[11]; // 軸4 ankle 減 軸3 knee

	Rstack[6] = CrdAll->data[21]-CrdAll->data[12];
	Rstack[7] = CrdAll->data[22]-CrdAll->data[13];
	Rstack[8] = CrdAll->data[23]-CrdAll->data[14]; // 軸7 foot 減 軸4 ankle

	//  right leg
	Rstack[9] = CrdAll->data[48]-CrdAll->data[45];
	Rstack[10] = CrdAll->data[49]-CrdAll->data[46];
	Rstack[11] = CrdAll->data[50]-CrdAll->data[47]; // 軸16 knee 減 軸15 hip

	Rstack[12] = CrdAll->data[51]-CrdAll->data[48];
	Rstack[13] = CrdAll->data[52]-CrdAll->data[49];
	Rstack[14] = CrdAll->data[53]-CrdAll->data[50]; // 軸17 ankle 減 軸16 knee

	Rstack[15] = CrdAll->data[60]-CrdAll->data[51];
	Rstack[16] = CrdAll->data[61]-CrdAll->data[52];
	Rstack[17] = CrdAll->data[62]-CrdAll->data[53]; // 軸20 foot 減 軸17 ankle

	// 歸零上半身COM位置後 重新計算這個step上半身COM的位置向量 
	BodyRCOM[0] = 0;
	BodyRCOM[1] = 0;
	BodyRCOM[2] = 0;
	int Index = 18;
	for (int i = 6 ; i < 16 ; i++)
	{
		BodyRCOM[0] += pv_stack[Index]*mass_com[i];
		BodyRCOM[1] += pv_stack[Index+1]*mass_com[i];
		BodyRCOM[2] += pv_stack[Index+2]*mass_com[i];
		Index += 3;
	}

	BodyRCOM[0] = BodyRCOM[0]/BodyW;
	BodyRCOM[1] = BodyRCOM[1]/BodyW;
	BodyRCOM[2] = BodyRCOM[2]/BodyW;

	// 計算軸到桿件重心的位置向量
		// 左腳support
		//Fixed Shank(小腿) wXrcom
		MatScalarMul(CrdAll->data+12, 3, -1, rCOMccw);  //CrdAll->data+12 軸4 ankle
		MatAddAB(rCOMccw,pv_stack+3,rCOMccw,3);			//pv_stack345為左小腿重心位置 減掉Ankle位置後就為向量r
		
		//Fixed Teigh wXrcom
		MatScalarMul(CrdAll->data+9, 3, -1, rCOMccw+3);	//CrdAll->data+9 軸3 knee
		MatAddAB(rCOMccw+3,pv_stack,rCOMccw+3,3);		//pv_stack012為左大腿重心位置 減掉膝蓋位置後就為向量r
		
		//Swing Waist wXrcom
		for (int i = 0 ; i < 3 ; i++)
			rCOMccw[i+6]=BodyRCOM[i]-CrdAll->data[6+i];		//上半身(和腰桿件結合)重心位置減CrdAll->data6 左腳 hip
		
		//Swing Teigh wXrcom
		MatScalarMul(CrdAll->data+45, 3, -1, rCOMccw+9);		//CrdAll->data+45 軸15右腳hip
		MatAddAB(rCOMccw+9,pv_stack+9,rCOMccw+9,3);			//pv_stack91011為右大腿重心位置 減掉髖關節位置後就為向量r
		
		//Swing Shank wXrcom
		MatScalarMul(CrdAll->data+48, 3, -1, rCOMccw+12);	//CrdAll->data+48 軸16右腳knee
		MatAddAB(rCOMccw+12,pv_stack+12,rCOMccw+12,3);		//pv_stack121314為右小腿重心位置 減掉膝蓋位置後就為向量r
		
		//Swing Foot wXrcom
		MatScalarMul(CrdAll->data+51, 3, -1, rCOMccw+15);		//CrdAll->data+51 軸17右腳ankle
		MatAddAB(rCOMccw+15,pv_stack+15,rCOMccw+15,3);		//pv_stack151617為右腳板重心位置 減掉腳踝位置後就為向量r

		// 右腳support
		
		//Fixed Shank wXrcom
		MatScalarMul(CrdAll->data+51, 3, -1, rCOMcw);	//CrdAll->data+51 右腳 ankle
		MatAddAB(rCOMcw,pv_stack+12,rCOMcw,3);			//pv_stack121314為右小腿重心位置 減掉Ankle位置後就為向量r
		
		//Fixed Teigh wXrcom
		MatScalarMul(CrdAll->data+48, 3, -1, rCOMcw+3);		//CrdAll->data+48 右腳 knee
		MatAddAB(rCOMcw+3,pv_stack+9,rCOMcw+3,3);			//pv_stack91011為右大腿重心位置 減掉knee位置後就為向量r
		
		//Swing Waist wXrcom
		for (int i = 0 ; i < 3 ; i++)		//上半身(和腰桿件結合)重心位置減CrdAll->data454647 右腳 hip
			rCOMcw[i+6]=BodyRCOM[i]-CrdAll->data[i+DHJump+6];
		
		//Swing Teigh wXrcom
		MatScalarMul(CrdAll->data+6, 3, -1, rCOMcw+9);	//CrdAll->data+678 左腳hip
		MatAddAB(rCOMcw+9,pv_stack,rCOMcw+9,3);			//pv_stack012為左大腿重心位置 減掉hip位置後就為向量r
		
		//Swing Shank wXrcom
		MatScalarMul(CrdAll->data+9, 3, -1, rCOMcw+12);		//CrdAll->data+91011 左腳knee
		MatAddAB(rCOMcw+12,pv_stack+3,rCOMcw+12,3);			//pv_stack345為左小腿重心位置 減掉knee位置後就為向量r
		
		//Swing Foot wXrcom
		MatScalarMul(CrdAll->data+12, 3, -1, rCOMcw+15);		//CrdAll->data+12 左腳ankel
		MatAddAB(rCOMcw+15,pv_stack+6,rCOMcw+15,3);			//pv_stack678為左腳板重心位置 減掉腳踝位置後就為向量r

		//// 以下測試用
		//fstream Fx;
		//Fx.open("TestrCOMccw.txt",ios::app);		
		//for(int i = 0 ; i < 6 ; i++)
		//	Fx << rCOMccw[i*3] << "\t"<< rCOMccw[i*3+1] << "\t"<< rCOMccw[i*3+2]<< "\t";
		//Fx << endl;
		//Fx.close();

		//Fx.open("TestrCOMcw.txt",ios::app);		
		//for(int i = 0 ; i < 6 ; i++)
		//	Fx << rCOMcw[i*3] << "\t"<< rCOMcw[i*3+1] << "\t"<< rCOMcw[i*3+2]<< "\t";
		//Fx << endl;
		//Fx.close();

}

void Kine::NumDiff(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 對各軸的角度進行數值微分 得角速度與角加速度 為"純量" (弧度)
	// 2012 Slongz Start
	******************************************************************/
	if(gInitCount< 4)
	{
		if(gInitCount==0)
			for(int i = 0 ; i < 12 ; i++)
			{
				ThetaDD[i]=0;
				ThetaD[i]=0;
				//if(i<3)
				//{
				//	FootPosDDL[3];
				//	FootPosDDR[3];
				//	VelTotalCOM[i]=0;
				//	AccelTotalCOM[i]=0;
				//}
			}
		for(int i = 0 ; i < 6 ; i++)
		{
			ThetaLog[i+(gInitCount)*12  ]=FKLLeg->theta[i+1];
			ThetaLog[i+(gInitCount)*12+6]=FKRLeg->theta[i+1];
			//if(i<3)
			//{
			//	TotalCOMLog[i+(gInitCount)*3]=COG[i];
			//	TotalFootLogL[i+(gInitCount)*3]=CrdAll->data[i+21];
			//	TotalFootLogR[i+(gInitCount)*3]=CrdAll->data[i+60];
			//}
		}
	}
	else
	{
		for(int i = 0 ; i < 6 ; i++)
			{
					ThetaLog[i+48]=FKLLeg->theta[i+1];
					ThetaLog[i+48+6]=FKRLeg->theta[i+1];
					//if(i<3)
					//{
					//	TotalCOMLog[i+12]=COG[i];
					//	TotalFootLogL[i+12]=CrdAll->data[i+21];
					//	TotalFootLogR[i+12]=CrdAll->data[i+60];
					//}
			}
		for(int i = 0 ; i < 12 ; i++)
			{
				ThetaDD[i]=(ThetaLog[i]*11-ThetaLog[i+12]*56+ThetaLog[i+24]*114-ThetaLog[i+36]*104+ThetaLog[i+48]*35)/12/0.005/0.005;
				ThetaD[i]=(ThetaLog[i]*3-ThetaLog[i+12]*16+ThetaLog[i+24]*36-ThetaLog[i+36]*48+ThetaLog[i+48]*25)/12/0.005;
				ThetaLog[i]=ThetaLog[i+12];
				ThetaLog[i+12]=ThetaLog[i+24];
				ThetaLog[i+24]=ThetaLog[i+36];
				ThetaLog[i+36]=ThetaLog[i+48];
				//if(i < 3)
				//{
				//AccelTotalCOM[i]=(TotalCOMLog[i]*11-TotalCOMLog[i+3]*56+TotalCOMLog[i+6]*114-TotalCOMLog[i+9]*104+TotalCOMLog[i+12]*35)/12/0.005/0.005;
				//VelTotalCOM[i]=(TotalCOMLog[i]*3-TotalCOMLog[i+3]*16+TotalCOMLog[i+6]*36-TotalCOMLog[i+9]*48+TotalCOMLog[i+12]*25)/12/0.005;
				//TotalCOMLog[i]=TotalCOMLog[i+3];
				//TotalCOMLog[i+3]=TotalCOMLog[i+6];
				//TotalCOMLog[i+6]=TotalCOMLog[i+9];
				//TotalCOMLog[i+9]=TotalCOMLog[i+12];
				//
				//FootPosDDL[i]=(TotalFootLogL[i]*11-TotalFootLogL[i+3]*56+TotalFootLogL[i+6]*114-TotalFootLogL[i+9]*104+TotalFootLogL[i+12]*35)/12/0.005/0.005;
				//TotalFootLogL[i]=TotalFootLogL[i+3];
				//TotalFootLogL[i+3]=TotalFootLogL[i+6];
				//TotalFootLogL[i+6]=TotalFootLogL[i+9];
				//TotalFootLogL[i+9]=TotalFootLogL[i+12];
				//
				//FootPosDDR[i]=(TotalFootLogR[i]*11-TotalFootLogR[i+3]*56+TotalFootLogR[i+6]*114-TotalFootLogR[i+9]*104+TotalFootLogR[i+12]*35)/12/0.005/0.005;
				//TotalFootLogR[i]=TotalFootLogR[i+3];
				//TotalFootLogR[i+3]=TotalFootLogR[i+6];
				//TotalFootLogR[i+6]=TotalFootLogR[i+9];
				//TotalFootLogR[i+9]=TotalFootLogR[i+12];
				//}
			}
	}
		////以下測試用
		//fstream Fx;
		//Fx.open("TestThetaD.txt",ios::app);		
		//for(int i = 0 ; i < 12 ; i++)
		//	Fx << ThetaD[i] <<"\t";
		//Fx << endl;
		//Fx.close();

		//Fx.open("TestThetaDD.txt",ios::app);		
		//for(int i = 0 ; i < 12 ; i++)
		//	Fx << ThetaDD[i] <<"\t";
		//Fx << endl;
		//Fx.close();

		//fstream Fx;
		//Fx.open("TestFootPosDDR.txt",ios::app);		
		//for(int i = 0 ; i < 3 ; i++)
		//	Fx << FootPosDDR[i] <<"\t";
		//Fx << endl;
		//Fx.close();
		//Fx.open("TestFootPosDDL.txt",ios::app);		
		//for(int i = 0 ; i < 3 ; i++)
		//	Fx << FootPosDDL[i] <<"\t";
		//Fx << endl;
		//Fx.close();
	

}

void Kine::FDOmegaL(void)	
{
	/******************************************************************
	input: void
	output: void

	Note:
	// Find Link Omega  w(i) = w(i-1)+w(local)    
	// 在此注意Index的排序方式
	// 軸共點的角速度直接相加 因為皆為世界中
	// support腳所造成的角度須加負號
	// 在這裡不看151617(左腳support) 333435(右腳support) 在現實中其實是指連接踝關節兩個Joint的方形小桿件
	// 請注意 地桿就是地桿 直接想成零 
	// 特別注意! 151617(左腳support) 與333435(右腳support)皆不為地桿 是連接踝關節兩個軸的小方型桿件
	// 測試時若要看腳底板值 左腳suppprt請看333435 地桿Alpha為0 右腳support請看151617 地桿Alpha為0
	// 2012 Slongz Start
	// 20121203 WeiZh Start
	******************************************************************/
	if (selIK == 0||selIK==2)// Double Support & Right leg lifiting 
	{
		//ThetaD * Z
			for (int i = 0 ; i < 6 ; i++)	//Left First Right Second
			{
				MatScalarMul(ZAxisAll->data+3*i, 3, ThetaD+i, LocalOmegaJ+3*i);
				MatScalarMul(ZAxisAll->data+3*i+DHJump, 3, ThetaD+i+6, LocalOmegaJ+3*i+AxisJump);
				MatScalarMul(LocalOmegaJ+3*i, 3,-1.0, LocalOmegaJ+3*i);	// For Supporting Leg(左腳)
			}
		//Accumulate
				//MatScalarMul(LocalOmegaJ+15, 3, 0.0, OmegaL+15);	// Fixed Ankle Omega=0 
			for(int i = 0 ; i < 3 ; i++)	// OmegaL151617為指連接踝關節兩個Joint的方形小桿件 因為地桿LocalOmegaG = 0
				OmegaL[i+15] = LocalOmegaJ[i+15];
			for (int i = 4 ; i >= 0  ; i--)	// Left First Right Second  由左踝(left ankle)向上
				MatAddAB(LocalOmegaJ+3*i,OmegaL+3*(i+1),OmegaL+3*i,3);	// OmegaL121314 = LocalOmegaJ121314 + OmegaL151617 因為OmegaL151617 = LocalOmegaJ151617
			for (int i = 0 ; i <6  ; i++)	// Left First Right Second  由右髖(right hip)向下
			{
				if (i==0)
				MatAddAB(LocalOmegaJ+AxisJump,OmegaL,OmegaL+AxisJump,3);
				else
				MatAddAB(LocalOmegaJ+3*i+AxisJump,OmegaL+3*(i-1)+AxisJump,OmegaL+3*i+AxisJump,3);
			}
	}
	else if(selIK == 1)	// Left leg lifiting 
	{
		//ThetaD * Z   計算LocalOmega
			for (int i = 0 ; i < 6 ; i++)	//Left First Right Second
			{
				MatScalarMul(ZAxisAll->data+3*i, 3, ThetaD+i, LocalOmegaJ+3*i);
				MatScalarMul(ZAxisAll->data+3*i+DHJump, 3, ThetaD+i+6, LocalOmegaJ+3*i+AxisJump);
				MatScalarMul(LocalOmegaJ+3*i+AxisJump, 3,-1.0, LocalOmegaJ+3*i+AxisJump);	//for support leg(右腳)
			}
		//Accumulate
				//MatScalarMul(LocalOmegaJ+15+AxisJump, 3, 0.0, OmegaL+15+AxisJump);	// Fixed Ankle Omega=0
			for(int i = 0 ; i < 3 ; i++)	// OmegaL333435為指連接踝關節兩個Joint的方形小桿件 因為地桿LocalOmegaG = 0
				OmegaL[i+15+AxisJump] = LocalOmegaJ[i+15+AxisJump];
			for (int i = 4 ; i >=0  ; i--)	//Left First Right Second  由右踝向上
				MatAddAB(OmegaL+3*(i+1)+AxisJump,LocalOmegaJ+3*i+AxisJump,OmegaL+3*i+AxisJump,3);	// OmegaL303132 = LocalOmegaJ303132+LocalOmegaJ333435 因為因為OmegaL333435 = LocalOmegaJ333435
			for (int i = 0 ; i <6  ; i++)	//Left First Right Second  由左髖向下
			{
				if (i==0)
				MatAddAB(OmegaL+AxisJump,LocalOmegaJ,OmegaL,3);
				else
				MatAddAB(OmegaL+3*(i-1),LocalOmegaJ+3*i,OmegaL+3*i,3);
			}
	}
		//以下測試用

		//fstream Fx;
		//Fx.open("TestLocalOmega.txt",ios::app);		
		//for(int i = 0 ; i < 12 ; i++)
		//	Fx << LocalOmegaJ[i*3] << "\t" << LocalOmegaJ[i*3+1] << "\t" << LocalOmegaJ[i*3+2] << "\t";
		//Fx << endl;
		//Fx.close();
	
		//fstream Fx;
		//Fx.open("TestOmegaFootL.txt",ios::app);		
		//if(selIK == 1)//(右腳support)
		//Fx << OmegaL[15] << "\t"<< OmegaL[16] << "\t"<< OmegaL[17]<< endl;
		//else if(selIK == 0||selIK==2) //左腳support
		//Fx << 0 << "\t"<< 0 << "\t"<< 0<< endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestOmegaFootR.txt",ios::app);		
		//if(selIK == 1)//(右腳support)
		//Fx << 0 << "\t"<< 0 << "\t"<< 0<< endl;
		//else if(selIK == 0||selIK==2) //左腳support
		//Fx << OmegaL[33] << "\t"<< OmegaL[34] << "\t"<< OmegaL[35]<< endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestOmegaShankL.txt",ios::app);		
		//if(selIK == 1)//(右腳support)
		//Fx << OmegaL[9] << "\t"<< OmegaL[10] << "\t"<< OmegaL[11]<< endl;
		//else if(selIK == 0||selIK==2) //左腳support
		//Fx << OmegaL[12] << "\t"<< OmegaL[13] << "\t"<< OmegaL[14]<< endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestOmegaShankR.txt",ios::app);		
		//if(selIK == 1)//(右腳support)
		//Fx << OmegaL[30] << "\t"<< OmegaL[31] << "\t"<< OmegaL[32]<< endl;
		//else if(selIK == 0||selIK==2) //左腳support
		//Fx << OmegaL[27] << "\t"<< OmegaL[28] << "\t"<< OmegaL[29]<< endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestOmegaWaist.txt",ios::app);		
		//if(selIK == 1)//(右腳support)
		//Fx << OmegaL[18] << "\t"<< OmegaL[19] << "\t"<< OmegaL[20]<< endl;
		//else if(selIK == 0||selIK==2) //左腳support
		//Fx << OmegaL[0] << "\t"<< OmegaL[1] << "\t"<< OmegaL[2]<< endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestOmegaThighL.txt",ios::app);		
		//if(selIK == 1)//(右腳support)
		//Fx << OmegaL[6] << "\t"<< OmegaL[7] << "\t"<< OmegaL[8]<< endl;
		//else if(selIK == 0||selIK==2) //左腳support
		//Fx << OmegaL[9] << "\t"<< OmegaL[10] << "\t"<< OmegaL[11]<< endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestOmegaThighR.txt",ios::app);		
		//if(selIK == 1)//(右腳support)
		//Fx << OmegaL[27] << "\t"<< OmegaL[28] << "\t"<< OmegaL[29]<< endl;
		//else if(selIK == 0||selIK==2) //左腳support
		//Fx << OmegaL[24] << "\t"<< OmegaL[25] << "\t"<< OmegaL[26]<< endl;
		//Fx.close();




}
void Kine::FDVelJ(void)	
{
	/******************************************************************
	input: void
	output: void

	Note:
	// Find Joint Vel v=v+ w X r r:Rstack 
	// 自己Joint的速度存在自己的VelJ 的Index裡 
	// 這裡的速度以世界座標描述 並直接指定support腳腳踝軸速度為0
	// 共點的軸直接指定相同速度
	// 2012 Slongz Start
	// 20121205 WeiZh Start
	******************************************************************/
	double TempPvStack[3];
	if (selIK == 0||selIK==2)// Double Support & Right leg lifiting  
	{
	//Vector Omega	cross r  計算LocalVel
		//Fixed ankle wXr
		for (int i = 0 ; i < 3 ; i++)
			LocalVelJ[i+12]=0;	// 在此直接指定LocalVelJ121314 為零是因為和Joint151617r=0 值就直接給定 跳過LocalVelJ151617是因為不需要 151617是地

		//Fixed Knee wXr
		Cross2Vd(Rstack+3,OmegaL+12,LocalVelJ+9); //小腿link的角速度為Omega121314 膝蓋Joint的Index為91011

		//Fixed Hip wXr
		Cross2Vd(Rstack,OmegaL+9,LocalVelJ+6);  //大腿的角速度為Omega91011 由下往上數第一個髖關節Joint的Index為678
		for (int i = 0 ; i < 6 ; i++)
			LocalVelJ[i]=0; // 在此直接指定LocalVelJ012 345 ,012 345軸的Local速度為零是因為和Joint678 r=0 值就直接給定

		//Swing Hip wXr 
		for (int i = 0 ; i < 3 ; i++)
			TempPvStack[i]=CrdAll->data[6+i+DHJump]-CrdAll->data[6+i];  //兩隻腿髖關節沒有設定Rstack 直接取DH值相減
		Cross2Vd(OmegaL,TempPvStack,LocalVelJ+AxisJump);    //角速度由678開始累計678+345+012的角速度為Omega012 和兩個髖關節的距離cross 存在右腳上面數下來第一個軸181920
		
		for (int i = 3 ; i < 9 ; i++)
			LocalVelJ[i+AxisJump]=0; // 在此直接指定LocalVelJ212223 242526 ,212223 242526軸的Local速度為零是因為和Joint181920的r=0 值就直接給定

		//Swing Knee wXr
		Cross2Vd(OmegaL+6+AxisJump,Rstack+StackJump,LocalVelJ+9+AxisJump);

		//Swing ankle wXr
		Cross2Vd(OmegaL+9+AxisJump,Rstack+3+StackJump,LocalVelJ+12+AxisJump);		
		
		for (int i = 0 ; i < 3 ; i++)
			LocalVelJ[i+15+AxisJump]=0; // 在此直接指定LocalVelJ333435 = 0, 因為333435與303132重合 r = 0 ; 故Local軸的速度 = 0 
	
	//Accumulate
		//MatScalarMul(LocalVelJ+15, 3, 0.0, VelJ+15);//r=0
		for (int i = 0 ; i < 3 ; i++)  // 在此直接指定VelJ151617 的Joint速度絕對值 因為151617為地 
			VelJ[i+15] = 0;  		
		for (int i = 4 ; i >= 0 ; i--)		//左腳下往上
			MatAddAB(VelJ+3*(i+1),LocalVelJ+3*i,VelJ+3*i,3);
		
		//右腳上往下
		MatAddAB(VelJ,LocalVelJ+AxisJump,VelJ+AxisJump,3);
		for (int i = 1 ; i < 6 ; i++)		
			MatAddAB(VelJ+3*(i-1)+AxisJump,LocalVelJ+3*i+AxisJump,VelJ+3*i+AxisJump,3);
	}
	else if(selIK == 1)// Left leg lifiting 
	{
	//Vector Omega	cross r  計算LocalVel
		//Fixed ankle wXr
		for (int i = 0 ; i < 3 ; i++)   // 在此直接指定LocalVelJ303132 為零是因為和Joint333435r=0 值就直接給定 跳過LocalVelJ333435是因為不需要 333435是地
			LocalVelJ[i+AxisJump+12]=0; //r=0

		//Fixed Knee wXr
		Cross2Vd(Rstack+StackJump+3, OmegaL+AxisJump+12,LocalVelJ+AxisJump+9); //  Joint272829的速度由OmegaLink303132 和-Rstack121314 cross 存在LocalVelJoint272829

		//Fixed Hip wXr
		Cross2Vd(Rstack+StackJump,OmegaL+AxisJump+9,LocalVelJ+AxisJump+6);  //大腿的角速度為Omega272829 由下往上數第一個髖關節Joint的Index為242526
		for (int i = 0 ; i < 6 ; i++)				// 在此直接指定LocalVelJ212223 181920 ,212223 181920軸的Local速度為零是因為和Joint242526 r=0 值就直接給定
			LocalVelJ[i+AxisJump]=0; // r=0

		//Swing Hip wXr 
		for (int i=0;i<3; i++)
			TempPvStack[i]=CrdAll->data[6+i]-CrdAll->data[6+i+DHJump];  //兩隻腿髖關節沒有設定Rstack 直接取DH值相減
		Cross2Vd(OmegaL+AxisJump,TempPvStack,LocalVelJ);		//角速度由242526開始累計242526+212223+181920的角速度為Omega181920 和兩個髖關節的距離cross 存在左腳上面數下來第一個軸012
		for (int i = 0 ; i < 6 ; i++)		// 在此直接指定LocalVelJ345 678 ,345 678軸的Local速度為零是因為和Joint012的r=0 值就直接給定
			LocalVelJ[i+3]=0; // r=0

		//Swing Knee wXr
		Cross2Vd(OmegaL+6,Rstack,LocalVelJ+9);

		//Swing ankle wXr
		Cross2Vd(OmegaL+9,Rstack+3,LocalVelJ+12);		
		
		for (int i = 0 ; i < 3 ; i++)		// 在此直接指定LocalVelJ151617 = 0, 因為151617與121314重合 r = 0 ; 故151617Local軸的速度 = 0 
			LocalVelJ[i+15]=0; //r=0
	
	//Accumulate
		//MatScalarMul(LocalVelJ+15+AxisJump, 3, 0.00, VelJ+15+AxisJump);//r=0
		for (int i = 0 ; i < 3 ; i++)   // 在此直接指定VelJ333435 的Joint速度絕對值 因為333435為地 
			VelJ[i+AxisJump+15] = 0;  				
		for (int i = 4 ; i >= 0; i--)	// 右腳下往上
			MatAddAB(VelJ+3*(i+1)+AxisJump,LocalVelJ+3*i+AxisJump,VelJ+3*i+AxisJump,3);
		
		//左腳上往下
		MatAddAB(VelJ+AxisJump,LocalVelJ,VelJ,3);
		for (int i = 1 ; i < 6 ; i++)	//左腳上往下
			MatAddAB(VelJ+3*(i-1),LocalVelJ+3*i,VelJ+3*i,3);
	}
		
	// 以下測試用	
		//fstream Fx;
		//Fx.open("TestAnklePosL.txt",ios::app);
		//	Fx << CrdAll->data[12] <<"\t" << CrdAll->data[13] <<"\t"<< CrdAll->data[14]<<  endl;
		//Fx.close();
		//
		//Fx.open("TestFootPosL.txt",ios::app);
		//	Fx << CrdAll->data[21] <<"\t" << CrdAll->data[22] <<"\t"<< CrdAll->data[23]<<  endl;
		//Fx.close();
	
		//Fx.open("TestKneePosL.txt",ios::app);
		//	Fx << CrdAll->data[9] <<"\t" << CrdAll->data[10] <<"\t"<< CrdAll->data[11]<<  endl;
		//Fx.close();
		//
		//Fx.open("TestHipPosL.txt",ios::app);
		//	Fx << CrdAll->data[6] <<"\t" << CrdAll->data[7] <<"\t"<< CrdAll->data[8]<<  endl;
		//Fx.close();
		//
		//Fx.open("TestAnklePosR.txt",ios::app);
		//	Fx << CrdAll->data[51] <<"\t" << CrdAll->data[52] <<"\t"<< CrdAll->data[53]<<  endl;
		//Fx.close();

		//Fx.open("TestFootPosR.txt",ios::app);
		//	Fx << CrdAll->data[60] <<"\t" << CrdAll->data[61] <<"\t"<< CrdAll->data[62]<<  endl;
		//Fx.close();

		//Fx.open("TestKneePosR.txt",ios::app);
		//	Fx << CrdAll->data[48] <<"\t" << CrdAll->data[49] <<"\t"<< CrdAll->data[50]<<  endl;
		//Fx.close();

		//Fx.open("TestHipPosR.txt",ios::app);
		//	Fx << CrdAll->data[45] <<"\t" << CrdAll->data[46] <<"\t"<< CrdAll->data[47]<<  endl;
		//Fx.close();

		//Fx.open("TestBodyPos.txt",ios::app);
		//	Fx << BodyRCOM[0] <<"\t" << BodyRCOM[1] <<"\t"<< BodyRCOM[2]<<  endl;
		//Fx.close();
	
		//Fx.open("TestVelJ.txt",ios::app);
		//for(int i = 0 ; i < 12 ; i++)
		//	Fx <<VelJ[i*3] << "\t" << VelJ[i*3+1] << "\t" << VelJ[i*3+2] << "\t";
		//Fx << endl;
		//Fx.close();

}
void Kine::FDAlphaL(void) 
{
	/******************************************************************
	input: void
	output: void

	Note:
	// find Link angle acceleration
	// alpha(i) = alpha(i-1)+alpha(local)   
	// 在這裡不看AlphaL151617 在現實中其實是指連接踝關節兩個Joint的方形小桿件
	// 請注意 地桿就是地桿 直接想成零 
	// 特別注意! 151617(左腳support) 與333435(右腳support)皆不為地桿 是連接踝關節兩個軸的小方型桿件
	// 測試時若要看腳底板值 左腳suppprt請看333435 地桿Alpha為0 右腳support請看151617 地桿Alpha為0
	// 2012 Slongz Start
	// 20121205 WeiZh Start
	******************************************************************/
	if (selIK == 0||selIK==2)// Double Support & Right leg lifiting 
	{
	//Find Joint Alpha      Checked
		//OmegaL cross Local OmegaJ        先算左腳(下往上)再算右腳(上往下)   VectorTemp1
			for (int i = 0 ; i < 3 ; i++)  //在此不計算VectorTemp1 151617 因為VectorTemp1 151617 = 0(BASE) 但仍然給值以利後面累加
				VectorTemp1[i+15] = 0;
			for (int i = 4 ; i >=0 ; i--) 
				Cross2Vd(OmegaL+3*(i+1), LocalOmegaJ+3*i,VectorTemp1+3*i);
			Cross2Vd(OmegaL, LocalOmegaJ+AxisJump,VectorTemp1+AxisJump);
			for (int i = 1 ; i < 6 ; i++)
				Cross2Vd(OmegaL+3*(i-1)+AxisJump, LocalOmegaJ+3*i+AxisJump,VectorTemp1+3*i+AxisJump);

		//Theta dot dot * z軸
			for (int i = 0 ; i < 6 ; i++)
			{
				MatScalarMul(ZAxisAll->data+3*i, 3, ThetaDD+i, VectorTemp2+3*i);
				MatScalarMul(ZAxisAll->data+3*i+DHJump, 3, ThetaDD+i+6, VectorTemp2+3*i+AxisJump);
				MatScalarMul(VectorTemp2+3*i, 3,-1.0, VectorTemp2+3*i);// for support leg
			}
		//Accumulate			
			for (int i = 0 ; i < 12 ; i++)  //直接累加012~333435的VectorTemp1和VectorTemp2 下面就不用累加了(槓掉的部分)
				MatAddAB(VectorTemp1+3*i,VectorTemp2+3*i,AlphaL+3*i,3);
			
			for (int i = 4 ; i >= 0 ; i--) //左腳(support腳)		在此不計算AlphaL151617的原因是在上面累加Temp1和Temp2時Alpha151617就已經計算完了
					MatAddAB(AlphaL+3*(i+1),AlphaL+3*i,AlphaL+3*i,3);
			
			//右腳
			MatAddAB(AlphaL,AlphaL+AxisJump,AlphaL+AxisJump,3);
			for (int i = 1 ; i < 6 ; i++) 
				MatAddAB(AlphaL+3*(i-1)+AxisJump,AlphaL+3*i+AxisJump,AlphaL+3*i+AxisJump,3);
	}
	else if(selIK == 1)// Left leg lifiting 
	{
	//Find Joint Alpha      Checked
		//OmegaL cross Local OmegaJ    (VectorTemp1)
			for (int i = 0 ; i < 3 ; i++)  //在此不計算VectorTemp1 333435 因為VectorTemp1 333435 = 0(BASE) 但仍然給值以利後面累加
				VectorTemp1[i+AxisJump+15] = 0;
			for (int i = 4 ; i >= 0 ; i--)  
				Cross2Vd(OmegaL+3*(i+1)+AxisJump, LocalOmegaJ+3*i+AxisJump,VectorTemp1+3*i+AxisJump);
			Cross2Vd(OmegaL+AxisJump, LocalOmegaJ,VectorTemp1);
			for (int i = 1 ; i < 6 ; i++)
				Cross2Vd(OmegaL+3*(i-1), LocalOmegaJ+3*i,VectorTemp1+3*i);

		//Theta dot dot * z軸
			for (int i = 0 ; i < 6 ; i++)
			{
				MatScalarMul(ZAxisAll->data+3*i, 3, ThetaDD+i, VectorTemp2+3*i);
				MatScalarMul(ZAxisAll->data+3*i+DHJump, 3, ThetaDD+i+6, VectorTemp2+3*i+AxisJump);
				MatScalarMul(VectorTemp2+3*i+AxisJump, 3,-1.0, VectorTemp2+3*i+AxisJump);// for support leg
			}
		//Accumulate
			for (int i = 0 ; i < 12 ; i++)  //直接累加012~333435的VectorTemp1和VectorTemp2 下面就不用累加了(槓掉的部分)
				MatAddAB(VectorTemp1+3*i,VectorTemp2+3*i,AlphaL+3*i,3);
			
			for (int i = 4 ; i >= 0 ; i--)  //在此不計算AlphaL333435的原因是在上面累加Temp1和Temp2時Alpha333435就已經計算完了
					MatAddAB(AlphaL+3*(i+1)+AxisJump,AlphaL+3*i+AxisJump,AlphaL+3*i+AxisJump,3);
			
			MatAddAB(AlphaL+AxisJump,AlphaL,AlphaL,3);
			for (int i = 1 ; i < 6 ; i++)
					MatAddAB(AlphaL+3*(i-1),AlphaL+3*i,AlphaL+3*i,3);
	}

		////以下測試用

		//fstream Fx;
		////Fx.open("TestLocalOmega.txt",ios::app);		
		////for(int i = 0 ; i < 12 ; i++)
		////	Fx << LocalOmegaJ[i*3] << "\t" << LocalOmegaJ[i*3+1] << "\t" << LocalOmegaJ[i*3+2] << "\t";
		////Fx << endl;
		////Fx.close();
	
		////fstream Fx;
		//Fx.open("TestAlphaLFootL.txt",ios::app);		
		//if(selIK == 1)//(右腳support)
		//Fx << AlphaL[15] << "\t"<< AlphaL[15] << "\t"<< AlphaL[15]<< endl;
		//else if(selIK == 0||selIK==2) //左腳support
		//Fx << 0 << "\t"<< 0 << "\t"<< 0 << endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestAlphaLFootR.txt",ios::app);		
		//if(selIK == 1)//(右腳support)
		//Fx << 0 << "\t"<< 0 << "\t"<< 0 << endl;
		//else if(selIK == 0||selIK==2) //左腳support
		//Fx << AlphaL[33] << "\t"<< AlphaL[34] << "\t"<< AlphaL[35] << endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestAlphaLShankL.txt",ios::app);		
		//if(selIK == 1)//(右腳support)
		//Fx << AlphaL[9] << "\t"<< AlphaL[10] << "\t"<< AlphaL[11]<< endl;
		//else if(selIK == 0||selIK==2) //左腳support
		//Fx << AlphaL[12] << "\t"<< AlphaL[13] << "\t"<< AlphaL[14]<< endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestAlphaLShankR.txt",ios::app);		
		//if(selIK == 1)//(右腳support)
		//Fx << AlphaL[30] << "\t"<< AlphaL[31] << "\t"<< AlphaL[32]<< endl;
		//else if(selIK == 0||selIK==2) //左腳support
		//Fx << AlphaL[27] << "\t"<< AlphaL[28] << "\t"<< AlphaL[29]<< endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestAlphaLWaist.txt",ios::app);		
		//if(selIK == 1)//(右腳support)
		//Fx << AlphaL[18] << "\t"<< AlphaL[19] << "\t"<< AlphaL[20]<< endl;
		//else if(selIK == 0||selIK==2) //左腳support
		//Fx << AlphaL[0] << "\t"<< AlphaL[1] << "\t"<< AlphaL[2]<< endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestAlphaLThighL.txt",ios::app);		
		//if(selIK == 1)//(右腳support)
		//Fx << AlphaL[6] << "\t"<< AlphaL[7] << "\t"<< AlphaL[8]<< endl;
		//else if(selIK == 0||selIK==2) //左腳support
		//Fx << AlphaL[9] << "\t"<< AlphaL[10] << "\t"<< AlphaL[11]<< endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestAlphaLThighR.txt",ios::app);		
		//if(selIK == 1)//(右腳support)
		//Fx << AlphaL[27] << "\t"<< AlphaL[28] << "\t"<< AlphaL[29]<< endl;
		//else if(selIK == 0||selIK==2) //左腳support
		//Fx << AlphaL[24] << "\t"<< AlphaL[25] << "\t"<< AlphaL[26]<< endl;
		//Fx.close();

}
void Kine::FDAccelJ(void)		
{
	/******************************************************************
	input: void
	output: void

	Note:
	// Find Joint Accel
	// 累加所有的VectorTemp1 VectorTemp2 存到自己的Index
	// AccelJ  = OmegaJ X LocalVelJ (VectorTemp1) + AlphaJ X r (VectorTemp2)
	// 2012 Slongz Start
	// 20121205 WeiZh Start
	******************************************************************/
	double TempPvStack[3];
	if (selIK == 0||selIK==2)// Double Support & Right leg lifiting 
	{
	//OmegaJ cross LocalVelJ  VectorTemp1
		for (int i = 0 ; i < 3 ; i++)	//在此給定151617的VectorTemp1 = 0 因為是地 給是為了下面累加VectorTemp1 VectorTemp2 方便
			VectorTemp1[i+15] = 0;	//r=0
		//左腳
		for (int i = 4 ; i >= 0 ; i--)	
			Cross2Vd(OmegaL+3*(i+1), LocalVelJ+3*i, VectorTemp1+3*i);
		
		//右腳	
			Cross2Vd(OmegaL, LocalVelJ+AxisJump, VectorTemp1+AxisJump);
		for (int i = 1 ; i < 6 ; i++)	
			Cross2Vd(OmegaL+3*(i-1)+AxisJump, LocalVelJ+3*i+AxisJump, VectorTemp1+3*i+AxisJump);
				
	//AlphaJ cross r		VectorTemp2
		//Fixed ankle AXr
		for (int i = 0 ; i < 6 ; i++)	//在此給定151617的VectorTemp2 = 0 因為是地 給是為了下面累加VectorTemp1 VectorTemp2 方便
			VectorTemp2[i+12]=0;	//Fixed on ground

		//Fixed Knee AXr
		Cross2Vd(Rstack+3, AlphaL+12, VectorTemp2+9);

		//Fixed Hip AXr
		Cross2Vd(Rstack, AlphaL+9, VectorTemp2+6);
		for (int i = 0 ; i < 6 ; i++)	//直接指定VectorTemp2 012 345 的值 r為0 直接指定
			VectorTemp2[i]=0;	// r=0

		//Swing Hip AXr 
		for (int i = 0 ; i < 3 ; i++)		
			TempPvStack[i]=CrdAll->data[6+i+DHJump]-CrdAll->data[6+i];
		Cross2Vd(AlphaL,TempPvStack,VectorTemp2+AxisJump);	//計算右腳第一軸的AlphaJ cross r 因為沒有算Rstack 直接取DH相減454647-678
		for (int i = 0 ; i < 6 ; i++)	//直接指定212223 242526的AlphaJ cross r = 0 因為r = 0
			VectorTemp2[i+3+AxisJump]=0;	// r=0

		//Swing Knee AXr
		Cross2Vd(AlphaL+6+AxisJump,Rstack+StackJump,VectorTemp2+9+AxisJump);

		//Swing ankle AXr
		Cross2Vd(AlphaL+9+AxisJump,Rstack+3+StackJump,VectorTemp2+12+AxisJump);		
		for (int i = 0 ; i < 3 ; i++)	//直接指定VectorTemp2 333435 = 0 因為和303132的r=0  
			VectorTemp2[i+15+AxisJump]=0;	//r=0
	
	//Accumulate
		for (int i = 0 ; i < 12 ; i++)	//累加所有的VectorTemp1 VectorTemp2 存到自己的Index
			MatAddAB(VectorTemp1+3*i,VectorTemp2+3*i,AccelJ+3*i,3);
		// 左腳
		for (int i = 0 ; i < 3 ; i++)	// 直接指定腳踝進地板第一個Joint為0
			AccelJ[i+15] = 0;
		for (int i = 4 ; i >= 0 ; i--)
			MatAddAB(AccelJ+3*(i+1),AccelJ+3*i,AccelJ+3*i,3);
		
		// 右腳
		MatAddAB(AccelJ,AccelJ+AxisJump,AccelJ+AxisJump,3);
		for (int i = 1 ; i < 6 ; i++)
				MatAddAB(AccelJ+3*(i-1)+AxisJump,AccelJ+3*i+AxisJump,AccelJ+3*i+AxisJump,3);
	}
	else if(selIK == 1)// Left leg lifiting 
	{
	//OmegaJ cross LocalVelJ VectorTemp1
		for (int i = 0 ; i < 3 ; i++)	//在此給定333435的VectorTemp1 = 0 因為是地 給是為了下面累加VectorTemp1 VectorTemp2 方便
			VectorTemp1[i+AxisJump+15]=0;	//r=0
		for (int i = 4 ; i >= 0 ; i--)
			Cross2Vd(OmegaL+3*(i+1)+AxisJump, LocalVelJ+3*i+AxisJump, VectorTemp1+3*i+AxisJump);
		
		//左腳
		Cross2Vd(OmegaL+AxisJump, LocalVelJ, VectorTemp1);	
		for (int i = 1 ; i < 6 ; i++)	
			Cross2Vd(OmegaL+3*(i-1), LocalVelJ+3*i, VectorTemp1+3*i);

	//AlphaJ cross r
		//Fixed ankle AXr
		for (int i = 0 ; i < 6 ; i++)	//在此給定333435的VectorTemp2 = 0 因為是地 給是為了下面累加VectorTemp1 VectorTemp2 方便
			VectorTemp2[i+AxisJump+12]=0;	//Fixed on ground

		//Fixed Knee AXr
		Cross2Vd(Rstack+12, AlphaL+AxisJump+12, VectorTemp2+AxisJump+9);

		//Fixed Hip AXr
		Cross2Vd(Rstack+9, AlphaL+AxisJump+9, VectorTemp2+AxisJump+6);
		for (int i = 0 ; i < 6 ; i++)	//直接指定VectorTemp2 212223 181920 的值 r為0 直接指定
			VectorTemp2[i+AxisJump]=0;	// r=0

		//Swing Hip AXr 
		for (int i = 0 ; i < 3 ; i++)
			TempPvStack[i]=CrdAll->data[6+i]-CrdAll->data[6+i+DHJump];
		Cross2Vd(AlphaL+AxisJump,TempPvStack,VectorTemp2);	//計算右腳第一軸的AlphaJ cross r 因為沒有算Rstack 直接取DH相減678-454647
		for (int i = 0 ; i < 6 ; i++)	//直接指定345 678的AlphaJ cross r = 0 因為r = 0
			VectorTemp2[i+3]=0;	// r=0

		//Swing Knee AXr
		Cross2Vd(AlphaL+6,Rstack+0,VectorTemp2+9);

		//Swing ankle AXr
		Cross2Vd(AlphaL+9,Rstack+3,VectorTemp2+12);		
		for (int i = 0 ; i < 3 ; i++)	//直接指定VectorTemp2 151617 = 0 因為和121314的r=0  
			VectorTemp2[i+15]=0; //r=0
	
	//Accumulate	
		for (int i = 0 ; i < 12 ; i++)	//累加所有的VectorTemp1 VectorTemp2 存到自己的Index
			MatAddAB(VectorTemp1+3*i,VectorTemp2+3*i,AccelJ+3*i,3);
		// 右腳
		for (int i = 0 ; i < 3 ; i++)	// 直接指定腳踝進地板第一個Joint為0
			AccelJ[i+15+AxisJump] = 0;

		for (int i = 4 ; i >= 0 ; i--)
			MatAddAB(AccelJ+3*(i+1)+AxisJump,AccelJ+3*i+AxisJump,AccelJ+3*i+AxisJump,3);
		
		// 左腳
		MatAddAB(AccelJ+AxisJump,AccelJ,AccelJ,3);
		for (int i = 1 ; i < 6 ; i++)
			MatAddAB(AccelJ+3*(i-1),AccelJ+3*i,AccelJ+3*i,3);
	}

		// 以下測試用
		//fstream Fx;
		//Fx.open("TestAccelJ.txt",ios::app);
		//for(int i = 0 ; i < 12 ; i++)
		//	Fx <<AccelJ[i*3] << "\t" << AccelJ[i*3+1] << "\t" << AccelJ[i*3+2] << "\t";
		//Fx << endl;
		//Fx.close();
		//Fx.open("TestAccelJTemp1.txt",ios::app);
		//for(int i = 0 ; i < 12 ; i++)
		//	Fx <<VectorTemp1[i*3] << "\t" << VectorTemp1[i*3+1] << "\t" << VectorTemp1[i*3+2] << "\t";
		//Fx << endl;
		//Fx.close();

		//Fx.open("TestAccelJTemp2.txt",ios::app);
		//for(int i = 0 ; i < 12 ; i++)
		//	Fx <<VectorTemp2[i*3] << "\t" << VectorTemp2[i*3+1] << "\t" << VectorTemp2[i*3+2] << "\t";
		//Fx << endl;
		//Fx.close();
}
void Kine::FDVelCOM(void)	
{	
	/******************************************************************
	input: void
	output: void

	Note:
	// Find Link COM Vel v=v+ w X r r:Rstack  作法及Index配置模仿FDVelJ 
	// 請注意rcom的計算方式!!!! 以及個別ComputeVelCOM的Index存放位置 
	// 因為ComputeVelCOM的位置在桿件上 所以存放方式和Omega、Alpha相同
	// 為了配合下面計算力矩時需用用到重心對兩端點Joint的向量 在此剛好利用左右腳support不同時對重心的向量指向方向不同 一並存取
	// 由下關節指向重心為 rCOMccw (逆時鐘) 由上關節指向重心為 rCOMcw (順時鐘) 各有3*6 = 18個Index
	// 但由於Torque計算需同時使用到ccw 和 cw的向量 故移至CalCoord計算
	// 2012 Slongz Start
	// 20121206 WeiZh Start 20121217 改COM向量
	******************************************************************/
	if (selIK == 0||selIK==2)// Double Support  & Right leg lifiting 
	{
	//Vector Omega cross rcom
		//Fixed Foot wXrcom
		for (int i = 0 ; i < 3 ; i++)	//在這裡指定LocalVelCOM151617是為了等等疊加方便
			LocalVelCOM[i+15]=0; //r=0

		//Fixed Shank(小腿) wXrcom
		//MatScalarMul(CrdAll->data+12, 3, -1, rCOMccw);  //CrdAll->data+12 軸4 ankle
		//MatAddAB(rCOMccw,pv_stack+3,rCOMccw,3);			//pv_stack345為左小腿重心位置 減掉Ankle位置後就為向量r
		Cross2Vd(OmegaL+12,rCOMccw,LocalVelCOM+12);		//小腿Link的Omega為121314 把重心速度存在121314

		//Fixed Teigh wXrcom
		//MatScalarMul(CrdAll->data+9, 3, -1, rCOMccw+3);	//CrdAll->data+9 軸3 knee
		//MatAddAB(rCOMccw+3,pv_stack,rCOMccw+3,3);		//pv_stack012為左大腿重心位置 減掉膝蓋位置後就為向量r
		Cross2Vd(OmegaL+9,rCOMccw+3,LocalVelCOM+9);		//大腿Link的Omega為91011 把重心速度存在91011
		for (int i = 0 ; i < 6 ; i++) // LocalVelCOM345 678=0  因為和012共點 所以Locak重心速度直接指定為0
			LocalVelCOM[i+3]=0;	// r=0

		//Swing Waist wXrcom
		//for (int i = 0 ; i < 3 ; i++)
		//	rCOMccw[i+6]=BodyRCOM[i]-CrdAll->data[6+i];		//上半身(和腰桿件結合)重心位置減CrdAll->data6 左腳 hip
		Cross2Vd(OmegaL,rCOMccw+6,LocalVelCOM);
		for (int i = 0 ; i < 6 ; i++)		//LocalVelCOM181920 212223=0 因為和242526共點 所以Locak重心速度為0
			LocalVelCOM[i+AxisJump]=0; // r=0

		//Swing Teigh wXrcom
		//MatScalarMul(CrdAll->data+45, 3, -1, rCOMccw+9);		//CrdAll->data+45 軸15右腳hip
		//MatAddAB(rCOMccw+9,pv_stack+9,rCOMccw+9,3);			//pv_stack91011為右大腿重心位置 減掉髖關節位置後就為向量r
		Cross2Vd(OmegaL+6+AxisJump,rCOMccw+9,LocalVelCOM+6+AxisJump);

		//Swing Shank wXrcom
		//MatScalarMul(CrdAll->data+48, 3, -1, rCOMccw+12);	//CrdAll->data+48 軸16右腳knee
		//MatAddAB(rCOMccw+12,pv_stack+12,rCOMccw+12,3);		//pv_stack121314為右小腿重心位置 減掉膝蓋位置後就為向量r
		Cross2Vd(OmegaL+9+AxisJump,rCOMccw+12,LocalVelCOM+9+AxisJump);	

		//Swing Foot wXrcom
		for (int i = 0 ; i < 3 ; i++)							// 直接指定LocalVelCOM303132的值為0 因為和333435共點
			LocalVelCOM[i+12+AxisJump]=0; // r=0

		//MatScalarMul(CrdAll->data+51, 3, -1, rCOMccw+15);		//CrdAll->data+51 軸17右腳ankle
		//MatAddAB(rCOMccw+15,pv_stack+15,rCOMccw+15,3);		//pv_stack151617為右腳板重心位置 減掉腳踝位置後就為向量r
		Cross2Vd(OmegaL+AxisJump+15,rCOMccw+15,LocalVelCOM+AxisJump+15);	

	//Accumulate
		for (int i = 0 ; i < 12 ; i++)
			MatAddAB(VelJ+3*i,LocalVelCOM+3*i,VelCOM+3*i,3);
	}
	else if(selIK == 1)// Left leg lifiting 
	{
	//Vector Omega cross rcom
		//Fixed Foot wXrcom
		for (int i = 0 ; i < 3 ; i++)		//在這裡指定LocalVelCOM333435是為了等等疊加方便
			LocalVelCOM[i+AxisJump+15]=0; //r=0

		//Fixed Shank wXrcom
		//MatScalarMul(CrdAll->data+51, 3, -1, rCOMcw);	//CrdAll->data+51 右腳 ankle
		//MatAddAB(rCOMcw,pv_stack+12,rCOMcw,3);			//pv_stack121314為右小腿重心位置 減掉Ankle位置後就為向量r
		Cross2Vd(OmegaL+AxisJump+12,rCOMcw,LocalVelCOM+AxisJump+12); //小腿Link的Omega為303132 把重心速度存在303132

		//Fixed Teigh wXrcom
		//MatScalarMul(CrdAll->data+48, 3, -1, rCOMcw+3);		//CrdAll->data+48 右腳 knee
		//MatAddAB(rCOMcw+3,pv_stack+9,rCOMcw+3,3);			//pv_stack91011為右小腿重心位置 減掉knee位置後就為向量r
		Cross2Vd(OmegaL+AxisJump+9,rCOMcw+3,LocalVelCOM+AxisJump+9);	//大腿Link的Omega為272829 把重心速度存在272829
		for (int i = 0 ; i < 6 ; i++)		 // LocalVelCOM242526 212223=0  因為和181920共點 所以Locak重心速度直接指定為0
			LocalVelCOM[i+AxisJump+3]=0; // r=0

		//Swing Waist wXrcom
		//for (int i = 0 ; i < 3 ; i++)		//上半身(和腰桿件結合)重心位置減CrdAll->data454647 右腳 hip
		//	rCOMcw[i+6]=BodyRCOM[i]-CrdAll->data[i+DHJump+6];
		Cross2Vd(OmegaL+AxisJump,rCOMcw+6,LocalVelCOM+AxisJump);
		for (int i = 0 ; i < 6 ; i++)		//LocalVelCOM012 345=0 因為和678共點 所以Locak重心速度為0
			LocalVelCOM[i]=0; // r=0

		//Swing Teigh wXrcom
		//MatScalarMul(CrdAll->data+6, 3, -1, rCOMcw+9);	//CrdAll->data+678 左腳hip
		//MatAddAB(rCOMcw+9,pv_stack,rCOMcw+9,3);			//pv_stack012為左大腿重心位置 減掉hip位置後就為向量r
		Cross2Vd(OmegaL+6,rCOMcw+9,LocalVelCOM+6);		//大腿Link的Omega為678 把重心速度存在678

		//Swing Shank wXrcom
		//MatScalarMul(CrdAll->data+9, 3, -1, rCOMcw+12);		//CrdAll->data+91011 左腳knee
		//MatAddAB(rCOMcw+12,pv_stack+3,rCOMcw+12,3);			//pv_stack345為左小腿重心位置 減掉knee位置後就為向量r
		Cross2Vd(OmegaL+9,rCOMcw+12,LocalVelCOM+9);			//小腿Link的Omega為91011 把重心速度存在91011

		//Swing Foot wXrcom
		for (int i = 0 ; i < 3 ; i++)		// 直接指定LocalVelCOM121314的值為0 因為和151617共點
			LocalVelCOM[i+12]=0; // r=0

		//MatScalarMul(CrdAll->data+12, 3, -1, rCOMcw+15);		//CrdAll->data+12 左腳ankel
		//MatAddAB(rCOMcw+15,pv_stack+6,rCOMcw+15,3);			//pv_stack678為左腳板重心位置 減掉腳踝位置後就為向量r
		Cross2Vd(OmegaL+15,rCOMcw+15,LocalVelCOM+15);	
	//Accumulate
		for (int i = 0 ; i < 12 ; i++)
			MatAddAB(VelJ+3*i,LocalVelCOM+3*i,VelCOM+3*i,3);
	}

	// 以下測試用

		//fstream Fx;

		//Fx.open("TestCOMPosFootL.txt",ios::app);
		//	Fx << pv_stack[6] <<"\t" << pv_stack[7] <<"\t"<< pv_stack[8]<<  endl;
		//Fx.close();
		//
		//Fx.open("TestCOMPosFootR.txt",ios::app);
		//	Fx << pv_stack[15] <<"\t" << pv_stack[16] <<"\t"<< pv_stack[17]<<  endl;
		//Fx.close();
	
		//Fx.open("TestCOMPosShankL.txt",ios::app);
		//	Fx << pv_stack[3] <<"\t" << pv_stack[4] <<"\t"<< pv_stack[5]<<  endl;
		//Fx.close();
		//
		//Fx.open("TestCOMPosShankR.txt",ios::app);
		//	Fx << pv_stack[12] <<"\t" << pv_stack[13] <<"\t"<< pv_stack[14]<<  endl;
		//Fx.close();
		//
		//Fx.open("TestCOMPosThighL.txt",ios::app);
		//	Fx << pv_stack[0] <<"\t" << pv_stack[1] <<"\t"<< pv_stack[2]<<  endl;
		//Fx.close();
		//
		//Fx.open("TestCOMPosThighR.txt",ios::app);
		//	Fx << pv_stack[9] <<"\t" << pv_stack[10] <<"\t"<< pv_stack[11]<<  endl;
		//Fx.close();
		//
		//Fx.open("TestCOMPosWaist.txt",ios::app);
		//	Fx << BodyRCOM[0] <<"\t" << BodyRCOM[1] <<"\t"<< BodyRCOM[2]<<  endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestVelCOMFootL.txt",ios::app);		
		//if(selIK == 1)//(右腳support)
		//Fx << VelCOM[15] << "\t"<< VelCOM[16] << "\t"<< VelCOM[17]<< endl;
		//else if(selIK == 0||selIK==2) //左腳support
		//Fx << 0 << "\t"<< 0 << "\t"<< 0 << endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestVelCOMFootR.txt",ios::app);		
		//if(selIK == 1)//(右腳support)
		//Fx << 0 << "\t"<< 0 << "\t"<< 0 << endl;
		//else if(selIK == 0||selIK==2) //左腳support
		//Fx << VelCOM[33] << "\t"<< VelCOM[34] << "\t"<< VelCOM[35] << endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestVelCOMShankL.txt",ios::app);		
		//if(selIK == 1)//(右腳support)
		//Fx << VelCOM[9] << "\t"<< VelCOM[10] << "\t"<< VelCOM[11]<< endl;
		//else if(selIK == 0||selIK==2) //左腳support
		//Fx << VelCOM[12] << "\t"<< VelCOM[13] << "\t"<< VelCOM[14]<< endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestVelCOMShankR.txt",ios::app);		
		//if(selIK == 1)//(右腳support)
		//Fx << VelCOM[30] << "\t"<< VelCOM[31] << "\t"<< VelCOM[32]<< endl;
		//else if(selIK == 0||selIK==2) //左腳support
		//Fx << VelCOM[27] << "\t"<< VelCOM[28] << "\t"<< VelCOM[29]<< endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestVelCOMWaist.txt",ios::app);		
		//if(selIK == 1)//(右腳support)
		//Fx << VelCOM[18] << "\t"<< VelCOM[19] << "\t"<< VelCOM[20]<< endl;
		//else if(selIK == 0||selIK==2) //左腳support
		//Fx << VelCOM[0] << "\t"<< VelCOM[1] << "\t"<< VelCOM[2]<< endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestVelCOMThighL.txt",ios::app);		
		//if(selIK == 1)//(右腳support)
		//Fx << VelCOM[6] << "\t"<< VelCOM[7] << "\t"<< VelCOM[8]<< endl;
		//else if(selIK == 0||selIK==2) //左腳support
		//Fx << VelCOM[9] << "\t"<< VelCOM[10] << "\t"<< VelCOM[11]<< endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestVelCOMThighR.txt",ios::app);		
		//if(selIK == 1)//(右腳support)
		//Fx << VelCOM[27] << "\t"<< VelCOM[28] << "\t"<< VelCOM[29]<< endl;
		//else if(selIK == 0||selIK==2) //左腳support
		//Fx << VelCOM[24] << "\t"<< VelCOM[25] << "\t"<< VelCOM[26]<< endl;
		//Fx.close();


}
void Kine::FDAccelCOM(void) 
{
	/******************************************************************
	input: void
	output: void

	Note:
	// Find Link COM Accel   
	// 請注意Index排列方式
	// 2012 Slongz Start
	// 20121206 WeiZh Start 20121217 改COM位置向量 rCOMccw rCOMcw
	******************************************************************/
	if (selIK==2||selIK == 0)// Double Support & Right leg lifiting
	{
	//OmegaL cross LocalVelCOM VectorTemp1 因為在前面都已經按照自己的Index排列好了 自己的Omega和自己的LocalVelCOM cross 存到自己的Index裡 所以左右腳可以一起
		//Fixed Leg 和 Swing Leg
		  for (int i = 0 ; i < 11 ; i++ )
			 Cross2Vd(OmegaL+i*3, LocalVelCOM+3*i, VectorTemp1+3*i);
	//AlphaJ cross r
		//Fixed Foot AXr
		for (int i = 0 ; i < 3 ; i++)		//在這裡指定VectorTemp2 151617是為了等等疊加方便
			VectorTemp2[i+15] = 0;			//Fixed on ground

		//Fixed Shank AXr
		Cross2Vd(AlphaL+12,rCOMccw, VectorTemp2+12);	//小腿Link的Alpha為121314 把重心切線加速度存在121314

		//Fixed Teight AXr
		Cross2Vd(AlphaL+9 ,rCOMccw+3,VectorTemp2+9);		//大腿Link的Alpha為91011 把重心加速度存在91011
		for (int i = 0 ; i < 6 ; i++)		//VectorTemp2 345 678=0  因為和012共點 所以Locak重心切線加速度直接指定為0
			VectorTemp2[i+3] = 0; // r=0

		//Swing Body(Waist) AXr 
		Cross2Vd(AlphaL,rCOMccw+6,VectorTemp2);
		for (int i = 0 ; i < 6 ; i++)		//VectorTemp2 181920 212223=0 因為和242526共點 所以Locak重心切線加速度為0
			VectorTemp2[i+AxisJump]=0; // r=0

		//Swing Teight AXr
		Cross2Vd(AlphaL+6+AxisJump,rCOMccw+9,VectorTemp2+6+AxisJump);

		//Swing Shank AXr
		Cross2Vd(AlphaL+9+AxisJump,rCOMccw+12,VectorTemp2+9+AxisJump);	

		//Swing Foot AXr
		for (int i = 0 ; i < 3 ; i++)							// 直接指定VectorTemp2 303132的值為0 因為和333435共點
			VectorTemp2[i+12+AxisJump]=0; // r=0
		Cross2Vd(AlphaL+15+AxisJump,rCOMccw+15,VectorTemp2+15+AxisJump);	

	//Accumalate 因為Index配合 可以全部一起加
		for (int i = 0 ; i < 12 ; i++)
		{
			MatAddAB(VectorTemp1+3*i,VectorTemp2+3*i,AccelCOM+3*i,3);
			MatAddAB(AccelJ+3*i,AccelCOM+3*i,AccelCOM+3*i,3);
		}
	}
	else if(selIK == 1)// Left leg lifiting 
	{
	//OmegaL cross LocalVelCOM VectorTemp1 因為在前面都已經按照自己的Indexw排列好了 自己的Omega和自己的LocalVelCOM cross 存到自己的Index裡 所以左右腳可以一起
		//Fixed Leg 和 Swing Leg
		  for (int i = 0 ; i < 11 ; i++ )
			 Cross2Vd(OmegaL+i*3, LocalVelCOM+3*i, VectorTemp1+3*i);

	//AlphaJ cross r
		//Fixed Foot AXr
		for (int i = 0 ; i < 3 ; i++)	  //在這裡指定VectorTemp2 333435是為了等等疊加方便
			VectorTemp2[i+AxisJump+15]=0; //Fixed on ground

		//Fixed Shank AXr
		Cross2Vd(AlphaL+AxisJump+12,rCOMcw, VectorTemp2+AxisJump+12);	//右小腿Link的Alpha為303132 把重心切線加速度存在303132

		//Fixed Teight AXr
		Cross2Vd(AlphaL+AxisJump+9 ,rCOMcw+3,VectorTemp2+AxisJump+9);	//右大腿Link的Alpha為272829 把重心切線加速度存在272829
		for (int i = 0 ; i < 6 ; i++)	//VectorTemp2 212223 242526=0  因為和181920共點 所以Locak重心切線加速度直接指定為0
			VectorTemp2[i+3+AxisJump]=0; // r=0

		//Swing Body(Waist) AXr 
		Cross2Vd(AlphaL+AxisJump,rCOMcw+6,VectorTemp2+AxisJump);
		for (int i = 0 ; i < 6 ; i++)		//VectorTemp2 012 345=0 因為和678共點 所以Locak重心切線加速度為0
			VectorTemp2[i]=0; // r=0

		//Swing Teight AXr
		Cross2Vd(AlphaL+6,rCOMcw+9,VectorTemp2+6);			//左大腿Link的Alpha為678 把重心切線加速度存在678

		//Swing Shank AXr
		Cross2Vd(AlphaL+9,rCOMcw+12,VectorTemp2+9);			//左小腿Link的Alpha為91011 把重心切線加速度存在91011

		//Swing Foot AXr
		for (int i = 0 ; i < 3 ; i++)							// 直接指定VectorTemp2 121314的值為0 因為和151617共點
			VectorTemp2[i+12]=0; // r=0
		Cross2Vd(AlphaL+15,rCOMcw+15,VectorTemp2+15);	

	//Accumalate 因為Index配合 可以全部一起加
		for (int i = 0 ; i < 12 ; i++)
		{
			MatAddAB(VectorTemp1+3*i,VectorTemp2+3*i,AccelCOM+3*i,3);
			MatAddAB(AccelJ+3*i,AccelCOM+3*i,AccelCOM+3*i,3);
		}
	}

		// 以下測試用

		//fstream Fx;
		//Fx.open("TestAccelCOMFootL.txt",ios::app);		
		//if(selIK == 1)//(右腳support)
		//Fx << AccelCOM[15] << "\t"<< AccelCOM[16] << "\t"<< AccelCOM[17]<< endl;
		//else if(selIK == 0||selIK==2) //左腳support
		//Fx << 0 << "\t"<< 0 << "\t"<< 0 << endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestAccelCOMFootR.txt",ios::app);		
		//if(selIK == 1)//(右腳support)
		//Fx << 0 << "\t"<< 0 << "\t"<< 0 << endl;
		//else if(selIK == 0||selIK==2) //左腳support
		//Fx << AccelCOM[33] << "\t"<< AccelCOM[34] << "\t"<< AccelCOM[35] << endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestAccelCOMShankL.txt",ios::app);		
		//if(selIK == 1)//(右腳support)
		//Fx << AccelCOM[9] << "\t"<< AccelCOM[10] << "\t"<< AccelCOM[11]<< endl;
		//else if(selIK == 0||selIK==2) //左腳support
		//Fx << AccelCOM[12] << "\t"<< AccelCOM[13] << "\t"<< AccelCOM[14]<< endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestAccelCOMShankR.txt",ios::app);		
		//if(selIK == 1)//(右腳support)
		//Fx << AccelCOM[30] << "\t"<< AccelCOM[31] << "\t"<< AccelCOM[32]<< endl;
		//else if(selIK == 0||selIK==2) //左腳support
		//Fx << AccelCOM[27] << "\t"<< AccelCOM[28] << "\t"<< AccelCOM[29]<< endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestAccelCOMWaist.txt",ios::app);		
		//if(selIK == 1)//(右腳support)
		//Fx << AccelCOM[18] << "\t"<< AccelCOM[19] << "\t"<< AccelCOM[20]<< endl;
		//else if(selIK == 0||selIK==2) //左腳support
		//Fx << AccelCOM[0] << "\t"<< AccelCOM[1] << "\t"<< AccelCOM[2]<< endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestAccelCOMThighL.txt",ios::app);		
		//if(selIK == 1)//(右腳support)
		//Fx << AccelCOM[6] << "\t"<< AccelCOM[7] << "\t"<< AccelCOM[8]<< endl;
		//else if(selIK == 0||selIK==2) //左腳support
		//Fx << AccelCOM[9] << "\t"<< AccelCOM[10] << "\t"<< AccelCOM[11]<< endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestAccelCOMThighR.txt",ios::app);		
		//if(selIK == 1)//(右腳support)
		//Fx << AccelCOM[27] << "\t"<< AccelCOM[28] << "\t"<< AccelCOM[29]<< endl;
		//else if(selIK == 0||selIK==2) //左腳support
		//Fx << AccelCOM[24] << "\t"<< AccelCOM[25] << "\t"<< AccelCOM[26]<< endl;
		//Fx.close();
}
void Kine::FDForceJ(void) 
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 力規值須再校正~!!!!
	
	// Find Joint Force 在此計算所有link質心受到質心加速度(AccelCOM)時 兩邊的Joint所需施的力
	// 計算時須注意Index的分配 在此模型已經大量簡化 請參照catia所量得之質量 軸共點的質量皆由最靠近末端點的軸來做代表
	// 其餘共軸的m一律設為0 又以最靠近末端點的軸帶表示因為這樣質心加速度(AccelCOM) 的Index才會正確
	// 在此運用到力規的資訊 和上面一樣分成左腳或是右腳support所以不用再另外分出腳尚未離地時的狀況 
	// 但在support腳 腳板的質量要再做新的Index 目前設為零 使地面反作用力(力規值) 等同於最後一軸的力   <---檢查用

	// 目前是以support腳的力規反推回來 為了減小累績誤差的影響 從support開始算比較好
	// 特別注意 在此計算力的座標仍和之前算角度 速度的座標一樣 同為 世界下 座標
	// 因為力的向量可以延伸 以世界座標算完之後等等再算Torque在一起轉就好
	// 2012 Slongz Start
	// 20121213 Wei-Zh Lai Start 
	******************************************************************/
	double ForceCOM[3];
	double Gravity[3];

	//double TempVectorL[3];
	//double TempVectorR[3];
	
	//double FRatioL;
	//double FRatioR;
	//double tempTotal=0;
	//double TempForce[3];
	//double FSensor_forcL[3] ;
	//double FSensor_forcR[3] ;

		//MatScalarMul(GravityZaxi, 3, GravityConst, GravityZaxi); // g*[0 0 -1]
/////////////////////////////////////////////////////////////////注意! 以下為SlongZ分配DSP的ZMP//////////////////////////////////////////////////////////////
//if(DSPFlag==1 || selIK==2) //Checked    //selIK = 2; // double support phas  這個狀態是依舊在DSP下但是已經準備要抬腳了 所以要藉ZMP做雙腳的力的分配
//{
//		//權重 Checked
//		MatScalarMul(CrdAll->data+12, 3, 1.0, TempVectorL );		//CrdAll->data+12 左腳ankle
//		MatScalarMul(CrdAll->data+51, 3, 1.0, TempVectorR );		//CrdAll->data+51 右腳ankle
//		TempVectorL[0]=TempVectorL[0]-ZMPcatch[0];
//		TempVectorL[1]=TempVectorL[1]-ZMPcatch[1];
//		TempVectorR[0]=TempVectorR[0]-ZMPcatch[0];
//		TempVectorR[1]=TempVectorR[1]-ZMPcatch[1];
//
//		tempTotal=sqrt(TempVectorL[0]*TempVectorL[0]+TempVectorL[1]*TempVectorL[1])+sqrt(TempVectorR[0]*TempVectorR[0]+TempVectorR[1]*TempVectorR[1]);
//		FRatioR=sqrt(TempVectorL[0]*TempVectorL[0]+TempVectorL[1]*TempVectorL[1]) / tempTotal;
//		FRatioL=sqrt(TempVectorR[0]*TempVectorR[0]+TempVectorR[1]*TempVectorR[1]) / tempTotal;
//		//cout<<"FRatioL "<<"\t"<<(FRatioL)<<"\t"<<"FRatioR  "<<"\t"<<(FRatioR)<<"\n";
//		//cout<<(tempTotal)<<"\n";
//		//Sleep(1);
//
//		TempZaxi[0]=0;
//		TempZaxi[1]=0;
//		TempZaxi[2]=-1;
//		MatScalarMul(TempZaxi, 3, GravityConst, TempZaxi); // g*[0 0 -1]
//
//		MatScalarMul(AccelTotalCOM, 3, SumMass, ForceTotalCOM ); //Total Ma			
//		MatScalarMul(TempZaxi, 3, SumMass, TempPvStack); //Total Mg
//		MatAddAB(TempPvStack,ForceTotalCOM , TempForce, 3); //Total Mg+Ma = GRF (Ground Reaction Force Downward
//		MatScalarMul(TempForce, 3, -1.0, TempForce); // GRF*-1 Upward
//		MatScalarMul(TempForce, 3, FRatioR, TempForce); // GRF*-1 Upward
//
//		// Single support Swing leg
//
//			//Swing Foot
//		MatScalarMul(TempZaxi, 3, mass_com+5, TempPvStack); //m*g
//		MatScalarMul(AccelCOM+15+AxisJump, 3, mass_com+5, TempVector); //m*a
//		MatScalarMul(TempVector, 3, -1.0, TempVector);//-m*a
//
//		MatAddAB(TempVector,TempPvStack,ForceJ+15+AxisJump,3);//0+mg-ma	
//		MatAddAB(ForceJ+15+AxisJump,TempForce,ForceJ+15+AxisJump,3);//0+mg-ma	
//		for (int i = 0 ; i < 3 ; i++)
//			ForceJ[i+12+AxisJump]=ForceJ[i+15+AxisJump];
//
//			//Swing Shank
//		MatScalarMul(TempZaxi, 3, mass_com+1+3, TempPvStack); //m*g
//
//		MatScalarMul(AccelCOM+9+AxisJump, 3, mass_com+1+3, TempVector); //m*a
//		MatScalarMul(TempVector, 3, -1.0, TempVector);//-m*a
//
//		MatAddAB(TempPvStack,TempVector ,ForceJ+9+AxisJump,3);//0+mg-ma		
//		MatAddAB(ForceJ+9+AxisJump,ForceJ+12+AxisJump ,ForceJ+9+AxisJump,3);//fi+mg-ma
//
//			//Swing Teight
//		MatScalarMul(TempZaxi, 3, mass_com+3, TempPvStack); //m*g
//
//		MatScalarMul(AccelCOM+6+AxisJump, 3, mass_com+3, TempVector); //m*a
//		MatScalarMul(TempVector, 3, -1.0, TempVector);//-m*a
//
//		MatAddAB(TempPvStack,TempVector ,ForceJ+6+AxisJump,3);//0+mg-ma		
//		MatAddAB(ForceJ+6+AxisJump,ForceJ+9+AxisJump ,ForceJ+6+AxisJump,3);//fi+mg-ma
//		for (int i = 0 ; i < 3 ; i++)
//		{
//			ForceJ[i+AxisJump]=ForceJ[i+6+AxisJump];
//			ForceJ[i+3+AxisJump]=ForceJ[i+6+AxisJump];
//		}
//
//			//Body 
//		MatScalarMul(TempZaxi, 3, BodyW, TempPvStack); //m*g
//
//		MatScalarMul(AccelCOM+AxisJump, 3, BodyW, TempVector); //m*a
//		MatScalarMul(TempVector, 3, -1.0, TempVector);//-m*a
//
//		MatAddAB(TempPvStack,TempVector ,ForceJ,3);//0+mg-ma		
//		MatAddAB(ForceJ+AxisJump,ForceJ ,ForceJ,3);//fi+mg-ma
//		for (int i = 0 ; i < 3 ; i++)
//		{
//			ForceJ[i+3]=ForceJ[i];
//			ForceJ[i+6]=ForceJ[i];
//		}
//
//			//Fixed Teight
//		MatScalarMul(TempZaxi, 3, mass_com, TempPvStack); //m*g
//
//		MatScalarMul(AccelCOM+9, 3, mass_com, TempVector); //m*a
//		MatScalarMul(TempVector, 3, -1.0, TempVector);//-m*a
//
//		MatAddAB(TempPvStack,TempVector ,ForceJ+9,3);//0+mg-ma		
//		MatAddAB(ForceJ+9,ForceJ+6 ,ForceJ+9,3);//fi+mg-ma
//
//			//Fixed Shank
//		MatScalarMul(TempZaxi, 3, mass_com+1, TempPvStack); //m*g
//		
//		MatScalarMul(AccelCOM+12, 3, mass_com+1, TempVector); //m*a
//		MatScalarMul(TempVector, 3, -1.0, TempVector);//-m*a
//
//		MatAddAB(TempPvStack,TempVector ,ForceJ+12,3);//0+mg-ma		
//		MatAddAB(ForceJ+12,ForceJ+9 ,ForceJ+12,3);//fi+mg-ma
//
//		for (int i = 0 ; i < 3 ; i++)
// 			ForceJ[i+15]=ForceJ[i+12];
//
//	//cout<<abs(TempVectorL[0])<<"\t"<<"TempVectorL x"<<"\t"<<abs(TempVectorR[0])<<"\t"<<"TempVectorR x"<<"\n";
//	//cout<<(TempVectorL[1])<<"\t"<<"TempVectorL y"<<"\t"<<(TempVectorR[1])<<"\t"<<"TempVectorR y"<<"\n";
//
//	//cout<<"FL "<<"\t"<<ForceJ[17]<<"\t"<<"FR "<<"\t"<<ForceJ[17+18]<<"\t"<<"QQ"<<"\n" ;
//	//Sleep(100);
//}
//
//else // single support phase
//{
/////////////////////////////////////////////////////////////////注意! 以上為SlongZ分配DSP的ZMP//////////////////////////////////////////////////////////////	
	

////////////////////////////////////////注意! 以下在此是從Fixed腳的力規值往Swing腳推算Joint的力//////////////////////////////////////////////////////////////
	
	//// 注意! 在此是從Fixed腳的力規值往Swing腳推算Joint的力
	//if(selIK==2||selIK == 0)	// Right leg lifiting 
	//{
	//	// Single support leg

	//		//Fixed Foot
	//	MatScalarMul(GravityZaxi, 3, mass_com+2, Gravity);	// m151617 * g
	//	MatScalarMul(AccelCOM+15, 3, mass_com+2, ForceCOM);	// ForceCOM151617 = m151617 * ac151617
	//	MatScalarMul(ForceCOM, 3, -1.0, ForceCOM);			// -ForceCOM151617 = -m151617 * ac151617
	//	MatAddAB(ForceCOM,Gravity,ForceJ+15,3);				// -ma + mg
	//	MatAddAB(ForceJ+15,FSensor_forcL,ForceJ+15,3);	// ForceSensor_forceL - ForceJ151617 + m151617 * g =  ForceCOM151617
	//	
	//	//////////////////////////////////////////////////////////////////////////////////////////
	//	// 其實若質量每個都有量 在此之後(121314~012)可利用for迴圈一次算完 但因為現在模型省略沒有 所以只能慢慢算 再用指定的
	//	// 要再檢查!
	//	//for (int i = 4 ; i >= 0 ; i--)
	//	//{
	//	//	MatScalarMul(GravityZaxi, 3, mass_com+3*i, Gravity);	// i = 5		// m121314 * g
	//	//	MatScalarMul(AccelCOM+3*i, 3, mass_com+3*i, ForceCOM);	// ForceCOM121314 = m121314 * ac121314
	//	//	MatScalarMul(ForceCOM, 3, -1.0, ForceCOM);				// -ForceCOM121314 = -m121314 * ac121314
	//	//	MatAddAB(ForceCOM,Gravity,ForceJ+3*i,3);				// -ma + mg
	//	//	MatAddAB(ForceJ+3*i,ForceJ+3*(i+1),ForceJ+3*i,3);		// ForceJ151617 - ForceJ121314 + mg =  ForceCOM121314
	//	//}
	//	//////////////////////////////////////////////////////////////////////////////////////////

	//	for (int i = 0 ; i < 3 ; i++)	// 因為151617 121314 軸共點 所以直接指定兩個力相等
	//		ForceJ[i+12]=ForceJ[i+15];

	//		//Fixed Shank
	//	MatScalarMul(GravityZaxi, 3, mass_com+1, Gravity);	// m91011 * g
	//	MatScalarMul(AccelCOM+9, 3, mass_com+1, ForceCOM);	// ForceCOM91011 = m91011 * ac91011
	//	MatScalarMul(ForceCOM, 3, -1.0, ForceCOM);			// -ForceCOM91011 = -m91011 * ac91011
	//	MatAddAB(ForceCOM,Gravity,ForceJ+9,3);				// -ma + mg
	//	MatAddAB(ForceJ+9,ForceJ+12,ForceJ+9,3);			// ForceJ121314 - ForceJ91011 + m91011 * g =  ForceCOM91011

	//		//Fixed Teight
	//	MatScalarMul(GravityZaxi, 3, mass_com, Gravity);	// m678 * g
	//	MatScalarMul(AccelCOM+6, 3, mass_com, ForceCOM);	// ForceCOM678 = m678 * ac678
	//	MatScalarMul(ForceCOM, 3, -1.0, ForceCOM);			// -ForceCOM678 = -m678 * ac678
	//	MatAddAB(ForceCOM,Gravity,ForceJ+6,3);				// -ma + mg
	//	MatAddAB(ForceJ+6,ForceJ+9,ForceJ+6,3);				// ForceJ91011 - ForceJ678 + m678 * g =  ForceCOM678
	//	
	//	for (int i = 0 ; i < 3 ; i++)	//直接指定共點軸的力 ForceJ345 = ForceJ012 = ForceJ678
	//	{
	//		ForceJ[i]=ForceJ[i+6];
	//		ForceJ[i+3]=ForceJ[i+6];
	//	}

	//		//Body 
	//	MatScalarMul(GravityZaxi, 3, BodyW, Gravity);		// m181920 * g
	//	MatScalarMul(AccelCOM+AxisJump, 3, BodyW, ForceCOM);// ForceCOM181920 = m181920 * ac181920
	//	MatScalarMul(ForceCOM, 3, -1.0, ForceCOM);			// -ForceCOM181920 = -m181920 * ac181920
	//	MatAddAB(ForceCOM,Gravity,ForceJ+AxisJump,3);		// -ma + mg
	//	MatAddAB(ForceJ+AxisJump,ForceJ,ForceJ+AxisJump,3);	// ForceJ012 - ForceJ181920 + m181920 * g =  ForceCOM181920
	//	
	//	//////////////////////////////////////////////////////////////////////////////////////////
	//	// 其實若質量每個都有量 在此之後(212223~333435)可利用for迴圈一次算完 但因為現在模型省略沒有 所以只能慢慢算 再用指定的
	//	// 要再檢查!
	//	//for (int i = 1 ; i <= 5 ; i++)
	//	//{
	//	//	MatScalarMul(GravityZaxi, 3, mass_com+3*i+AxisJump, Gravity);						// i = 1		// m212223 * g
	//	//	MatScalarMul(AccelCOM+3*i+AxisJump, 3, mass_com+3*i+AxisJump, ForceCOM);			// ForceCOM212223 = m212223 * ac212223
	//	//	MatScalarMul(ForceCOM, 3, -1.0, ForceCOM);											// -ForceCOM212223 = -m212223 * ac212223
	//	//	MatAddAB(ForceCOM,Gravity,ForceJ+3*i+AxisJump,3);									// -ma + mg
	//	//	MatAddAB(ForceJ+3*i+AxisJump,ForceJ+3*(i-1)+AxisJump,ForceJ+3*i+AxisJump,3);		// ForceJ181920 - ForceJ212223 + mg =  ForceCOM212223
	//	//}
	//	//////////////////////////////////////////////////////////////////////////////////////////

	//	for (int i = 0 ; i < 3 ; i++)		//直接指定共點軸的力 ForceJ212223 = ForceJ242526 = ForceJ181920
	//	{
	//		ForceJ[i+3+AxisJump]=ForceJ[i+AxisJump];
	//		ForceJ[i+6+AxisJump]=ForceJ[i+AxisJump];
	//	}

	//		//Swing Teight
	//	MatScalarMul(GravityZaxi, 3, mass_com+3, Gravity);						// m272829 * g
	//	MatScalarMul(AccelCOM+9+AxisJump, 3, mass_com+3, ForceCOM);				// ForceCOM272829 = m272829 * ac272829
	//	MatScalarMul(ForceCOM, 3, -1.0, ForceCOM);								// -ForceCOM272829 = -m272829 * ac272829
	//	MatAddAB(ForceCOM, Gravity, ForceJ+9+AxisJump, 3);						// -ma + mg		
	//	MatAddAB(ForceJ+9+AxisJump, ForceJ+6+AxisJump , ForceJ+9+AxisJump, 3);	// ForceJ242526 - ForceJ272829 + m272829 * g =  m272829 * ac272829 = ForceCOM272829

	//		//Swing Shank
	//	MatScalarMul(GravityZaxi, 3, mass_com+4, Gravity);						// m303132 * g
	//	MatScalarMul(AccelCOM+12+AxisJump, 3, mass_com+4, ForceCOM);			// ForceCOM303132 = m303132 * ac303132
	//	MatScalarMul(ForceCOM, 3, -1.0, ForceCOM);								// -ForceCOM303132 = -m303132 * ac303132
	//	MatAddAB(ForceCOM, Gravity, ForceJ+12+AxisJump, 3);						// -ma + mg		
	//	MatAddAB(ForceJ+12+AxisJump, ForceJ+9+AxisJump , ForceJ+12+AxisJump, 3);	// ForceJ272829 - ForceJ303132 + m303132 * g =  m303132 * ac303132 = ForceCOM303132

	//	for (int i = 0 ; i < 3 ; i++)		//直接指定共點軸的力 ForceJ333435 = ForceJ303132 
	//		ForceJ[i+15+AxisJump]=ForceJ[i+12+AxisJump];

	//}
	//else if(selIK == 1 )	// Left leg lifiting 
	//{		
	//	// Single support Swing leg

	//		//Fixed Foot
	//	MatScalarMul(GravityZaxi, 3, mass_com+5, Gravity);				// m333435 * g
	//	MatScalarMul(AccelCOM+15+AxisJump, 3, mass_com+5, ForceCOM);	// ForceCOM333435 = m333435 * ac333435
	//	MatScalarMul(ForceCOM, 3, -1.0, ForceCOM);						// -ForceCOM333435 = -m333435 * ac333435
	//	MatAddAB(ForceCOM,Gravity,ForceJ+15+AxisJump,3);				// ForceJ333435 = -ma + mg
	//	MatAddAB(ForceJ+15+AxisJump,FSensor_forcR,ForceJ+15+AxisJump,3);// ForceSensor_forceR - ForceJ333435 + mg =  ForceCOM333435

	//	//////////////////////////////////////////////////////////////////////////////////////////
	//	// 其實若質量每個都有量 在此之後(303132~181920)可利用for迴圈一次算完 但因為現在模型省略沒有 所以只能慢慢算 再用指定的
	//	// 要再檢查!
	//	//for (int i = 4 ; i >= 0 ; i--)
	//	//{
	//	//	MatScalarMul(GravityZaxi, 3, mass_com+AxisJump+3*i, Gravity);				// i = 4		// m303132 * g
	//	//	MatScalarMul(AccelCOM+3*i+AxisJump, 3, mass_com+AxisJump+3*i, ForceCOM);	// ForceCOM303132 = m303132 * ac303132
	//	//	MatScalarMul(ForceCOM, 3, -1.0, ForceCOM);									// -ForceCOM303132 = -m303132 * ac303132
	//	//	MatAddAB(ForceCOM,Gravity,ForceJ+3*i+AxisJump,3);							// -ma + mg
	//	//	MatAddAB(ForceJ+3*i+AxisJump,ForceJ+3*(i+1)+AxisJump,ForceJ+3*i+AxisJump,3);		// ForceJ333435 - ForceJ303132 + mg =  ForceCOM303132	
	//	//}
	//	//////////////////////////////////////////////////////////////////////////////////////////

	//	for (int i = 0 ; i < 3 ; i++)		// 因為333435 303132 軸共點 所以直接指定兩個力相等
	//		ForceJ[i+12+AxisJump]=ForceJ[i+15+AxisJump];

	//		//Fixed Shank
	//	MatScalarMul(GravityZaxi, 3, mass_com+4, Gravity);			// m272829 * g
	//	MatScalarMul(AccelCOM+9+AxisJump, 3, mass_com+4, ForceCOM); // ForceCOM272829 = m272829 * ac272829
	//	MatScalarMul(ForceCOM, 3, -1.0, ForceCOM);					// -ForceCOM272829 = -m272829 * ac272829
	//	MatAddAB(ForceCOM, Gravity, ForceJ+9+AxisJump, 3);			// -ma + mg		
	//	MatAddAB(ForceJ+9+AxisJump, ForceJ+12+AxisJump , ForceJ+9+AxisJump, 3);	// ForceJ(i) - ForceJ(i-1) + m(i-1) * g =  m(i-1) * ac(i-1) = ForceCOM(i-1)

	//		//Fixed Teight
	//	MatScalarMul(GravityZaxi, 3, mass_com+3, Gravity);			// m242526 * g
	//	MatScalarMul(AccelCOM+6+AxisJump, 3, mass_com+3, ForceCOM);	// ForceCOM242526 = m242526 * ac242526
	//	MatScalarMul(ForceCOM, 3, -1.0, ForceCOM);					// -ForceCOM242526 = -m242526 * ac242526
	//	MatAddAB(ForceCOM,Gravity ,ForceJ+6+AxisJump,3);			// -ma + mg		
	//	MatAddAB(ForceJ+6+AxisJump,ForceJ+9+AxisJump ,ForceJ+6+AxisJump,3);		// ForceJ272829 - ForceJ242526 + m242526 * g =  m242526 * ac242526 = ForceCOM242526
	//	
	//	for (int i = 0 ; i < 3 ; i++)		//直接指定共點軸的力 ForceJ242526 = ForceJ212223 = ForceJ181920
	//	{
	//		ForceJ[i+AxisJump]   = ForceJ[i+6+AxisJump];
	//		ForceJ[i+3+AxisJump] = ForceJ[i+6+AxisJump];
	//	}

	//		//Body (waist)
	//	MatScalarMul(GravityZaxi, 3, BodyW, Gravity);	// m012 * g
	//	MatScalarMul(AccelCOM, 3, BodyW, ForceCOM);		// ForceCOM012 = m012 * a012
	//	MatScalarMul(ForceCOM, 3, -1.0, ForceCOM);		// -ForceCOM012 = -m * a
	//	MatAddAB(ForceCOM,Gravity ,ForceJ,3);			// -ma + mg		
	//	MatAddAB(ForceJ, ForceJ+AxisJump,ForceJ,3);		// ForceJ181920 - ForceJ012 + m012 * g =  m012 * ac012 = ForceCOM012
	//	
	//	//////////////////////////////////////////////////////////////////////////////////////////
	//	// 其實若質量每個都有量 在此之後(345~151617)可利用for迴圈一次算完 但因為現在模型省略沒有 所以只能慢慢算 再用指定的
	//	// 要再檢查!
	//	//for (int i = 1 ; i <= 5 ; i++)
	//	//{
	//	//	MatScalarMul(GravityZaxi, 3, mass_com+3*i, Gravity);	// i = 0		// m345 * g
	//	//	MatScalarMul(AccelCOM+3*i, 3, mass_com+3*i, ForceCOM);	// ForceCOM345 = m345 * ac345
	//	//	MatScalarMul(ForceCOM, 3, -1.0, ForceCOM);				// -ForceCOM345 = -m345 * ac345
	//	//	MatAddAB(ForceCOM,Gravity,ForceJ+3*i,3);				// -ma + mg
	//	//	MatAddAB(ForceJ+3*i,ForceJ+3*(i-1),ForceJ+3*i,3);		// ForceJ012 - ForceJ345 + mg =  ForceCOM345
	//	//}
	//	//////////////////////////////////////////////////////////////////////////////////////////

	//	for (int i = 0 ; i < 3 ; i++)		//直接指定共點軸的力 ForceJ012 = ForceJ345 = ForceJ678
	//	{
	//		ForceJ[i+3]=ForceJ[i];
	//		ForceJ[i+6]=ForceJ[i];
	//	}

	//		//Swing Teight
	//	MatScalarMul(GravityZaxi, 3, mass_com, Gravity);// m91011 * g
	//	MatScalarMul(AccelCOM+9, 3, mass_com, ForceCOM);// ForceCOM91011 = m91011 * ac91011
	//	MatScalarMul(ForceCOM, 3, -1.0, ForceCOM);		// -ForceCOM91011 = -m91011 * ac91011
	//	MatAddAB(ForceCOM,Gravity ,ForceJ+9,3);			// -ma + mg		
	//	MatAddAB(ForceJ+9, ForceJ+6, ForceJ+9,3);		// ForceJ678 - ForceJ91011 + m91011 * g =  m91011 * ac91011 = ForceCOM91011

	//		//Swing Shank
	//	MatScalarMul(GravityZaxi, 3, mass_com+1, Gravity);	// m121314 * g
	//	MatScalarMul(AccelCOM+12, 3, mass_com+1, ForceCOM);	// ForceCOM121314 = m121314 * ac121314
	//	MatScalarMul(ForceCOM, 3, -1.0, ForceCOM);			// -ForceCOM121314 = -m121314 * ac121314
	//	MatAddAB(ForceCOM,Gravity ,ForceJ+12,3);			// -ma + mg		
	//	MatAddAB(ForceJ+12, ForceJ+9, ForceJ+12,3);			// ForceJ91011 - ForceJ121314 + m121314 * g =  m121314 * ac121314 = ForceCOM121314

	//	for (int i = 0 ; i < 3 ; i++)	// 直接指定共點軸的力 ForceJ151617 = ForceJ121314 
	//		ForceJ[i+15] = ForceJ[i+12];

	//	for( int i = 0 ; i < 36 ; i++)	// 因為自由體圖的原因 令其中一邊全部加負號在Matlab上"人"看起來才是正確的 電腦計算則不用 因為哪個mode會配合自己的正負
	//		ForceJ[i] = -ForceJ[i];
	//}
////////////////////////////////////////注意! 以上在此是從Fixed腳的力規值往Swing腳推算Joint的力//////////////////////////////////////////////////////////////



////////////////////////////////////////注意! 以下在此是從swing腳的力規值往Fixed腳推算Joint的力//////////////////////////////////////////////////////////////
	if(selIK == 0 ||selIK == 2)	// Right leg lifiting & double support
	{
		// Single support Swing leg

			//Swing Foot
		MatScalarMul(GravityZaxi, 3, mass_com+5, Gravity);				// m333435 * g
		MatScalarMul(AccelCOM+15+AxisJump, 3, mass_com+5, ForceCOM);	// ForceCOM333435 = m333435 * ac333435
		MatScalarMul(ForceCOM, 3, -1.0, ForceCOM);						// -ForceCOM333435 = -m333435 * ac333435
		MatAddAB(ForceCOM,Gravity,ForceJ+15+AxisJump,3);				// ForceJ333435 = -ma + mg
		MatAddAB(ForceJ+15+AxisJump,FSensor_forcR,ForceJ+15+AxisJump,3);// ForceSensor_forceR - ForceJ333435 + mg =  ForceCOM333435

		//////////////////////////////////////////////////////////////////////////////////////////
		// 其實若質量每個都有量 在此之後(303132~181920)可利用for迴圈一次算完 但因為現在模型省略沒有 所以只能慢慢算 再用指定的
		// 要再檢查!
		//for (int i = 4 ; i >= 0 ; i--)
		//{
		//	MatScalarMul(GravityZaxi, 3, mass_com+AxisJump+3*i, Gravity);				// i = 4		// m303132 * g
		//	MatScalarMul(AccelCOM+3*i+AxisJump, 3, mass_com+AxisJump+3*i, ForceCOM);	// ForceCOM303132 = m303132 * ac303132
		//	MatScalarMul(ForceCOM, 3, -1.0, ForceCOM);									// -ForceCOM303132 = -m303132 * ac303132
		//	MatAddAB(ForceCOM,Gravity,ForceJ+3*i+AxisJump,3);							// -ma + mg
		//	MatAddAB(ForceJ+3*i+AxisJump,ForceJ+3*(i+1)+AxisJump,ForceJ+3*i+AxisJump,3);// ForceJ333435 - ForceJ303132 + mg =  ForceCOM303132	
		//}
		//////////////////////////////////////////////////////////////////////////////////////////

		for (int i = 0 ; i < 3 ; i++)		// 因為333435 303132 軸共點 所以直接指定兩個力相等
			ForceJ[i+12+AxisJump]=ForceJ[i+15+AxisJump];

			//Swing Shank
		MatScalarMul(GravityZaxi, 3, mass_com+4, Gravity);			// m272829 * g
		MatScalarMul(AccelCOM+9+AxisJump, 3, mass_com+4, ForceCOM); // ForceCOM272829 = m272829 * ac272829
		MatScalarMul(ForceCOM, 3, -1.0, ForceCOM);					// -ForceCOM272829 = -m272829 * ac272829
		MatAddAB(ForceCOM, Gravity, ForceJ+9+AxisJump, 3);			// -ma + mg		
		MatAddAB(ForceJ+9+AxisJump, ForceJ+12+AxisJump , ForceJ+9+AxisJump, 3);	// ForceJ(i) - ForceJ(i-1) + m(i-1) * g =  m(i-1) * ac(i-1) = ForceCOM(i-1)

			//Swing Teight
		MatScalarMul(GravityZaxi, 3, mass_com+3, Gravity);			// m242526 * g
		MatScalarMul(AccelCOM+6+AxisJump, 3, mass_com+3, ForceCOM);	// ForceCOM242526 = m242526 * ac242526
		MatScalarMul(ForceCOM, 3, -1.0, ForceCOM);					// -ForceCOM242526 = -m242526 * ac242526
		MatAddAB(ForceCOM, Gravity, ForceJ+6+AxisJump,3);			// -ma + mg		
		MatAddAB(ForceJ+6+AxisJump,ForceJ+9+AxisJump ,ForceJ+6+AxisJump,3);		// ForceJ272829 - ForceJ242526 + m242526 * g =  m242526 * ac242526 = ForceCOM242526
		
		for (int i = 0 ; i < 3 ; i++)		//直接指定共點軸的力 ForceJ242526 = ForceJ212223 = ForceJ181920
		{
			ForceJ[i+AxisJump]   = ForceJ[i+6+AxisJump];
			ForceJ[i+3+AxisJump] = ForceJ[i+6+AxisJump];
		}

			//Body (waist)
		MatScalarMul(GravityZaxi, 3, BodyW, Gravity);	// m012 * g
		MatScalarMul(AccelCOM, 3, BodyW, ForceCOM);		// ForceCOM012 = m012 * a012
		MatScalarMul(ForceCOM, 3, -1.0, ForceCOM);		// -ForceCOM012 = -m * a
		MatAddAB(ForceCOM, Gravity, ForceJ,3);			// -ma + mg		
		MatAddAB(ForceJ, ForceJ+AxisJump, ForceJ,3);	// ForceJ181920 - ForceJ012 + m012 * g =  m012 * ac012 = ForceCOM012
		
		//////////////////////////////////////////////////////////////////////////////////////////
		// 其實若質量每個都有量 在此之後(345~151617)可利用for迴圈一次算完 但因為現在模型省略沒有 所以只能慢慢算 再用指定的
		// 要再檢查!
		//for (int i = 1 ; i <= 5 ; i++)
		//{
		//	MatScalarMul(GravityZaxi, 3, mass_com+3*i, Gravity);	// i = 0		// m345 * g
		//	MatScalarMul(AccelCOM+3*i, 3, mass_com+3*i, ForceCOM);	// ForceCOM345 = m345 * ac345
		//	MatScalarMul(ForceCOM, 3, -1.0, ForceCOM);				// -ForceCOM345 = -m345 * ac345
		//	MatAddAB(ForceCOM,Gravity,ForceJ+3*i,3);				// -ma + mg
		//	MatAddAB(ForceJ+3*i,ForceJ+3*(i-1),ForceJ+3*i,3);		// ForceJ012 - ForceJ345 + mg =  ForceCOM345
		//}
		//////////////////////////////////////////////////////////////////////////////////////////
	
		for (int i = 0 ; i < 3 ; i++)		//直接指定共點軸的力 ForceJ012 = ForceJ345 = ForceJ678
		{
			ForceJ[i+3]=ForceJ[i];
			ForceJ[i+6]=ForceJ[i];
		}

			//Fixed Teight
		MatScalarMul(GravityZaxi, 3, mass_com, Gravity);// m91011 * g
		MatScalarMul(AccelCOM+9, 3, mass_com, ForceCOM);// ForceCOM91011 = m91011 * ac91011
		MatScalarMul(ForceCOM, 3, -1.0, ForceCOM);		// -ForceCOM91011 = -m91011 * ac91011
		MatAddAB(ForceCOM, Gravity, ForceJ+9,3);		// -ma + mg		
		MatAddAB(ForceJ+9, ForceJ+6, ForceJ+9,3);		// ForceJ678 - ForceJ91011 + m91011 * g =  m91011 * ac91011 = ForceCOM91011

			//Fixed Shank
		MatScalarMul(GravityZaxi, 3, mass_com+1, Gravity);	// m121314 * g
		MatScalarMul(AccelCOM+12, 3, mass_com+1, ForceCOM);	// ForceCOM121314 = m121314 * ac121314
		MatScalarMul(ForceCOM, 3, -1.0, ForceCOM);			// -ForceCOM121314 = -m121314 * ac121314
		MatAddAB(ForceCOM,Gravity ,ForceJ+12,3);			// -ma + mg		
		MatAddAB(ForceJ+12, ForceJ+9, ForceJ+12,3);			// ForceJ91011 - ForceJ121314 + m121314 * g =  m121314 * ac121314 = ForceCOM121314

		for (int i = 0 ; i < 3 ; i++)		//直接指定共點軸的力 ForceJ151617 = ForceJ121314 
			ForceJ[i+15] = ForceJ[i+12];

	}
	else if(selIK == 1 )	// Left leg lifiting 
	{
		// Single support Swing leg

			//Swing Foot
		MatScalarMul(GravityZaxi, 3, mass_com+2, Gravity);	// m151617 * g
		MatScalarMul(AccelCOM+15, 3, mass_com+2, ForceCOM);	// ForceCOM151617 = m151617 * ac151617
		MatScalarMul(ForceCOM, 3, -1.0, ForceCOM);			// -ForceCOM151617 = -m151617 * ac151617
		MatAddAB(ForceCOM, Gravity, ForceJ+15,3);			// -ma + mg
		MatAddAB(ForceJ+15, FSensor_forcL, ForceJ+15,3);	// ForceSensor_forceL - ForceJ151617 + m151617 * g =  ForceCOM151617
		
		//////////////////////////////////////////////////////////////////////////////////////////
		// 其實若質量每個都有量 在此之後(121314~012)可利用for迴圈一次算完 但因為現在模型省略沒有 所以只能慢慢算 再用指定的
		// 要再檢查!
		//for (int i = 4 ; i >= 0 ; i--)
		//{
		//	MatScalarMul(GravityZaxi, 3, mass_com+3*i, Gravity);	// i = 5		// m121314 * g
		//	MatScalarMul(AccelCOM+3*i, 3, mass_com+3*i, ForceCOM);	// ForceCOM121314 = m121314 * ac121314
		//	MatScalarMul(ForceCOM, 3, -1.0, ForceCOM);				// -ForceCOM121314 = -m121314 * ac121314
		//	MatAddAB(ForceCOM,Gravity,ForceJ+3*i,3);				// -ma + mg
		//	MatAddAB(ForceJ+3*i,ForceJ+3*(i+1),ForceJ+3*i,3);		// ForceJ151617 - ForceJ121314 + mg =  ForceCOM121314
		//}
		//////////////////////////////////////////////////////////////////////////////////////////

		for (int i = 0 ; i < 3 ; i++)	// 因為151617 121314 軸共點 所以直接指定兩個力相等
			ForceJ[i+12]=ForceJ[i+15];

			//Swing Shank
		MatScalarMul(GravityZaxi, 3, mass_com+1, Gravity);	// m91011 * g
		MatScalarMul(AccelCOM+9, 3, mass_com+1, ForceCOM);	// ForceCOM91011 = m91011 * ac91011
		MatScalarMul(ForceCOM, 3, -1.0, ForceCOM);			// -ForceCOM91011 = -m91011 * ac91011
		MatAddAB(ForceCOM, Gravity, ForceJ+9,3);			// -ma + mg
		MatAddAB(ForceJ+9, ForceJ+12, ForceJ+9,3);			// ForceJ121314 - ForceJ91011 + m91011 * g =  ForceCOM91011

			//Swing Teight
		MatScalarMul(GravityZaxi, 3, mass_com, Gravity);	// m678 * g
		MatScalarMul(AccelCOM+6, 3, mass_com, ForceCOM);	// ForceCOM678 = m678 * ac678
		MatScalarMul(ForceCOM, 3, -1.0, ForceCOM);			// -ForceCOM678 = -m678 * ac678
		MatAddAB(ForceCOM, Gravity, ForceJ+6,3);			// -ma + mg
		MatAddAB(ForceJ+6 ,ForceJ+9, ForceJ+6,3);			// ForceJ91011 - ForceJ678 + m678 * g =  ForceCOM678
		
		for (int i = 0 ; i < 3 ; i++)	//直接指定共點軸的力 ForceJ345 = ForceJ012 = ForceJ678
		{
			ForceJ[i]=ForceJ[i+6];
			ForceJ[i+3]=ForceJ[i+6];
		}

			//Body 
		MatScalarMul(GravityZaxi, 3, BodyW, Gravity);			// m181920 * g
		MatScalarMul(AccelCOM+AxisJump, 3, BodyW, ForceCOM);	// ForceCOM181920 = m181920 * ac181920
		MatScalarMul(ForceCOM, 3, -1.0, ForceCOM);				// -ForceCOM181920 = -m181920 * ac181920
		MatAddAB(ForceCOM, Gravity, ForceJ+AxisJump,3);			// -ma + mg
		MatAddAB(ForceJ+AxisJump, ForceJ, ForceJ+AxisJump,3);	// ForceJ012 - ForceJ181920 + m181920 * g =  ForceCOM181920
		
		//////////////////////////////////////////////////////////////////////////////////////////
		// 其實若質量每個都有量 在此之後(212223~333435)可利用for迴圈一次算完 但因為現在模型省略沒有 所以只能慢慢算 再用指定的
		// 要再檢查!
		//for (int i = 1 ; i <= 5 ; i++)
		//{
		//	MatScalarMul(GravityZaxi, 3, mass_com+3*i+AxisJump, Gravity);						// i = 1		// m212223 * g
		//	MatScalarMul(AccelCOM+3*i+AxisJump, 3, mass_com+3*i+AxisJump, ForceCOM);			// ForceCOM212223 = m212223 * ac212223
		//	MatScalarMul(ForceCOM, 3, -1.0, ForceCOM);											// -ForceCOM212223 = -m212223 * ac212223
		//	MatAddAB(ForceCOM,Gravity,ForceJ+3*i+AxisJump,3);									// -ma + mg
		//	MatAddAB(ForceJ+3*i+AxisJump,ForceJ+3*(i-1)+AxisJump,ForceJ+3*i+AxisJump,3);		// ForceJ181920 - ForceJ212223 + mg =  ForceCOM212223
		//}
		//////////////////////////////////////////////////////////////////////////////////////////

		for (int i = 0 ; i < 3 ; i++)		//直接指定共點軸的力 ForceJ212223 = ForceJ242526 = ForceJ181920
		{
			ForceJ[i+3+AxisJump]=ForceJ[i+AxisJump];
			ForceJ[i+6+AxisJump]=ForceJ[i+AxisJump];
		}

			//Fixed Teight
		MatScalarMul(GravityZaxi, 3, mass_com+3, Gravity);						// m272829 * g
		MatScalarMul(AccelCOM+9+AxisJump, 3, mass_com+3, ForceCOM);				// ForceCOM272829 = m272829 * ac272829
		MatScalarMul(ForceCOM, 3, -1.0, ForceCOM);								// -ForceCOM272829 = -m272829 * ac272829
		MatAddAB(ForceCOM, Gravity, ForceJ+9+AxisJump, 3);						// -ma + mg		
		MatAddAB(ForceJ+9+AxisJump, ForceJ+6+AxisJump , ForceJ+9+AxisJump, 3);	// ForceJ242526 - ForceJ272829 + m272829 * g =  m272829 * ac272829 = ForceCOM272829

			//Fixed Shank
		MatScalarMul(GravityZaxi, 3, mass_com+4, Gravity);						// m303132 * g
		MatScalarMul(AccelCOM+12+AxisJump, 3, mass_com+4, ForceCOM);			// ForceCOM303132 = m303132 * ac303132
		MatScalarMul(ForceCOM, 3, -1.0, ForceCOM);								// -ForceCOM303132 = -m303132 * ac303132
		MatAddAB(ForceCOM, Gravity, ForceJ+12+AxisJump, 3);						// -ma + mg		
		MatAddAB(ForceJ+12+AxisJump, ForceJ+9+AxisJump , ForceJ+12+AxisJump, 3);	// ForceJ272829 - ForceJ303132 + m303132 * g =  m303132 * ac303132 = ForceCOM303132

		for (int i = 0 ; i < 3 ; i++)	// 直接指定共點軸的力 ForceJ333435 = ForceJ303132 
			ForceJ[i+15+AxisJump]=ForceJ[i+12+AxisJump];
		
		// 因為自由體圖的原因 令其中一邊全部加負號在Matlab上"人"看起來才是正確的 電腦計算則不用 因為哪個mode會配合自己的正負
		// 現在在右腳support時加負號 所以力的方向要看右腳到左腳的方向
		// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~注意!!~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~在這裡就加負號是否會影響接下來算Torque? 再想想!
		//for( int i = 0 ; i < 36 ; i++)	
		//	ForceJ[i] = -ForceJ[i];
	}
////////////////////////////////////////注意! 以上在此是從swing腳的力規值往Fixed腳推算Joint的力//////////////////////////////////////////////////////////////

		// 以下測試用

		//fstream Fx;
		//Fx.open("TestForceOrig.txt",ios::app);		
		//for(int i = 0 ; i < 12 ; i++)
		//	Fx << ForceJ[i*3] << "\t"<< ForceJ[i*3+1] << "\t"<< ForceJ[i*3+2]<< "\t";
		//Fx << endl;
		//Fx.close();
}
void Kine::FDTorqueJ(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 力規值須再校正~!!!!
	// 在此先利用各桿件質心座標描述轉動慣量 並且在此座標下計算質心力矩運動方程
	// 再將剛剛算得的值轉回世界 與前面Joint的力所造成的力矩計算並且迭代
	// 注意坐標軸的變換以及是給機器算或是給人看所要加的正負號
	// 若在一般情形下COM運動幾乎可以視為等速
	// 2012 Slongz Start
	// 20121218 Wei-Zh Lai Start 
	******************************************************************/
	double TempVector[3];
	double ROmega[3];
	double IAlpha[3];
	double OmegaIOmega[3];
	double FCrossR1[3];
	double FCrossR2[3];
	double Center[3];
	double Rot_Part_Rn[9];
	//double Rot_Part_Rn2[9];
	//double temp_Rn[9];

//if(DSPFlag==1 || selIK==2) //Checked
//{
	//	//權重 Checked
	//	MatScalarMul(CrdAll->data+12, 3, 1.0, TempVectorL );
	//	MatScalarMul(CrdAll->data+12+39, 3, 1.0, TempVectorR );
	//	TempVectorL[0]=TempVectorL[0]-ZMPcatch[0];
	//	TempVectorL[1]=TempVectorL[1]-ZMPcatch[1];
	//	TempVectorR[0]=TempVectorR[0]-ZMPcatch[0];
	//	TempVectorR[1]=TempVectorR[1]-ZMPcatch[1];

	//	tempTotal=sqrt(TempVectorL[0]*TempVectorL[0]+TempVectorL[1]*TempVectorL[1])+sqrt(TempVectorR[0]*TempVectorR[0]+TempVectorR[1]*TempVectorR[1]);
	//	FRatioR=sqrt(TempVectorL[0]*TempVectorL[0]+TempVectorL[1]*TempVectorL[1]) / tempTotal;
	//	FRatioL=sqrt(TempVectorR[0]*TempVectorR[0]+TempVectorR[1]*TempVectorR[1]) / tempTotal;


	//	TempZaxi[0]=0;
	//	TempZaxi[1]=0;
	//	TempZaxi[2]=-1;
	//	MatScalarMul(TempZaxi, 3, GravityConst, TempZaxi); // g*[0 0 -1]

	//	MatScalarMul(AccelTotalCOM, 3, SumMass, ForceTotalCOM ); //Total Ma			
	//	MatScalarMul(TempZaxi, 3, SumMass, TempPvStack); //Total Mg
	//	MatAddAB(TempPvStack,ForceTotalCOM , TempForce, 3); //Total Mg+Ma = GRF (Ground Reaction Force Downward
	//	//MatScalarMul(TempForce, 3, -1.0, TempForce); // GRF*-1 Upward
	//	MatScalarMul(TempForce, 3, FRatioR, TempForce); // GRF*-1 Upward
	//	//Single support Swing leg
	//	// Swing Foot
	//		//最後一軸Theta dot * z軸 + OmegaJ 15 (OmegaJ 15不包含自身轉動給予的角速度貢獻)
	//			MatScalarMul(ZAxisAll->data+3*5+DHJump, 3, ThetaD+5+6, TempVector1);
	//			MatAddAB(OmegaJ+15+AxisJump,TempVector1,TempOmegaJ,3);

	//		//最後一軸OmegaJ cross Local OmegaJ + OmegaJ 15 (AlphaJ 15不包含自身轉動給予的角加速度貢獻)
	//			//OmegaJ cross Local OmegaJ
	//					Cross2Vd(OmegaJ+3*5+AxisJump, LocalOmegaJ+3*5+AxisJump,TempVector1);
	//			//Theta dot dot * z軸
	//					MatScalarMul(ZAxisAll->data+3*5+DHJump, 3, ThetaDD+5+6, TempVector2);
	//					MatAddAB(TempVector1,TempVector2,TempAlphaJ,3);
	//			MatAddAB(AlphaJ+15+AxisJump,TempAlphaJ,TempAlphaJ,3);

	//		MatMulAB(InerLcRFoot,3,3,TempAlphaJ,3,1,TempVector1); //I*Alpha
	//		MatScalarMul(TempVector1, 3, -1, TempVector1);  //-I*Alpha

	//		MatMulAB(InerLcRFoot,3,3,TempOmegaJ,3,1,TempVector2); //I*Omega
	//		Cross2Vd(TempOmegaJ,TempVector2,TempVector2);     //Omega Cross I*Omega
	//		MatScalarMul(TempVector2, 3, -1, TempVector2);  //-Omega Cross I*Omega

	//		MatScalarMul(pv_stack+6+9, 3, -1, TempPvStack);
	//		MatAddAB(TempPvStack,CrdAll->data+12+DHJump,TempPvStack,3); //r com->i+1
	//		MatScalarMul(TempPvStack, 3, -1, TempPvStack);       //-(r com->i+1)
	//		Cross2Vd(TempPvStack,ForceJ+12+AxisJump,TempVector3);   
	//		//cout<<CrdAll->data[14]<<"\t"<<CrdAll->data[23]<<"\n";

	//		MatScalarMul(pv_stack+6+9, 3, -1, TempPvStack);
	//		MatAddAB(TempPvStack,CrdAll->data+23+DHJump,TempPvStack,3); //r com->i
	//		Cross2Vd(TempPvStack,TempForce,TempVector4);  

	//		MatAddAB(TempVector1,TempVector2,TorqueJ+15+AxisJump,3);
	//		MatAddAB(TorqueJ+15+AxisJump,TempVector3,TorqueJ+15+AxisJump,3);
	//		MatAddAB(TorqueJ+15+AxisJump,TempVector4,TorqueJ+15+AxisJump,3);

	//		for (int i = 0 ; i < 3 ; i++)

	//			TorqueJ[i+12+AxisJump]=TorqueJ[i+15+AxisJump];

	//	//Swing Shank
	//		MatMulAB(InerLcRShank,3,3,AlphaJ+12+AxisJump,3,1,TempVector1); //I*Alpha
	//		MatScalarMul(TempVector1, 3, -1, TempVector1);  //-I*Alpha

	//		MatMulAB(InerLcRShank,3,3,OmegaJ+12+AxisJump,3,1,TempVector2); //I*Omega
	//		Cross2Vd(OmegaJ+12+AxisJump,TempVector2,TempVector2);     //Omega Cross I*Omega
	//		MatScalarMul(TempVector2, 3, -1, TempVector2);  //-Omega Cross I*Omega

	//		MatScalarMul(pv_stack+3+9, 3, -1, TempPvStack);
	//		MatAddAB(TempPvStack,CrdAll->data+9+DHJump,TempPvStack,3); //r com->i+1
	//		MatScalarMul(TempPvStack, 3, -1, TempPvStack);       //-(r com->i+1)
	//		Cross2Vd(TempPvStack,ForceJ+9+AxisJump,TempVector3);     

	//		MatScalarMul(pv_stack+3+9, 3, -1, TempPvStack);
	//		MatAddAB(TempPvStack,CrdAll->data+12+DHJump,TempPvStack,3); //r com->i
	//		Cross2Vd(TempPvStack,ForceJ+12+AxisJump,TempVector4);  
	//	
	//		MatAddAB(TempVector1,TempVector2,TorqueJ+9+AxisJump,3);
	//		MatAddAB(TorqueJ+9+AxisJump,TempVector3,TorqueJ+9+AxisJump,3);
	//		MatAddAB(TorqueJ+9+AxisJump,TempVector4,TorqueJ+9+AxisJump,3);
	//		MatAddAB(TorqueJ+9+AxisJump,TorqueJ+12+AxisJump,TorqueJ+9+AxisJump,3);

	//	//Swing Teight
	//		MatMulAB(InerLcRThigh,3,3,AlphaJ+9+AxisJump,3,1,TempVector1); //I*Alpha
	//		MatScalarMul(TempVector1, 3, -1, TempVector1);  //-I*Alpha

	//		MatMulAB(InerLcRThigh,3,3,OmegaJ+9+AxisJump,3,1,TempVector2); //I*Omega
	//		Cross2Vd(OmegaJ+9+AxisJump,TempVector2,TempVector2);     //Omega Cross I*Omega
	//		MatScalarMul(TempVector2, 3, -1, TempVector2);  //-Omega Cross I*Omega

	//		MatScalarMul(pv_stack+9, 3, -1, TempPvStack);
	//		MatAddAB(TempPvStack,CrdAll->data+6+DHJump,TempPvStack,3); //r com->i+1
	//		MatScalarMul(TempPvStack, 3, -1, TempPvStack);       //-(r com->i+1)
	//		Cross2Vd(TempPvStack,ForceJ+6+AxisJump,TempVector3);     

	//		MatScalarMul(pv_stack+9, 3, -1, TempPvStack);
	//		MatAddAB(TempPvStack,CrdAll->data+9+DHJump,TempPvStack,3); //r com->i
	//		Cross2Vd(TempPvStack,ForceJ+9+AxisJump,TempVector4);  
	//	
	//		MatAddAB(TempVector1,TempVector2,TorqueJ+6+AxisJump,3);
	//		MatAddAB(TorqueJ+6+AxisJump,TempVector3,TorqueJ+6+AxisJump,3);
	//		MatAddAB(TorqueJ+6+AxisJump,TempVector4,TorqueJ+6+AxisJump,3);
	//		MatAddAB(TorqueJ+6+AxisJump,TorqueJ+9+AxisJump,TorqueJ+6+AxisJump,3);
	//		for (int i = 0 ; i < 3 ; i++)
	//		{
	//			TorqueJ[i+AxisJump]=TorqueJ[i+6+AxisJump];
	//			TorqueJ[i+3+AxisJump]=TorqueJ[i+6+AxisJump];
	//		}

	//	//Body //!!!!!!!!!!!
	//		MatMulAB(InerLcUpperBody,3,3,AlphaJ+AxisJump,3,1,TempVector1); //I*Alpha
	//		MatScalarMul(TempVector1, 3, -1, TempVector1);  //-I*Alpha

	//		MatMulAB(InerLcUpperBody,3,3,OmegaJ+AxisJump,3,1,TempVector2); //I*Omega
	//		Cross2Vd(OmegaJ+AxisJump,TempVector2,TempVector2);     //Omega Cross I*Omega
	//		MatScalarMul(TempVector2, 3, -1, TempVector2);  //-Omega Cross I*Omega

	//		MatScalarMul(BodyRCOM, 3, -1, TempPvStack);
	//		MatAddAB(TempPvStack,CrdAll->data+6,TempPvStack,3); //r com->i+1
	//		MatScalarMul(TempPvStack, 3, -1, TempPvStack);       //-(r com->i+1)
	//		Cross2Vd(TempPvStack,ForceJ,TempVector3);     

	//		MatScalarMul(BodyRCOM, 3, -1, TempPvStack);
	//		MatAddAB(TempPvStack,CrdAll->data+6+39,TempPvStack,3); //r com->i
	//		Cross2Vd(TempPvStack,ForceJ+AxisJump,TempVector4);  
	//	
	//		MatAddAB(TempVector1,TempVector2,TorqueJ,3);
	//		MatAddAB(TorqueJ,TempVector3,TorqueJ,3);
	//		MatAddAB(TorqueJ,TempVector4,TorqueJ,3);
	//		MatAddAB(TorqueJ,TorqueJ+AxisJump,TorqueJ,3);

	//		for (int i = 0 ; i < 3 ; i++)
	//		{
	//			TorqueJ[i+3]=TorqueJ[i];
	//			TorqueJ[i+6]=TorqueJ[i];
	//		}
	//		//Fixed Teight
	//		MatMulAB(InerLcLThighLsup,3,3,AlphaJ+6,3,1,TempVector1); //I*Alpha
	//		MatScalarMul(TempVector1, 3, -1, TempVector1);  //-I*Alpha

	//		MatMulAB(InerLcLThighLsup,3,3,OmegaJ+6,3,1,TempVector2); //I*Omega
	//		Cross2Vd(OmegaJ+6,TempVector2,TempVector2);     //Omega Cross I*Omega
	//		MatScalarMul(TempVector2, 3, -1, TempVector2);  //-Omega Cross I*Omega

	//		MatScalarMul(pv_stack, 3, -1, TempPvStack);
	//		MatAddAB(TempPvStack,CrdAll->data+9,TempPvStack,3); //r com->i+1
	//		MatScalarMul(TempPvStack, 3, -1, TempPvStack);       //-(r com->i+1)
	//		Cross2Vd(TempPvStack,ForceJ+9,TempVector3);     

	//		MatScalarMul(pv_stack, 3, -1, TempPvStack);
	//		MatAddAB(TempPvStack,CrdAll->data+6,TempPvStack,3); //r com->i
	//		Cross2Vd(TempPvStack,ForceJ+6,TempVector4);  
	//	
	//		MatAddAB(TempVector1,TempVector2,TorqueJ+9,3);
	//		MatAddAB(TorqueJ+9,TempVector3,TorqueJ+9,3);
	//		MatAddAB(TorqueJ+9,TempVector4,TorqueJ+9,3);
	//		MatAddAB(TorqueJ+9,TorqueJ+6,TorqueJ+9,3);

	////		//Fixed Shank
	//		MatMulAB(InerLcLShankLsup,3,3,AlphaJ+9,3,1,TempVector1); //I*Alpha
	//		MatScalarMul(TempVector1, 3, -1, TempVector1);  //-I*Alpha

	//		MatMulAB(InerLcLShankLsup,3,3,OmegaJ+9,3,1,TempVector2); //I*Omega
	//		Cross2Vd(OmegaJ+9,TempVector2,TempVector2);     //Omega Cross I*Omega
	//		MatScalarMul(TempVector2, 3, -1, TempVector2);  //-Omega Cross I*Omega

	//		MatScalarMul(pv_stack+3, 3, -1, TempPvStack);
	//		MatAddAB(TempPvStack,CrdAll->data+12,TempPvStack,3); //r com->i+1
	//		MatScalarMul(TempPvStack, 3, -1, TempPvStack);       //-(r com->i+1)
	//		Cross2Vd(TempPvStack,ForceJ+12,TempVector3);     

	//		MatScalarMul(pv_stack+3, 3, -1, TempPvStack);
	//		MatAddAB(TempPvStack,CrdAll->data+9,TempPvStack,3); //r com->i
	//		Cross2Vd(TempPvStack,ForceJ+9,TempVector4);  
	//	
	//		MatAddAB(TempVector1,TempVector2,TorqueJ+12,3);
	//		MatAddAB(TorqueJ+12,TempVector3,TorqueJ+12,3);
	//		MatAddAB(TorqueJ+12,TempVector4,TorqueJ+12,3);
	//		MatAddAB(TorqueJ+12,TorqueJ+9,TorqueJ+12,3);

	//		for (int i = 0 ; i < 3 ; i++)
	//			TorqueJ[i+15]=TorqueJ[i+12];
	//		for (int i = 0 ; i < 36 ; i++)
 	//			TorqueJ[i]=0;
//}
//else
//{
	if(selIK==2||selIK == 0)// Right leg lifiting 
	{
		//Single support Swing leg
		// Swing Foot
			// 取得下一軸的旋轉矩陣	
			GetRotPartRn(&FKRLeg->Rn[5],Rot_Part_Rn);	
			
			// I*Alpha (IAlpha)	
			MatMulAB(Rot_Part_Rn,3,3,AlphaL+15+AxisJump,3,1,TempVector);
			MatMulAB(IcRFoot,3,3,TempVector,3,1,IAlpha);	
			
			// Omega Cross I*Omega	(OmegaIOmega)	
			MatMulAB(Rot_Part_Rn,3,3, OmegaL+15+AxisJump,3,1, ROmega);
			MatMulAB(IcRFoot,3,3, ROmega,3,1, TempVector);
			Cross2Vd(ROmega, TempVector, OmegaIOmega);
			
			// 將質心運動(I*Alpha (IAlpha)	+ Omega Cross I*Omega (OmegaIOmega))轉回世界 並且加負號 等一下直接加起來
			MatAddAB(IAlpha, OmegaIOmega, TempVector,3);
			MatMulAtB(Rot_Part_Rn,3,3,TempVector,3,1,Center);
			MatScalarMul(Center, 3, -1, Center);

			// 計算力所造成的力矩
			Cross2Vd(rCOMccw+15, ForceJ+AxisJump+15, FCrossR1);	// r Cross force

			MatScalarMul(Rstack+15, 3, -1, TempVector);
			MatAddAB(TempVector, rCOMccw+15, TempVector,3);
			MatAddAB(TempVector, SensorOffset, TempVector,3);
			Cross2Vd(FSensor_forcR, TempVector, FCrossR2);	// force Cross r

			// 計算該軸力矩
			MatAddAB(FSensor_TorqR, FCrossR2, TempVector,3);
			MatAddAB(FCrossR1, Center, TorqueJ+15+AxisJump,3);
			MatAddAB(TempVector, TorqueJ+15+AxisJump, TorqueJ+15+AxisJump,3);
			 
			// 指定TorqueJ303132 = TorqueJ333435
			for (int i = 0 ; i < 3 ; i++)
				TorqueJ[i+AxisJump+12]=TorqueJ[i+AxisJump+15];

		//Swing Shank
			// 取得下一軸的旋轉矩陣
			GetRotPartRn(&FKRLeg->Rn[3],Rot_Part_Rn);	
			
			// I*Alpha (IAlpha)	
			MatMulAB(Rot_Part_Rn,3,3,AlphaL+9+AxisJump,3,1,TempVector);
			MatMulAB(IcRShank,3,3,TempVector,3,1,IAlpha);	
			
			// Omega Cross I*Omega	(OmegaIOmega)	
			MatMulAB(Rot_Part_Rn,3,3, OmegaL+9+AxisJump,3,1, ROmega);
			MatMulAB(IcRShank,3,3, ROmega,3,1, TempVector);
			Cross2Vd(ROmega, TempVector, OmegaIOmega);
			
			// 將質心運動(I*Alpha (IAlpha)	+ Omega Cross I*Omega (OmegaIOmega))轉回世界 並且加負號 等一下直接加起來
			MatAddAB(IAlpha, OmegaIOmega, TempVector,3);
			MatMulAtB(Rot_Part_Rn,3,3,TempVector,3,1,Center);
			MatScalarMul(Center, 3, -1, Center);

			// 計算力所造成的力矩
			Cross2Vd(rCOMccw+12, ForceJ+AxisJump+9, FCrossR1);	// r Cross force
			Cross2Vd(ForceJ+AxisJump+12,rCOMcw,FCrossR2);	// force Cross r

			// 計算該軸力矩
			MatAddAB(TorqueJ+12+AxisJump, FCrossR2, TempVector,3);
			MatAddAB(FCrossR1, Center, TorqueJ+9+AxisJump,3);
			MatAddAB(TempVector, TorqueJ+9+AxisJump, TorqueJ+9+AxisJump,3);

		//Swing Thigh
			// 取得下一軸的旋轉矩陣
			GetRotPartRn(&FKRLeg->Rn[2],Rot_Part_Rn);	
			
			// I*Alpha (IAlpha)	
			MatMulAB(Rot_Part_Rn,3,3,AlphaL+6+AxisJump,3,1,TempVector);
			MatMulAB(IcRThigh,3,3,TempVector,3,1,IAlpha);	
			
			// Omega Cross I*Omega	(OmegaIOmega)	
			MatMulAB(Rot_Part_Rn,3,3, OmegaL+6+AxisJump,3,1, ROmega);
			MatMulAB(IcRThigh,3,3, ROmega,3,1, TempVector);
			Cross2Vd(ROmega, TempVector, OmegaIOmega);
			
			// 將質心運動(I*Alpha (IAlpha)	+ Omega Cross I*Omega (OmegaIOmega))轉回世界 並且加負號 等一下直接加起來
			MatAddAB(IAlpha, OmegaIOmega, TempVector,3);
			MatMulAtB(Rot_Part_Rn,3,3,TempVector,3,1,Center);
			MatScalarMul(Center, 3, -1, Center);

			// 計算力所造成的力矩
			Cross2Vd(rCOMccw+9, ForceJ+AxisJump+6, FCrossR1);	// r Cross force
			Cross2Vd(ForceJ+AxisJump+9,rCOMcw+3,FCrossR2);	// force Cross r

			// 計算該軸力矩
			MatAddAB(TorqueJ+9+AxisJump, FCrossR2, TempVector,3);
			MatAddAB(FCrossR1, Center, TorqueJ+6+AxisJump,3);
			MatAddAB(TempVector, TorqueJ+6+AxisJump, TorqueJ+6+AxisJump,3);
			
			// 指定TorqueJ181920 = TorqueJ212223 = TorqueJ242526
			for (int i = 0 ; i < 3 ; i++)
			{
				TorqueJ[i+AxisJump]=TorqueJ[i+6+AxisJump];
				TorqueJ[i+3+AxisJump]=TorqueJ[i+6+AxisJump];
			}

		//Body 
			// 取得下一軸的旋轉矩陣
			GetRotPartRn(&FKLArm->Rn[0],Rot_Part_Rn);	
			// 在此選擇軸678 作為旋轉矩陣是否錯誤? 應要選012?
			// 但若這樣選擇 則在FindInertia2LocalCOM那邊也要照左右腳 support來旋轉Local慣量
			// 其實取座標的重點在於座標需固定在桿件上隨著Link轉動 故這裡最後選擇FKLArm->Rn[0] 為腰的第一軸
			
			// I*Alpha (IAlpha)	
			MatMulAB(Rot_Part_Rn,3,3,AlphaL,3,1,TempVector);
			MatMulAB(IcBody,3,3,TempVector,3,1,IAlpha);	
			
			// Omega Cross I*Omega	(OmegaIOmega)	
			MatMulAB(Rot_Part_Rn,3,3, OmegaL,3,1, ROmega);
			MatMulAB(IcBody,3,3, ROmega,3,1, TempVector);
			Cross2Vd(ROmega, TempVector, OmegaIOmega);
			
			// 將質心運動(I*Alpha (IAlpha)	+ Omega Cross I*Omega (OmegaIOmega))轉回世界 並且加負號 等一下直接加起來
			MatAddAB(IAlpha, OmegaIOmega, TempVector,3);
			MatMulAtB(Rot_Part_Rn,3,3,TempVector,3,1,Center);
			MatScalarMul(Center, 3, -1, Center);

			// 計算力所造成的力矩
			Cross2Vd(rCOMccw+6, ForceJ, FCrossR1);	// r Cross force
			Cross2Vd(ForceJ+AxisJump,rCOMcw+6,FCrossR2);	// force Cross r

			// 計算該軸力矩
			MatAddAB(TorqueJ+AxisJump, FCrossR2, TempVector,3);
			MatAddAB(FCrossR1, Center, TorqueJ,3);
			MatAddAB(TempVector, TorqueJ, TorqueJ,3);

			// 指定TorqueJ345 = TorqueJ678 = TorqueJ012
			for (int i = 0 ; i < 3 ; i++) 
			{
				TorqueJ[i+3]=TorqueJ[i];
				TorqueJ[i+6]=TorqueJ[i];
			}
			
		//Fixed Thigh
			// 取得下一軸的旋轉矩陣
			GetRotPartRn(&FKLLeg->Rn[2],Rot_Part_Rn);	
			
			// I*Alpha (IAlpha)	
			MatMulAB(Rot_Part_Rn,3,3,AlphaL+9,3,1,TempVector);
			MatMulAB(IcLThigh,3,3,TempVector,3,1,IAlpha);	
			
			// Omega Cross I*Omega	(OmegaIOmega)	
			MatMulAB(Rot_Part_Rn,3,3, OmegaL+9,3,1, ROmega);
			MatMulAB(IcLThigh,3,3, ROmega,3,1, TempVector);
			Cross2Vd(ROmega, TempVector, OmegaIOmega);
			
			// 將質心運動(I*Alpha (IAlpha)	+ Omega Cross I*Omega (OmegaIOmega))轉回世界 並且加負號 等一下直接加起來
			MatAddAB(IAlpha, OmegaIOmega, TempVector,3);
			MatMulAtB(Rot_Part_Rn,3,3,TempVector,3,1,Center);
			MatScalarMul(Center, 3, -1, Center);

			// 計算力所造成的力矩
			Cross2Vd(rCOMccw+3, ForceJ+9, FCrossR1);	// r Cross force
			Cross2Vd(ForceJ+6,rCOMcw+9,FCrossR2);	// force Cross r

			// 計算該軸力矩
			MatAddAB(TorqueJ+6, FCrossR2, TempVector,3);
			MatAddAB(FCrossR1, Center, TorqueJ+9,3);
			MatAddAB(TempVector, TorqueJ+9, TorqueJ+9,3);

		//Fixed Shank
			// 取得下一軸的旋轉矩陣
			GetRotPartRn(&FKLLeg->Rn[3],Rot_Part_Rn);	
			
			// I*Alpha (IAlpha)	
			MatMulAB(Rot_Part_Rn,3,3,AlphaL+12,3,1,TempVector);
			MatMulAB(IcLShank,3,3,TempVector,3,1,IAlpha);	
			
			// Omega Cross I*Omega	(OmegaIOmega)	
			MatMulAB(Rot_Part_Rn,3,3, OmegaL+12,3,1, ROmega);
			MatMulAB(IcLShank,3,3, ROmega,3,1, TempVector);
			Cross2Vd(ROmega, TempVector, OmegaIOmega);
			
			// 將質心運動(I*Alpha (IAlpha)	+ Omega Cross I*Omega (OmegaIOmega))轉回世界 並且加負號 等一下直接加起來
			MatAddAB(IAlpha, OmegaIOmega, TempVector,3);
			MatMulAtB(Rot_Part_Rn,3,3,TempVector,3,1,Center);
			MatScalarMul(Center, 3, -1, Center);

			// 計算力所造成的力矩
			Cross2Vd(rCOMccw, ForceJ+12, FCrossR1);	// r Cross force
			Cross2Vd(ForceJ+9,rCOMcw+12,FCrossR2);	// force Cross r

			// 計算該軸力矩
			MatAddAB(TorqueJ+9, FCrossR2, TempVector,3);
			MatAddAB(FCrossR1, Center, TorqueJ+12,3);
			MatAddAB(TempVector, TorqueJ+12, TorqueJ+12,3);

			// 指定TorqueJ151617 = TorqueJ121314
			for (int i = 0 ; i < 3 ; i++)
 				TorqueJ[i+15]=TorqueJ[i+12];

		// 因為自由體圖的原因 令其中一邊全部加負號在Matlab上"人"看起來才是正確的 電腦計算則不用 因為哪個mode會配合自己的正負
		// 現在在左腳support時右腳加負號 所以力和力矩的方向要由腰看到腳底
		for( int i = 18 ; i < 36 ; i++)	
		{
			ForceJ[i] = -ForceJ[i];
			TorqueJ[i] = -TorqueJ[i];
		}

	}
	else if(selIK == 1 )// Left leg lifiting 
	{
		//Single support Swing leg
		// Swing Foot
			// 取得下一軸的旋轉矩陣	
			GetRotPartRn(&FKLLeg->Rn[5],Rot_Part_Rn);	
			
			// I*Alpha (IAlpha)	
			MatMulAB(Rot_Part_Rn,3,3,AlphaL+15,3,1,TempVector);
			MatMulAB(IcLFoot,3,3,TempVector,3,1,IAlpha);	
			
			// Omega Cross I*Omega	(OmegaIOmega)	
			MatMulAB(Rot_Part_Rn,3,3, OmegaL+15,3,1, ROmega);
			MatMulAB(IcLFoot,3,3, ROmega,3,1, TempVector);
			Cross2Vd(ROmega, TempVector, OmegaIOmega);
			
			// 將質心運動(I*Alpha (IAlpha)	+ Omega Cross I*Omega (OmegaIOmega))轉回世界 並且加負號 等一下直接加起來
			MatAddAB(IAlpha, OmegaIOmega, TempVector,3);
			MatMulAtB(Rot_Part_Rn,3,3,TempVector,3,1,Center);
			MatScalarMul(Center, 3, -1, Center);

			// 計算力所造成的力矩
			Cross2Vd(rCOMcw+15, ForceJ+15, FCrossR1);	// r Cross force

			MatScalarMul(Rstack+6, 3, -1, TempVector);
			MatAddAB(TempVector, rCOMcw+15, TempVector,3);
			MatAddAB(TempVector, SensorOffset, TempVector,3);
			Cross2Vd(FSensor_forcL, TempVector, FCrossR2);	// force Cross r

		//		 //以下測試用
		//fstream Fx;
		//Fx.open("TestLFootR1.txt",ios::app);		
		//	Fx << FCrossR1[0] << "\t"<< FCrossR1[1] << "\t"<< FCrossR1[2]<< "\t";
		//Fx << endl;
		//Fx.close();

		//Fx.open("TestLFootR2.txt",ios::app);		
		//	Fx << FCrossR2[0] << "\t"<< FCrossR2[1] << "\t"<< FCrossR2[2]<< "\t";
		//Fx << endl;
		//Fx.close();

		//Fx.open("TestLFootCenter.txt",ios::app);		
		//	Fx << Center[0] << "\t"<< Center[1] << "\t"<< Center[2]<< "\t";
		//Fx << endl;
		//Fx.close();



			// 計算該軸力矩
			MatAddAB(FSensor_TorqL, FCrossR2, TempVector,3);
			MatAddAB(FCrossR1, Center, TorqueJ+15,3);
			MatAddAB(TempVector, TorqueJ+15, TorqueJ+15,3);
			 
			// 指定TorqueJ121314 = TorqueJ151617
			for (int i = 0 ; i < 3 ; i++)
				TorqueJ[i+12]=TorqueJ[i+15];

		//Swing Shank
			// 取得下一軸的旋轉矩陣
			GetRotPartRn(&FKLLeg->Rn[3],Rot_Part_Rn);	
			
			// I*Alpha (IAlpha)	
			MatMulAB(Rot_Part_Rn,3,3,AlphaL+9,3,1,TempVector);
			MatMulAB(IcLShank,3,3,TempVector,3,1,IAlpha);	
			
			// Omega Cross I*Omega	(OmegaIOmega)	
			MatMulAB(Rot_Part_Rn,3,3, OmegaL+9,3,1, ROmega);
			MatMulAB(IcLShank,3,3, ROmega,3,1, TempVector);
			Cross2Vd(ROmega, TempVector, OmegaIOmega);
			
			// 將質心運動(I*Alpha (IAlpha)	+ Omega Cross I*Omega (OmegaIOmega))轉回世界 並且加負號 等一下直接加起來
			MatAddAB(IAlpha, OmegaIOmega, TempVector,3);
			MatMulAtB(Rot_Part_Rn,3,3,TempVector,3,1,Center);
			MatScalarMul(Center, 3, -1, Center);

			// 計算力所造成的力矩
			Cross2Vd(rCOMcw+12, ForceJ+9, FCrossR1);	// r Cross force
			Cross2Vd(ForceJ+12,rCOMccw,FCrossR2);	// force Cross r

		//	 //以下測試用
		////fstream Fx;
		//Fx.open("TestLShankR1.txt",ios::app);		
		//	Fx << FCrossR1[0] << "\t"<< FCrossR1[1] << "\t"<< FCrossR1[2]<< "\t";
		//Fx << endl;
		//Fx.close();

		//Fx.open("TestLShankR2.txt",ios::app);		
		//	Fx << FCrossR2[0] << "\t"<< FCrossR2[1] << "\t"<< FCrossR2[2]<< "\t";
		//Fx << endl;
		//Fx.close();

		//Fx.open("TestLShankCenter.txt",ios::app);		
		//	Fx << Center[0] << "\t"<< Center[1] << "\t"<< Center[2]<< "\t";
		//Fx << endl;
		//Fx.close();

			// 計算該軸力矩
			MatAddAB(TorqueJ+12, FCrossR2, TempVector,3);
			MatAddAB(FCrossR1, Center, TorqueJ+9,3);
			MatAddAB(TempVector, TorqueJ+9, TorqueJ+9,3);

		//Swing Thigh
			// 取得下一軸的旋轉矩陣
			GetRotPartRn(&FKLLeg->Rn[2],Rot_Part_Rn);	
			
			// I*Alpha (IAlpha)	
			MatMulAB(Rot_Part_Rn,3,3,AlphaL+6,3,1,TempVector);
			MatMulAB(IcLThigh,3,3,TempVector,3,1,IAlpha);	
			
			// Omega Cross I*Omega	(OmegaIOmega)	
			MatMulAB(Rot_Part_Rn,3,3, OmegaL+6,3,1, ROmega);
			MatMulAB(IcLThigh,3,3, ROmega,3,1, TempVector);
			Cross2Vd(ROmega, TempVector, OmegaIOmega);
			
			// 將質心運動(I*Alpha (IAlpha)	+ Omega Cross I*Omega (OmegaIOmega))轉回世界 並且加負號 等一下直接加起來
			MatAddAB(IAlpha, OmegaIOmega, TempVector,3);
			MatMulAtB(Rot_Part_Rn,3,3,TempVector,3,1,Center);
			MatScalarMul(Center, 3, -1, Center);

			// 計算力所造成的力矩
			Cross2Vd(rCOMcw+9, ForceJ+6, FCrossR1);	// r Cross force
			Cross2Vd(ForceJ+9,rCOMccw+3,FCrossR2);	// force Cross r

		//		 //以下測試用
		////fstream Fx;
		//Fx.open("TestLThighR1.txt",ios::app);		
		//	Fx << FCrossR1[0] << "\t"<< FCrossR1[1] << "\t"<< FCrossR1[2]<< "\t";
		//Fx << endl;
		//Fx.close();

		//Fx.open("TestLThighR2.txt",ios::app);		
		//	Fx << FCrossR2[0] << "\t"<< FCrossR2[1] << "\t"<< FCrossR2[2]<< "\t";
		//Fx << endl;
		//Fx.close();

		//Fx.open("TestLThighCenter.txt",ios::app);		
		//	Fx << Center[0] << "\t"<< Center[1] << "\t"<< Center[2]<< "\t";
		//Fx << endl;
		//Fx.close();

			// 計算該軸力矩
			MatAddAB(TorqueJ+9, FCrossR2, TempVector,3);
			MatAddAB(FCrossR1, Center, TorqueJ+6,3);
			MatAddAB(TempVector, TorqueJ+6, TorqueJ+6,3);
			
			// 指定TorqueJ012 = TorqueJ345 = TorqueJ678
			for (int i = 0 ; i < 3 ; i++)
			{
				TorqueJ[i]=TorqueJ[i+6];
				TorqueJ[i+3]=TorqueJ[i+6];
			}

		//Body 
			// 取得下一軸的旋轉矩陣
			GetRotPartRn(&FKLArm->Rn[0],Rot_Part_Rn);	
			// 在此選擇軸242526 作為旋轉矩陣是否錯誤? 應要選181920?
			// 但若這樣選擇 則在FindInertia2LocalCOM那邊也要照左右腳 support來旋轉Local慣量
			// 其實取座標的重點在於座標需固定在桿件上隨著Link轉動 故這裡最後選擇FKLArm->Rn[0] 為腰的第一軸

			// I*Alpha (IAlpha)	
			MatMulAB(Rot_Part_Rn,3,3,AlphaL+AxisJump,3,1,TempVector);
			MatMulAB(IcBody,3,3,TempVector,3,1,IAlpha);	
			
			// Omega Cross I*Omega	(OmegaIOmega)	
			MatMulAB(Rot_Part_Rn,3,3, OmegaL+AxisJump,3,1, ROmega);
			MatMulAB(IcBody,3,3, ROmega,3,1, TempVector);
			Cross2Vd(ROmega, TempVector, OmegaIOmega);
			
			// 將質心運動(I*Alpha (IAlpha)	+ Omega Cross I*Omega (OmegaIOmega))轉回世界 並且加負號 等一下直接加起來
			MatAddAB(IAlpha, OmegaIOmega, TempVector,3);
			MatMulAtB(Rot_Part_Rn,3,3,TempVector,3,1,Center);
			MatScalarMul(Center, 3, -1, Center);

			// 計算力所造成的力矩
			Cross2Vd(rCOMcw+6, ForceJ+AxisJump, FCrossR1);	// r Cross force
			Cross2Vd(ForceJ,rCOMccw+6,FCrossR2);	// force Cross r

		//		 //以下測試用
		////fstream Fx;
		//Fx.open("TestBodyR1.txt",ios::app);		
		//	Fx << FCrossR1[0] << "\t"<< FCrossR1[1] << "\t"<< FCrossR1[2]<< "\t";
		//Fx << endl;
		//Fx.close();

		//Fx.open("TestBodyR2.txt",ios::app);		
		//	Fx << FCrossR2[0] << "\t"<< FCrossR2[1] << "\t"<< FCrossR2[2]<< "\t";
		//Fx << endl;
		//Fx.close();

		//Fx.open("TestRBodyCenter.txt",ios::app);		
		//	Fx << Center[0] << "\t"<< Center[1] << "\t"<< Center[2]<< "\t";
		//Fx << endl;
		//Fx.close();

			// 計算該軸力矩
			MatAddAB(TorqueJ, FCrossR2, TempVector,3);
			MatAddAB(FCrossR1, Center, TorqueJ+AxisJump,3);
			MatAddAB(TempVector, TorqueJ+AxisJump, TorqueJ+AxisJump,3);

			// 指定TorqueJ242526 = TorqueJ212223 = TorqueJ181920
			for (int i = 0 ; i < 3 ; i++) 
			{
				TorqueJ[i+3+AxisJump]=TorqueJ[i+AxisJump];
				TorqueJ[i+6+AxisJump]=TorqueJ[i+AxisJump];
			}
			
		//Fixed Thigh
			// 取得下一軸的旋轉矩陣
			GetRotPartRn(&FKRLeg->Rn[2],Rot_Part_Rn);	
			
			// I*Alpha (IAlpha)	
			MatMulAB(Rot_Part_Rn,3,3,AlphaL+AxisJump+9,3,1,TempVector);
			MatMulAB(IcRThigh,3,3,TempVector,3,1,IAlpha);	
			
			// Omega Cross I*Omega	(OmegaIOmega)	
			MatMulAB(Rot_Part_Rn,3,3, OmegaL+AxisJump+9,3,1, ROmega);
			MatMulAB(IcRThigh,3,3, ROmega,3,1, TempVector);
			Cross2Vd(ROmega, TempVector, OmegaIOmega);
			
			// 將質心運動(I*Alpha (IAlpha)	+ Omega Cross I*Omega (OmegaIOmega))轉回世界 並且加負號 等一下直接加起來
			MatAddAB(IAlpha, OmegaIOmega, TempVector,3);
			MatMulAtB(Rot_Part_Rn,3,3,TempVector,3,1,Center);
			MatScalarMul(Center, 3, -1, Center);

			// 計算力所造成的力矩
			Cross2Vd(rCOMcw+3, ForceJ+AxisJump+9, FCrossR1);	// r Cross force
			Cross2Vd(ForceJ+AxisJump+6,rCOMccw+9,FCrossR2);	// force Cross r

		//		 //以下測試用
		////fstream Fx;
		//Fx.open("TestRThighR1.txt",ios::app);		
		//	Fx << FCrossR1[0] << "\t"<< FCrossR1[1] << "\t"<< FCrossR1[2]<< "\t";
		//Fx << endl;
		//Fx.close();

		//Fx.open("TestRThighR2.txt",ios::app);		
		//	Fx << FCrossR2[0] << "\t"<< FCrossR2[1] << "\t"<< FCrossR2[2]<< "\t";
		//Fx << endl;
		//Fx.close();

		//Fx.open("TestRThighCenter.txt",ios::app);		
		//	Fx << Center[0] << "\t"<< Center[1] << "\t"<< Center[2]<< "\t";
		//Fx << endl;
		//Fx.close();

			// 計算該軸力矩
			MatAddAB(TorqueJ+AxisJump+6, FCrossR2, TempVector,3);
			MatAddAB(FCrossR1, Center, TorqueJ+AxisJump+9,3);
			MatAddAB(TempVector, TorqueJ+AxisJump+9, TorqueJ+AxisJump+9,3);

		//Fixed Shank
			// 取得下一軸的旋轉矩陣
			GetRotPartRn(&FKRLeg->Rn[3],Rot_Part_Rn);	
			
			// I*Alpha (IAlpha)	
			MatMulAB(Rot_Part_Rn,3,3,AlphaL+AxisJump+12,3,1,TempVector);
			MatMulAB(IcRShank,3,3,TempVector,3,1,IAlpha);	
			
			// Omega Cross I*Omega	(OmegaIOmega)	
			MatMulAB(Rot_Part_Rn,3,3, OmegaL+AxisJump+12,3,1, ROmega);
			MatMulAB(IcRShank,3,3, ROmega,3,1, TempVector);
			Cross2Vd(ROmega, TempVector, OmegaIOmega);
			
			// 將質心運動(I*Alpha (IAlpha)	+ Omega Cross I*Omega (OmegaIOmega))轉回世界 並且加負號 等一下直接加起來
			MatAddAB(IAlpha, OmegaIOmega, TempVector,3);
			MatMulAtB(Rot_Part_Rn,3,3,TempVector,3,1,Center);
			MatScalarMul(Center, 3, -1, Center);

			// 計算力所造成的力矩
			Cross2Vd(rCOMcw, ForceJ+AxisJump+12, FCrossR1);	// r Cross force
			Cross2Vd(ForceJ+AxisJump+9,rCOMccw+12,FCrossR2);	// force Cross r

		//		 //以下測試用
		////fstream Fx;
		//Fx.open("TestRShankR1.txt",ios::app);		
		//	Fx << FCrossR1[0] << "\t"<< FCrossR1[1] << "\t"<< FCrossR1[2]<< "\t";
		//Fx << endl;
		//Fx.close();

		//Fx.open("TestRShankR2.txt",ios::app);		
		//	Fx << FCrossR2[0] << "\t"<< FCrossR2[1] << "\t"<< FCrossR2[2]<< "\t";
		//Fx << endl;
		//Fx.close();

		//Fx.open("TestRShankCenter.txt",ios::app);		
		//	Fx << Center[0] << "\t"<< Center[1] << "\t"<< Center[2]<< "\t";
		//Fx << endl;
		//Fx.close();

			// 計算該軸力矩
			MatAddAB(TorqueJ+AxisJump+9, FCrossR2, TempVector,3);
			MatAddAB(FCrossR1, Center, TorqueJ+AxisJump+12,3);
			MatAddAB(TempVector, TorqueJ+AxisJump+12, TorqueJ+AxisJump+12,3);

			// 指定TorqueJ333435 = TorqueJ303132
			for (int i = 0 ; i < 3 ; i++)
 				TorqueJ[i+AxisJump+15]=TorqueJ[i+AxisJump+12];
	
		// 因為自由體圖的原因 令其中一邊全部加負號在Matlab上"人"看起來才是正確的 電腦計算則不用 因為哪個mode會配合自己的正負
		// 現在在右腳support時左腳加負號 所以力和力矩的方向由腰看向腳底
		for( int i = 0 ; i < 18 ; i++)	
		{
			ForceJ[i] = -ForceJ[i];
			TorqueJ[i] = -TorqueJ[i];
		}
		
	}
		
	// 把各軸的Torque投影到該軸馬達可施力的Z軸上(DH model)
	for (int i = 0 ; i < 6 ; i++)
	{		
		MatMulAB(ZAxisAll->data+3*i,1,3,TorqueJ+i*3,3,1,MotorTorq+i);
		MatMulAB(ZAxisAll->data+3*i+DHJump,1,3,TorqueJ+i*3+AxisJump,3,1,MotorTorq+i+6);
	}

	//// 投影完後對該群資料做濾波(Kalman Filter)
	//for (int i = 0 ; i < 12 ; i++)	
	//{
 //       //do a prediction
	//		x_temp_estMotor[i] = x_est_lastMotor[i];
	//		P_tempMotor[i] = P_lastMotor[i] + QMotor[i];
 //       //calculate the Kalman gain
	//		KMotor[i] = P_tempMotor[i] * (1.0/(P_tempMotor[i] + RMotor[i]));
 //       //measure
	//		z_measuredMotor[i] = MotorTorq[i] ; //the measurement with noise
 //       //correct
	//		x_estMotor[i] = x_temp_estMotor[i] + KMotor[i] * (z_measuredMotor[i] - x_temp_estMotor[i]); 
	//		PMotor[i] = (1- KMotor[i]) * P_tempMotor[i];
 //       //we have our new system
	//		//printf("Ideal    position: %6.3f \n",z_real);
	//		//printf("Mesaured position: %6.3f [diff:%.3f]\n",z_measured,fabs(z_real-z_measured));
	//		//printf("Kalman   position: %6.3f [diff:%.3f]\n",x_est,fabs(z_real - x_est));
 //       
 //       //sum_error_kalman += fabs(z_real - x_est);
 //       //sum_error_measure += fabs(z_real-z_measured);
 //       
 //       //update our last's
	//		P_lastMotor[i] = PMotor[i];
	//		x_est_lastMotor[i] = x_estMotor[i];
 //   }   

	//// 將算出之Motor Torque 轉換成Rated Torque 為了輸入EPOS3 Torque offset
	//// C++算出的Torque單位為 N*m*10^-9
	//// 所以要先把單位便成 N*m 故須先*10^-9
	//// 注意!!!! 在此輸出的Motor單位為 千分之一的Rated Torque!!!!!!!!!!!
	//// 還未換成實際機構的正負
	//for (int i = 0 ; i < 12 ; i++)
	//{
	//	//MotorTorq[i] = (x_estMotor[i]) / 1000000000 / RatedTorque[i] / GearRatio[i] ;
	//	MotorTorq[i] = (MotorTorq[i]) / 1000000000 / RatedTorque[i] / GearRatio[i] ;	// 不要濾波 去MATLAB濾
	//}


	//for (int i = 0 ; i < 12 ; i++) //unit test
	//{
	//	MotorTorq[i] = (x_estMotor[i]);
	//}



	//// 將算出之Motor Torque 轉換成N*mm 為了方便在Adams裡檢查
	//for (int i = 0 ; i < 6 ; i++)
	//{
	//	MotorTorq[i] = x_estMotor[6+i]/1000000 ;
	//	MotorTorq[6+i] = x_estMotor[6+i]/1000000 ;
	//}

	//????? 20130423
	//for (int i = 0 ; i < 12 ; i++)
	//	MotorTorq[i] = MotorTorq[i]*10;
	
	// 初始化MotorTorq 使Torque Control一開始脫離Singular
	//if(CountMotor<20)
	//{
	//		/*MotorTorq[0] = 0*CountMotor*0.05;
	//		MotorTorq[1] = -1560*CountMotor*0.05;
	//		MotorTorq[2] = 3320*CountMotor*0.05;*/
	//		MotorTorq[3] = -6120*(1-CountMotor*0.1);
	//		/*MotorTorq[4] = 3320*CountMotor*0.05;
	//		MotorTorq[5] = -1409*CountMotor*0.05;*/

	//		/*MotorTorq[6] = 0*CountMotor*0.05;
	//		MotorTorq[7] = 1484*CountMotor*0.05;
	//		MotorTorq[8] = 2005*CountMotor*0.05;*/
	//		MotorTorq[9] = -6108*(1-CountMotor*0.1);
	//	/*	MotorTorq[10] = 2011*CountMotor*0.05;
	//		MotorTorq[11] = 1507*CountMotor*0.05;*/
	//		CountMotor=CountMotor+1;
	//}

	// //20130115 直接輸入Adams量到的MotionTorque (失敗)
	//for (int i = 0 ; i < 12 ; i++)
	//	MotorTorq[i] = AdamsMotionT[i];


	
	// 20130116 測試Adams中軸力矩施力的+-與C++中值的正負 只先看pitch方向
	//for (int i = 0 ; i < 12 ; i++)
	//	MotorTorq[i] = 0;
	
		//MotorTorq[4] = 3000;
		//MotorTorq[10] = 3000;
		//MotorTorq[3] = -3000;
		//MotorTorq[9] = -3000;
	//	MotorTorq[2] = 6000;
	//	MotorTorq[8] = 6000;

	//MotorTorq[2] = AdamsMotionT[2];
	//MotorTorq[3] = -AdamsMotionT[3];
	//MotorTorq[4] = AdamsMotionT[4];

	//MotorTorq[8] = AdamsMotionT[8];
	//MotorTorq[9] = -AdamsMotionT[9];
	//MotorTorq[10] = AdamsMotionT[10];


		//MotorTorq[4] = 240;
		//MotorTorq[10] = 240;

		//MotorTorq[3] = -12290;
		//MotorTorq[9] = -12290;

		//MotorTorq[2] = -1900;
		//MotorTorq[8] = -1900;
	



	 ////以下測試用
		//fstream Fx;
		
		//Fx.open("TestForce.txt",ios::app);		
		//for(int i = 0 ; i < 12 ; i++)
		//	Fx << ForceJ[i*3] << "\t"<< ForceJ[i*3+1] << "\t"<< ForceJ[i*3+2]<< "\t";
		//Fx << endl;
		//Fx.close();

		//Fx.open("TestTorque.txt",ios::app);		
		//for(int i = 0 ; i < 12 ; i++)
		//	Fx << TorqueJ[i*3] << "\t"<< TorqueJ[i*3+1] << "\t"<< TorqueJ[i*3+2]<< "\t";
		//Fx << endl;
		//Fx.close();

		//Fx.open("TestMotor.txt",ios::app);		
		//for(int i = 0 ; i < 12 ; i++)
		//	Fx << MotorTorq[i] << "\t";
		//Fx << endl;
		//Fx.close();
}

void Kine::Friction(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 加入摩擦力 摩擦力的單位為N*m 角度的單位為弧度
	// 20130507 Wei-Zh Lai Start 
	******************************************************************/
	double Coulomb;
	double Viscous;

    for(int i = 0 ; i < 12 ; i++)
	{
		if (ThetaD[i] == 0)
			Coulomb = 0;
        else if (ThetaD[i] > 0)
			Coulomb = FrictionJ[i+12];
        else
			Coulomb = -1 * FrictionJ[i+12];

		Viscous = FrictionJ[i] * ThetaD[i];
		
		MotorTorq[i] = MotorTorq[i] / 1000000000 ;	// 各軸需要的的Torque(理想) N*m 
		MotorTorqF[i] = MotorTorq[i] + Viscous + Coulomb*2.5;
		MotorTorq[i] = MotorTorq[i] / RatedTorque[i] / GearRatio[i];
		MotorTorqF[i] = MotorTorqF[i] / RatedTorque[i] / GearRatio[i];
	}	
	
	// 將算出之Motor Torque 轉換成Rated Torque 為了輸入EPOS3 Torque offset
	// C++算出的Torque單位為 N*m*10^-9
	// 所以要先把單位便成 N*m 故須先*10^-9
	// 注意!!!! 在此輸出的Motor單位為 千分之一的Rated Torque!!!!!!!!!!!
	// 還未換成實際機構的正負
	//for (int i = 0 ; i < 12 ; i++)
	//{
	//	//MotorTorq[i] = (MotorTorq[i]) / 1000000000 / RatedTorque[i] / GearRatio[i] ;	// 不要濾波 去MATLAB濾
	//}


	 ////以下測試用
		//fstream Fx;

		//Fx.open("TestMotorF.txt",ios::app);		
		//for(int i = 0 ; i < 12 ; i++)
		//	Fx << MotorTorqF[i] << "\t";
		//Fx << endl;
		//Fx.close();
		//
		//Fx.open("TestMotor.txt",ios::app);		
		//for(int i = 0 ; i < 12 ; i++)
		//	Fx << MotorTorq[i] << "\t";
		//Fx << endl;
		//Fx.close();

}


void Kine::ForceSensorData(int FlagSim, int FlagGo, int SensorCount, int LogCount, double *FS_DataL, double *FS_DataR)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 線上收力規值 或是離線讀txt檔(經由Adams模擬而得Adams單位F:N T:N*mm)
	// 將Adams座標轉到C++座標 請注意Adams輸出的是地對腳還是腳對地 是向量的相反還是座標軸的相反
	// 利用Kalman Filter濾波 請特別注意收回值的大小 單位 方向
	// 如果走路劇情有改 離線txt請記得一起改
	// 詳情請參照C2M系列 輸入順序如下
	// VLForceX, VLForceYN,VLForceZ,
	// VLTorqueX, VLTorqueYN, VLTorqueZ, 
	// VRForceX, VRForceYN,VRForceZ,
	// VRTorqueX, VRTorqueYN, VRTorqueZ
	// 在前20個Step 先藉由 InitialD 調整QR 相信Sensor 脫離Singular
	// CountMotor > 20 後將QR 調回相信Model 為了移除Impulse
	// 20130313 將z_measured利用gKineAll調整成以RobotAllDlg來指定
	// Model為SlongZ 20130111 NinoKai
	// 20130103 Wei-Zh Lai Start 
	******************************************************************/
	double OrigCoPLx;	// 為處理轉彎力規顯示問題 尚未乘旋轉矩陣前的兩腳COP 為local frame (乘完旋轉矩陣變成CoPL)
	double OrigCoPLy;
	double OrigCoPRx;
	double OrigCoPRy;
	int DataCount_FS = SensorCount+LogCount;

	// 若為adams模擬 將adams力規raw data輸入至 FS_DataL 與 FS_DataR (gForceDataLLeg, gForceDataRLeg) 
	// 輸入模擬ADAMS回來的力規值轉回C++座標 注意順序 正負 單位 力規直向上看為正
	// 先將單位轉成與力規相同 
	// 力規  ForceUnits="N" TorqueUnits="N-m"
	// Adams單位 F: N   T: N*mm
	if(FlagSim == ADAMSSimu){
		FS_DataL[(DataCount_FS)*6] = AdamsFS[0];
		FS_DataL[(DataCount_FS)*6+1] = -AdamsFS[1];
		FS_DataL[(DataCount_FS)*6+2] = AdamsFS[2];	
		FS_DataL[(DataCount_FS)*6+3] = AdamsFS[3]*0.001;
		FS_DataL[(DataCount_FS)*6+4] = -AdamsFS[4]*0.001;	
		FS_DataL[(DataCount_FS)*6+5] = AdamsFS[5]*0.001;	
		FS_DataR[(DataCount_FS)*6] = AdamsFS[6];
		FS_DataR[(DataCount_FS)*6+1] = -AdamsFS[7];
		FS_DataR[(DataCount_FS)*6+2] = AdamsFS[8];
		FS_DataR[(DataCount_FS)*6+3] = AdamsFS[9]*0.001;
		FS_DataR[(DataCount_FS)*6+4] = -AdamsFS[10]*0.001;
		FS_DataR[(DataCount_FS)*6+5] = AdamsFS[11]*0.001;

		for (int i = 0 ; i < 12 ; ++i)	// ADAMS 多濾一點
		{
			Q_KF[i] = 0.01;	// Q為model的coverence 越小越相信
			R_KF[i] = 10;	// R為measuremant的coverence 越小越相信	
		}
	}

	// 在不同狀況下給力規資料
	// 輸入力規raw data txt檔單位一律為 F: N   T:N*m	
	if(FlagSim == CppSimu && LogCount ==0){
		KineFile.open("force_data_LLeg.txt",ios::in);
		for (int i = 0 ; i <SensorBufferSize  ; i++){
			KineFile >> FS_DataL[i];
		}
		KineFile.close();

		KineFile.open("force_data_RLeg.txt",ios::in);
		for (int i = 0 ; i < SensorBufferSize ; i++){
			KineFile >> FS_DataR[i];
		}
		KineFile.close();
	}

	for(int i = 0 ; i < 6 ; i ++){
		if(DataCount_FS == 0){
			x_est_last[i] = FS_DataL[i];// 尚未傳值前 第一筆資料直接當預測值
			x_est_last[i+6] = FS_DataR[i];
		}
		z_measured[i] = FS_DataL[(DataCount_FS)*6+i];
		z_measured[i+6] = FS_DataR[(DataCount_FS)*6+i];
	}

	// 收完後對該群資料做濾波(Kalman Filter)
	for (int i = 0 ; i < 12 ; i++){	
        //do a prediction
			x_temp_est[i] = x_est_last[i];
			P_temp[i] = P_last[i] + Q_KF[i];
        //calculate the Kalman gain
			K_KF[i] = P_temp[i] * (1.0/(P_temp[i] + R_KF[i]));
        //correct
			x_est[i] = x_temp_est[i] + K_KF[i] * (z_measured[i] - x_temp_est[i]); 
			P_KF[i] = (1- K_KF[i]) * P_temp[i];   
        //update our last's
			P_last[i] = P_KF[i];
			x_est_last[i] = x_est[i];
	}

	// 輸入力規TXT 回來的力規值轉回C++座標 注意順序 正負 單位 力規直向上看為正
	// Adams單位 F: N   T: N*mm
	// C++單位 F:g*mm/s^2 = 10^-6 N   T:g*mm/s^2 * mm = 10^-9 N*m
	// 線上收的真實力規值已經轉為機器人座標
	// 力規的單位 ForceUnits="N" TorqueUnits="N-m"
	FSensor_forcL[0] = x_est[0]*(1e+6);
	FSensor_forcL[1] = x_est[1]*(1e+6);
	FSensor_forcL[2] = x_est[2]*(1e+6);
	FSensor_TorqL[0] = x_est[3]*(1e+9);
	FSensor_TorqL[1] = x_est[4]*(1e+9);
	FSensor_TorqL[2] = x_est[5]*(1e+9);
	FSensor_forcR[0] = x_est[6]*(1e+6);
	FSensor_forcR[1] = x_est[7]*(1e+6);
	FSensor_forcR[2] = x_est[8]*(1e+6);
	FSensor_TorqR[0] = x_est[9]*(1e+9);
	FSensor_TorqR[1] = x_est[10]*(1e+9);
	FSensor_TorqR[2] = x_est[11]*(1e+9);

	// Cali結束 開始傳值後 按下save存入 已濾過波的力規資料
	// 會這樣存是因為在外面(PMS)想要看到txt的單位不要太巨大
	// 單位為 F: N   T:N*m
	ForceDataKFL[(DataCount_FS)*6] = x_est[0];
	ForceDataKFL[(DataCount_FS)*6+1] = x_est[1];
	ForceDataKFL[(DataCount_FS)*6+2] = x_est[2];	
	ForceDataKFL[(DataCount_FS)*6+3] = x_est[3];
	ForceDataKFL[(DataCount_FS)*6+4] = x_est[4];	
	ForceDataKFL[(DataCount_FS)*6+5] = x_est[5];	
	ForceDataKFR[(DataCount_FS)*6] = x_est[6];
	ForceDataKFR[(DataCount_FS)*6+1] = x_est[7];
	ForceDataKFR[(DataCount_FS)*6+2] = x_est[8];
	ForceDataKFR[(DataCount_FS)*6+3] = x_est[9];
	ForceDataKFR[(DataCount_FS)*6+4] = x_est[10];
	ForceDataKFR[(DataCount_FS)*6+5] = x_est[11];	
	

	////20130308 WeiZh
	// 省略腳板的加速度和重量 應該要加回去! 
	// 但加過好像沒差多少@@a 應該是因為接近靜行走 可以忽略
	// 注意在此COP的準位已為+-80 若腳的起始寬度有改變 則gCoPCali的deadzone要手動調整
	// 60是腳底板到力規的距離
	// 乘上選轉矩陣以利在轉彎時仍能記下正確的COP ZMP (注意單位:mm)
	if(FlagGo==0){	// Go按下 CrdAll開始有值	
		// X = (x*Fz-z*Fx-My)/Fz
		// Y = (y*Fz-z*Fy+Mx)/Fz
		OrigCoPLx = (-FSensor_TorqL[1] - (60)*FSensor_forcL[0] + (CrdAll->data[21]*FSensor_forcL[2]) ) / FSensor_forcL[2];
		OrigCoPLy = (FSensor_TorqL[0] - (60)*FSensor_forcL[1] + (CrdAll->data[22]*FSensor_forcL[2]) ) / FSensor_forcL[2];
		OrigCoPRx = (-FSensor_TorqR[1] - (60)*FSensor_forcR[0] + (CrdAll->data[60]*FSensor_forcR[2]) ) / FSensor_forcR[2];
		OrigCoPRy = (FSensor_TorqR[0] - (60)*FSensor_forcR[1] + (CrdAll->data[61]*FSensor_forcR[2]) ) / FSensor_forcR[2];

		if(FSensor_forcL[2] < 20*(1e+6)){	//左腳離地(< 20N) SSP ZMP不看左腳力規值 右腳為原點SUP
			CoPL[LogCount*2] = TarRotMSw[0] * OrigCoPLx + TarRotMSw[1] * OrigCoPLy;
			CoPL[LogCount*2+1] = TarRotMSw[3] * OrigCoPLx + TarRotMSw[4] * OrigCoPLy;
			CoPR[LogCount*2] = TarRotMFx[0] * OrigCoPRx + TarRotMFx[1] * OrigCoPRy;
			CoPR[LogCount*2+1] = TarRotMFx[3] * OrigCoPRx + TarRotMFx[4] * OrigCoPRy;
						
			FS_ZMP[LogCount*2] = CoPR[DataCount_FS*2];
			FS_ZMP[LogCount*2+1] = CoPR[DataCount_FS*2+1];
		}
		else if(FSensor_forcR[2] < 20*(1e+6)){	//右腳離地(< 20N) SSP ZMP不看右腳力規值 左腳為原點SUP
			CoPL[LogCount*2] = TarRotMFx[0] * OrigCoPLx + TarRotMFx[1] * OrigCoPLy;
			CoPL[LogCount*2+1] = TarRotMFx[3] * OrigCoPLx + TarRotMFx[4] * OrigCoPLy;
			CoPR[LogCount*2] = TarRotMSw[0] * OrigCoPRx + TarRotMSw[1] * OrigCoPRy;
			CoPR[LogCount*2+1] = TarRotMSw[3] * OrigCoPRx + TarRotMSw[4] * OrigCoPRy;
						
			FS_ZMP[LogCount*2] = CoPL[DataCount_FS*2];
			FS_ZMP[LogCount*2+1] = CoPL[DataCount_FS*2+1];
		}
		else{	//DSP					
			// Xp = (FzL*XL+FzR*XR)/(FzL+FzR)
			// Yp = (FzL*YL+FzR*YR)/(FzL+FzR)
			CoPL[LogCount*2] = TarRotMSw[0] * OrigCoPLx + TarRotMSw[1] * OrigCoPLy;
			CoPL[LogCount*2+1] = TarRotMSw[3] * OrigCoPLx + TarRotMSw[4] * OrigCoPLy;
			CoPR[LogCount*2] = TarRotMFx[0] * OrigCoPRx + TarRotMFx[1] * OrigCoPRy;
			CoPR[LogCount*2+1] = TarRotMFx[3] * OrigCoPRx + TarRotMFx[4] * OrigCoPRy;
						
			FS_ZMP[LogCount*2] = (FSensor_forcL[2]*CoPL[DataCount_FS*2] + FSensor_forcR[2]*CoPR[DataCount_FS*2]) / (FSensor_forcL[2] + FSensor_forcR[2]);
			FS_ZMP[LogCount*2+1] = (FSensor_forcL[2]*CoPL[DataCount_FS*2+1] + FSensor_forcR[2]*CoPR[DataCount_FS*2+1]) / (FSensor_forcL[2] + FSensor_forcR[2]);
		}
	}
	else{	// Go前 CrdAll無值 DSP (兩腳都在原點 所以Xs = 0)
		CoPL[0] = (-FSensor_TorqL[1] - (60)*FSensor_forcL[0] ) / FSensor_forcL[2];
		CoPL[1] = (FSensor_TorqL[0] - (60)*FSensor_forcL[1] + (80*FSensor_forcL[2]) ) / FSensor_forcL[2];
		CoPR[0] = (-FSensor_TorqR[1] - (60)*FSensor_forcR[0] ) / FSensor_forcR[2];
		CoPR[1] = (FSensor_TorqR[0] - (60)*FSensor_forcR[1] + ((-80)*FSensor_forcR[2]) ) / FSensor_forcR[2];

		FS_ZMP[0] = (FSensor_forcL[2]*CoPL[0] + FSensor_forcR[2]*CoPR[0]) / (FSensor_forcL[2] + FSensor_forcR[2]);
		FS_ZMP[1] = (FSensor_forcL[2]*CoPL[1] + FSensor_forcR[2]*CoPR[1]) / (FSensor_forcL[2] + FSensor_forcR[2]);			
	}		

	// 以下測試用
		//fstream Fx;
		//Fx.open("TestX_est.txt",ios::app);	
		//for(int i = 0 ; i < 12 ; i++)	
		//	Fx << x_est[i] << "\t";
		//Fx << endl;
		//Fx.close();

		//Fx.open("TestAdamsMotionT.txt",ios::app);		
		//for(int i = 0 ; i < 4 ; i++)
		//	Fx << AdamsMotionT[i*3] << "\t"<< AdamsMotionT[i*3+1] << "\t"<< AdamsMotionT[i*3+2]<< "\t";
		//Fx << endl;
		//Fx.close();
	
		//Fx.open("TestAdamsSimOriginal.txt",ios::app);		
		//for(int i = 0 ; i < 4 ; i++)
		//	Fx << AdamsFS[i*3] << "\t"<< AdamsFS[i*3+1] << "\t"<< AdamsFS[i*3+2]<< "\t";
		//Fx << endl;
		//Fx.close();
	
		//Fx.open("TestFSensor_forcL.txt",ios::app);		
		//	Fx << FSensor_forcL[0] << "\t"<< FSensor_forcL[1] << "\t"<< FSensor_forcL[2] << "\t";
		//Fx << endl;
		//Fx.close();

		//Fx.open("TestFSensor_forcR.txt",ios::app);		
		//Fx << FSensor_forcR[0] << "\t"<< FSensor_forcR[1] << "\t"<< FSensor_forcR[2] << "\t";
		//Fx << endl;
		//Fx.close();

		//Fx.open("TestFSensor_TorqL.txt",ios::app);		
		//Fx << FSensor_TorqL[0] << "\t"<< FSensor_TorqL[1] << "\t"<< FSensor_TorqL[2] << "\t";
		//Fx << endl;
		//Fx.close();

		//Fx.open("TestFSensor_TorqR.txt",ios::app);		
		//Fx << FSensor_TorqL[0] << "\t"<< FSensor_TorqL[1] << "\t"<< FSensor_TorqL[2] << "\t";
		//Fx << endl;
		//Fx.close();

		//Fx.open("TestselIK.txt",ios::app);		
		//Fx << selIK << "\t" << endl;
		//Fx.close();
		
}

void Kine::FindInertia2LocalCOM(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 目前放在FindDIni呼叫本函式 只呼叫一次
	// 以各桿件自己的座標表示該桿件的轉動慣量 如此每個桿件的轉動慣量是一個定值 只需計算一次
	// 若有時間須在校正 這裡的慣量都是錯的!
	// 請注意旋轉矩陣左乘右乘的問題Ic = Rt Ia R
	// 身體部分由於是上半身為一整塊 故需使用三維平行軸定理
	// 腳不用平行軸是因為當初慣量在catia下量測就是以該桿件COM為原點 只是座標軸為世界
	// 腳只取235軸作為坐標系是因為要坐標系固定在Link上就 所以由腰的Base往下要取235才能代表該COM的坐標系
	// 在這裡要將Theta先設為零取出Rn的原因是我們在世界下量的慣量是在機器人站直的時候 也就是Theta = 0 時
	// 改變Theta後記得將角度還原
	// 2012 Slongz Start
	// 20121218 Wei-Zh Lai Start 
	******************************************************************/
	// catia and solidworks 量到的每個桿件在世界下的 inertia matrix
	double IR_FPad[9] = {6000, -550.1, 2000, -550.1, 9000, 98.94, 2000, 98.94, 7000}; 
	double IL_FPad[9] = {6000, 546, 2000, 546, 9000, -97.5, 2000, -97.5, 7000}; 
	double IR_KneeDown[9] = {17000, 106.1, -383, 106.1, 16000, 1000, -383, 1000, 3000};
	double IL_KneeDown[9] = {17000, -132.1, -450.3, -132.1, 16000, -1000, -450.3, -1000, 3000};
	double IR_KneeUp[9] = {33000, -200.2, -191.1, -200.2, 28000, -889.7, -191.1, -889.7, 8000};
	double IL_KneeUp[9] = {33000, 196.2, -201.5, 196.2, 28000, 877.5, -201.5, 877.5, 8000};

	//double IR_FPad[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}; // 先disable
	//double IL_FPad[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}; // 先disable
	//double IR_KneeDown[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}; // 先disable
	//double IL_KneeDown[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}; // 先disable
	//double IR_KneeUp[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}; // 先disable
	//double IL_KneeUp[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}; // 先disable
	//// IB1  (肚子與腰)
	//double IB1[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}; // 先disable
	//// 胸部
	//double IB2[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}; // 先disable


	// IB1  (肚子與腰)
	double IB1[9] = {11000, -34.4, -70.96, -34.4, 11000, 445.7, -70.96, 445.7, 17000}; 
	// 胸部
	double IB2[9] = {184000, -1000, 1000, -1000, 201000, -4000, 1000, -4000, 173000};  

	double IR_Arm[9] = {42000, -18.45, -134.8, -18.45, 42000, 54.74, -134.8, 54.74, 2000};
	double IL_Arm[9] = {42000, -19.39, -136.3, -19.39, 42000, -76.87, -136.3, -76.87, 2000};	
	//double IR_Arm[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}; // 先disable 手臂，故意弄小
	//double IL_Arm[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}; // 先disable 手臂，故意弄小

	// 以上已放大10^6倍 需再放大1000倍(下面) 單位變成 g.mm^2 (catia 裡面是 kg.m^2 乘上10^9 就會變成 g.mm^2)
	
	// 先將theta設為0，取完DH之後 復原theta值，照常執行程式
	double* tempThLL;
	tempThLL = new double[FKLLeg->NumJoint];

	double* tempThRL;
	tempThRL = new double[FKRLeg->NumJoint];

	double* tempThLA;
	tempThLA = new double[FKLArm->NumJoint];

	double* tempThRA;
	tempThRA = new double[FKRArm->NumJoint];

	for (int i = 0 ; i < FKLLeg->NumJoint ; i++)
	{
		tempThLL[i] = FKLLeg->theta[i];
		FKLLeg->theta[i] = 0;
	}

	for (int i = 0 ; i < FKRLeg->NumJoint ; i++)
	{
		tempThRL[i] = FKRLeg->theta[i];
		FKRLeg->theta[i] = 0;
	}

	for (int i = 0 ; i < FKRArm->NumJoint ; i++)
	{
		tempThRA[i] = FKRArm->theta[i];
		FKRArm->theta[i] = 0;
	}

	for (int i = 0 ; i < FKLArm->NumJoint ; i++)
	{
		tempThLA[i] = FKLArm->theta[i];
		FKLArm->theta[i] = 0;
	}

	double temp_compute[9];
	double Rot_Part_Rn[9];

	// 先將theta設為0，取完DH之後 復原theta值，照常執行程式
	FKLLeg->DHConstruct();
	FKRLeg->DHConstruct();
	FKLArm->DHConstruct();
	FKRArm->DHConstruct();

	// 取出旋轉矩陣並且將轉動慣量轉到該桿件的COM座標下
	// 因為皆由腰的base往下長出DH 所以無論是左腳還是右腳support 取的旋轉座標皆為532
	////// IcLFoot = A_LL0(1:3,25:27)'*IL_FPad*A_LL0(1:3,25:27); // 第6軸 Rn5
	GetRotPartRn(&FKLLeg->Rn[5],Rot_Part_Rn);
	MatMulAB(IL_FPad,3,3,Rot_Part_Rn,3,3,temp_compute);
	MatMulAtB(Rot_Part_Rn,3,3,temp_compute,3,3,IcLFoot);

	////// IcLShank = A_LL0(1:3,17:19)'*IL_KneeDown*A_LL0(1:3,17:19); // 第4軸 Rn3
	GetRotPartRn(&FKLLeg->Rn[3],Rot_Part_Rn);
	MatMulAB(IL_KneeDown,3,3,Rot_Part_Rn,3,3,temp_compute);
	MatMulAtB(Rot_Part_Rn,3,3,temp_compute,3,3,IcLShank);

	//////IcLThigh =  A_LL0(1:3,13:15)'*IL_KneeUp*A_LL0(1:3,13:15); // 第3軸 Rn2
	GetRotPartRn(&FKLLeg->Rn[2],Rot_Part_Rn);
	MatMulAB(IL_KneeUp,3,3,Rot_Part_Rn,3,3,temp_compute);
	MatMulAtB(Rot_Part_Rn,3,3,temp_compute,3,3,IcLThigh);

	////// IcRFoot = A_RL0(1:3,25:27)'*IR_FPad*A_RL0(1:3,25:27); // 第6軸 Rn5
	GetRotPartRn(&FKRLeg->Rn[5],Rot_Part_Rn);
	MatMulAB(IR_FPad,3,3,Rot_Part_Rn,3,3,temp_compute);
	MatMulAtB(Rot_Part_Rn,3,3,temp_compute,3,3,IcRFoot);

	////// IcRShank = A_RL0(1:3,17:19)'*IR_Shank*A_RL0(1:3,17:19); // 第4軸 Rn3
	GetRotPartRn(&FKRLeg->Rn[3],Rot_Part_Rn);
	MatMulAB(IR_KneeDown,3,3,Rot_Part_Rn,3,3,temp_compute);
	MatMulAtB(Rot_Part_Rn,3,3,temp_compute,3,3,IcRShank);

	////// IcRThigh = A_RL0(1:3,13:15)'*IR_Thigh*A_RL0(1:3,13:15); // 第3軸 Rn2
	GetRotPartRn(&FKRLeg->Rn[2],Rot_Part_Rn);
	MatMulAB(IR_KneeUp,3,3,Rot_Part_Rn,3,3,temp_compute);
	MatMulAtB(Rot_Part_Rn,3,3,temp_compute,3,3,IcRThigh);

	// 在上半身慣量的部分 通通旋轉到腰第一軸的座標 因為上半身視作一坨的座標 也就是FKLArm->Rn[0]
	// 腰部 胸部 左手臂和右手臂皆轉到同一個坐標系下 也就是Leg->Rn[0] 故只在第一個(腰部)抓旋轉座標
	// 腰部
	GetRotPartRn(&FKLArm->Rn[0],Rot_Part_Rn);
	MatMulAB(IB1,3,3,Rot_Part_Rn,3,3,temp_compute);
	MatMulAtB(Rot_Part_Rn,3,3,temp_compute,3,3,IcLowBody);
	
	// 胸部
	//GetRotPartRn(&FKRLeg->Rn[0],Rot_Part_Rn);
	MatMulAB(IB2,3,3,Rot_Part_Rn,3,3,temp_compute);
	MatMulAtB(Rot_Part_Rn,3,3,temp_compute,3,3,IcUpBody);

	//////% 整隻右手臂  
	//GetRotPartRn(&FKRLeg->Rn[0],Rot_Part_Rn);
	MatMulAB(IR_Arm,3,3,Rot_Part_Rn,3,3,temp_compute);
	MatMulAtB(Rot_Part_Rn,3,3,temp_compute,3,3,IcRArm);
	
	//////% 整隻左手臂 
	//GetRotPartRn(&FKRLeg->Rn[0],Rot_Part_Rn);
	MatMulAB(IL_Arm,3,3,Rot_Part_Rn,3,3,temp_compute);
	MatMulAtB(Rot_Part_Rn,3,3,temp_compute,3,3,IcLArm);	
	// 以上計算方法與InitInertia相同 只是存的地方變成InerLc 如此一來 所有原本在世界下描述的慣量 皆轉換成以該桿件COM座標描述
	
	//Sigma I
	for(int i = 0 ; i < 9 ; i++)
		IcBody[i] = 0;
	MatAddAB(IcLowBody,IcBody , IcBody, 9);	
	MatAddAB(IcUpBody,IcBody , IcBody, 9);	
	MatAddAB(IcLArm,IcBody , IcBody, 9); 	
	MatAddAB(IcRArm,IcBody , IcBody, 9);
	
	// 再放大1000倍 使單位變成 g.mm^2 (catia 裡面是 kg.m^2 乘上10^9 就會變成 g.mm^2)
	for (int i = 0 ; i < 9 ; i++)
	{
		IcLFoot[i] *= 1000;
		IcLShank[i] *= 1000;
		IcLThigh[i] *= 1000;
		IcRFoot[i] *= 1000;
		IcRShank[i] *= 1000;
		IcRThigh[i] *= 1000;
		IcBody[i] *= 1000;
	}

	// 身體平行軸計算 IcUpBody = Sigma I + Parallel
	double TempVector1[3];
	double TempVector2[9];
	double TempVector3[9];
	double E[9]={1,0,0,0,1,0,0,0,1};	

	// 平行軸-IcLowBody
	MatScalarMul(pv_stack+42, 3, -1.0, TempVector1); 
	MatAddAB(BodyRCOM, TempVector1, TempVector1, 3);	// R
	MatMulAB(TempVector1,1,3,TempVector1,3,1,TempVector2);	// R‧R
	MatScalarMul(E, 9, TempVector2, TempVector2);	// (R‧R)*E

	MatMulAB(TempVector1,3,1,TempVector1,1,3,TempVector3);	// R outerproduct R
	MatScalarMul(TempVector3, 9, -1.0, TempVector3);	// -(R outerproduct R)
	MatAddAB( TempVector2, TempVector3 , TempVector3 , 9);	// (R‧R) * E - (R outerproduct R)
	MatScalarMul(TempVector3, 9, 2796, TempVector3);	// m*((R‧R) * E - (R outerproduct R))

	MatAddAB(TempVector3, IcBody, IcBody, 9);

	// 平行軸-IcUpBody
	MatScalarMul(pv_stack+45, 3, -1.0, TempVector1); 
	MatAddAB(BodyRCOM, TempVector1 , TempVector1, 3);	// R
	MatMulAB(TempVector1,1,3,TempVector1,3,1,TempVector2);	// R‧R
	MatScalarMul(E, 9, TempVector2, TempVector2);	// (R‧R)*E

	MatMulAB(TempVector1,3,1,TempVector1,1,3,TempVector3);	// R outerproduct R
	MatScalarMul(TempVector3, 9, -1.0, TempVector3);	// -(R outerproduct R)
	MatAddAB( TempVector2, TempVector3 , TempVector3 , 9);	// (R‧R) * E - (R outerproduct R)
	MatScalarMul(TempVector3, 9, 22952, TempVector3);	// m*((R‧R) * E - (R outerproduct R))
	
	MatAddAB(TempVector3, IcBody, IcBody, 9);

	// 平行軸-IcLArm 
	MatScalarMul(LArmCOM, 3, -1.0, TempVector1); 
	MatAddAB(BodyRCOM,TempVector1 , TempVector1, 3);	// R
	MatMulAB(TempVector1,1,3,TempVector1,3,1,TempVector2);	// R‧R
	MatScalarMul(E, 9, TempVector2, TempVector2);	// R‧R*E

	MatMulAB(TempVector1,3,1,TempVector1,1,3,TempVector3);	// R outerproduct R
	MatScalarMul(TempVector3, 9, -1.0, TempVector3);	// R outerproduct R*-1
	MatAddAB( TempVector2, TempVector3 , TempVector3 , 9);	// R‧R * E - 1 * R outerproduct R
	MatScalarMul(TempVector3, 9, 2496, TempVector3);	// m*(R‧R * E - 1 * R outerproduct R)
	
	MatAddAB(TempVector3, IcBody, IcBody, 9);

	// 平行軸-IcRArm
	MatScalarMul(RArmCOM, 3, -1.0, TempVector1); 
	MatAddAB(BodyRCOM,TempVector1 , TempVector1, 3);	// R
	MatMulAB(TempVector1,1,3,TempVector1,3,1,TempVector2);	// R‧R
	MatScalarMul(E, 9, TempVector2, TempVector2);	// R‧R*E

	MatMulAB(TempVector1,3,1,TempVector1,1,3,TempVector3);	// R outerproduct R
	MatScalarMul(TempVector3, 9, -1.0, TempVector3);	// R outerproduct R*-1
	MatAddAB( TempVector2, TempVector3 , TempVector3 , 9);	// R‧R * E - 1 * R outerproduct R
	MatScalarMul(TempVector3, 9, 2496, TempVector3);	// m*(R‧R * E - 1 * R outerproduct R)
	
	MatAddAB(TempVector3, IcBody, IcBody, 9);
	
	// 計算完成後復原Theta值
	for (int i = 0 ; i < FKLLeg->NumJoint ; i++)
	{
		FKLLeg->theta[i] = tempThLL[i];
	}
	delete[] tempThLL;

	for (int i = 0 ; i < FKRLeg->NumJoint ; i++)
	{
		FKRLeg->theta[i] = tempThRL[i];
	}
	delete[] tempThRL;

	for (int i = 0 ; i < FKLArm->NumJoint ; i++)
	{
		FKLArm->theta[i] = tempThLA[i];
	}
	delete[] tempThLA;

	for (int i = 0 ; i < FKRArm->NumJoint ; i++)
	{
		FKRArm->theta[i] = tempThRA[i];
	}
	delete[] tempThRA;

	//// 以下測試用

	//fstream Fx;
	//Fx.open("TestIcLFoot.txt",ios::app);		
	//for(int i = 0 ; i < 9 ; i++)
	//Fx << IcLFoot[i] << "\t";
	//Fx << endl;
	//Fx.close();

	//Fx.open("TestIcLShank.txt",ios::app);		
	//for(int i = 0 ; i < 9 ; i++)
	//Fx << IcLShank[i] << "\t";
	//Fx << endl;
	//Fx.close();

	//Fx.open("TestIcLThigh.txt",ios::app);		
	//for(int i = 0 ; i < 9 ; i++)
	//Fx << IcLThigh[i] << "\t";
	//Fx << endl;
	//Fx.close();

	//Fx.open("TestIcRFoot.txt",ios::app);		
	//for(int i = 0 ; i < 9 ; i++)
	//Fx << IcRFoot[i] << "\t";
	//Fx << endl;
	//Fx.close();

	//Fx.open("TestIcRShank.txt",ios::app);		
	//for(int i = 0 ; i < 9 ; i++)
	//Fx << IcRShank[i] << "\t";
	//Fx << endl;
	//Fx.close();

	//Fx.open("TestIcRThigh.txt",ios::app);		
	//for(int i = 0 ; i < 9 ; i++)
	//Fx << IcRThigh[i] << "\t";
	//Fx << endl;
	//Fx.close();

	//Fx.open("TestIcBody.txt",ios::app);		
	//for(int i = 0 ; i < 9 ; i++)
	//Fx << IcBody[i] << "\t";
	//Fx << endl;
	//Fx.close();
}

void Kine::kalmanfilter(double  datanow, double  datafilterbefore ,  double *result ){

	double  Q1 ,R1 ,P1, P_last1= 0;
	double x_est_last1 = 0 ;
	double 	 x_temp_est1 , Kinf,  P_temp1 ,x_est1,z_measured1  ;
	
	Q1 =  0.08   ;       //0.8 //0.004       // Q為model的covariance 越小越相信
	R1 =   1.60311  ;       //0.7976  	// R為measuremant的covariance 越小越相信
	
	z_measured1 = datanow;
	x_est_last1 = datafilterbefore ;
	
			
        //do a prediction
			x_temp_est1 = x_est_last1;
			P_temp1 = P_last1 + Q1;
        //calculate the Kalman gain
			Kinf = P_temp1 * (1.0/(P_temp1 + R1));
        //measure
			//z_measured[i] = AdamsSim[i] ; //the measurement with noise
        //correct
			x_est1 = x_temp_est1 + Kinf * ( z_measured1 - x_temp_est1); 
			P1 = (1- Kinf) * P_temp1;
     
			*result = x_est1; // 存入指定buffer中


        //update our last's
			P_last1 = P1;
			x_est_last1 = x_est1;
    }




void Kine::smoothfilter(double  *data,int count )
{
	if (count<6){
		data[count] = data[count];
	}
	else{
		data[count] = ( data[count]+data[count-1]+data[count-2]+data[count-3]+data[count-4])/5;
	}
}


//void Kine::InfaredSensorData(double ADCL1, double ADCL2, double ADCL3 , double ADCL4, double ADCR1, double ADCR2, double ADCR3, double ADCR4)
//{
//
//	/******************************************************************
//	input: void
//	output: void
//
//	Note:
//	// kalman filter 後轉成距離值
//		
//	******************************************************************/
//	 //哲軒 cali後 以二次式回歸線 已經將CALI量減掉
//	//Lleg
//	InfraredLdisdata1[infraredcount]= 10*(1.253679629673433e-09*pow(ADCL1,4) -3.258294125232162e-06* pow(ADCL1,3)  +0.003160432554155*pow(ADCL1,2)-1.369255013228468* ADCL1 + 2.322886274302908e+02  -4.91)  -infraredbiasL1 ;
//	InfraredLdisdata2[infraredcount]= 10*(1.223335087293077e-09*pow(ADCL2,4) -3.209265925492400e-06* pow(ADCL2,3)  +0.003142425968186*pow(ADCL2,2)-1.374125552950027* ADCL2 + 2.350436607553530e+02  -5.08)  -infraredbiasL2 ; 
//	InfraredLdisdata3[infraredcount]= 10*(1.578808061060311e-09*pow(ADCL3,4) -4.120075043889547e-06* pow(ADCL3,3)  +0.004006512966092*pow(ADCL3,2)-1.733945969593775* ADCL3  + 2.906085780964760e+02 -5.44)  -infraredbiasL3 ; 
//	InfraredLdisdata4[infraredcount]= 10*(1.143638986829752e-09*pow(ADCL4,4) -2.951748842011788e-06* pow(ADCL4,3)  +0.002845758481195*pow(ADCL4,2)-1.227874742044600* ADCL4  + 2.086928652598685e+02 -4.96)  -infraredbiasL4 ;
//	
//	//Rleg
//	InfraredRdisdata1[infraredcount]= 9.027517774714052e-09*pow(ADCR1,4) -2.288662961368428e-05* pow(ADCR1,3)  +0.021681781641444*pow(ADCR1,2)-9.209258839239556* ADCR1 + 1.559644270664676e+03 -infraredbiasR1 ;
//	//1.843830231765809e-08*pow(ADCR1,4) -4.185243259522504e-05* pow(ADCR1,3)  +0.035447361383679*pow(ADCR1,2)-13.409418860060434* ADCR1 + 2.000966366063756e+03    ;
//	InfraredRdisdata2[infraredcount]= 9.027517774714052e-09*pow(ADCR2,4) -2.288662961368428e-05* pow(ADCR2,3)  +0.021681781641444*pow(ADCR2,2)-9.209258839239556* ADCR2 + 1.559644270664676e+03 -infraredbiasR2 ;
//	//3.165817666750415e-09*pow(ADCR2,4) -8.551693903425129e-06 *pow(ADCR2,3)  +0.008750567279166*pow(ADCR2,2)-4.125282137825477* ADCR2 + 8.340353011265341e+02     ;  
//	InfraredRdisdata3[infraredcount]= 9.027517774714052e-09*pow(ADCR3,4) -2.288662961368428e-05* pow(ADCR3,3)  +0.021681781641444*pow(ADCR3,2)-9.209258839239556* ADCR3 + 1.559644270664676e+03 -infraredbiasR3 ;
//	InfraredRdisdata4[infraredcount]= 1.253679629673433e-09*pow(ADCR4,4) -3.258294125232162e-06* pow(ADCR4,3)  +0.003160432554155*pow(ADCR4,2)-1.369255013228468* ADCR4 + 2.322886274302908e+02 -infraredbiasR4 ;	  
//
//	infraredcount++;
//
//	if (infraredcount > infraedbuffersize ){ 
//		infraredcount = infraedbuffersize ;   
//	}
//}

void Kine::PhsIndex(double StepNumb,double StepCount, double InitialCount, double EndCount, double Scale)
{
	for (int i=0;i<InitialCount;i++)
	{
		SupportPhs[i]=-1*Scale;
	}
	for (int i=0;i<StepNumb-3;i++)
	{
		for(int j=0;j<StepCount;j++)
		{
			//if(selSupport[i]==2||selSupport[i]==1)
			if(i%2==0)
			SupportPhs[int(i*StepCount+InitialCount+j)]=Scale;
			else
			SupportPhs[int(i*StepCount+InitialCount+j)]=-1*Scale;
		}
	}
	//for (int i=0;i<EndCount;i++)
	//{
	//	SupportPhs[int(i+InitialCount+(StepNumb-3)*StepCount)]=-1*Scale;
	//}
	if(PhsFlag==0)
	{
		system("delete EncPhs.txt");
		fstream Phs;
		Phs.open("EncPhs.txt",ios::app);
		for(int i=0;i<((StepNumb-3)*StepCount+InitialCount+EndCount);i++)
		{						  
			Phs<<SupportPhs[i]<<"\t";
		}
		Phs.close();
	}
}

void Kine::FindEncCOG(double* Enc_theta, double* Enc_COG)
{	
	/******************************************************************
	input: Enc_theta
	output: void

	Note:
	// 線上收值或是離線匯入enc文字檔
	// 再藉由FindFK FindCOG來得到DHmodel中的COG位置
	// 在這裡會利用參數記憶下原本的值 (EX平移位置 旋轉矩陣...etc)
	// 須注意在最後要把系統計算出的IK還給系統
	// 在回算ENC時不更新OpenGL
	// 函式放在IKStep後面 所以在換腳時的count數要注意
	// 20130911 WZ
	******************************************************************/
	double temp_theta[12];	// 暫存理想角度
	int	WriteIndex;

	// 取出理想的角度位置 並且丟入回傳的Enc轉角度 算FK COG
	for (int h = 0 ; h < 6 ; h++){	
		temp_theta[h] = FKLLeg->theta[h+1];	// 左腳
		FKLLeg->theta[h+1] = Enc_theta[h];
		temp_theta[h+6] = FKRLeg->theta[h+1];	// 右腳
		FKRLeg->theta[h+1] = Enc_theta[h+6];	
	}
	// forward kinematics
	FKLLeg->DHConstruct();
	FKRLeg->DHConstruct();
	FKLArm->DHConstruct();
	FKRArm->DHConstruct();

	// 取出腳底板旋轉矩陣 並且將整隻機器人旋轉至適當方向
	if (selSupport[Enc_stepIndex] == LeftSupport){
		// x axis -> joint 9 - joint 10
		FixEndEffRMot[0] = (FKLLeg->Rn[9].data[3]-FKLLeg->Rn[10].data[3])/LenEdgeXYZ[0];
		FixEndEffRMot[4] = (FKLLeg->Rn[9].data[7]-FKLLeg->Rn[10].data[7])/LenEdgeXYZ[0];
		FixEndEffRMot[8] = (FKLLeg->Rn[9].data[11]-FKLLeg->Rn[10].data[11])/LenEdgeXYZ[0];

		// y axis -> joint 11 - joint 10
		FixEndEffRMot[1] = (FKLLeg->Rn[11].data[3]-FKLLeg->Rn[10].data[3])/LenEdgeXYZ[1];
		FixEndEffRMot[5] = (FKLLeg->Rn[11].data[7]-FKLLeg->Rn[10].data[7])/LenEdgeXYZ[1];
		FixEndEffRMot[9] = (FKLLeg->Rn[11].data[11]-FKLLeg->Rn[10].data[11])/LenEdgeXYZ[1];

		// z axis -> joint 6 - joint 7
		FixEndEffRMot[2] = (FKLLeg->Rn[6].data[3]-FKLLeg->Rn[7].data[3])/LenEdgeXYZ[2];
		FixEndEffRMot[6] = (FKLLeg->Rn[6].data[7]-FKLLeg->Rn[7].data[7])/LenEdgeXYZ[2];
		FixEndEffRMot[10] = (FKLLeg->Rn[6].data[11]-FKLLeg->Rn[7].data[11])/LenEdgeXYZ[2];

		for (int i = 0 ; i < LegDHLen; i++){
			MatMulAtB(FixEndEffRMot,4,4,FKLLeg->Rn[i].data,4,4,temp44MatMul);
			MatMulAB(Enc_LSwitchRMot,4,4,temp44MatMul,4,4,FKLLeg->Rn[i].data);
		}
		for (int i = 0 ; i < LegDHLen; i++){
			MatMulAtB(FixEndEffRMot,4,4,FKRLeg->Rn[i].data,4,4,temp44MatMul);
			MatMulAB(Enc_LSwitchRMot,4,4,temp44MatMul,4,4,FKRLeg->Rn[i].data);
		}
		for (int i = 0 ; i < ArmDHLen; i++){
			MatMulAtB(FixEndEffRMot,4,4,FKLArm->Rn[i].data,4,4,temp44MatMul);
			MatMulAB(Enc_LSwitchRMot,4,4,temp44MatMul,4,4,FKLArm->Rn[i].data);
		}
		for (int i = 0 ; i < ArmDHLen; i++){
			MatMulAtB(FixEndEffRMot,4,4,FKRArm->Rn[i].data,4,4,temp44MatMul);
			MatMulAB(Enc_LSwitchRMot,4,4,temp44MatMul,4,4,FKRArm->Rn[i].data);
		}
	}
	else if (selSupport[Enc_stepIndex] == RightSupport || selSupport[Enc_stepIndex] == DoubleSupport){
		// x axis -> joint 8 - joint 11
		FixEndEffRMot[0] = (FKRLeg->Rn[8].data[3]-FKRLeg->Rn[11].data[3])/LenEdgeXYZ[0];
		FixEndEffRMot[4] = (FKRLeg->Rn[8].data[7]-FKRLeg->Rn[11].data[7])/LenEdgeXYZ[0];
		FixEndEffRMot[8] = (FKRLeg->Rn[8].data[11]-FKRLeg->Rn[11].data[11])/LenEdgeXYZ[0];

		// y axis -> joint 10 - joint 11
		FixEndEffRMot[1] = (FKRLeg->Rn[10].data[3]-FKRLeg->Rn[11].data[3])/LenEdgeXYZ[1];
		FixEndEffRMot[5] = (FKRLeg->Rn[10].data[7]-FKRLeg->Rn[11].data[7])/LenEdgeXYZ[1];
		FixEndEffRMot[9] = (FKRLeg->Rn[10].data[11]-FKRLeg->Rn[11].data[11])/LenEdgeXYZ[1];

		// z axis -> joint 6 - joint 7
		FixEndEffRMot[2] = (FKRLeg->Rn[6].data[3]-FKRLeg->Rn[7].data[3])/LenEdgeXYZ[2];
		FixEndEffRMot[6] = (FKRLeg->Rn[6].data[7]-FKRLeg->Rn[7].data[7])/LenEdgeXYZ[2];
		FixEndEffRMot[10] = (FKRLeg->Rn[6].data[11]-FKRLeg->Rn[7].data[11])/LenEdgeXYZ[2];

		for (int i = 0 ; i < LegDHLen; i++){
			MatMulAtB(FixEndEffRMot,4,4,FKLLeg->Rn[i].data,4,4,temp44MatMul);
			MatMulAB(Enc_RSwitchRMot,4,4,temp44MatMul,4,4,FKLLeg->Rn[i].data);
		}
		for (int i = 0 ; i < LegDHLen; i++){
			MatMulAtB(FixEndEffRMot,4,4,FKRLeg->Rn[i].data,4,4,temp44MatMul);
			MatMulAB(Enc_RSwitchRMot,4,4,temp44MatMul,4,4,FKRLeg->Rn[i].data);
		}
		for (int i = 0 ; i < ArmDHLen; i++){
			MatMulAtB(FixEndEffRMot,4,4,FKLArm->Rn[i].data,4,4,temp44MatMul);
			MatMulAB(Enc_RSwitchRMot,4,4,temp44MatMul,4,4,FKLArm->Rn[i].data);
		}
		for (int i = 0 ; i < ArmDHLen; i++){
			MatMulAtB(FixEndEffRMot,4,4,FKRArm->Rn[i].data,4,4,temp44MatMul);
			MatMulAB(Enc_RSwitchRMot,4,4,temp44MatMul,4,4,FKRArm->Rn[i].data);
		}
	}
	// selIK = 0 : left foot is the supporter
    // selIK = 1 : right foot is the supporter

	// 旋轉完後取出所有軸點座標
	WriteIndex = 0;
	for (int i = 0 ; i < LegDHLen; i++){
		CrdAll->data[WriteIndex] = FKLLeg->Rn[i].data[3];
		CrdAll->data[WriteIndex+1] = FKLLeg->Rn[i].data[7];
		CrdAll->data[WriteIndex+2] = FKLLeg->Rn[i].data[11];
		WriteIndex += 3;
	}
	for (int i = 0 ; i < LegDHLen; i++){
		CrdAll->data[WriteIndex] = FKRLeg->Rn[i].data[3];
		CrdAll->data[WriteIndex+1] = FKRLeg->Rn[i].data[7];
		CrdAll->data[WriteIndex+2] = FKRLeg->Rn[i].data[11];
		WriteIndex += 3;
	}
	for (int i = 0 ; i < ArmDHLen; i++){
		CrdAll->data[WriteIndex] = FKLArm->Rn[i].data[3];
		CrdAll->data[WriteIndex+1] = FKLArm->Rn[i].data[7];
		CrdAll->data[WriteIndex+2] = FKLArm->Rn[i].data[11];
		WriteIndex += 3;
	}
	for (int i = 0 ; i < ArmDHLen; i++){
		CrdAll->data[WriteIndex] = FKRArm->Rn[i].data[3];
		CrdAll->data[WriteIndex+1] = FKRArm->Rn[i].data[7];
		CrdAll->data[WriteIndex+2] = FKRArm->Rn[i].data[11];
		WriteIndex += 3;
	}

	// Move to the fixed point
	WriteIndex = 0;
	if(FirstEncCOG==true){
		if (selSupport[Enc_stepIndex] == LeftSupport){
			RobotFixVector[0] = Enc_shiftLL[0]-CrdAll->data[21];
			RobotFixVector[1] = Enc_shiftLL[1]-CrdAll->data[22];
			RobotFixVector[2] = Enc_shiftLL[2]-CrdAll->data[23];
		}
		else if (selSupport[Enc_stepIndex] == RightSupport || selSupport[Enc_stepIndex] == DoubleSupport){
			RobotFixVector[0] = Enc_shiftRL[0]-CrdAll->data[60];
			RobotFixVector[1] = Enc_shiftRL[1]-CrdAll->data[61];
			RobotFixVector[2] = Enc_shiftRL[2]-CrdAll->data[62];
		}
		for (int i = 0 ; i < LegDHLen*2+ArmDHLen*2; i++){
			CrdAll->data[WriteIndex] += RobotFixVector[0];
			CrdAll->data[WriteIndex+1] += RobotFixVector[1];
			CrdAll->data[WriteIndex+2] += RobotFixVector[2];
			WriteIndex += 3;
		}	
	}
	else{	// 第一次找FK
		FirstEncCOG = true;
		if (selSupport[Enc_stepIndex] == LeftSupport){
			RobotFixVector[0] = 0-CrdAll->data[21];
			RobotFixVector[1] = 0.5*(CrdAll->data[13]-CrdAll->data[52])-CrdAll->data[22];
			RobotFixVector[2] = 0-CrdAll->data[23];		
		}
		else if (selSupport[Enc_stepIndex] == RightSupport || selSupport[Enc_stepIndex] == DoubleSupport){
			RobotFixVector[0] = 0-CrdAll->data[60];
			RobotFixVector[1] = -0.5*(CrdAll->data[13]-CrdAll->data[52])-CrdAll->data[61];
			RobotFixVector[2] = 0-CrdAll->data[62];	
		}
		for (int i = 0 ; i < LegDHLen*2+ArmDHLen*2; i++){
			CrdAll->data[WriteIndex] += RobotFixVector[0];
			CrdAll->data[WriteIndex+1] += RobotFixVector[1];
			CrdAll->data[WriteIndex+2] += RobotFixVector[2];
			WriteIndex += 3;
		}				
		Enc_shiftRL[0] = CrdAll->data[60];
		Enc_shiftRL[1] = CrdAll->data[61];
		Enc_shiftRL[2] = CrdAll->data[62];
			
		//Enc_shiftLL[0] = CrdAll->data[21];
		//Enc_shiftLL[1] = CrdAll->data[22];
		//Enc_shiftLL[2] = CrdAll->data[23];
	}
		DHOrigin[0] = RobotFixVector[0];
		DHOrigin[1] = RobotFixVector[1];
		DHOrigin[2] = RobotFixVector[2];

	FindCOG();

	for (int i = 0 ; i < 3 ; i++){
		Enc_COG[i] = COG[i];	// Enc_COG為FK COG算出的ENCCOG
	}
	//////////////////////////////////////////////////////////////////////////
	//下一步要換腳
	if (gIthIK != 0 && (gIthIK) % gStepSample == 0){	//因為放在IKStep後面gIthIK已經加過1了 這裡只要用gIthIK即可
		Enc_stepIndex += 1;	// 下一步的SUP狀態
		if (selSupport[Enc_stepIndex] == LeftSupport){	//下一步是左腳SUP CrdAll->data[21]左腳腳底板
			Enc_shiftLL[0] = CrdAll->data[21];
			Enc_shiftLL[1] = CrdAll->data[22];
			Enc_shiftLL[2] = CrdAll->data[23];
		}
		else if (selSupport[Enc_stepIndex] == RightSupport || selSupport[Enc_stepIndex] == DoubleSupport){
			Enc_shiftRL[0] = CrdAll->data[60];
			Enc_shiftRL[1] = CrdAll->data[61];
			Enc_shiftRL[2] = CrdAll->data[62];
		}
		GetLegsCoords();
		Enc_LSwitchRMot[0] = LLegRotM[0];
		Enc_LSwitchRMot[1] = LLegRotM[1];
		Enc_LSwitchRMot[2] = LLegRotM[2];
		Enc_LSwitchRMot[4] = LLegRotM[3];
		Enc_LSwitchRMot[5] = LLegRotM[4];
		Enc_LSwitchRMot[6] = LLegRotM[5];
		Enc_LSwitchRMot[8] = LLegRotM[6];
		Enc_LSwitchRMot[9] = LLegRotM[7];
		Enc_LSwitchRMot[10] = LLegRotM[8];
		Enc_RSwitchRMot[0] = RLegRotM[0];
		Enc_RSwitchRMot[1] = RLegRotM[1];
		Enc_RSwitchRMot[2] = RLegRotM[2];
		Enc_RSwitchRMot[4] = RLegRotM[3];
		Enc_RSwitchRMot[5] = RLegRotM[4];
		Enc_RSwitchRMot[6] = RLegRotM[5];
		Enc_RSwitchRMot[8] = RLegRotM[6];
		Enc_RSwitchRMot[9] = RLegRotM[7];
		Enc_RSwitchRMot[10] = RLegRotM[8];
	}

	//UpdateDrawingBuffer(); // 打開看見ENC 火柴人 FindFK()中的此函數要關掉!

	//////////////////////////////////////////////////////////////////////////
	// 還給系統FK COG 為理想的軌跡
	for (int h = 0 ; h < 6 ; h++){	
		FKLLeg->theta[h+1] = temp_theta[h];	// 左腳	
		FKRLeg->theta[h+1] = temp_theta[h+6];	// 右腳		
	}
	
	FindFK();
	FindCOG();
	// 還給系統FK COG 為理想的軌跡

}


void Kine::Distributor(double *ZMPxd, double *ZMPyd, double *StepX, double *StepY, int StepNum, int FKCount, int IKCount, int HomePos)
{
	/******************************************************************
	input: 
	output: void

	Note:	
	// DSPr = ZMP橫向移動向量
	// ZMP計算時沒有包括一開始的FK 要再依homing情況加入點數!!!!!!!!!!
	// StepNum = IK點數
	// FKCount = 開頭FK點數
	// swing腳離開地面在1000(FK)+180+480+180-45(Nwait) 並不是在Nab才開始抬腳
	// 20131011 WZ
	******************************************************************/
	double A[8*3];	
	double forceratio[225];
	double F_DSPratio[180*2];
	double CotPoint[16];
	double GAF[3];
	double pinv_temp[9];
	double pinv_A_w[8*3];
	double temp[8*3];
	double force_temp[3];
	double A_SSP[4*3];
	double pinv_SSP[4*3];
	double F_dis[8];
	double ZMP[3];
	double Q0[8*8];
	int HomeCount;
	double ratioL;
	double ratioR;
	double A_w[8*3];
	int end_step = 100;

	bool Recordfile = false;

	if (Recordfile == true)
	{
		system("del Testratio.txt");
		system("del ratio.txt");
		system("del Testforce.txt");
	}

	ZMP[0] = 0;
	ZMP[1] = 0;
	ZMP[2] = 1;

	GAF[0] = 0;
	GAF[1] = 0;
	GAF[2] = SumMass*GravityConst;	
	// L leg	
	CotPoint[0] = 114;CotPoint[8] = 72;		CotPoint[1] = 114;CotPoint[9] = -68;
	CotPoint[3] = -96;CotPoint[11] = 72;	CotPoint[2] = -96;CotPoint[10] = -68;
	// R leg
	CotPoint[5] = 114;CotPoint[13] = 68;	CotPoint[4] = 114;CotPoint[12] = -72;
	CotPoint[6] = -96;CotPoint[14] = 68;	CotPoint[7] = -96;CotPoint[15] = -72;

	for (int m = 0 ; m < 8 ; m++){	// Q0對角矩陣 對角值為forceratio開根號
		for (int n = 0 ; n < 8 ; n++){
			if(m == n){
				Q0[8*m+n] = 1;}
			else{
				Q0[8*m+n] = 0;}
		}
	}

	if (FlagStayMode == 1) // stay 模式，機器人理應是腳伸直 並且直接結束
	{
			HomeCount = 300;
	}
	else // 正常模式 由jp與kp分配軌跡count數
	{
		if (HomePos == ZeroHome)
		{
			HomeCount = 1000;
		}
		else if (HomePos == BentHome)
		{
			HomeCount = 600;
		}
		else if (HomePos == ShiftZeroHome)
		{
			HomeCount = 1000;
		}
		else if (HomePos == SlopeHome)
		{
			HomeCount = 1000;
		}
	}

	for (int k = 0 ; k < 4 ; k++)
	{						
		A[k] = CotPoint[k];			A[4+k] = CotPoint[k+4];
		A[8+k] = CotPoint[k+8]+80;	A[12+k] = CotPoint[k+12]-80;
		A[16+k] = 1;				A[20+k] = 1;
	}	
	
	MatScalarMul(ZMP,3,GAF[2],force_temp);

	MatMulABt(A,3,8,A,3,8,pinv_temp);
	InvSqMat(pinv_temp,3);
	MatMulAtB(A,3,8,pinv_temp,3,3,pinv_A_w);

	MatMulAB(pinv_A_w,8,3,force_temp,3,1,F_dis);

	F_total[0] = F_dis[0]+F_dis[1]+F_dis[2]+F_dis[3];	// L Force
	F_total[1] = F_dis[4]+F_dis[5]+F_dis[6]+F_dis[7];	// R Force

	T_total[0] = -1*( (F_dis[0]+F_dis[1])*CotPoint[0] + (F_dis[2]+F_dis[3])*CotPoint[3] );	// L Pitch
	T_total[1] = ( (F_dis[0]+F_dis[3])*CotPoint[8] + (F_dis[1]+F_dis[2])*CotPoint[9] );		// L Roll
	T_total[2] =( (F_dis[4]+F_dis[5])*CotPoint[5] + (F_dis[6]+F_dis[7])*CotPoint[6] );		// R Pitchi
	T_total[3] = -1*( (F_dis[5]+F_dis[6])*CotPoint[13] + (F_dis[4]+F_dis[7])*CotPoint[12] );	// R Roll

	fstream QQ;	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	if (FlagStayMode == 1 || selSupport[2] == DoubleSupport)	// 不會解IK
	{
		for (int j = 1 ; j < FKCount+IKCount+HomeCount ; j++)
		{
			F_total[j*2] = F_total[0];
			F_total[j*2+1] = F_total[1];
			T_total[j*4] = T_total[0];
			T_total[j*4+1] = T_total[1];
			T_total[j*4+2] = T_total[2];
			T_total[j*4+3] = T_total[3];
			
			if (Recordfile == true)
			{			
				QQ.open("Testforce.txt",ios::app);
				for(int p = 0 ; p < 8 ; p++){
					QQ << F_dis[p]<< "\t";
				}
				QQ<< endl;
				QQ.close();
			}

		}
	}
	else
	{
		for (int j = 1 ; j < FKCount+Nza+Nab ; j++)	// 開始都是零 FK~IK[0~gNab]確定是零 sure???????????????????????????
		{
			F_total[j*2] = F_total[0];
			F_total[j*2+1] = F_total[1];
			T_total[j*4] = T_total[0];
			T_total[j*4+1] = T_total[1];
			T_total[j*4+2] = T_total[2];
			T_total[j*4+3] = T_total[3];

			if (Recordfile == true)
			{
				QQ.open("Testforce.txt",ios::app);
				for(int p = 0 ; p < 8 ; p++){
					QQ << F_dis[p]<< "\t";
				}
				QQ<< endl;
				QQ.close();
			}
		}
	
		for (int i = 0 ; i < end_step ; i++)
		{
			//// 分配ratio
			if (i == 0)	// 第一步特別處理
			{	
				if (selSupport[i+1] == LeftSupport)	// 下一步是左腳SUP
				{
					for (int k = 0 ; k < 4 ; k++)
					{
						A[k] = CotPoint[k] + StepX[i+1];		A[4+k] = CotPoint[k+4] + StepX[i+1];
						A[8+k] = CotPoint[k+8] + StepY[i+1];	A[12+k] = CotPoint[k+12] - StepY[i+1];
					}
					Gen7DegPolyMod(0.5,1,225,forceratio);	
				}
				else if (selSupport[i+1] == RightSupport)	// 下一步是右腳SUP
				{
					for (int k = 0 ; k < 4 ; k++)
					{
						A[k] = CotPoint[k] + StepX[i+1];		A[4+k] = CotPoint[k+4] + StepX[i+1];
						A[8+k] = CotPoint[k+8] - StepY[i+1];	A[12+k] = CotPoint[k+12] + StepY[i+1];
					}
					Gen7DegPolyMod(0.5,0,225,forceratio);	
				}
			}		
			else
			{
				if (selSupport[i+1] == LeftSupport)	// 下一步是左腳
				{
					for (int k = 0 ; k < 4 ; k++)
					{
						A[k] = CotPoint[k] + StepX[i+1];		A[4+k] = CotPoint[k+4] + StepX[i];
						A[8+k] = CotPoint[k+8] + StepY[i+1];	A[12+k] = CotPoint[k+12] + StepY[i];
					}
					Gen7DegPolyMod(0,1,225,forceratio);
				} 
				else if (selSupport[i+1] == RightSupport)	// 下一步是右腳
				{
					for (int k = 0 ; k < 4 ; k++)
					{						
						A[k] = CotPoint[k] + StepX[i];		A[4+k] = CotPoint[k+4] + StepX[i+1];
						A[8+k] = CotPoint[k+8] + StepY[i];	A[12+k] = CotPoint[k+12] + StepY[i+1];
					}
					Gen7DegPolyMod(1,0,225,forceratio);
				}
				else if (selSupport[i+1] == DoubleSupport)	// 下一步是最後一步
				{
					end_step = i+1;
					if (selSupport[i] == LeftSupport)	// 這一步是左腳
					{
						for (int k = 0 ; k < 4 ; k++)
						{
							A[k] = CotPoint[k] + StepX[i];		A[4+k] = CotPoint[k+4] + StepX[i];
							A[8+k] = CotPoint[k+8] + StepY[i];	A[12+k] = CotPoint[k+12] - StepY[i];
						}
						Gen7DegPolyMod(1,0.5,180*2,F_DSPratio);
					}
					else if (selSupport[i] == RightSupport)	// 這一步是右腳
					{
						for (int k = 0 ; k < 4 ; k++)
						{
							A[k] = CotPoint[k] + StepX[i];		A[4+k] = CotPoint[k+4] + StepX[i];
							A[8+k] = CotPoint[k+8] - StepY[i];	A[12+k] = CotPoint[k+12] + StepY[i];
						}
						Gen7DegPolyMod(0,0.5,180*2,F_DSPratio);
					}
				}		
			}		
			//// 分配ratio

			if (Recordfile == true)
			{			
				QQ.open("ratio.txt",ios::app);
				for(int p = 0 ; p < 225 ; p++){
					QQ << forceratio[p]<< "\t";
				}		
				QQ.close();
			}


			//// 開始解pseudo inverse
			for (int j = i*gStepSample+Nza+Nab ; j < (i+1)*gStepSample+Nza+Nab ; j++)
			{	
				if (j < (end_step+1)*gStepSample-Nab-Nzb)	// 為了最後DSP的Nab直接指定
				{
					ZMP[0] = ZMPyd[j];
					ZMP[1] = ZMPxd[j];
					if (j >= end_step*gStepSample - Nzb)	// final DSP
					{
						ratioL = 1/(2*F_DSPratio[j-(end_step*gStepSample - Nzb)]);	// L
						ratioR = 1/(2*(1-F_DSPratio[j-(end_step*gStepSample - Nzb)]));	// R

						for (int m = 0 ; m < 8 ; m++){	// Q0對角矩陣 對角值為forceratio開根號
							for (int n = 0 ; n < 8 ; n++){
								if(m == n){
									if (m < 4){	// L
										Q0[8*m+n] = sqrt(ratioL);} 
									else{	// R
										Q0[8*m+n] = sqrt(ratioR);}
								}
								else{
									Q0[8*m+n] = 0;
								}
							}
						}	

						InvSqMat(Q0,8);
						MatMulAB(A,3,8,Q0,8,8,A_w);

						MatScalarMul(ZMP,3,GAF[2],force_temp);

						MatMulABt(A_w,3,8,A_w,3,8,pinv_temp);
						InvSqMat(pinv_temp,3);
						MatMulAtB(A_w,3,8,pinv_temp,3,3,pinv_A_w);

						MatMulAB(Q0,8,8,pinv_A_w,8,3,temp);

						MatMulAB(temp,8,3,force_temp,3,1,F_dis);
					} 
					else if(j < ((i+1)*gStepSample+Nza*0.25))	// DSP 
					{				
						ratioL = 1/(2*forceratio[j-(i*gStepSample+Nza+Nab)]);	// L
						ratioR = 1/(2*(1-forceratio[j-(i*gStepSample+Nza+Nab)]));	// R

						for (int m = 0 ; m < 8 ; m++){	// Q0對角矩陣 對角值為forceratio開根號
							for (int n = 0 ; n < 8 ; n++){
								if(m == n){
									if (m < 4){	// L
										Q0[8*m+n] = sqrt(ratioL);} 
									else{	// R
										Q0[8*m+n] = sqrt(ratioR);}
								}
								else{
									Q0[8*m+n] = 0;
								}
							}
						}	

						InvSqMat(Q0,8);
						MatMulAB(A,3,8,Q0,8,8,A_w);

						MatScalarMul(ZMP,3,GAF[2],force_temp);

						MatMulABt(A_w,3,8,A_w,3,8,pinv_temp);
						InvSqMat(pinv_temp,3);
						MatMulAtB(A_w,3,8,pinv_temp,3,3,pinv_A_w);

						MatMulAB(Q0,8,8,pinv_A_w,8,3,temp);

						MatMulAB(temp,8,3,force_temp,3,1,F_dis);
					}
					else	// SSP
					{
						if (selSupport[i+1] == LeftSupport)	// 下一步是左腳
						{
							for (int k = 0 ; k < 4 ; k++)
							{	
								A_SSP[k] = A[k];
								A_SSP[4+k] = A[8+k];
								A_SSP[8+k] = A[16+k];
							}	
						} 
						else if (selSupport[i+1] == RightSupport)	// 下一步是右腳
						{
							for (int k = 0 ; k < 4 ; k++)
							{	
								A_SSP[k] = A[4+k];
								A_SSP[4+k] = A[12+k];
								A_SSP[8+k] = A[20+k];
							}	
						}

						MatScalarMul(ZMP,3,GAF[2],force_temp);

						MatMulABt(A_SSP,3,4,A_SSP,3,4,pinv_temp);
						InvSqMat(pinv_temp,3);
						MatMulAtB(A_SSP,3,4,pinv_temp,3,3,pinv_SSP);

						if (selSupport[i+1] == LeftSupport)	// 下一步是左腳SUP
						{
							MatMulAB(pinv_SSP,4,3,force_temp,3,1,F_dis);
							F_dis[4] = 0;	F_dis[5] = 0;	F_dis[6] = 0;	F_dis[7] = 0;
						} 
						else if (selSupport[i+1] == RightSupport)	// 下一步是右腳SUP
						{
							MatMulAB(pinv_SSP,4,3,force_temp,3,1,F_dis+4);
							F_dis[0] = 0;	F_dis[1] = 0;	F_dis[2] = 0;	F_dis[3] = 0;
						}
					}

					F_total[(FKCount+j)*2] = F_dis[0]+F_dis[1]+F_dis[2]+F_dis[3];
					F_total[(FKCount+j)*2+1] = F_dis[4]+F_dis[5]+F_dis[6]+F_dis[7];

					T_total[(FKCount+j)*4] = -1*( (F_dis[0]+F_dis[1])*CotPoint[0] + (F_dis[2]+F_dis[3])*CotPoint[3] );	// L Pitch
					T_total[(FKCount+j)*4+1] = (F_dis[0]+F_dis[3])*CotPoint[8] + (F_dis[1]+F_dis[2])*CotPoint[9];		// L Roll
					T_total[(FKCount+j)*4+2] =(F_dis[4]+F_dis[5])*CotPoint[5] + (F_dis[6]+F_dis[7])*CotPoint[6];		// R Pitch
					T_total[(FKCount+j)*4+3] = -1*( (F_dis[5]+F_dis[6])*CotPoint[13] + (F_dis[4]+F_dis[7])*CotPoint[12] );	// R Roll
			//// 開始解pseudo inverse
					if (Recordfile == true)
					{
						QQ.open("Testforce.txt",ios::app);
						for(int p = 0 ; p < 8 ; p++){
							QQ << F_dis[p]<< "\t";
						}		
						QQ<< endl;
						QQ.close();

						QQ.open("Testratio.txt",ios::app);
						QQ << ratioL<< "\t"<< ratioR<< "\t"<< endl;
						QQ.close();					
					}
				}
			}
		}

		for (int i = FKCount+(end_step+1)*gStepSample-Nab-Nzb ; i < FKCount+IKCount+HomeCount; i++)
		{
			F_total[i*2] = F_dis[0]+F_dis[1]+F_dis[2]+F_dis[3];
			F_total[i*2+1] = F_dis[4]+F_dis[5]+F_dis[6]+F_dis[7];

			T_total[i*4] = -1*( (F_dis[0]+F_dis[1])*CotPoint[0] + (F_dis[2]+F_dis[3])*CotPoint[3] );	// L Pitch
			T_total[i*4+1] = ( (F_dis[0]+F_dis[3])*CotPoint[8] + (F_dis[1]+F_dis[2])*CotPoint[9] );		// L Roll
			T_total[i*4+2] =( (F_dis[4]+F_dis[5])*CotPoint[5] + (F_dis[6]+F_dis[7])*CotPoint[6] );		// R Pitch
			T_total[i*4+3] = -1*( (F_dis[5]+F_dis[6])*CotPoint[13] + (F_dis[4]+F_dis[7])*CotPoint[12] );	// R Roll

			if (Recordfile == true)
			{
				QQ.open("Testforce.txt",ios::app);
				for(int p = 0 ; p < 8 ; p++){
					QQ << F_dis[p]<< "\t";
				}		
				QQ<< endl;
				QQ.close();
			}
		}
	}


		//fstream Fx;
		//Fx.open("TestZMPd.txt",ios::app);
		//for(int i = 0 ; i < 10000 ; i++){
		//Fx << ZMPxd[i]<< "\t"<< ZMPyd[i]<< endl;;
		//}			
		//Fx.close();

		//Fx.open("TestStep.txt",ios::app);
		//for(int i = 0 ; i < 10000 ; i++){
		//	Fx << StepX[i]<< "\t"<< StepY[i]<< "\t";
		//	Fx << endl;
		//}				
		//Fx.close();

		//Fx.open("Test.txt",ios::app);
		//for(int i = 0 ; i < 50 ; i++){
		//	Fx << DSPr[i]<< "\t"<< StepNum<< "\t"<<selSupport[i];
		//	Fx << endl;
		//}				
		//Fx.close();		

		//Fx.open("TestF_Dis.txt",ios::app);
		//for(int i = 0 ; i < 8 ; i++){
		//Fx << F_dis[i]<< "\t";
		//}			
		//Fx << endl;
		//Fx.close();

		//Fx.open("Testforcetemp.txt",ios::app);
		////for(int i = 0 ; i < 8 ; i++){
		//Fx << force_temp[0]<< "\t"<< force_temp[1]<< "\t"<< force_temp[2]<< "\t";
		////}			
		//Fx << endl;
		//Fx.close();

		//Fx.open("TestA.txt",ios::app);
		////for(int i = 0 ; i < 8 ; i++){
		//Fx << A[0]<< "\t"<< A[8]<< "\t"<< A[16]<< "\t";
		////}			
		//Fx << endl;
		//Fx.close();

		//Fx.open("TestZMP.txt",ios::app);
		////for(int i = 0 ; i < 8 ; i++){
		//Fx << ZMP[0]<< "\t"<< ZMP[1]<< "\t"<< ZMP[2]<< "\t";
		////}			
		//Fx << endl;
		//Fx.close();		

		//Fx.open("TestF_total.txt",ios::app);
		//for(int i = 0 ; i < FKCount+IKCount+HomeCount ; i++){
		//	Fx << F_total[i*2]<< "\t"<< F_total[i*2+1]<< endl;;
		//}			
		//Fx.close();

		//Fx.open("TestT_total.txt",ios::app);
		//for(int i = 0 ; i < FKCount+IKCount+HomeCount ; i++){
		//	Fx << T_total[i*4]<< "\t"<< T_total[i*4+1]<< "\t"<< T_total[i*4+2]<< "\t"<< T_total[i*4+3]<< endl;;
		//}				
		//Fx.close();
}
