/**************************************************************************************************
Copyright, 2010-2012, Robotics Lab., Dept. of M.E., National Taiwan University
File Name: Kine.h

Author: Jiu-Lou Yan
Version: 1.2
Date: 2010/07/20 => 2070/07/26 by Slongz & 泓逸

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

#include "FwdKine.h"
#include "GlobalConst.h"
#include "QFwdKine.h"
#include "QuaternionRotation.h"
#include <fstream>

class Kine
{
public:
	Kine(void); // 建構子
	~Kine(void); // 解構子

	void InitKineTrains(void); // 初始化DH parameters 
	void FindFK(void); // 計算所有forward kinematics
	void ComputeJacobians(void); // 計算整個機器人的Jacobian matirx
	// 計算swing trajectory
	void GenSwingTraj(double v0, double a0, double x1_percentage, double y1, double v1, double x2, double y2, double v2, double a2, int Np, int KeepPosCount, double* resultXY, double* resultZ);	
	// 計算swing trajectory	修改版 看起來更像人又不會損失很多穩定性
	void GenSwingTrajMod(double v0, double a0, double x1_percentage, double z1, double v1, double xy2, double z2, double v2, double a2, int Np, int KeepPosCount, double* resultXY, double* resultZ);
	// 利用高階多項式生成平滑的ZMP軌跡 jerk 不能指定
	void GenSmoothZMPShift(double y_start, double y_end, int Np, double* result);
	// 利用高階多項式生成平滑的ZMP軌跡 jerk = 0
	void GenSmoothZMPShift_ZeroJerk(double y_start, double y_end, int Np, double* result);
	// 利用高階多項式生成平滑的ZMP軌跡 所有參數可以自由指定
	void GenSmoothZMPShift_ZeroJerkshiftstair(double y_start, double y_end, int Np, double* result,int mode1);
	// 利用高階多項式生成平滑的ZMP軌跡 針對走樓梯shift 所有參數可以自由指定 哲軒20121127	
	void GenZMPFreeAssign(double x2, double y0, double v0, double a0, double j0, double y2, double v2, double a2, double j2, int Np, double* result);
	// 生出七次多項式	
	void Gen7DegPoly(double x1, double y1, double y2, int Np, double* result);
	// 生出九次多項式
	void Gen9DegPoly(double x1, double y1, double y2, int Np, double* result);// StairMode = 1為開啟樓梯銜接Z方向軌跡擷取
	// 生出上下樓梯Z方向的swing軌跡 連接兩個九次多項式
	void Gen9DegPolyStair(double x1, double y1, double y2, int Np, double* result);
	// 生出十次多項式
	void Gen10DegPoly(double x1, double y1,double x2, double y2,double x3, double y3, double yend, int Np, double* result);
	// 生成不是鐘形的多項式軌跡
	void Gen7DegPolyMod(double y0, double y1, int Np, double* result);
	// 將整個array 3個3個依次normalize成長度為一
	void NormVecArray(double* VecArray, double* Norm, int Len);
	// 計算COG Jacobian
	void GetCOGJacobian(void);
	// 計算COG 位置
	void FindCOG(void);
	// 解一格IK
	void IKSolve(double* tCOG, double* tSwing, double* tRSwing, double* tRFixed,double* tLArm, double* tRArm,double* tRLArm, double* tRRArm,int IKMode ,int* status);

	// WLS joint limit avoidance
	void InitWLN(void);
	void FindWLN(void);

	// Singularity Avoidance
	void InvWithSingTest(double* JJT, double* tempJJT_inv, int MRowJJT, long* _ipiv, double* _work, double MinDet, double alph);

	// basic vector operaion
	void Cross2Vd(double* v1, double* v2, double* v3); // cross
	void Cross2Vf(float* v1, float* v2, float* v3); // cross
	double NormXYZD(double* Vec); // normalize

	// Euler angle functions
	void ComputeEulerAng(double* RotM, double* EulerAng);
	void GetLegsCoords(void); // 算出現在雙腳跟身體在空間中的坐標系

	// For Inertia Matrix
	void InitInertia(void); // initialize

	void FindPseudoJ(void); // 計算pseudo inverse Jacobian matrix
	void SetIdentity(double* RotationMatrix); // 將矩陣設為identity matrix
	void CheckJointLimit(void); // 檢查機器人是否達到joint limit
	void GetRotPartRn(YMatLite* HomoMat, double* RotPart); // 取出homogeneous matrix 的 rotation part

	void UpdateDrawingBuffer(void); // 更新繪圖，只有在FK結束時執行

	void NumDiff(void);
	// 對腳的各個Joint角度做數值微分 得各軸角速度與角加速度
	
	//Dynamics
	void FindD(void);
	// 執行所有Dynamic含式 計算出Torque
	void FindDIni(void);
	// 初始化Dynamic參數
	void CalCoord(void);
	// 計算所有軸在世界中的位置與向量, 各桿件重心位置
	void FDOmegaL(void);
	// 計算Link在世界中的角速度
	void FDVelJ(void);
	// 計算Joint在世界中的速度
	void FDAlphaL(void);
	// 計算Link在世界中的角加速度
	void FDAccelJ(void);
	// 計算Joint在世界中的加速度
	void FDVelCOM(void);
	// 計算各桿件重心在世界中的速度
	void FDAccelCOM(void);
	// 計算各桿件重心在世界中的加速度
	void FDForceJ(void);
	// 計算各軸的力
	void FDTorqueJ(void);
	// 計算各軸的力矩
	void FindInertia2LocalCOM(void);
	// 以各桿件座標系描述該桿件的轉動慣量(定值)
	void Friction(void);
	// 加入摩擦力
	//Dynamics

	void ForceSensorData(int FlagSim, int FlagGo, int SensorCount, int LogCount, double *FS_DataL, double *FS_DataR);
	// 線上收力規值 濾波 或是離線收txt
	void FindEncCOG(double* Enc_theta, double* Enc_COG);
	// 利用Encoder Feedback透過FindFK FindCOG 算出COG
	void Distributor(double *ZMPxd, double *ZMPyd, double *StepX, double *StepY, int StepNum, int FKCount, int IKCount, int HomePos);
	// 分配desired force to preset contact point

		
	// infrared 相關參數  20130419  哲軒	
	//void  InfaredSensorData(double ADCL1, double ADCL2, double ADCL3 , double ADCL4, double ADCR1 , double ADCR2, double ADCR3  , double ADCR4  ) ;
	// 線上收紅外線值濾波 或是離線收txt  讀入各腳的ADC值將其轉換為距離  0 (LLEG)  1 (RLEG) 
	void  kalmanfilter(double  datanow, double  datafilterbefore ,  double *result ); 
	// datanow (當下data)     datafilterbefore(濾過的前一刻data)    *result (存入指定矩陣)	
	
	void smoothfilter(double  *data , int count );
	//將前n個值做平均   有濾波效果   n個sampling time delay  	


	//# define infraedbuffersize 250000 //紅外線所需buffer大小
	//double infrared[infraedbuffersize];  //存下所有infrared data
	//double infforce[infraedbuffersize];   
	//double deltaz[infraedbuffersize]; //存下deltaz 用於 landingtimecontrol 	
	//double deltazfilter[infraedbuffersize];
	
	//int infraredcount;      //infraredcount數       哲軒2013/4/3
	//
	//double InfraredLdata1[infraedbuffersize];
	//double InfraredLfilterdata1[infraedbuffersize];
	//double InfraredLkalmanfilterdata1[infraedbuffersize];
	//
	//double InfraredLdata2[infraedbuffersize];
	//double InfraredLfilterdata2[infraedbuffersize];
	//double InfraredLkalmanfilterdata2[infraedbuffersize];

	//double InfraredLdata3[infraedbuffersize];
	//double InfraredLfilterdata3[infraedbuffersize];
	//double InfraredLkalmanfilterdata3[infraedbuffersize];

	//double InfraredLdata4[infraedbuffersize];
	//double InfraredLfilterdata4[infraedbuffersize];
	//double InfraredLkalmanfilterdata4[infraedbuffersize];

	////轉成距離值
	//double InfraredLdisdata1[infraedbuffersize];
	//double InfraredLdisdata2[infraedbuffersize];
	//double InfraredLdisdata3[infraedbuffersize];
	//double InfraredLdisdata4[infraedbuffersize];
	//
	//double InfraredRdata1[infraedbuffersize];
	//double InfraredRfilterdata1[infraedbuffersize];
	//double InfraredRkalmanfilterdata1[infraedbuffersize];
	//
	//double InfraredRdata2[infraedbuffersize];
	//double InfraredRfilterdata2[infraedbuffersize];
	//double InfraredRkalmanfilterdata2[infraedbuffersize];

	//double InfraredRdata3[infraedbuffersize];
	//double InfraredRfilterdata3[infraedbuffersize];
	//double InfraredRkalmanfilterdata3[infraedbuffersize];

	//double InfraredRdata4[infraedbuffersize];
	//double InfraredRfilterdata4[infraedbuffersize];
	//double InfraredRkalmanfilterdata4[infraedbuffersize];

	////距離
	//double InfraredRdisdata1[infraedbuffersize];
	//double InfraredRdisdata2[infraedbuffersize];
	//double InfraredRdisdata3[infraedbuffersize];
	//double InfraredRdisdata4[infraedbuffersize];
	//
	//double infraredcaliL1  ;
	//double infraredcaliL2  ;
	//double infraredcaliL3  ;
	//double infraredcaliL4  ;
	//double infraredcaliR1  ;
	//double infraredcaliR2  ;
	//double infraredcaliR3  ;
	//double infraredcaliR4  ;

	//double infraredbiasL1  ;
	//double infraredbiasL2  ;
	//double infraredbiasL3  ;
	//double infraredbiasL4  ;
	//double infraredbiasR1  ;
	//double infraredbiasR2  ;
	//double infraredbiasR3  ;
	//double infraredbiasR4  ;
	// infrared 相關參數  20130419  哲軒


	int LegDHLen; // 腳的DH長度
	int ArmDHLen; // 手的DH長度
	FwdKine* FKLLeg; // 左腳Kinematics train
	FwdKine* FKRLeg; // 右腳Kinematics train
	FwdKine* FKLArm; // 左手Kinematics train
	FwdKine* FKRArm; // 右手Kinematics train

	YMatLite* CrdAll;	// 儲存所有軸的位置
	YMatLite* ZAxisAll; // 儲存所有z軸方向
	YMatLite* Ja; // Jacobian matrix
	int JaMRow; // Jacobian matrix 的row數
	int JaNCol; // Jacobian matrix 的column數
	
	// 計算Jacobian 用的 temp 暫存區
	double* tempJ;
	double* tempJT;
	double* tempJiWJT;
	double* tempInv;

	double* PseudoInv; // pseudo inverse matrix

	// 連續複製，讀取時使用之指標號碼
	int ind_x; 
	int ind_y;
	int ind_z;
	int ind_source;
	int ind_dest;

	double* r_com;		// 重心距離桿件距離
	double* mass_com;	// 桿件質量

	int selIK; // 左右腳支撐狀態

	double* pv_stack;	// 桿件重心於世界中之座標
	double* temp_Norm;	// 長度計算暫存空間

	double COG[3]; // COG 就存在這
	double BodyUpVec[3]; // 指向身體向上方向
	double BodyRightVec[3]; // 指向身體向右方向
	double BodyFaceVec[3]; // 指向身體面向方向
	double DHOrigin[3]; // DH之原點，會隨著機器人行走旋轉而變
	double UpBodyUpVec[3];

	double SumMass; // 機器人全重

	// COG jacobian 計算用暫存空間
	double temp_double_1;
	double temp_r_ef[3];
	double temp_cross[3];
	double temp_scale;

	//泓逸start111221
	double temp_scale_LA;
	double temp_scale_RA;
	double local_COG_LA[3];
	double local_COG_RA[3];
	double temp_mass_sum_LA;
	double temp_mass_sum_RA;
	double temp_mass_sum_next_LA;
	double temp_mass_sum_next_RA;
	//泓逸end111221

	double temp_vec1[3];
	double temp_vec2[3];
	double temp_vec3[3];
	double temp_vec4[3];
	double local_COG[3];
	double temp_mass_sum;
	double temp_mass_sum_next;

	// Jacobian計算用 end-effector 相關
	double* EndEffDiff;
	double EndEff[3];

	// IK 解出之 theta速度
	double* dth;
	double* dx;

	double RobotFixPoint[3];	// 機器人平移座標要到的地方(左腳或是右腳的end eff要到的地方)
	double RobotFixVector[3];	// 機器人平移向量

	double TempForCross[3];

	// swing腳 多項式專用參數
	double p1[5]; // 四次多項式係數
	double p2[6]; // 五次多項式
	double A1[4]; // 方陣，算p1用
	double A2[36]; // 方陣，算p2用
	double r1[2]; // boundary conditions
	double r2[6]; // boundary conditions
	double tempVal; // 計算暫存
	double SwingBufferx[10000]; // 存當下的swing trajectory 
	double SwingBuffery[10000]; // 存當下的swing trajectory  
	double SwingBufferz[10000]; // 存當下的swing trajectory  

	int Nab; // Nab = Na + Nb for SSP
	int Na; // SSP前段長度
	int Nb; // SSP後段長度
	int N_step; // 整步格數
	double TempXp[4]; // 計算暫存
	double x_p; // xy軸方向內插用的點

	// 時機控制參數，步行軌跡參數
	// DSP SSP代表行走中，ZMP切換的雙腳支撐時間百分比
	// DSP = 0.4; // double support phase 
	// SSP = 0.6 // single support phase
	// DSP + SSP = 1.0

	int Nza; // for DSP之補零，前段
	int Nzb; // for DSP之補零，後段

	// swing腳 多項式專用參數	

	//泓逸start111227
	double initRA[3];
	double initLA[3];
	double remLA[3];//左手第九軸
	double remRA[3];//右手第九軸
	double shiftLA[3];//左手第十軸
	double shiftRA[3];//右手第十軸
	
	//泓逸end111227

	double initCOG[3];	// 初始COG位置
	double initLL[3];	// 初始左腳位置
	double initRL[3];	// 初始右腳位置
	double remLL[3];	// 記住換角時左腳的位置
	double remRL[3];	// 記住換角時右腳的位置
	double shiftLL[3];	// 計算腳固定在空間中的向量
	double shiftRL[3];	// 計算腳固定在空間中的向量
	//double dx[12]; // the input to the IK solver

	double SwErrLim;	// swing 解IK 誤差值
	double COGErrLim;	// COG 解IK 誤差值
	double AngleErrLim;	// 可接受Angle error
	double SwingErr[3];	//  error of swing leg position in xyz 
	double COGErr[3];	// error of COG position in xyz
	//泓逸start111227
	double LArmErr[3];	// in xyz
	double RArmErr[3];	// in xyz
	double dx_temp[6]; //泓逸測試用dx

	double dthArmOffline[12];
	double dxArmOffline[12];
	double tempJArmOffline[12*12];
	//泓逸end111227


	double MaxSwingError; // maximum input position trajectory difference 防止機構衝太快
	double MaxCOGError; //maximum input position trajectory difference 防止機構衝太快
	double MaxRotationError; // maximum input angle trajectory difference 防止機構衝太快
	double MaxJointAngleChange; // 最大角度旋轉量

	double selSupport[4000]; // 設定雙腳的support phase
	double StepHeight[4000]; // 設定每步腳抬高量
	//Slongz 20130425
	double SupportPhs[20000]; // 設定雙腳的support phase	
	//Slongz
	int stepIndex; // 記住現在第幾步

	int cntIK; // 記住現在IK跑了幾圈

	bool FirstFKFound; // 第一次IK取完的旗標
	int FlagSumoMode; // 是否機器人要長時間單腳站立
	int FlagStayMode; // 機器人是否要進入stay mode
	bool FlagStaySquatMode;//機器人是否要進入stay squat mode
	bool FlagStayBreak;//機器人可經由鍵盤觸發結束stay mode
	

	// WLN parameters
	double MaxLL[6]; // 角度極限值
	double MinLL[6]; // 角度極限值
	double MaxRL[6]; // 角度極限值
	double MinRL[6]; // 角度極限值
	
	double MaxLLBound[6]; // 角度極限值
	double MinLLBound[6]; // 角度極限值
	double MaxRLBound[6]; // 角度極限值
	double MinRLBound[6]; // 角度極限值

	// 機構硬限制 上下限 
	double JointUpLimitLL[6]; // 左腳上限
	double JointLoLimitLL[6]; // 左腳下限
	double JointUpLimitRL[6]; // 右腳上限
	double JointLoLimitRL[6]; // 右腳下限

	double A_PC_LL[6]; // pre calculated para A for LL
	double A_PC_RL[6]; // pre calculated para A for RL
	double B_PC_LL[6]; // pre calculated para B for LL
	double B_PC_RL[6]; // pre calculated para B for RL

	double invWLNMat[12];
	// Note: dth = inv(W)*J'*inv(J*inv(W)*J')*dx
	// inv(W) = invWLNMat[12]

	double LastInvWLN_Mat[12]; // 記住inverse WLN 矩陣值
	double TempSquareVal; // 暫存記憶體
	double TempTh; // 暫存記憶體
	bool IfWLNEqualOne[12];  // 判斷WLN值是不是1
	// WLN

	// Variables for Singularity Avoidance 
	double* work_clapack;  // 給CLAPACK用的記憶體
	long* ipiv_clapack; // 給CLAPACK用的記憶體
	double SingularJudge; // 有沒有singular旗標
	double SingularAdd; // singular 要加上的數字

	double* tempJJT_for_inv; // 暫存記憶體 

	// Variables for Euler Angle
	double TarRotMSw[9]; // swing腳的目標旋轉矩陣
	double TarRotMFx[9]; // fixed腳的目標旋轉矩陣 
	double DiffRotMatSw[9]; // difference between two rotation matrices (注意差距要小 才能線性化)
	double DiffRotMatFx[9]; // difference between two rotation matrices (注意差距要小 才能線性化)
	double DiffRotMatBody[9]; // difference between two rotation matrices (注意差距要小 才能線性化)

	
	//20121214doratom//
	//pitch
	double TarRotMSwPitch[9]; // swing腳的目標旋轉矩陣
	double TarRotMFxPitch[9]; // fixed腳的目標旋轉矩陣 
	double DiffRotMatSwPitch[9]; // difference between two rotation matrices (注意差距要小 才能線性化)
	double DiffRotMatFxPitch[9]; // difference between two rotation matrices (注意差距要小 才能線性化)
	double DiffRotMatBodyPitch[9]; // difference between two rotation matrices (注意差距要小 才能線性化)
	//20121214doratom//

	//泓逸start120222
	double DiffRotMatLA[9]; // difference between two rotation matrices (注意差距要小 才能線性化)
	double DiffRotMatRA[9]; // difference between two rotation matrices (注意差距要小 才能線性化)
	//泓逸end120222

	double TarRotMBody[9]; // body的目標旋轉矩陣
	double LLegRotM[9]; // 左腳的旋轉矩陣
	double RLegRotM[9]; // 右腳的旋轉矩陣
	double BodyRotM[9]; // 身體的旋轉矩陣

	//泓逸start111227
	double TarRotMLA[9];
	double TarRotMRA[9];
	double LArmRotM[9];
	double RArmRotM[9];
	double EuAngLA[3];
	double EuAngRA[3];
	double EuAngTarLA[3];
	double EuAngTarRA[3];
	double AngErrLA[3];
	double AngErrRA[3];
	//泓逸end111227

	double LenEdgeXYZ[3]; // 這是腳底板的長度資料，在建構子裡面被設定

	double LSwitchRMot[16]; // 記錄換腳時腳底板旋轉矩陣 左腳
	double RSwitchRMot[16]; // 記錄換腳時腳底板旋轉矩陣 右腳
	double Enc_LSwitchRMot[16]; // 記錄Enc Feedback換腳時腳底板旋轉矩陣 左腳
	double Enc_RSwitchRMot[16]; // 記錄Enc Feedback換腳時腳底板旋轉矩陣 右腳
	double temp44MatMul[16]; // 4x4矩陣相乘暫存區

	double FixEndEffRMot[16]; // 剛算完FK時候的Fix Leg 坐標系

	float* ArmDrawingBuffer;
	float* LegDrawingBuffer;
	int IndexBuf;
	int	IndexBufRev;

	// variables for inertia matrix and computation

	// 單位 每個element乘上1000以後，單位是 (g x mm^2)
	//double IR_FPad[9];// = {6000, -550.1, 2000, -550.1, 9000, 98.94, 2000, 98.94, 7000}; 
	//IR_FPad =   [6000    -550.1  2000 ;
    //             -550.1  9000    98.94;
    //             2000    98.94   7000]; % 腳底板 A_RL0 - 6 
	// IR_FPad = 世界中的轉動慣量   IR_Foot = A_RL0(1:3,25:27)'*IR_FPad*A_RL0(1:3,25:27);
	double IR_Foot[9];

	//double IL_FPad[9]; 
	//IL_FPad =   [6000   546    2000 ;
    //             546    9000   -97.5;
    //             2000   -97.5  7000]; % 腳底板 A_LL0 - 6
	// IL_FPad = 世界中的轉動慣量 IL_Foot = A_LL0(1:3,25:27)'*IL_FPad*A_LL0(1:3,25:27);
	double IL_Foot[9];
	
	//double IR_KneeDown[9];
	//IR_KneeDown = [17000  106.1  -383 ;
	//				 106.1 16000   1000 ;
	//			    -383   1000   3000] ; % 腳踝連接膝蓋Yaw馬達的ㄇ字型 A_RL0 - 4
	// IR_Shank = A_RL0(1:3,17:19)'*IR_KneeDown*A_RL0(1:3,17:19);
	double IR_Shank[9];


	//double IL_KneeDown[9];
	//IL_KneeDown = [17000 -132.1   -450.3 ;
	//               -132.1 16000   -1000 ;
	//               -450.3  -1000    3000]; % 腳踝連接膝蓋Yaw馬達的ㄇ字型  A_LL0 - 4
	//IL_Shank = A_LL0(1:3,17:19)'*IL_KneeDown*A_LL0(1:3,17:19);
	double IL_Shank[9];

	//double IR_KneeUp[9];
	//IR_KneeUp   = [33000 -200.2  -191.1;
	//              -200.2 28000  -889.7;
	//              -191.1 -889.7  8000]; % 膝蓋以及連接髖關節的整塊  A_RL0 - 2
	//IR_Thigh = A_RL0(1:3,13:15)'*IR_KneeUp*A_RL0(1:3,13:15);
	double IR_Thigh[9];

	//double IL_KneeUp[9];
	//IL_KneeUp   = [33000  196.2  -201.5;
	//			  196.2  28000  877.5;
	//			  -201.5 877.5  8000]; % 膝蓋以及連接髖關節的整塊 A_LL0 - 2       
	//IL_Thigh =  A_LL0(1:3,13:15)'*IL_KneeUp*A_LL0(1:3,13:15); % 注意，A_LL0的第16 column是膝蓋的位置，此時這個座標系的原點是膝蓋，xyz軸是大腿方向
	double IL_Thigh[9];


	// 這個剛好就在世界下
	//IB1 = [76000   75    -10000;
	//	   75   55000    71;
	//	   -10000  71    57000];
	//% 整個下半身體  
	//% Block 腰下半部
	//% 由於先預設不轉，故直接就是世界值       
	double IB1[9]; 

	//% 身體有轉的地方   
	//double IBody[9];
	//IBody       = [71000 84   94;
	//               84   87000 3000;
	//               94    3000  85000]; % 軀幹 -- A_LA, A_RA -- R2 for roll,  R1是 pitch                          
    // IB2 = A_RA0(1:3,9:11)'*IBody*A_RA0(1:3,9:11);   % 腰的座標系應該取腰旋轉以後會受到影響的所以取9~11，注意，A_R0的第12 column是右手臂肩膀的位置
	double IB2[9];


	//% 整隻右手臂 
	//double IR_Arm[9];
	//IR_Arm      = [42000   -18.45   -134.8;
	//               -18.45   42000    54.74;
	//               -134.8   54.74   2000]; % 手臂 前後擺動方向 A_RA -- 4
	//           
	//IR_Arm = eye(3); % 故意給很小的    
	//IRA = A_RA0(1:3,13:15)'*IR_Arm*A_RA0(1:3,13:15);        
	double IRA[9];


	//% 整隻左手臂 
	//double IL_Arm[9];
	// IL_Arm      = [42000   -19.39   -136.3;
	//               -19.39    42000   -76.87;
	//               -136.3   -76.87   2000]; % 手臂 前後擺動方向 A_LA -- 4
	//IL_Arm = eye(3); % 故意給很小的   
	//ILA = A_LA0(1:3,13:15)'*IL_Arm*A_LA0(1:3,13:15);
	double ILA[9];
              

	////double IL_Foot_w[9]; // 世界中的，會即時更新
	////double IR_Foot_w[9]; // 世界中的，會即時更新
	////double IL_Shank_w[9]; // 世界中的，會即時更新
	////double IR_Shank_w[9]; // 世界中的，會即時更新
	////double IL_Thigh_w[9]; // 世界中的，會即時更新
	////double IR_Thigh_w[9]; // 世界中的，會即時更新
	////double IB1_w[9]; // 世界中的，會即時更新
	////double IB2_w[9]; // 世界中的，會即時更新
	////double ILA_w[9]; // 世界中的，會即時更新
	////double IRA_w[9]; // 世界中的，會即時更新


	double IR_Foot_sup_L[9]; // 依照support腳不一樣 inertia的原始點也不同
	double IL_Foot_sup_L[9];
	double IR_Shank_sup_L[9];
	double IL_Shank_sup_L[9];
	double IR_Thigh_sup_L[9];
	double IL_Thigh_sup_L[9];
	//double IB1_sup_L[9]; 
	//double IB2_sup_L[9];     
	//double IRA_sup_L[9];
	//double ILA_sup_L[9];

	double IR_Foot_sup_R[9]; // 依照support腳不一樣 inertia的原始點也不同
	double IL_Foot_sup_R[9];
	double IR_Shank_sup_R[9];
	double IL_Shank_sup_R[9];
	double IR_Thigh_sup_R[9];
	double IL_Thigh_sup_R[9];
	//double IB1_sup_R[9]; 
	//double IB2_sup_R[9];     
	//double IRA_sup_R[9];
	//double ILA_sup_R[9];

	//_______________________________________QCCD (Slongz)_______________________________________

	///CCD Par
	QFwdKine* QFKLLeg; //QFk Object: 左腳
	QFwdKine* QFKRLeg; //QFk Object: 右腳

	double CCDErr[3];  //CCD position rrror
	double CCDWei[6];  //CCD weighting facotors on 6-axis (leg)
	double Pid[3];  //CCD vector: from current axis to desired target
	double Pic[3];  //CCD vector: from current axis to current end-effector position
	double* K1; //CCD orientation indicators: k1~k3 ,corresponding pointers: K1~K3
	double* K2;
	double* K3;
	double k1;
	double k2;
	double k3;
	double QCCDdth[6]; //dth result from CCD


	int CCDcntIK; // 記住現在IK跑了幾圈
	void CCDInit(void); // Parameter Initialization for CCD
	void InitQKineTrains(void); // Initialization of kinematic chains using quaternion
	void FindSwingQFK(QFwdKine* QLeg, bool LLeg); // Forword kinematics with  quaternion (swing leg)
	void FindFixQFK(QFwdKine* QLeg, bool LLeg); // Forword kinematics with  homogeneous transkorm matrix (fixed leg)
	void CCDIKSolve(QFwdKine* QLeg,FwdKine* FLeg,double* tSwing, double* tRSwing,bool LLeg, int* status); //IK solver for swing leg with CCD method
	void CCDCOGSolve(QFwdKine* QLeg,FwdKine* FLeg,double* tCOG, double* tRFixed,bool LLeg, int* status); //IK solver for fixed leg with CCD method
	void GenCOGZTrajMod(double v0, double a0,double y0, double x1_percentage, double y1, double v1, double x2, double y2, double v2, double a2, int Np, int KeepPosCount, double* resultXY, double* resultZ);//Modified COGz pattern

	//.......................................TEST.......................................
	///Modified Ja
	YMatLite* FixJa;
	void ComputeFixJacobians(void); // 計算機器人的Jacobian matirx (fixed leg only)
	void GetCOGFixJacobian(void); // 計算機器人的COG Jacobian matirx (fixed leg only)
	void IKFixSolve(double* tCOG, double* tSwing, double* tRSwing, double* tRFixed, int* status);// IK solving preprocess for fixed leg(Jacobian) and swing leg(CCD) 
	void IKCCDSolve(double* tCOG, double* tSwing, double* tRSwing, double* tRFixed, int* status);;// IK solving preprocess for fixed leg(CCD) and swing leg(CCD) 
	///	
	/// Swing traj with flucutated footpad
	double AnklePitchRef[1200]; // Buffer for ankle pitch swing
	void GenSwingTrajMod2(double v0, double a0, double x1_percentage, double y1, double v1, double x2, double y2, double v2, double a2, int Np, int KeepPosCount, double* resultXY, double* resultZ);
	///
	//.......................................TEST.......................................
	double COGDev[20000];

	//_______________________________________QCCD (Slongz)_______________________________________
	//_______________________________________Check (Slongz)_______________________________________

	void PhsIndex(double StepNumb,double StepCount, double InitialCount, double EndCount, double Scale);
	bool PhsFlag;
	//_______________________________________Check (Slongz)_______________________________________


	//FreeMotion Slongz
	double SwingFreeCoef[1200];
	Quaternion* qTarSwing;
	Quaternion* qShankSwing;
	Quaternion* qChangedSwing;
	void GetShankCoords(void);
	double LShankRotM[9];
	double RShankRotM[9];
	int RotPhaseFlag;
	double ThetaInital;
	double ThetaTemp;
	bool RotFlagSeq[100];

	//NumDiff
	double ThetaLog[60];	// Numerical Buffer for Joint differential
	double ThetaDD[12];	// 腳12軸的角加速度
	double ThetaD[12];	// 腳12軸的角速度
	double TempTheta[12];
	double TotalCOMLog[15]; // Numerical Buffer for COG differential
	//double FootPosDDL[3];	// 腳底板的加速度(已不用) 因為我們腳步規劃的關係 在support時那隻腳一定加速度為0
	//double FootPosDDR[3];
	double TotalFootLogL[15];
	double TotalFootLogR[15];

	//Dynamics
	int AxisJump; // Jump因為support腳不同 計算換腳時Index需跳動 AxisJump = 18
	int DHJump;	 
	int StackJump;

	double Rstack[18];	//Linkage length of RCOM2Joint
	double VectorTemp1[36];
	double VectorTemp2[36];
	double rCOMccw[18];	// 逆時針方向各軸到桿件重心位置向量
	double rCOMcw[18];	// 順時針方向各軸到桿件重心位置向量
	double GravityZaxi[3];	// 重力在世界座標下的指向(0,0,-1)
	double BodyW;	// 身軀總重
	double LArmW;	// 左手臂總重
	double RArmW;	// 右手臂總重
	double LArmCOM[3];	// 左手COM位置
	double RArmCOM[3];	// 右手COM位置
	double AdamsFS[12];	// 存Adams模擬力規的值
	fstream KineFile;	// 讀力規值的txt
	//fstream AdamsMotionFile;	// AdamsFS量到的MotionT //20130115 直接輸入Adams量到的MotionTorque (失敗)
	double SensorOffset[3];	// 平移力規位置 原本力規r是在腳底板 往上移60
	//double AdamsMotionT[12];	// 將Adams量到的MotionT存下 //20130115 直接輸入Adams量到的MotionTorque (失敗)

	//Joint Kine
		double LocalOmegaJ[36]; //theta * 轉軸z
		double OmegaL[36];	//各桿件在世界中角速度
		double AlphaL[36];	//各桿件在世界中角加速度
		double LocalVelJ[36];	//各軸的LocalVel
		double VelJ[36];	// 各軸在世界中的速度
		double AccelJ[36];	// 各軸在世界中的加速度
	//Link COM Kine
		double BodyRCOM[3]; // 上半身COM位置
		double LocalVelCOM[36]; // 以各桿件座標系描述該桿件COM的速度
		double VelCOM[36];	// 各桿件在世界中的速度
		double AccelCOM[36];	// 各桿件在世界中的加速度
		
		/////以下Test用變數
		double VelTotalCOM[3];   //Vel   of COG
		double AccelTotalCOM[3]; //Accel of COG
		double CP[3]; 
		double ForceTotalCOM[3];
		/////以上Test用變數

	//Joint Dyna
		double FSensor_forcL[3] ;	// 左腳力規量測值(Force)
		double FSensor_forcR[3] ;	// 右腳力規量測值(Force)
		double FSensor_TorqL[3] ;	// 左腳力規量測值(Torque)
		double FSensor_TorqR[3] ;	// 右腳力規量測值(Torque)
		double ForceJ[36];	// 各軸在世界中的力
		double TorqueJ[36];	// 各軸在世界中的力矩
		double MotorTorq[12];	// 馬達出力 (理想)
		double RatedTorque[12];	// EPOS3 Torque Offset的單位
		double GearRatio[12];	// 各軸減速比 (皮帶輪+HD)
		double FrictionJ[24];	// 各軸摩擦力
		double MotorTorqF[12];	// 馬達出力 (加入摩擦力)
		double AdamsTorque[12];	
		double CoPL[SensorBufferSize];
		double CoPR[SensorBufferSize];
		double FS_ZMP[SensorBufferSize];
		double ForceDataKFL[SensorBufferSize*10];	// 20130405WZ
		double ForceDataKFR[SensorBufferSize*10];	// 20130405WZ
	// Infrared data
		double InfaredSensor_dataL[5]; //左腳紅外線量測值
		double InfaredSensor_dataR[4]; //右腳紅外線量測值  (哲軒20130329)



	//////// Inertia (Corrdinate about Local COM: Lc ) //////
	//  Swing Linkage
		double IcRFoot[9];	// 以右腳腳底板COM座標描述右腳腳底板轉動慣量
		double IcLFoot[9];	// 以左腳腳底板COM座標描述左腳腳底板轉動慣量
		double IcRShank[9];	// 以右腳小腿COM座標描述右腳小腿轉動慣量
		double IcLShank[9];	// 以左腳小腿COM座標描述左腳小腿轉動慣量
		double IcRThigh[9];	// 以右腳大腿COM座標描述右腳大腿轉動慣量
		double IcLThigh[9];	// 以左腳大腿COM座標描述左腳大腿轉動慣量
			
		double IcUpBody[9];	// 以胸部COM座標描述胸部轉動慣量
		double IcLowBody[9];	// 以腰部COM座標描述腰部轉動慣量
		double IcRArm[9];	// 以右手COM座標描述右手轉動慣量
		double IcLArm[9];	// 以左手COM座標描述左手轉動慣量
		double IcBody[9];	// 以上半身COM座標描述上半身轉動慣量

	//Others
		int GLindex;		 //Flag for Kine Checking (Initialing Point
		double RP[18];		 //Variable for Kine Checking
		double LP[18];		 //Variable for Kine Checking
		double VelJOld[36];  //Variable for 梯形積分(not used)
		bool DSPFlag;        //Flag of Double Support Phase
		int CountMotor;
	// Kalman Filter
		//initial values for the kalman filter	(ForceSensor)
		double x_est_last[12];
		double P_last[12];
		//the noise in the system
		double Q_KF[12];
		double R_KF[12];
		double K_KF[12];
		double P_KF[12];
		double P_temp[12];
		double x_temp_est[12];
		double x_est[12];
		double z_measured[12]; //the 'noisy' value we measured
		//double z_real[12]; //the ideal value we wish to measure    
		//initial values for the kalman filter	(Motor)
		double x_est_lastMotor[12];
		double P_lastMotor[12];
		//the noise in the system
		double QMotor[12];
		double RMotor[12];
		double KMotor[12];
		double PMotor[12];
		double P_tempMotor[12];
		double x_temp_estMotor[12];
		double x_estMotor[12];
		double z_measuredMotor[12]; //the 'noisy' value we measured
		//double z_realMotor[12];
	 // Kalman Filter For Infaredsensor  哲軒20130329
		//initial values for the kalman filter
		double x_est_last1[8];
		double P_last1[8];
		//the noise in the system
		double Q1[8];
		double R1[8];
		double Kinf[8];
		double P1[8];
		double P_temp1[8];
		double x_temp_est1[8];
		double x_est1[8];
		double z_measured1[8]; //the 'noisy' value we measured
		//double z_real1[8]; //the ideal value we wish to measure

	// 利用ENC FK FindCOG得到的COG位置 以下為使用變數
		long COG_ENCload[12*16632];	// 存下load進來的ENC
		int OfflineNum;	// 離線軌跡數目
		//double EncCOGDD[12];	// 腳12軸的角加速度
		//double EncCOGLog[15];
		//double RobotAllCOGDD[3];
		double Enc_shiftLL[3];	// 計算腳固定在空間中的向量
		double Enc_shiftRL[3];	// 計算腳固定在空間中的向量
		bool FirstEncCOG;
		int Enc_stepIndex;
		// Wei-Zh 130908
	// Ankel Strategy
		double F_total[2*20000];
		double T_total[4*20000];

};
