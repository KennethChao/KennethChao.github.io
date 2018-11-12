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
     ���{���D�n�Φb�p��Ҧ��B�ʾǥ\��A�]�tforward kinematics, inverse kinematics, 
	 Jacobian matrix, joint limit avoidance, singularity avoidance, dynamics ���D�n�\��
	 �U�禡�P�ܼƤ������иԨ��U��ŧi�P�w�q�B������

Note: None
***************************************************************************************************/


#include "stdafx.h"
#include <iostream>
#include <fstream>
#include "Kine.h"
#include <math.h>


using namespace std;

//#define PI 3.1415926  // �bGlobal Constant �]�w
#define OldRobot 0 //Kine.cpp r_com mass_com �Ѽƫ��w
	

extern unsigned int gIthIK; // �O��IK�Ѩ�ĴX��
extern int gStepSample; // �O�Ш���ĴX�B
extern int gInitCount; // �p���ĴX��
extern int check_slopeangle;
extern int gFlagSimulation;
extern bool gUpStair;

Kine::Kine(void) // not used
{ 
	/******************************************************************
	input: void
	output: void

	Note:
	// Class constructor  ��l�ƩҦ��ݭn�Ψ쪺�ܼƻP�x�}
	******************************************************************/
	LegDHLen = 13;
	//�l�hstart111213
	ArmDHLen = 10;
	//ArmDHLen = 4;
	//�l�hend111213

	FKLLeg = new FwdKine(13);
	FKRLeg = new FwdKine(13);
	//�l�hstart111213
//	FK_LArm = new FwdKine(4); // ����
//	FK_RArm = new FwdKine(4); // �k��
	FKLArm = new FwdKine(10); // ����
	FKRArm = new FwdKine(10); // �k��
	//�l�hend111213

	CrdAll = new YMatLite(LegDHLen*2+ArmDHLen*2,3);
	ZAxisAll= new YMatLite(LegDHLen*2+ArmDHLen*2,3);

	EndEffDiff = new double[36];

	dth = new double[24]; // solved angular vel in each IK loop
	dx = new double[24];

	selIK = 2; // double support phase
	// selIK = 0 : left foot is the supporter
    // selIK = 1 : right foot is the supporter

	//�l�hstart111220
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
	//�l�hstart111227

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

	//�l�hstart111214
	r_com = new double[16];
	mass_com = new double[16];
	pv_stack = new double[6*3+8*3+2*3];
	temp_Norm = new double[6*1+8+2];
	//�@���}�T�� �@����|�� �y���
	UpBodyUpVec[0] = 0;
	UpBodyUpVec[1] = 0;
	UpBodyUpVec[2] = 0;
	//�l�hend111214
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

//mass_com = [5200 2284 2743 ... %LLeg    Hip �v�r�������H�ΤW����SHD25�򰨹F�A�W�y�U�b���[�� 620(�v�r��) + 910 (SHD25) + 480 (150W motor)
//           5200 2284 2743 ... %RLeg
//           5  ...             %LArm
//           5  ...             %RArm
//           3678+(620+910+480)*2 6432];                  %Body�U�b�� = 7698 �W�b�� = body value 14000  , 6423�O�O�h���q�������q
//r_com = [109.88 134.74 56.95 ... %LLeg
//         109.88 134.74 56.95 ... %RLeg
//         206.16 ...                          %LArm
//         206.16 ...                          %RArm
//         45.6178 85];                    %Body ���[�q�����ɭԡA�̫�@�� 109.25

// from Matlab

	//�l�hstart111214
	r_com[0] = 109.88;  	r_com[1] = 134.74;  	r_com[2] = 56.95;   // LL
	r_com[3] = 109.88;  	r_com[4] = 134.74;  	r_com[5] = 56.95;   // RL
	r_com[6] = 136.41;  	r_com[7] = 130.98;		r_com[8] = 76.53;		r_com[9] = 5;// LArm
	r_com[10] = 136.41;     	r_com[11] = 130.98;		r_com[12] = 76.53;		r_com[13] = 5;// RArm
	r_com[14] = 45.6178;		r_com[15] = 109.25;								// Body ���[�q�����ɭԡA�̫�@�� 109.25 // �u���Ÿy���ɭ�85

	mass_com[0] = 5200;  	mass_com[1] = 2284;  	mass_com[2] = 2743;	// LL
	mass_com[3] = 5200;  	mass_com[4] = 2284;  	mass_com[5] = 2743;	// RL
	mass_com[6] = 3091;     mass_com[7] = 2751;  	mass_com[8] = 1508;		mass_com[9] = 5;	// LArm
	mass_com[10] = 3091;     mass_com[11] = 2751;  	mass_com[12] = 1508;		mass_com[13] = 5;// RArm
	mass_com[14] = 7698;		mass_com[15] = 14000;							
	// Body�U�b�� = 7698(���k��}0-1,1-2�b��LINK)  
	// �W�b�� = body value 14000(�q��+�T�x��)  , 6423�O�O�h���q�������q
	// ���k���`��14710
	// 2800 �O�t��
	// �`��56862

	//#if RunDynamics 130925 ���Xdefine
	// �H�s�q�쪺���q���D ����b�쥻���᭱ r_com�h���ΰ� �j�P�۵�(�b�����I) �[�q��
	mass_com[0] = 6300;  	mass_com[1] = 2800;  	mass_com[2] = 3800;	// LL
	mass_com[3] = 6300;  	mass_com[4] = 2800;  	mass_com[5] = 3800;	// RL ���k�}�`��25800
	mass_com[14] =10698;	mass_com[15] = 20000; 
	//#endif


	#if  OldRobot
	//r_com = [109.88 134.74 56.95 ... %LLeg
	//         109.88 134.74 56.95 ... %RLeg
	//         206.16 ...                          %LArm
	//         206.16 ...                          %RArm
	//         28.894 160.771];                       %Body (�q������I14.416�V�U) (93.352�q����U��pitch�b�V�W)

	r_com[8] = 28.894;		r_com[9] = 160.771;								// Body ���[�q�����ɭԡA�̫�@�� 109.25 // �u���Ÿy���ɭ�85

	//mass_com = [3117 1837 2730 ... %LLeg
	//            3117 1837 2730 ... %RLeg
	//            2434  ...                %LArm
	//            2434  ...                %RArm
	//            4367 22492];                  %Body�U�b�� = 493 �W�b�� = body value 770.37 
	mass_com[0] = 3117;  	mass_com[1] = 1837;  	mass_com[2] = 2730;	// LL
	mass_com[3] = 3117;  	mass_com[4] = 1837;  	mass_com[5] = 2730;	// RL
	mass_com[6] = 2434;  													// LArm
	mass_com[7] = 2434;  													// RArm
	mass_com[8] = 4367;		mass_com[9] = 22492;							// %Body�U�b�� = 7698 �W�b�� = body value 14000  , 6423�O�O�h���q�������q
	#endif

	SumMass = 0;
	for (int i = 0 ; i < 16 ; i++)
	{
		SumMass += mass_com[i];
	}
	//�l�hend111214

 // �h�����A�ΥH�p��swing�}�y��
 //A1 = [0       0       0      0      0      1; % x0 position eqn
 //      x1^5    x1^4    x1^3   x1^2   x1     1; % x1 position eqn
 //      0       0       0      0      1      0; % x0 velocity eqn
 //      5*x1^4  4*x1^3  3*x1^2 2*x1   1      0; % x1 velocity eqn
 //      0       0       0      2      0      0; % x0 acceleration eqn
 //      20*x2^3 12*x2^2 6*x2   2      0      0];% x1 acceleration eqn
 // ��l�ƪ��ɭԡA�⤣�|�ܪ����]�n�F
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
 // �h�����A�ΥH�p��swing�}�y��

	N_step = T_P/dt;
	Nab = N_step*SSP;

	Nza = (N_step-Nab)/2;
	Nzb = N_step-Nab-Nza;

    SwErrLim = 0.1; // swing ��IK �~�t�� mm 0.1
	COGErrLim = 0.01; // COG ��IK �~�t�� mm 0.01
	AngleErrLim = 0.001; // radian 0.001

    MaxSwingError = 1.5; // maximum input position trajectory difference ������c�Ĥӧ�
	// 1.5 * 200 = 300mm/s �̧֨C��30cm
	MaxCOGError = 1.5; //maximum input position trajectory difference ������c�Ĥӧ�
	// 1.5 * 200 = 300mm/s �̧֨C��30cm
	MaxRotationError = 0.00655; // maximum input angle trajectory difference ������c�Ĥӧ�
	// 0.00655 * 200 * 57 �C��̧�75��
	MaxJointAngleChange = 0.007;  // �C��̧�90��

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

	//// ���}�ɭԸ}���O����x�} ��l�Ʀ����x�}
	//LSwitchRMot[0] = 1;	LSwitchRMot[1] = 0;	LSwitchRMot[2] = 0;
	//LSwitchRMot[3] = 0;	LSwitchRMot[4] = 1;	LSwitchRMot[5] = 0;
	//LSwitchRMot[6] = 0;	LSwitchRMot[7] = 0;	LSwitchRMot[8] = 1;

	//// ���}�ɭԸ}���O����x�} ��l�Ʀ����x�}
	//RSwitchRMot[0] = 1;	RSwitchRMot[1] = 0;	RSwitchRMot[2] = 0;
	//RSwitchRMot[3] = 0;	RSwitchRMot[4] = 1;	RSwitchRMot[5] = 0;
	//RSwitchRMot[6] = 0;	RSwitchRMot[7] = 0;	RSwitchRMot[8] = 1;

	// ���}�ɭԸ}���O����x�} ��l�Ʀ����x�}
	LSwitchRMot[0] = 1;	    LSwitchRMot[1] = 0;  	LSwitchRMot[2] = 0;   LSwitchRMot[3] = 0;
	LSwitchRMot[4] = 0;	    LSwitchRMot[5] = 1;	    LSwitchRMot[6] = 0;   LSwitchRMot[7] = 0;
	LSwitchRMot[8] = 0;	    LSwitchRMot[9] = 0;	    LSwitchRMot[10]= 1;   LSwitchRMot[11]= 0;
	LSwitchRMot[12]= 0;	    LSwitchRMot[13]= 0; 	LSwitchRMot[14]= 0;   LSwitchRMot[15]= 1;

	// ���}�ɭԸ}���O����x�} ��l�Ʀ����x�}
	RSwitchRMot[0] = 1;	    RSwitchRMot[1] = 0;  	RSwitchRMot[2] = 0;   RSwitchRMot[3] = 0;
	RSwitchRMot[4] = 0;	    RSwitchRMot[5] = 1;	    RSwitchRMot[6] = 0;   RSwitchRMot[7] = 0;
	RSwitchRMot[8] = 0;	    RSwitchRMot[9] = 0;	    RSwitchRMot[10]= 1;   RSwitchRMot[11]= 0;
	RSwitchRMot[12]= 0;	    RSwitchRMot[13]= 0; 	RSwitchRMot[14]= 0;   RSwitchRMot[15]= 1;

	// ��⧹FK��Fix Leg �}���O���Шt ��l�Ʀ����x�}
	FixEndEffRMot[0] = 1;	    FixEndEffRMot[1] = 0;  	FixEndEffRMot[2] = 0;   FixEndEffRMot[3] = 0;
	FixEndEffRMot[4] = 0;	    FixEndEffRMot[5] = 1;	FixEndEffRMot[6] = 0;   FixEndEffRMot[7] = 0;
	FixEndEffRMot[8] = 0;	    FixEndEffRMot[9] = 0;	FixEndEffRMot[10]= 1;   FixEndEffRMot[11]= 0;
	FixEndEffRMot[12]= 0;	    FixEndEffRMot[13]= 0; 	FixEndEffRMot[14]= 0;   FixEndEffRMot[15]= 1;

	// clear and initialize �C�B�ﰪ
	for (int i = 0 ; i < 4000 ; i++)
		StepHeight[i] = 0.0;


	// �]�w���c�W�U������
	//      yaw                        roll                       pitch                      pitch                     pitch                     roll
	JointUpLimitLL[0] = 45.0;  JointUpLimitLL[1] = 38.0;   JointUpLimitLL[2] = 13.77;   JointUpLimitLL[3] = 95.0; JointUpLimitLL[4] = 40.545; JointUpLimitLL[5] = 38.0; // ���}�W��
	JointLoLimitLL[0] = -22.0;  JointLoLimitLL[1] = -16.45;   JointLoLimitLL[2] = -74.9;   JointLoLimitLL[3] = -10.0; JointLoLimitLL[4] = -63.4; JointLoLimitLL[5] = -29.0; // ���}�U��
	JointUpLimitRL[0] = 22.0;  JointUpLimitRL[1] = 16.45;   JointUpLimitRL[2] = 74.9;   JointUpLimitRL[3] = 10.0; JointUpLimitRL[4] = 63.4; JointUpLimitRL[5] = 29.0; // ���}�W��
	JointLoLimitRL[0] = -45.0;  JointLoLimitRL[1] = -38.0;   JointLoLimitRL[2] = -13.77;   JointLoLimitRL[3] = -95.0; JointLoLimitRL[4] = -40.545; JointLoLimitRL[5] = -38.0; // ���}�U��

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
 	//_________________________��ڼ����������H�Ufor loop �]���H�U�OŪ�w�s�U�O�Wtxt WZ______________________
		//KineFile.open("TestAdamsSimOriginalStairFilt.txt",ios::in);	// �W�ӱ�
		//KineFile.open("TestAdamsSimOriginalFilt.txt",ios::in);	// ����(9,200)
		//KineFile.open("FSMerge04271800.txt",ios::in);	// EXP����(9,200)
		//KineFile.open("FSMerge310.txt",ios::in); // ��ڶq�^���O�W��
		//AdamsMotionFile.open("TestAda\msMotionT.txt",ios::in);	//20130115 ������JAdams�q�쪺MotionTorque (����)
		//KineFile.open("EncLogData.txt",ios::in);	// Ū�^ENC�ܦ�COG 20130525 WZ
		//for (int i = 0 ; i < 12*8816+1 ; i++)
		//{
		//	KineFile >> COG_ENCload[i];	// �C���q�w�g�bKine���}����󤤦�12�����ŪENC
		//}
	//_________________________��ڼ����������H�Ufor loop �]���H�U�OŪ�H�s�U�O�Wtxt______________________
	
	// Rated Torque ��� = N*m*10^-6
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

	// RatedTorque���վ�+TCAT�������F��쬰�d�����@��Rated Torque
	for(int i = 0 ; i < 12 ; i++)
		RatedTorque[i] *=0.000001;

	// �C�Ӱ��F�ۤv����t��
	GearRatio[0] = 160*40/26 ; GearRatio[1] = 160*50/34 ; GearRatio[2] = 160*50/34 ;
	GearRatio[3] = 160*50/34 ; GearRatio[4] = 160*50/34 ; GearRatio[5] = 160*50/34 ;
	GearRatio[6] = 160*40/26 ; GearRatio[7] = 160*50/34 ; GearRatio[8] = 160*50/34 ;
	GearRatio[9] = 160*50/34 ; GearRatio[10] = 160*50/34 ; GearRatio[11] = 160*50/34 ;

	// �U�b�������O ��Viscous �A Coulomb Friction
	FrictionJ[0] = 65;	FrictionJ[1] = 85;	FrictionJ[2] = 50;	FrictionJ[3] = 85;
	FrictionJ[4] = 50;	FrictionJ[5] = 50;	FrictionJ[6] = 80;	FrictionJ[7] = 80;
	FrictionJ[8] = 40;	FrictionJ[9] = 85;	FrictionJ[10] = 50;	FrictionJ[11] = 40;	

	for(int i = 12 ; i < 24 ; i++)	// Coulomb Friction �ҥO��1
		FrictionJ[i] = 1;

	system("del Testerrorframe.txt");	// WZ131123
	system("del TestJoint.txt");	// WZ131123
	system("del TestdeltaKF.txt");	// WZ131123
	check_slopeangle = 1;	// �Q�αשY��IK�x�}
	#endif

	#if OfflineTraj
		OfflineNum = 12400;	// ��ʿ�J���u���y�񵧼� �����F��encoder�� �w���LPNjoint
		KineFile.open("test0001.txt",ios::in);	// Ū�^ENC�ܦ�COG 20130525 WZ
		for (int i = 0 ; i < 12*OfflineNum+1 ; i++){
			KineFile >> COG_ENCload[i];	// �C���q�w�g�bKine���}����󤤦�12�����ŪENC
		}
		KineFile.close();
	#endif

	#if cogestimate
		if (gFlagSimulation == CppSimu || gFlagSimulation == ADAMSSimu)
		{
			OfflineNum = 8816;	// ��ʿ�J���u���y�񵧼�
			KineFile.open("EncLogData.txt",ios::in);	// Ū�^ENC�ܦ�COG 20130525 WZ
			for (int i = 0 ; i < 12*OfflineNum+1 ; i++){
				KineFile >> COG_ENCload[i];	// �C���q�w�g�bKine���}����󤤦�12�����ŪENC
			}
			KineFile.close();
		}
	#endif

	FirstEncCOG = false;
	Enc_stepIndex = 0;
	
	// Enc Feedback���}�ɭԸ}���O����x�} ��l�Ʀ����x�}
	Enc_LSwitchRMot[0] = 1;	    Enc_LSwitchRMot[1] = 0;  	Enc_LSwitchRMot[2] = 0;   Enc_LSwitchRMot[3] = 0;
	Enc_LSwitchRMot[4] = 0;	    Enc_LSwitchRMot[5] = 1;	    Enc_LSwitchRMot[6] = 0;   Enc_LSwitchRMot[7] = 0;
	Enc_LSwitchRMot[8] = 0;	    Enc_LSwitchRMot[9] = 0;	    Enc_LSwitchRMot[10]= 1;   Enc_LSwitchRMot[11]= 0;
	Enc_LSwitchRMot[12]= 0;	    Enc_LSwitchRMot[13]= 0; 	Enc_LSwitchRMot[14]= 0;   Enc_LSwitchRMot[15]= 1;

	// Enc Feedback���}�ɭԸ}���O����x�} ��l�Ʀ����x�}
	Enc_RSwitchRMot[0] = 1;	    Enc_RSwitchRMot[1] = 0;  	Enc_RSwitchRMot[2] = 0;   Enc_RSwitchRMot[3] = 0;
	Enc_RSwitchRMot[4] = 0;	    Enc_RSwitchRMot[5] = 1;	    Enc_RSwitchRMot[6] = 0;   Enc_RSwitchRMot[7] = 0;
	Enc_RSwitchRMot[8] = 0;	    Enc_RSwitchRMot[9] = 0;	    Enc_RSwitchRMot[10]= 1;   Enc_RSwitchRMot[11]= 0;
	Enc_RSwitchRMot[12]= 0;	    Enc_RSwitchRMot[13]= 0; 	Enc_RSwitchRMot[14]= 0;   Enc_RSwitchRMot[15]= 1;
	// WZ 201309005


	// KF Initial
	for (int i = 0 ; i < 12 ; i ++){
		Q_KF[i] = 0.05;	// Q��model��coverence �V�p�V�۫H
		R_KF[i] = 3;	// R��measuremant��coverence �V�p�V�۫H	
		x_est_last[i] = 0;
		P_last[i] = 0;
		//x_est_lastMotor[i] = 0;
		//P_lastMotor[i] = 0;
		//QMotor[i] = 0.1;	// Q��model��coverence �V�p�V�۫H
		//RMotor[i] = 1;	// R��measuremant��coverence �V�p�V�۫H
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
	// Class destructor  ����Ҧ��ʺA�O����
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
	// Ū����r�ɮױo��Ҧ�kinematics train�� DH parameters
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
	// �D�X�Ҧ�kinematics train �� forward kinematics
	******************************************************************/

	// forward kinematics
	FKLLeg->DHConstruct();
	FKRLeg->DHConstruct();
	FKLArm->DHConstruct();
	FKRArm->DHConstruct();

	// ���X�}���O����x�} �åB�N�㰦�����H����ܾA���V
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

	// ���৹����X�Ҧ��b�I�y��
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

	// �����Ҧ������H�b����m��@�ɤ����`��m
	if (FirstFKFound == true) // �����l�Ʈɨ�����
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

			//�l�hstart1112
			remLA[0] = CrdAll->data[105];
			remLA[1] = CrdAll->data[106];
			remLA[2] = CrdAll->data[107];
			//����ĤQ�b 
			shiftLA[0] = CrdAll->data[105];
			shiftLA[1] = CrdAll->data[106];
			shiftLA[2] = CrdAll->data[107];
			//����ĤQ�b 

			remRA[0] = CrdAll->data[135];
			remRA[1] = CrdAll->data[136];
			remRA[2] = CrdAll->data[137];
			//�k��ĤE�b
			shiftRA[0] = CrdAll->data[135];
			shiftRA[1] = CrdAll->data[136];
			shiftRA[2] = CrdAll->data[137];
			//�k��ĤQ�b
			//�l�hend111227
		}
	}


	// ����������H���I��m
	DHOrigin[0] = RobotFixVector[0];
	DHOrigin[1] = RobotFixVector[1];
	DHOrigin[2] = RobotFixVector[2];

	// ���X�Ҧ�Z�b��V �H�Ѥ���Q��
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
	// �D�X�����H���骺Jacobian matrix 
	******************************************************************/
	// dth => [���}6�b ; �k�}6�b ; ����6�b ; �k��6�b]
	// selIK = 0 : left foot is the supporter
    // selIK = 1 : right foot is the supporter

	// when selIK = 0,   dx = [sw_x sw_y sw_z sw_tx sw_ty sw_tz fix_tx fix_ty fix_tz COGx COGy COGz ]
	// when selIK = 1,2, dx = [sw_x sw_y sw_z sw_tx sw_ty sw_tz fix_tx fix_ty fix_tz COGx COGy COGz ]
	// �Q�Ʀ��ۦP��!!
	// when selIK = 0,   dx = [sw_x sw_y sw_z sw_tx sw_ty sw_tz fix_tx fix_ty fix_tz LA RA COGx COGy COGz ]
	// when selIK = 1,2, dx = [sw_x sw_y sw_z sw_tx sw_ty sw_tz fix_tx fix_ty fix_tz LA RA COGx COGy COGz ]

	if (selIK == LeftSupport) // support leg = left
	{
		// �� 4 5 6 rol
		ind_x = JaNCol*3;
		ind_y = ind_x + JaNCol;
		ind_z = ind_y + JaNCol; 
		ind_source = 0;

		// swing theta x y z, 3 rows, 12 cols
		// ��g���t�� Jacobian  fixed leg to swing leg
		for (int i = 0 ; i < 6 ; i++) // ���]6�� 1~6 column  4~6 row(1)
		{
			Ja->data[ind_x] = -ZAxisAll->data[ind_source];
			Ja->data[ind_y] = -ZAxisAll->data[ind_source+1];
			Ja->data[ind_z] = -ZAxisAll->data[ind_source+2];
			ind_source += 3;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		// ��g���t�� Jacobian  swing leg to swing leg
		ind_source = 39;
		for (int i = 0 ; i < 6 ; i++) // �A�]6�� 7~12 column 4~6 row(2)
		{
			Ja->data[ind_x] = ZAxisAll->data[ind_source];
			Ja->data[ind_y] = ZAxisAll->data[ind_source+1];
			Ja->data[ind_z] = ZAxisAll->data[ind_source+2];
			ind_source += 3;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		// ��g���t�� Jacobian  left arm to swing leg,right arm to swing leg
		for(int i = 0; i<12 ; i++)// �]12�� 13~24 column 4~6 row(3)
		{
			Ja->data[ind_x] = 0;
			Ja->data[ind_y] = 0;
			Ja->data[ind_z] = 0;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		// �� 7 8 9 rol
		ind_x = JaNCol*6;
		ind_y = ind_x + JaNCol;
		ind_z = ind_y + JaNCol; 

		// fixed theta x y z, 3 rows, 12 cols
		// ��g���t�� Jacobian  fixed leg to fixed leg
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

		// ��g���t�� Jacobian  swing leg to fixed leg,left arm to fixed leg,right arm to fixed leg(5)
		for (int i = 0 ; i < 18 ; i++)
		{
			Ja->data[ind_x] = 0;
			Ja->data[ind_y] = 0;
			Ja->data[ind_z] = 0;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		// ��g��V�t�� Jacobian �Ҧ��b�A���swing�}��endeffector
		ind_source = 0;
		ind_dest = 0;
		EndEff[0] = CrdAll->data[51];
		EndEff[1] = CrdAll->data[52];
		EndEff[2] = CrdAll->data[53];

		for (int i = 0 ; i <6 ; i++)//fixed�}�C�@�b��swing�}endefector���Z��
		{
			EndEffDiff[ind_dest] =  CrdAll->data[ind_source] - EndEff[0];
			EndEffDiff[ind_dest+1] = CrdAll->data[ind_source+1] - EndEff[1];
			EndEffDiff[ind_dest+2] = CrdAll->data[ind_source+2] - EndEff[2];
			ind_source += 3;
			ind_dest += 3;
		}

		ind_source = 39;
		//int_dest �~��[��
		for (int i = 6 ; i <12 ; i++)//swing�}�C�@�b��swing�}endefector���Z��
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
		// ��g��V�t�� Jacobian fixed leg to swing leg
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
		ind_dest = 18; // �ɳo���ܼƨӥΡAind_dest�쥻���N��O �n�s�쪺�a�褧index
		// ��g��V�t�� Jacobian swing leg to swing leg
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

		// ��g��V�t�� Jacobian  left arm to swing leg,right arm to swing leg
		for (int i = 0 ; i < 12 ; i++)//13~24 column 1~3 row(8)
		{
			Ja->data[ind_x] = 0;
			Ja->data[ind_y] = 0;
			Ja->data[ind_z] = 0;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		// �� 13 14 15 rol
		ind_x = JaNCol*12;
		ind_y = ind_x + JaNCol;
		ind_z = ind_y + JaNCol; 
		ind_source = 0;

		// ��g���t�� Jacobian  fixed leg to left arm
		for (int i = 0 ; i < 6 ; i++) // ���]6�� 1~6 column  13~15 row(9)
		{
			Ja->data[ind_x] = -ZAxisAll->data[ind_source];
			Ja->data[ind_y] = -ZAxisAll->data[ind_source+1];
			Ja->data[ind_z] = -ZAxisAll->data[ind_source+2];
			ind_source += 3;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		// ��g���t�� Jacobian  swing leg to left arm
		for (int i = 0 ; i < 6 ; i++) // ���]6�� 7~12 column  13~15 row(10)
		{
			Ja->data[ind_x] = 0;
			Ja->data[ind_y] = 0;
			Ja->data[ind_z] = 0;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		ind_source = 87;
		// ��g���t�� Jacobian  left arm to left arm
		for (int i = 0 ; i < 6 ; i++) // ���]6�� 13~18 column  13~15 row(11)
		{
			Ja->data[ind_x] = ZAxisAll->data[ind_source];
			Ja->data[ind_y] = ZAxisAll->data[ind_source+1];
			Ja->data[ind_z] = ZAxisAll->data[ind_source+2];
			ind_source += 3;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		// ��g���t�� Jacobian  right arm to left arm
		for (int i = 0 ; i < 6 ; i++) // ���]6�� 19~24 column  13~15 row(12)
		{
			Ja->data[ind_x] = 0;
			Ja->data[ind_y] = 0;
			Ja->data[ind_z] = 0;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		// �� 19 20 21 rol
		ind_x = JaNCol*18;
		ind_y = ind_x + JaNCol;
		ind_z = ind_y + JaNCol; 
		ind_source = 0;

		// ��g���t�� Jacobian  fixed leg to right arm
		for (int i = 0 ; i < 6 ; i++) // ���]6�� 1~6 column  19~21 row(13)
		{
			Ja->data[ind_x] = -ZAxisAll->data[ind_source];
			Ja->data[ind_y] = -ZAxisAll->data[ind_source+1];
			Ja->data[ind_z] = -ZAxisAll->data[ind_source+2];
			ind_source += 3;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		// ��g���t�� Jacobian  swing leg to right arm,left arm to right arm
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
		// ��g���t�� Jacobian  right arm to right arm
		for (int i = 0 ; i < 6 ; i++) // ���]6�� 19~24 column  19~21 row(15)
		{
			Ja->data[ind_x] = ZAxisAll->data[ind_source];
			Ja->data[ind_y] = ZAxisAll->data[ind_source+1];
			Ja->data[ind_z] = ZAxisAll->data[ind_source+2];
			ind_source += 3;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		// ��g��V�t�� Jacobian �Ҧ��b�A���left arm��endeffector
		ind_source = 0;
		ind_dest = 0;
		EndEff[0] = CrdAll->data[102];
		EndEff[1] = CrdAll->data[103];
		EndEff[2] = CrdAll->data[104];

		for (int i = 0 ; i <6 ; i++)//fixed�}�C�@�b��left arm endeffector���Z��
		{
			EndEffDiff[ind_dest] =  CrdAll->data[ind_source] - EndEff[0];
			EndEffDiff[ind_dest+1] = CrdAll->data[ind_source+1] - EndEff[1];
			EndEffDiff[ind_dest+2] = CrdAll->data[ind_source+2] - EndEff[2];
			ind_source += 3;
			ind_dest += 3;
		}

		ind_source = 87;
		//int_dest �~��[��
		for (int i = 6 ; i <12 ; i++)//left arm�C�@�b��left arm endeffector���Z��
		{
			EndEffDiff[ind_dest] = EndEff[0] - CrdAll->data[ind_source];
			EndEffDiff[ind_dest+1] = EndEff[1] - CrdAll->data[ind_source+1];
			EndEffDiff[ind_dest+2] = EndEff[2] - CrdAll->data[ind_source+2];
			ind_source += 3;
			ind_dest += 3;
		}

		//��10 11 12 row
		ind_x = JaNCol*9;
		ind_y = ind_x + JaNCol;
		ind_z = ind_y + JaNCol; 
		ind_source = 0;
		// ��g��V�t�� Jacobian fixed leg to left arm
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

		// ��g��V�t�� Jacobian  swing leg to left arm
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
		ind_dest = 18; // �ɳo���ܼƨӥΡAind_dest�쥻���N��O �n�s�쪺�a�褧index
		// ��g��V�t�� Jacobian left arm to left arm
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

		// ��g��V�t�� Jacobian  right arm to left arm
		for (int i = 0 ; i < 6 ; i++)//19~24 column 10~12 row(19)
		{
			Ja->data[ind_x] = 0;
			Ja->data[ind_y] = 0;
			Ja->data[ind_z] = 0;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		// ��g��V�t�� Jacobian �Ҧ��b�A���right arm��endeffector
		ind_source = 0;
		ind_dest = 0;
		EndEff[0] = CrdAll->data[132];
		EndEff[1] = CrdAll->data[133];
		EndEff[2] = CrdAll->data[134];

		for (int i = 0 ; i <6 ; i++)//fixed�}�C�@�b��right arm endeffector���Z��
		{
			EndEffDiff[ind_dest] =  CrdAll->data[ind_source] - EndEff[0];
			EndEffDiff[ind_dest+1] = CrdAll->data[ind_source+1] - EndEff[1];
			EndEffDiff[ind_dest+2] = CrdAll->data[ind_source+2] - EndEff[2];
			ind_source += 3;
			ind_dest += 3;
		}

		ind_source = 117;
		//int_dest �~��[��
		for (int i = 6 ; i <12 ; i++)//right arm�C�@�b��right arm endeffector���Z��
		{
			EndEffDiff[ind_dest] = EndEff[0] - CrdAll->data[ind_source];
			EndEffDiff[ind_dest+1] = EndEff[1] - CrdAll->data[ind_source+1];
			EndEffDiff[ind_dest+2] = EndEff[2] - CrdAll->data[ind_source+2];
			ind_source += 3;
			ind_dest += 3;
		}

		//��16 17 18 row
		ind_x = JaNCol*15;
		ind_y = ind_x + JaNCol;
		ind_z = ind_y + JaNCol; 
		ind_source = 0;
		// ��g��V�t�� Jacobian fixed leg to right arm
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
		// ��g��V�t�� Jacobian  swing leg to right arm,left arm to right arm
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
		ind_dest = 18; // �ɳo���ܼƨӥΡAind_dest�쥻���N��O �n�s�쪺�a�褧index
		// ��g��V�t�� Jacobian right arm to right arm
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
		// ��g���t�� Jacobian  swing leg to swing leg     swing = left
		for (int i = 0 ; i < 6 ; i++) // ���]6��  column 1~6 row 4~6(1)
		{
			Ja->data[ind_x] = ZAxisAll->data[ind_source];
			Ja->data[ind_y] = ZAxisAll->data[ind_source+1];
			Ja->data[ind_z] = ZAxisAll->data[ind_source+2];
			ind_source += 3;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		// ��g���t�� Jacobian  fixed leg to swing leg     swing = left
		ind_source = 39;
		for (int i = 0 ; i < 6 ; i++) // �A�]6�� column 7~12 row 4~6(2)
		{
			Ja->data[ind_x] = -ZAxisAll->data[ind_source];
			Ja->data[ind_y] = -ZAxisAll->data[ind_source+1];
			Ja->data[ind_z] = -ZAxisAll->data[ind_source+2];
			ind_source += 3;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		// ��g���t�� Jacobian  left arm to swing leg, right arm to swing leg
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
		// ��g���t�� Jacobian  swing leg to fixed leg (all zeros)
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

		// ��g���t�� Jacobian  fix leg to fixed leg
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
		// ��g���t�� Jacobian  left arm to fixed leg ,right arm to fixed leg(all zeros)
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


		// ��g��V�t�� Jacobian �Ҧ��b�A���endeffector

		EndEff[0] = CrdAll->data[12];
		EndEff[1] = CrdAll->data[13];
		EndEff[2] = CrdAll->data[14];

		ind_source = 0;
		ind_dest = 0;
		for (int i = 0 ; i <6 ; i++)//swing�}�C�@�b��swing�}endeffector���Z��
		{
			EndEffDiff[ind_dest] =  EndEff[0] - CrdAll->data[ind_source];
			EndEffDiff[ind_dest+1] = EndEff[1] - CrdAll->data[ind_source+1];
			EndEffDiff[ind_dest+2] = EndEff[2] - CrdAll->data[ind_source+2];
			ind_source += 3;
			ind_dest += 3;
		}

		ind_source = 39;
		//ind_dest �~��[��
		for (int i = 6 ; i <12 ; i++)//fixed�}�C�@�b��swing�}endeffector���Z��
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
		// ��g��V�t�� Jacobian swing leg to swing leg
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
		ind_dest = 18; // �ɳo���ܼƨӥΡAind_dest�쥻���N��O �n�s�쪺�a�褧index
		// ��g��V�t�� Jacobian fixed leg to swing leg
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
		// ��g��V�t�� Jacobian  left arm to swing leg ,right arm to swing leg(all zeros)
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

		// �� 13 14 15 rol
		ind_x = JaNCol*12;
		ind_y = ind_x + JaNCol;
		ind_z = ind_y + JaNCol;

		// ��g���t�� Jacobian  swing leg to left arm
		for (int i = 0 ; i < 6 ; i++) // ���]6�� 1~6 column  13~15 row(10)
		{
			Ja->data[ind_x] = 0;
			Ja->data[ind_y] = 0;
			Ja->data[ind_z] = 0;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}
		ind_source = 39;
		// ��g���t�� Jacobian  fixed leg to left arm
		for (int i = 0 ; i < 6 ; i++) // ���]6�� 7~12 column  13~15 row(11)
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
		// ��g���t�� Jacobian  left arm to left arm
		for (int i = 0 ; i < 6 ; i++) // ���]6�� 13~18 column  13~15 row(12)
		{
			Ja->data[ind_x] = ZAxisAll->data[ind_source];
			Ja->data[ind_y] = ZAxisAll->data[ind_source+1];
			Ja->data[ind_z] = ZAxisAll->data[ind_source+2];
			ind_source += 3;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}
		// ��g���t�� Jacobian  right arm to left arm
		for (int i = 0 ; i < 6 ; i++) // ���]6�� 19~24 column  13~15 row(13)
		{
			Ja->data[ind_x] = 0;
			Ja->data[ind_y] = 0;
			Ja->data[ind_z] = 0;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		// �� 19 20 21 rol
		ind_x = JaNCol*18;
		ind_y = ind_x + JaNCol;
		ind_z = ind_y + JaNCol; 


		// ��g���t�� Jacobian  swing leg to right arm
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
		// ��g���t�� Jacobian  fixed leg to right arm
		for (int i = 0 ; i < 6 ; i++) // ���]6�� 7~12 column  19~21 row(15)
		{
			Ja->data[ind_x] = -ZAxisAll->data[ind_source];
			Ja->data[ind_y] = -ZAxisAll->data[ind_source+1];
			Ja->data[ind_z] = -ZAxisAll->data[ind_source+2];
			ind_source += 3;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		// ��g���t�� Jacobian  left arm to right arm
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
		// ��g���t�� Jacobian  right arm to right arm
		for (int i = 0 ; i < 6 ; i++) // ���]6�� 19~24 column  19~21 row(17)
		{
			Ja->data[ind_x] = ZAxisAll->data[ind_source];
			Ja->data[ind_y] = ZAxisAll->data[ind_source+1];
			Ja->data[ind_z] = ZAxisAll->data[ind_source+2];
			ind_source += 3;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}

		// ��g��V�t�� Jacobian �Ҧ��b�A���left arm��endeffector
		ind_source = 39;
		ind_dest = 0;
		EndEff[0] = CrdAll->data[102];
		EndEff[1] = CrdAll->data[103];
		EndEff[2] = CrdAll->data[104] ;

		for (int i = 0 ; i <6 ; i++)//fixed�}�C�@�b��left arm endeffector���Z��
		{
			EndEffDiff[ind_dest] =  CrdAll->data[ind_source] - EndEff[0];
			EndEffDiff[ind_dest+1] = CrdAll->data[ind_source+1] - EndEff[1];
			EndEffDiff[ind_dest+2] = CrdAll->data[ind_source+2] - EndEff[2];
			ind_source += 3;
			ind_dest += 3;
		}

		ind_source = 87;
		//int_dest �~��[��
		for (int i = 6 ; i <12 ; i++)//left arm�C�@�b��left arm endeffector���Z��
		{
			EndEffDiff[ind_dest] = EndEff[0] - CrdAll->data[ind_source];
			EndEffDiff[ind_dest+1] = EndEff[1] - CrdAll->data[ind_source+1];
			EndEffDiff[ind_dest+2] = EndEff[2] - CrdAll->data[ind_source+2];
			ind_source += 3;
			ind_dest += 3;
		}

		//��10 11 12 row
		ind_x = JaNCol*9;
		ind_y = ind_x + JaNCol;
		ind_z = ind_y + JaNCol; 
		

		// ��g��V�t�� Jacobian  swing leg to left arm
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
		ind_dest = 0; // �ɳo���ܼƨӥΡAind_dest�쥻���N��O �n�s�쪺�a�褧index
		// ��g��V�t�� Jacobian fixed leg to left arm
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
		ind_dest = 18; // �ɳo���ܼƨӥΡAind_dest�쥻���N��O �n�s�쪺�a�褧index
		// ��g��V�t�� Jacobian left arm to left arm
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

		// ��g��V�t�� Jacobian  right arm to left arm
		for (int i = 0 ; i < 6 ; i++)//19~24 column 10~12 row(21)
		{
			Ja->data[ind_x] = 0;
			Ja->data[ind_y] = 0;
			Ja->data[ind_z] = 0;
			ind_x += 1;
			ind_y += 1;
			ind_z += 1;
		}



		// ��g��V�t�� Jacobian �Ҧ��b�A���right arm��endeffector
		ind_source = 39;
		ind_dest = 0;
		EndEff[0] = CrdAll->data[132];
		EndEff[1] = CrdAll->data[133];
		EndEff[2] = CrdAll->data[134];


		for (int i = 0 ; i <6 ; i++)//fixed�}�C�@�b��right arm endeffector���Z��
		{
			EndEffDiff[ind_dest] =  CrdAll->data[ind_source] - EndEff[0];
			EndEffDiff[ind_dest+1] = CrdAll->data[ind_source+1] - EndEff[1];
			EndEffDiff[ind_dest+2] = CrdAll->data[ind_source+2] - EndEff[2];
			ind_source += 3;
			ind_dest += 3;
		}

		ind_source = 117;
		//int_dest �~��[��
		for (int i = 6 ; i <12 ; i++)//right arm�C�@�b��right arm endeffector���Z��
		{
			EndEffDiff[ind_dest] = EndEff[0] - CrdAll->data[ind_source];
			EndEffDiff[ind_dest+1] = EndEff[1] - CrdAll->data[ind_source+1];
			EndEffDiff[ind_dest+2] = EndEff[2] - CrdAll->data[ind_source+2];
			ind_source += 3;
			ind_dest += 3;
		}

		//��16 17 18 row
		ind_x = JaNCol*15;
		ind_y = ind_x + JaNCol;
		ind_z = ind_y + JaNCol; 

		// ��g��V�t�� Jacobian  swing leg to right arm
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
		ind_dest = 0; // �ɳo���ܼƨӥΡAind_dest�쥻���N��O �n�s�쪺�a�褧index
		// ��g��V�t�� Jacobian fixed leg to right arm
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

		// ��g��V�t�� Jacobian  left arm to right arm
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
		ind_dest = 18; // �ɳo���ܼƨӥΡAind_dest�쥻���N��O �n�s�쪺�a�褧index
		// ��g��V�t�� Jacobian right arm to right arm
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

	// �p������HCOG Jacobian
	GetCOGJacobian();

}

void Kine::ComputeFixJacobians(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// �D�X�����H���骺Jacobian matrix 
	******************************************************************/
	// dth => [���}6�b ; �k�}6�b]
	// selIK = 0 : left foot is the supporter
    // selIK = 1 : right foot is the supporter

	// when selIK = 0,   dx = [sw_x sw_y sw_z sw_tx sw_ty sw_tz fix_tx fix_ty fix_tz COGx COGy COGz ]
	// when selIK = 1,2, dx = [sw_x sw_y sw_z sw_tx sw_ty sw_tz fix_tx fix_ty fix_tz COGx COGy COGz ]
	// �Q�Ʀ��ۦP��!!

	if (selIK == 0) // support leg = left  // �٨S��
	{
		//// �� 4 5 6 rol
		//ind_x = JaNCol*3;
		//ind_y = ind_x + JaNCol;
		//ind_z = ind_y + JaNCol; 
		//ind_source = 0;

		//// swing theta x y z, 3 rows, 12 cols
		//// ��g���t�� Jacobian  fixed leg to swing leg
		//for (int i = 0 ; i < 6 ; i++) // ���]6�� 1~6 column
		//{
		//	Ja->data[ind_x] = -ZAxisAll->data[ind_source];
		//	Ja->data[ind_y] = -ZAxisAll->data[ind_source+1];
		//	Ja->data[ind_z] = -ZAxisAll->data[ind_source+2];
		//	ind_source += 3;
		//	ind_x += 1;
		//	ind_y += 1;
		//	ind_z += 1;
		//}

		//// ��g���t�� Jacobian  swing leg to swing leg
		//ind_source = 39;
		//for (int i = 0 ; i < 6 ; i++) // �A�]6�� 7~12 column
		//{
		//	Ja->data[ind_x] = ZAxisAll->data[ind_source];
		//	Ja->data[ind_y] = ZAxisAll->data[ind_source+1];
		//	Ja->data[ind_z] = ZAxisAll->data[ind_source+2];
		//	ind_source += 3;
		//	ind_x += 1;
		//	ind_y += 1;
		//	ind_z += 1;
		//}

		//// �� 7 8 9 rol
		//ind_x = JaNCol*6;
		//ind_y = ind_x + JaNCol;
		//ind_z = ind_y + JaNCol; 
		int JaNColF=6;

		ind_x = 0;
		ind_y = ind_x + JaNColF;
		ind_z = ind_y + JaNColF; 

		// fixed theta x y z, 3 rows, 12 cols
		// ��g���t�� Jacobian  fixed leg to fixed leg
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

		//// ��g���t�� Jacobian  swing leg to fixed leg
		//for (int i = 0 ; i < 6 ; i++)
		//{
		//	Ja->data[ind_x] = 0;
		//	Ja->data[ind_y] = 0;
		//	Ja->data[ind_z] = 0;
		//	ind_x += 1;
		//	ind_y += 1;
		//	ind_z += 1;
		//}

		//// ��g��V�t�� Jacobian �Ҧ��b�A���swing�}��endeffector
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
		////int_dest �~��[��
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
		//ind_dest = 18; // �ɳo���ܼƨӥΡAind_dest�쥻���N��O �n�s�쪺�a�褧index
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
		//// ��g���t�� Jacobian  swing leg to swing leg     swing = left
		//for (int i = 0 ; i < 6 ; i++) // ���]6��
		//{
		//	Ja->data[ind_x] = ZAxisAll->data[ind_source];
		//	Ja->data[ind_y] = ZAxisAll->data[ind_source+1];
		//	Ja->data[ind_z] = ZAxisAll->data[ind_source+2];
		//	ind_source += 3;
		//	ind_x += 1;
		//	ind_y += 1;
		//	ind_z += 1;
		//}

		//// ��g���t�� Jacobian  fixed leg to swing leg     swing = left
		//ind_source = 39;
		//for (int i = 0 ; i < 6 ; i++) // �A�]6��
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

		// ��g���t�� Jacobian  fix leg to fixed leg
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

		//// ��g��V�t�� Jacobian �Ҧ��b�A���endeffector

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
		////ind_dest �~��[��
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
		//ind_dest = 18; // �ɳo���ܼƨӥΡAind_dest�쥻���N��O �n�s�쪺�a�褧index
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

	// �p������HCOG Jacobian
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
	input: VecArray ��J�� vector array �T�ӤT�Ӥ@�� �Ʀ����array, Len �N���J�x�}������
	output: Norm �⧹���Ҧ�norm�Ȥ���X

	Note:
	// �N��J�x�}���� �T�ӤT�ӨD�onorm�����X
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
	// �p��COG jacobian
	******************************************************************/

	// dth => [���}6�b ; �k�}6�b]
	// selIK = 0 : left foot is the supporter
    // selIK = 1 : right foot is the supporter

	// when selIK = 0, dx = [sw_x sw_y sw_z sw_tx sw_ty sw_tz fix_tx fix_ty fix_tz COGx COGy COGz]'
	// when selIK = 0, dx = [sw_x sw_y sw_z sw_tx sw_ty sw_tz fix_tx fix_ty fix_tz LA_x LA_y LA_z LA_tx LA_ty LA_tz RA_x RA_y RA_z RA_tx RA_ty RA_tz COGx COGy COGz]'

	// ��X�U��linkage�����q���W��b�@�ɤ�����m
	// �D�X�� �����Q�η|�v�T��e�ֿn��q���W��m���� �p��COG Jacobian
	// �ԲӤ����P���ɽаѨ��פ�
	//�l�hstart111223
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

		////�l�hstart111228
		////����ʤ��v�T����
		//for(int i = 12;i<24;i++)
		//{
		//	Ja->ValSet(22,i,0);
		//	Ja->ValSet(23,i,0);
		//	Ja->ValSet(24,i,0);
		//}
		////�l�hend111228



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


		//�l�hstart111228
		//����ʤ��v�T����
		/*for(int i = 13;i<25;i++)
		{
			Ja->ValSet(22,i,0);
			Ja->ValSet(23,i,0);
			Ja->ValSet(24,i,0);
		}*/
		//�l�hend111228


	}
		//�l�hend11122

}

void Kine::GetCOGFixJacobian(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// �p��COG jacobian
	******************************************************************/

	// dth => [���}6�b ; �k�}6�b]
	// selIK = 0 : left foot is the supporter
    // selIK = 1 : right foot is the supporter

	// when selIK = 0, dx = [sw_x sw_y sw_z sw_tx sw_ty sw_tz fix_tx fix_ty fix_tz COGx COGy COGz]'

	// ��X�U��linkage�����q���W��b�@�ɤ�����m
	// �D�X�� �����Q�η|�v�T��e�ֿn��q���W��m���� �p��COG Jacobian
	// �ԲӤ����P���ɽаѨ��פ�
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
	// �Q�α�󭫶q���W��b�Ŷ�������m �����[�_�� �[�v���� �i�o��COG ��m
	******************************************************************/

	//  left leg ��q���W��m �e�B�z ��m����
	pv_stack[0] = CrdAll->data[9]-CrdAll->data[6];
	pv_stack[1] = CrdAll->data[10]-CrdAll->data[7];
	pv_stack[2] = CrdAll->data[11]-CrdAll->data[8]; // �b3 knee �� �b2 hip

	pv_stack[3] = CrdAll->data[12]-CrdAll->data[9];
	pv_stack[4] = CrdAll->data[13]-CrdAll->data[10];
	pv_stack[5] = CrdAll->data[14]-CrdAll->data[11]; // �b4 ankle �� �b3 knee

	pv_stack[6] = CrdAll->data[21]-CrdAll->data[12];
	pv_stack[7] = CrdAll->data[22]-CrdAll->data[13];
	pv_stack[8] = CrdAll->data[23]-CrdAll->data[14]; // �b7 foot �� �b4 ankle


	//  right leg ��q���W��m �e�B�z ��m����
	pv_stack[9] = CrdAll->data[48]-CrdAll->data[45];
	pv_stack[10] = CrdAll->data[49]-CrdAll->data[46];
	pv_stack[11] = CrdAll->data[50]-CrdAll->data[47]; // �b16 knee �� �b15 hip

	pv_stack[12] = CrdAll->data[51]-CrdAll->data[48];
	pv_stack[13] = CrdAll->data[52]-CrdAll->data[49];
	pv_stack[14] = CrdAll->data[53]-CrdAll->data[50]; // �b17 ankle �� �b16 knee

	pv_stack[15] = CrdAll->data[60]-CrdAll->data[51];
	pv_stack[16] = CrdAll->data[61]-CrdAll->data[52];
	pv_stack[17] = CrdAll->data[62]-CrdAll->data[53]; // �b20 foot �� �b17 ankle

	//�l�hstart111214


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
	


	//�l�hend111214  

	// body   up = �V�W���V�q Right = �V�k  Face = ���V  ��q���W��m �e�B�z ��m����
	BodyUpVec[0] = DHOrigin[0]-(CrdAll->data[0]+CrdAll->data[39])/2.0;
	BodyUpVec[1] = DHOrigin[1]-(CrdAll->data[1]+CrdAll->data[40])/2.0;
	BodyUpVec[2] = DHOrigin[2]-(CrdAll->data[2]+CrdAll->data[41])/2.0;

	temp_double_1 = sqrt(BodyUpVec[0]*BodyUpVec[0]+BodyUpVec[1]*BodyUpVec[1]+BodyUpVec[2]*BodyUpVec[2]);
	BodyUpVec[0] = BodyUpVec[0]/temp_double_1;
	BodyUpVec[1] = BodyUpVec[1]/temp_double_1;
	BodyUpVec[2] = BodyUpVec[2]/temp_double_1;

	BodyRightVec[0] = CrdAll->data[39]-CrdAll->data[0]; // �k�
	BodyRightVec[1] = CrdAll->data[40]-CrdAll->data[1];
	BodyRightVec[2] = CrdAll->data[41]-CrdAll->data[2];

	temp_double_1 = sqrt(BodyRightVec[0]*BodyRightVec[0]+BodyRightVec[1]*BodyRightVec[1]+BodyRightVec[2]*BodyRightVec[2]);
	BodyRightVec[0] = BodyRightVec[0]/temp_double_1;
	BodyRightVec[1] = BodyRightVec[1]/temp_double_1;
	BodyRightVec[2] = BodyRightVec[2]/temp_double_1;

	Cross2Vd(BodyUpVec, BodyRightVec, BodyFaceVec);

//�l�hstart111214

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
	pv_stack[44] = -40.689*BodyUpVec[2]-20.625*BodyFaceVec[2];//�U�b��

	
	/*// body block up
	pv_stack[45] = BodyUpVec[0];
	pv_stack[46] = BodyUpVec[1];
	pv_stack[47] = BodyUpVec[2];//�W�b��*/

	// body block up
	pv_stack[45] = UpBodyUpVec[0];
	pv_stack[46] = UpBodyUpVec[1];
	pv_stack[47] = UpBodyUpVec[2];//�W�b��


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
	pv_stack[2] += CrdAll->data[8]; // �b3 knee �� �b2 hip

	pv_stack[3] += CrdAll->data[9];
	pv_stack[4] += CrdAll->data[10];
	pv_stack[5] += CrdAll->data[11]; // �b4 ankle �� �b3 knee

	pv_stack[6] += CrdAll->data[12];
	pv_stack[7] += CrdAll->data[13];
	pv_stack[8] += CrdAll->data[14]; // �b7 foot �� �b4 ankle


	//  right leg
	pv_stack[9] += CrdAll->data[45];
	pv_stack[10] += CrdAll->data[46];
	pv_stack[11] += CrdAll->data[47]; // �b16 knee �� �b15 hip

	pv_stack[12] += CrdAll->data[48];
	pv_stack[13] += CrdAll->data[49];
	pv_stack[14] += CrdAll->data[50]; // �b17 ankle �� �b16 knee

	pv_stack[15] += CrdAll->data[51];
	pv_stack[16] += CrdAll->data[52];
	pv_stack[17] += CrdAll->data[53]; // �b20 foot �� �b17 ankle

	//�l�hstart111214
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
	// �V�q�~�n
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
	// �V�q�~�n
	******************************************************************/
	v3[0] = v1[1]*v2[2]-v1[2]*v2[1];
	v3[1] = v1[2]*v2[0]-v1[0]*v2[2];
	v3[2] = v1[0]*v2[1]-v1[1]*v2[0];
}

double Kine::NormXYZD(double* Vec)
{
	/******************************************************************
	input: vec �n�D�Xnorm���V�q
	output: �^��norm

	Note:
	// �V�q�~�n
	******************************************************************/
	return sqrt(Vec[0]*Vec[0]+Vec[1]*Vec[1]+Vec[2]*Vec[2]);
}


void Kine::GenSwingTraj(double v0, double a0, double x1_percentage, double y1, double v1, double x2, double y2, double v2, double a2, int Np, int KeepPosCount, double* resultXY, double* resultZ)
{
	/******************************************************************
	input: v0 a0 y1 v1 x2 y2 v2 a2, 0�N���l�I 1�N�����I 2�N�����I x�N���b y�N���a�b�� v a �N���a�b�t�׻P�[�t��
	       x1_percentage �N�����I��m�ʤ���,  Np �N���t�I��, KeepPosCount �N������`�ƪ��I��, resultXY �x�sx�b���G, resultZ �x�sy�b���G
	output: void

	Note:
	// �p��Xswing�}�y�� --> �C���n��X�h���y��
	// KeepPosCount �Y�u�y���F�ɶ� �D�n�Φb�W�ӱ� ��L�����s (���a�i�H�û����s)
	// x1_percentage�N�� x1�b 0~x2�h�֦ʤ��񪺦a��

	******************************************************************/


	// �ѦҶ}�l��m��0
	// x0 = y0 = 0;

	// Na = Np �]�N�OSSP�`�@�I�ƪ��e�b
	Na = (int)(x1_percentage*Np);
	// Na = Np �]�N�OSSP�`�@�I�ƪ���b
	Nb = Np-Na; // �]�tx1���U�@���x2

	for (int i =0 ; i<Nza ; i++)
	{
		resultXY[i] = 0;
		resultZ[i] = 0;
	}

	// ������V �ɥΥ���ZMP�ͦ���� �}�l�P�����t�� �[�t�� jerk = 0
	GenSmoothZMPShift_ZeroJerk(0,x2,Np-KeepPosCount,resultXY+Nza); 
	// ������V �Q�ΤE���h�����ͦ�
	Gen9DegPoly(x1_percentage, y1, y2, Np-KeepPosCount, resultZ+Nza);
	
	x_p = x2;

	for (int i =Nza+Np-KeepPosCount ; i< N_step ; i++)
	{
		resultXY[i] = x_p;
		resultZ[i] = y2;
	}

	#if SaveSwingTraj
 	// �n�s���ɮ״N���}�U���{��
		fstream Fx;
		
		Fx.open("swingx.txt",ios::out);
		Fx.precision(10);
		
		for (int i=0 ; i< N_step ; i++)
		{
			Fx << SwingBufferx[i] <<  endl;
		}
		
		Fx.close();
			
		Fx.open("swingz.txt",ios::out| ios::trunc);  // ����^
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
	input: v0 a0 y1 v1 xy2 z2 v2 a2, 0�N���l�I 1�N�����I 2�N�����I xy�N���b(gStrideX gStrideY) z�N���a�b��(gStrideZ) v a �N���a�b�t�׻P�[�t��
	       x1_percentage �N�����I��m�ʤ���,  Np �N���t�I��, KeepPosCount �N������`�ƪ��I��, resultXY �x�sx y�b���G, resultZ �x�sz�b���G
	output: void

	Note:
	// �p��Xswing�}�y�� --> �C���n��X�h���y��
	// KeepPosCount �Y�u�y���F�ɶ� �D�n�Φb�W�ӱ� ��L�����s (���a�i�H�û����s)
	// x1_percentage�N�� x1�b 0~x2�h�֦ʤ��񪺦a��
	// Mod������swing�}������_ �o�˴N�i�H���欰�󹳤H �H�� �󤣮e���b�e�i������}�F��joint limit���樫�󶶧Q
	******************************************************************/

	// Na�OSSP�`�@�I�ƪ��e�b
	Na = (int)(x1_percentage*Np);
	// Nb�OSSP�`�@�I�ƪ���b 
	Nb = Np-Na; // �]�tx1���U�@���x2

	int NWait = int(Nza*0.25); // 25% of Nza, Nza is not used now
	if (gUpStair){
		NWait=Nza*0.30;
	}
	/******************************************************************
	Flag
		gUpStair ���ӱ�ϥ� �w�益���N�}���O�P���誺�Z���̤j��(�վ�ɶ������Ѽ�)
	******************************************************************/		

	for (int i =0 ; i<NWait ; i++)
	{
		resultXY[i] = 0;
		resultZ[i] = 0;
	}
	//// ������V �ɥΥ���ZMP�ͦ���� �}�l�P�����t�� �[�t�� jerk = 0
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


	// ������V �Q�ΤE���h�����ͦ�
	if(z2 >= 15 || z2 <= -15 )
	{
		//GenZMPFreeAssign(1.5,0,0,0,0,y2/2+30,0.30,0,0,300,resultZ+NWait);
		//GenZMPFreeAssign(1.5,y2/2+30,0.30,0,0,y2+20,0.1,0,0, 300,resultZ+NWait+300);
		//GenZMPFreeAssign(2.0,y2+20,0.1,0,0,y2,0,0,0,400,resultZ+NWait+600);
		//GenZMPFreeAssign(1.0,y2,0,0,0,y2,0,0,0,200,resultZ+NWait+1000);
			// for �涥slongz
		//GenZMPFreeAssign(3,0,0,0,0,y2+30,0.30,0,0,400-100,resultZ+NWait+100);
		////GenZMPFreeAssign(1.5,y2/2+30,0.30,0,0,y2+20,0.1,0,0, 300,resultZ+NWait+300);
		////GenZMPFreeAssign(2.0,y2+20,0.1,0,0,y2,0,0,0,400,resultZ+NWait+600);
		//GenZMPFreeAssign(3.0,y2+30,0,0,0,y2,0,0,0,700,resultZ+NWait+400);
		//GenZMPFreeAssign(3.0,y2,0,0,0,y2,0,0,0,100,resultZ+NWait+1100);
			// for �涥slongz
		Gen9DegPolyStair(x1_percentage, z1, z2, Np-KeepPosCount+Nza-NWait, resultZ+NWait);
	}
	else	// ���a
	{
		Gen9DegPoly(x1_percentage, z1, z2, Np-KeepPosCount+Nza-NWait, resultZ+NWait);
	}
 	//Gen10DegPoly(x1_percentage, y1,0.25,105,0.75,-105, y2, Np-KeepPosCount+Nza-NWait, resultZ+NWait);
	x_p = xy2;

	for (int i =Nza+Np-KeepPosCount ; i< N_step ; i++)	// ��Nzb ����qDSP 
	{
		resultXY[i] = x_p;
		resultZ[i] = z2;
	}

	for (int i=0 ; i< N_step ; i++)
		AnklePitchRef[i]=0;

	#if SaveSwingTraj
	// �n�s���ɮ״N���}�U���{��
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
	input: v0 a0 y1 v1 x2 y2 v2 a2, 0�N���l�I 1�N�����I 2�N�����I x�N���b y�N���a�b�� v a �N���a�b�t�׻P�[�t��
	       x1_percentage �N�����I��m�ʤ���,  Np �N���t�I��, KeepPosCount �N������`�ƪ��I��, resultXY �x�sx�b���G, resultZ �x�sy�b���G
	output: void

	Note:
	// �p��Xswing�}�y�� --> �C���n��X�h���y��
	// KeepPosCount �Y�u�y���F�ɶ� �D�n�Φb�W�ӱ� ��L�����s (���a�i�H�û����s)
	// x1_percentage�N�� x1�b 0~x2�h�֦ʤ��񪺦a��
	// Mod������swing�}������_ �o�˴N�i�H���欰�󹳤H �H�� �󤣮e���b�e�i������}�F��joint limit
	// ���樫�󶶧Q
	//Mod2�W�[�}��_��A���樫��۵M
	//�����i���e���IĲjoint limit �V��!!
	******************************************************************/

	// �ѦҶ}�l��m��0
	// x0 = y0 = 0;

	// Na�OSSP�`�@�I�ƪ��e�b
	Na = (int)(x1_percentage*Np);
	// Nb�OSSP�`�@�I�ƪ���b
	Nb = Np-Na; // �]�tx1���U�@���x2

	int NWait = int(Nza*0.25); // 25% of Nza, Nza is not used now,

	for (int i =0 ; i<NWait ; i++)
	{
		resultXY[i] = 0;
		resultZ[i] = 0;
	}

	// ������V �ɥΥ���ZMP�ͦ���� �}�l�P�����t�� �[�t�� jerk = 0
	GenSmoothZMPShift_ZeroJerk(0,x2,Np-KeepPosCount+Nza-NWait,resultXY+NWait); 
	// ������V �Q�ΤE���h�����ͦ�
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
	// �n�s���ɮ״N���}�U���{��
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
	input: v0 a0 y1 v1 x2 y2 v2 a2, 0�N���l�I 1�N�����I 2�N�����I x�N���b y�N���a�b�� v a �N���a�b�t�׻P�[�t��
	       x1_percentage �N�����I��m�ʤ���,  Np �N���t�I��, KeepPosCount �N������`�ƪ��I��, resultXY �x�sx�b���G, resultZ �x�sy�b���G
	output: void

	Note:
	// �p��Xswing�}�y�� --> �C���n��X�h���y��
	// KeepPosCount �Y�u�y���F�ɶ� �D�n�Φb�W�ӱ� ��L�����s (���a�i�H�û����s)
	// x1_percentage�N�� x1�b 0~x2�h�֦ʤ��񪺦a��
	// Mod������swing�}������_ �o�˴N�i�H���欰�󹳤H �H�� �󤣮e���b�e�i������}�F��joint limit
	// ���樫�󶶧Q
	//Mod2�W�[�}��_��A���樫��۵M//�����i���e���IĲjoint limit �V��!!//????????????? �O�W�����Ө禡�a
	******************************************************************/

	// �ѦҶ}�l��m��0
	// x0 = y0 = 0;

	// Na�OSSP�`�@�I�ƪ��e�b
	Na = (int)(x1_percentage*Np);
	// Nb�OSSP�`�@�I�ƪ���b
	Nb = Np-Na; // �]�tx1���U�@���x2

	//int NWait = int(Nza*0.25); // 25% of Nza, Nza is not used now,

	//for (int i =0 ; i<NWait ; i++)
	//{
	//	resultXY[i] = 0;
	//	resultZ[i] = 0;
	//}

	// ������V �ɥΥ���ZMP�ͦ���� �}�l�P�����t�� �[�t�� jerk = 0
	GenSmoothZMPShift_ZeroJerk(0,x2,Np-KeepPosCount+Nza,resultXY); 
	// ������V �Q�ΤE���h�����ͦ�
	Gen9DegPoly(x1_percentage, y1, y2, Np-KeepPosCount+Nza, resultZ);
	
	//x_p = x2;

	//for (int i =Np-KeepPosCount+Nza ; i< Np ; i++)
	//{
	//	resultXY[i] = x_p;
	//	resultZ[i] = y2;
	//}

// �n�s���ɮ״N���}�U���{��
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
	input: x1 y1 y2, 1�N���l�I 2�N�����I x�N���b y�N���a�b��
	       Np �N���t�I��, result�x�s���G
	output: void

	Note:
	// �p��C���h�����y�񤺴�
	// x1 ��J�нզb 0.43~0.57 ����
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
	input: x1 y1 y2, 1�N���l�I 2�N�����I x�N���b y�N���a�b��
	       Np �N���t�I��, result�x�s���G
	output: void

	Note:
	// �p��E���h�����y�񤺴�
	// x1 ��J�нզb 0.44~0.56 ����
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
	input: x1 y1 y2, 1�N���l�I 2�N�����I x�N���b y�N���a�b��
	       Np �N���t�I��, result�x�s���G
	output: void
	
	����ﳡ��

	Note:
	// x1�н�0.35~0.26
	// �p��W�ӱ�Z��V �N��q�ѤE���h���������ұo���y��b�̰��I(StepHeight+GroundHeight[i+1])
	// �b���O�N50%���E�������b�̰��I�P%�Ƹ��֪����X
	// �N50%�����t�I�Ʃ�j���쥻(1- x1)���⭿ �M����᭱�@�b�P%�Ƹ��֪��α� ���`�N��Jresult���int double�㰣���D �p�h�y�񤣳s�� �j�h�{��crash
	// x1 ��J�V��(%)�V�֩�}��L�ӱ�

	// �b�U�ӱ誺�W���� �ؼЧ��J��y1 y2 ��@�W�ӱ�ӭp��
	// �A�Q��resultTemp��W�ӱ誺�y��s�U�� �ϧǥ�i�U�ӱ誺�y��
	// Wei-Zh Lai 20121123
	******************************************************************/
//�Y�n�α��⦸���� ���N�b��@�� ���OZ���Z��(y1)�OStepHeight(y1-y2)
	
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
	

	//�N��b0.5������ �s�J��b��index��  




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
	for (int i = 1 ; i <x1*Np ; i++)  // �e�b�ϰ쪺�I��
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
	input: x1 y1 y2, 1�N���l�I 2�N�����I x�N���b y�N���a�b��
	       Np �N���t�I��, result�x�s���G
	output: void

	Note:
	// �p��Q���h�����y�񤺴�
	// x1 ��J�нզb 0.44~0.56 ����
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
	input: y0 y1, 0�N���l�I 1�N�����I x�N���b y�N���a�b��
	       Np �N���t�I��, result�x�s���G
	output: void

	Note:
	// �p��C���h�����y�񤺴�
	// �ͦ����h�����D���� x��0~Np y��y0~y1
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

	double Step = 1.0/double(Np-1);	// ��Ĥ@�B

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
	input: tCOG �N�� target COG, tSwing �N�� target Swing, tRSwing �N�� target Swing rotation matrix,
	       tRFixed �N�� target Fixed rotation matrix, status �^��IK�O�_���\

	output: void

	Note:
	// ��IK���U��end-effector�F��ؼЦ�m�P����

	// �������x�}���ѻ�
		// �U���ѻ��D�X���P���Шt�������t�� �åB���IK input����k
		// ���]����۪񧤼Шt(R1 R2 ���O3x3�x�})
		// �n�NR1���T�Ӷb �����R2�W�A ��������R��R1�W
		// R�O�L�p����q������x�}�A ���O�k�� �]�����жb�O�V�q �O�Q���઺�ؼ�
		// �ҥH R x R1 = R2, R = R2 x R1' �p�P�U�����l
		// �ӥB ���] R = RxRyRz �åB�O�i�洫���L�p����
		// RxRyRz = [1 -z y ; z 1 -x ; -y x 1]; 
		// �ҥH�i�H���XDiffRotMatSw DiffRotMatBody �b�@�ɤU������b
		// �����N������b���V�q��JIK �N�i�H��end-effector����F!!
	******************************************************************/

	// selIK = 0 : left foot is the supporter
    // selIK = 1 : right foot is the supporter

	//SwErrLim = 0.1; // swing ��IK �~�t��
	//COGErrLim = 0.1; // COG ��IK �~�t��
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

		//�l�hstart111227
		LArmErr[0] = tLArm[0] - CrdAll->data[105];
		LArmErr[1] = tLArm[1] - CrdAll->data[106];
		LArmErr[2] = tLArm[2] - CrdAll->data[107];
		RArmErr[0] = tRArm[0] - CrdAll->data[135];
		RArmErr[1] = tRArm[1] - CrdAll->data[136];
		RArmErr[2] = tRArm[2] - CrdAll->data[137];
		//�l�hend111227

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
		Cross2Vd(temp_vec1, temp_vec2, temp_vec3);	// TarRotMBody�s��Z�b ��XY�~�n�Ө�

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

		//MaxSwingError = 0.5; // maximum input position trajectory difference ������c�Ĥӧ�
	    // 0.5 * 200 = 100mm/s �̧֨C��10cm
	    //MaxCOGError = 0.25; //maximum input position trajectory difference ������c�Ĥӧ�
	    // 0.25 * 200 = 50mm/s �̧֨C��5cm
	    //MaxRotationError = 0.0035; // maximum input angle trajectory difference ������c�Ĥӧ�
    	// 0.0035 * 200 * 57 �C��̧�40��
		if (NormXYZD(SwingErr) >=  MaxSwingError)
		{
			printf("\nĵ�i!! ��J Swing �y�񤣳s�� �Ϊ� IK �z��!!\n");
			printf("error = %f mm\n",NormXYZD(SwingErr));
		//	system("pause");
		}
		if (NormXYZD(COGErr) >=  MaxCOGError)
		{
			printf("\nĵ�i!! ��J COG �y�񤣳s�� �Ϊ� IK �z��!!\n");
			printf("error = %f mm\n",NormXYZD(COGErr));
		//	system("pause");
		}
		if (NormXYZD(temp_vec2) >=  MaxRotationError)
		{
			printf("\nĵ�i!! ��J Swing���� �y�񤣳s�� �Ϊ� IK �z��!!\n");
			printf("rotation speed = %f X 200/s\n",NormXYZD(temp_vec2));
		//	system("pause");
		}
		if (NormXYZD(temp_vec1) >=  MaxRotationError)
		{
			printf("\nĵ�i!! ��J Stance Foot ���� �y�񤣳s�� �Ϊ� IK �z��!!\n");
			printf("rotation speed = %f X 200/s\n",NormXYZD(temp_vec1));
		//	system("pause");
		}
		if(IKMode == 0)
		{
			if (NormXYZD(temp_vec3) >=  MaxRotationError)
			{
				printf("\nĵ�i!! ��J LeftArm ���� �y�񤣳s�� �Ϊ� IK �z��!!\n");
				printf("rotation speed = %f X 200/s\n",NormXYZD(temp_vec3));
			//	system("pause");
			}
			if (NormXYZD(temp_vec4) >=  MaxRotationError)
			{
				printf("\nĵ�i!! ��J RightArm ���� �y�񤣳s�� �Ϊ� IK �z��!!\n");
				printf("rotation speed = %f X 200/s\n",NormXYZD(temp_vec4));
			//	system("pause");
			}
			if (NormXYZD(LArmErr) >=  MaxSwingError)
			{
				printf("\nĵ�i!! ��J LArmErr �y�񤣳s�� �Ϊ� IK �z��!!\n");
				printf("error = %f mm\n",NormXYZD(LArmErr));
			//	system("pause");
			}
			if (NormXYZD(RArmErr) >=  MaxSwingError)
			{
				printf("\nĵ�i!! ��J RArmErr �y�񤣳s�� �Ϊ� IK �z��!!\n");
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
		else if(IKMode == 1)	// �u�Ѹ}
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

		ComputeJacobians();	// �b30���H�� �S���Ѧ��\
		cntIK += 1;


	// when selIK = 0,   dx = [sw_x sw_y sw_z sw_tx sw_ty sw_tz fix_tx fix_ty fix_tz COGx COGy COGz ]
	// when selIK = 1,2, dx = [sw_x sw_y sw_z sw_tx sw_ty sw_tz fix_tx fix_ty fix_tz COGx COGy COGz ]
	// �Q�Ʀ��ۦP��!!

	// dth => [���}6�b ; �k�}6�b]
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

		////// ���]�tWLN ��IK
		////InvSqMat(Ja->data,12);
		////MatMulAB(Ja->data,12,12,dx,12,1,dth);
		////// ���]�tWLN ��IK


		//// �]�tWLN ��IK
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
					printf("\nĵ�i!! �ѥX����%d�b���t�׹L�� �F�� �C��%f��\n",i+1,dth[i]/3.1415926*180*200);
					printf("�i��O��J�y�񤣳s��Ϊ�IK�z��\n");
					//system("pause");
				}
			}
			for(int i = 0;i<12;i++)
			{
				if (fabs(dth[i+12]) > MaxJointAngleChange)
				{
					//cout<<dth[i]/3.1415926*180*200<<endl;
					printf("\nĵ�i!! �ѥX����%d�b���t�׹L�� �F�� �C��%f��\n",i+1+12,dth[i+12]/3.1415926*180*200);
					printf("�i��O��J�y�񤣳s��Ϊ�IK�z��\n");
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
					printf("\nĵ�i!! �ѥX����%d�b���t�׹L�� �F�� �C��%f��\n",i+1,dthArmOffline[i]/3.1415926*180*200);
					printf("�i��O��J�y�񤣳s��Ϊ�IK�z��\n");
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

		//// �]�tWLN ��IK

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

	// �ˬd�����H�O�_�W�L���׷��� �Y�W�L �h����D�� �{���Ȱ�
	CheckJointLimit();

}

void Kine::IKFixSolve(double* tCOG, double* tSwing, double* tRSwing, double* tRFixed, int* status)
{
	/******************************************************************
	input: tCOG �N�� target COG, tSwing �N�� target Swing, tRSwing �N�� target Swing rotation matrix,
	       tRFixed �N�� target Fixed rotation matrix, status �^��IK�O�_���\

	output: void

	Note:
	// ��IK���U��end-effector�F��ؼЦ�m�P����

	// �������x�}���ѻ�
		// �U���ѻ��D�X���P���Шt�������t�� �åB���IK input����k
		// ���]����۪񧤼Шt(R1 R2 ���O3x3�x�})
		// �n�NR1���T�Ӷb �����R2�W�A ��������R��R1�W
		// R�O�L�p����q������x�}�A ���O�k�� �]�����жb�O�V�q �O�Q���઺�ؼ�
		// �ҥH R x R1 = R2, R = R2 x R1' �p�P�U�����l
		// �ӥB ���] R = RxRyRz �åB�O�i�洫���L�p����
		// RxRyRz = [1 -z y ; z 1 -x ; -y x 1]; 
		// �ҥH�i�H���XDiffRotMatSw DiffRotMatBody �b�@�ɤU������b
		// �����N������b���V�q��JIK �N�i�H��end-effector����F!!
	******************************************************************/

	// selIK = 0 : left foot is the supporter
    // selIK = 1 : right foot is the supporter

	//SwErrLim = 0.1; // swing ��IK �~�t��
	//COGErrLim = 0.1; // COG ��IK �~�t��
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

		//MaxSwingError = 0.5; // maximum input position trajectory difference ������c�Ĥӧ�
	    // 0.5 * 200 = 100mm/s �̧֨C��10cm
	    //MaxCOGError = 0.25; //maximum input position trajectory difference ������c�Ĥӧ�
	    // 0.25 * 200 = 50mm/s �̧֨C��5cm
	    //MaxRotationError = 0.0035; // maximum input angle trajectory difference ������c�Ĥӧ�
    	// 0.0035 * 200 * 57 �C��̧�40��
		//if (NormXYZD(SwingErr) >=  MaxSwingError)
		//{
		//	printf("\nĵ�i!! ��J Swing �y�񤣳s�� �Ϊ� IK �z��!!\n");
		//	printf("error = %f mm\n",NormXYZD(SwingErr));
		//	system("pause");
		//}
		if (NormXYZD(COGErr) >=  MaxCOGError)
		{
			printf("\nĵ�i!! ��J COG �y�񤣳s�� �Ϊ� IK �z��!!\n");
			printf("error = %f mm\n",NormXYZD(COGErr));
			system("pause");
		}
		//if (NormXYZD(temp_vec2) >=  MaxRotationError)
		//{
		//	printf("\nĵ�i!! ��J Swing���� �y�񤣳s�� �Ϊ� IK �z��!!\n");
		//	printf("rotation speed = %f X 200/s\n",NormXYZD(temp_vec2));
		//	system("pause");
		//}


		if (NormXYZD(temp_vec1) >=  MaxRotationError)
		{
			printf("\nĵ�i!! ��J Stance Foot ���� �y�񤣳s�� �Ϊ� IK �z��!!\n");
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
	// �Q�Ʀ��ۦP��!!

	// dth => [���}6�b ; �k�}6�b]
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

		////// ���]�tWLN ��IK
		InvSqMat(FixJa->data,6);
		MatMulAB(FixJa->data,6,6,dx,6,1,dth);
		////// ���]�tWLN ��IK
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



		//// �]�tWLN ��IK
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

		//// �]�tWLN ��IK

		//// PseudoInv
		//MatMulAtB(tempJT,Ja->MRow,Ja->NCol,tempJJT_for_inv,Ja->MRow,Ja->MRow,PseudoInv);


 		for (int i = 0 ; i < 6 ; i++)
		{
			if (fabs(dth[i]) > MaxJointAngleChange)
			{
				printf("\nĵ�i!! �ѥX����%d�b���t�׹L�� �F�� �C��%f��\n",i+1,dth[i]/3.1415926*180*200);
				printf("�i��O��J�y�񤣳s��Ϊ�IK�z��\n");
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

	// �ˬd�����H�O�_�W�L���׷��� �Y�W�L �h����D�� �{���Ȱ�
	//CheckJointLimit();

}

void Kine::IKCCDSolve(double* tCOG, double* tSwing, double* tRSwing, double* tRFixed, int* status)
{
	/******************************************************************
	input: tCOG �N�� target COG, tSwing �N�� target Swing, tRSwing �N�� target Swing rotation matrix,
	       tRFixed �N�� target Fixed rotation matrix, status �^��IK�O�_���\

	output: void

	Note:
	// ��IK���U��end-effector�F��ؼЦ�m�P����

	// �������x�}���ѻ�
		// �U���ѻ��D�X���P���Шt�������t�� �åB���IK input����k
		// ���]����۪񧤼Шt(R1 R2 ���O3x3�x�})
		// �n�NR1���T�Ӷb �����R2�W�A ��������R��R1�W
		// R�O�L�p����q������x�}�A ���O�k�� �]�����жb�O�V�q �O�Q���઺�ؼ�
		// �ҥH R x R1 = R2, R = R2 x R1' �p�P�U�����l
		// �ӥB ���] R = RxRyRz �åB�O�i�洫���L�p����
		// RxRyRz = [1 -z y ; z 1 -x ; -y x 1]; 
		// �ҥH�i�H���XDiffRotMatSw DiffRotMatBody �b�@�ɤU������b
		// �����N������b���V�q��JIK �N�i�H��end-effector����F!!
	******************************************************************/

	// selIK = 0 : left foot is the supporter
    // selIK = 1 : right foot is the supporter

	//SwErrLim = 0.1; // swing ��IK �~�t��
	//COGErrLim = 0.1; // COG ��IK �~�t��

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

		//MaxSwingError = 0.5; // maximum input position trajectory difference ������c�Ĥӧ�
	    // 0.5 * 200 = 100mm/s �̧֨C��10cm
	    //MaxCOGError = 0.25; //maximum input position trajectory difference ������c�Ĥӧ�
	    // 0.25 * 200 = 50mm/s �̧֨C��5cm
	    //MaxRotationError = 0.0035; // maximum input angle trajectory difference ������c�Ĥӧ�
    	// 0.0035 * 200 * 57 �C��̧�40��
		//if (NormXYZD(SwingErr) >=  MaxSwingError)
		//{
		//	printf("\nĵ�i!! ��J Swing �y�񤣳s�� �Ϊ� IK �z��!!\n");
		//	printf("error = %f mm\n",NormXYZD(SwingErr));
		//	system("pause");
		//}
		if (NormXYZD(COGErr) >=  MaxCOGError)
		{
			printf("\nĵ�i!! ��J COG �y�񤣳s�� �Ϊ� IK �z��!!\n");
			printf("error = %f mm\n",NormXYZD(COGErr));
			system("pause");
		}
		//if (NormXYZD(temp_vec2) >=  MaxRotationError)
		//{
		//	printf("\nĵ�i!! ��J Swing���� �y�񤣳s�� �Ϊ� IK �z��!!\n");
		//	printf("rotation speed = %f X 200/s\n",NormXYZD(temp_vec2));
		//	system("pause");
		//}


		if (NormXYZD(temp_vec1) >=  MaxRotationError)
		{
			printf("\nĵ�i!! ��J Stance Foot ���� �y�񤣳s�� �Ϊ� IK �z��!!\n");
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
	// �Q�Ʀ��ۦP��!!

	// dth => [���}6�b ; �k�}6�b]
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

		////// ���]�tWLN ��IK
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
		////// ���]�tWLN ��IK



		//// �]�tWLN ��IK
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

		//// �]�tWLN ��IK

		//// PseudoInv
		//MatMulAtB(tempJT,Ja->MRow,Ja->NCol,tempJJT_for_inv,Ja->MRow,Ja->MRow,PseudoInv);


 		for (int i = 0 ; i < 6 ; i++)
		{
			if (fabs(dth[i]) > MaxJointAngleChange)
			{
				printf("\nĵ�i!! QQ�ѥX����%d�b���t�׹L�� �F�� �C��%f��\n",i+1,dth[i]/3.1415926*180*200);
				printf("�i��O��J�y�񤣳s��Ϊ�IK�z��\n");
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

	// �ˬd�����H�O�_�W�L���׷��� �Y�W�L �h����D�� �{���Ȱ�
	//CheckJointLimit();

}

void Kine::InitWLN(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	   ��l��WLN joint limit avoidance �ҭn���ܼ�
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

	// �Ψӭ���IK�ѧ��W�X���Ӧ������
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
	    �p��WLN joint limit avoid �v���x�}
	******************************************************************/

	// dth => [���}6�b ; �k�}6�b]
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

		if (invWLNMat[i] > LastInvWLN_Mat[i]) // ���ۤϩ�joint limit��V�e�i ����N���ݭn����
		{
			invWLNMat[i] = 1.0;
			IfWLNEqualOne[i] = true;
		} // �Ϥ� �h�ݭn����A�O�d��X���v����
		else
		{
			IfWLNEqualOne[i] = false;
		}

		LastInvWLN_Mat[i] = invWLNMat[i];
	}
	// �{�b�� invWLNMat �O �פ夤 W �x�}���﨤�u�����A�åB�C�ӳ��O�˼� �]�N�O 1/wi

}

void Kine::InvWithSingTest(double* JJT, double* tempJJT_inv, int MRowJJT, long* _ipiv, double* _work, double MinDet, double alph)
{
	/******************************************************************
	input: JJT ��JJ*J', tempJJT_inv ��J�Ȧs�Ϧ�m,  MRowJJT ��JJJT size, 
	       _ipiv _work �OCLAPACK���Ȧs�ϡA MinDet�O�̤p�����\deteriment��,  alph�Osingular�ɭԭn�[�W�o�﨤�u��
	output: void

	Note:
	    �p��O�_��singular configuration

	// JJT �O �Q���ҬO�_ singular ���x�}
	// tempJJT_inv �̫�|�s�� inverse���G
	// ���Y�p�� MinDet�h�Q�{�� singular
	// �Q�{��singular�H�� �﨤�u�C�泣�|�Q�[�W alph ��@ singularity avoidance
	// JJT �O��}�AMRowJJT �N��� ���P�e����
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
	

	// detValue = U�x�}���﨤�u�s��
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
	input: RotM ��J����x�} ���ӨD�שԨ�, EulerAng ��X���שԨ�
	       
	output: void

	Note:

	//�q����x�}�p��שԨ�
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

	Note: ���X�}�H�θy�����Шt����x�}
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

	//�l�hstart111227
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

	//�l�hend111227
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

	Note: ��l��inertia matrices
	******************************************************************/	

	// catia and solidworks �q�쪺�C�ӱ��b�@�ɤU�� inertia matrix
	double IR_FPad[9] = {6000, -550.1, 2000, -550.1, 9000, 98.94, 2000, 98.94, 7000}; 
	double IL_FPad[9] = {6000, 546, 2000, 546, 9000, -97.5, 2000, -97.5, 7000}; 
	double IR_KneeDown[9] = {17000, 106.1, -383, 106.1, 16000, 1000, -383, 1000, 3000};
	double IL_KneeDown[9] = {17000, -132.1, -450.3, -132.1, 16000, -1000, -450.3, -1000, 3000};
	double IR_KneeUp[9] = {33000, -200.2, -191.1, -200.2, 28000, -889.7, -191.1, -889.7, 8000};
	double IL_KneeUp[9] = {33000, 196.2, -201.5, 196.2, 28000, 877.5, -201.5, 877.5, 8000};

	// IB1 ��l��n�N�b�@�ɤU�A�ҥH���α���
	IB1[0] = 76000;  IB1[1] = 75;	  IB1[2] = -10000; 
	IB1[3] = 75;	 IB1[4] = 55000;  IB1[5] = 71;
	IB1[6] = -10000; IB1[7] = 71;	  IB1[8] = 57000;

	double IBody[9] = {71000, 84, 94, 84, 87000, 3000, 94, 3000, 85000};  

	//double IR_Arm = {42000, -18.45, -134.8, -18.45, 42000, 54.74, -134.8, 54.74, 2000};
	double IR_Arm[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}; // ��disable ���u�A�G�N�ˤp
	//double IL_Arm = {42000, -19.39, -136.3, -19.39, 42000, -76.87, -136.3, -76.87, 2000};
	double IL_Arm[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}; // ��disable ���u�A�G�N�ˤp

	// ��j1000�� ����ܦ� g.mm^2 (catia �̭��O kg.m^2 ���W10^9 �N�|�ܦ� g.mm^2)
	

                    
//clear IL_Arm IR_Arm IBody;
//clear A_RL0 A_LL0 A_RA0 A_LA0;         

//////////////////////IRL = [IR_Foot IR_Shank IR_Thigh]*1000;
//////////////////////ILL = [IL_Foot IL_Shank IL_Thigh]*1000;
//////////////////////IB =  [IB1 IB2].*1000;
//////////////////////ILA=ILA.*1000;
//////////////////////IRA=IRA.*1000;
//////////////////////clear IB1 IB2 IR_Foot IL_Foot IR_Shank IL_Shank IR_Thigh IL_Thigh
//////////////////////
//////////////////////warndlg('\n\n�Фp��!! momentum�A�J�����s���ɭ�(�����H���O����e�誺�ɭ�)�A�|�����D�A�y�Ф]�n��!!\n\n')
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

	// ���Ntheta�]��0�A����DH���� �_��theta�ȡA�ӱ`����{��
	FKLLeg->DHConstruct();
	FKRLeg->DHConstruct();
	FKLArm->DHConstruct();
	FKRArm->DHConstruct();

	////// IL_Foot = A_LL0(1:3,25:27)'*IL_FPad*A_LL0(1:3,25:27); // ��6�b Rn5
	GetRotPartRn(&FKLLeg->Rn[5],Rot_Part_Rn);
	MatMulAB(IL_FPad,3,3,Rot_Part_Rn,3,3,temp_compute);
	MatMulAtB(Rot_Part_Rn,3,3,temp_compute,3,3,IL_Foot);

	////// IL_Shank = A_LL0(1:3,17:19)'*IL_KneeDown*A_LL0(1:3,17:19); // ��4�b Rn3
	GetRotPartRn(&FKLLeg->Rn[3],Rot_Part_Rn);
	MatMulAB(IL_KneeDown,3,3,Rot_Part_Rn,3,3,temp_compute);
	MatMulAtB(Rot_Part_Rn,3,3,temp_compute,3,3,IL_Shank);

	//////IL_Thigh =  A_LL0(1:3,13:15)'*IL_KneeUp*A_LL0(1:3,13:15); // ��3�b Rn2
	GetRotPartRn(&FKLLeg->Rn[2],Rot_Part_Rn);
	MatMulAB(IL_KneeUp,3,3,Rot_Part_Rn,3,3,temp_compute);
	MatMulAtB(Rot_Part_Rn,3,3,temp_compute,3,3,IL_Thigh);

	////// IR_Foot = A_RL0(1:3,25:27)'*IR_FPad*A_RL0(1:3,25:27); // ��6�b Rn5
	GetRotPartRn(&FKRLeg->Rn[5],Rot_Part_Rn);
	MatMulAB(IR_FPad,3,3,Rot_Part_Rn,3,3,temp_compute);
	MatMulAtB(Rot_Part_Rn,3,3,temp_compute,3,3,IR_Foot);

	////// IR_Shank = A_RL0(1:3,17:19)'*IR_KneeDown*A_RL0(1:3,17:19); // ��4�b Rn3
	GetRotPartRn(&FKRLeg->Rn[3],Rot_Part_Rn);
	MatMulAB(IR_KneeDown,3,3,Rot_Part_Rn,3,3,temp_compute);
	MatMulAtB(Rot_Part_Rn,3,3,temp_compute,3,3,IR_Shank);

	////// IR_Thigh = A_RL0(1:3,13:15)'*IR_KneeUp*A_RL0(1:3,13:15); // ��3�b Rn2
	GetRotPartRn(&FKRLeg->Rn[2],Rot_Part_Rn);
	MatMulAB(IR_KneeUp,3,3,Rot_Part_Rn,3,3,temp_compute);
	MatMulAtB(Rot_Part_Rn,3,3,temp_compute,3,3,IR_Thigh);

	// ���uDH����: �yyaw->�ypitch->���upitch->���uend-effector
	////// IB2 = A_RA0(1:3,5:7)'*IBody*A_RA0(1:3,5:7);   // ��2�bRn1  % �y���y�Шt���Ө��y����H��|����v�T���ҥH��5~7
	GetRotPartRn(&FKRArm->Rn[1],Rot_Part_Rn);
	MatMulAB(IBody,3,3,Rot_Part_Rn,3,3,temp_compute);
	MatMulAtB(Rot_Part_Rn,3,3,temp_compute,3,3,IB2);

	//////% �㰦�k���u  
	//////IRA = A_RA0(1:3,9:11)'*IR_Arm*A_RA0(1:3,9:11);  // ��3�b Rn2
	GetRotPartRn(&FKRArm->Rn[2],Rot_Part_Rn);
	MatMulAB(IR_Arm,3,3,Rot_Part_Rn,3,3,temp_compute);
	MatMulAtB(Rot_Part_Rn,3,3,temp_compute,3,3,IRA);
	
	//////% �㰦�����u 
	//////ILA = A_LA0(1:3,9:11)'*IL_Arm*A_LA0(1:3,9:11);  // ��3�b Rn2
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

	// �̷�support�}���@�� inertia����l�I�]���P
	for (int i = 0 ; i < 9 ; i++)
	{
		IL_Foot_sup_R[i] = IL_Foot[i];
		IL_Shank_sup_R[i] = IL_Shank[i];
		IL_Thigh_sup_R[i] = IL_Thigh[i];
		IR_Foot_sup_L[i] = IR_Foot[i];
		IR_Shank_sup_L[i] = IR_Shank[i];
		IR_Thigh_sup_L[i] = IR_Thigh[i];

		// support foot ���γQ�L���A�]�����|��
		IR_Foot_sup_R[i] = IR_Foot[i];
		IL_Foot_sup_L[i] = IL_Foot[i];
	}

	// �̷�support�}���@�� inertia����l�I�]���P
	////// IL_Shank = A_LL0(1:3,17:19)'*IL_KneeDown*A_LL0(1:3,17:19); // ��4�b Rn3
	GetRotPartRn(&FKLLeg->Rn[3],Rot_Part_Rn);
	GetRotPartRn(&FKLLeg->Rn[7],Rot_Part_Rn2);
	MatMulAtB(Rot_Part_Rn2,3,3,Rot_Part_Rn,3,3,temp_Rn); // �d�U A7'*A6'*A5'*A4' (���\�H�U)
	MatMulAB(IL_KneeDown,3,3,temp_Rn,3,3,temp_compute);
	MatMulAtB(temp_Rn,3,3,temp_compute,3,3,IL_Shank_sup_L);

	//////IL_Thigh =  A_LL0(1:3,13:15)'*IL_KneeUp*A_LL0(1:3,13:15); // ��3�b Rn2
	GetRotPartRn(&FKLLeg->Rn[2],Rot_Part_Rn);
	MatMulAtB(Rot_Part_Rn2,3,3,Rot_Part_Rn,3,3,temp_Rn); // �d�U A7'*A6'*A5'*A4'*A3' (Hip�H�U)
	MatMulAB(IL_KneeUp,3,3,temp_Rn,3,3,temp_compute);
	MatMulAtB(temp_Rn,3,3,temp_compute,3,3,IL_Thigh_sup_L);

	////// IR_Shank = A_RL0(1:3,17:19)'*IR_KneeDown*A_RL0(1:3,17:19); // ��4�b Rn3
	GetRotPartRn(&FKRLeg->Rn[3],Rot_Part_Rn);
	GetRotPartRn(&FKRLeg->Rn[7],Rot_Part_Rn2);
	MatMulAtB(Rot_Part_Rn2,3,3,Rot_Part_Rn,3,3,temp_Rn); // �d�U A7'*A6'*A5'*A4' (���\�H�U)
	MatMulAB(IR_KneeDown,3,3,temp_Rn,3,3,temp_compute);
	MatMulAtB(temp_Rn,3,3,temp_compute,3,3,IR_Shank_sup_R);

	//////IR_Thigh =  A_RL0(1:3,13:15)'*IR_KneeUp*A_RL0(1:3,13:15); // ��3�b Rn2
	GetRotPartRn(&FKRLeg->Rn[2],Rot_Part_Rn);
	MatMulAtB(Rot_Part_Rn2,3,3,Rot_Part_Rn,3,3,temp_Rn); // �d�U A7'*A6'*A5'*A4'*A3' (Hip�H�U)
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
	input: HomoMat��J��homogeneous matrix, RotPart ��X������x�}
	output: void

	Note: �qhomogeneous matrix ���X����x�}
	******************************************************************/
	// ���X 4*4 �x�}�� ���ೡ��
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
	input: y_start ��l��m, y_end ������m, Np �`���t�I��, result ���G�s�b result
	output: void

	Note: �������ƪ�ZMP���u Jerk���i���w �t�� �[�t�׬�0
	******************************************************************/

	// �ѦҶ}�l��m��0
	// x0 = y0 = 0;

	double y1 = y_end-y_start;
	double a[6]; // 5���h���� �@������ ��J0~1�����h�����i�H�o��}�l�P�����t�׻P�[�t�׳���0�����u (�}�l�by_start �����b y_end)
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
	input: y_start ��l��m, y_end ������m, Np �`���t�I��, result ���G�s�b result
	output: void

	Note: �������ƪ�ZMP���u Jerk �[�t�� �t�׬�0
	******************************************************************/

	// �ѦҶ}�l��m��0
	// x0 = y0 = 0;

	double y1 = y_end-y_start;
	double a[8]; // 7���h���� �@��8�� ��J0~1�����h�����i�H�o��}�l�P�����t�׻P�[�t�׳���0�����u Jerk�]��0 (�}�l�by_start �����b y_end)
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
	input: y_start ��l��m, y_end ������m, Np �`���t�I��, result ���G�s�b result
	output: void

	Note: �������ƪ�ZMP���u Jerk �[�t�� �t�׬�0
	******************************************************************/
	// �ѦҶ}�l��m��0
	// x0 = y0 = 0;
	//�쥻�O����1200���I  �אּ����300���I
	// mode1 == 0 (�W�ӱ�)   mode ==  1 (�U�ӱ�)   

	double y1 = y_end-y_start;
	double a[8]; // 7���h���� �@��8�� ��J0~1�����h�����i�H�o��}�l�P�����t�׻P�[�t�׳���0�����u Jerk�]��0 (�}�l�by_start �����b y_end)
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
	input: x2, y0, v0, a0, j0, y2, v2, a2,j2, ���Ĥ@�I�򵲧��I��m �t�� �[�t�� jerk ���i�H�ۥѿ�J
	       Np �`�����I�ơA���G�s�bresult
	output: void

	Note: �������ƪ�ZMP���u ��m�t�ץ[�t��jerk�������i�H�ۥѫ��w
	// A = [1 1 1 1 ; 4 5 6 7 ; 12 20 30 42 ; 24 60 120 210]; // �h�����ƦC�Y�Ʀ��x�}
	// �C���h���� �ۥѿ�J ��m �t�� �[�t�� jerk �@�K���ܼ�
	// x �b x1 = 0  x2 = input �Ш̷ӯu���ƿ�J �Ъ`�Nx2���O�ɶ�	
	******************************************************************/

	double xpoly[8];
	xpoly[0] = 1;
	xpoly[1] = x2;
	for (int i = 2;i<8;i++)
		xpoly[i] = xpoly[i-1]*x2;

	// ����J�S��inverse���� �����N�n��inverse 
	double invA[16] = {xpoly[4],xpoly[5],xpoly[6],xpoly[7],4*xpoly[3],5*xpoly[4],6*xpoly[5],7*xpoly[6],12*xpoly[2], 20*xpoly[3], 30*xpoly[4], 42*xpoly[5], 24*xpoly[1], 60*xpoly[2], 120*xpoly[3], 210*xpoly[4]};
	InvSqMat(invA,4);

	double p[8]; // 7���h���� �@��8��  (�}�l�by1 �����b y2)
	p[0] = y0; p[1] = v0; p[2] = 0.5*a0; p[3] = j0/6.0;

	// ���F a4~a7 �H�~������ �n�� invA*r �D�X a4~a7
	double r[4] = {y2-y0-v0*xpoly[1]-0.5*a0*xpoly[2]-j0*xpoly[3]/6.0, v2-v0-a0*xpoly[1]-0.5*j0*xpoly[2], a2-a0-j0*xpoly[1], j2-j0};
	MatMulAB(invA,4,4,r,4,1,p+4);

	// �u�ʤ��� x�b 0~1
    double step_int = x2/double(Np-1);

	double* x;
	x = new double[Np];

	x[0] = 0;
	for (int i = 1 ; i < Np-1; i++)
	{
		x[i] = x[i-1] + step_int;
	}
	x[Np-1] = 1.0;


	double x_acc[8]; // �O��U��x �b 1~7����

	// ��X�C���h�������G
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

	// �M���ʺA�O����
	delete[] x;

}



void Kine::FindPseudoJ(void)
{
	/******************************************************************
	input: void
	output: void

	Note: �p��Pseudo Jacobian matrix
	      �������ƨϥ�FK���禡�Ө��oJ+
	******************************************************************/

	int MatIndex;
	ComputeJacobians();
	FindWLN();

	// �]�tWLN ��IK
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

	// �]�tWLN ��IK

	// PseudoInv
	MatMulAtB(tempJT,Ja->MRow,Ja->NCol,tempJJT_for_inv,Ja->MRow,Ja->MRow,PseudoInv);

}

void Kine::SetIdentity(double* RotationMatrix)
{
	/******************************************************************
	input: ��J RotationMatrix�A�åB�N��]��identity matrix
	output: void

	Note: ��J RotationMatrix�A�åB�N��]��identity matrix
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

	Note:�۰ʽT�{��U�ʧ@�O�_�W�L���c����
	******************************************************************/

	for (int i = 0 ; i < 6 ; i++)
	{
		if (FKLLeg->theta[i+1] > JointUpLimitLL[i])
		{
			printf("\nĵ�i!!! ���}��%d�b����%f�׶W�L�W��%f��\n",i+1,FKLLeg->theta[i+1]/3.1415926*180,JointUpLimitLL[i]/3.1415926*180);
			system("pause");
		}
		if (FKRLeg->theta[i+1] > JointUpLimitRL[i])
		{
			printf("\nĵ�i!!! �k�}��%d�b����%f�׶W�L�W��%f��\n",i+1,FKRLeg->theta[i+1]/3.1415926*180,JointUpLimitRL[i]/3.1415926*180);
			system("pause");
		}
		if (FKLLeg->theta[i+1] < JointLoLimitLL[i])
		{
			printf("\nĵ�i!!! ���}��%d�b����%f�פp��U��%f��\n",i+1,FKLLeg->theta[i+1]/3.1415926*180,JointLoLimitLL[i]/3.1415926*180);
			system("pause");
		}
		if (FKRLeg->theta[i+1] < JointLoLimitRL[i])
		{
			printf("\nĵ�i!!! �k�}��%d�b����%f�פp��U��%f��\n",i+1,FKRLeg->theta[i+1]/3.1415926*180,JointLoLimitRL[i]/3.1415926*180);
			system("pause");
		}
	}
}

void Kine::UpdateDrawingBuffer(void) // ��sø�ϡA�u���bFK�����ɰ���
{
	/******************************************************************
	input: void
	output: void

	Note:FK������ ��J�s���y�Э�
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

	// ���u
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
	// �B��Ҧ�Dynamics�禡 �ت����o��U�bTorque
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
	// ��l�ƩҦ�Dynamics�ܼ� �]�A�O�W���k�s
	// �ثe��bStart��ҰʮɩI�s���禡
	// FindDIni�аȥ���bFindCOG���� �]���b�p��W�b����ʺD�q�ɷ|�Ψ�
	// 2012 Slongz Start
	// 20121210 WeiZh Lai Start
	******************************************************************/
	AxisJump = 3*6;
	DHJump = 3*LegDHLen;
	StackJump = 3*3;

	//for(int i = 0 ; i < 3 ; i++){	//�O�W���k�s
	//FSensor_forcL[i] = 0 ;
	//FSensor_forcR[i] = 0 ;
	//FSensor_TorqL[i] = 0 ;
	//FSensor_TorqR[i] = 0 ;
	//}
	
	// �b���q�w�@�ɤU�����O�V�q g*[0 0 -1]
		GravityZaxi[0]=0;
		GravityZaxi[1]=0;
		GravityZaxi[2]=-1*GravityConst;

	// �p��W�b�����q
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
	
	// �p�⥪���u���ߦ�m(�����ӻH)
	int Index = 18;
	for (int i = 7 ; i < 10 ; i++){
	LArmCOM[0] += pv_stack[Index]*mass_com[i];
	LArmCOM[1] += pv_stack[Index+1]*mass_com[i];
	LArmCOM[2] += pv_stack[Index+2]*mass_com[i];
	Index += 3;
	}

	// �p��k���u���ߦ�m(�����ӻH)
	Index = 30;
	for (int i = 11 ; i < 14 ; i++){
	RArmCOM[0] += pv_stack[Index]*mass_com[i];
	RArmCOM[1] += pv_stack[Index+1]*mass_com[i];
	RArmCOM[2] += pv_stack[Index+2]*mass_com[i];
	Index += 3;
	}

	// �p��W�b�����ߦ�m �b�o�̧�CalCoord�������l���ӳo�̭p�� �]��Initial�u���@��
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

	// �H�U����߮y�дy�z�ӱ����ʺD�q ���@�w�� �G�b���p��
	FindInertia2LocalCOM();
	
	//for(int i = 0 ; i < 12 ; i++)
	//	MotorTorq[i] = 0;

	CountMotor = 0;
			
	for(int i = 0 ; i < 36 ; i++){
	ForceJ[i];	
	TorqueJ[i];	
	}

	//// initialize Kalman Filter
	//	// �ե�KF initial�Ȯɥ��}
	//	for (int i = 0 ; i < 12 ; i ++)
	//	{
	//		x_est_last[i] = 0;
	//		x_est_lastMotor[i] = 0;
	//	//}

	//	//�Ĥ@���Ȫ����qAdams���Y�����ɪ��O�W�ȶפJ
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
	//	//// �Ĥ@��Motor�Ȫ����qAdams���Y�����ɪ�Motor�ȶפJ
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
	//	Q_KF[i] = 0.05;	// Q��model��coverence �V�p�V�۫H
	//	R_KF[i] = 3;	// R��measuremant��coverence �V�p�V�۫H

	//	P_lastMotor[i] = 0;
	//	QMotor[i] = 0.1;	// Q��model��coverence �V�p�V�۫H
	//	RMotor[i] = 1;	// R��measuremant��coverence �V�p�V�۫H
	//	}


	//// �Q�α���KF���� Q��R �ӹF��@�}�l����Singular�����A
	//	for (int i = 0 ; i < 12 ; i++)
	//	{
	//		P_last[i] = 0;
	//	//the noise in the system
	//		Q[i] = 0.1;	// Q��model��coveriance �V�p�V�۫H
	//		R[i] = 0.5;	// R��measuremant��coveriance �V�p�V�۫H

	//		P_lastMotor[i] = 0;
	//	//the noise in the system
	//		QMotor[i] = 0.1;	// Q��model��coveriance �V�p�V�۫H
	//		RMotor[i] = 0.07;	// R��measuremant��coveriance �V�p�V�۫H		
	//	}	
	//
	// �����O�W��m �쥻�O�Wr�O�b�}���O ���W��80
	// ����q�S�� ���F�O�_���T? �O�_�p������T?
		SensorOffset[0] = 0;
		SensorOffset[1] = 0;
		SensorOffset[2] = -60;
	////////////////////////SlongZ �ΥH�bDSP�ɤ��ΤO�W�ӥ�ZMP���t�O�q/////////////////////////
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
	////////////////////////SlongZ �ΥH�bDSP�ɤ��ΤO�W�ӥ�ZMP���t�O�q/////////////////////////
}

void Kine::CalCoord(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// �B��Dynamics�|�Ψ쪺�Ҧ���m�V�q
	// 2012 Slongz Start
	// 20121221 WeiZh Lai Start
	******************************************************************/
	// �p�⥪�k�}�U�b�b�@�ɤ�����m�V�q
	//	left leg
	Rstack[0] = CrdAll->data[9]-CrdAll->data[6];
	Rstack[1] = CrdAll->data[10]-CrdAll->data[7];
	Rstack[2] = CrdAll->data[11]-CrdAll->data[8]; // �b3 knee �� �b2 hip

	Rstack[3] = CrdAll->data[12]-CrdAll->data[9];
	Rstack[4] = CrdAll->data[13]-CrdAll->data[10];
	Rstack[5] = CrdAll->data[14]-CrdAll->data[11]; // �b4 ankle �� �b3 knee

	Rstack[6] = CrdAll->data[21]-CrdAll->data[12];
	Rstack[7] = CrdAll->data[22]-CrdAll->data[13];
	Rstack[8] = CrdAll->data[23]-CrdAll->data[14]; // �b7 foot �� �b4 ankle

	//  right leg
	Rstack[9] = CrdAll->data[48]-CrdAll->data[45];
	Rstack[10] = CrdAll->data[49]-CrdAll->data[46];
	Rstack[11] = CrdAll->data[50]-CrdAll->data[47]; // �b16 knee �� �b15 hip

	Rstack[12] = CrdAll->data[51]-CrdAll->data[48];
	Rstack[13] = CrdAll->data[52]-CrdAll->data[49];
	Rstack[14] = CrdAll->data[53]-CrdAll->data[50]; // �b17 ankle �� �b16 knee

	Rstack[15] = CrdAll->data[60]-CrdAll->data[51];
	Rstack[16] = CrdAll->data[61]-CrdAll->data[52];
	Rstack[17] = CrdAll->data[62]-CrdAll->data[53]; // �b20 foot �� �b17 ankle

	// �k�s�W�b��COM��m�� ���s�p��o��step�W�b��COM����m�V�q 
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

	// �p��b���󭫤ߪ���m�V�q
		// ���}support
		//Fixed Shank(�p�L) wXrcom
		MatScalarMul(CrdAll->data+12, 3, -1, rCOMccw);  //CrdAll->data+12 �b4 ankle
		MatAddAB(rCOMccw,pv_stack+3,rCOMccw,3);			//pv_stack345�����p�L���ߦ�m �Ankle��m��N���V�qr
		
		//Fixed Teigh wXrcom
		MatScalarMul(CrdAll->data+9, 3, -1, rCOMccw+3);	//CrdAll->data+9 �b3 knee
		MatAddAB(rCOMccw+3,pv_stack,rCOMccw+3,3);		//pv_stack012�����j�L���ߦ�m ����\��m��N���V�qr
		
		//Swing Waist wXrcom
		for (int i = 0 ; i < 3 ; i++)
			rCOMccw[i+6]=BodyRCOM[i]-CrdAll->data[6+i];		//�W�b��(�M�y��󵲦X)���ߦ�m��CrdAll->data6 ���} hip
		
		//Swing Teigh wXrcom
		MatScalarMul(CrdAll->data+45, 3, -1, rCOMccw+9);		//CrdAll->data+45 �b15�k�}hip
		MatAddAB(rCOMccw+9,pv_stack+9,rCOMccw+9,3);			//pv_stack91011���k�j�L���ߦ�m ��b���`��m��N���V�qr
		
		//Swing Shank wXrcom
		MatScalarMul(CrdAll->data+48, 3, -1, rCOMccw+12);	//CrdAll->data+48 �b16�k�}knee
		MatAddAB(rCOMccw+12,pv_stack+12,rCOMccw+12,3);		//pv_stack121314���k�p�L���ߦ�m ����\��m��N���V�qr
		
		//Swing Foot wXrcom
		MatScalarMul(CrdAll->data+51, 3, -1, rCOMccw+15);		//CrdAll->data+51 �b17�k�}ankle
		MatAddAB(rCOMccw+15,pv_stack+15,rCOMccw+15,3);		//pv_stack151617���k�}�O���ߦ�m ��}���m��N���V�qr

		// �k�}support
		
		//Fixed Shank wXrcom
		MatScalarMul(CrdAll->data+51, 3, -1, rCOMcw);	//CrdAll->data+51 �k�} ankle
		MatAddAB(rCOMcw,pv_stack+12,rCOMcw,3);			//pv_stack121314���k�p�L���ߦ�m �Ankle��m��N���V�qr
		
		//Fixed Teigh wXrcom
		MatScalarMul(CrdAll->data+48, 3, -1, rCOMcw+3);		//CrdAll->data+48 �k�} knee
		MatAddAB(rCOMcw+3,pv_stack+9,rCOMcw+3,3);			//pv_stack91011���k�j�L���ߦ�m �knee��m��N���V�qr
		
		//Swing Waist wXrcom
		for (int i = 0 ; i < 3 ; i++)		//�W�b��(�M�y��󵲦X)���ߦ�m��CrdAll->data454647 �k�} hip
			rCOMcw[i+6]=BodyRCOM[i]-CrdAll->data[i+DHJump+6];
		
		//Swing Teigh wXrcom
		MatScalarMul(CrdAll->data+6, 3, -1, rCOMcw+9);	//CrdAll->data+678 ���}hip
		MatAddAB(rCOMcw+9,pv_stack,rCOMcw+9,3);			//pv_stack012�����j�L���ߦ�m �hip��m��N���V�qr
		
		//Swing Shank wXrcom
		MatScalarMul(CrdAll->data+9, 3, -1, rCOMcw+12);		//CrdAll->data+91011 ���}knee
		MatAddAB(rCOMcw+12,pv_stack+3,rCOMcw+12,3);			//pv_stack345�����p�L���ߦ�m �knee��m��N���V�qr
		
		//Swing Foot wXrcom
		MatScalarMul(CrdAll->data+12, 3, -1, rCOMcw+15);		//CrdAll->data+12 ���}ankel
		MatAddAB(rCOMcw+15,pv_stack+6,rCOMcw+15,3);			//pv_stack678�����}�O���ߦ�m ��}���m��N���V�qr

		//// �H�U���ե�
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
	// ��U�b�����׶i��ƭȷL�� �o���t�׻P���[�t�� ��"�¶q" (����)
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
		////�H�U���ե�
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
	// �b���`�NIndex���ƧǤ覡
	// �b�@�I�����t�ת����ۥ[ �]���Ҭ��@�ɤ�
	// support�}�ҳy�������׶��[�t��
	// �b�o�̤���151617(���}support) 333435(�k�}support) �b�{�ꤤ���O���s�������`���Joint����Τp���
	// �Ъ`�N �a��N�O�a�� �����Q���s 
	// �S�O�`�N! 151617(���}support) �P333435(�k�}support)�Ҥ����a�� �O�s�������`��Ӷb���p�諬���
	// ���ծɭY�n�ݸ}���O�� ���}suppprt�Ь�333435 �a��Alpha��0 �k�}support�Ь�151617 �a��Alpha��0
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
				MatScalarMul(LocalOmegaJ+3*i, 3,-1.0, LocalOmegaJ+3*i);	// For Supporting Leg(���})
			}
		//Accumulate
				//MatScalarMul(LocalOmegaJ+15, 3, 0.0, OmegaL+15);	// Fixed Ankle Omega=0 
			for(int i = 0 ; i < 3 ; i++)	// OmegaL151617�����s�������`���Joint����Τp��� �]���a��LocalOmegaG = 0
				OmegaL[i+15] = LocalOmegaJ[i+15];
			for (int i = 4 ; i >= 0  ; i--)	// Left First Right Second  �ѥ���(left ankle)�V�W
				MatAddAB(LocalOmegaJ+3*i,OmegaL+3*(i+1),OmegaL+3*i,3);	// OmegaL121314 = LocalOmegaJ121314 + OmegaL151617 �]��OmegaL151617 = LocalOmegaJ151617
			for (int i = 0 ; i <6  ; i++)	// Left First Right Second  �ѥk�b(right hip)�V�U
			{
				if (i==0)
				MatAddAB(LocalOmegaJ+AxisJump,OmegaL,OmegaL+AxisJump,3);
				else
				MatAddAB(LocalOmegaJ+3*i+AxisJump,OmegaL+3*(i-1)+AxisJump,OmegaL+3*i+AxisJump,3);
			}
	}
	else if(selIK == 1)	// Left leg lifiting 
	{
		//ThetaD * Z   �p��LocalOmega
			for (int i = 0 ; i < 6 ; i++)	//Left First Right Second
			{
				MatScalarMul(ZAxisAll->data+3*i, 3, ThetaD+i, LocalOmegaJ+3*i);
				MatScalarMul(ZAxisAll->data+3*i+DHJump, 3, ThetaD+i+6, LocalOmegaJ+3*i+AxisJump);
				MatScalarMul(LocalOmegaJ+3*i+AxisJump, 3,-1.0, LocalOmegaJ+3*i+AxisJump);	//for support leg(�k�})
			}
		//Accumulate
				//MatScalarMul(LocalOmegaJ+15+AxisJump, 3, 0.0, OmegaL+15+AxisJump);	// Fixed Ankle Omega=0
			for(int i = 0 ; i < 3 ; i++)	// OmegaL333435�����s�������`���Joint����Τp��� �]���a��LocalOmegaG = 0
				OmegaL[i+15+AxisJump] = LocalOmegaJ[i+15+AxisJump];
			for (int i = 4 ; i >=0  ; i--)	//Left First Right Second  �ѥk��V�W
				MatAddAB(OmegaL+3*(i+1)+AxisJump,LocalOmegaJ+3*i+AxisJump,OmegaL+3*i+AxisJump,3);	// OmegaL303132 = LocalOmegaJ303132+LocalOmegaJ333435 �]���]��OmegaL333435 = LocalOmegaJ333435
			for (int i = 0 ; i <6  ; i++)	//Left First Right Second  �ѥ��b�V�U
			{
				if (i==0)
				MatAddAB(OmegaL+AxisJump,LocalOmegaJ,OmegaL,3);
				else
				MatAddAB(OmegaL+3*(i-1),LocalOmegaJ+3*i,OmegaL+3*i,3);
			}
	}
		//�H�U���ե�

		//fstream Fx;
		//Fx.open("TestLocalOmega.txt",ios::app);		
		//for(int i = 0 ; i < 12 ; i++)
		//	Fx << LocalOmegaJ[i*3] << "\t" << LocalOmegaJ[i*3+1] << "\t" << LocalOmegaJ[i*3+2] << "\t";
		//Fx << endl;
		//Fx.close();
	
		//fstream Fx;
		//Fx.open("TestOmegaFootL.txt",ios::app);		
		//if(selIK == 1)//(�k�}support)
		//Fx << OmegaL[15] << "\t"<< OmegaL[16] << "\t"<< OmegaL[17]<< endl;
		//else if(selIK == 0||selIK==2) //���}support
		//Fx << 0 << "\t"<< 0 << "\t"<< 0<< endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestOmegaFootR.txt",ios::app);		
		//if(selIK == 1)//(�k�}support)
		//Fx << 0 << "\t"<< 0 << "\t"<< 0<< endl;
		//else if(selIK == 0||selIK==2) //���}support
		//Fx << OmegaL[33] << "\t"<< OmegaL[34] << "\t"<< OmegaL[35]<< endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestOmegaShankL.txt",ios::app);		
		//if(selIK == 1)//(�k�}support)
		//Fx << OmegaL[9] << "\t"<< OmegaL[10] << "\t"<< OmegaL[11]<< endl;
		//else if(selIK == 0||selIK==2) //���}support
		//Fx << OmegaL[12] << "\t"<< OmegaL[13] << "\t"<< OmegaL[14]<< endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestOmegaShankR.txt",ios::app);		
		//if(selIK == 1)//(�k�}support)
		//Fx << OmegaL[30] << "\t"<< OmegaL[31] << "\t"<< OmegaL[32]<< endl;
		//else if(selIK == 0||selIK==2) //���}support
		//Fx << OmegaL[27] << "\t"<< OmegaL[28] << "\t"<< OmegaL[29]<< endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestOmegaWaist.txt",ios::app);		
		//if(selIK == 1)//(�k�}support)
		//Fx << OmegaL[18] << "\t"<< OmegaL[19] << "\t"<< OmegaL[20]<< endl;
		//else if(selIK == 0||selIK==2) //���}support
		//Fx << OmegaL[0] << "\t"<< OmegaL[1] << "\t"<< OmegaL[2]<< endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestOmegaThighL.txt",ios::app);		
		//if(selIK == 1)//(�k�}support)
		//Fx << OmegaL[6] << "\t"<< OmegaL[7] << "\t"<< OmegaL[8]<< endl;
		//else if(selIK == 0||selIK==2) //���}support
		//Fx << OmegaL[9] << "\t"<< OmegaL[10] << "\t"<< OmegaL[11]<< endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestOmegaThighR.txt",ios::app);		
		//if(selIK == 1)//(�k�}support)
		//Fx << OmegaL[27] << "\t"<< OmegaL[28] << "\t"<< OmegaL[29]<< endl;
		//else if(selIK == 0||selIK==2) //���}support
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
	// �ۤvJoint���t�צs�b�ۤv��VelJ ��Index�� 
	// �o�̪��t�ץH�@�ɮy�дy�z �ê������wsupport�}�}��b�t�׬�0
	// �@�I���b�������w�ۦP�t��
	// 2012 Slongz Start
	// 20121205 WeiZh Start
	******************************************************************/
	double TempPvStack[3];
	if (selIK == 0||selIK==2)// Double Support & Right leg lifiting  
	{
	//Vector Omega	cross r  �p��LocalVel
		//Fixed ankle wXr
		for (int i = 0 ; i < 3 ; i++)
			LocalVelJ[i+12]=0;	// �b���������wLocalVelJ121314 ���s�O�]���MJoint151617r=0 �ȴN�������w ���LLocalVelJ151617�O�]�����ݭn 151617�O�a

		//Fixed Knee wXr
		Cross2Vd(Rstack+3,OmegaL+12,LocalVelJ+9); //�p�Llink�����t�׬�Omega121314 ���\Joint��Index��91011

		//Fixed Hip wXr
		Cross2Vd(Rstack,OmegaL+9,LocalVelJ+6);  //�j�L�����t�׬�Omega91011 �ѤU���W�ƲĤ@���b���`Joint��Index��678
		for (int i = 0 ; i < 6 ; i++)
			LocalVelJ[i]=0; // �b���������wLocalVelJ012 345 ,012 345�b��Local�t�׬��s�O�]���MJoint678 r=0 �ȴN�������w

		//Swing Hip wXr 
		for (int i = 0 ; i < 3 ; i++)
			TempPvStack[i]=CrdAll->data[6+i+DHJump]-CrdAll->data[6+i];  //�Ⱖ�L�b���`�S���]�wRstack ������DH�Ȭ۴�
		Cross2Vd(OmegaL,TempPvStack,LocalVelJ+AxisJump);    //���t�ץ�678�}�l�֭p678+345+012�����t�׬�Omega012 �M����b���`���Z��cross �s�b�k�}�W���ƤU�ӲĤ@�Ӷb181920
		
		for (int i = 3 ; i < 9 ; i++)
			LocalVelJ[i+AxisJump]=0; // �b���������wLocalVelJ212223 242526 ,212223 242526�b��Local�t�׬��s�O�]���MJoint181920��r=0 �ȴN�������w

		//Swing Knee wXr
		Cross2Vd(OmegaL+6+AxisJump,Rstack+StackJump,LocalVelJ+9+AxisJump);

		//Swing ankle wXr
		Cross2Vd(OmegaL+9+AxisJump,Rstack+3+StackJump,LocalVelJ+12+AxisJump);		
		
		for (int i = 0 ; i < 3 ; i++)
			LocalVelJ[i+15+AxisJump]=0; // �b���������wLocalVelJ333435 = 0, �]��333435�P303132���X r = 0 ; �GLocal�b���t�� = 0 
	
	//Accumulate
		//MatScalarMul(LocalVelJ+15, 3, 0.0, VelJ+15);//r=0
		for (int i = 0 ; i < 3 ; i++)  // �b���������wVelJ151617 ��Joint�t�׵���� �]��151617���a 
			VelJ[i+15] = 0;  		
		for (int i = 4 ; i >= 0 ; i--)		//���}�U���W
			MatAddAB(VelJ+3*(i+1),LocalVelJ+3*i,VelJ+3*i,3);
		
		//�k�}�W���U
		MatAddAB(VelJ,LocalVelJ+AxisJump,VelJ+AxisJump,3);
		for (int i = 1 ; i < 6 ; i++)		
			MatAddAB(VelJ+3*(i-1)+AxisJump,LocalVelJ+3*i+AxisJump,VelJ+3*i+AxisJump,3);
	}
	else if(selIK == 1)// Left leg lifiting 
	{
	//Vector Omega	cross r  �p��LocalVel
		//Fixed ankle wXr
		for (int i = 0 ; i < 3 ; i++)   // �b���������wLocalVelJ303132 ���s�O�]���MJoint333435r=0 �ȴN�������w ���LLocalVelJ333435�O�]�����ݭn 333435�O�a
			LocalVelJ[i+AxisJump+12]=0; //r=0

		//Fixed Knee wXr
		Cross2Vd(Rstack+StackJump+3, OmegaL+AxisJump+12,LocalVelJ+AxisJump+9); //  Joint272829���t�ץ�OmegaLink303132 �M-Rstack121314 cross �s�bLocalVelJoint272829

		//Fixed Hip wXr
		Cross2Vd(Rstack+StackJump,OmegaL+AxisJump+9,LocalVelJ+AxisJump+6);  //�j�L�����t�׬�Omega272829 �ѤU���W�ƲĤ@���b���`Joint��Index��242526
		for (int i = 0 ; i < 6 ; i++)				// �b���������wLocalVelJ212223 181920 ,212223 181920�b��Local�t�׬��s�O�]���MJoint242526 r=0 �ȴN�������w
			LocalVelJ[i+AxisJump]=0; // r=0

		//Swing Hip wXr 
		for (int i=0;i<3; i++)
			TempPvStack[i]=CrdAll->data[6+i]-CrdAll->data[6+i+DHJump];  //�Ⱖ�L�b���`�S���]�wRstack ������DH�Ȭ۴�
		Cross2Vd(OmegaL+AxisJump,TempPvStack,LocalVelJ);		//���t�ץ�242526�}�l�֭p242526+212223+181920�����t�׬�Omega181920 �M����b���`���Z��cross �s�b���}�W���ƤU�ӲĤ@�Ӷb012
		for (int i = 0 ; i < 6 ; i++)		// �b���������wLocalVelJ345 678 ,345 678�b��Local�t�׬��s�O�]���MJoint012��r=0 �ȴN�������w
			LocalVelJ[i+3]=0; // r=0

		//Swing Knee wXr
		Cross2Vd(OmegaL+6,Rstack,LocalVelJ+9);

		//Swing ankle wXr
		Cross2Vd(OmegaL+9,Rstack+3,LocalVelJ+12);		
		
		for (int i = 0 ; i < 3 ; i++)		// �b���������wLocalVelJ151617 = 0, �]��151617�P121314���X r = 0 ; �G151617Local�b���t�� = 0 
			LocalVelJ[i+15]=0; //r=0
	
	//Accumulate
		//MatScalarMul(LocalVelJ+15+AxisJump, 3, 0.00, VelJ+15+AxisJump);//r=0
		for (int i = 0 ; i < 3 ; i++)   // �b���������wVelJ333435 ��Joint�t�׵���� �]��333435���a 
			VelJ[i+AxisJump+15] = 0;  				
		for (int i = 4 ; i >= 0; i--)	// �k�}�U���W
			MatAddAB(VelJ+3*(i+1)+AxisJump,LocalVelJ+3*i+AxisJump,VelJ+3*i+AxisJump,3);
		
		//���}�W���U
		MatAddAB(VelJ+AxisJump,LocalVelJ,VelJ,3);
		for (int i = 1 ; i < 6 ; i++)	//���}�W���U
			MatAddAB(VelJ+3*(i-1),LocalVelJ+3*i,VelJ+3*i,3);
	}
		
	// �H�U���ե�	
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
	// �b�o�̤���AlphaL151617 �b�{�ꤤ���O���s�������`���Joint����Τp���
	// �Ъ`�N �a��N�O�a�� �����Q���s 
	// �S�O�`�N! 151617(���}support) �P333435(�k�}support)�Ҥ����a�� �O�s�������`��Ӷb���p�諬���
	// ���ծɭY�n�ݸ}���O�� ���}suppprt�Ь�333435 �a��Alpha��0 �k�}support�Ь�151617 �a��Alpha��0
	// 2012 Slongz Start
	// 20121205 WeiZh Start
	******************************************************************/
	if (selIK == 0||selIK==2)// Double Support & Right leg lifiting 
	{
	//Find Joint Alpha      Checked
		//OmegaL cross Local OmegaJ        ���⥪�}(�U���W)�A��k�}(�W���U)   VectorTemp1
			for (int i = 0 ; i < 3 ; i++)  //�b�����p��VectorTemp1 151617 �]��VectorTemp1 151617 = 0(BASE) �����M���ȥH�Q�᭱�֥[
				VectorTemp1[i+15] = 0;
			for (int i = 4 ; i >=0 ; i--) 
				Cross2Vd(OmegaL+3*(i+1), LocalOmegaJ+3*i,VectorTemp1+3*i);
			Cross2Vd(OmegaL, LocalOmegaJ+AxisJump,VectorTemp1+AxisJump);
			for (int i = 1 ; i < 6 ; i++)
				Cross2Vd(OmegaL+3*(i-1)+AxisJump, LocalOmegaJ+3*i+AxisJump,VectorTemp1+3*i+AxisJump);

		//Theta dot dot * z�b
			for (int i = 0 ; i < 6 ; i++)
			{
				MatScalarMul(ZAxisAll->data+3*i, 3, ThetaDD+i, VectorTemp2+3*i);
				MatScalarMul(ZAxisAll->data+3*i+DHJump, 3, ThetaDD+i+6, VectorTemp2+3*i+AxisJump);
				MatScalarMul(VectorTemp2+3*i, 3,-1.0, VectorTemp2+3*i);// for support leg
			}
		//Accumulate			
			for (int i = 0 ; i < 12 ; i++)  //�����֥[012~333435��VectorTemp1�MVectorTemp2 �U���N���β֥[�F(�b��������)
				MatAddAB(VectorTemp1+3*i,VectorTemp2+3*i,AlphaL+3*i,3);
			
			for (int i = 4 ; i >= 0 ; i--) //���}(support�})		�b�����p��AlphaL151617����]�O�b�W���֥[Temp1�MTemp2��Alpha151617�N�w�g�p�⧹�F
					MatAddAB(AlphaL+3*(i+1),AlphaL+3*i,AlphaL+3*i,3);
			
			//�k�}
			MatAddAB(AlphaL,AlphaL+AxisJump,AlphaL+AxisJump,3);
			for (int i = 1 ; i < 6 ; i++) 
				MatAddAB(AlphaL+3*(i-1)+AxisJump,AlphaL+3*i+AxisJump,AlphaL+3*i+AxisJump,3);
	}
	else if(selIK == 1)// Left leg lifiting 
	{
	//Find Joint Alpha      Checked
		//OmegaL cross Local OmegaJ    (VectorTemp1)
			for (int i = 0 ; i < 3 ; i++)  //�b�����p��VectorTemp1 333435 �]��VectorTemp1 333435 = 0(BASE) �����M���ȥH�Q�᭱�֥[
				VectorTemp1[i+AxisJump+15] = 0;
			for (int i = 4 ; i >= 0 ; i--)  
				Cross2Vd(OmegaL+3*(i+1)+AxisJump, LocalOmegaJ+3*i+AxisJump,VectorTemp1+3*i+AxisJump);
			Cross2Vd(OmegaL+AxisJump, LocalOmegaJ,VectorTemp1);
			for (int i = 1 ; i < 6 ; i++)
				Cross2Vd(OmegaL+3*(i-1), LocalOmegaJ+3*i,VectorTemp1+3*i);

		//Theta dot dot * z�b
			for (int i = 0 ; i < 6 ; i++)
			{
				MatScalarMul(ZAxisAll->data+3*i, 3, ThetaDD+i, VectorTemp2+3*i);
				MatScalarMul(ZAxisAll->data+3*i+DHJump, 3, ThetaDD+i+6, VectorTemp2+3*i+AxisJump);
				MatScalarMul(VectorTemp2+3*i+AxisJump, 3,-1.0, VectorTemp2+3*i+AxisJump);// for support leg
			}
		//Accumulate
			for (int i = 0 ; i < 12 ; i++)  //�����֥[012~333435��VectorTemp1�MVectorTemp2 �U���N���β֥[�F(�b��������)
				MatAddAB(VectorTemp1+3*i,VectorTemp2+3*i,AlphaL+3*i,3);
			
			for (int i = 4 ; i >= 0 ; i--)  //�b�����p��AlphaL333435����]�O�b�W���֥[Temp1�MTemp2��Alpha333435�N�w�g�p�⧹�F
					MatAddAB(AlphaL+3*(i+1)+AxisJump,AlphaL+3*i+AxisJump,AlphaL+3*i+AxisJump,3);
			
			MatAddAB(AlphaL+AxisJump,AlphaL,AlphaL,3);
			for (int i = 1 ; i < 6 ; i++)
					MatAddAB(AlphaL+3*(i-1),AlphaL+3*i,AlphaL+3*i,3);
	}

		////�H�U���ե�

		//fstream Fx;
		////Fx.open("TestLocalOmega.txt",ios::app);		
		////for(int i = 0 ; i < 12 ; i++)
		////	Fx << LocalOmegaJ[i*3] << "\t" << LocalOmegaJ[i*3+1] << "\t" << LocalOmegaJ[i*3+2] << "\t";
		////Fx << endl;
		////Fx.close();
	
		////fstream Fx;
		//Fx.open("TestAlphaLFootL.txt",ios::app);		
		//if(selIK == 1)//(�k�}support)
		//Fx << AlphaL[15] << "\t"<< AlphaL[15] << "\t"<< AlphaL[15]<< endl;
		//else if(selIK == 0||selIK==2) //���}support
		//Fx << 0 << "\t"<< 0 << "\t"<< 0 << endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestAlphaLFootR.txt",ios::app);		
		//if(selIK == 1)//(�k�}support)
		//Fx << 0 << "\t"<< 0 << "\t"<< 0 << endl;
		//else if(selIK == 0||selIK==2) //���}support
		//Fx << AlphaL[33] << "\t"<< AlphaL[34] << "\t"<< AlphaL[35] << endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestAlphaLShankL.txt",ios::app);		
		//if(selIK == 1)//(�k�}support)
		//Fx << AlphaL[9] << "\t"<< AlphaL[10] << "\t"<< AlphaL[11]<< endl;
		//else if(selIK == 0||selIK==2) //���}support
		//Fx << AlphaL[12] << "\t"<< AlphaL[13] << "\t"<< AlphaL[14]<< endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestAlphaLShankR.txt",ios::app);		
		//if(selIK == 1)//(�k�}support)
		//Fx << AlphaL[30] << "\t"<< AlphaL[31] << "\t"<< AlphaL[32]<< endl;
		//else if(selIK == 0||selIK==2) //���}support
		//Fx << AlphaL[27] << "\t"<< AlphaL[28] << "\t"<< AlphaL[29]<< endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestAlphaLWaist.txt",ios::app);		
		//if(selIK == 1)//(�k�}support)
		//Fx << AlphaL[18] << "\t"<< AlphaL[19] << "\t"<< AlphaL[20]<< endl;
		//else if(selIK == 0||selIK==2) //���}support
		//Fx << AlphaL[0] << "\t"<< AlphaL[1] << "\t"<< AlphaL[2]<< endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestAlphaLThighL.txt",ios::app);		
		//if(selIK == 1)//(�k�}support)
		//Fx << AlphaL[6] << "\t"<< AlphaL[7] << "\t"<< AlphaL[8]<< endl;
		//else if(selIK == 0||selIK==2) //���}support
		//Fx << AlphaL[9] << "\t"<< AlphaL[10] << "\t"<< AlphaL[11]<< endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestAlphaLThighR.txt",ios::app);		
		//if(selIK == 1)//(�k�}support)
		//Fx << AlphaL[27] << "\t"<< AlphaL[28] << "\t"<< AlphaL[29]<< endl;
		//else if(selIK == 0||selIK==2) //���}support
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
	// �֥[�Ҧ���VectorTemp1 VectorTemp2 �s��ۤv��Index
	// AccelJ  = OmegaJ X LocalVelJ (VectorTemp1) + AlphaJ X r (VectorTemp2)
	// 2012 Slongz Start
	// 20121205 WeiZh Start
	******************************************************************/
	double TempPvStack[3];
	if (selIK == 0||selIK==2)// Double Support & Right leg lifiting 
	{
	//OmegaJ cross LocalVelJ  VectorTemp1
		for (int i = 0 ; i < 3 ; i++)	//�b�����w151617��VectorTemp1 = 0 �]���O�a ���O���F�U���֥[VectorTemp1 VectorTemp2 ��K
			VectorTemp1[i+15] = 0;	//r=0
		//���}
		for (int i = 4 ; i >= 0 ; i--)	
			Cross2Vd(OmegaL+3*(i+1), LocalVelJ+3*i, VectorTemp1+3*i);
		
		//�k�}	
			Cross2Vd(OmegaL, LocalVelJ+AxisJump, VectorTemp1+AxisJump);
		for (int i = 1 ; i < 6 ; i++)	
			Cross2Vd(OmegaL+3*(i-1)+AxisJump, LocalVelJ+3*i+AxisJump, VectorTemp1+3*i+AxisJump);
				
	//AlphaJ cross r		VectorTemp2
		//Fixed ankle AXr
		for (int i = 0 ; i < 6 ; i++)	//�b�����w151617��VectorTemp2 = 0 �]���O�a ���O���F�U���֥[VectorTemp1 VectorTemp2 ��K
			VectorTemp2[i+12]=0;	//Fixed on ground

		//Fixed Knee AXr
		Cross2Vd(Rstack+3, AlphaL+12, VectorTemp2+9);

		//Fixed Hip AXr
		Cross2Vd(Rstack, AlphaL+9, VectorTemp2+6);
		for (int i = 0 ; i < 6 ; i++)	//�������wVectorTemp2 012 345 ���� r��0 �������w
			VectorTemp2[i]=0;	// r=0

		//Swing Hip AXr 
		for (int i = 0 ; i < 3 ; i++)		
			TempPvStack[i]=CrdAll->data[6+i+DHJump]-CrdAll->data[6+i];
		Cross2Vd(AlphaL,TempPvStack,VectorTemp2+AxisJump);	//�p��k�}�Ĥ@�b��AlphaJ cross r �]���S����Rstack ������DH�۴�454647-678
		for (int i = 0 ; i < 6 ; i++)	//�������w212223 242526��AlphaJ cross r = 0 �]��r = 0
			VectorTemp2[i+3+AxisJump]=0;	// r=0

		//Swing Knee AXr
		Cross2Vd(AlphaL+6+AxisJump,Rstack+StackJump,VectorTemp2+9+AxisJump);

		//Swing ankle AXr
		Cross2Vd(AlphaL+9+AxisJump,Rstack+3+StackJump,VectorTemp2+12+AxisJump);		
		for (int i = 0 ; i < 3 ; i++)	//�������wVectorTemp2 333435 = 0 �]���M303132��r=0  
			VectorTemp2[i+15+AxisJump]=0;	//r=0
	
	//Accumulate
		for (int i = 0 ; i < 12 ; i++)	//�֥[�Ҧ���VectorTemp1 VectorTemp2 �s��ۤv��Index
			MatAddAB(VectorTemp1+3*i,VectorTemp2+3*i,AccelJ+3*i,3);
		// ���}
		for (int i = 0 ; i < 3 ; i++)	// �������w�}��i�a�O�Ĥ@��Joint��0
			AccelJ[i+15] = 0;
		for (int i = 4 ; i >= 0 ; i--)
			MatAddAB(AccelJ+3*(i+1),AccelJ+3*i,AccelJ+3*i,3);
		
		// �k�}
		MatAddAB(AccelJ,AccelJ+AxisJump,AccelJ+AxisJump,3);
		for (int i = 1 ; i < 6 ; i++)
				MatAddAB(AccelJ+3*(i-1)+AxisJump,AccelJ+3*i+AxisJump,AccelJ+3*i+AxisJump,3);
	}
	else if(selIK == 1)// Left leg lifiting 
	{
	//OmegaJ cross LocalVelJ VectorTemp1
		for (int i = 0 ; i < 3 ; i++)	//�b�����w333435��VectorTemp1 = 0 �]���O�a ���O���F�U���֥[VectorTemp1 VectorTemp2 ��K
			VectorTemp1[i+AxisJump+15]=0;	//r=0
		for (int i = 4 ; i >= 0 ; i--)
			Cross2Vd(OmegaL+3*(i+1)+AxisJump, LocalVelJ+3*i+AxisJump, VectorTemp1+3*i+AxisJump);
		
		//���}
		Cross2Vd(OmegaL+AxisJump, LocalVelJ, VectorTemp1);	
		for (int i = 1 ; i < 6 ; i++)	
			Cross2Vd(OmegaL+3*(i-1), LocalVelJ+3*i, VectorTemp1+3*i);

	//AlphaJ cross r
		//Fixed ankle AXr
		for (int i = 0 ; i < 6 ; i++)	//�b�����w333435��VectorTemp2 = 0 �]���O�a ���O���F�U���֥[VectorTemp1 VectorTemp2 ��K
			VectorTemp2[i+AxisJump+12]=0;	//Fixed on ground

		//Fixed Knee AXr
		Cross2Vd(Rstack+12, AlphaL+AxisJump+12, VectorTemp2+AxisJump+9);

		//Fixed Hip AXr
		Cross2Vd(Rstack+9, AlphaL+AxisJump+9, VectorTemp2+AxisJump+6);
		for (int i = 0 ; i < 6 ; i++)	//�������wVectorTemp2 212223 181920 ���� r��0 �������w
			VectorTemp2[i+AxisJump]=0;	// r=0

		//Swing Hip AXr 
		for (int i = 0 ; i < 3 ; i++)
			TempPvStack[i]=CrdAll->data[6+i]-CrdAll->data[6+i+DHJump];
		Cross2Vd(AlphaL+AxisJump,TempPvStack,VectorTemp2);	//�p��k�}�Ĥ@�b��AlphaJ cross r �]���S����Rstack ������DH�۴�678-454647
		for (int i = 0 ; i < 6 ; i++)	//�������w345 678��AlphaJ cross r = 0 �]��r = 0
			VectorTemp2[i+3]=0;	// r=0

		//Swing Knee AXr
		Cross2Vd(AlphaL+6,Rstack+0,VectorTemp2+9);

		//Swing ankle AXr
		Cross2Vd(AlphaL+9,Rstack+3,VectorTemp2+12);		
		for (int i = 0 ; i < 3 ; i++)	//�������wVectorTemp2 151617 = 0 �]���M121314��r=0  
			VectorTemp2[i+15]=0; //r=0
	
	//Accumulate	
		for (int i = 0 ; i < 12 ; i++)	//�֥[�Ҧ���VectorTemp1 VectorTemp2 �s��ۤv��Index
			MatAddAB(VectorTemp1+3*i,VectorTemp2+3*i,AccelJ+3*i,3);
		// �k�}
		for (int i = 0 ; i < 3 ; i++)	// �������w�}��i�a�O�Ĥ@��Joint��0
			AccelJ[i+15+AxisJump] = 0;

		for (int i = 4 ; i >= 0 ; i--)
			MatAddAB(AccelJ+3*(i+1)+AxisJump,AccelJ+3*i+AxisJump,AccelJ+3*i+AxisJump,3);
		
		// ���}
		MatAddAB(AccelJ+AxisJump,AccelJ,AccelJ,3);
		for (int i = 1 ; i < 6 ; i++)
			MatAddAB(AccelJ+3*(i-1),AccelJ+3*i,AccelJ+3*i,3);
	}

		// �H�U���ե�
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
	// Find Link COM Vel v=v+ w X r r:Rstack  �@�k��Index�t�m�ҥ�FDVelJ 
	// �Ъ`�Nrcom���p��覡!!!! �H�έӧOComputeVelCOM��Index�s���m 
	// �]��ComputeVelCOM����m�b���W �ҥH�s��覡�MOmega�BAlpha�ۦP
	// ���F�t�X�U���p��O�x�ɻݥΥΨ쭫�߹����IJoint���V�q �b����n�Q�Υ��k�}support���P�ɹﭫ�ߪ��V�q���V��V���P �@�æs��
	// �ѤU���`���V���߬� rCOMccw (�f����) �ѤW���`���V���߬� rCOMcw (������) �U��3*6 = 18��Index
	// ���ѩ�Torque�p��ݦP�ɨϥΨ�ccw �M cw���V�q �G����CalCoord�p��
	// 2012 Slongz Start
	// 20121206 WeiZh Start 20121217 ��COM�V�q
	******************************************************************/
	if (selIK == 0||selIK==2)// Double Support  & Right leg lifiting 
	{
	//Vector Omega cross rcom
		//Fixed Foot wXrcom
		for (int i = 0 ; i < 3 ; i++)	//�b�o�̫��wLocalVelCOM151617�O���F�����|�[��K
			LocalVelCOM[i+15]=0; //r=0

		//Fixed Shank(�p�L) wXrcom
		//MatScalarMul(CrdAll->data+12, 3, -1, rCOMccw);  //CrdAll->data+12 �b4 ankle
		//MatAddAB(rCOMccw,pv_stack+3,rCOMccw,3);			//pv_stack345�����p�L���ߦ�m �Ankle��m��N���V�qr
		Cross2Vd(OmegaL+12,rCOMccw,LocalVelCOM+12);		//�p�LLink��Omega��121314 �⭫�߳t�צs�b121314

		//Fixed Teigh wXrcom
		//MatScalarMul(CrdAll->data+9, 3, -1, rCOMccw+3);	//CrdAll->data+9 �b3 knee
		//MatAddAB(rCOMccw+3,pv_stack,rCOMccw+3,3);		//pv_stack012�����j�L���ߦ�m ����\��m��N���V�qr
		Cross2Vd(OmegaL+9,rCOMccw+3,LocalVelCOM+9);		//�j�LLink��Omega��91011 �⭫�߳t�צs�b91011
		for (int i = 0 ; i < 6 ; i++) // LocalVelCOM345 678=0  �]���M012�@�I �ҥHLocak���߳t�ת������w��0
			LocalVelCOM[i+3]=0;	// r=0

		//Swing Waist wXrcom
		//for (int i = 0 ; i < 3 ; i++)
		//	rCOMccw[i+6]=BodyRCOM[i]-CrdAll->data[6+i];		//�W�b��(�M�y��󵲦X)���ߦ�m��CrdAll->data6 ���} hip
		Cross2Vd(OmegaL,rCOMccw+6,LocalVelCOM);
		for (int i = 0 ; i < 6 ; i++)		//LocalVelCOM181920 212223=0 �]���M242526�@�I �ҥHLocak���߳t�׬�0
			LocalVelCOM[i+AxisJump]=0; // r=0

		//Swing Teigh wXrcom
		//MatScalarMul(CrdAll->data+45, 3, -1, rCOMccw+9);		//CrdAll->data+45 �b15�k�}hip
		//MatAddAB(rCOMccw+9,pv_stack+9,rCOMccw+9,3);			//pv_stack91011���k�j�L���ߦ�m ��b���`��m��N���V�qr
		Cross2Vd(OmegaL+6+AxisJump,rCOMccw+9,LocalVelCOM+6+AxisJump);

		//Swing Shank wXrcom
		//MatScalarMul(CrdAll->data+48, 3, -1, rCOMccw+12);	//CrdAll->data+48 �b16�k�}knee
		//MatAddAB(rCOMccw+12,pv_stack+12,rCOMccw+12,3);		//pv_stack121314���k�p�L���ߦ�m ����\��m��N���V�qr
		Cross2Vd(OmegaL+9+AxisJump,rCOMccw+12,LocalVelCOM+9+AxisJump);	

		//Swing Foot wXrcom
		for (int i = 0 ; i < 3 ; i++)							// �������wLocalVelCOM303132���Ȭ�0 �]���M333435�@�I
			LocalVelCOM[i+12+AxisJump]=0; // r=0

		//MatScalarMul(CrdAll->data+51, 3, -1, rCOMccw+15);		//CrdAll->data+51 �b17�k�}ankle
		//MatAddAB(rCOMccw+15,pv_stack+15,rCOMccw+15,3);		//pv_stack151617���k�}�O���ߦ�m ��}���m��N���V�qr
		Cross2Vd(OmegaL+AxisJump+15,rCOMccw+15,LocalVelCOM+AxisJump+15);	

	//Accumulate
		for (int i = 0 ; i < 12 ; i++)
			MatAddAB(VelJ+3*i,LocalVelCOM+3*i,VelCOM+3*i,3);
	}
	else if(selIK == 1)// Left leg lifiting 
	{
	//Vector Omega cross rcom
		//Fixed Foot wXrcom
		for (int i = 0 ; i < 3 ; i++)		//�b�o�̫��wLocalVelCOM333435�O���F�����|�[��K
			LocalVelCOM[i+AxisJump+15]=0; //r=0

		//Fixed Shank wXrcom
		//MatScalarMul(CrdAll->data+51, 3, -1, rCOMcw);	//CrdAll->data+51 �k�} ankle
		//MatAddAB(rCOMcw,pv_stack+12,rCOMcw,3);			//pv_stack121314���k�p�L���ߦ�m �Ankle��m��N���V�qr
		Cross2Vd(OmegaL+AxisJump+12,rCOMcw,LocalVelCOM+AxisJump+12); //�p�LLink��Omega��303132 �⭫�߳t�צs�b303132

		//Fixed Teigh wXrcom
		//MatScalarMul(CrdAll->data+48, 3, -1, rCOMcw+3);		//CrdAll->data+48 �k�} knee
		//MatAddAB(rCOMcw+3,pv_stack+9,rCOMcw+3,3);			//pv_stack91011���k�p�L���ߦ�m �knee��m��N���V�qr
		Cross2Vd(OmegaL+AxisJump+9,rCOMcw+3,LocalVelCOM+AxisJump+9);	//�j�LLink��Omega��272829 �⭫�߳t�צs�b272829
		for (int i = 0 ; i < 6 ; i++)		 // LocalVelCOM242526 212223=0  �]���M181920�@�I �ҥHLocak���߳t�ת������w��0
			LocalVelCOM[i+AxisJump+3]=0; // r=0

		//Swing Waist wXrcom
		//for (int i = 0 ; i < 3 ; i++)		//�W�b��(�M�y��󵲦X)���ߦ�m��CrdAll->data454647 �k�} hip
		//	rCOMcw[i+6]=BodyRCOM[i]-CrdAll->data[i+DHJump+6];
		Cross2Vd(OmegaL+AxisJump,rCOMcw+6,LocalVelCOM+AxisJump);
		for (int i = 0 ; i < 6 ; i++)		//LocalVelCOM012 345=0 �]���M678�@�I �ҥHLocak���߳t�׬�0
			LocalVelCOM[i]=0; // r=0

		//Swing Teigh wXrcom
		//MatScalarMul(CrdAll->data+6, 3, -1, rCOMcw+9);	//CrdAll->data+678 ���}hip
		//MatAddAB(rCOMcw+9,pv_stack,rCOMcw+9,3);			//pv_stack012�����j�L���ߦ�m �hip��m��N���V�qr
		Cross2Vd(OmegaL+6,rCOMcw+9,LocalVelCOM+6);		//�j�LLink��Omega��678 �⭫�߳t�צs�b678

		//Swing Shank wXrcom
		//MatScalarMul(CrdAll->data+9, 3, -1, rCOMcw+12);		//CrdAll->data+91011 ���}knee
		//MatAddAB(rCOMcw+12,pv_stack+3,rCOMcw+12,3);			//pv_stack345�����p�L���ߦ�m �knee��m��N���V�qr
		Cross2Vd(OmegaL+9,rCOMcw+12,LocalVelCOM+9);			//�p�LLink��Omega��91011 �⭫�߳t�צs�b91011

		//Swing Foot wXrcom
		for (int i = 0 ; i < 3 ; i++)		// �������wLocalVelCOM121314���Ȭ�0 �]���M151617�@�I
			LocalVelCOM[i+12]=0; // r=0

		//MatScalarMul(CrdAll->data+12, 3, -1, rCOMcw+15);		//CrdAll->data+12 ���}ankel
		//MatAddAB(rCOMcw+15,pv_stack+6,rCOMcw+15,3);			//pv_stack678�����}�O���ߦ�m ��}���m��N���V�qr
		Cross2Vd(OmegaL+15,rCOMcw+15,LocalVelCOM+15);	
	//Accumulate
		for (int i = 0 ; i < 12 ; i++)
			MatAddAB(VelJ+3*i,LocalVelCOM+3*i,VelCOM+3*i,3);
	}

	// �H�U���ե�

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
		//if(selIK == 1)//(�k�}support)
		//Fx << VelCOM[15] << "\t"<< VelCOM[16] << "\t"<< VelCOM[17]<< endl;
		//else if(selIK == 0||selIK==2) //���}support
		//Fx << 0 << "\t"<< 0 << "\t"<< 0 << endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestVelCOMFootR.txt",ios::app);		
		//if(selIK == 1)//(�k�}support)
		//Fx << 0 << "\t"<< 0 << "\t"<< 0 << endl;
		//else if(selIK == 0||selIK==2) //���}support
		//Fx << VelCOM[33] << "\t"<< VelCOM[34] << "\t"<< VelCOM[35] << endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestVelCOMShankL.txt",ios::app);		
		//if(selIK == 1)//(�k�}support)
		//Fx << VelCOM[9] << "\t"<< VelCOM[10] << "\t"<< VelCOM[11]<< endl;
		//else if(selIK == 0||selIK==2) //���}support
		//Fx << VelCOM[12] << "\t"<< VelCOM[13] << "\t"<< VelCOM[14]<< endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestVelCOMShankR.txt",ios::app);		
		//if(selIK == 1)//(�k�}support)
		//Fx << VelCOM[30] << "\t"<< VelCOM[31] << "\t"<< VelCOM[32]<< endl;
		//else if(selIK == 0||selIK==2) //���}support
		//Fx << VelCOM[27] << "\t"<< VelCOM[28] << "\t"<< VelCOM[29]<< endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestVelCOMWaist.txt",ios::app);		
		//if(selIK == 1)//(�k�}support)
		//Fx << VelCOM[18] << "\t"<< VelCOM[19] << "\t"<< VelCOM[20]<< endl;
		//else if(selIK == 0||selIK==2) //���}support
		//Fx << VelCOM[0] << "\t"<< VelCOM[1] << "\t"<< VelCOM[2]<< endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestVelCOMThighL.txt",ios::app);		
		//if(selIK == 1)//(�k�}support)
		//Fx << VelCOM[6] << "\t"<< VelCOM[7] << "\t"<< VelCOM[8]<< endl;
		//else if(selIK == 0||selIK==2) //���}support
		//Fx << VelCOM[9] << "\t"<< VelCOM[10] << "\t"<< VelCOM[11]<< endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestVelCOMThighR.txt",ios::app);		
		//if(selIK == 1)//(�k�}support)
		//Fx << VelCOM[27] << "\t"<< VelCOM[28] << "\t"<< VelCOM[29]<< endl;
		//else if(selIK == 0||selIK==2) //���}support
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
	// �Ъ`�NIndex�ƦC�覡
	// 2012 Slongz Start
	// 20121206 WeiZh Start 20121217 ��COM��m�V�q rCOMccw rCOMcw
	******************************************************************/
	if (selIK==2||selIK == 0)// Double Support & Right leg lifiting
	{
	//OmegaL cross LocalVelCOM VectorTemp1 �]���b�e�����w�g���Ӧۤv��Index�ƦC�n�F �ۤv��Omega�M�ۤv��LocalVelCOM cross �s��ۤv��Index�� �ҥH���k�}�i�H�@�_
		//Fixed Leg �M Swing Leg
		  for (int i = 0 ; i < 11 ; i++ )
			 Cross2Vd(OmegaL+i*3, LocalVelCOM+3*i, VectorTemp1+3*i);
	//AlphaJ cross r
		//Fixed Foot AXr
		for (int i = 0 ; i < 3 ; i++)		//�b�o�̫��wVectorTemp2 151617�O���F�����|�[��K
			VectorTemp2[i+15] = 0;			//Fixed on ground

		//Fixed Shank AXr
		Cross2Vd(AlphaL+12,rCOMccw, VectorTemp2+12);	//�p�LLink��Alpha��121314 �⭫�ߤ��u�[�t�צs�b121314

		//Fixed Teight AXr
		Cross2Vd(AlphaL+9 ,rCOMccw+3,VectorTemp2+9);		//�j�LLink��Alpha��91011 �⭫�ߥ[�t�צs�b91011
		for (int i = 0 ; i < 6 ; i++)		//VectorTemp2 345 678=0  �]���M012�@�I �ҥHLocak���ߤ��u�[�t�ת������w��0
			VectorTemp2[i+3] = 0; // r=0

		//Swing Body(Waist) AXr 
		Cross2Vd(AlphaL,rCOMccw+6,VectorTemp2);
		for (int i = 0 ; i < 6 ; i++)		//VectorTemp2 181920 212223=0 �]���M242526�@�I �ҥHLocak���ߤ��u�[�t�׬�0
			VectorTemp2[i+AxisJump]=0; // r=0

		//Swing Teight AXr
		Cross2Vd(AlphaL+6+AxisJump,rCOMccw+9,VectorTemp2+6+AxisJump);

		//Swing Shank AXr
		Cross2Vd(AlphaL+9+AxisJump,rCOMccw+12,VectorTemp2+9+AxisJump);	

		//Swing Foot AXr
		for (int i = 0 ; i < 3 ; i++)							// �������wVectorTemp2 303132���Ȭ�0 �]���M333435�@�I
			VectorTemp2[i+12+AxisJump]=0; // r=0
		Cross2Vd(AlphaL+15+AxisJump,rCOMccw+15,VectorTemp2+15+AxisJump);	

	//Accumalate �]��Index�t�X �i�H�����@�_�[
		for (int i = 0 ; i < 12 ; i++)
		{
			MatAddAB(VectorTemp1+3*i,VectorTemp2+3*i,AccelCOM+3*i,3);
			MatAddAB(AccelJ+3*i,AccelCOM+3*i,AccelCOM+3*i,3);
		}
	}
	else if(selIK == 1)// Left leg lifiting 
	{
	//OmegaL cross LocalVelCOM VectorTemp1 �]���b�e�����w�g���Ӧۤv��Indexw�ƦC�n�F �ۤv��Omega�M�ۤv��LocalVelCOM cross �s��ۤv��Index�� �ҥH���k�}�i�H�@�_
		//Fixed Leg �M Swing Leg
		  for (int i = 0 ; i < 11 ; i++ )
			 Cross2Vd(OmegaL+i*3, LocalVelCOM+3*i, VectorTemp1+3*i);

	//AlphaJ cross r
		//Fixed Foot AXr
		for (int i = 0 ; i < 3 ; i++)	  //�b�o�̫��wVectorTemp2 333435�O���F�����|�[��K
			VectorTemp2[i+AxisJump+15]=0; //Fixed on ground

		//Fixed Shank AXr
		Cross2Vd(AlphaL+AxisJump+12,rCOMcw, VectorTemp2+AxisJump+12);	//�k�p�LLink��Alpha��303132 �⭫�ߤ��u�[�t�צs�b303132

		//Fixed Teight AXr
		Cross2Vd(AlphaL+AxisJump+9 ,rCOMcw+3,VectorTemp2+AxisJump+9);	//�k�j�LLink��Alpha��272829 �⭫�ߤ��u�[�t�צs�b272829
		for (int i = 0 ; i < 6 ; i++)	//VectorTemp2 212223 242526=0  �]���M181920�@�I �ҥHLocak���ߤ��u�[�t�ת������w��0
			VectorTemp2[i+3+AxisJump]=0; // r=0

		//Swing Body(Waist) AXr 
		Cross2Vd(AlphaL+AxisJump,rCOMcw+6,VectorTemp2+AxisJump);
		for (int i = 0 ; i < 6 ; i++)		//VectorTemp2 012 345=0 �]���M678�@�I �ҥHLocak���ߤ��u�[�t�׬�0
			VectorTemp2[i]=0; // r=0

		//Swing Teight AXr
		Cross2Vd(AlphaL+6,rCOMcw+9,VectorTemp2+6);			//���j�LLink��Alpha��678 �⭫�ߤ��u�[�t�צs�b678

		//Swing Shank AXr
		Cross2Vd(AlphaL+9,rCOMcw+12,VectorTemp2+9);			//���p�LLink��Alpha��91011 �⭫�ߤ��u�[�t�צs�b91011

		//Swing Foot AXr
		for (int i = 0 ; i < 3 ; i++)							// �������wVectorTemp2 121314���Ȭ�0 �]���M151617�@�I
			VectorTemp2[i+12]=0; // r=0
		Cross2Vd(AlphaL+15,rCOMcw+15,VectorTemp2+15);	

	//Accumalate �]��Index�t�X �i�H�����@�_�[
		for (int i = 0 ; i < 12 ; i++)
		{
			MatAddAB(VectorTemp1+3*i,VectorTemp2+3*i,AccelCOM+3*i,3);
			MatAddAB(AccelJ+3*i,AccelCOM+3*i,AccelCOM+3*i,3);
		}
	}

		// �H�U���ե�

		//fstream Fx;
		//Fx.open("TestAccelCOMFootL.txt",ios::app);		
		//if(selIK == 1)//(�k�}support)
		//Fx << AccelCOM[15] << "\t"<< AccelCOM[16] << "\t"<< AccelCOM[17]<< endl;
		//else if(selIK == 0||selIK==2) //���}support
		//Fx << 0 << "\t"<< 0 << "\t"<< 0 << endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestAccelCOMFootR.txt",ios::app);		
		//if(selIK == 1)//(�k�}support)
		//Fx << 0 << "\t"<< 0 << "\t"<< 0 << endl;
		//else if(selIK == 0||selIK==2) //���}support
		//Fx << AccelCOM[33] << "\t"<< AccelCOM[34] << "\t"<< AccelCOM[35] << endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestAccelCOMShankL.txt",ios::app);		
		//if(selIK == 1)//(�k�}support)
		//Fx << AccelCOM[9] << "\t"<< AccelCOM[10] << "\t"<< AccelCOM[11]<< endl;
		//else if(selIK == 0||selIK==2) //���}support
		//Fx << AccelCOM[12] << "\t"<< AccelCOM[13] << "\t"<< AccelCOM[14]<< endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestAccelCOMShankR.txt",ios::app);		
		//if(selIK == 1)//(�k�}support)
		//Fx << AccelCOM[30] << "\t"<< AccelCOM[31] << "\t"<< AccelCOM[32]<< endl;
		//else if(selIK == 0||selIK==2) //���}support
		//Fx << AccelCOM[27] << "\t"<< AccelCOM[28] << "\t"<< AccelCOM[29]<< endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestAccelCOMWaist.txt",ios::app);		
		//if(selIK == 1)//(�k�}support)
		//Fx << AccelCOM[18] << "\t"<< AccelCOM[19] << "\t"<< AccelCOM[20]<< endl;
		//else if(selIK == 0||selIK==2) //���}support
		//Fx << AccelCOM[0] << "\t"<< AccelCOM[1] << "\t"<< AccelCOM[2]<< endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestAccelCOMThighL.txt",ios::app);		
		//if(selIK == 1)//(�k�}support)
		//Fx << AccelCOM[6] << "\t"<< AccelCOM[7] << "\t"<< AccelCOM[8]<< endl;
		//else if(selIK == 0||selIK==2) //���}support
		//Fx << AccelCOM[9] << "\t"<< AccelCOM[10] << "\t"<< AccelCOM[11]<< endl;
		//Fx.close();

		////fstream Fx;
		//Fx.open("TestAccelCOMThighR.txt",ios::app);		
		//if(selIK == 1)//(�k�}support)
		//Fx << AccelCOM[27] << "\t"<< AccelCOM[28] << "\t"<< AccelCOM[29]<< endl;
		//else if(selIK == 0||selIK==2) //���}support
		//Fx << AccelCOM[24] << "\t"<< AccelCOM[25] << "\t"<< AccelCOM[26]<< endl;
		//Fx.close();
}
void Kine::FDForceJ(void) 
{
	/******************************************************************
	input: void
	output: void

	Note:
	// �O�W�ȶ��A�ե�~!!!!
	
	// Find Joint Force �b���p��Ҧ�link��ߨ����ߥ[�t��(AccelCOM)�� ���䪺Joint�һݬI���O
	// �p��ɶ��`�NIndex�����t �b���ҫ��w�g�j�q²�� �аѷ�catia�Ҷq�o����q �b�@�I����q�ҥѳ̾a�񥽺��I���b�Ӱ��N��
	// ��l�@�b��m�@�߳]��0 �S�H�̾a�񥽺��I���b�a��ܦ]���o�˽�ߥ[�t��(AccelCOM) ��Index�~�|���T
	// �b���B�Ψ�O�W����T �M�W���@�ˤ������}�άO�k�}support�ҥH���ΦA�t�~���X�}�|�����a�ɪ����p 
	// ���bsupport�} �}�O����q�n�A���s��Index �ثe�]���s �Ϧa���ϧ@�ΤO(�O�W��) ���P��̫�@�b���O   <---�ˬd��

	// �ثe�O�Hsupport�}���O�W�ϱ��^�� ���F��p���Z�~�t���v�T �qsupport�}�l�����n
	// �S�O�`�N �b���p��O���y�Ф��M���e�⨤�� �t�ת��y�Ф@�� �P�� �@�ɤU �y��
	// �]���O���V�q�i�H���� �H�@�ɮy�к⧹���ᵥ���A��Torque�b�@�_��N�n
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
/////////////////////////////////////////////////////////////////�`�N! �H�U��SlongZ���tDSP��ZMP//////////////////////////////////////////////////////////////
//if(DSPFlag==1 || selIK==2) //Checked    //selIK = 2; // double support phas  �o�Ӫ��A�O���¦bDSP�U���O�w�g�ǳƭn��}�F �ҥH�n��ZMP�����}���O�����t
//{
//		//�v�� Checked
//		MatScalarMul(CrdAll->data+12, 3, 1.0, TempVectorL );		//CrdAll->data+12 ���}ankle
//		MatScalarMul(CrdAll->data+51, 3, 1.0, TempVectorR );		//CrdAll->data+51 �k�}ankle
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
/////////////////////////////////////////////////////////////////�`�N! �H�W��SlongZ���tDSP��ZMP//////////////////////////////////////////////////////////////	
	

////////////////////////////////////////�`�N! �H�U�b���O�qFixed�}���O�W�ȩ�Swing�}����Joint���O//////////////////////////////////////////////////////////////
	
	//// �`�N! �b���O�qFixed�}���O�W�ȩ�Swing�}����Joint���O
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
	//	// ���Y��q�C�ӳ����q �b������(121314~012)�i�Q��for�j��@���⧹ ���]���{�b�ҫ��ٲ��S�� �ҥH�u��C�C�� �A�Ϋ��w��
	//	// �n�A�ˬd!
	//	//for (int i = 4 ; i >= 0 ; i--)
	//	//{
	//	//	MatScalarMul(GravityZaxi, 3, mass_com+3*i, Gravity);	// i = 5		// m121314 * g
	//	//	MatScalarMul(AccelCOM+3*i, 3, mass_com+3*i, ForceCOM);	// ForceCOM121314 = m121314 * ac121314
	//	//	MatScalarMul(ForceCOM, 3, -1.0, ForceCOM);				// -ForceCOM121314 = -m121314 * ac121314
	//	//	MatAddAB(ForceCOM,Gravity,ForceJ+3*i,3);				// -ma + mg
	//	//	MatAddAB(ForceJ+3*i,ForceJ+3*(i+1),ForceJ+3*i,3);		// ForceJ151617 - ForceJ121314 + mg =  ForceCOM121314
	//	//}
	//	//////////////////////////////////////////////////////////////////////////////////////////

	//	for (int i = 0 ; i < 3 ; i++)	// �]��151617 121314 �b�@�I �ҥH�������w��ӤO�۵�
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
	//	for (int i = 0 ; i < 3 ; i++)	//�������w�@�I�b���O ForceJ345 = ForceJ012 = ForceJ678
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
	//	// ���Y��q�C�ӳ����q �b������(212223~333435)�i�Q��for�j��@���⧹ ���]���{�b�ҫ��ٲ��S�� �ҥH�u��C�C�� �A�Ϋ��w��
	//	// �n�A�ˬd!
	//	//for (int i = 1 ; i <= 5 ; i++)
	//	//{
	//	//	MatScalarMul(GravityZaxi, 3, mass_com+3*i+AxisJump, Gravity);						// i = 1		// m212223 * g
	//	//	MatScalarMul(AccelCOM+3*i+AxisJump, 3, mass_com+3*i+AxisJump, ForceCOM);			// ForceCOM212223 = m212223 * ac212223
	//	//	MatScalarMul(ForceCOM, 3, -1.0, ForceCOM);											// -ForceCOM212223 = -m212223 * ac212223
	//	//	MatAddAB(ForceCOM,Gravity,ForceJ+3*i+AxisJump,3);									// -ma + mg
	//	//	MatAddAB(ForceJ+3*i+AxisJump,ForceJ+3*(i-1)+AxisJump,ForceJ+3*i+AxisJump,3);		// ForceJ181920 - ForceJ212223 + mg =  ForceCOM212223
	//	//}
	//	//////////////////////////////////////////////////////////////////////////////////////////

	//	for (int i = 0 ; i < 3 ; i++)		//�������w�@�I�b���O ForceJ212223 = ForceJ242526 = ForceJ181920
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

	//	for (int i = 0 ; i < 3 ; i++)		//�������w�@�I�b���O ForceJ333435 = ForceJ303132 
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
	//	// ���Y��q�C�ӳ����q �b������(303132~181920)�i�Q��for�j��@���⧹ ���]���{�b�ҫ��ٲ��S�� �ҥH�u��C�C�� �A�Ϋ��w��
	//	// �n�A�ˬd!
	//	//for (int i = 4 ; i >= 0 ; i--)
	//	//{
	//	//	MatScalarMul(GravityZaxi, 3, mass_com+AxisJump+3*i, Gravity);				// i = 4		// m303132 * g
	//	//	MatScalarMul(AccelCOM+3*i+AxisJump, 3, mass_com+AxisJump+3*i, ForceCOM);	// ForceCOM303132 = m303132 * ac303132
	//	//	MatScalarMul(ForceCOM, 3, -1.0, ForceCOM);									// -ForceCOM303132 = -m303132 * ac303132
	//	//	MatAddAB(ForceCOM,Gravity,ForceJ+3*i+AxisJump,3);							// -ma + mg
	//	//	MatAddAB(ForceJ+3*i+AxisJump,ForceJ+3*(i+1)+AxisJump,ForceJ+3*i+AxisJump,3);		// ForceJ333435 - ForceJ303132 + mg =  ForceCOM303132	
	//	//}
	//	//////////////////////////////////////////////////////////////////////////////////////////

	//	for (int i = 0 ; i < 3 ; i++)		// �]��333435 303132 �b�@�I �ҥH�������w��ӤO�۵�
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
	//	for (int i = 0 ; i < 3 ; i++)		//�������w�@�I�b���O ForceJ242526 = ForceJ212223 = ForceJ181920
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
	//	// ���Y��q�C�ӳ����q �b������(345~151617)�i�Q��for�j��@���⧹ ���]���{�b�ҫ��ٲ��S�� �ҥH�u��C�C�� �A�Ϋ��w��
	//	// �n�A�ˬd!
	//	//for (int i = 1 ; i <= 5 ; i++)
	//	//{
	//	//	MatScalarMul(GravityZaxi, 3, mass_com+3*i, Gravity);	// i = 0		// m345 * g
	//	//	MatScalarMul(AccelCOM+3*i, 3, mass_com+3*i, ForceCOM);	// ForceCOM345 = m345 * ac345
	//	//	MatScalarMul(ForceCOM, 3, -1.0, ForceCOM);				// -ForceCOM345 = -m345 * ac345
	//	//	MatAddAB(ForceCOM,Gravity,ForceJ+3*i,3);				// -ma + mg
	//	//	MatAddAB(ForceJ+3*i,ForceJ+3*(i-1),ForceJ+3*i,3);		// ForceJ012 - ForceJ345 + mg =  ForceCOM345
	//	//}
	//	//////////////////////////////////////////////////////////////////////////////////////////

	//	for (int i = 0 ; i < 3 ; i++)		//�������w�@�I�b���O ForceJ012 = ForceJ345 = ForceJ678
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

	//	for (int i = 0 ; i < 3 ; i++)	// �������w�@�I�b���O ForceJ151617 = ForceJ121314 
	//		ForceJ[i+15] = ForceJ[i+12];

	//	for( int i = 0 ; i < 36 ; i++)	// �]���ۥ���Ϫ���] �O�䤤�@������[�t���bMatlab�W"�H"�ݰ_�Ӥ~�O���T�� �q���p��h���� �]������mode�|�t�X�ۤv�����t
	//		ForceJ[i] = -ForceJ[i];
	//}
////////////////////////////////////////�`�N! �H�W�b���O�qFixed�}���O�W�ȩ�Swing�}����Joint���O//////////////////////////////////////////////////////////////



////////////////////////////////////////�`�N! �H�U�b���O�qswing�}���O�W�ȩ�Fixed�}����Joint���O//////////////////////////////////////////////////////////////
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
		// ���Y��q�C�ӳ����q �b������(303132~181920)�i�Q��for�j��@���⧹ ���]���{�b�ҫ��ٲ��S�� �ҥH�u��C�C�� �A�Ϋ��w��
		// �n�A�ˬd!
		//for (int i = 4 ; i >= 0 ; i--)
		//{
		//	MatScalarMul(GravityZaxi, 3, mass_com+AxisJump+3*i, Gravity);				// i = 4		// m303132 * g
		//	MatScalarMul(AccelCOM+3*i+AxisJump, 3, mass_com+AxisJump+3*i, ForceCOM);	// ForceCOM303132 = m303132 * ac303132
		//	MatScalarMul(ForceCOM, 3, -1.0, ForceCOM);									// -ForceCOM303132 = -m303132 * ac303132
		//	MatAddAB(ForceCOM,Gravity,ForceJ+3*i+AxisJump,3);							// -ma + mg
		//	MatAddAB(ForceJ+3*i+AxisJump,ForceJ+3*(i+1)+AxisJump,ForceJ+3*i+AxisJump,3);// ForceJ333435 - ForceJ303132 + mg =  ForceCOM303132	
		//}
		//////////////////////////////////////////////////////////////////////////////////////////

		for (int i = 0 ; i < 3 ; i++)		// �]��333435 303132 �b�@�I �ҥH�������w��ӤO�۵�
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
		
		for (int i = 0 ; i < 3 ; i++)		//�������w�@�I�b���O ForceJ242526 = ForceJ212223 = ForceJ181920
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
		// ���Y��q�C�ӳ����q �b������(345~151617)�i�Q��for�j��@���⧹ ���]���{�b�ҫ��ٲ��S�� �ҥH�u��C�C�� �A�Ϋ��w��
		// �n�A�ˬd!
		//for (int i = 1 ; i <= 5 ; i++)
		//{
		//	MatScalarMul(GravityZaxi, 3, mass_com+3*i, Gravity);	// i = 0		// m345 * g
		//	MatScalarMul(AccelCOM+3*i, 3, mass_com+3*i, ForceCOM);	// ForceCOM345 = m345 * ac345
		//	MatScalarMul(ForceCOM, 3, -1.0, ForceCOM);				// -ForceCOM345 = -m345 * ac345
		//	MatAddAB(ForceCOM,Gravity,ForceJ+3*i,3);				// -ma + mg
		//	MatAddAB(ForceJ+3*i,ForceJ+3*(i-1),ForceJ+3*i,3);		// ForceJ012 - ForceJ345 + mg =  ForceCOM345
		//}
		//////////////////////////////////////////////////////////////////////////////////////////
	
		for (int i = 0 ; i < 3 ; i++)		//�������w�@�I�b���O ForceJ012 = ForceJ345 = ForceJ678
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

		for (int i = 0 ; i < 3 ; i++)		//�������w�@�I�b���O ForceJ151617 = ForceJ121314 
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
		// ���Y��q�C�ӳ����q �b������(121314~012)�i�Q��for�j��@���⧹ ���]���{�b�ҫ��ٲ��S�� �ҥH�u��C�C�� �A�Ϋ��w��
		// �n�A�ˬd!
		//for (int i = 4 ; i >= 0 ; i--)
		//{
		//	MatScalarMul(GravityZaxi, 3, mass_com+3*i, Gravity);	// i = 5		// m121314 * g
		//	MatScalarMul(AccelCOM+3*i, 3, mass_com+3*i, ForceCOM);	// ForceCOM121314 = m121314 * ac121314
		//	MatScalarMul(ForceCOM, 3, -1.0, ForceCOM);				// -ForceCOM121314 = -m121314 * ac121314
		//	MatAddAB(ForceCOM,Gravity,ForceJ+3*i,3);				// -ma + mg
		//	MatAddAB(ForceJ+3*i,ForceJ+3*(i+1),ForceJ+3*i,3);		// ForceJ151617 - ForceJ121314 + mg =  ForceCOM121314
		//}
		//////////////////////////////////////////////////////////////////////////////////////////

		for (int i = 0 ; i < 3 ; i++)	// �]��151617 121314 �b�@�I �ҥH�������w��ӤO�۵�
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
		
		for (int i = 0 ; i < 3 ; i++)	//�������w�@�I�b���O ForceJ345 = ForceJ012 = ForceJ678
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
		// ���Y��q�C�ӳ����q �b������(212223~333435)�i�Q��for�j��@���⧹ ���]���{�b�ҫ��ٲ��S�� �ҥH�u��C�C�� �A�Ϋ��w��
		// �n�A�ˬd!
		//for (int i = 1 ; i <= 5 ; i++)
		//{
		//	MatScalarMul(GravityZaxi, 3, mass_com+3*i+AxisJump, Gravity);						// i = 1		// m212223 * g
		//	MatScalarMul(AccelCOM+3*i+AxisJump, 3, mass_com+3*i+AxisJump, ForceCOM);			// ForceCOM212223 = m212223 * ac212223
		//	MatScalarMul(ForceCOM, 3, -1.0, ForceCOM);											// -ForceCOM212223 = -m212223 * ac212223
		//	MatAddAB(ForceCOM,Gravity,ForceJ+3*i+AxisJump,3);									// -ma + mg
		//	MatAddAB(ForceJ+3*i+AxisJump,ForceJ+3*(i-1)+AxisJump,ForceJ+3*i+AxisJump,3);		// ForceJ181920 - ForceJ212223 + mg =  ForceCOM212223
		//}
		//////////////////////////////////////////////////////////////////////////////////////////

		for (int i = 0 ; i < 3 ; i++)		//�������w�@�I�b���O ForceJ212223 = ForceJ242526 = ForceJ181920
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

		for (int i = 0 ; i < 3 ; i++)	// �������w�@�I�b���O ForceJ333435 = ForceJ303132 
			ForceJ[i+15+AxisJump]=ForceJ[i+12+AxisJump];
		
		// �]���ۥ���Ϫ���] �O�䤤�@������[�t���bMatlab�W"�H"�ݰ_�Ӥ~�O���T�� �q���p��h���� �]������mode�|�t�X�ۤv�����t
		// �{�b�b�k�}support�ɥ[�t�� �ҥH�O����V�n�ݥk�}�쥪�}����V
		// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~�`�N!!~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~�b�o�̴N�[�t���O�_�|�v�T���U�Ӻ�Torque? �A�Q�Q!
		//for( int i = 0 ; i < 36 ; i++)	
		//	ForceJ[i] = -ForceJ[i];
	}
////////////////////////////////////////�`�N! �H�W�b���O�qswing�}���O�W�ȩ�Fixed�}����Joint���O//////////////////////////////////////////////////////////////

		// �H�U���ե�

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
	// �O�W�ȶ��A�ե�~!!!!
	// �b�����Q�ΦU����߮y�дy�z��ʺD�q �åB�b���y�ФU�p���ߤO�x�B�ʤ�{
	// �A�N����o������^�@�� �P�e��Joint���O�ҳy�����O�x�p��åB���N
	// �`�N���жb���ܴ��H�άO��������άO���H�ݩҭn�[�����t��
	// �Y�b�@�뱡�ΤUCOM�B�ʴX�G�i�H�������t
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
	//	//�v�� Checked
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
	//		//�̫�@�bTheta dot * z�b + OmegaJ 15 (OmegaJ 15���]�t�ۨ���ʵ��������t�װ^�m)
	//			MatScalarMul(ZAxisAll->data+3*5+DHJump, 3, ThetaD+5+6, TempVector1);
	//			MatAddAB(OmegaJ+15+AxisJump,TempVector1,TempOmegaJ,3);

	//		//�̫�@�bOmegaJ cross Local OmegaJ + OmegaJ 15 (AlphaJ 15���]�t�ۨ���ʵ��������[�t�װ^�m)
	//			//OmegaJ cross Local OmegaJ
	//					Cross2Vd(OmegaJ+3*5+AxisJump, LocalOmegaJ+3*5+AxisJump,TempVector1);
	//			//Theta dot dot * z�b
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
			// ���o�U�@�b������x�}	
			GetRotPartRn(&FKRLeg->Rn[5],Rot_Part_Rn);	
			
			// I*Alpha (IAlpha)	
			MatMulAB(Rot_Part_Rn,3,3,AlphaL+15+AxisJump,3,1,TempVector);
			MatMulAB(IcRFoot,3,3,TempVector,3,1,IAlpha);	
			
			// Omega Cross I*Omega	(OmegaIOmega)	
			MatMulAB(Rot_Part_Rn,3,3, OmegaL+15+AxisJump,3,1, ROmega);
			MatMulAB(IcRFoot,3,3, ROmega,3,1, TempVector);
			Cross2Vd(ROmega, TempVector, OmegaIOmega);
			
			// �N��߹B��(I*Alpha (IAlpha)	+ Omega Cross I*Omega (OmegaIOmega))��^�@�� �åB�[�t�� ���@�U�����[�_��
			MatAddAB(IAlpha, OmegaIOmega, TempVector,3);
			MatMulAtB(Rot_Part_Rn,3,3,TempVector,3,1,Center);
			MatScalarMul(Center, 3, -1, Center);

			// �p��O�ҳy�����O�x
			Cross2Vd(rCOMccw+15, ForceJ+AxisJump+15, FCrossR1);	// r Cross force

			MatScalarMul(Rstack+15, 3, -1, TempVector);
			MatAddAB(TempVector, rCOMccw+15, TempVector,3);
			MatAddAB(TempVector, SensorOffset, TempVector,3);
			Cross2Vd(FSensor_forcR, TempVector, FCrossR2);	// force Cross r

			// �p��Ӷb�O�x
			MatAddAB(FSensor_TorqR, FCrossR2, TempVector,3);
			MatAddAB(FCrossR1, Center, TorqueJ+15+AxisJump,3);
			MatAddAB(TempVector, TorqueJ+15+AxisJump, TorqueJ+15+AxisJump,3);
			 
			// ���wTorqueJ303132 = TorqueJ333435
			for (int i = 0 ; i < 3 ; i++)
				TorqueJ[i+AxisJump+12]=TorqueJ[i+AxisJump+15];

		//Swing Shank
			// ���o�U�@�b������x�}
			GetRotPartRn(&FKRLeg->Rn[3],Rot_Part_Rn);	
			
			// I*Alpha (IAlpha)	
			MatMulAB(Rot_Part_Rn,3,3,AlphaL+9+AxisJump,3,1,TempVector);
			MatMulAB(IcRShank,3,3,TempVector,3,1,IAlpha);	
			
			// Omega Cross I*Omega	(OmegaIOmega)	
			MatMulAB(Rot_Part_Rn,3,3, OmegaL+9+AxisJump,3,1, ROmega);
			MatMulAB(IcRShank,3,3, ROmega,3,1, TempVector);
			Cross2Vd(ROmega, TempVector, OmegaIOmega);
			
			// �N��߹B��(I*Alpha (IAlpha)	+ Omega Cross I*Omega (OmegaIOmega))��^�@�� �åB�[�t�� ���@�U�����[�_��
			MatAddAB(IAlpha, OmegaIOmega, TempVector,3);
			MatMulAtB(Rot_Part_Rn,3,3,TempVector,3,1,Center);
			MatScalarMul(Center, 3, -1, Center);

			// �p��O�ҳy�����O�x
			Cross2Vd(rCOMccw+12, ForceJ+AxisJump+9, FCrossR1);	// r Cross force
			Cross2Vd(ForceJ+AxisJump+12,rCOMcw,FCrossR2);	// force Cross r

			// �p��Ӷb�O�x
			MatAddAB(TorqueJ+12+AxisJump, FCrossR2, TempVector,3);
			MatAddAB(FCrossR1, Center, TorqueJ+9+AxisJump,3);
			MatAddAB(TempVector, TorqueJ+9+AxisJump, TorqueJ+9+AxisJump,3);

		//Swing Thigh
			// ���o�U�@�b������x�}
			GetRotPartRn(&FKRLeg->Rn[2],Rot_Part_Rn);	
			
			// I*Alpha (IAlpha)	
			MatMulAB(Rot_Part_Rn,3,3,AlphaL+6+AxisJump,3,1,TempVector);
			MatMulAB(IcRThigh,3,3,TempVector,3,1,IAlpha);	
			
			// Omega Cross I*Omega	(OmegaIOmega)	
			MatMulAB(Rot_Part_Rn,3,3, OmegaL+6+AxisJump,3,1, ROmega);
			MatMulAB(IcRThigh,3,3, ROmega,3,1, TempVector);
			Cross2Vd(ROmega, TempVector, OmegaIOmega);
			
			// �N��߹B��(I*Alpha (IAlpha)	+ Omega Cross I*Omega (OmegaIOmega))��^�@�� �åB�[�t�� ���@�U�����[�_��
			MatAddAB(IAlpha, OmegaIOmega, TempVector,3);
			MatMulAtB(Rot_Part_Rn,3,3,TempVector,3,1,Center);
			MatScalarMul(Center, 3, -1, Center);

			// �p��O�ҳy�����O�x
			Cross2Vd(rCOMccw+9, ForceJ+AxisJump+6, FCrossR1);	// r Cross force
			Cross2Vd(ForceJ+AxisJump+9,rCOMcw+3,FCrossR2);	// force Cross r

			// �p��Ӷb�O�x
			MatAddAB(TorqueJ+9+AxisJump, FCrossR2, TempVector,3);
			MatAddAB(FCrossR1, Center, TorqueJ+6+AxisJump,3);
			MatAddAB(TempVector, TorqueJ+6+AxisJump, TorqueJ+6+AxisJump,3);
			
			// ���wTorqueJ181920 = TorqueJ212223 = TorqueJ242526
			for (int i = 0 ; i < 3 ; i++)
			{
				TorqueJ[i+AxisJump]=TorqueJ[i+6+AxisJump];
				TorqueJ[i+3+AxisJump]=TorqueJ[i+6+AxisJump];
			}

		//Body 
			// ���o�U�@�b������x�}
			GetRotPartRn(&FKLArm->Rn[0],Rot_Part_Rn);	
			// �b����ܶb678 �@������x�}�O�_���~? ���n��012?
			// ���Y�o�˿�� �h�bFindInertia2LocalCOM����]�n�ӥ��k�} support�ӱ���Local�D�q
			// �����y�Ъ����I�b��y�лݩT�w�b���W�H��Link��� �G�o�̳̫���FKLArm->Rn[0] ���y���Ĥ@�b
			
			// I*Alpha (IAlpha)	
			MatMulAB(Rot_Part_Rn,3,3,AlphaL,3,1,TempVector);
			MatMulAB(IcBody,3,3,TempVector,3,1,IAlpha);	
			
			// Omega Cross I*Omega	(OmegaIOmega)	
			MatMulAB(Rot_Part_Rn,3,3, OmegaL,3,1, ROmega);
			MatMulAB(IcBody,3,3, ROmega,3,1, TempVector);
			Cross2Vd(ROmega, TempVector, OmegaIOmega);
			
			// �N��߹B��(I*Alpha (IAlpha)	+ Omega Cross I*Omega (OmegaIOmega))��^�@�� �åB�[�t�� ���@�U�����[�_��
			MatAddAB(IAlpha, OmegaIOmega, TempVector,3);
			MatMulAtB(Rot_Part_Rn,3,3,TempVector,3,1,Center);
			MatScalarMul(Center, 3, -1, Center);

			// �p��O�ҳy�����O�x
			Cross2Vd(rCOMccw+6, ForceJ, FCrossR1);	// r Cross force
			Cross2Vd(ForceJ+AxisJump,rCOMcw+6,FCrossR2);	// force Cross r

			// �p��Ӷb�O�x
			MatAddAB(TorqueJ+AxisJump, FCrossR2, TempVector,3);
			MatAddAB(FCrossR1, Center, TorqueJ,3);
			MatAddAB(TempVector, TorqueJ, TorqueJ,3);

			// ���wTorqueJ345 = TorqueJ678 = TorqueJ012
			for (int i = 0 ; i < 3 ; i++) 
			{
				TorqueJ[i+3]=TorqueJ[i];
				TorqueJ[i+6]=TorqueJ[i];
			}
			
		//Fixed Thigh
			// ���o�U�@�b������x�}
			GetRotPartRn(&FKLLeg->Rn[2],Rot_Part_Rn);	
			
			// I*Alpha (IAlpha)	
			MatMulAB(Rot_Part_Rn,3,3,AlphaL+9,3,1,TempVector);
			MatMulAB(IcLThigh,3,3,TempVector,3,1,IAlpha);	
			
			// Omega Cross I*Omega	(OmegaIOmega)	
			MatMulAB(Rot_Part_Rn,3,3, OmegaL+9,3,1, ROmega);
			MatMulAB(IcLThigh,3,3, ROmega,3,1, TempVector);
			Cross2Vd(ROmega, TempVector, OmegaIOmega);
			
			// �N��߹B��(I*Alpha (IAlpha)	+ Omega Cross I*Omega (OmegaIOmega))��^�@�� �åB�[�t�� ���@�U�����[�_��
			MatAddAB(IAlpha, OmegaIOmega, TempVector,3);
			MatMulAtB(Rot_Part_Rn,3,3,TempVector,3,1,Center);
			MatScalarMul(Center, 3, -1, Center);

			// �p��O�ҳy�����O�x
			Cross2Vd(rCOMccw+3, ForceJ+9, FCrossR1);	// r Cross force
			Cross2Vd(ForceJ+6,rCOMcw+9,FCrossR2);	// force Cross r

			// �p��Ӷb�O�x
			MatAddAB(TorqueJ+6, FCrossR2, TempVector,3);
			MatAddAB(FCrossR1, Center, TorqueJ+9,3);
			MatAddAB(TempVector, TorqueJ+9, TorqueJ+9,3);

		//Fixed Shank
			// ���o�U�@�b������x�}
			GetRotPartRn(&FKLLeg->Rn[3],Rot_Part_Rn);	
			
			// I*Alpha (IAlpha)	
			MatMulAB(Rot_Part_Rn,3,3,AlphaL+12,3,1,TempVector);
			MatMulAB(IcLShank,3,3,TempVector,3,1,IAlpha);	
			
			// Omega Cross I*Omega	(OmegaIOmega)	
			MatMulAB(Rot_Part_Rn,3,3, OmegaL+12,3,1, ROmega);
			MatMulAB(IcLShank,3,3, ROmega,3,1, TempVector);
			Cross2Vd(ROmega, TempVector, OmegaIOmega);
			
			// �N��߹B��(I*Alpha (IAlpha)	+ Omega Cross I*Omega (OmegaIOmega))��^�@�� �åB�[�t�� ���@�U�����[�_��
			MatAddAB(IAlpha, OmegaIOmega, TempVector,3);
			MatMulAtB(Rot_Part_Rn,3,3,TempVector,3,1,Center);
			MatScalarMul(Center, 3, -1, Center);

			// �p��O�ҳy�����O�x
			Cross2Vd(rCOMccw, ForceJ+12, FCrossR1);	// r Cross force
			Cross2Vd(ForceJ+9,rCOMcw+12,FCrossR2);	// force Cross r

			// �p��Ӷb�O�x
			MatAddAB(TorqueJ+9, FCrossR2, TempVector,3);
			MatAddAB(FCrossR1, Center, TorqueJ+12,3);
			MatAddAB(TempVector, TorqueJ+12, TorqueJ+12,3);

			// ���wTorqueJ151617 = TorqueJ121314
			for (int i = 0 ; i < 3 ; i++)
 				TorqueJ[i+15]=TorqueJ[i+12];

		// �]���ۥ���Ϫ���] �O�䤤�@������[�t���bMatlab�W"�H"�ݰ_�Ӥ~�O���T�� �q���p��h���� �]������mode�|�t�X�ۤv�����t
		// �{�b�b���}support�ɥk�}�[�t�� �ҥH�O�M�O�x����V�n�Ѹy�ݨ�}��
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
			// ���o�U�@�b������x�}	
			GetRotPartRn(&FKLLeg->Rn[5],Rot_Part_Rn);	
			
			// I*Alpha (IAlpha)	
			MatMulAB(Rot_Part_Rn,3,3,AlphaL+15,3,1,TempVector);
			MatMulAB(IcLFoot,3,3,TempVector,3,1,IAlpha);	
			
			// Omega Cross I*Omega	(OmegaIOmega)	
			MatMulAB(Rot_Part_Rn,3,3, OmegaL+15,3,1, ROmega);
			MatMulAB(IcLFoot,3,3, ROmega,3,1, TempVector);
			Cross2Vd(ROmega, TempVector, OmegaIOmega);
			
			// �N��߹B��(I*Alpha (IAlpha)	+ Omega Cross I*Omega (OmegaIOmega))��^�@�� �åB�[�t�� ���@�U�����[�_��
			MatAddAB(IAlpha, OmegaIOmega, TempVector,3);
			MatMulAtB(Rot_Part_Rn,3,3,TempVector,3,1,Center);
			MatScalarMul(Center, 3, -1, Center);

			// �p��O�ҳy�����O�x
			Cross2Vd(rCOMcw+15, ForceJ+15, FCrossR1);	// r Cross force

			MatScalarMul(Rstack+6, 3, -1, TempVector);
			MatAddAB(TempVector, rCOMcw+15, TempVector,3);
			MatAddAB(TempVector, SensorOffset, TempVector,3);
			Cross2Vd(FSensor_forcL, TempVector, FCrossR2);	// force Cross r

		//		 //�H�U���ե�
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



			// �p��Ӷb�O�x
			MatAddAB(FSensor_TorqL, FCrossR2, TempVector,3);
			MatAddAB(FCrossR1, Center, TorqueJ+15,3);
			MatAddAB(TempVector, TorqueJ+15, TorqueJ+15,3);
			 
			// ���wTorqueJ121314 = TorqueJ151617
			for (int i = 0 ; i < 3 ; i++)
				TorqueJ[i+12]=TorqueJ[i+15];

		//Swing Shank
			// ���o�U�@�b������x�}
			GetRotPartRn(&FKLLeg->Rn[3],Rot_Part_Rn);	
			
			// I*Alpha (IAlpha)	
			MatMulAB(Rot_Part_Rn,3,3,AlphaL+9,3,1,TempVector);
			MatMulAB(IcLShank,3,3,TempVector,3,1,IAlpha);	
			
			// Omega Cross I*Omega	(OmegaIOmega)	
			MatMulAB(Rot_Part_Rn,3,3, OmegaL+9,3,1, ROmega);
			MatMulAB(IcLShank,3,3, ROmega,3,1, TempVector);
			Cross2Vd(ROmega, TempVector, OmegaIOmega);
			
			// �N��߹B��(I*Alpha (IAlpha)	+ Omega Cross I*Omega (OmegaIOmega))��^�@�� �åB�[�t�� ���@�U�����[�_��
			MatAddAB(IAlpha, OmegaIOmega, TempVector,3);
			MatMulAtB(Rot_Part_Rn,3,3,TempVector,3,1,Center);
			MatScalarMul(Center, 3, -1, Center);

			// �p��O�ҳy�����O�x
			Cross2Vd(rCOMcw+12, ForceJ+9, FCrossR1);	// r Cross force
			Cross2Vd(ForceJ+12,rCOMccw,FCrossR2);	// force Cross r

		//	 //�H�U���ե�
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

			// �p��Ӷb�O�x
			MatAddAB(TorqueJ+12, FCrossR2, TempVector,3);
			MatAddAB(FCrossR1, Center, TorqueJ+9,3);
			MatAddAB(TempVector, TorqueJ+9, TorqueJ+9,3);

		//Swing Thigh
			// ���o�U�@�b������x�}
			GetRotPartRn(&FKLLeg->Rn[2],Rot_Part_Rn);	
			
			// I*Alpha (IAlpha)	
			MatMulAB(Rot_Part_Rn,3,3,AlphaL+6,3,1,TempVector);
			MatMulAB(IcLThigh,3,3,TempVector,3,1,IAlpha);	
			
			// Omega Cross I*Omega	(OmegaIOmega)	
			MatMulAB(Rot_Part_Rn,3,3, OmegaL+6,3,1, ROmega);
			MatMulAB(IcLThigh,3,3, ROmega,3,1, TempVector);
			Cross2Vd(ROmega, TempVector, OmegaIOmega);
			
			// �N��߹B��(I*Alpha (IAlpha)	+ Omega Cross I*Omega (OmegaIOmega))��^�@�� �åB�[�t�� ���@�U�����[�_��
			MatAddAB(IAlpha, OmegaIOmega, TempVector,3);
			MatMulAtB(Rot_Part_Rn,3,3,TempVector,3,1,Center);
			MatScalarMul(Center, 3, -1, Center);

			// �p��O�ҳy�����O�x
			Cross2Vd(rCOMcw+9, ForceJ+6, FCrossR1);	// r Cross force
			Cross2Vd(ForceJ+9,rCOMccw+3,FCrossR2);	// force Cross r

		//		 //�H�U���ե�
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

			// �p��Ӷb�O�x
			MatAddAB(TorqueJ+9, FCrossR2, TempVector,3);
			MatAddAB(FCrossR1, Center, TorqueJ+6,3);
			MatAddAB(TempVector, TorqueJ+6, TorqueJ+6,3);
			
			// ���wTorqueJ012 = TorqueJ345 = TorqueJ678
			for (int i = 0 ; i < 3 ; i++)
			{
				TorqueJ[i]=TorqueJ[i+6];
				TorqueJ[i+3]=TorqueJ[i+6];
			}

		//Body 
			// ���o�U�@�b������x�}
			GetRotPartRn(&FKLArm->Rn[0],Rot_Part_Rn);	
			// �b����ܶb242526 �@������x�}�O�_���~? ���n��181920?
			// ���Y�o�˿�� �h�bFindInertia2LocalCOM����]�n�ӥ��k�} support�ӱ���Local�D�q
			// �����y�Ъ����I�b��y�лݩT�w�b���W�H��Link��� �G�o�̳̫���FKLArm->Rn[0] ���y���Ĥ@�b

			// I*Alpha (IAlpha)	
			MatMulAB(Rot_Part_Rn,3,3,AlphaL+AxisJump,3,1,TempVector);
			MatMulAB(IcBody,3,3,TempVector,3,1,IAlpha);	
			
			// Omega Cross I*Omega	(OmegaIOmega)	
			MatMulAB(Rot_Part_Rn,3,3, OmegaL+AxisJump,3,1, ROmega);
			MatMulAB(IcBody,3,3, ROmega,3,1, TempVector);
			Cross2Vd(ROmega, TempVector, OmegaIOmega);
			
			// �N��߹B��(I*Alpha (IAlpha)	+ Omega Cross I*Omega (OmegaIOmega))��^�@�� �åB�[�t�� ���@�U�����[�_��
			MatAddAB(IAlpha, OmegaIOmega, TempVector,3);
			MatMulAtB(Rot_Part_Rn,3,3,TempVector,3,1,Center);
			MatScalarMul(Center, 3, -1, Center);

			// �p��O�ҳy�����O�x
			Cross2Vd(rCOMcw+6, ForceJ+AxisJump, FCrossR1);	// r Cross force
			Cross2Vd(ForceJ,rCOMccw+6,FCrossR2);	// force Cross r

		//		 //�H�U���ե�
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

			// �p��Ӷb�O�x
			MatAddAB(TorqueJ, FCrossR2, TempVector,3);
			MatAddAB(FCrossR1, Center, TorqueJ+AxisJump,3);
			MatAddAB(TempVector, TorqueJ+AxisJump, TorqueJ+AxisJump,3);

			// ���wTorqueJ242526 = TorqueJ212223 = TorqueJ181920
			for (int i = 0 ; i < 3 ; i++) 
			{
				TorqueJ[i+3+AxisJump]=TorqueJ[i+AxisJump];
				TorqueJ[i+6+AxisJump]=TorqueJ[i+AxisJump];
			}
			
		//Fixed Thigh
			// ���o�U�@�b������x�}
			GetRotPartRn(&FKRLeg->Rn[2],Rot_Part_Rn);	
			
			// I*Alpha (IAlpha)	
			MatMulAB(Rot_Part_Rn,3,3,AlphaL+AxisJump+9,3,1,TempVector);
			MatMulAB(IcRThigh,3,3,TempVector,3,1,IAlpha);	
			
			// Omega Cross I*Omega	(OmegaIOmega)	
			MatMulAB(Rot_Part_Rn,3,3, OmegaL+AxisJump+9,3,1, ROmega);
			MatMulAB(IcRThigh,3,3, ROmega,3,1, TempVector);
			Cross2Vd(ROmega, TempVector, OmegaIOmega);
			
			// �N��߹B��(I*Alpha (IAlpha)	+ Omega Cross I*Omega (OmegaIOmega))��^�@�� �åB�[�t�� ���@�U�����[�_��
			MatAddAB(IAlpha, OmegaIOmega, TempVector,3);
			MatMulAtB(Rot_Part_Rn,3,3,TempVector,3,1,Center);
			MatScalarMul(Center, 3, -1, Center);

			// �p��O�ҳy�����O�x
			Cross2Vd(rCOMcw+3, ForceJ+AxisJump+9, FCrossR1);	// r Cross force
			Cross2Vd(ForceJ+AxisJump+6,rCOMccw+9,FCrossR2);	// force Cross r

		//		 //�H�U���ե�
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

			// �p��Ӷb�O�x
			MatAddAB(TorqueJ+AxisJump+6, FCrossR2, TempVector,3);
			MatAddAB(FCrossR1, Center, TorqueJ+AxisJump+9,3);
			MatAddAB(TempVector, TorqueJ+AxisJump+9, TorqueJ+AxisJump+9,3);

		//Fixed Shank
			// ���o�U�@�b������x�}
			GetRotPartRn(&FKRLeg->Rn[3],Rot_Part_Rn);	
			
			// I*Alpha (IAlpha)	
			MatMulAB(Rot_Part_Rn,3,3,AlphaL+AxisJump+12,3,1,TempVector);
			MatMulAB(IcRShank,3,3,TempVector,3,1,IAlpha);	
			
			// Omega Cross I*Omega	(OmegaIOmega)	
			MatMulAB(Rot_Part_Rn,3,3, OmegaL+AxisJump+12,3,1, ROmega);
			MatMulAB(IcRShank,3,3, ROmega,3,1, TempVector);
			Cross2Vd(ROmega, TempVector, OmegaIOmega);
			
			// �N��߹B��(I*Alpha (IAlpha)	+ Omega Cross I*Omega (OmegaIOmega))��^�@�� �åB�[�t�� ���@�U�����[�_��
			MatAddAB(IAlpha, OmegaIOmega, TempVector,3);
			MatMulAtB(Rot_Part_Rn,3,3,TempVector,3,1,Center);
			MatScalarMul(Center, 3, -1, Center);

			// �p��O�ҳy�����O�x
			Cross2Vd(rCOMcw, ForceJ+AxisJump+12, FCrossR1);	// r Cross force
			Cross2Vd(ForceJ+AxisJump+9,rCOMccw+12,FCrossR2);	// force Cross r

		//		 //�H�U���ե�
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

			// �p��Ӷb�O�x
			MatAddAB(TorqueJ+AxisJump+9, FCrossR2, TempVector,3);
			MatAddAB(FCrossR1, Center, TorqueJ+AxisJump+12,3);
			MatAddAB(TempVector, TorqueJ+AxisJump+12, TorqueJ+AxisJump+12,3);

			// ���wTorqueJ333435 = TorqueJ303132
			for (int i = 0 ; i < 3 ; i++)
 				TorqueJ[i+AxisJump+15]=TorqueJ[i+AxisJump+12];
	
		// �]���ۥ���Ϫ���] �O�䤤�@������[�t���bMatlab�W"�H"�ݰ_�Ӥ~�O���T�� �q���p��h���� �]������mode�|�t�X�ۤv�����t
		// �{�b�b�k�}support�ɥ��}�[�t�� �ҥH�O�M�O�x����V�Ѹy�ݦV�}��
		for( int i = 0 ; i < 18 ; i++)	
		{
			ForceJ[i] = -ForceJ[i];
			TorqueJ[i] = -TorqueJ[i];
		}
		
	}
		
	// ��U�b��Torque��v��Ӷb���F�i�I�O��Z�b�W(DH model)
	for (int i = 0 ; i < 6 ; i++)
	{		
		MatMulAB(ZAxisAll->data+3*i,1,3,TorqueJ+i*3,3,1,MotorTorq+i);
		MatMulAB(ZAxisAll->data+3*i+DHJump,1,3,TorqueJ+i*3+AxisJump,3,1,MotorTorq+i+6);
	}

	//// ��v�����Ӹs��ư��o�i(Kalman Filter)
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

	//// �N��X��Motor Torque �ഫ��Rated Torque ���F��JEPOS3 Torque offset
	//// C++��X��Torque��쬰 N*m*10^-9
	//// �ҥH�n������K�� N*m �G����*10^-9
	//// �`�N!!!! �b����X��Motor��쬰 �d�����@��Rated Torque!!!!!!!!!!!
	//// �٥�������ھ��c�����t
	//for (int i = 0 ; i < 12 ; i++)
	//{
	//	//MotorTorq[i] = (x_estMotor[i]) / 1000000000 / RatedTorque[i] / GearRatio[i] ;
	//	MotorTorq[i] = (MotorTorq[i]) / 1000000000 / RatedTorque[i] / GearRatio[i] ;	// ���n�o�i �hMATLAB�o
	//}


	//for (int i = 0 ; i < 12 ; i++) //unit test
	//{
	//	MotorTorq[i] = (x_estMotor[i]);
	//}



	//// �N��X��Motor Torque �ഫ��N*mm ���F��K�bAdams���ˬd
	//for (int i = 0 ; i < 6 ; i++)
	//{
	//	MotorTorq[i] = x_estMotor[6+i]/1000000 ;
	//	MotorTorq[6+i] = x_estMotor[6+i]/1000000 ;
	//}

	//????? 20130423
	//for (int i = 0 ; i < 12 ; i++)
	//	MotorTorq[i] = MotorTorq[i]*10;
	
	// ��l��MotorTorq ��Torque Control�@�}�l����Singular
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

	// //20130115 ������JAdams�q�쪺MotionTorque (����)
	//for (int i = 0 ; i < 12 ; i++)
	//	MotorTorq[i] = AdamsMotionT[i];


	
	// 20130116 ����Adams���b�O�x�I�O��+-�PC++���Ȫ����t �u����pitch��V
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
	



	 ////�H�U���ե�
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
	// �[�J�����O �����O����쬰N*m ���ת���쬰����
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
		
		MotorTorq[i] = MotorTorq[i] / 1000000000 ;	// �U�b�ݭn����Torque(�z�Q) N*m 
		MotorTorqF[i] = MotorTorq[i] + Viscous + Coulomb*2.5;
		MotorTorq[i] = MotorTorq[i] / RatedTorque[i] / GearRatio[i];
		MotorTorqF[i] = MotorTorqF[i] / RatedTorque[i] / GearRatio[i];
	}	
	
	// �N��X��Motor Torque �ഫ��Rated Torque ���F��JEPOS3 Torque offset
	// C++��X��Torque��쬰 N*m*10^-9
	// �ҥH�n������K�� N*m �G����*10^-9
	// �`�N!!!! �b����X��Motor��쬰 �d�����@��Rated Torque!!!!!!!!!!!
	// �٥�������ھ��c�����t
	//for (int i = 0 ; i < 12 ; i++)
	//{
	//	//MotorTorq[i] = (MotorTorq[i]) / 1000000000 / RatedTorque[i] / GearRatio[i] ;	// ���n�o�i �hMATLAB�o
	//}


	 ////�H�U���ե�
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
	// �u�W���O�W�� �άO���uŪtxt��(�g��Adams�����ӱoAdams���F:N T:N*mm)
	// �NAdams�y�����C++�y�� �Ъ`�NAdams��X���O�a��}�٬O�}��a �O�V�q���ۤ��٬O�y�жb���ۤ�
	// �Q��Kalman Filter�o�i �ЯS�O�`�N���^�Ȫ��j�p ��� ��V
	// �p�G�����@������ ���utxt�аO�o�@�_��
	// �Ա��аѷ�C2M�t�C ��J���Ǧp�U
	// VLForceX, VLForceYN,VLForceZ,
	// VLTorqueX, VLTorqueYN, VLTorqueZ, 
	// VRForceX, VRForceYN,VRForceZ,
	// VRTorqueX, VRTorqueYN, VRTorqueZ
	// �b�e20��Step ���ǥ� InitialD �վ�QR �۫HSensor ����Singular
	// CountMotor > 20 ��NQR �զ^�۫HModel ���F����Impulse
	// 20130313 �Nz_measured�Q��gKineAll�վ㦨�HRobotAllDlg�ӫ��w
	// Model��SlongZ 20130111 NinoKai
	// 20130103 Wei-Zh Lai Start 
	******************************************************************/
	double OrigCoPLx;	// ���B�z���s�O�W��ܰ��D �|��������x�}�e����}COP ��local frame (��������x�}�ܦ�CoPL)
	double OrigCoPLy;
	double OrigCoPRx;
	double OrigCoPRy;
	int DataCount_FS = SensorCount+LogCount;

	// �Y��adams���� �Nadams�O�Wraw data��J�� FS_DataL �P FS_DataR (gForceDataLLeg, gForceDataRLeg) 
	// ��J����ADAMS�^�Ӫ��O�W����^C++�y�� �`�N���� ���t ��� �O�W���V�W�ݬ���
	// ���N����ন�P�O�W�ۦP 
	// �O�W  ForceUnits="N" TorqueUnits="N-m"
	// Adams��� F: N   T: N*mm
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

		for (int i = 0 ; i < 12 ; ++i)	// ADAMS �h�o�@�I
		{
			Q_KF[i] = 0.01;	// Q��model��coverence �V�p�V�۫H
			R_KF[i] = 10;	// R��measuremant��coverence �V�p�V�۫H	
		}
	}

	// �b���P���p�U���O�W���
	// ��J�O�Wraw data txt�ɳ��@�߬� F: N   T:N*m	
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
			x_est_last[i] = FS_DataL[i];// �|���ǭȫe �Ĥ@����ƪ�����w����
			x_est_last[i+6] = FS_DataR[i];
		}
		z_measured[i] = FS_DataL[(DataCount_FS)*6+i];
		z_measured[i+6] = FS_DataR[(DataCount_FS)*6+i];
	}

	// �������Ӹs��ư��o�i(Kalman Filter)
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

	// ��J�O�WTXT �^�Ӫ��O�W����^C++�y�� �`�N���� ���t ��� �O�W���V�W�ݬ���
	// Adams��� F: N   T: N*mm
	// C++��� F:g*mm/s^2 = 10^-6 N   T:g*mm/s^2 * mm = 10^-9 N*m
	// �u�W�����u��O�W�Ȥw�g�ର�����H�y��
	// �O�W����� ForceUnits="N" TorqueUnits="N-m"
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

	// Cali���� �}�l�ǭȫ� ���Usave�s�J �w�o�L�i���O�W���
	// �|�o�˦s�O�]���b�~��(PMS)�Q�n�ݨ�txt����줣�n�ӥ��j
	// ��쬰 F: N   T:N*m
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
	// �ٲ��}�O���[�t�שM���q ���ӭn�[�^�h! 
	// ���[�L�n���S�t�h��@@a ���ӬO�]�������R�樫 �i�H����
	// �`�N�b��COP���Ǧ�w��+-80 �Y�}���_�l�e�צ����� �hgCoPCali��deadzone�n��ʽվ�
	// 60�O�}���O��O�W���Z��
	// ���W����x�}�H�Q�b���s�ɤ���O�U���T��COP ZMP (�`�N���:mm)
	if(FlagGo==0){	// Go���U CrdAll�}�l����	
		// X = (x*Fz-z*Fx-My)/Fz
		// Y = (y*Fz-z*Fy+Mx)/Fz
		OrigCoPLx = (-FSensor_TorqL[1] - (60)*FSensor_forcL[0] + (CrdAll->data[21]*FSensor_forcL[2]) ) / FSensor_forcL[2];
		OrigCoPLy = (FSensor_TorqL[0] - (60)*FSensor_forcL[1] + (CrdAll->data[22]*FSensor_forcL[2]) ) / FSensor_forcL[2];
		OrigCoPRx = (-FSensor_TorqR[1] - (60)*FSensor_forcR[0] + (CrdAll->data[60]*FSensor_forcR[2]) ) / FSensor_forcR[2];
		OrigCoPRy = (FSensor_TorqR[0] - (60)*FSensor_forcR[1] + (CrdAll->data[61]*FSensor_forcR[2]) ) / FSensor_forcR[2];

		if(FSensor_forcL[2] < 20*(1e+6)){	//���}���a(< 20N) SSP ZMP���ݥ��}�O�W�� �k�}�����ISUP
			CoPL[LogCount*2] = TarRotMSw[0] * OrigCoPLx + TarRotMSw[1] * OrigCoPLy;
			CoPL[LogCount*2+1] = TarRotMSw[3] * OrigCoPLx + TarRotMSw[4] * OrigCoPLy;
			CoPR[LogCount*2] = TarRotMFx[0] * OrigCoPRx + TarRotMFx[1] * OrigCoPRy;
			CoPR[LogCount*2+1] = TarRotMFx[3] * OrigCoPRx + TarRotMFx[4] * OrigCoPRy;
						
			FS_ZMP[LogCount*2] = CoPR[DataCount_FS*2];
			FS_ZMP[LogCount*2+1] = CoPR[DataCount_FS*2+1];
		}
		else if(FSensor_forcR[2] < 20*(1e+6)){	//�k�}���a(< 20N) SSP ZMP���ݥk�}�O�W�� ���}�����ISUP
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
	else{	// Go�e CrdAll�L�� DSP (��}���b���I �ҥHXs = 0)
		CoPL[0] = (-FSensor_TorqL[1] - (60)*FSensor_forcL[0] ) / FSensor_forcL[2];
		CoPL[1] = (FSensor_TorqL[0] - (60)*FSensor_forcL[1] + (80*FSensor_forcL[2]) ) / FSensor_forcL[2];
		CoPR[0] = (-FSensor_TorqR[1] - (60)*FSensor_forcR[0] ) / FSensor_forcR[2];
		CoPR[1] = (FSensor_TorqR[0] - (60)*FSensor_forcR[1] + ((-80)*FSensor_forcR[2]) ) / FSensor_forcR[2];

		FS_ZMP[0] = (FSensor_forcL[2]*CoPL[0] + FSensor_forcR[2]*CoPR[0]) / (FSensor_forcL[2] + FSensor_forcR[2]);
		FS_ZMP[1] = (FSensor_forcL[2]*CoPL[1] + FSensor_forcR[2]*CoPR[1]) / (FSensor_forcL[2] + FSensor_forcR[2]);			
	}		

	// �H�U���ե�
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
	// �ثe��bFindDIni�I�s���禡 �u�I�s�@��
	// �H�U���ۤv���y�Ъ�ܸӱ����ʺD�q �p���C�ӱ����ʺD�q�O�@�өw�� �u�ݭp��@��
	// �Y���ɶ����b�ե� �o�̪��D�q���O����!
	// �Ъ`�N����x�}�����k�������DIc = Rt Ia R
	// ���鳡���ѩ�O�W�b�����@��� �G�ݨϥΤT������b�w�z
	// �}���Υ���b�O�]�����D�q�bcatia�U�q���N�O�H�ӱ��COM�����I �u�O�y�жb���@��
	// �}�u��235�b�@�����Шt�O�]���n���Шt�T�w�bLink�W�N �ҥH�Ѹy��Base���U�n��235�~��N���COM�����Шt
	// �b�o�̭n�NTheta���]���s���XRn����]�O�ڭ̦b�@�ɤU�q���D�q�O�b�����H�������ɭ� �]�N�OTheta = 0 ��
	// ����Theta��O�o�N�����٭�
	// 2012 Slongz Start
	// 20121218 Wei-Zh Lai Start 
	******************************************************************/
	// catia and solidworks �q�쪺�C�ӱ��b�@�ɤU�� inertia matrix
	double IR_FPad[9] = {6000, -550.1, 2000, -550.1, 9000, 98.94, 2000, 98.94, 7000}; 
	double IL_FPad[9] = {6000, 546, 2000, 546, 9000, -97.5, 2000, -97.5, 7000}; 
	double IR_KneeDown[9] = {17000, 106.1, -383, 106.1, 16000, 1000, -383, 1000, 3000};
	double IL_KneeDown[9] = {17000, -132.1, -450.3, -132.1, 16000, -1000, -450.3, -1000, 3000};
	double IR_KneeUp[9] = {33000, -200.2, -191.1, -200.2, 28000, -889.7, -191.1, -889.7, 8000};
	double IL_KneeUp[9] = {33000, 196.2, -201.5, 196.2, 28000, 877.5, -201.5, 877.5, 8000};

	//double IR_FPad[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}; // ��disable
	//double IL_FPad[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}; // ��disable
	//double IR_KneeDown[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}; // ��disable
	//double IL_KneeDown[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}; // ��disable
	//double IR_KneeUp[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}; // ��disable
	//double IL_KneeUp[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}; // ��disable
	//// IB1  (�{�l�P�y)
	//double IB1[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}; // ��disable
	//// �ݳ�
	//double IB2[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}; // ��disable


	// IB1  (�{�l�P�y)
	double IB1[9] = {11000, -34.4, -70.96, -34.4, 11000, 445.7, -70.96, 445.7, 17000}; 
	// �ݳ�
	double IB2[9] = {184000, -1000, 1000, -1000, 201000, -4000, 1000, -4000, 173000};  

	double IR_Arm[9] = {42000, -18.45, -134.8, -18.45, 42000, 54.74, -134.8, 54.74, 2000};
	double IL_Arm[9] = {42000, -19.39, -136.3, -19.39, 42000, -76.87, -136.3, -76.87, 2000};	
	//double IR_Arm[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}; // ��disable ���u�A�G�N�ˤp
	//double IL_Arm[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}; // ��disable ���u�A�G�N�ˤp

	// �H�W�w��j10^6�� �ݦA��j1000��(�U��) ����ܦ� g.mm^2 (catia �̭��O kg.m^2 ���W10^9 �N�|�ܦ� g.mm^2)
	
	// ���Ntheta�]��0�A����DH���� �_��theta�ȡA�ӱ`����{��
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

	// ���Ntheta�]��0�A����DH���� �_��theta�ȡA�ӱ`����{��
	FKLLeg->DHConstruct();
	FKRLeg->DHConstruct();
	FKLArm->DHConstruct();
	FKRArm->DHConstruct();

	// ���X����x�}�åB�N��ʺD�q���ӱ��COM�y�ФU
	// �]���ҥѸy��base���U���XDH �ҥH�L�׬O���}�٬O�k�}support ��������y�ЬҬ�532
	////// IcLFoot = A_LL0(1:3,25:27)'*IL_FPad*A_LL0(1:3,25:27); // ��6�b Rn5
	GetRotPartRn(&FKLLeg->Rn[5],Rot_Part_Rn);
	MatMulAB(IL_FPad,3,3,Rot_Part_Rn,3,3,temp_compute);
	MatMulAtB(Rot_Part_Rn,3,3,temp_compute,3,3,IcLFoot);

	////// IcLShank = A_LL0(1:3,17:19)'*IL_KneeDown*A_LL0(1:3,17:19); // ��4�b Rn3
	GetRotPartRn(&FKLLeg->Rn[3],Rot_Part_Rn);
	MatMulAB(IL_KneeDown,3,3,Rot_Part_Rn,3,3,temp_compute);
	MatMulAtB(Rot_Part_Rn,3,3,temp_compute,3,3,IcLShank);

	//////IcLThigh =  A_LL0(1:3,13:15)'*IL_KneeUp*A_LL0(1:3,13:15); // ��3�b Rn2
	GetRotPartRn(&FKLLeg->Rn[2],Rot_Part_Rn);
	MatMulAB(IL_KneeUp,3,3,Rot_Part_Rn,3,3,temp_compute);
	MatMulAtB(Rot_Part_Rn,3,3,temp_compute,3,3,IcLThigh);

	////// IcRFoot = A_RL0(1:3,25:27)'*IR_FPad*A_RL0(1:3,25:27); // ��6�b Rn5
	GetRotPartRn(&FKRLeg->Rn[5],Rot_Part_Rn);
	MatMulAB(IR_FPad,3,3,Rot_Part_Rn,3,3,temp_compute);
	MatMulAtB(Rot_Part_Rn,3,3,temp_compute,3,3,IcRFoot);

	////// IcRShank = A_RL0(1:3,17:19)'*IR_Shank*A_RL0(1:3,17:19); // ��4�b Rn3
	GetRotPartRn(&FKRLeg->Rn[3],Rot_Part_Rn);
	MatMulAB(IR_KneeDown,3,3,Rot_Part_Rn,3,3,temp_compute);
	MatMulAtB(Rot_Part_Rn,3,3,temp_compute,3,3,IcRShank);

	////// IcRThigh = A_RL0(1:3,13:15)'*IR_Thigh*A_RL0(1:3,13:15); // ��3�b Rn2
	GetRotPartRn(&FKRLeg->Rn[2],Rot_Part_Rn);
	MatMulAB(IR_KneeUp,3,3,Rot_Part_Rn,3,3,temp_compute);
	MatMulAtB(Rot_Part_Rn,3,3,temp_compute,3,3,IcRThigh);

	// �b�W�b���D�q������ �q�q�����y�Ĥ@�b���y�� �]���W�b�����@�@�@���y�� �]�N�OFKLArm->Rn[0]
	// �y�� �ݳ� �����u�M�k���u�����P�@�ӧ��Шt�U �]�N�OLeg->Rn[0] �G�u�b�Ĥ@��(�y��)�����y��
	// �y��
	GetRotPartRn(&FKLArm->Rn[0],Rot_Part_Rn);
	MatMulAB(IB1,3,3,Rot_Part_Rn,3,3,temp_compute);
	MatMulAtB(Rot_Part_Rn,3,3,temp_compute,3,3,IcLowBody);
	
	// �ݳ�
	//GetRotPartRn(&FKRLeg->Rn[0],Rot_Part_Rn);
	MatMulAB(IB2,3,3,Rot_Part_Rn,3,3,temp_compute);
	MatMulAtB(Rot_Part_Rn,3,3,temp_compute,3,3,IcUpBody);

	//////% �㰦�k���u  
	//GetRotPartRn(&FKRLeg->Rn[0],Rot_Part_Rn);
	MatMulAB(IR_Arm,3,3,Rot_Part_Rn,3,3,temp_compute);
	MatMulAtB(Rot_Part_Rn,3,3,temp_compute,3,3,IcRArm);
	
	//////% �㰦�����u 
	//GetRotPartRn(&FKRLeg->Rn[0],Rot_Part_Rn);
	MatMulAB(IL_Arm,3,3,Rot_Part_Rn,3,3,temp_compute);
	MatMulAtB(Rot_Part_Rn,3,3,temp_compute,3,3,IcLArm);	
	// �H�W�p���k�PInitInertia�ۦP �u�O�s���a���ܦ�InerLc �p���@�� �Ҧ��쥻�b�@�ɤU�y�z���D�q ���ഫ���H�ӱ��COM�y�дy�z
	
	//Sigma I
	for(int i = 0 ; i < 9 ; i++)
		IcBody[i] = 0;
	MatAddAB(IcLowBody,IcBody , IcBody, 9);	
	MatAddAB(IcUpBody,IcBody , IcBody, 9);	
	MatAddAB(IcLArm,IcBody , IcBody, 9); 	
	MatAddAB(IcRArm,IcBody , IcBody, 9);
	
	// �A��j1000�� �ϳ���ܦ� g.mm^2 (catia �̭��O kg.m^2 ���W10^9 �N�|�ܦ� g.mm^2)
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

	// ���饭��b�p�� IcUpBody = Sigma I + Parallel
	double TempVector1[3];
	double TempVector2[9];
	double TempVector3[9];
	double E[9]={1,0,0,0,1,0,0,0,1};	

	// ����b-IcLowBody
	MatScalarMul(pv_stack+42, 3, -1.0, TempVector1); 
	MatAddAB(BodyRCOM, TempVector1, TempVector1, 3);	// R
	MatMulAB(TempVector1,1,3,TempVector1,3,1,TempVector2);	// R�ER
	MatScalarMul(E, 9, TempVector2, TempVector2);	// (R�ER)*E

	MatMulAB(TempVector1,3,1,TempVector1,1,3,TempVector3);	// R outerproduct R
	MatScalarMul(TempVector3, 9, -1.0, TempVector3);	// -(R outerproduct R)
	MatAddAB( TempVector2, TempVector3 , TempVector3 , 9);	// (R�ER) * E - (R outerproduct R)
	MatScalarMul(TempVector3, 9, 2796, TempVector3);	// m*((R�ER) * E - (R outerproduct R))

	MatAddAB(TempVector3, IcBody, IcBody, 9);

	// ����b-IcUpBody
	MatScalarMul(pv_stack+45, 3, -1.0, TempVector1); 
	MatAddAB(BodyRCOM, TempVector1 , TempVector1, 3);	// R
	MatMulAB(TempVector1,1,3,TempVector1,3,1,TempVector2);	// R�ER
	MatScalarMul(E, 9, TempVector2, TempVector2);	// (R�ER)*E

	MatMulAB(TempVector1,3,1,TempVector1,1,3,TempVector3);	// R outerproduct R
	MatScalarMul(TempVector3, 9, -1.0, TempVector3);	// -(R outerproduct R)
	MatAddAB( TempVector2, TempVector3 , TempVector3 , 9);	// (R�ER) * E - (R outerproduct R)
	MatScalarMul(TempVector3, 9, 22952, TempVector3);	// m*((R�ER) * E - (R outerproduct R))
	
	MatAddAB(TempVector3, IcBody, IcBody, 9);

	// ����b-IcLArm 
	MatScalarMul(LArmCOM, 3, -1.0, TempVector1); 
	MatAddAB(BodyRCOM,TempVector1 , TempVector1, 3);	// R
	MatMulAB(TempVector1,1,3,TempVector1,3,1,TempVector2);	// R�ER
	MatScalarMul(E, 9, TempVector2, TempVector2);	// R�ER*E

	MatMulAB(TempVector1,3,1,TempVector1,1,3,TempVector3);	// R outerproduct R
	MatScalarMul(TempVector3, 9, -1.0, TempVector3);	// R outerproduct R*-1
	MatAddAB( TempVector2, TempVector3 , TempVector3 , 9);	// R�ER * E - 1 * R outerproduct R
	MatScalarMul(TempVector3, 9, 2496, TempVector3);	// m*(R�ER * E - 1 * R outerproduct R)
	
	MatAddAB(TempVector3, IcBody, IcBody, 9);

	// ����b-IcRArm
	MatScalarMul(RArmCOM, 3, -1.0, TempVector1); 
	MatAddAB(BodyRCOM,TempVector1 , TempVector1, 3);	// R
	MatMulAB(TempVector1,1,3,TempVector1,3,1,TempVector2);	// R�ER
	MatScalarMul(E, 9, TempVector2, TempVector2);	// R�ER*E

	MatMulAB(TempVector1,3,1,TempVector1,1,3,TempVector3);	// R outerproduct R
	MatScalarMul(TempVector3, 9, -1.0, TempVector3);	// R outerproduct R*-1
	MatAddAB( TempVector2, TempVector3 , TempVector3 , 9);	// R�ER * E - 1 * R outerproduct R
	MatScalarMul(TempVector3, 9, 2496, TempVector3);	// m*(R�ER * E - 1 * R outerproduct R)
	
	MatAddAB(TempVector3, IcBody, IcBody, 9);
	
	// �p�⧹����_��Theta��
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

	//// �H�U���ե�

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
	
	Q1 =  0.08   ;       //0.8 //0.004       // Q��model��covariance �V�p�V�۫H
	R1 =   1.60311  ;       //0.7976  	// R��measuremant��covariance �V�p�V�۫H
	
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
     
			*result = x_est1; // �s�J���wbuffer��


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
//	// kalman filter ���ন�Z����
//		
//	******************************************************************/
//	 //���a cali�� �H�G�����^�k�u �w�g�NCALI�q�
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
	// �u�W���ȩάO���u�פJenc��r��
	// �A�ǥ�FindFK FindCOG�ӱo��DHmodel����COG��m
	// �b�o�̷|�Q�ΰѼưO�ФU�쥻���� (EX������m ����x�}...etc)
	// ���`�N�b�̫�n��t�έp��X��IK�ٵ��t��
	// �b�^��ENC�ɤ���sOpenGL
	// �禡��bIKStep�᭱ �ҥH�b���}�ɪ�count�ƭn�`�N
	// 20130911 WZ
	******************************************************************/
	double temp_theta[12];	// �Ȧs�z�Q����
	int	WriteIndex;

	// ���X�z�Q�����צ�m �åB��J�^�Ǫ�Enc�ਤ�� ��FK COG
	for (int h = 0 ; h < 6 ; h++){	
		temp_theta[h] = FKLLeg->theta[h+1];	// ���}
		FKLLeg->theta[h+1] = Enc_theta[h];
		temp_theta[h+6] = FKRLeg->theta[h+1];	// �k�}
		FKRLeg->theta[h+1] = Enc_theta[h+6];	
	}
	// forward kinematics
	FKLLeg->DHConstruct();
	FKRLeg->DHConstruct();
	FKLArm->DHConstruct();
	FKRArm->DHConstruct();

	// ���X�}���O����x�} �åB�N�㰦�����H����ܾA���V
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

	// ���৹����X�Ҧ��b�I�y��
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
	else{	// �Ĥ@����FK
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
		Enc_COG[i] = COG[i];	// Enc_COG��FK COG��X��ENCCOG
	}
	//////////////////////////////////////////////////////////////////////////
	//�U�@�B�n���}
	if (gIthIK != 0 && (gIthIK) % gStepSample == 0){	//�]����bIKStep�᭱gIthIK�w�g�[�L1�F �o�̥u�n��gIthIK�Y�i
		Enc_stepIndex += 1;	// �U�@�B��SUP���A
		if (selSupport[Enc_stepIndex] == LeftSupport){	//�U�@�B�O���}SUP CrdAll->data[21]���}�}���O
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

	//UpdateDrawingBuffer(); // ���}�ݨ�ENC ����H FindFK()��������ƭn����!

	//////////////////////////////////////////////////////////////////////////
	// �ٵ��t��FK COG ���z�Q���y��
	for (int h = 0 ; h < 6 ; h++){	
		FKLLeg->theta[h+1] = temp_theta[h];	// ���}	
		FKRLeg->theta[h+1] = temp_theta[h+6];	// �k�}		
	}
	
	FindFK();
	FindCOG();
	// �ٵ��t��FK COG ���z�Q���y��

}


void Kine::Distributor(double *ZMPxd, double *ZMPyd, double *StepX, double *StepY, int StepNum, int FKCount, int IKCount, int HomePos)
{
	/******************************************************************
	input: 
	output: void

	Note:	
	// DSPr = ZMP��V���ʦV�q
	// ZMP�p��ɨS���]�A�@�}�l��FK �n�A��homing���p�[�J�I��!!!!!!!!!!
	// StepNum = IK�I��
	// FKCount = �}�YFK�I��
	// swing�}���}�a���b1000(FK)+180+480+180-45(Nwait) �ä��O�bNab�~�}�l��}
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

	for (int m = 0 ; m < 8 ; m++){	// Q0�﨤�x�} �﨤�Ȭ�forceratio�}�ڸ�
		for (int n = 0 ; n < 8 ; n++){
			if(m == n){
				Q0[8*m+n] = 1;}
			else{
				Q0[8*m+n] = 0;}
		}
	}

	if (FlagStayMode == 1) // stay �Ҧ��A�����H�z���O�}���� �åB��������
	{
			HomeCount = 300;
	}
	else // ���`�Ҧ� ��jp�Pkp���t�y��count��
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
	
	if (FlagStayMode == 1 || selSupport[2] == DoubleSupport)	// ���|��IK
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
		for (int j = 1 ; j < FKCount+Nza+Nab ; j++)	// �}�l���O�s FK~IK[0~gNab]�T�w�O�s sure???????????????????????????
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
			//// ���tratio
			if (i == 0)	// �Ĥ@�B�S�O�B�z
			{	
				if (selSupport[i+1] == LeftSupport)	// �U�@�B�O���}SUP
				{
					for (int k = 0 ; k < 4 ; k++)
					{
						A[k] = CotPoint[k] + StepX[i+1];		A[4+k] = CotPoint[k+4] + StepX[i+1];
						A[8+k] = CotPoint[k+8] + StepY[i+1];	A[12+k] = CotPoint[k+12] - StepY[i+1];
					}
					Gen7DegPolyMod(0.5,1,225,forceratio);	
				}
				else if (selSupport[i+1] == RightSupport)	// �U�@�B�O�k�}SUP
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
				if (selSupport[i+1] == LeftSupport)	// �U�@�B�O���}
				{
					for (int k = 0 ; k < 4 ; k++)
					{
						A[k] = CotPoint[k] + StepX[i+1];		A[4+k] = CotPoint[k+4] + StepX[i];
						A[8+k] = CotPoint[k+8] + StepY[i+1];	A[12+k] = CotPoint[k+12] + StepY[i];
					}
					Gen7DegPolyMod(0,1,225,forceratio);
				} 
				else if (selSupport[i+1] == RightSupport)	// �U�@�B�O�k�}
				{
					for (int k = 0 ; k < 4 ; k++)
					{						
						A[k] = CotPoint[k] + StepX[i];		A[4+k] = CotPoint[k+4] + StepX[i+1];
						A[8+k] = CotPoint[k+8] + StepY[i];	A[12+k] = CotPoint[k+12] + StepY[i+1];
					}
					Gen7DegPolyMod(1,0,225,forceratio);
				}
				else if (selSupport[i+1] == DoubleSupport)	// �U�@�B�O�̫�@�B
				{
					end_step = i+1;
					if (selSupport[i] == LeftSupport)	// �o�@�B�O���}
					{
						for (int k = 0 ; k < 4 ; k++)
						{
							A[k] = CotPoint[k] + StepX[i];		A[4+k] = CotPoint[k+4] + StepX[i];
							A[8+k] = CotPoint[k+8] + StepY[i];	A[12+k] = CotPoint[k+12] - StepY[i];
						}
						Gen7DegPolyMod(1,0.5,180*2,F_DSPratio);
					}
					else if (selSupport[i] == RightSupport)	// �o�@�B�O�k�}
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
			//// ���tratio

			if (Recordfile == true)
			{			
				QQ.open("ratio.txt",ios::app);
				for(int p = 0 ; p < 225 ; p++){
					QQ << forceratio[p]<< "\t";
				}		
				QQ.close();
			}


			//// �}�l��pseudo inverse
			for (int j = i*gStepSample+Nza+Nab ; j < (i+1)*gStepSample+Nza+Nab ; j++)
			{	
				if (j < (end_step+1)*gStepSample-Nab-Nzb)	// ���F�̫�DSP��Nab�������w
				{
					ZMP[0] = ZMPyd[j];
					ZMP[1] = ZMPxd[j];
					if (j >= end_step*gStepSample - Nzb)	// final DSP
					{
						ratioL = 1/(2*F_DSPratio[j-(end_step*gStepSample - Nzb)]);	// L
						ratioR = 1/(2*(1-F_DSPratio[j-(end_step*gStepSample - Nzb)]));	// R

						for (int m = 0 ; m < 8 ; m++){	// Q0�﨤�x�} �﨤�Ȭ�forceratio�}�ڸ�
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

						for (int m = 0 ; m < 8 ; m++){	// Q0�﨤�x�} �﨤�Ȭ�forceratio�}�ڸ�
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
						if (selSupport[i+1] == LeftSupport)	// �U�@�B�O���}
						{
							for (int k = 0 ; k < 4 ; k++)
							{	
								A_SSP[k] = A[k];
								A_SSP[4+k] = A[8+k];
								A_SSP[8+k] = A[16+k];
							}	
						} 
						else if (selSupport[i+1] == RightSupport)	// �U�@�B�O�k�}
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

						if (selSupport[i+1] == LeftSupport)	// �U�@�B�O���}SUP
						{
							MatMulAB(pinv_SSP,4,3,force_temp,3,1,F_dis);
							F_dis[4] = 0;	F_dis[5] = 0;	F_dis[6] = 0;	F_dis[7] = 0;
						} 
						else if (selSupport[i+1] == RightSupport)	// �U�@�B�O�k�}SUP
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
			//// �}�l��pseudo inverse
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
