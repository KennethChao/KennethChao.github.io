/**************************************************************************************************
Copyright, 2010-2012, Robotics Lab., Dept. of M.E., National Taiwan University
File Name: Kine.h

Author: Jiu-Lou Yan
Version: 1.2
Date: 2010/07/20 => 2070/07/26 by Slongz & �l�h

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

#include "FwdKine.h"
#include "GlobalConst.h"
#include "QFwdKine.h"
#include "QuaternionRotation.h"
#include <fstream>

class Kine
{
public:
	Kine(void); // �غc�l
	~Kine(void); // �Ѻc�l

	void InitKineTrains(void); // ��l��DH parameters 
	void FindFK(void); // �p��Ҧ�forward kinematics
	void ComputeJacobians(void); // �p���Ӿ����H��Jacobian matirx
	// �p��swing trajectory
	void GenSwingTraj(double v0, double a0, double x1_percentage, double y1, double v1, double x2, double y2, double v2, double a2, int Np, int KeepPosCount, double* resultXY, double* resultZ);	
	// �p��swing trajectory	�ק睊 �ݰ_�ӧ󹳤H�S���|�l���ܦhí�w��
	void GenSwingTrajMod(double v0, double a0, double x1_percentage, double z1, double v1, double xy2, double z2, double v2, double a2, int Np, int KeepPosCount, double* resultXY, double* resultZ);
	// �Q�ΰ����h�����ͦ����ƪ�ZMP�y�� jerk ������w
	void GenSmoothZMPShift(double y_start, double y_end, int Np, double* result);
	// �Q�ΰ����h�����ͦ����ƪ�ZMP�y�� jerk = 0
	void GenSmoothZMPShift_ZeroJerk(double y_start, double y_end, int Np, double* result);
	// �Q�ΰ����h�����ͦ����ƪ�ZMP�y�� �Ҧ��Ѽƥi�H�ۥѫ��w
	void GenSmoothZMPShift_ZeroJerkshiftstair(double y_start, double y_end, int Np, double* result,int mode1);
	// �Q�ΰ����h�����ͦ����ƪ�ZMP�y�� �w�飼�ӱ�shift �Ҧ��Ѽƥi�H�ۥѫ��w ���a20121127	
	void GenZMPFreeAssign(double x2, double y0, double v0, double a0, double j0, double y2, double v2, double a2, double j2, int Np, double* result);
	// �ͥX�C���h����	
	void Gen7DegPoly(double x1, double y1, double y2, int Np, double* result);
	// �ͥX�E���h����
	void Gen9DegPoly(double x1, double y1, double y2, int Np, double* result);// StairMode = 1���}�Ҽӱ�α�Z��V�y���^��
	// �ͥX�W�U�ӱ�Z��V��swing�y�� �s����ӤE���h����
	void Gen9DegPolyStair(double x1, double y1, double y2, int Np, double* result);
	// �ͥX�Q���h����
	void Gen10DegPoly(double x1, double y1,double x2, double y2,double x3, double y3, double yend, int Np, double* result);
	// �ͦ����O���Ϊ��h�����y��
	void Gen7DegPolyMod(double y0, double y1, int Np, double* result);
	// �N���array 3��3�Ө̦�normalize�����׬��@
	void NormVecArray(double* VecArray, double* Norm, int Len);
	// �p��COG Jacobian
	void GetCOGJacobian(void);
	// �p��COG ��m
	void FindCOG(void);
	// �Ѥ@��IK
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
	void GetLegsCoords(void); // ��X�{�b���}����b�Ŷ��������Шt

	// For Inertia Matrix
	void InitInertia(void); // initialize

	void FindPseudoJ(void); // �p��pseudo inverse Jacobian matrix
	void SetIdentity(double* RotationMatrix); // �N�x�}�]��identity matrix
	void CheckJointLimit(void); // �ˬd�����H�O�_�F��joint limit
	void GetRotPartRn(YMatLite* HomoMat, double* RotPart); // ���Xhomogeneous matrix �� rotation part

	void UpdateDrawingBuffer(void); // ��sø�ϡA�u���bFK�����ɰ���

	void NumDiff(void);
	// ��}���U��Joint���װ��ƭȷL�� �o�U�b���t�׻P���[�t��
	
	//Dynamics
	void FindD(void);
	// ����Ҧ�Dynamic�t�� �p��XTorque
	void FindDIni(void);
	// ��l��Dynamic�Ѽ�
	void CalCoord(void);
	// �p��Ҧ��b�b�@�ɤ�����m�P�V�q, �U��󭫤ߦ�m
	void FDOmegaL(void);
	// �p��Link�b�@�ɤ������t��
	void FDVelJ(void);
	// �p��Joint�b�@�ɤ����t��
	void FDAlphaL(void);
	// �p��Link�b�@�ɤ������[�t��
	void FDAccelJ(void);
	// �p��Joint�b�@�ɤ����[�t��
	void FDVelCOM(void);
	// �p��U��󭫤ߦb�@�ɤ����t��
	void FDAccelCOM(void);
	// �p��U��󭫤ߦb�@�ɤ����[�t��
	void FDForceJ(void);
	// �p��U�b���O
	void FDTorqueJ(void);
	// �p��U�b���O�x
	void FindInertia2LocalCOM(void);
	// �H�U���y�Шt�y�z�ӱ����ʺD�q(�w��)
	void Friction(void);
	// �[�J�����O
	//Dynamics

	void ForceSensorData(int FlagSim, int FlagGo, int SensorCount, int LogCount, double *FS_DataL, double *FS_DataR);
	// �u�W���O�W�� �o�i �άO���u��txt
	void FindEncCOG(double* Enc_theta, double* Enc_COG);
	// �Q��Encoder Feedback�z�LFindFK FindCOG ��XCOG
	void Distributor(double *ZMPxd, double *ZMPyd, double *StepX, double *StepY, int StepNum, int FKCount, int IKCount, int HomePos);
	// ���tdesired force to preset contact point

		
	// infrared �����Ѽ�  20130419  ���a	
	//void  InfaredSensorData(double ADCL1, double ADCL2, double ADCL3 , double ADCL4, double ADCR1 , double ADCR2, double ADCR3  , double ADCR4  ) ;
	// �u�W�����~�u���o�i �άO���u��txt  Ū�J�U�}��ADC�ȱN���ഫ���Z��  0 (LLEG)  1 (RLEG) 
	void  kalmanfilter(double  datanow, double  datafilterbefore ,  double *result ); 
	// datanow (��Udata)     datafilterbefore(�o�L���e�@��data)    *result (�s�J���w�x�})	
	
	void smoothfilter(double  *data , int count );
	//�N�en�ӭȰ�����   ���o�i�ĪG   n��sampling time delay  	


	//# define infraedbuffersize 250000 //���~�u�һ�buffer�j�p
	//double infrared[infraedbuffersize];  //�s�U�Ҧ�infrared data
	//double infforce[infraedbuffersize];   
	//double deltaz[infraedbuffersize]; //�s�Udeltaz �Ω� landingtimecontrol 	
	//double deltazfilter[infraedbuffersize];
	
	//int infraredcount;      //infraredcount��       ���a2013/4/3
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

	////�ন�Z����
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

	////�Z��
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
	// infrared �����Ѽ�  20130419  ���a


	int LegDHLen; // �}��DH����
	int ArmDHLen; // �⪺DH����
	FwdKine* FKLLeg; // ���}Kinematics train
	FwdKine* FKRLeg; // �k�}Kinematics train
	FwdKine* FKLArm; // ����Kinematics train
	FwdKine* FKRArm; // �k��Kinematics train

	YMatLite* CrdAll;	// �x�s�Ҧ��b����m
	YMatLite* ZAxisAll; // �x�s�Ҧ�z�b��V
	YMatLite* Ja; // Jacobian matrix
	int JaMRow; // Jacobian matrix ��row��
	int JaNCol; // Jacobian matrix ��column��
	
	// �p��Jacobian �Ϊ� temp �Ȧs��
	double* tempJ;
	double* tempJT;
	double* tempJiWJT;
	double* tempInv;

	double* PseudoInv; // pseudo inverse matrix

	// �s��ƻs�AŪ���ɨϥΤ����и��X
	int ind_x; 
	int ind_y;
	int ind_z;
	int ind_source;
	int ind_dest;

	double* r_com;		// ���߶Z�����Z��
	double* mass_com;	// ����q

	int selIK; // ���k�}�伵���A

	double* pv_stack;	// ��󭫤ߩ�@�ɤ����y��
	double* temp_Norm;	// ���׭p��Ȧs�Ŷ�

	double COG[3]; // COG �N�s�b�o
	double BodyUpVec[3]; // ���V����V�W��V
	double BodyRightVec[3]; // ���V����V�k��V
	double BodyFaceVec[3]; // ���V���魱�V��V
	double DHOrigin[3]; // DH�����I�A�|�H�۾����H�樫�������
	double UpBodyUpVec[3];

	double SumMass; // �����H����

	// COG jacobian �p��μȦs�Ŷ�
	double temp_double_1;
	double temp_r_ef[3];
	double temp_cross[3];
	double temp_scale;

	//�l�hstart111221
	double temp_scale_LA;
	double temp_scale_RA;
	double local_COG_LA[3];
	double local_COG_RA[3];
	double temp_mass_sum_LA;
	double temp_mass_sum_RA;
	double temp_mass_sum_next_LA;
	double temp_mass_sum_next_RA;
	//�l�hend111221

	double temp_vec1[3];
	double temp_vec2[3];
	double temp_vec3[3];
	double temp_vec4[3];
	double local_COG[3];
	double temp_mass_sum;
	double temp_mass_sum_next;

	// Jacobian�p��� end-effector ����
	double* EndEffDiff;
	double EndEff[3];

	// IK �ѥX�� theta�t��
	double* dth;
	double* dx;

	double RobotFixPoint[3];	// �����H�����y�Эn�쪺�a��(���}�άO�k�}��end eff�n�쪺�a��)
	double RobotFixVector[3];	// �����H�����V�q

	double TempForCross[3];

	// swing�} �h�����M�ΰѼ�
	double p1[5]; // �|���h�����Y��
	double p2[6]; // �����h����
	double A1[4]; // ��}�A��p1��
	double A2[36]; // ��}�A��p2��
	double r1[2]; // boundary conditions
	double r2[6]; // boundary conditions
	double tempVal; // �p��Ȧs
	double SwingBufferx[10000]; // �s��U��swing trajectory 
	double SwingBuffery[10000]; // �s��U��swing trajectory  
	double SwingBufferz[10000]; // �s��U��swing trajectory  

	int Nab; // Nab = Na + Nb for SSP
	int Na; // SSP�e�q����
	int Nb; // SSP��q����
	int N_step; // ��B���
	double TempXp[4]; // �p��Ȧs
	double x_p; // xy�b��V�����Ϊ��I

	// �ɾ�����ѼơA�B��y��Ѽ�
	// DSP SSP�N��樫���AZMP���������}�伵�ɶ��ʤ���
	// DSP = 0.4; // double support phase 
	// SSP = 0.6 // single support phase
	// DSP + SSP = 1.0

	int Nza; // for DSP���ɹs�A�e�q
	int Nzb; // for DSP���ɹs�A��q

	// swing�} �h�����M�ΰѼ�	

	//�l�hstart111227
	double initRA[3];
	double initLA[3];
	double remLA[3];//����ĤE�b
	double remRA[3];//�k��ĤE�b
	double shiftLA[3];//����ĤQ�b
	double shiftRA[3];//�k��ĤQ�b
	
	//�l�hend111227

	double initCOG[3];	// ��lCOG��m
	double initLL[3];	// ��l���}��m
	double initRL[3];	// ��l�k�}��m
	double remLL[3];	// �O�����ɥ��}����m
	double remRL[3];	// �O�����ɥk�}����m
	double shiftLL[3];	// �p��}�T�w�b�Ŷ������V�q
	double shiftRL[3];	// �p��}�T�w�b�Ŷ������V�q
	//double dx[12]; // the input to the IK solver

	double SwErrLim;	// swing ��IK �~�t��
	double COGErrLim;	// COG ��IK �~�t��
	double AngleErrLim;	// �i����Angle error
	double SwingErr[3];	//  error of swing leg position in xyz 
	double COGErr[3];	// error of COG position in xyz
	//�l�hstart111227
	double LArmErr[3];	// in xyz
	double RArmErr[3];	// in xyz
	double dx_temp[6]; //�l�h���ե�dx

	double dthArmOffline[12];
	double dxArmOffline[12];
	double tempJArmOffline[12*12];
	//�l�hend111227


	double MaxSwingError; // maximum input position trajectory difference ������c�Ĥӧ�
	double MaxCOGError; //maximum input position trajectory difference ������c�Ĥӧ�
	double MaxRotationError; // maximum input angle trajectory difference ������c�Ĥӧ�
	double MaxJointAngleChange; // �̤j���ױ���q

	double selSupport[4000]; // �]�w���}��support phase
	double StepHeight[4000]; // �]�w�C�B�}�ﰪ�q
	//Slongz 20130425
	double SupportPhs[20000]; // �]�w���}��support phase	
	//Slongz
	int stepIndex; // �O��{�b�ĴX�B

	int cntIK; // �O��{�bIK�]�F�X��

	bool FirstFKFound; // �Ĥ@��IK�������X��
	int FlagSumoMode; // �O�_�����H�n���ɶ���}����
	int FlagStayMode; // �����H�O�_�n�i�Jstay mode
	bool FlagStaySquatMode;//�����H�O�_�n�i�Jstay squat mode
	bool FlagStayBreak;//�����H�i�g����LĲ�o����stay mode
	

	// WLN parameters
	double MaxLL[6]; // ���׷�����
	double MinLL[6]; // ���׷�����
	double MaxRL[6]; // ���׷�����
	double MinRL[6]; // ���׷�����
	
	double MaxLLBound[6]; // ���׷�����
	double MinLLBound[6]; // ���׷�����
	double MaxRLBound[6]; // ���׷�����
	double MinRLBound[6]; // ���׷�����

	// ���c�w���� �W�U�� 
	double JointUpLimitLL[6]; // ���}�W��
	double JointLoLimitLL[6]; // ���}�U��
	double JointUpLimitRL[6]; // �k�}�W��
	double JointLoLimitRL[6]; // �k�}�U��

	double A_PC_LL[6]; // pre calculated para A for LL
	double A_PC_RL[6]; // pre calculated para A for RL
	double B_PC_LL[6]; // pre calculated para B for LL
	double B_PC_RL[6]; // pre calculated para B for RL

	double invWLNMat[12];
	// Note: dth = inv(W)*J'*inv(J*inv(W)*J')*dx
	// inv(W) = invWLNMat[12]

	double LastInvWLN_Mat[12]; // �O��inverse WLN �x�}��
	double TempSquareVal; // �Ȧs�O����
	double TempTh; // �Ȧs�O����
	bool IfWLNEqualOne[12];  // �P�_WLN�ȬO���O1
	// WLN

	// Variables for Singularity Avoidance 
	double* work_clapack;  // ��CLAPACK�Ϊ��O����
	long* ipiv_clapack; // ��CLAPACK�Ϊ��O����
	double SingularJudge; // ���S��singular�X��
	double SingularAdd; // singular �n�[�W���Ʀr

	double* tempJJT_for_inv; // �Ȧs�O���� 

	// Variables for Euler Angle
	double TarRotMSw[9]; // swing�}���ؼб���x�}
	double TarRotMFx[9]; // fixed�}���ؼб���x�} 
	double DiffRotMatSw[9]; // difference between two rotation matrices (�`�N�t�Z�n�p �~��u�ʤ�)
	double DiffRotMatFx[9]; // difference between two rotation matrices (�`�N�t�Z�n�p �~��u�ʤ�)
	double DiffRotMatBody[9]; // difference between two rotation matrices (�`�N�t�Z�n�p �~��u�ʤ�)

	
	//20121214doratom//
	//pitch
	double TarRotMSwPitch[9]; // swing�}���ؼб���x�}
	double TarRotMFxPitch[9]; // fixed�}���ؼб���x�} 
	double DiffRotMatSwPitch[9]; // difference between two rotation matrices (�`�N�t�Z�n�p �~��u�ʤ�)
	double DiffRotMatFxPitch[9]; // difference between two rotation matrices (�`�N�t�Z�n�p �~��u�ʤ�)
	double DiffRotMatBodyPitch[9]; // difference between two rotation matrices (�`�N�t�Z�n�p �~��u�ʤ�)
	//20121214doratom//

	//�l�hstart120222
	double DiffRotMatLA[9]; // difference between two rotation matrices (�`�N�t�Z�n�p �~��u�ʤ�)
	double DiffRotMatRA[9]; // difference between two rotation matrices (�`�N�t�Z�n�p �~��u�ʤ�)
	//�l�hend120222

	double TarRotMBody[9]; // body���ؼб���x�}
	double LLegRotM[9]; // ���}������x�}
	double RLegRotM[9]; // �k�}������x�}
	double BodyRotM[9]; // ���骺����x�}

	//�l�hstart111227
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
	//�l�hend111227

	double LenEdgeXYZ[3]; // �o�O�}���O�����׸�ơA�b�غc�l�̭��Q�]�w

	double LSwitchRMot[16]; // �O�����}�ɸ}���O����x�} ���}
	double RSwitchRMot[16]; // �O�����}�ɸ}���O����x�} �k�}
	double Enc_LSwitchRMot[16]; // �O��Enc Feedback���}�ɸ}���O����x�} ���}
	double Enc_RSwitchRMot[16]; // �O��Enc Feedback���}�ɸ}���O����x�} �k�}
	double temp44MatMul[16]; // 4x4�x�}�ۭ��Ȧs��

	double FixEndEffRMot[16]; // ��⧹FK�ɭԪ�Fix Leg ���Шt

	float* ArmDrawingBuffer;
	float* LegDrawingBuffer;
	int IndexBuf;
	int	IndexBufRev;

	// variables for inertia matrix and computation

	// ��� �C��element���W1000�H��A���O (g x mm^2)
	//double IR_FPad[9];// = {6000, -550.1, 2000, -550.1, 9000, 98.94, 2000, 98.94, 7000}; 
	//IR_FPad =   [6000    -550.1  2000 ;
    //             -550.1  9000    98.94;
    //             2000    98.94   7000]; % �}���O A_RL0 - 6 
	// IR_FPad = �@�ɤ�����ʺD�q   IR_Foot = A_RL0(1:3,25:27)'*IR_FPad*A_RL0(1:3,25:27);
	double IR_Foot[9];

	//double IL_FPad[9]; 
	//IL_FPad =   [6000   546    2000 ;
    //             546    9000   -97.5;
    //             2000   -97.5  7000]; % �}���O A_LL0 - 6
	// IL_FPad = �@�ɤ�����ʺD�q IL_Foot = A_LL0(1:3,25:27)'*IL_FPad*A_LL0(1:3,25:27);
	double IL_Foot[9];
	
	//double IR_KneeDown[9];
	//IR_KneeDown = [17000  106.1  -383 ;
	//				 106.1 16000   1000 ;
	//			    -383   1000   3000] ; % �}��s�����\Yaw���F���v�r�� A_RL0 - 4
	// IR_Shank = A_RL0(1:3,17:19)'*IR_KneeDown*A_RL0(1:3,17:19);
	double IR_Shank[9];


	//double IL_KneeDown[9];
	//IL_KneeDown = [17000 -132.1   -450.3 ;
	//               -132.1 16000   -1000 ;
	//               -450.3  -1000    3000]; % �}��s�����\Yaw���F���v�r��  A_LL0 - 4
	//IL_Shank = A_LL0(1:3,17:19)'*IL_KneeDown*A_LL0(1:3,17:19);
	double IL_Shank[9];

	//double IR_KneeUp[9];
	//IR_KneeUp   = [33000 -200.2  -191.1;
	//              -200.2 28000  -889.7;
	//              -191.1 -889.7  8000]; % ���\�H�γs���b���`�����  A_RL0 - 2
	//IR_Thigh = A_RL0(1:3,13:15)'*IR_KneeUp*A_RL0(1:3,13:15);
	double IR_Thigh[9];

	//double IL_KneeUp[9];
	//IL_KneeUp   = [33000  196.2  -201.5;
	//			  196.2  28000  877.5;
	//			  -201.5 877.5  8000]; % ���\�H�γs���b���`����� A_LL0 - 2       
	//IL_Thigh =  A_LL0(1:3,13:15)'*IL_KneeUp*A_LL0(1:3,13:15); % �`�N�AA_LL0����16 column�O���\����m�A���ɳo�Ӯy�Шt�����I�O���\�Axyz�b�O�j�L��V
	double IL_Thigh[9];


	// �o�ӭ�n�N�b�@�ɤU
	//IB1 = [76000   75    -10000;
	//	   75   55000    71;
	//	   -10000  71    57000];
	//% ��ӤU�b����  
	//% Block �y�U�b��
	//% �ѩ���w�]����A�G�����N�O�@�ɭ�       
	double IB1[9]; 

	//% ���馳�઺�a��   
	//double IBody[9];
	//IBody       = [71000 84   94;
	//               84   87000 3000;
	//               94    3000  85000]; % �߷F -- A_LA, A_RA -- R2 for roll,  R1�O pitch                          
    // IB2 = A_RA0(1:3,9:11)'*IBody*A_RA0(1:3,9:11);   % �y���y�Шt���Ө��y����H��|����v�T���ҥH��9~11�A�`�N�AA_R0����12 column�O�k���u�ӻH����m
	double IB2[9];


	//% �㰦�k���u 
	//double IR_Arm[9];
	//IR_Arm      = [42000   -18.45   -134.8;
	//               -18.45   42000    54.74;
	//               -134.8   54.74   2000]; % ���u �e���\�ʤ�V A_RA -- 4
	//           
	//IR_Arm = eye(3); % �G�N���ܤp��    
	//IRA = A_RA0(1:3,13:15)'*IR_Arm*A_RA0(1:3,13:15);        
	double IRA[9];


	//% �㰦�����u 
	//double IL_Arm[9];
	// IL_Arm      = [42000   -19.39   -136.3;
	//               -19.39    42000   -76.87;
	//               -136.3   -76.87   2000]; % ���u �e���\�ʤ�V A_LA -- 4
	//IL_Arm = eye(3); % �G�N���ܤp��   
	//ILA = A_LA0(1:3,13:15)'*IL_Arm*A_LA0(1:3,13:15);
	double ILA[9];
              

	////double IL_Foot_w[9]; // �@�ɤ����A�|�Y�ɧ�s
	////double IR_Foot_w[9]; // �@�ɤ����A�|�Y�ɧ�s
	////double IL_Shank_w[9]; // �@�ɤ����A�|�Y�ɧ�s
	////double IR_Shank_w[9]; // �@�ɤ����A�|�Y�ɧ�s
	////double IL_Thigh_w[9]; // �@�ɤ����A�|�Y�ɧ�s
	////double IR_Thigh_w[9]; // �@�ɤ����A�|�Y�ɧ�s
	////double IB1_w[9]; // �@�ɤ����A�|�Y�ɧ�s
	////double IB2_w[9]; // �@�ɤ����A�|�Y�ɧ�s
	////double ILA_w[9]; // �@�ɤ����A�|�Y�ɧ�s
	////double IRA_w[9]; // �@�ɤ����A�|�Y�ɧ�s


	double IR_Foot_sup_L[9]; // �̷�support�}���@�� inertia����l�I�]���P
	double IL_Foot_sup_L[9];
	double IR_Shank_sup_L[9];
	double IL_Shank_sup_L[9];
	double IR_Thigh_sup_L[9];
	double IL_Thigh_sup_L[9];
	//double IB1_sup_L[9]; 
	//double IB2_sup_L[9];     
	//double IRA_sup_L[9];
	//double ILA_sup_L[9];

	double IR_Foot_sup_R[9]; // �̷�support�}���@�� inertia����l�I�]���P
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
	QFwdKine* QFKLLeg; //QFk Object: ���}
	QFwdKine* QFKRLeg; //QFk Object: �k�}

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


	int CCDcntIK; // �O��{�bIK�]�F�X��
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
	void ComputeFixJacobians(void); // �p������H��Jacobian matirx (fixed leg only)
	void GetCOGFixJacobian(void); // �p������H��COG Jacobian matirx (fixed leg only)
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
	double ThetaDD[12];	// �}12�b�����[�t��
	double ThetaD[12];	// �}12�b�����t��
	double TempTheta[12];
	double TotalCOMLog[15]; // Numerical Buffer for COG differential
	//double FootPosDDL[3];	// �}���O���[�t��(�w����) �]���ڭ̸}�B�W�������Y �bsupport�ɨ����}�@�w�[�t�׬�0
	//double FootPosDDR[3];
	double TotalFootLogL[15];
	double TotalFootLogR[15];

	//Dynamics
	int AxisJump; // Jump�]��support�}���P �p�⴫�}��Index�ݸ��� AxisJump = 18
	int DHJump;	 
	int StackJump;

	double Rstack[18];	//Linkage length of RCOM2Joint
	double VectorTemp1[36];
	double VectorTemp2[36];
	double rCOMccw[18];	// �f�ɰw��V�U�b���󭫤ߦ�m�V�q
	double rCOMcw[18];	// ���ɰw��V�U�b���󭫤ߦ�m�V�q
	double GravityZaxi[3];	// ���O�b�@�ɮy�ФU�����V(0,0,-1)
	double BodyW;	// �����`��
	double LArmW;	// �����u�`��
	double RArmW;	// �k���u�`��
	double LArmCOM[3];	// ����COM��m
	double RArmCOM[3];	// �k��COM��m
	double AdamsFS[12];	// �sAdams�����O�W����
	fstream KineFile;	// Ū�O�W�Ȫ�txt
	//fstream AdamsMotionFile;	// AdamsFS�q�쪺MotionT //20130115 ������JAdams�q�쪺MotionTorque (����)
	double SensorOffset[3];	// �����O�W��m �쥻�O�Wr�O�b�}���O ���W��60
	//double AdamsMotionT[12];	// �NAdams�q�쪺MotionT�s�U //20130115 ������JAdams�q�쪺MotionTorque (����)

	//Joint Kine
		double LocalOmegaJ[36]; //theta * ��bz
		double OmegaL[36];	//�U���b�@�ɤ����t��
		double AlphaL[36];	//�U���b�@�ɤ����[�t��
		double LocalVelJ[36];	//�U�b��LocalVel
		double VelJ[36];	// �U�b�b�@�ɤ����t��
		double AccelJ[36];	// �U�b�b�@�ɤ����[�t��
	//Link COM Kine
		double BodyRCOM[3]; // �W�b��COM��m
		double LocalVelCOM[36]; // �H�U���y�Шt�y�z�ӱ��COM���t��
		double VelCOM[36];	// �U���b�@�ɤ����t��
		double AccelCOM[36];	// �U���b�@�ɤ����[�t��
		
		/////�H�UTest���ܼ�
		double VelTotalCOM[3];   //Vel   of COG
		double AccelTotalCOM[3]; //Accel of COG
		double CP[3]; 
		double ForceTotalCOM[3];
		/////�H�WTest���ܼ�

	//Joint Dyna
		double FSensor_forcL[3] ;	// ���}�O�W�q����(Force)
		double FSensor_forcR[3] ;	// �k�}�O�W�q����(Force)
		double FSensor_TorqL[3] ;	// ���}�O�W�q����(Torque)
		double FSensor_TorqR[3] ;	// �k�}�O�W�q����(Torque)
		double ForceJ[36];	// �U�b�b�@�ɤ����O
		double TorqueJ[36];	// �U�b�b�@�ɤ����O�x
		double MotorTorq[12];	// ���F�X�O (�z�Q)
		double RatedTorque[12];	// EPOS3 Torque Offset�����
		double GearRatio[12];	// �U�b��t�� (�ֱa��+HD)
		double FrictionJ[24];	// �U�b�����O
		double MotorTorqF[12];	// ���F�X�O (�[�J�����O)
		double AdamsTorque[12];	
		double CoPL[SensorBufferSize];
		double CoPR[SensorBufferSize];
		double FS_ZMP[SensorBufferSize];
		double ForceDataKFL[SensorBufferSize*10];	// 20130405WZ
		double ForceDataKFR[SensorBufferSize*10];	// 20130405WZ
	// Infrared data
		double InfaredSensor_dataL[5]; //���}���~�u�q����
		double InfaredSensor_dataR[4]; //�k�}���~�u�q����  (���a20130329)



	//////// Inertia (Corrdinate about Local COM: Lc ) //////
	//  Swing Linkage
		double IcRFoot[9];	// �H�k�}�}���OCOM�y�дy�z�k�}�}���O��ʺD�q
		double IcLFoot[9];	// �H���}�}���OCOM�y�дy�z���}�}���O��ʺD�q
		double IcRShank[9];	// �H�k�}�p�LCOM�y�дy�z�k�}�p�L��ʺD�q
		double IcLShank[9];	// �H���}�p�LCOM�y�дy�z���}�p�L��ʺD�q
		double IcRThigh[9];	// �H�k�}�j�LCOM�y�дy�z�k�}�j�L��ʺD�q
		double IcLThigh[9];	// �H���}�j�LCOM�y�дy�z���}�j�L��ʺD�q
			
		double IcUpBody[9];	// �H�ݳ�COM�y�дy�z�ݳ���ʺD�q
		double IcLowBody[9];	// �H�y��COM�y�дy�z�y����ʺD�q
		double IcRArm[9];	// �H�k��COM�y�дy�z�k����ʺD�q
		double IcLArm[9];	// �H����COM�y�дy�z������ʺD�q
		double IcBody[9];	// �H�W�b��COM�y�дy�z�W�b����ʺD�q

	//Others
		int GLindex;		 //Flag for Kine Checking (Initialing Point
		double RP[18];		 //Variable for Kine Checking
		double LP[18];		 //Variable for Kine Checking
		double VelJOld[36];  //Variable for ��οn��(not used)
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
	 // Kalman Filter For Infaredsensor  ���a20130329
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

	// �Q��ENC FK FindCOG�o�쪺COG��m �H�U���ϥ��ܼ�
		long COG_ENCload[12*16632];	// �s�Uload�i�Ӫ�ENC
		int OfflineNum;	// ���u�y��ƥ�
		//double EncCOGDD[12];	// �}12�b�����[�t��
		//double EncCOGLog[15];
		//double RobotAllCOGDD[3];
		double Enc_shiftLL[3];	// �p��}�T�w�b�Ŷ������V�q
		double Enc_shiftRL[3];	// �p��}�T�w�b�Ŷ������V�q
		bool FirstEncCOG;
		int Enc_stepIndex;
		// Wei-Zh 130908
	// Ankel Strategy
		double F_total[2*20000];
		double T_total[4*20000];

};
