/**************************************************************************************************
Copyright, 2010-2012, Robotics Lab., Dept. of M.E., National Taiwan University
File Name: GlobalConst.h

Author: Jiu-Lou Yan
Version: 1.5
Date: 2012/02/01

Functions:None

Classes: None

Description:
     �����Y�ɩw�q�F����B�檺��¦���z�q

Note: None
***************************************************************************************************/

// �����P�������
// �B�����
#define GravityConst	9810 // ���O�`��
#define dt				0.005 // control sampling time
#define dtArm           0.005
#define dtUSB2C32       0.02
#define PI				3.1415926

#define T_P				6.0 // �C�B�Ҫ�ɶ�
#define StepSam			(T_P/dt) // �C�B��sample��

// �ɾ�����ѼơA�B��y��Ѽ�
// DSP SSP�N��樫���AZMP���������}�伵�ɶ��ʤ���
// DSP = 0.4; // double support phase
// SSP = 0.6 // single support phase
// DSP + SSP = 1.0
#define DSP  0.3
#define	SSP  0.7

#define LQSIBufferSize 120000 // ���FLQSI����B�⪺�w�d�O�������
#define SensorBufferSize 180000 // �Ҧ�sensor feedback buffer size

// �]�w Timer�ӷ�
enum TimerSource{MControl,Thread}; //Mcontrol: �ϥ�Motion Control Thread�p�ɡA Thread: �t�}�@��Timer�M�� Thread�p��(�է@)



// �����H���V���ʺX�Щw�q
enum SideWalkDirection{DirectionLeft, DirectionRight};
// ZeroHome �^�k�������� BentHome 1 �^�k�ɦ^�k���ۦn ShiftZeroHome 2 // �u�������H�ۧڤ��з|�� �����N���ߩ��Ჾ�@�I SlopeHome 3 �שY�M�� �@����̫�@��
enum HomeType{ZeroHome, BentHome, ShiftZeroHome, SlopeHome};
// �����H����@���ɪ����A��
enum HomeSteps{HomeInitial, Homing, HomeFinished};
// �����Hsupport�}���y�z
enum SupportPhases{LeftSupport, RightSupport, DoubleSupport};
// �����H�ާ@�Ҧ� �]�t C++����, ADAMS����, �u�����
enum RobotExeMode{CppSimu, ADAMSSimu, RealExp};
// �]�w�P��Ϊ̸T��
enum EnableFlags{DisableFlag, EnableFlag};

// �]�w gPreProcessData �ѼơA�o�O�ΨӸ�����H���q�n�]�w�����H�ѼƥΪ�function
enum RobotCommunication{SetHome,SetOnePID,Manual,SetIdle,Reserved1,Traj,SetAllPID,SetPWMLims,SetCali,SetEncReadFlag};

//===================================
// Others
//===================================
//Plot and Data 
#define CheckingMode 0 //1: �s�U�b�y��(����) ���y���u&���ZMP COG�y�� & ��u 0: �\������

//ZMP Feedback
//#define ZMPFeedbackMode 0 //1: Matlab�q����ZMP state feedback�^LQSI 0: �\������ (Warning: May possibly failed in current version)

//Knee-stretch Motion
#define SupportCCD 0 // =1: Support�}��IKCCDSolve��IK
#define QCCDMode 0 // =1: IK Solver ������QCCD Mode
#define QCCDSwAngle 10 // (Degree) for IK-solver swithcing
#define FootpadRotation 0// =0: ���� =1: �Ұ�Toe-off and Heel-contact Motions 
#define ConstantCOGMode 1// =0: Flu =1 Constant COG
#define ErrorRatio 0.05// QCCD Error Bound Modification

//Save Data
#define WriteInitZMPtxt 0 // �NInitial�ɭԪ��W���n��ZMP �H��COG�g�Jtxt file
#define SaveSwingTraj 1 // �]�wKine���󤧤U GenSwingTraj() GenSwingTrajMod() �n���n�g�J��r�ɮ׳��� (�\�ʸ}�B�ʭy��)
#define SaveLQSIStates 0 // �]�wLQSISolver���󤧤U DummyControl() �n���n�g�J��r�ɮ׳��� (�˳��\�p�⵲�G)

//Simulation on Adams
#define AdamsSMode 1 // =0: �аt�X�l�hMatlab�{�� =1: �аt�XSlongz Matlab�{��

//Arm DH Settings
#define ArmDH 1 //		1:�j��(1st Demo�θ}�չw�]���u��l��V�]�w)		0: �t��(��y�y����u��l��V�]�w)

//Stair-Climbing
//#define StairClimbing 1 //	 //0516 Modified: �����w�q�κA (�ŧi�bRobotAllDlg.cpp)

//===================================
// Sensors
//===================================
#define LaserCatch 0 //1: �}�ҹp�g 0: ����
#define PMSUpdate 1	//�O�_�}��PMS�e�� 

//#define TwinCAT_Mode 1	��m����:MainLoops.h
//�������� �i�H���u���� �����PTwinCAT���q 
//���`�N�ݭn�� ����`��-RobotAll(�k��)-�ݩ�-�s����-��J-��L�̩ۨ� �R��TCatIoDrv.lib �~�i�H���u����

#define RunDynamics 0	// �}�ҰʤO�ǭp�� = 1
#define OfflineTraj 0	// ���u�y�񪩥�

//===================================
// Controllers
//===================================
#define BangBangControl 0; //	1: �}��Sliding Mode ���v��     0:����
#define PreloadTorque 0;
#define ILC_UpStair 0;//ILC up offset buf1
#define ILC_DownStair 0;// buf2

#define SoundCtrl 0;// 0:�����Ĥ@�q�e�m�y��

////���a20140218
#define uprightfeedback 0  //�}�ҤW�����A���v = 1 
#define cogestimate 0	// �}�Ҧ���COG (FK_COG) = 1 


///////// Define ��z�W��

//Define�Ӧh���e����z..

//���n���إB���|�]�Ҧ��ΰѳN���������ݩʪ̥�
//���n���ؽЧ令flag(�pExp Simu ADAMS ���۰ʤ������P�Ҧ���), ��K�L�թΦbUI�w�q�ק�
//
//�ɶq�令�`�n�Ҧ� (�b�P�@��mode�ɶq�������}�Υ���)
/////  Debug or �ˬd�֥Ϊ̺ɶq���ϥ�define�Ӷ}�}��
//�Фj�a�O���}�n�ߺD�A�۩w�q���\��}���H���v�T��L�ϥΪ̩έ쥻�{���B�椧�]�w���D