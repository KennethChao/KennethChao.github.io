/**************************************************************************************************
Copyright, 2010-2012, Robotics Lab., Dept. of M.E., National Taiwan University
File Name: RobotAllDlg.h

Author: Many People
Version: 1.0
Date: 2012/03/20

Functions:
      gIKStep() gThreadTest() gInitTurnLeft()
      gInitWalkStraight() gInitStepHere()
      gInitStair() gInitSquat()

	  �Ҧ����s
	  OnBnClickedButton1(); // Start Button
	  OnBnClickedButton2(); // Auto Button
	  OnBnClickedButton3(); // Go Button
	  OnBnClickedOk(); // OK ���s �����{����
	  OnBnClickedCancel(); // �����s���ϥ� �`�٪Ŷ�
	  OnBnClickedButton4(); // �Q��X�� Auto button �� �|�۰ʳQ���U
	  OnBnClickedButton5(); // �Q��X�� Auto button �� �|�۰ʳQ���U
	  OnCbnSelchangeCombo1(); // �]�w��ʼҦ��� �n������H�i�檺�ʧ@
	  OnBnClickedButton6(); // ���s "Send" ��ʼҦ��M��
	  OnBnClickedButton7(); // ���s PMS/BMS
	  OnBnClickedButton8(); // ���s Init_PBMS
	  OnBnClickedButton9(); // ���s PMS/BMS Save
	  OnBnClickedCheck1(); // ���� ���� ADAMS ���� �T�ؼҦ�
	  OnBnClickedCheck2(); // ���� ���� ADAMS ���� �T�ؼҦ�
	  OnBnClickedCheck3(); // ���� ���� ADAMS ���� �T�ؼҦ�
	  OnBnClickedCheck4(); // Checkbox --> Read Encoder
	  OnBnClickedCheck5(); // Checkbox --> Read 6 axis force sensor
	  OnBnClickedCheck6(); // �O�_ø�sZMP����p�y
	  OnBnClickedCheck8(); // Checkbox --> Speech�]�w�n���n�}�һy���Ҳ�(�ثe�S�Ψ�)
	  OnBnClickedCheck7(); // Checkbox --> Read Infrared (old: Skin Module)
	  OnBnClickedCheck9(); // Checkbox --> Boost
	  OnBnClickedCheck10();// Checkbox --> IMU
	  OnBnClickedCheck11();// Checkbox --> 3D Renderings
	  OnBnClickedCheck12();// Checkbox --> Manual Mode
	  OnBnClickedCheck13();// Checkbox --> 3D Model Rendering Mode
	  OnBnClickedButton10();// ���s Biped Control Emergent Stop
	  OnBnClickedCheck14();// Checkbox --> Arm Control Included
	  OnBnClickedCheck15();// Checkbox --> Hand Control Included
	  OnBnClickedButton11();// ���s Epos3 Fault Reset

Classes: None

Description:
     ���{���O��ӵ{����GUI����֤ߡA�Ҧ����s�P�ƥ�
	 �Ҧ��H���������ѳo�ӵ{���ӱ���
	 �U�禡�P�ܼƤ������иԨ��U��ŧi�P�w�q�B������

Note: None
***************************************************************************************************/

#pragma once
#include "./OpenGL/OpenGLControl.h"
#include "afxcmn.h"
#include "DataProcess.h"
#include "fstream"
#include "iostream"
#include "serial_port.h"
#include "afxwin.h"
#include  "Sensor\laser.h"
#include "afxmt.h"
#if TwinCAT_Mode
	#include "TCatIoApi.h" 	// header file shipped with TwinCAT I/O 

	#ifndef TASK_RLEG_H
	#define TASK_RLEG_H 
	#include "Task_RLeg.h"
	#endif    

	#ifndef TASK_LLEG_H
	#define TASK_LLEG_H 
	#include "Task_LLeg.h"
	#endif
#endif

#ifndef SD_H
#define SD_H
#include "SlidingModeCtrl.h"
#endif

#ifndef IMU_H
#define IMU_H
#include "IMU.h"
#endif

#ifndef ESTIMATE_H
#define ESTIMATE_H
#include "estimate.h"
#endif


using namespace std;

void gIKStep(void); // IK �Ѥ@��
void gThreadTest(void); // ���� sleep�ɶ���ǩ�

// ��l�ƦU�ذʧ@
void gInitTurnLeft(void); // ���F���s�ʧ@��l��
void gInitTurnRight(int StepNum, double TurnRadius, double TurnAngle, double Dist2L); // ���F���s�ʧ@��l��
void gInitWalkStraight(int StepInput, double StepLength); // ���F�����ʧ@��l��
void gInitStepHere(void); // ���F��a��B�ʧ@��l��
void gInitStair(void); // ���F�ӱ�ʧ@��l��
//���a20121129
void gInitdownStair(void); // ���F�U�ӱ�ʧ@��l��
void gIntLEDFace (int mode) ; //control  LEDFace 

//���a20121129
//Slongz 20130516
void gInitStairMod(int StepNum, double Dist2L, double StairHeight);   // Demo 2nd  �@���@�B�Ҧ�(�U�ӱ��)
void gInitUpStairMod(int StepNum, double Dist2L, double StairHeight); // Demo 2nd  �@����B�Ҧ�(�W�ӱ��)
//Slongz 20130516

void gInitSquat(void); // ���F�ۯ��ʧ@��l��
void gInitStayMode(double StayTime); // �����H���F�n��a�ۧڤ��Ъ��Ҧ�
void gInitSideWalk(int Direction); // ���V�樫 �i��ܥ��k
void gInitSumoMotion(void); // ��_��} �����ۼ��ʧ@

//�l�hstart20120309
void gInitArmWaveWalkStraight(void);		//�����u�ɤⳡ�y���l��
void gInitArmPushCartWalkStraight(void);	//���F��ɤⳡ�y���l��
void gInitRArmCarryWalkStraight(void);		//�Υk�ⴣ�F��ɤⳡ�y���l��
void gInitArmWaveTurn(int Direction);				//���s�ɤⳡ�y���l��
void gInitArmSideWalkWave(int Direction);//���V�樫
void gInitArmSumoMotion(void); // ��}���߼Ҧ� ���u�B�ʩw�q
void gInitArmIntroductionMode(double Time,int Mode,int Count);//�ۧڤ��Х�
//�l�hend20120309

//Slongz20130416
void gInitArmManualWalkStraight(int StepInput);// �pDemo  ���u�\�ʼҦ�
//Slongz start20120531
void gInitQCCDWalkStraight(int StepInput, double StepLength); //�����u�}�Ԫ��y���l��(�ݤ��)
//Slongz end 20120531

void gEncFeedback(void);	// �Q��Encoder IMU����COG error 20130522 WZ
void gSendOfflineTraj(void);	// �������J���uENC�y�� 20140224 WZ
void gSpeakSignLanguage(int IndexCount);//20130525 CCC
void uprightcontrol(float *IMUroll,float *IMUpitch , double *Ldeltatheta, double *Rdeltatheta , int count );	//add by ���a 20140218����W������


// CRobotAllDlg dialog
class CRobotAllDlg : public CDialogEx
{
// Construction
public:
	CRobotAllDlg(CWnd* pParent = NULL);	// standard constructor

// Dialog Data
	enum { IDD = IDD_ROBOTALL_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support

// Implementation
protected:
	HICON m_hIcon;

	// Generated message map functions
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()

	public:
		COpenGLControl openGLControl; // MFC GL����ŧi
	public:
		CRect rect; // MFC GL����e���j�p

	public:
		CButton mButton_Start; // Start ���s����ŧi
	public:
		CButton mButton_Auto; // Auto ���s����ŧi
	public:
		CButton mButton_Go; // Go ���s����ŧi
	public:
		CButton mButton_Init_PBMS;// Init_PMS ���s����ŧi
	public:
		CButton mButton_PMSBMS;// PMS/BMS ���s����ŧi
	public:
		int SendIndex; // sending data mode index


public:
	float m_scale;;
public:
	CComboBox SendMode;// drop list
public:
	CEdit EDIT1; // �U�誺��r��
	CButton CheckSimu; // ���s����w�q
	CButton CheckExp; // ���s����w�q
	CButton CheckADAMS; // ���s����w�q
	CButton CheckEncoder; // ���s����w�q
	CButton CheckForceSensor; // ���s����w�q
	CButton CheckZMPPlot; // ���s����w�q
	CButton CheckInfrared; // ���s����w�q
	CButton CheckSpeech; // ���s����w�q
	CButton CheckBoost; // �M�w�O�_CPU���t��IK
	CButton CheckIMU; // �O�_Ū��IMU
	CButton CheckOpenGL;// �O�_ø�sOpenGL model
	CButton CheckManualMode;// ���s����w�q
	CButton CheckPE;// ���s����w�q
	CButton CheckEmergentStop; // ���s����w�q
	CButton CheckArmCtrl; // ���s����w�q
	CButton CheckHandCtrl; // ���s����w�q
public:
	afx_msg void OnBnClickedButton1(); // Start Button
	afx_msg void OnBnClickedButton2(); // Auto Button
	afx_msg void OnBnClickedButton3(); // Go Button
	afx_msg void OnBnClickedOk(); // OK ���s �����{����
	afx_msg void OnBnClickedCancel(); // �����s���ϥ� �`�٪Ŷ�
	afx_msg void OnBnClickedButton4(); // �Q��X�� Auto button �� �|�۰ʳQ���U
	afx_msg void OnBnClickedButton5(); // �Q��X�� Auto button �� �|�۰ʳQ���U
	afx_msg void OnCbnSelchangeCombo1(); // �]�w��ʼҦ��� �n������H�i�檺�ʧ@
	afx_msg void OnBnClickedButton6(); // ���s "Send" ��ʼҦ��M��
	afx_msg void OnBnClickedButton7(); // ���s PMS/BMS
	afx_msg void OnBnClickedButton8(); // ���s Init_PBMS
	afx_msg void OnBnClickedButton9(); // ���s PMS/BMS Save
	afx_msg void OnBnClickedCheck1(); // ���� ���� ADAMS ���� �T�ؼҦ�
	afx_msg void OnBnClickedCheck2(); // ���� ���� ADAMS ���� �T�ؼҦ�
	afx_msg void OnBnClickedCheck3(); // ���� ���� ADAMS ���� �T�ؼҦ�
	afx_msg void OnBnClickedCheck4(); // Checkbox --> Read Encoder
	afx_msg void OnBnClickedCheck5(); // Checkbox --> Read 6 axis force sensor
	afx_msg void OnBnClickedCheck6(); // �O�_ø�sZMP����p�y
	afx_msg void OnBnClickedCheck8(); // Checkbox --> Speech�]�w�n���n�}�һy���Ҳ�(�ثe�S�Ψ�)
	afx_msg void OnBnClickedCheck7(); // Checkbox --> Read Infrared (old: Skin Module)
	afx_msg void OnBnClickedCheck9(); // Checkbox --> Boost
	afx_msg void OnBnClickedCheck10();// Checkbox --> IMU
	afx_msg void OnBnClickedCheck11();// Checkbox --> 3D Renderings
	afx_msg void OnBnClickedCheck12();// Checkbox --> Manual Mode
	afx_msg void OnBnClickedCheck13();// Checkbox --> 3D Model Rendering Mode
	afx_msg void OnBnClickedButton10();// ���s Biped Control Emergent Stop
	afx_msg void OnBnClickedCheck14();// Checkbox --> Arm Control Included
	afx_msg void OnBnClickedCheck15();// Checkbox --> Hand Control Included
	afx_msg void OnBnClickedButton11();// ���s Epos3 Fault Reset
};
