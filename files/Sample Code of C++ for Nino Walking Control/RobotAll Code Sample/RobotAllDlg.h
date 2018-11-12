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
	  OnBnClickedCheck8(); // Checkbox --> Speech設定要不要開啟語音模組(目前沒用到)
	  OnBnClickedCheck7(); // Checkbox --> Read Infrared (old: Skin Module)
	  OnBnClickedCheck9(); // Checkbox --> Boost
	  OnBnClickedCheck10();// Checkbox --> IMU
	  OnBnClickedCheck11();// Checkbox --> 3D Renderings
	  OnBnClickedCheck12();// Checkbox --> Manual Mode
	  OnBnClickedCheck13();// Checkbox --> 3D Model Rendering Mode
	  OnBnClickedButton10();// 按鈕 Biped Control Emergent Stop
	  OnBnClickedCheck14();// Checkbox --> Arm Control Included
	  OnBnClickedCheck15();// Checkbox --> Hand Control Included
	  OnBnClickedButton11();// 按鈕 Epos3 Fault Reset

Classes: None

Description:
     本程式是整個程式的GUI控制核心，所有按鈕與事件
	 所有人機介面都由這個程式來控管
	 各函式與變數之說明請詳見下方宣告與定義處之說明

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

void gIKStep(void); // IK 解一格
void gThreadTest(void); // 測試 sleep時間精準性

// 初始化各種動作
void gInitTurnLeft(void); // 為了轉彎動作初始化
void gInitTurnRight(int StepNum, double TurnRadius, double TurnAngle, double Dist2L); // 為了轉彎動作初始化
void gInitWalkStraight(int StepInput, double StepLength); // 為了直走動作初始化
void gInitStepHere(void); // 為了原地踏步動作初始化
void gInitStair(void); // 為了樓梯動作初始化
//哲軒20121129
void gInitdownStair(void); // 為了下樓梯動作初始化
void gIntLEDFace (int mode) ; //control  LEDFace 

//哲軒20121129
//Slongz 20130516
void gInitStairMod(int StepNum, double Dist2L, double StairHeight);   // Demo 2nd  一階一步模式(下樓梯用)
void gInitUpStairMod(int StepNum, double Dist2L, double StairHeight); // Demo 2nd  一階兩步模式(上樓梯用)
//Slongz 20130516

void gInitSquat(void); // 為了蹲站動作初始化
void gInitStayMode(double StayTime); // 機器人為了要原地自我介紹的模式
void gInitSideWalk(int Direction); // 側向行走 可選擇左右
void gInitSumoMotion(void); // 抬起單腳 類似相撲動作

//泓逸start20120309
void gInitArmWaveWalkStraight(void);		//走直線時手部軌跡初始化
void gInitArmPushCartWalkStraight(void);	//推東西時手部軌跡初始化
void gInitRArmCarryWalkStraight(void);		//用右手提東西時手部軌跡初始化
void gInitArmWaveTurn(int Direction);				//轉彎時手部軌跡初始化
void gInitArmSideWalkWave(int Direction);//側向行走
void gInitArmSumoMotion(void); // 單腳站立模式 手臂運動定義
void gInitArmIntroductionMode(double Time,int Mode,int Count);//自我介紹用
//泓逸end20120309

//Slongz20130416
void gInitArmManualWalkStraight(int StepInput);// 小Demo  手臂擺動模式
//Slongz start20120531
void gInitQCCDWalkStraight(int StepInput, double StepLength); //走直線腳拉直軌跡初始化(需手調)
//Slongz end 20120531

void gEncFeedback(void);	// 利用Encoder IMU估算COG error 20130522 WZ
void gSendOfflineTraj(void);	// 直接餵入離線ENC軌跡 20140224 WZ
void gSpeakSignLanguage(int IndexCount);//20130525 CCC
void uprightcontrol(float *IMUroll,float *IMUpitch , double *Ldeltatheta, double *Rdeltatheta , int count );	//add by 哲軒 20140218控制上身水平


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
		COpenGLControl openGLControl; // MFC GL物件宣告
	public:
		CRect rect; // MFC GL物件畫布大小

	public:
		CButton mButton_Start; // Start 按鈕物件宣告
	public:
		CButton mButton_Auto; // Auto 按鈕物件宣告
	public:
		CButton mButton_Go; // Go 按鈕物件宣告
	public:
		CButton mButton_Init_PBMS;// Init_PMS 按鈕物件宣告
	public:
		CButton mButton_PMSBMS;// PMS/BMS 按鈕物件宣告
	public:
		int SendIndex; // sending data mode index


public:
	float m_scale;;
public:
	CComboBox SendMode;// drop list
public:
	CEdit EDIT1; // 下方的文字框
	CButton CheckSimu; // 按鈕控制項定義
	CButton CheckExp; // 按鈕控制項定義
	CButton CheckADAMS; // 按鈕控制項定義
	CButton CheckEncoder; // 按鈕控制項定義
	CButton CheckForceSensor; // 按鈕控制項定義
	CButton CheckZMPPlot; // 按鈕控制項定義
	CButton CheckInfrared; // 按鈕控制項定義
	CButton CheckSpeech; // 按鈕控制項定義
	CButton CheckBoost; // 決定是否CPU全速解IK
	CButton CheckIMU; // 是否讀取IMU
	CButton CheckOpenGL;// 是否繪製OpenGL model
	CButton CheckManualMode;// 按鈕控制項定義
	CButton CheckPE;// 按鈕控制項定義
	CButton CheckEmergentStop; // 按鈕控制項定義
	CButton CheckArmCtrl; // 按鈕控制項定義
	CButton CheckHandCtrl; // 按鈕控制項定義
public:
	afx_msg void OnBnClickedButton1(); // Start Button
	afx_msg void OnBnClickedButton2(); // Auto Button
	afx_msg void OnBnClickedButton3(); // Go Button
	afx_msg void OnBnClickedOk(); // OK 按鈕 結束程式用
	afx_msg void OnBnClickedCancel(); // 此按鈕不使用 節省空間
	afx_msg void OnBnClickedButton4(); // 被整合到 Auto button 中 會自動被壓下
	afx_msg void OnBnClickedButton5(); // 被整合到 Auto button 中 會自動被壓下
	afx_msg void OnCbnSelchangeCombo1(); // 設定手動模式時 要對機器人進行的動作
	afx_msg void OnBnClickedButton6(); // 按鈕 "Send" 手動模式專用
	afx_msg void OnBnClickedButton7(); // 按鈕 PMS/BMS
	afx_msg void OnBnClickedButton8(); // 按鈕 Init_PBMS
	afx_msg void OnBnClickedButton9(); // 按鈕 PMS/BMS Save
	afx_msg void OnBnClickedCheck1(); // 切換 模擬 ADAMS 實驗 三種模式
	afx_msg void OnBnClickedCheck2(); // 切換 模擬 ADAMS 實驗 三種模式
	afx_msg void OnBnClickedCheck3(); // 切換 模擬 ADAMS 實驗 三種模式
	afx_msg void OnBnClickedCheck4(); // Checkbox --> Read Encoder
	afx_msg void OnBnClickedCheck5(); // Checkbox --> Read 6 axis force sensor
	afx_msg void OnBnClickedCheck6(); // 是否繪製ZMP紅色小球
	afx_msg void OnBnClickedCheck8(); // Checkbox --> Speech設定要不要開啟語音模組(目前沒用到)
	afx_msg void OnBnClickedCheck7(); // Checkbox --> Read Infrared (old: Skin Module)
	afx_msg void OnBnClickedCheck9(); // Checkbox --> Boost
	afx_msg void OnBnClickedCheck10();// Checkbox --> IMU
	afx_msg void OnBnClickedCheck11();// Checkbox --> 3D Renderings
	afx_msg void OnBnClickedCheck12();// Checkbox --> Manual Mode
	afx_msg void OnBnClickedCheck13();// Checkbox --> 3D Model Rendering Mode
	afx_msg void OnBnClickedButton10();// 按鈕 Biped Control Emergent Stop
	afx_msg void OnBnClickedCheck14();// Checkbox --> Arm Control Included
	afx_msg void OnBnClickedCheck15();// Checkbox --> Hand Control Included
	afx_msg void OnBnClickedButton11();// 按鈕 Epos3 Fault Reset
};
