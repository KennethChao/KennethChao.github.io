/**************************************************************************************************
Copyright, 2010-2012, Robotics Lab., Dept. of M.E., National Taiwan University
File Name: MainLoops.h

Author: Many People
Version: 1.0
Date: 2012/03/20

Functions:
     gInitialization() gOpenPort() gClosePort() gPreProcessData() gLoadfile() gInitializeKinematicsGL()
     gRenderSceneEmpty() gRenderSceneThread() gMouseFunc() gMousePressedMove() gMouseNotPressedMove() 
	 gOnSizeMyGL() gNorm2Pd() gNorm2Pf() gNorm1Pf() gCross2Vf() gCross2Vd() gRotYMat() gRotXMat()
	 gGLDrawLinePoint() C2MWrite2Txt() C2MLoadTraj() gCoPCali() gAnkleStraytegy() gCaliManual()

Classes: None

Description:
     本程式是整個程式的控制核心，管理了openGL的事件
	 COM Part通訊 以及機器人的初始化與控制等等
	 各函式與變數之說明請詳見下方宣告與定義處之說明

Note: None
***************************************************************************************************/

#pragma once
//#include "PEO\PhysicalEngineODE.h"
#include "Pattern Generator\LQSISolver.h"  // kine.h 已被包含在 lqsi.h之中了
#include <fstream>
#include "DataProcess.h"
#include "serial_port.h"
#include "ConstIO.h"
#include "glut.h"

#define TwinCAT_Mode 0	//關掉此項 可以離線測試 不須與TwinCAT溝通 
//但注意需要把 方案總管-RobotAll(右鍵)-屬性-連結器-輸入-其他相依性 刪掉TCatIoDrv.lib 才可以離線測試

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

void gInitialization(); // 初始化機器人
void gOpenPort(); // 開啟所有USB and COM Ports
void gClosePort(); //  關閉所有USB and COM Ports
void gPreProcessData(unsigned int mode); // 機器人的各種控制
void gLoadfile(); // 讀取所有需要從.txt .dat的資料
void gInitializeKinematicsGL(void); // 初始化openGL視窗
void gTWINCATCom(unsigned int index);

void gTestHand();

// ========================================
	// Kinematics Drawing functions

	// functions
	void gRenderSceneEmpty(void); // 空函式 openGL不執行任何自動觸發行為
	void gRenderSceneThread(void); // 真正在繪製GL的部分
	void gMouseFunc(int button, int state, int x, int y); // GL滑鼠事件
	void gMousePressedMove(int x, int y);  // GL滑鼠事件
	void gMouseNotPressedMove(int x, int y);  // GL滑鼠事件
	void gOnSizeMyGL(int w, int h);  // GL更改畫布大小

	double gNorm2Pd(double* p1, double* p2); // 取出2個點的距離 double 版本
	float gNorm2Pf(float* p1, float* p2); // 取出2個點的距離 float 版本
	float gNorm1Pf(float* p1); // 取出norm
	float gNorm1Pd(double* p1); // 取出norm
	void gCross2Vf(float* v1, float* v2, float* v3); // 向量 cross float version
	void gCross2Vd(double* v1, double* v2, double* v3); //  向量 cross double version
	void gRotYMat(float th, YMatLite* Rxn); // 滑鼠事件裡面讓GL世界旋轉的旋轉矩陣Y軸
	void gRotXMat(float th, YMatLite* Ryn); // 滑鼠事件裡面讓GL世界旋轉的旋轉矩陣X軸
	void gGLDrawLinePoint(float* CrdAll,int Len); // 利用GL自動化出點跟線 繪製出機器人棒棒人模型

	// extern variables
	extern bool gStartSensing; // 確認是否壓下"Start"鈕了
	
	void C2MWrite2Txt(int datasize, double data[], int dataindexstart, int switchnumb, fstream &tempfile); // 將輸入給ADAMS的變數寫入txt
	void C2MLoadTraj(int datanumb, double data[],int dataindexstart, fstream &tempfile); // 將所需的變數存到指定記憶體，接下來要寫到txt之中傳遞給ADAMS/Simulink

	void gCoPCali(int* FlagL, int* FlagR);	// 藉由力規值得到的兩腳CoP位置調整Cali的腳底板角度
	void gCaliManual(void);	// 手動調整每一軸的角度

	void gLoadfileHand(void); //load file // 至峻 20130410
	void gPreProcessDataHand(unsigned int mode,unsigned int times,float interval ,unsigned int axinumb); //選mode // 至峻 20130410
	void posturecalibration(float *IMUpitch , float *IMUroll , int count ); //藉由IMU calibration 上身角度  20140218 哲軒 