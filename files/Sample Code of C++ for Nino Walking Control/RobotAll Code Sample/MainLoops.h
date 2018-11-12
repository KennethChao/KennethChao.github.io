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
     ���{���O��ӵ{��������֤ߡA�޲z�FopenGL���ƥ�
	 COM Part�q�T �H�ξ����H����l�ƻP�����
	 �U�禡�P�ܼƤ������иԨ��U��ŧi�P�w�q�B������

Note: None
***************************************************************************************************/

#pragma once
//#include "PEO\PhysicalEngineODE.h"
#include "Pattern Generator\LQSISolver.h"  // kine.h �w�Q�]�t�b lqsi.h�����F
#include <fstream>
#include "DataProcess.h"
#include "serial_port.h"
#include "ConstIO.h"
#include "glut.h"

#define TwinCAT_Mode 0	//�������� �i�H���u���� �����PTwinCAT���q 
//���`�N�ݭn�� ����`��-RobotAll(�k��)-�ݩ�-�s����-��J-��L�̩ۨ� �R��TCatIoDrv.lib �~�i�H���u����

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

void gInitialization(); // ��l�ƾ����H
void gOpenPort(); // �}�ҩҦ�USB and COM Ports
void gClosePort(); //  �����Ҧ�USB and COM Ports
void gPreProcessData(unsigned int mode); // �����H���U�ر���
void gLoadfile(); // Ū���Ҧ��ݭn�q.txt .dat�����
void gInitializeKinematicsGL(void); // ��l��openGL����
void gTWINCATCom(unsigned int index);

void gTestHand();

// ========================================
	// Kinematics Drawing functions

	// functions
	void gRenderSceneEmpty(void); // �Ũ禡 openGL���������۰�Ĳ�o�欰
	void gRenderSceneThread(void); // �u���bø�sGL������
	void gMouseFunc(int button, int state, int x, int y); // GL�ƹ��ƥ�
	void gMousePressedMove(int x, int y);  // GL�ƹ��ƥ�
	void gMouseNotPressedMove(int x, int y);  // GL�ƹ��ƥ�
	void gOnSizeMyGL(int w, int h);  // GL���e���j�p

	double gNorm2Pd(double* p1, double* p2); // ���X2���I���Z�� double ����
	float gNorm2Pf(float* p1, float* p2); // ���X2���I���Z�� float ����
	float gNorm1Pf(float* p1); // ���Xnorm
	float gNorm1Pd(double* p1); // ���Xnorm
	void gCross2Vf(float* v1, float* v2, float* v3); // �V�q cross float version
	void gCross2Vd(double* v1, double* v2, double* v3); //  �V�q cross double version
	void gRotYMat(float th, YMatLite* Rxn); // �ƹ��ƥ�̭���GL�@�ɱ��઺����x�}Y�b
	void gRotXMat(float th, YMatLite* Ryn); // �ƹ��ƥ�̭���GL�@�ɱ��઺����x�}X�b
	void gGLDrawLinePoint(float* CrdAll,int Len); // �Q��GL�۰ʤƥX�I��u ø�s�X�����H�δΤH�ҫ�

	// extern variables
	extern bool gStartSensing; // �T�{�O�_���U"Start"�s�F
	
	void C2MWrite2Txt(int datasize, double data[], int dataindexstart, int switchnumb, fstream &tempfile); // �N��J��ADAMS���ܼƼg�Jtxt
	void C2MLoadTraj(int datanumb, double data[],int dataindexstart, fstream &tempfile); // �N�һݪ��ܼƦs����w�O����A���U�ӭn�g��txt�����ǻ���ADAMS/Simulink

	void gCoPCali(int* FlagL, int* FlagR);	// �ǥѤO�W�ȱo�쪺��}CoP��m�վ�Cali���}���O����
	void gCaliManual(void);	// ��ʽվ�C�@�b������

	void gLoadfileHand(void); //load file // �ܮm 20130410
	void gPreProcessDataHand(unsigned int mode,unsigned int times,float interval ,unsigned int axinumb); //��mode // �ܮm 20130410
	void posturecalibration(float *IMUpitch , float *IMUroll , int count ); //�ǥ�IMU calibration �W������  20140218 ���a 