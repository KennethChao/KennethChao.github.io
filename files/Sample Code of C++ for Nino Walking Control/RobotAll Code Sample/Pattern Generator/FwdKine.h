/**************************************************************************************************
Copyright, 2010-2012, Robotics Lab., Dept. of M.E., National Taiwan University
File Name: FwdKine.h

Author: Jiu-Lou Yan
Version: 1.0
Date: 2010/07/20

Functions:
	FwdKine() ~FwdKine() DHToMatrices() DHConstruct() InitKine()

Classes: FwdKine

Description:
     本程式主要用在建立順向運動學物件
	 利用此物件與函式 我們可以輕易地計算順向運動學
	 各函式與變數之說明請詳見下方宣告與定義處之說明

Note: None
***************************************************************************************************/

#include "YMatLite.h"
#ifndef FWDKINE_H //防止fwdkine.h被重複定義
#define FWDKINE_H //防止fwdkine.h被重複定義
#include "QuaternionRotation.h"

class FwdKine
{
public:
	FwdKine(void); // constructor
	FwdKine(int NofJ); // constructor with initialization
	~FwdKine(void); // descructor
	void DHToMatrices(double a, double d, double theta, double theta_home, double alpha, YMatLite* A); // calculate 4x4 homogeneous matrix
	void DHConstruct(void); // calculate the whold kinematics train
	void InitKine(int NofJ); // initialize the object

	int NumJoint; // total number of joints

	// DH parameters
	double* a;
	double* d;
	double* theta;
	double* theta_home;
	double* alpha;

	YMatLite* Rn; // All homogeneous matrices
	YMatLite* RotMats; // All rotation matrices

	YMatLite TempDH; // buffer for calculation
	YMatLite TempDH2;; // buffer for calculation
};
#endif //防止fwdkine.h被重複定義