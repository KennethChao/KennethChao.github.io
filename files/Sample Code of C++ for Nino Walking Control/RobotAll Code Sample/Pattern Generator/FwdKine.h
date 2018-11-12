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
     ���{���D�n�Φb�إ߶��V�B�ʾǪ���
	 �Q�Φ�����P�禡 �ڭ̥i�H�����a�p�ⶶ�V�B�ʾ�
	 �U�禡�P�ܼƤ������иԨ��U��ŧi�P�w�q�B������

Note: None
***************************************************************************************************/

#include "YMatLite.h"
#ifndef FWDKINE_H //����fwdkine.h�Q���Ʃw�q
#define FWDKINE_H //����fwdkine.h�Q���Ʃw�q
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
#endif //����fwdkine.h�Q���Ʃw�q