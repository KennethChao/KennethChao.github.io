/**************************************************************************************************
Copyright, 2010-2012, Robotics Lab., Dept. of M.E., National Taiwan University
File Name: FwdKine.cpp

Author: Jiu-Lou Yan
Version: 1.0
Date: 2010/07/20

Functions:
	FwdKine() ~FwdKine() DHToMatrices() DHConstruct(void) InitKine(int NofJ)

Classes: FwdKine

Description:
     ���{���D�n�Φb�إ߶��V�B�ʾǪ���
	 �Q�Φ�����P�禡 �ڭ̥i�H�����a�p�ⶶ�V�B�ʾ�
	 �U�禡�P�ܼƤ������иԨ��U��ŧi�P�w�q�B������

Note: None
***************************************************************************************************/

#include "stdafx.h"
#include "FwdKine.h"

FwdKine::FwdKine(void) // not used
{ 
	/******************************************************************
	input: void
	output: void

	Note:
	// Class constructor
	******************************************************************/
}

FwdKine::FwdKine(int NofJ)
{ 
	/******************************************************************
	input: NofJ ��J�`�b��
	output: void

	Note:
	// Class constructor ��l���ܼƻP�O������t
	******************************************************************/

	NumJoint = NofJ;
	a = new double[NofJ];
	d = new double[NofJ];
	theta = new double[NofJ];
	theta_home = new double[NofJ];
	alpha = new double[NofJ];

	Rn = new YMatLite[NofJ];
	RotMats = new YMatLite[NofJ];
	for (int i = 0; i < NofJ ; i++)
	{
		Rn[i].InitPara(4,4); // DH matrices are all 4*4
		RotMats[i].InitPara(3,3); // DH matrices are all 4*4
	}

	TempDH.InitPara(4,4); 
	TempDH2.InitPara(4,4); 
	//TempDH.data[0] = 1;	TempDH.data[1] = 0;	TempDH.data[2] = 0;	TempDH.data[3] = 0;	
	//TempDH.data[4] = 0;	TempDH.data[5] = 1;	TempDH.data[6] = 0;	TempDH.data[7] = 0;
	//TempDH.data[8] = 0;	TempDH.data[9] = 0;	TempDH.data[10] = 1;	TempDH.data[11] = 0;
	//TempDH.data[12] = 0;	TempDH.data[13] = 0;	TempDH.data[14] = 0;	TempDH.data[15] = 1;

}

void FwdKine::InitKine(int NofJ)
{ 
	/******************************************************************
	input: NofJ ��J�`�b��
	output: void

	Note:
	// ��󥼳Q��l�ƪ�FWDKine����i���l���ܼƻP�O������t
	******************************************************************/
	NumJoint = NofJ;
	a = new double[NofJ];
	d = new double[NofJ];
	theta = new double[NofJ];
	theta_home = new double[NofJ];
	alpha = new double[NofJ];

	Rn = new YMatLite[NofJ];
	for (int i = 0; i < NofJ ; i++)
	{
		Rn[i].InitPara(4,4); // DH matrices are all 4*4
	}

	TempDH.InitPara(4,4); 
	TempDH2.InitPara(4,4); 
	//TempDH.data[0] = 1;	TempDH.data[1] = 0;	TempDH.data[2] = 0;	TempDH.data[3] = 0;	
	//TempDH.data[4] = 0;	TempDH.data[5] = 1;	TempDH.data[6] = 0;	TempDH.data[7] = 0;
	//TempDH.data[8] = 0;	TempDH.data[9] = 0;	TempDH.data[10] = 1;	TempDH.data[11] = 0;
	//TempDH.data[12] = 0;	TempDH.data[13] = 0;	TempDH.data[14] = 0;	TempDH.data[15] = 1;

}


FwdKine::~FwdKine(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// Class destructor �M���ʺA�O����
	******************************************************************/

	delete[] a;
	delete[] d;
	delete[] theta;
	delete[] theta_home;
	delete[] alpha;
	delete[] Rn;
	delete[] RotMats;
}

void FwdKine::DHToMatrices(double a, double d, double theta, double theta_home, double alpha, YMatLite* A)
{
	/******************************************************************
	input: DH parameters: a d theta theta_home alpha, ���G�x�s�b A ��
	output: void

	Note:
	// �N��J��DH parameters �p�⦨ 4x4 homogeneous matrix �x�s�bA��
	******************************************************************/
	double th = theta+theta_home;
	A->data[0] = cos(th);	A->data[1] = -sin(th)*cos(alpha);	A->data[2] = sin(th)*sin(alpha);	A->data[3] = a*cos(th);
	A->data[4] = sin(th);	A->data[5] = cos(th)*cos(alpha);	A->data[6] = -cos(th)*sin(alpha);	A->data[7] = a*sin(th);
	A->data[8] = 0;			A->data[9] = sin(alpha);			A->data[10] = cos(alpha);			A->data[11] = d;
	A->data[12] = 0;		A->data[13] = 0;					A->data[14] = 0;					A->data[15] = 1;

	/*
	M =  [cos(theta+theta_home)   (-1)*sin(theta+theta_home)*cos(alpha)  sin(theta+theta_home)*sin(alpha)        a*cos(theta+theta_home);
		  sin(theta+theta_home)   cos(theta+theta_home)*cos(alpha)       (-1)*cos(theta+theta_home)*sin(alpha)   a*sin(theta+theta_home);
		  0                       sin(alpha)                             cos(alpha)                              d                      ;
		  0                       0                                      0                                       1                      ];	
	*/
}


void FwdKine::DHConstruct(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// �۰ʭp���� kinematics train �p�P�ǲξ����H�ǱN�C�� homogeneous matrix ���_��
	******************************************************************/
	
	DHToMatrices(a[0],d[0],theta[0],theta_home[0],alpha[0],&Rn[0]);

	RotMats[0].data[0] = Rn[0].data[0]; // ��۩�A�G�N���Fcross��K
	RotMats[0].data[3] = Rn[0].data[1];
	RotMats[0].data[6] = Rn[0].data[2];
	RotMats[0].data[1] = Rn[0].data[4];
	RotMats[0].data[4] = Rn[0].data[5];
	RotMats[0].data[7] = Rn[0].data[6];
	RotMats[0].data[2] = Rn[0].data[8];
	RotMats[0].data[5] = Rn[0].data[9];
	RotMats[0].data[8] = Rn[0].data[10];

	for (int i=0 ; i<16 ; i++) // copy
	{
		TempDH.data[i] = Rn[0].data[i];
	}

	for (int j=0 ; j<NumJoint-1 ; j++)
	{
		DHToMatrices(a[j+1],d[j+1],theta[j+1],theta_home[j+1],alpha[j+1],&TempDH2);

		RotMats[j+1].data[0] = TempDH2.data[0];
		RotMats[j+1].data[3] = TempDH2.data[1];
		RotMats[j+1].data[6] = TempDH2.data[2];
		RotMats[j+1].data[1] = TempDH2.data[4];
		RotMats[j+1].data[4] = TempDH2.data[5];
		RotMats[j+1].data[7] = TempDH2.data[6];
		RotMats[j+1].data[2] = TempDH2.data[8];
		RotMats[j+1].data[5] = TempDH2.data[9];
		RotMats[j+1].data[8] = TempDH2.data[10];

		MatMulAB(TempDH.data,4,4,TempDH2.data,4,4,Rn[j+1].data);

		for (int i=0 ; i<16 ; i++) // copy
		{
			TempDH.data[i] = Rn[j+1].data[i];
		}
	}
}

