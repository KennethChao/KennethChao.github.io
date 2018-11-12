/**************************************************************************************************
Copyright, 2010-2012, Robotics Lab., Dept. of M.E., National Taiwan University
File Name: QuaternionRotation.h

Author: Jiu-Lou Yan
Version: 1.2
Date: 2012/03/01 => 2012/07/26 Revised by Slongz

Functions:
	Quaternion() ~Quaternion() Normalize() GetAngle() GetAxisPointer() 
	GetVectorPartPointer() CopyAxis()  SetAngle()  SetAxis() Getw() Setw()
	ShowQuaternion() ReCalFormData() SetData()

Classes: Quaternion

Description:
     ���{���D�n�Φb�إߥ|�����k����
	 �ؼЦb��F���Q�ΥH�|�����k���C���Ӱ���Ŷ�������
	 �U�禡�P�ܼƤ������иԨ��U��ŧi�P�w�q�B������

Note: None
***************************************************************************************************/

#include "MatOptInterface.h"
#ifndef QUATERNIONROTATION_H // ����quaternion.h�Q���Ʃw�q
#define QUATERNIONROTATION_H // ����quaternion.h�Q���Ʃw�q
class Quaternion
{
public:

	// functions
	Quaternion(void); // constructor
	~Quaternion(void); // destructor
	void Normalize(void); // normailze the quaternion
	double GetAngle(void); // get the rotation angle
	double* GetAxisPointer(void); // get the pointer of the axis
	double* GetVectorPartPointer(void); // get the vector part of the quaternion (v = vxsin(th/2) vysin(th/2) vzsin(th/2))
	void CopyAxis(double* copy); // copy the axis value to a 3-by-1 array
	void SetAngle(double angleValue); // set the rotation angle
	void SetAxis(double x, double y, double z); // set the rotaiton axis
	double Getw(void); // get cos(th/2)
	void Setw(double wValue); // set cos(th/2)
	void ShowQuaternion(void); // print the quaternion values in command window
	void ReCalFormData(void); // when the four data elements are renewed, re-calculate the angle and axis from data
	void SetData(double w, double x, double y, double z);  // set the 4 values of the quaternion

	// variables
	double* data; // the values of the quaternion data = [w v], it is defined as a 1-by-4 row vector
	double* wp; // the first element of the quaternion w = cos(angle/2.0);
	// vp ������ data+1���N��
	//double* vp; // the vector part of the quaternion, v (the second~fourth elements)
	double* axis; // the rotation axis is a 1-by-3 vector
	double angle; // the angle of rotation, w = cos(angle/2.0)

};

// global functions for quaternion manipulation
// function set of quaternion rotation
void QuatRotMul(Quaternion* q1, Quaternion* q2, Quaternion* result); // quaternion multiplication (original version : result = q1 or q2 =>forbidden)
void QuatToMat(Quaternion* q1, double* R1); //quaternion translated to matrix
void QuatFromMat(double* R1, Quaternion* q1);//matrix translated to quaternion
void QuatConj(Quaternion* q1, Quaternion* result); // find conjugate quaternion
void QuatVectorRotation(double* v1, Quaternion* q1, double* result); //vector rotated by a given quaternion
//.......................................TEST.......................................
void QuatMul(Quaternion* q1, Quaternion* q2, Quaternion* result); // quaternion multiplication (modified version : result = q1 or q2 =>OK)
void QuatToMatT(Quaternion* q1, double* R1); //quaternion translated to matrix (transpose)
void QuatMulConjQ(Quaternion* q1, Quaternion* q2, Quaternion* result);// quaternion multiplication: q1*conj(q2) (modified version : result = q1 or q2 =>OK)
void QuatMulPureQ(Quaternion* q1, Quaternion* q2, Quaternion* result); // quaternion multiplication for pure quaternions (modified version : result = q1 or q2 =>OK)
void QuatAdd(Quaternion* q1, Quaternion* q2, Quaternion* result); // quaternion addision
//.......................................TEST.......................................

// SLERP �y�Τ���
void QuatInitSLERP(Quaternion* q1, Quaternion* q2); // �ثe�u�䴩���t����
void QuatInitMatrixSLERP(double* R1, double* R2); // �ثe�u�䴩���t����A���ӥi�H�﨤�t�צ����ۥѫ��w���t�ױ���
void QuatSLERP(double t, Quaternion* result); // 0 <= t <= 1 // �ثe�u�䴩���t����
void QuatMatrixSLERP(double t, double* result); // 0 <= t <= 1  // �ثe�u�䴩���t����A���ӥi�H�﨤�t�צ����ۥѫ��w���t�ױ���

// �@��B��
void Cross2Vec(double* v1, double* v2, double* v3); // vector cross

#endif // ����quaternion.h�Q���Ʃw�q