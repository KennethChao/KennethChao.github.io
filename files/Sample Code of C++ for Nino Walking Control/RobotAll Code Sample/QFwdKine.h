/**************************************************************************************************
Copyright, 2010-2012, Robotics Lab., Dept. of M.E., National Taiwan University
File Name: FwdKine.h

Author: Slongz
Version: 1.0
Date: 2012/08/07

Functions:
	QFwdKine() ~QFwdKine() 

Classes: QFwdKine

Description:
     ���{���D�n�Φb�إ߶��V�B�ʾǪ���A�MClass FwdKine���P�B���ϥ�quaternion�ӧ@���y�z����¦����
	 �Q�Φ�����P�禡 �ڭ̥i�H�����a�p�ⶶ�V�B�ʾ�
	 �U�禡�P�ܼƤ������иԨ��U��ŧi�P�w�q�B������

Note: None
***************************************************************************************************/

#ifndef QFWDKINE_H //����QuaternionRotation.h�Q���Ʃw�q
#define QFWDKINE_H //����QuaternionRotation.h�Q���Ʃw�q
#include "QuaternionRotation.h"

class QFwdKine
{
public:
	QFwdKine(void); //�غc�l
	QFwdKine(int NofJ); //���w�b�ƪ������غc�l(�ثe���ϥ�)
	~QFwdKine(void); //�Ѻc�l

	Quaternion* QRotate[6]; //�N��U��b���઺quaternion
	Quaternion* Q1n[5]; //�N��U�b��@�ɱ��઺quaternion
	Quaternion* QJointPos[6];//�N��U�b��@�ɦ첾��quaternion
	Quaternion* QJointZaxis[6]; //�N��U�b�b�@�ɤ��y�Ъ�quaternion
	Quaternion* QZaxis[6]; //�N��U�b�b�@�ɤ�����lZ�b��Vquaternion
	Quaternion* QEndeffector[6]; //�N��U�b�b�@�ɤ������Z�b���V��quaternion

	double Qth[6]; //�U�b������
	double QthHome[6]; //�U�b���ת�home��
	double Rot[9]; //End-effector��rotation matrix
	double TarRot[9]; //��IK�ɥΪ�End-effector��target rotation matrix
};

#endif //����QuaternionRotation.h�Q���Ʃw�q
