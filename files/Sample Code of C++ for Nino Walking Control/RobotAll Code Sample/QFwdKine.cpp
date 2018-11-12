/**************************************************************************************************
Copyright, 2010-2012, Robotics Lab., Dept. of M.E., National Taiwan University
File Name: FwdKine.cpp

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

#include "StdAfx.h"
#include "QFwdKine.h"


QFwdKine::QFwdKine(void)
{
	for(int i=0;i<6;i++)  //�ثe���w6�ۥѫ� for swing leg
	{
		QRotate[i] = new Quaternion(); //�N��U��b���઺quaternion
		Q1n[i] = new Quaternion(); //�N��U�b��@�ɱ��઺quaternion
		QJointPos[i] = new Quaternion();//�N��U�b��@�ɦ첾��quaternion
		QEndeffector[i] = new Quaternion(); //�N��U�b�b�@�ɤ��y�Ъ�quaternion
		QJointZaxis[i] = new Quaternion(); //�N��U�b�b�@�ɤ�����lZ�b��Vquaternion
		QZaxis[i] = new Quaternion(); //�N��U�b�b�@�ɤ������Z�b���V��quaternion
	}
	for (int i=0;i<6;i++)
		Qth[i]=0; //�U�b������
	for (int i=0;i<6;i++)
		QthHome[i]=0; //�U�b���ת�home��
}
QFwdKine::~QFwdKine(void)
{
	for(int i=0;i<6;i++)  //�ثe���w6�ۥѫ� for swing leg
	{
		delete QRotate[i];
		delete Q1n[i];
		delete QJointPos[i];
		delete QEndeffector[i];
		delete QJointZaxis[i];
		delete QZaxis[i];
	}
}
