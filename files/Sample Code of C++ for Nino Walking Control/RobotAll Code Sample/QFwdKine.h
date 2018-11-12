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
     本程式主要用在建立順向運動學物件，和Class FwdKine不同處為使用quaternion來作為描述的基礎元素
	 利用此物件與函式 我們可以輕易地計算順向運動學
	 各函式與變數之說明請詳見下方宣告與定義處之說明

Note: None
***************************************************************************************************/

#ifndef QFWDKINE_H //防止QuaternionRotation.h被重複定義
#define QFWDKINE_H //防止QuaternionRotation.h被重複定義
#include "QuaternionRotation.h"

class QFwdKine
{
public:
	QFwdKine(void); //建構子
	QFwdKine(int NofJ); //指定軸數版本的建構子(目前未使用)
	~QFwdKine(void); //解構子

	Quaternion* QRotate[6]; //代表各轉軸旋轉的quaternion
	Quaternion* Q1n[5]; //代表各軸對世界旋轉的quaternion
	Quaternion* QJointPos[6];//代表各軸對世界位移的quaternion
	Quaternion* QJointZaxis[6]; //代表各軸在世界中座標的quaternion
	Quaternion* QZaxis[6]; //代表各軸在世界中的初始Z軸方向quaternion
	Quaternion* QEndeffector[6]; //代表各軸在世界中的實際Z軸指向的quaternion

	double Qth[6]; //各軸的角度
	double QthHome[6]; //各軸角度的home值
	double Rot[9]; //End-effector的rotation matrix
	double TarRot[9]; //解IK時用的End-effector的target rotation matrix
};

#endif //防止QuaternionRotation.h被重複定義
