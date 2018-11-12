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
     本程式主要用在建立順向運動學物件，和Class FwdKine不同處為使用quaternion來作為描述的基礎元素
	 利用此物件與函式 我們可以輕易地計算順向運動學
	 各函式與變數之說明請詳見下方宣告與定義處之說明

Note: None
***************************************************************************************************/

#include "StdAfx.h"
#include "QFwdKine.h"


QFwdKine::QFwdKine(void)
{
	for(int i=0;i<6;i++)  //目前限定6自由度 for swing leg
	{
		QRotate[i] = new Quaternion(); //代表各轉軸旋轉的quaternion
		Q1n[i] = new Quaternion(); //代表各軸對世界旋轉的quaternion
		QJointPos[i] = new Quaternion();//代表各軸對世界位移的quaternion
		QEndeffector[i] = new Quaternion(); //代表各軸在世界中座標的quaternion
		QJointZaxis[i] = new Quaternion(); //代表各軸在世界中的初始Z軸方向quaternion
		QZaxis[i] = new Quaternion(); //代表各軸在世界中的實際Z軸指向的quaternion
	}
	for (int i=0;i<6;i++)
		Qth[i]=0; //各軸的角度
	for (int i=0;i<6;i++)
		QthHome[i]=0; //各軸角度的home值
}
QFwdKine::~QFwdKine(void)
{
	for(int i=0;i<6;i++)  //目前限定6自由度 for swing leg
	{
		delete QRotate[i];
		delete Q1n[i];
		delete QJointPos[i];
		delete QEndeffector[i];
		delete QJointZaxis[i];
		delete QZaxis[i];
	}
}
