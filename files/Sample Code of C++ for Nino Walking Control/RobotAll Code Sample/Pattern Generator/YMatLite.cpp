/**************************************************************************************************
Copyright, 2010-2012, Robotics Lab., Dept. of M.E., National Taiwan University
File Name: YMatLite.cpp

Author: Jiu-Lou Yan
Version: 1.1
Date: 2010/07/20

Functions:
	YMatLite() ~YMatLite() ShowMat() InitPara() ValSet()

Classes: YMatLite

Description:
     本程式主要用在建立矩陣法物件
	 除了儲存矩陣資料外，此矩陣物件可被用來操作各式各樣的矩陣運算
	 各函式與變數之說明請詳見下方宣告與定義處之說明

Note: None
***************************************************************************************************/

#include "stdafx.h"
#include "YMatLite.h"

YMatLite::YMatLite(void) //
{ 
	/******************************************************************
	input: void
	output: void

	Note:
	// Class constructor 
	******************************************************************/
}

YMatLite::YMatLite(int Row, int Column) // for buffer-type YMat
{
	/******************************************************************
	input: Row Column 矩陣的行列大小
	output: void

	Note:
	// Class constructor 初始化變數與記憶體分配
	******************************************************************/

	NCol = Column;
	MRow = Row;
	MSize = NCol*MRow;
	data = new double[MSize]; 

	for (int i = 0 ; i < MSize ; i++)
		data[i] = 0;
}


void YMatLite::InitPara(int Row, int Column)  // for buffer-type YMat
{
	/******************************************************************
	input: Row Column 矩陣的行列大小
	output: void

	Note:
	// 對於未初始化之矩陣進行 初始化變數與記憶體分配
	******************************************************************/
	NCol = Column;
	MRow = Row;
	MSize = NCol*MRow;
	data = new double[MSize];

	for (int i = 0 ; i < MSize ; i++)
		data[i] = 0;
}

YMatLite::~YMatLite(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// Class destructor 矩陣解構子
	******************************************************************/
	delete[] data;
}

void YMatLite::ShowMat(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 將矩陣列印在command window之中
	******************************************************************/

	cout << "Matrix Data: " << endl;

	for (int i = 0 ; i < MRow ; i++)
	{
		for (int j = 0 ; j < NCol ; j++)
		{

			if (data[j+i*NCol] == 0)
				cout << "0" << "\t";
			else if (fabs(data[j+i*NCol]) < 0.0001)
				cout << "0.0" << "\t";
			else
				cout << data[j+i*NCol] << "\t";
		}
		cout << endl;
	}
}

void YMatLite::ValSet( int Row, int Col, double Val)
{
	/******************************************************************
	input: Row Col Val (將矩陣的第Row列的第Col行設為Val值)
	output: void

	Note:
	// 設定單一矩陣元素值
	******************************************************************/
	// 	int NCol, MRow, MSize;
	data[(Row-1)*NCol+Col-1] = Val;
}