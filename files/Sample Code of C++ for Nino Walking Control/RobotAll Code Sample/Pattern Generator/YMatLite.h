/**************************************************************************************************
Copyright, 2010-2012, Robotics Lab., Dept. of M.E., National Taiwan University
File Name: YMatLite.h

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

#include "MatOptInterface.h"

class YMatLite
{
public:
	YMatLite(void); // constructor
	YMatLite(int Row, int Column); // constructor with initialization
	~YMatLite(void); // destructor
	void ShowMat(void); // print the matrix
	void InitPara(int Row, int Column); // initialize the matrix object
	void ValSet( int Row, int Col, double Val); // set a single value in the matrix

	int NCol, MRow, MSize; // 矩陣長、寬、大小
	double* data; // 矩陣值
};
