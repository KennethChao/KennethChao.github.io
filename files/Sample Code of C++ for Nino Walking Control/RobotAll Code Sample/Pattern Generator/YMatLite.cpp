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
     ���{���D�n�Φb�إ߯x�}�k����
	 ���F�x�s�x�}��ƥ~�A���x�}����i�Q�ΨӾާ@�U���U�˪��x�}�B��
	 �U�禡�P�ܼƤ������иԨ��U��ŧi�P�w�q�B������

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
	input: Row Column �x�}����C�j�p
	output: void

	Note:
	// Class constructor ��l���ܼƻP�O������t
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
	input: Row Column �x�}����C�j�p
	output: void

	Note:
	// ��󥼪�l�Ƥ��x�}�i�� ��l���ܼƻP�O������t
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
	// Class destructor �x�}�Ѻc�l
	******************************************************************/
	delete[] data;
}

void YMatLite::ShowMat(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// �N�x�}�C�L�bcommand window����
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
	input: Row Col Val (�N�x�}����Row�C����Col��]��Val��)
	output: void

	Note:
	// �]�w��@�x�}������
	******************************************************************/
	// 	int NCol, MRow, MSize;
	data[(Row-1)*NCol+Col-1] = Val;
}