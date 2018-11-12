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
     ���{���D�n�Φb�إ߯x�}�k����
	 ���F�x�s�x�}��ƥ~�A���x�}����i�Q�ΨӾާ@�U���U�˪��x�}�B��
	 �U�禡�P�ܼƤ������иԨ��U��ŧi�P�w�q�B������

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

	int NCol, MRow, MSize; // �x�}���B�e�B�j�p
	double* data; // �x�}��
};
