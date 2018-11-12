/**************************************************************************************************
Copyright, 2010-2012, Robotics Lab., Dept. of M.E., National Taiwan University
File Name: QuaternionRotation.cpp

Author: Jiu-Lou Yan
Version: 1.1
Date: 2012/03/01

Functions:
	Quaternion() ~Quaternion() Normalize() GetAngle() GetAxisPointer() 
	GetVectorPartPointer() CopyAxis()  SetAngle()  SetAxis() Getw() Setw()
	ShowQuaternion() ReCalFormData()

Classes: Quaternion

Description:
     ���{���D�n�Φb�إߥ|�����k����
	 �ؼЦb��F���Q�ΥH�|�����k���C���Ӱ���Ŷ�������
	 �U�禡�P�ܼƤ������иԨ��U��ŧi�P�w�q�B������

Note: None
***************************************************************************************************/

#include <stdafx.h>
#include <QuaternionRotation.h>


// variables for computing SLERP should be only declared in QuaternionRotation.cpp
double QuatHalfTh; // half of rotation angle between two quaternions
double QuatCosHalfTh; // cos half of rotation angle between two quaternions
double QuatSinHalfTh; // sin half of rotation angle between two quaternions
Quaternion qa; // remember the q1 address for SLERP
Quaternion qb; // remember the q2 address for SLERP
double QuatScale1; // sin((1-t)theta)/sin(theta)
double QuatScale2; // sin(t*theta)/sin(theta)

double RotMa[9]; // Ra
double RotMb[9]; // Rb

double RotMbat[9]; // Rb*Ra'
double RotMSLERP[9]; // �����L�{������x�}
Quaternion QuatRotM; // �Ȧs�ܼ�
Quaternion QuatRotMSLERP; // �Ȧs�ܼ�
double QuatAxis[3]; // �Ȧs�ܼ�

// Member functions class: Quaternion
Quaternion::Quaternion(void) // constructor
{
	/******************************************************************
	input: void
	output: void

	Note:
	// Class constructor ��l���ܼƻP�O������t
	******************************************************************/

	data = new double[4];
	axis = new double[3];
	angle = 0.0; // rotation angle
	wp = data; // angle part of the quaternion

	// default quaternion = [0 0 0 1];
	data[0] = 0; data[1] = 0; data[2] = 0; data[3] = 1; 
	             axis[0] = 0; axis[1] = 0; axis[2] = 1;

}

// Member functions class: Quaternion
Quaternion::~Quaternion(void) // destructor
{
	/******************************************************************
	input: void
	output: void

	Note:
	// Class destructor �R���ʺA�O����
	******************************************************************/

	delete[] data;
	delete[] axis;
	// wp �]�� = data �ҥH���ΦA�� delete wp
}

// Member functions class: Quaternion
void Quaternion::Normalize(void) // normailze the quaternion
{
	/******************************************************************
	input: void
	output: void

	Note:
	// Normalize��e�|��������
	******************************************************************/

	double norm = 0.0;

	// normalize quaternion
	for (int i = 0 ; i < 4 ; i++)
		norm += data[i]*data[i];

	norm = sqrt(norm);

	for (int i = 0 ; i < 4 ; i++)
		data[i] /= norm;

	// normalize axis
	norm = 0.0;
	for (int i = 0 ; i < 3 ; i++)
		norm += axis[i]*axis[i];

	norm = sqrt(norm);

	for (int i = 0 ; i < 3 ; i++)
		axis[i] /= norm;

}

// Member functions class: Quaternion
double Quaternion::GetAngle(void) // get the rotation angle (w = cos(th/2.0))
{
	/******************************************************************
	input: void
	output: angle

	Note:
	// �^�Ƿ�e�|���������ਤ��
	******************************************************************/

	return angle;
}

// Member functions class: Quaternion
double* Quaternion::GetAxisPointer(void) // get the pointer of the axis
{
	/******************************************************************
	input: void
	output: axis

	Note:
	// �^�Ƿ�e�|����������b
	******************************************************************/
	return axis;
}

// Member functions class: Quaternion
double* Quaternion::GetVectorPartPointer(void) // get the vector part of the quaternion (v = vxsin(th/2) vysin(th/2) vzsin(th/2))
{
	/******************************************************************
	input: void
	output: axis pointer

	Note:
	// �^�Ƿ�e�|������b�x�s��}����
	******************************************************************/
	return data+1;
}

// Member functions class: Quaternion
void Quaternion::CopyAxis(double* copy) // copy the axis value to a 3-by-1 array
{
	/******************************************************************
	input: copy
	output: void

	Note:
	// �ƻs��e�|����������b�Ȩ��J���Щҫ����O���餤
	******************************************************************/
	copy[0] = axis[0];
	copy[1] = axis[1];
	copy[2] = axis[2];
}

// Member functions class: Quaternion
void Quaternion::SetAngle(double angleValue) // set the rotation angle
{
	/******************************************************************
	input: angleValue
	output: void

	Note:
	// �]�w��e�|�������ਤ �åB�۰ʹB��۹���������(�]�t�|�����ȧ��ܻP�ਤ�ȽƼg)
	******************************************************************/

	angle = angleValue;
	data[0] = cos(0.5*angle);
	data[1] = axis[0]*sin(0.5*angle);
	data[2] = axis[1]*sin(0.5*angle);
	data[3] = axis[2]*sin(0.5*angle);
}

// Member functions class: Quaternion
void Quaternion::SetAxis(double x, double y, double z) // set the rotaiton axis
{
	/******************************************************************
	input: x y z 
	output: void

	Note:
	// �]�w�|������b�A�åB�۰�normalize����b�V�q��s��|������
	******************************************************************/
	double norm = sqrt(x*x+y*y+z*z);

	if (norm == 0)
	{
		printf("��J�b���s�V�q�A���˹�{�����~\n");
		system("pause");
	}


	axis[0] = x/norm;
	axis[1] = y/norm;
	axis[2] = z/norm;

	// data[0] �ਤ ���Χ���
	data[1] = axis[0]*sin(0.5*angle);
	data[2] = axis[1]*sin(0.5*angle);
	data[3] = axis[2]*sin(0.5*angle);

}

// Member functions class: Quaternion
double Quaternion::Getw(void) // get cos(th/2)
{
	/******************************************************************
	input: void
	output: w

	Note:
	// ���X�|�������Ĥ@�ӭ�(w = cos(theta/2.0))
	******************************************************************/
	return data[0];
}

// Member functions class: Quaternion
void Quaternion::Setw(double wValue) // set cos(th/2)
{
	/******************************************************************
	input: wValue
	output: void

	Note:
	// �]�w�|�������Ĥ@�ӭ�(w = cos(theta/2.0)) �åB���ܬ۹������|��������
	******************************************************************/
	angle = 2.0*acos(wValue);
	SetAngle(angle);
}

// Member functions class: Quaternion
void Quaternion::SetData(double w, double x, double y, double z) // set the rotaiton axis
{
	/******************************************************************
	input: w x y z  
	output: void

	Note:
	// ��� void Quaternion::SetAxis
	// �����]�w�|������ (�Dunit quaternion�i��)
	******************************************************************/
	data[0]=w;
	data[1]=x;
	data[2]=y;
	data[3]=z;

}

// Member functions class: Quaternion
void Quaternion::ReCalFormData(void) // when the four data elements are renewed, re-calculate the angle and axis from data
{
	/******************************************************************
	input: void
	output: void

	Note:
	// ���Y�~�����j���g�|�����ȡA�ЧQ�Φ��禡���s��z�|����������� �]�t �ਤ ��b �|�����Ȫ�����
	******************************************************************/

	double sinHalfAngle;
	angle = 2.0*acos(data[0]);
	sinHalfAngle = sqrt(1-data[0]*data[0]);

	if (sinHalfAngle < 0.000000001 && sinHalfAngle > -0.000000001)
	{
		// do nothing because the rotation angle is zero, keep the original axis
	}
	else
	{
		axis[0] = data[1]/sinHalfAngle;
		axis[1] = data[2]/sinHalfAngle;
		axis[2] = data[3]/sinHalfAngle;
	}


}

// Member functions class: Quaternion
void Quaternion::ShowQuaternion(void) // print the quaternion values in command window
{
	/******************************************************************
	input: void
	output: void

	Note:
	// �b command window �C�L�X�|������
	******************************************************************/
	printf("Quaternion = %f %f %f %f\n",data[0],data[1],data[2],data[3]);
}




// global functions for quaternion manipulation  ~function set of quaternion rotation~
void QuatRotMul(Quaternion* q1, Quaternion* q2, Quaternion* result) // quaternion multiplication
{
	/******************************************************************
	input: q1 ���䪺�|�����A q2 �k�䪺�|���� (���k�ۭ�)�A result ���G�x�s�bresult�|������
	output: void

	Note:
	// �|�������k�A�аѷӥ|�������k���� �άO�U����matlab code
	//// Matlab Code
	////function q3 = QuatMul(q1,q2)
	////v1 = q1(2:4);
	////v2 = q2(2:4);
	////q3 = [q1(1)*q2(1)-v1*v2' cross(v1,v2)+q1(1)*v2+q2(1)*v1];
	******************************************************************/

	double* data1 = q1->data;
	double* data2 = q2->data;
	double* res = result->data;

	res[0] = data1[0]*data2[0]-data1[1]*data2[1]-data1[2]*data2[2]-data1[3]*data2[3];
	Cross2Vec(data1+1,data2+1,res+1);
	res[1] += data1[0]*data2[1]+data2[0]*data1[1];
	res[2] += data1[0]*data2[2]+data2[0]*data1[2];
	res[3] += data1[0]*data2[3]+data2[0]*data1[3];

	result->ReCalFormData(); // �q�|�������Ϻ⨤�׻P����b

}

// global functions for quaternion manipulation  ~function set of quaternion rotation~
void QuatToMatT(Quaternion* q1, double* R1) // convert quaternion to rotation matrix 123
{
	/******************************************************************
	input: q1 ��J���|�����A
	output: R1 �۹���������x�}��transpose

	Note:
	// �|������x�}�A�аѷӥ|������x�}���� �άO�U����matlab code
	// Matlab code
	//function R = QuatToMat(q0)

    //   w = q0(1);
    //   x = q0(2);
    //   y = q0(3);
    //   z = q0(4);

    //R = [1-2*y^2-2*z^2  2*x*y-2*z*w    2*x*z+2*y*w;
    //     2*x*y+2*z*w    1-2*x^2-2*z^2    2*y*z-2*x*w;
    //     2*x*z-2*y*w    2*y*z+2*x*w    1-2*x^2-2*y^2];
	
	//Result:R��transpose
	******************************************************************/

	double w = q1->data[0];
	double x = q1->data[1];
	double y = q1->data[2];
	double z = q1->data[3];

	R1[0] = 1.0-2.0*y*y-2.0*z*z; R1[3] = 2.0*x*y-2.0*z*w;      R1[6] = 2.0*x*z+2.0*y*w;  
	R1[1] = 2.0*x*y+2.0*z*w;     R1[4] = 1.0-2.0*x*x-2.0*z*z;  R1[7] = 2.0*y*z-2.0*x*w;  
	R1[2] = 2.0*x*z-2.0*y*w;     R1[5] = 2.0*y*z+2.0*x*w;      R1[8] = 1.0-2.0*x*x-2.0*y*y;  

}

// global functions for quaternion manipulation  ~function set of quaternion rotation~
void QuatConj(Quaternion* q1, Quaternion* result) // find conjugate quaternion
{
	/******************************************************************
	input: q1 ��J���|�����A result ���G�x�s�bresult�|������
	output: void

	Note:
	// ���@�m�|���� �b�ϹL�� ���פ@��
	******************************************************************/

	result->data[0] = q1->data[0];
	result->data[1] = -q1->data[1];
	result->data[2] = -q1->data[2];
	result->data[3] = -q1->data[3];
	result->axis[0] = -q1->axis[0];
	result->axis[1] = -q1->axis[1];
	result->axis[2] = -q1->axis[2];
	result->angle = q1->angle;
}

// global functions for quaternion manipulation  ~function set of quaternion rotation~
void QuatToMat(Quaternion* q1, double* R1) // convert quaternion to rotation matrix
{
	/******************************************************************
	input: q1 ��J���|�����A
	output: R1 �۹���������x�}

	Note:
	// �|������x�}�A�аѷӥ|������x�}���� �άO�U����matlab code
	// Matlab code
	//function R = QuatToMat(q0)

    //   w = q0(1);
    //   x = q0(2);
    //   y = q0(3);
    //   z = q0(4);

    //R = [1-2*y^2-2*z^2  2*x*y-2*z*w    2*x*z+2*y*w;
    //     2*x*y+2*z*w    1-2*x^2-2*z^2    2*y*z-2*x*w;
    //     2*x*z-2*y*w    2*y*z+2*x*w    1-2*x^2-2*y^2];
	******************************************************************/

	double w = q1->data[0];
	double x = q1->data[1];
	double y = q1->data[2];
	double z = q1->data[3];

	R1[0] = 1.0-2.0*y*y-2.0*z*z; R1[1] = 2.0*x*y-2.0*z*w;      R1[2] = 2.0*x*z+2.0*y*w;  
	R1[3] = 2.0*x*y+2.0*z*w;     R1[4] = 1.0-2.0*x*x-2.0*z*z;  R1[5] = 2.0*y*z-2.0*x*w;  
	R1[6] = 2.0*x*z-2.0*y*w;     R1[7] = 2.0*y*z+2.0*x*w;      R1[8] = 1.0-2.0*x*x-2.0*y*y;  

}

// global functions for quaternion manipulation  ~function set of quaternion rotation~
void QuatFromMat(double* R1, Quaternion* q1) // convert rotation matrix to quaternion
{
	/******************************************************************
	input: R1 ��J������x�}�A
	output: q1 �۹������|����

	Note:
	// �x�}��|�����A�аѷӯx�}��|�������� �άO�U����matlab code
	// Matlab code
	//function q0 = QuatFromMat(R)

   //   % Q = [1-2y^2-2z^2  2xy-2zw  2xz+2yw
   //   %      2xy+2zw 1-x^2-2z^2 2yz-2xw
   //   %      2xz-2yw 2yz+2xw 1-2x^2-2y^2]

   //   w = 0.5*sqrt(1+R(1,1)+R(2,2)+R(3,3)); % 0.5sqrt(trace+1)
   //   x = 0.5*sqrt(1+R(1,1)-R(2,2)-R(3,3))*sign(R(3,2)-R(2,3));
   //   y = 0.5*sqrt(1-R(1,1)+R(2,2)-R(3,3))*sign(R(1,3)-R(3,1));
   //   z = 0.5*sqrt(1-R(1,1)-R(2,2)+R(3,3))*sign(R(2,1)-R(1,2));

   //   q0 = [w x y z];
	******************************************************************/

	double tempValue;

	tempValue = 1+R1[0]+R1[4]+R1[8];
	if (tempValue < 0)
		q1->data[0] = 0.0;
	else
		q1->data[0] = 0.5*sqrt(tempValue);


    if (R1[7]-R1[5] >= 0)
	{
		tempValue = 1+R1[0]-R1[4]-R1[8];
		if (tempValue < 0)
			q1->data[1] = 0;
		else
			q1->data[1] = 0.5*sqrt(tempValue);
	}
	else
	{
		tempValue = 1+R1[0]-R1[4]-R1[8];
		if (tempValue < 0)
			q1->data[1] = 0;
		else
			q1->data[1] = -0.5*sqrt(tempValue);
	}

    if (R1[2]-R1[6] >= 0)
	{
		tempValue = 1-R1[0]+R1[4]-R1[8];
		if (tempValue < 0)
			q1->data[2] = 0;
		else
			q1->data[2] = 0.5*sqrt(tempValue);
	}
	else
	{
		tempValue = 1-R1[0]+R1[4]-R1[8];
		if (tempValue < 0)
			q1->data[2] = 0;
		else
			q1->data[2] = -0.5*sqrt(tempValue);
	}

    if (R1[3]-R1[1] >= 0)
	{
		tempValue = 1-R1[0]-R1[4]+R1[8];
		if (tempValue < 0)
			q1->data[3] = 0;
		else
			q1->data[3] = 0.5*sqrt(tempValue);
	}
	else
	{
		tempValue = 1-R1[0]-R1[4]+R1[8];
		if (tempValue < 0)
			q1->data[3] = 0;
		else
			q1->data[3] = -0.5*sqrt(tempValue);
	}

	q1->ReCalFormData();

}

// global functions for quaternion manipulation  ~function set of quaternion rotation~
void QuatVectorRotation(double* v1, Quaternion* q1, double* result) // rotate a vector in space using qutaternion method
{	
	/******************************************************************
	input: v1 ��J���V�q�A�|�Q�|����q1�ұ���,  q1 ����v1���|����, ���G�s�b result��
	output: void

	Note:
	// �Q�Υ|�����N�@��J�V�q����
	******************************************************************/

	Quaternion TempQ;
	Quaternion ConjQ;
	Quaternion v;
	// create a dummy quaternion for the vector
	// v' = q v conj(q)
	v.data[1] = v1[0];
	v.data[2] = v1[1];
	v.data[3] = v1[2];
	// v.SetAngle(0.0);

	QuatConj(q1,&ConjQ);
	QuatMul(q1,&v,&TempQ);
	QuatMul(&TempQ,&ConjQ,&v); // �b�o�@�� v ��@�Ȧs���x�s����᪺ quaternion 

	result[0] = v.data[1];
	result[1] = v.data[2];
	result[2] = v.data[3];

}

//// SLERP �y�Τ���
void QuatInitSLERP(Quaternion* q1, Quaternion* q2)
{
	/******************************************************************
	input: q1 �}�l�|���� q2 �����|���� 
	output: void

	Note:
	// ���F�קK���ư����l�ƪ��ʧ@ �ҥH�N���t�P��l�Ʃ�}
	// �|�����N�� q1 ������ q2
	******************************************************************/

	for (int i = 0 ; i < 4 ; i++)
	{
		qa.data[i] = q1->data[i];
		qb.data[i] = q2->data[i];
	}

	qa.angle = q1->angle;

	for (int i = 0 ; i < 3 ; i++)
	{
		qa.axis[i] = q1->axis[i];
		qb.axis[i] = q2->axis[i];
	}

	QuatCosHalfTh = 0.0;
	for (int i = 0; i < 4 ; i++)
	{
		QuatCosHalfTh += q1->data[i]*q2->data[i]; // something like dot product of two quaternions
	}
	QuatHalfTh = acos(QuatCosHalfTh);
	if (QuatHalfTh > 3.1415926/2.0) // �b�����i�W�L90��
	{
		QuatHalfTh -= 3.1415926;
	}
	else if (QuatHalfTh < -3.1415926/2.0)
	{
		QuatHalfTh += 3.1415926;
	}
	QuatSinHalfTh = sin(QuatHalfTh);

}

// global functions for quaternion manipulation  ~function set of quaternion rotation~
void QuatSLERP(double t, Quaternion* result) // 0 <= t <= 1
{
	/******************************************************************
	input: t �п�J0~1 �N���t���ʤ���, ���G�s�b result�� (�ؼЬ�: q1 �}�l�|���� q2 �����|���� )
	output: void

	Note:
	// �h�������Jt�q0~1 �N�i�H�o�줺���|�����ǦC
	// �U���O�d��

	// Quaternion SLERP
	// qm = (q1*sin((1-t)*th)+q2*sin(t*th))/s2th; �������аѷ�SLERP����
	// �p�������i�H�F�쵥���t�ת��ĪG
	// ��������k���P�󪽱����quaternion�����׭Ƚu�ʤ���
	// �]���P�� �� 2.0*acos(w) �u�ʤ���


	//////////////////EXAMPLE//////////////////////
	//Quaternion q1;
	//Quaternion q2;
	//Quaternion q3;

	//q1.data[0] = 0.8377; q1.data[1] =    0.3817 ; q1.data[2] =   0.0008  ; q1.data[3] =  0.3906;
	//q2.data[0] = 0.8969; q2.data[1] =    0.2076 ; q2.data[2] =   -0.3923  ; q2.data[3] =  0.0095;

	//QuatInitSLERP( &q1, &q2);
	//QuatSLERP(0.3,&q3);
	//q3.ShowQuaternion();

	//////////////////EXAMPLE//////////////////////

	******************************************************************/

	// �`�Nsin cos���Ȱ� �H�γQ���ϤT������� ���U��z�Ѫ��z�N�q
	if (QuatSinHalfTh < 0.000000001) // �N���� 0 , �]���O�G�����@�ਤ�A�ҥH���|���W�L90�ת����D �i�H�w�ߥΡA�]���|�o��sin(pi) = 0
	{
		// �@�w�۵��A�ҥH������Jq2
		for (int i = 0; i < 4 ; i++)
		{
			result->data[i] = qb.data[i];
		}
	}
	else
	{
		QuatScale1 = sin((1-t)*QuatHalfTh)/QuatSinHalfTh;
		QuatScale2 = sin(t*QuatHalfTh)/QuatSinHalfTh;
		for (int i = 0; i < 4 ; i++)
		{
			result->data[i] = qa.data[i]*QuatScale1+qb.data[i]*QuatScale2;
		}
	}
	result->ReCalFormData();

}

void QuatInitMatrixSLERP(double* R1, double* R2)
{
	/******************************************************************
	input: R1 �}�l����x�} R2 ��������x�} 
	output: void

	Note:
	// ���F�קK���ư����l�ƪ��ʧ@ �ҥH�N���t�P��l�Ʃ�}
	// ����x�}�N�� R1 ������ R2
	******************************************************************/
	for (int i = 0 ; i < 9 ; i++)
	{
		RotMa[i] = R1[i];
		RotMb[i] = R2[i];
	}

	MatMulABt(R2,3,3,R1,3,3,RotMbat); // calculate Rb*Ra'
	QuatFromMat(RotMbat,&QuatRotM);
	QuatCosHalfTh = QuatRotM.data[0];

	QuatHalfTh = acos(QuatRotM.data[0]);

	if (QuatHalfTh > 3.1415926/2.0) // �b�����i�W�L90��
	{
		QuatHalfTh -= 3.1415926;
	}
	else if (QuatHalfTh < -3.1415926/2.0)
	{
		QuatHalfTh += 3.1415926;
	}

	if (QuatCosHalfTh  >= 0.99999999) // no rotation
	{
		// do nothing
	}
	else
	{
		 QuatSinHalfTh = sqrt(1-QuatCosHalfTh*QuatCosHalfTh);
	     QuatAxis[0] = QuatRotM.data[1]/QuatSinHalfTh;
	     QuatAxis[1] = QuatRotM.data[2]/QuatSinHalfTh;
	     QuatAxis[2] = QuatRotM.data[3]/QuatSinHalfTh;
	}

}

// global functions for quaternion manipulation  ~function set of quaternion rotation~
void QuatMatrixSLERP(double t, double* result) // 0 <= t <= 1
{
	/******************************************************************
	input: t �п�J0~1 �N���t���ʤ���, ���G�s�b result�� (�ؼЬ�: R1 -> R2 )
	output: void

	Note:
	// �h�������Jt�q0~1 �N�i�H�o�줺������x�}�ǦC
	// �U���O�d��

	// Rotation Matrix SLERP

	//////////////////////EXAMPLE/////////////////////
	//double R1[9];
	//R1[0] = 0.9043; R1[1] = 0.1852; R1[2] = -0.3847; 
	//R1[3] = -0.4257; R1[4] = 0.3208; R1[5] = -0.8461; 
	//R1[6] = -0.0333; R1[7] = 0.9289; R1[8] = 0.3689; 
	//double R2[9];
	//R2[0] = 0.8401; R2[1] = -0.4517; R2[2] = 0.3003; 
	//R2[3] = -0.5055; R2[4] = -0.8527; R2[5] = 0.1315; 
	//R2[6] = 0.1967; R2[7] = -0.2623; R2[8] =-0.9447; 
	//double R3[9];

	//QuatInitMatrixSLERP(R1,R2);
	//QuatMatrixSLERP(0.3,R3);
	//////////////////////EXAMPLE/////////////////////

	******************************************************************/

	if (QuatCosHalfTh  >= 0.99999999)
	{
		for (int i  = 0 ; i < 9 ; i++)
			result[i] = RotMb[i];
	}
	else
	{
		QuatRotMSLERP.data[0] = cos(t*QuatHalfTh);
		QuatRotMSLERP.data[1] = QuatAxis[0]*sin(t*QuatHalfTh);
		QuatRotMSLERP.data[2] = QuatAxis[1]*sin(t*QuatHalfTh);
		QuatRotMSLERP.data[3] = QuatAxis[2]*sin(t*QuatHalfTh);
		//QuatRotMSLERP.ReCalFormData();

		QuatToMat(&QuatRotMSLERP,RotMSLERP);
		MatMulAB(RotMSLERP,3,3,RotMa,3,3,result);
	}

}

// cross two doubles
void Cross2Vec(double* v1, double* v2, double* v3)
{
	/******************************************************************
	input: v1 ����V�q v2�k��V�q v3 = v1 x v2 
	output: void

	Note:
	// �V�q�~�n
	******************************************************************/
	v3[0] = v1[1]*v2[2]-v1[2]*v2[1];
	v3[1] = v1[2]*v2[0]-v1[0]*v2[2];
	v3[2] = v1[0]*v2[1]-v1[1]*v2[0];
}

// global functions for quaternion manipulation  ~function set of quaternion manipulation~
void QuatMul(Quaternion* q1, Quaternion* q2, Quaternion* result) // quaternion multiplication
{
	/******************************************************************
	input: q1 ���䪺�|�����A q2 �k�䪺�|���� (���k�ۭ�)�A result ���G�x�s�bresult�|������
	output: void

	Note:
	// �|�������k�A�аѷӥ|�������k���� �άO�U����matlab code
	//// Matlab Code
	////function q3 = QuatMul(q1,q2)
	////v1 = q1(2:4);
	////v2 = q2(2:4);
	////q3 = [q1(1)*q2(1)-v1*v2' cross(v1,v2)+q1(1)*v2+q2(1)*v1];

	 (modified version : result = q1 or q2 =>OK)
	 �޼ƥi�H�Moutput�ۦP�Aelement���|���e�Q�л\�ɭP�B����~
	******************************************************************/

	double* data1 = q1->data;
	double* data2 = q2->data;
	double* res = result->data;

	double buffer[4];
	buffer[0] = data1[0]*data2[0]-data1[1]*data2[1]-data1[2]*data2[2]-data1[3]*data2[3];
	Cross2Vec(data1+1,data2+1,buffer+1);
	buffer[1] += data1[0]*data2[1]+data2[0]*data1[1];
	buffer[2] += data1[0]*data2[2]+data2[0]*data1[2];
	buffer[3] += data1[0]*data2[3]+data2[0]*data1[3];

	for(int i=0;i<4;i++)
		res[i]=buffer[i];

	//result->ReCalFormData(); // �q�|�������Ϻ⨤�׻P����b

}

// global functions for quaternion manipulation  ~function set of pure quaternion manipulation~
void QuatMulPureQ(Quaternion* q1, Quaternion* q2, Quaternion* result) // quaternion multiplication for pure quaternions 
{
	/******************************************************************
	input: q1 ���䪺�|�����A q2 �k�䪺�|���� (���k�ۭ�)�A result ���G�x�s�bresult�|������
	output: void

	Note:
	// �|�������k�A�аѷӥ|�������k���� �άO�U����matlab code
	//// Matlab Code
	////function q3 = QuatMul(q1,q2)
	////v1 = q1(2:4);
	////v2 = q2(2:4);
	////q3 = [q1(1)*q2(1)-v1*v2' cross(v1,v2)+q1(1)*v2+q2(1)*v1];

	conditions of pure quaternions : q1(1) = q2(1) = 0

	(modified version : result = q1 or q2 =>OK)
	�޼ƥi�H�Moutput�ۦP�Aelement���|���e�Q�л\�ɭP�B����~
	******************************************************************/

	double* data1 = q1->data;
	double* data2 = q2->data;
	double* res = result->data;

	double buffer[4];
	buffer[0] = -data1[1]*data2[1]-data1[2]*data2[2]-data1[3]*data2[3];
	Cross2Vec(data1+1,data2+1,buffer+1);
	buffer[1] += data1[0]*data2[1];
	buffer[2] += data1[0]*data2[2];
	buffer[3] += data1[0]*data2[3];

	for(int i=0;i<4;i++)
		res[i]=buffer[i];

	//result->ReCalFormData(); // �q�|�������Ϻ⨤�׻P����b

}

// global functions for quaternion manipulation  ~function set of quaternion manipulation~
void QuatAdd(Quaternion* q1, Quaternion* q2, Quaternion* result) // quaternion addision
{
	/******************************************************************
	input: q1 ���䪺�|�����A q2 �k�䪺�|���� (���k�ۥ[)�A result ���G�x�s�bresult�|������
	output: void

	Note:
	// �|�����[�k�A�p�P�@��V�q�[�k

	�޼ƥi�Moutput�ۦP
	******************************************************************/

	double* data1 = q1->data;
	double* data2 = q2->data;
	double* res = result->data;

	double buffer[4];
	buffer[0] = data1[0]+data2[0];
	buffer[1] = data1[1]+data2[1];
	buffer[2] = data1[2]+data2[2];
	buffer[3] = data1[3]+data2[3];

	for(int i=0;i<4;i++)
		res[i]=buffer[i];

}

// global functions for quaternion manipulation  ~function set of quaternion manipulation~
void QuatMulConjQ(Quaternion* q1, Quaternion* q2, Quaternion* result) // quaternion multiplication: q1*conj(q2) (modified version : result = q1 or q2 =>OK)
{
	/******************************************************************
	input: q1 ���䪺�|�����A q2 �k�䪺�|���� (q2��conj�ᥪ�k�ۭ�)�A result ���G�x�s�bresult�|������
	output: void

	Note:
	// �|�������k�A�аѷӥ|�������k���� �άO�U����matlab code
	//// Matlab Code
	////function q3 = QuatMul(q1,q2)
	////v1 = q1(2:4);
	////v2 = q2(2:4);
	////q3 = [q1(1)*q2(1)-v1*v2' cross(v1,v2)+q1(1)*v2+q2(1)*v1];

	(modified version : result = q1 or q2 =>OK)
	�޼ƥi�H�Moutput�ۦP�Aelement���|���e�Q�л\�ɭP�B����~
	******************************************************************/

	double* data1 = q1->data;
	double* data2 = q2->data;
	double* res = result->data;

	double buffer[4];

	for(int i=0;i<3;i++)
	data2[i+1]=data2[i+1]*(-1.0);

	buffer[0] = data1[0]*data2[0]-data1[1]*data2[1]-data1[2]*data2[2]-data1[3]*data2[3];
	Cross2Vec(data1+1,data2+1,buffer+1);
	buffer[1] += data1[0]*data2[1]+data2[0]*data1[1];
	buffer[2] += data1[0]*data2[2]+data2[0]*data1[2];
	buffer[3] += data1[0]*data2[3]+data2[0]*data1[3];

	for(int i=0;i<3;i++)
	data2[i+1]=data2[i+1]*(-1.0);
	for(int i=0;i<4;i++)
		res[i]=buffer[i];
	//result->ReCalFormData(); // �q�|�������Ϻ⨤�׻P����b

}