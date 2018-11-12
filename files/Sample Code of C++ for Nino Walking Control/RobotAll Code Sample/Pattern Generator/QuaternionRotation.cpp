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
     本程式主要用在建立四元素法物件
	 目標在於達成利用以四元素法為媒介來執行空間中旋轉
	 各函式與變數之說明請詳見下方宣告與定義處之說明

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
double RotMSLERP[9]; // 內插過程的旋轉矩陣
Quaternion QuatRotM; // 暫存變數
Quaternion QuatRotMSLERP; // 暫存變數
double QuatAxis[3]; // 暫存變數

// Member functions class: Quaternion
Quaternion::Quaternion(void) // constructor
{
	/******************************************************************
	input: void
	output: void

	Note:
	// Class constructor 初始化變數與記憶體分配
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
	// Class destructor 刪除動態記憶體
	******************************************************************/

	delete[] data;
	delete[] axis;
	// wp 因為 = data 所以不用再行 delete wp
}

// Member functions class: Quaternion
void Quaternion::Normalize(void) // normailze the quaternion
{
	/******************************************************************
	input: void
	output: void

	Note:
	// Normalize當前四元素物件
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
	// 回傳當前四元素之旋轉角度
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
	// 回傳當前四元素之旋轉軸
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
	// 回傳當前四元素轉軸儲存位址指標
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
	// 複製當前四元素之旋轉軸值到輸入指標所指之記憶體中
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
	// 設定當前四元素之轉角 並且自動運算相對應的改變(包含四元素值改變與轉角值複寫)
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
	// 設定四元素轉軸，並且自動normalize該轉軸向量後存到四元素中
	******************************************************************/
	double norm = sqrt(x*x+y*y+z*z);

	if (norm == 0)
	{
		printf("輸入軸為零向量，請檢察程式錯誤\n");
		system("pause");
	}


	axis[0] = x/norm;
	axis[1] = y/norm;
	axis[2] = z/norm;

	// data[0] 轉角 不用改變
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
	// 取出四元素的第一個值(w = cos(theta/2.0))
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
	// 設定四元素的第一個值(w = cos(theta/2.0)) 並且改變相對應之四元素部分
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
	// 改自 void Quaternion::SetAxis
	// 直接設定四元素值 (非unit quaternion可用)
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
	// 假若外部有強制改寫四元素值，請利用此函式重新整理四元素內部資料 包含 轉角 轉軸 四元素值的部分
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
	// 在 command window 列印出四員素值
	******************************************************************/
	printf("Quaternion = %f %f %f %f\n",data[0],data[1],data[2],data[3]);
}




// global functions for quaternion manipulation  ~function set of quaternion rotation~
void QuatRotMul(Quaternion* q1, Quaternion* q2, Quaternion* result) // quaternion multiplication
{
	/******************************************************************
	input: q1 左邊的四元素， q2 右邊的四元素 (左右相乘)， result 結果儲存在result四元素中
	output: void

	Note:
	// 四元素乘法，請參照四元素乘法公式 或是下面的matlab code
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

	result->ReCalFormData(); // 從四元素中反算角度與旋轉軸

}

// global functions for quaternion manipulation  ~function set of quaternion rotation~
void QuatToMatT(Quaternion* q1, double* R1) // convert quaternion to rotation matrix 123
{
	/******************************************************************
	input: q1 輸入的四元素，
	output: R1 相對應的旋轉矩陣的transpose

	Note:
	// 四元素轉矩陣，請參照四元素轉矩陣公式 或是下面的matlab code
	// Matlab code
	//function R = QuatToMat(q0)

    //   w = q0(1);
    //   x = q0(2);
    //   y = q0(3);
    //   z = q0(4);

    //R = [1-2*y^2-2*z^2  2*x*y-2*z*w    2*x*z+2*y*w;
    //     2*x*y+2*z*w    1-2*x^2-2*z^2    2*y*z-2*x*w;
    //     2*x*z-2*y*w    2*y*z+2*x*w    1-2*x^2-2*y^2];
	
	//Result:R的transpose
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
	input: q1 輸入的四元素， result 結果儲存在result四元素中
	output: void

	Note:
	// 取共軛四元素 軸反過來 角度一樣
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
	input: q1 輸入的四元素，
	output: R1 相對應的旋轉矩陣

	Note:
	// 四元素轉矩陣，請參照四元素轉矩陣公式 或是下面的matlab code
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
	input: R1 輸入的旋轉矩陣，
	output: q1 相對應的四元素

	Note:
	// 矩陣轉四元素，請參照矩陣轉四元素公式 或是下面的matlab code
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
	input: v1 輸入的向量，會被四元素q1所旋轉,  q1 旋轉v1的四元素, 結果存在 result中
	output: void

	Note:
	// 利用四元素將一輸入向量旋轉
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
	QuatMul(&TempQ,&ConjQ,&v); // 在這一行 v 當作暫存區儲存旋轉後的 quaternion 

	result[0] = v.data[1];
	result[1] = v.data[2];
	result[2] = v.data[3];

}

//// SLERP 球形內插
void QuatInitSLERP(Quaternion* q1, Quaternion* q2)
{
	/******************************************************************
	input: q1 開始四元素 q2 結束四元素 
	output: void

	Note:
	// 為了避免重複執行初始化的動作 所以將內差與初始化拆開
	// 四元素將由 q1 內插到 q2
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
	if (QuatHalfTh > 3.1415926/2.0) // 半角不可超過90度
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
	input: t 請輸入0~1 代表內差的百分比, 結果存在 result中 (目標為: q1 開始四元素 q2 結束四元素 )
	output: void

	Note:
	// 多次執行輸入t從0~1 就可以得到內插四元素序列
	// 下面是範例

	// Quaternion SLERP
	// qm = (q1*sin((1-t)*th)+q2*sin(t*th))/s2th; 此公式請參照SLERP公式
	// 如此內插可以達到等角速度的效果
	// 此內插方法等同於直接對於quaternion的角度值線性內插
	// 也等同於 對 2.0*acos(w) 線性內插


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

	// 注意sin cos的值域 以及被取反三角的函數 有助於理解物理意義
	if (QuatSinHalfTh < 0.000000001) // 代表接近 0 , 因為是二分之一轉角，所以不會有超過90度的問題 可以安心用，也不會發生sin(pi) = 0
	{
		// 一定相等，所以直接輸入q2
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
	input: R1 開始旋轉矩陣 R2 結束旋轉矩陣 
	output: void

	Note:
	// 為了避免重複執行初始化的動作 所以將內差與初始化拆開
	// 旋轉矩陣將由 R1 內插到 R2
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

	if (QuatHalfTh > 3.1415926/2.0) // 半角不可超過90度
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
	input: t 請輸入0~1 代表內差的百分比, 結果存在 result中 (目標為: R1 -> R2 )
	output: void

	Note:
	// 多次執行輸入t從0~1 就可以得到內插旋轉矩陣序列
	// 下面是範例

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
	input: v1 左邊向量 v2右邊向量 v3 = v1 x v2 
	output: void

	Note:
	// 向量外積
	******************************************************************/
	v3[0] = v1[1]*v2[2]-v1[2]*v2[1];
	v3[1] = v1[2]*v2[0]-v1[0]*v2[2];
	v3[2] = v1[0]*v2[1]-v1[1]*v2[0];
}

// global functions for quaternion manipulation  ~function set of quaternion manipulation~
void QuatMul(Quaternion* q1, Quaternion* q2, Quaternion* result) // quaternion multiplication
{
	/******************************************************************
	input: q1 左邊的四元素， q2 右邊的四元素 (左右相乘)， result 結果儲存在result四元素中
	output: void

	Note:
	// 四元素乘法，請參照四元素乘法公式 或是下面的matlab code
	//// Matlab Code
	////function q3 = QuatMul(q1,q2)
	////v1 = q1(2:4);
	////v2 = q2(2:4);
	////q3 = [q1(1)*q2(1)-v1*v2' cross(v1,v2)+q1(1)*v2+q2(1)*v1];

	 (modified version : result = q1 or q2 =>OK)
	 引數可以和output相同，element不會提前被覆蓋導致運算錯誤
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

	//result->ReCalFormData(); // 從四元素中反算角度與旋轉軸

}

// global functions for quaternion manipulation  ~function set of pure quaternion manipulation~
void QuatMulPureQ(Quaternion* q1, Quaternion* q2, Quaternion* result) // quaternion multiplication for pure quaternions 
{
	/******************************************************************
	input: q1 左邊的四元素， q2 右邊的四元素 (左右相乘)， result 結果儲存在result四元素中
	output: void

	Note:
	// 四元素乘法，請參照四元素乘法公式 或是下面的matlab code
	//// Matlab Code
	////function q3 = QuatMul(q1,q2)
	////v1 = q1(2:4);
	////v2 = q2(2:4);
	////q3 = [q1(1)*q2(1)-v1*v2' cross(v1,v2)+q1(1)*v2+q2(1)*v1];

	conditions of pure quaternions : q1(1) = q2(1) = 0

	(modified version : result = q1 or q2 =>OK)
	引數可以和output相同，element不會提前被覆蓋導致運算錯誤
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

	//result->ReCalFormData(); // 從四元素中反算角度與旋轉軸

}

// global functions for quaternion manipulation  ~function set of quaternion manipulation~
void QuatAdd(Quaternion* q1, Quaternion* q2, Quaternion* result) // quaternion addision
{
	/******************************************************************
	input: q1 左邊的四元素， q2 右邊的四元素 (左右相加)， result 結果儲存在result四元素中
	output: void

	Note:
	// 四元素加法，如同一般向量加法

	引數可和output相同
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
	input: q1 左邊的四元素， q2 右邊的四元素 (q2轉conj後左右相乘)， result 結果儲存在result四元素中
	output: void

	Note:
	// 四元素乘法，請參照四元素乘法公式 或是下面的matlab code
	//// Matlab Code
	////function q3 = QuatMul(q1,q2)
	////v1 = q1(2:4);
	////v2 = q2(2:4);
	////q3 = [q1(1)*q2(1)-v1*v2' cross(v1,v2)+q1(1)*v2+q2(1)*v1];

	(modified version : result = q1 or q2 =>OK)
	引數可以和output相同，element不會提前被覆蓋導致運算錯誤
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
	//result->ReCalFormData(); // 從四元素中反算角度與旋轉軸

}