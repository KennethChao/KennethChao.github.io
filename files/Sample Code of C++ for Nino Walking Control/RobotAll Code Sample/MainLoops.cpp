/**************************************************************************************************
Copyright, 2010-2012, Robotics Lab., Dept. of M.E., National Taiwan University
File Name: MainLoops.cpp

Author: Many People
Version: 1.0
Date: 2012/03/20

Functions:
     gInitialization() gOpenPort() gClosePort() gPreProcessData() gLoadfile() gInitializeKinematicsGL()
     gRenderSceneEmpty() gRenderSceneThread() gMouseFunc() gMousePressedMove() gMouseNotPressedMove() 
	 gOnSizeMyGL() gNorm2Pd() gNorm2Pf() gNorm1Pf() gCross2Vf() gCross2Vd() gRotYMat() gRotXMat()
	 gGLDrawLinePoint() C2MWrite2Txt() C2MLoadTraj()

Classes: None

Description:
     本程式是整個程式的控制核心，管理了openGL的事件
	 COM Part通訊 以及機器人的初始化與控制等等
	 各函式與變數之說明請詳見下方宣告與定義處之說明

Note: None
***************************************************************************************************/

#include "stdafx.h"
#include "MainLoops.h"
#include <stdio.h>
#include "GLModel.h"//dora
#include <sstream>
GLModel gGLModel;//dora

#if TwinCAT_Mode
#include "TwinCAT_COM.h"
extern TwinCAT_COM *TCAT;
#endif

bool gShutDownFlag=false; //130306

extern bool gFlagGLMode;//dora
extern bool gFlagPE;//slongz

extern int gFlagReadForceSensor;
extern int gFlagInfrared;


// 模擬是否執行旗標
	extern bool gFlagArmCtrl;
	extern bool gFlagHandCtrl; // 至峻 20130410
	extern int gFlagSimulation;
	//extern bool gFlagSkinModule;

//  global flag
	bool gStartSensing = false; // 確認是壓下 start
	bool gSendContTraj = false; // true時可傳送控制腳的連續軌跡
	bool gSendContTrajArm = false; // true時可傳送控制手的連續軌跡

// loading trajectory 
	fstream gfLALen; // 左手軌跡檔案長度
	fstream gfRALen; // 右手軌跡檔案長度
	fstream gfTorsoLen;  // 腰軌跡檔案長度
	fstream gfLATraj; // 左手軌跡儲存記憶體
	fstream gfRATraj; // 右手軌跡儲存記憶體
	fstream gfTorsoTraj;// 腰軌跡儲存記憶體

	int gContTrajLen = 0; // 腳的總軌跡長度
	int gStopTrajLen = 0; // 腳要停下來的軌跡是在第gStopTrajLen個的位置

	extern bool gContTrajLock; // 要先利用start 跟 auto 兩顆按鈕解鎖 不然 不接受連續軌跡輸出
	extern bool gContTrajLockArm; // 要先利用start 跟 auto 兩顆按鈕解鎖 不然 不接受連續軌跡輸出
	extern 	LQSISolver gLQs; // LQSI solver 物件宣告
	extern Kine gKineAll; // 機器人全身 kinematics trains 宣告

	bool gSetPIDDone = false; // 設定完

// ========================================
	// Kinematics Drawing Variables

	int gGLID = 0; // 創建GL視窗時回傳的ID 要殺掉這個視窗的時候要使用此ID
	bool gIKGLOpened = false; // 旗標GL初始化後會變成true
	double gViewCenterInit[3]; // GL視窗中心點初始值
	double gEyeCenterInit[3]; // GL攝影位置初始值
	double gCamUpInit[3]; // GL攝影機上方向量初始值

	GLfloat gLenViewDist; // GL視界距離

	bool gLeftMouse = false; // 左鍵是否壓下
	bool gRightMouse = false; // 右鍵是否壓下

	int gLastMx, gLastMy; // 最後一次感應到滑鼠的時候的 x y 的點
	float gViewRangeNear = 0.1; // GL視界近點距離
	float gViewRangeFar = 10000.0; // GL視界遠點距離
	float gEyeDistance = 2000.0; // GL攝影機距離
	float gMoveSensivity = gEyeDistance*0.004; // 滑鼠靈敏度
	float gRotSensivity = 0.008; // 旋轉靈敏度
	float gScaleSensivity = 0.95; // 拉伸靈敏度

	YMatLite gRotMat(3,3); // 暫存旋轉矩陣
	YMatLite gTempM(3,3); // 暫存旋轉矩陣 
	YMatLite gTempMRes(3,3); // 暫存旋轉矩陣

	GLfloat gViewCenter[3]; // GL視窗中心點
	GLfloat gEyeCenter[3]; // GL攝影位置
	GLfloat gVisionLine[3]; // GL視線
	GLfloat gCamUp[3];// GL攝影機上方向量

	HANDLE gThreadRender; // 畫圖thread handle
    DWORD gTIDRender; // 畫圖thread ID
	bool gRenderLife = true; // 是否結束繪圖視窗

	HANDLE gThreadControl; // 控制腳thread handle
    DWORD gTIDControl; // 控制腳thread ID
	bool gControlLife = true; // 是否結束機器人控制

	HANDLE gThreadFace; // face control thread handle
    DWORD gTIDFace;// face control control thread ID
	bool gFaceLife = true; // face control control 是否工作旗標

	HANDLE gThreadArmControl; // 控制手thread handle
    DWORD gTIDArmControl; // 控制手thread ID
	bool gArmControlLife = true; // 是否結束機器人控制

	HANDLE gThreadRenderPMS;// 繪圖PMS thread handle
    DWORD gTIDRenderPMS;// 繪圖PMS thread ID
	bool gRenderPMSLife = true;// 繪圖PMSthread是否結束

	// Slongz 0218  //130306
	HANDLE gThreadTimer; // 控制Timer thread handle
    DWORD gTIDTimer; // 控制Timer thread ID
	bool gTimerLife = true; // 是否結束Timer
	// Slongz 0218

	//_______________________________________(Dora)_________________________________________
	#if LaserCatch
	HANDLE gThreadLaserRender; // 畫圖thread handle
    DWORD gTIDLaserRender; // 畫圖thread ID
	bool gLaserRenderLife = true; // 是否結束繪圖視窗

	extern	int laserbufX[681];
	extern 	int laserbufY[681];
	#endif LaserCatch
	//_______________________________________(Dora)_________________________________________

	//_______________________________________(chehsuan 20130508)_________________________________________
	
//#if IMUCatch 
	HANDLE gThreadIMU; 
	DWORD gTIDIMU; 
	bool gIMULife = true; 
//#endif
	//_______________________________________(chehsuan 20130508)_________________________________________





	int gDrawingSeqSize = (13+13+1)*3; // 左腳13 右腳13 中間點1,  x3代表xyz軸
	//泓逸start111219
	//int DrawingSeqSizeArm = (4+4+1)*3; // 左手4 右手4 中間點1
	int gDrawingSeqSizeArm = (10+10+1)*3; // 左手10 右手10 中間點1
	//泓逸end111219
	float* gpRobotDrawingBuffer; // 繪圖暫存記憶體
	float* gpRobotDrawingBufferArm; // 繪圖暫存記憶體

	bool gDrawingBufCreated = false; // 繪圖buffer創建後旗標=true


	extern LARGE_INTEGER gStartTime; // 傳輸開始的時間點
	extern bool gStartTimeAcquired; // 看看是否有取得開始時間了

	extern char gGlobalMessage[60]; // GL視窗左上角print訊息

	extern bool gRenderKineWorking; // 防止thread互搶GL繪圖功能
	extern bool gRenderPMSWorking; // 防止thread互搶GL繪圖功能

	// Arm motions
	extern int gLAMotionIndex; // 記住動作撥放到第幾格
	extern int gRAMotionIndex; // 記住動作撥放到第幾格
	extern int gTorsoMotionIndex; // 記住動作撥放到第幾格
	extern int gLAMotionLen; // 軌跡總長
	extern int gRAMotionLen; // 軌跡總長
	extern int gTorsoMotionLen; // 軌跡總長
	extern unsigned char gLAMotion[30][LQSIBufferSize];  // 軌跡儲存區
	extern unsigned char gRAMotion[30][LQSIBufferSize];  // 軌跡儲存區
	extern unsigned char gTorsoMotion[30][LQSIBufferSize];  // 軌跡儲存區
	// Arm motions


	// 地板與ZMP資訊
	extern double gGroundHeight[10000]; // 每步地面高度
	extern double gFstpY[10000]; // 每步ZMP位置
	extern double gFstpX[10000]; // 每步ZMP位置
	extern unsigned int gIthIK; // 記憶IK解到第幾格
	extern double gInpZMPHeight[LQSIBufferSize]; // ZMP高度 用來算倒單擺高度用的
	extern bool gFlagZMPPlot; // GL是否繪製ZMP位置旗標


	//_______________________________________Added by Slongz_______________________________________
	#if CheckingMode 
	int tempIKindex=0;
	#endif
	extern int gStepSample;
	extern char gBipedInfo1[60]; // GL視窗左上角print訊息
	extern char gBipedInfo2[60]; // GL視窗左上角print訊息
	extern char gBipedInfo3[60]; // GL視窗左上角print訊息
	//_______________________________________Added by Slongz_______________________________________

DataProcess *gpDP ; // C32傳輸物件宣告
///SerialPort *gpPortLL; // 左腳PORT
//SerialPort *gpPortRL; // 右腳PORT
SerialPort *gpPortLA; // 左手PORT
SerialPort *gpPortRA; // 右手PORT
SerialPort *gpPortTorso; // TORSO PORT
SerialPort *gpPortHead; // Head Port

//SerialPort *gpPortLSkin; // 左手皮膚PORT
//SerialPort *gpPortRSkin; // 左手皮膚PORT

SerialPort *gpPortneck ; //脖子

//20130312 WeiZh
double CaliPitchAngL;
double CaliPitchAngR;
double CaliRollAngL;
double CaliRollAngR;
double CaliRollHipL;
double CaliRollHipR;
double CaliPitchHipL;
double CaliPitchHipR;
double CaliTemp[12];
//  因為在RobotAllDlg中含數放在while中 故CaliCountL CaliCountR CaliCount不可放在函數CoPCali中  
int CaliCountL = 0;
int CaliCountR = 0;
int CaliCount = 0;
//
extern double delta_ankle[12];
extern int axisNumber;
extern double CaliAngle;
//

//==========================================================// 至峻 20130410
DataProcess *DP ;
DataProcess * DP1;
SerialPort *gpPortLH;
SerialPort *gpPortRH;
//=======================================================
	fstream gfLen;
	fstream gfRH;
	fstream gfLH;
	int gfContTrajLenRH = 0;
	int gfContTrajLenLH = 0;
	unsigned char *gfContTrajDataRH;
	unsigned char *gfContTrajDataLH;
//=======================================================
	const unsigned int packet= 540; // axi number15 * 9 * 10
	unsigned char buffer[packet]; // Ontimer in buf
	#define IDR_LastCMD_Succ MID*CMD_SET+7 // 上一個指令完成，回傳告訴C32
	int commmode=0;
//==========================================================// 至峻 20130410

//==========================================================
// function descrition
//==========================================================
void gInitialization()
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 初始化各個COM PORT 以及物件初始化
	******************************************************************/
	if (gFlagSimulation == RealExp)
	{
		 ///gpPortLL = new SerialPort(); // LL
		 ///gpPortRL = new SerialPort(); // RL
		 gpPortLA = new SerialPort(); // LA
		 gpPortRA = new SerialPort(); // RA
		 gpPortTorso = new SerialPort(); // Torso
		 gpPortHead = new SerialPort(); // Head
		 gpPortLH = new SerialPort(); // LH 
		 gpPortRH = new SerialPort(); // RH
	}

	//if (gFlagSkinModule == true) // 假若要讀取皮膚則打開
	//{
	//	//gpPortLSkin = new SerialPort();
	//	gpPortRSkin = new SerialPort();
	//}

	gpDP = new DataProcess();
	gpDP->DPInit();
	gStartSensing = true;

	DP= new DataProcess(); // 至峻 20130410
	DP->DPInit();
	DP1= new DataProcess();
	DP1->DPInit(); // 至峻 20130410
}

void gInitializeKinematicsGL(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 初始化GL視窗
	******************************************************************/
 // Initialize GL window for Kinematics
	gViewCenterInit[0] = 0.0f; gViewCenterInit[1] = 600.0f; gViewCenterInit[2] = 0.0f; 
	gEyeCenterInit[0] = 0.0f; gEyeCenterInit[1] = 600.0f; gEyeCenterInit[2] = gEyeDistance;
	gCamUpInit[0] = 0.0f;	  gCamUpInit[1] = 1.0f;		gCamUpInit[2] =0.0f;	


	if (fabs((gEyeCenterInit[0]-gViewCenterInit[0])*gCamUpInit[0]+(gEyeCenterInit[1]-gViewCenterInit[1])*gCamUpInit[1]+(gEyeCenterInit[2]-gViewCenterInit[2])*gCamUpInit[2])<0.0001)
	{
		// do nothing, 視線跟攝影機上方向量有垂直，很好，可喜可賀
	}
	else
	{
		printf("視線與攝影機上方向量不垂直，會造成旋轉混亂，請注意或修改，這個小錯誤不影響FK IK等");
		system("pause");
	}

	// 攝影機參數
	gViewCenter[0] = gViewCenterInit[0];
	gViewCenter[1] = gViewCenterInit[1];
	gViewCenter[2] = gViewCenterInit[2];
	gEyeCenter[0] = gEyeCenterInit[0];
	gEyeCenter[1] = gEyeCenterInit[1];
	gEyeCenter[2] = gEyeCenterInit[2];
	gCamUp[0] = gCamUpInit[0];
	gCamUp[1] = gCamUpInit[1];
	gCamUp[2] = gCamUpInit[2];

	gRotMat.data[0] = 1;
	gRotMat.data[1] = 0;
	gRotMat.data[2] = 0;
	gRotMat.data[3] = 0;
	gRotMat.data[4] = 1;
	gRotMat.data[5] = 0;
	gRotMat.data[6] = 0;
	gRotMat.data[7] = 0;
	gRotMat.data[8] = 1;


	gLenViewDist = gNorm2Pf(gEyeCenter,gViewCenter);

	gLastMx = 0;
	gLastMy = 0;

	// 基本繪圖模式
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH); // double buffering + depth test

	glutInitWindowSize(640,640); // 創立 640 x 640 視窗
	glutInitWindowPosition(250,0);  // 設定視窗位置
	gGLID = glutCreateWindow("Kinematics View"); // 開始視窗


	//PhysicalEngineODE PEO;
	
	if(gFlagGLMode==1)//dora===========如要使用GL完整模型在此載入
		//if(gFlagPE)
		//	PEO.ODEMain();
		//else
			gGLModel.InitGLModel();
		

	glutReshapeFunc(gOnSizeMyGL);					// 當變更 Windows size 時呼叫 OnSize
	glutDisplayFunc(gRenderSceneEmpty); // 設定 display function 為空

	// 滑鼠事件
	glutMouseFunc(gMouseFunc); // 滑鼠壓下或放開
	glutMotionFunc(gMousePressedMove); // 滑鼠壓著移動
	glutPassiveMotionFunc(gMouseNotPressedMove); // 滑鼠沒被壓下時移動 

	// 光影與材料參數
	GLfloat LightDistance = 2000.0;
	GLfloat LightPosition1[] = { 1.0, 1.0, 1.0, 0.0 }; // x y z scale, scale = 0時 光源無窮遠
	//GLfloat LightPosition2[] = { 0, 0, LightDistance, 1.0 }; // x y z scale
    GLfloat MaterialSpecular[] = { 0.1, 0.1, 0.1, 1.0 };
    GLfloat MaterialAmbient[] = { 0.1, 0.1, 0.1, 1.0 };//ATI color enable 左上字顏色
    GLfloat MaterialDiffuse[] = { 0.1, 0.1, 0.1, 1.0 };
    GLfloat MaterialShininess[] = { 5.0 };
	GLfloat MaterialEmission[] = {0.0, 0.0, 0.0};
	GLfloat light_ambient[]  = { 1.0, 1.0, 1.0, 1.0};//dora
	GLfloat light_diffuse[]  = { 1.0, 1.0, 1.0, 1.0};//dora
	GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0}; //dora


	//glLightfv(GL_LIGHT0, GL_POSITION, LightPosition1);
	//glLightfv(GL_LIGHT1, GL_POSITION, LightPosition2);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, MaterialSpecular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, MaterialShininess);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, MaterialAmbient);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, MaterialDiffuse);
	glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, MaterialEmission);
	glLightfv( GL_LIGHT0, GL_POSITION, LightPosition1);//dora
	glLightfv( GL_LIGHT0, GL_AMBIENT, light_ambient);//dora
	glLightfv( GL_LIGHT0, GL_DIFFUSE, light_diffuse);//dora      //散射光(Diffuse Light)
	glLightfv( GL_LIGHT0, GL_SPECULAR,light_specular);//dora

	glEnable(GL_LIGHTING);
	glColorMaterial(GL_FRONT_AND_BACK, GL_EMISSION );
    glEnable(GL_COLOR_MATERIAL );
	glEnable(GL_LIGHT0);
	//glEnable(GL_LIGHT1);


	glEnable(GL_DEPTH_TEST);

	//GLfloat light_specular[] = {0.5, 0.5, 0.5, 1.0};
	//GLfloat light_diffuse[] = {0.5, 0.5, 0.5, 1.0};
	//GLfloat light_ambient[] = {0.5, 0.5, 0.5, 1.0};
	//GLfloat light_position[] = {12.0f, 8.0f, 42.0f, 1.0f};

	//glEnable (GL_LIGHTING);

	//// 設定發光體的光源的特性
	//glLightfv(GL_LIGHT0,GL_AMBIENT,light_ambient);		// 環境光(Ambient Light)
	//glLightfv(GL_LIGHT0,GL_DIFFUSE,light_diffuse);		// 散射光(Diffuse Light)
	//glLightfv(GL_LIGHT0,GL_SPECULAR,light_specular);	// 反射光(Specular Light)
	//glLightfv(GL_LIGHT0,GL_POSITION,light_position);	// 光的座標

 //   glEnable (GL_LIGHT0);


	//GLfloat material_ambient[] = {0.0, 0.0, 0.5, 1.0};
	//GLfloat material_diffuse[] = { 0.0, 0.0, 0.5, 1.0};
	//GLfloat material_specular[] = {0.0, 0.0, 0.5, 1.0};

	//glMaterialfv(GL_FRONT, GL_AMBIENT, material_ambient);
	//glMaterialfv(GL_FRONT, GL_DIFFUSE, material_diffuse);
	//glMaterialfv(GL_FRONT, GL_SPECULAR, material_specular);


	//glEnable(GL_COLOR_MATERIAL);

	/*glClearColor(0.0f,0.0f,0.0f,1.0f);
	glutSwapBuffers();
	glClearColor(0.0f,0.0f,0.0f,1.0f);*/
	glutSwapBuffers();

	gIKGLOpened = true;
	glutMainLoop();

}


//=============================================================
// Communication Command
//=============================================================
void gOpenPort()
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 開啟各個COM PORT
	******************************************************************/

	if(	 gStartSensing == true)
	{
		try
		{
		///gpPortLL->open("\\\\.\\COM3",NORMAL_RS232);
		///gpPortLL->_set_baudrate(115200);
		///gpPortRL->open("\\\\.\\COM4",NORMAL_RS232);

		///gpPortRL->_set_baudrate(115200);

		gpPortLA->open("\\\\.\\COM88",NORMAL_RS232);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
		gpPortLA->_set_baudrate(115200);
		gpPortRA->open("\\\\.\\COM89",NORMAL_RS232);
		gpPortRA->_set_baudrate(115200);
//======================================================================// 至峻 20130410
		gpPortRH->open("\\\\.\\COM91",NORMAL_RS232);					
		gpPortRH->_set_baudrate(115200);
		gpPortLH->open("\\\\.\\COM90",NORMAL_RS232);					
		gpPortLH->_set_baudrate(115200);
//=====================================================================// 至峻 20130410

		//gpPortneck->open("\\\\.\\COM5",NORMAL_RS232);
		//gpPortneck->_set_baudrate(115200);                         //哲軒20130412 已移至gInitneck



		//gpPortTorso->open("\\\\.\\COM11",NORMAL_RS232);
		//gpPortTorso->_set_baudrate(115200);
		//gpPortLSkin->open("\\\\.\\COM13",NORMAL_RS232);
		//gpPortLSkin->_set_baudrate(115200);

		gpPortHead->open("\\\\.\\COM6",NORMAL_RS232);
		gpPortHead->_set_baudrate(115200);


		//if(gFlagSkinModule == true)
		//{
		//	gpPortRSkin->open("\\\\.\\COM30",NORMAL_RS232);
		//	gpPortRSkin->_set_baudrate(115200);
		//}


		}
		catch(...)
		{
		cout<<"COM open faied"<<endl;
		}
	}
	else
		cout<<"please push initialize"<<endl;
}
void gClosePort()
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 關閉各個COM PORT
	******************************************************************/
	if(	 gStartSensing == true)
	{
		///gpPortLL->close();
		///gpPortRL->close();
		gpPortLA->close();
		gpPortRA->close();
		gpPortTorso->close();
		gpPortHead->close();
		gpPortRH->close();
		gpPortLH->close();
		cout<<"serial port closed"<<endl;
	}
	else
		cout<<"communication is not started yet"<<endl;

	//if (gFlagSkinModule == true)
	//{
	//	//gpPortLSkin ->close();
	//	gpPortRSkin ->close();
	//}

}

void gPreProcessData(unsigned int mode)
{
	/******************************************************************
	input: mode 選擇不同模式來跟機器人初始化通訊
	output: void

	Note:
	// 依照傳入的 "mode" 選擇要執行的功能
	// 這個函式是跟機器人溝通的最重要程式
	******************************************************************/
	if(	 gStartSensing == true)
	{
		int number1=0;
		int kp=0;int ki=0;int kd=0;int errorsumbound=0; float ang=0; long encoder=0;
		int timeout=0;
		int waittimes=80;

		// 設定哪些軸要Active
		//unsigned char AxisOnBus[13] = {253,0,1,1,1,1,1,0,1,1,1,1,1}; // 第一個是識別碼，從第二個開始是馬達各軸要不要active
		//                            // cmd,1,2,3,4,5,6,7,8,9,0,1,2
		unsigned char AxisOnBus[13] = {253,1,1,1,1,1,1,1,1,1,1,1,1}; // 第一個是識別碼，從第二個開始是馬達各軸要不要active 第1~12軸
									// cmd,1,2,3,4,5,6,7,8,9,0,1,2
		unsigned char AxisOnBus2[13] = {254,1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // 第一個是識別碼，從第二個開始是馬達各軸要不要active 第13~24軸
									 // cmd,13,14,15,16,17,18,19,20,21,22,23,24

		unsigned int NumOfTrajSample = 0;

		unsigned char UpperLim = 99;
		unsigned char LowerLim = 10;
		unsigned int  OkErrBound = 6000; // 800/2000/160/1.47*360= 0.61度

		int jointNum = 0;

		int CaliTotalCount = 50; // Calibriation切成50格 不然機器人會衝

		//for 電池0528
			double PIDRatio=0.9;
		//for 電池0528


		switch(mode)
		{
		case 0: // 機構歸home

			// not used now

			break; 

		case SetOnePID:
			//	SetPID();
			cout<<"PID Parameter Setting Mode"<<endl;
			
			// 設定軸數
			cout<<"Please Type the Axis Number You Want to Modify: ";
				scanf("%d", &number1); // %d 代表十進位整數

			if (number1 <= 12)
			{
				printf("Leg Axis Number = %d\n",number1);
			}
			else if (number1 == 13 || number1 == 14)
			{
				printf("Torso Axis Number = %d\n",number1-12);
			}
			else if (number1 >= 15 && number1 <= 20)
			{
				printf("Left Arm Axis Number = %d\n",number1-14);
			}
			else if (number1 >= 21 && number1 <= 26)
			{
				printf("Right Arm Axis Number = %d\n",number1-20);
			}
			else
			{
				printf("Undefined Axis Number, nothing will be sent.\n");
			}

			// 輸入PID值
			cout<<"Please Type the Kp: ";
				scanf("%d", &kp);
			cout<<"Please Type the Ki: ";
				scanf("%d", &ki);
			cout<<"Please Type the Kd: ";
				scanf("%d", &kd);
			cout<<"Please Type the Bound of ErrorSum: ";
				scanf("%d", &errorsumbound);

			// 將PID傳出
			if (number1 <= 12)
			{
				///gpDP->TxData[0]=ID_SetPID+CMD_SET*number1;
				///gpDP->TxData[1]=(unsigned char)kp;
				///gpDP->TxData[2]=(unsigned char)(kp>>8);
				///gpDP->TxData[3]=(unsigned char)ki;
				///gpDP->TxData[4]=(unsigned char)(ki>>8);
				///gpDP->TxData[5]=(unsigned char)kd;
				///gpDP->TxData[6]=(unsigned char)(kd>>8);
				///gpDP->TxData[7]=(unsigned char)errorsumbound;
				///gpDP->TxData[8]=(unsigned char)(errorsumbound>>8);
				///gpPortLL->_write(gpDP->TxData,9); 
				///gpPortRL->_write(gpDP->TxData,9); 
			}
			else if (number1 == 13 && number1 == 14) // torso
			{
				gpDP->TxData[0]=ID_SetPID+CMD_SET*(number1-12);
				gpDP->TxData[1]=(unsigned char)kp;
				gpDP->TxData[2]=(unsigned char)(kp>>8);
				gpDP->TxData[3]=(unsigned char)ki;
				gpDP->TxData[4]=(unsigned char)(ki>>8);
				gpDP->TxData[5]=(unsigned char)kd;
				gpDP->TxData[6]=(unsigned char)(kd>>8);
				gpDP->TxData[7]=(unsigned char)errorsumbound;
				gpDP->TxData[8]=(unsigned char)(errorsumbound>>8);
				gpPortTorso->_write(gpDP->TxData,9); 
			}
			else if (number1 >= 15 && number1 <= 20) // left arm
			{
				gpDP->TxData[0]=ID_SetPID+CMD_SET*(number1-14);
				gpDP->TxData[1]=(unsigned char)kp;
				gpDP->TxData[2]=(unsigned char)(kp>>8);
				gpDP->TxData[3]=(unsigned char)ki;
				gpDP->TxData[4]=(unsigned char)(ki>>8);
				gpDP->TxData[5]=(unsigned char)kd;
				gpDP->TxData[6]=(unsigned char)(kd>>8);
				gpDP->TxData[7]=(unsigned char)errorsumbound;
				gpDP->TxData[8]=(unsigned char)(errorsumbound>>8);
				gpPortLA->_write(gpDP->TxData,9);
				gpPortHead->_write(gpDP->TxData,9);
			}
			else if (number1 >= 21 && number1 <= 26) // right arm
			{
				gpDP->TxData[0]=ID_SetPID+CMD_SET*(number1-20);
				gpDP->TxData[1]=(unsigned char)kp;
				gpDP->TxData[2]=(unsigned char)(kp>>8);
				gpDP->TxData[3]=(unsigned char)ki;
				gpDP->TxData[4]=(unsigned char)(ki>>8);
				gpDP->TxData[5]=(unsigned char)kd;
				gpDP->TxData[6]=(unsigned char)(kd>>8);
				gpDP->TxData[7]=(unsigned char)errorsumbound;
				gpDP->TxData[8]=(unsigned char)(errorsumbound>>8);
				gpPortRA->_write(gpDP->TxData,9); 
			}

			break;

		case Manual:
			//	Manual();

			if (gSetPIDDone == false && gFlagArmCtrl==true)
			{
				cout << "先設定PID用以解鎖，不然無法連續控制" << endl;
			}
			else if (gFlagArmCtrl==false)
			{
				cout << "Arm/Hand Control: Closed" << endl;
				// do nothing

				gSendContTraj=true;
				gContTrajLock = false; // 解鎖
				gSendContTrajArm=true;
				gContTrajLockArm = false; // 解鎖
			}
			else
			{
				
				// 設定手動模式
				gpDP->State[2]=ID_SetOneAxis;
				cout<<"Please Type the Axi Number You Want to Modify: ";
					scanf("%d", &number1);

				if (number1 <= 12)
				{
					printf("Leg Axis Number = %d\n",number1);
					///gpPortLL->_write(gpDP->State,5);
					///gpPortRL->_write(gpDP->State,5);
				}
				else if (number1 == 13 || number1 == 14)
				{
					printf("Torso Axis Number = %d\n",number1-12);
					gpPortTorso->_write(gpDP->State,5);
				}
				else if (number1 >= 15 && number1 <= 20)
				{
					gpPortLA->_write(gpDP->State,5);
					printf("Left Arm Axis Number = %d\n",number1-14);
				}
				else if (number1 >= 21 && number1 <= 26)
				{
					gpPortRA->_write(gpDP->State,5);
					printf("Right Arm Axis Number = %d\n",number1-20);
				}
				else if (number1 >= 27 && number1 <= 32)
				{
					gpPortHead->_write(gpDP->State,5);
					printf("Head Axis Number = %d\n",number1-26);
				}
				else
				{
					printf("Undefined Axis Number, nothing will be sent.\n");
				}

				// 設定輸入角度
				cout<<"Please Assign the Turning Angle: ";
					scanf("%f", &ang);

				// 傳出角度給機器人
				if (number1 <= 12)
				{
					printf("Leg Axis Number = %d\n",number1);
					///gpPortLL->_write(gpDP->State,5);
					///gpPortRL->_write(gpDP->State,5);
					///Sleep(2);
					///encoder= (long int)((ang*500*4*160*50)/360.0/34.0); //!!!!!
					///gpDP->TransENC(encoder,  ID_SetENC+number1*CMD_SET);
					///gpPortLL->_write(gpDP->TxData,9); 
					///gpPortRL->_write(gpDP->TxData,9); 

				}
				else if (number1 == 13 || number1 == 14)
				{
					printf("Torso Axis Number = %d\n",number1-12);
					gpPortTorso->_write(gpDP->State,5);
					Sleep(2);
					encoder= (long int)((ang*500*4*160*75)/360.0/25.0); //!!!!!
					gpDP->TransENC(encoder,  ID_SetENC+(number1-12)*CMD_SET);
					gpPortTorso->_write(gpDP->TxData,9); 
				}
				else if (number1 >= 15 && number1 <= 20)
				{
					gpPortLA->_write(gpDP->State,5);
					printf("Left Arm Axis Number = %d\n",number1-14);
					Sleep(2);
					if (number1 == 20)
						encoder= (long int)((ang*512*4*268)/360.0); //!!!!!
					else
						encoder= (long int)((ang*512*4*150)/360.0); //!!!!!
					gpDP->TransENC(encoder,  ID_SetENC+(number1-14)*CMD_SET);
					gpPortLA->_write(gpDP->TxData,9); 
				}
				else if (number1 >= 21 && number1 <= 26)
				{
					gpPortRA->_write(gpDP->State,5);
					printf("Right Arm Axis Number = %d\n",number1-20);
					Sleep(2);
					if (number1 == 26)
						encoder= (long int)((ang*512*4*268)/360.0); //!!!!!
					else
						encoder= (long int)((ang*512*4*150)/360.0); //!!!!!
					gpDP->TransENC(encoder,  ID_SetENC+(number1-20)*CMD_SET);
					gpPortRA->_write(gpDP->TxData,9); 
				}
				else if (number1 >= 27 && number1 <= 32)
				{
					gpPortHead->_write(gpDP->State,5);
					printf("Head Axis Number = %d\n",number1-26);
					Sleep(2);
					if (number1 == 32)
						encoder= (long int)((ang*512*4*268)/360.0); //!!!!!
					else
						encoder= (long int)((ang*512*4*150)/360.0); //!!!!!
					gpDP->TransENC(encoder,  ID_SetENC+(number1-20)*CMD_SET);
					gpPortHead->_write(gpDP->TxData,9); 
				}
				else
				{
					printf("Undefined Axis Number, nothing will be sent.\n");
				}
			}

			break; 

		case SetIdle:
			//	Set Idle();  ok
 			gpDP->State[2]=SP_IDLE;
			///gpPortLL->_write(gpDP->State,5); 
			///gpPortRL->_write(gpDP->State,5); 
			gpPortLA->_write(gpDP->State,5); 
			gpPortRA->_write(gpDP->State,5);
			//gpPortHead->_write(gpDP->State,5);
			//gpPortTorso->_write(gpDP->State,5); 

			break; 

		case Reserved1:

			break;

		case Traj:
			//	Traj();	

			if (gSetPIDDone == false && gFlagArmCtrl==true)
			{
				cout << "先設定PID用以解鎖，不然無法連續控制" << endl;
			}
			else if (gFlagArmCtrl==false)
			{
				cout << "Arm/Hand Control: Closed" << endl;

				gSendContTraj=true;
				gContTrajLock = false; // 解鎖
				gSendContTrajArm=true;
				gContTrajLockArm = false; // 解鎖
				// do nothing
			}
			else
			{
				// 設定連續軌跡模式
				if (gFlagSimulation == 2)
				{
					//gpPortLA->_write(AxisOnBus,13); // 設定需要啟動的軸 1~12
					//gpPortRA->_write(AxisOnBus,13); // 設定需要啟動的軸 1~12
					////gpPortTorso->_write(AxisOnBus,13); // 設定需要啟動的軸 1~12
					/////gpPortLL->_write(AxisOnBus,13); // 設定需要啟動的軸 1~12
					/////gpPortRL->_write(AxisOnBus,13); // 設定需要啟動的軸 1~12
					////gpPortHead->_write(AxisOnBus,13);
					//Sleep(25);

					//gpPortLA->_write(AxisOnBus2,13); // 設定需要啟動的軸 13~24
					//gpPortRA->_write(AxisOnBus2,13); // 設定需要啟動的軸 13~24
					////gpPortTorso->_write(AxisOnBus2,13); // 設定需要啟動的軸 13~24
					/////gpPortLL->_write(AxisOnBus2,13); // 設定需要啟動的軸 13~24
					/////gpPortRL->_write(AxisOnBus2,13); // 設定需要啟動的軸 13~24
					////gpPortHead->_write(AxisOnBus2,13);
					//Sleep(25);

					gpDP->State[2]=ID_SetTrajPacket; //ID_SetTrajPacket
					gpPortLA->_write(gpDP->State,5);
					gpPortRA->_write(gpDP->State,5);
					//gpPortTorso->_write(gpDP->State,5);
					///gpPortLL->_write(gpDP->State,5);
					///gpPortRL->_write(gpDP->State,5);
					//gpPortHead->_write(gpDP->State,5);
					Sleep(25);

					NumOfTrajSample = gContTrajLen;

					gpDP->TxData[0]=0; // reserved
					gpDP->TxData[1]=int(dt*1000); // sapling time  = TxData[1] + TxData[2] * 256
					gpDP->TxData[2]=0;
					gpDP->TxData[3]=6; // num of axis = TxData[3] + TxData[4] * 256
					gpDP->TxData[4]=0;
					gpDP->TxData[5]=(unsigned char)NumOfTrajSample;
					gpDP->TxData[6]=(unsigned char)(NumOfTrajSample>>8); // NumOfTrajSample = TxData[5] + TxData[6] * 256
					// TxData[5] 還有 TxData[6] 先不用到，也保留，PIC32會先直接四筆四筆抓
					gpDP->TxData[7]=0; // reserved
					gpDP->TxData[8]=0; // reserved
					///gpPortLL->_write(gpDP->TxData,9); 
					///gpPortRL->_write(gpDP->TxData,9);

					gpDP->TxData[0]=0; // reserved
					gpDP->TxData[1]=int(dt*1000); // sapling time  = TxData[1] + TxData[2] * 256
					gpDP->TxData[2]=0;
					gpDP->TxData[3]=6; // num of axis = TxData[3] + TxData[4] * 256
					gpDP->TxData[4]=0;
					gpDP->TxData[5]=(unsigned char)NumOfTrajSample;
					gpDP->TxData[6]=(unsigned char)(NumOfTrajSample>>8); // NumOfTrajSample = TxData[5] + TxData[6] * 256
					// TxData[5] 還有 TxData[6] 先不用到，也保留，PIC32會先直接四筆四筆抓
					gpDP->TxData[7]=0; // reserved
					gpDP->TxData[8]=0; // reserved

					gpPortLA->_write(gpDP->TxData,9); 
					gpPortRA->_write(gpDP->TxData,9); 
					//gpPortTorso->_write(gpDP->TxData,9); 
					gpPortHead->_write(gpDP->TxData,9);

					Sleep(1);
				}

				gSendContTraj=true;
				gContTrajLock = false; // 解鎖
				gSendContTrajArm=true;
				gContTrajLockArm = false; // 解鎖

			}

			break; 


		case SetAllPID:
			//	SetAllPID();
			// 批次設定PID值
			//for (int i = 1 ; i < 13 ; i++)
			//{
			//	if (AxisOnBus[i] == 1)
			//	{

			//		if (i == 1 || i == 7) // hip yaw
			//		{
			//			kp = 360;
			//			ki = 0;
			//			kd = 0;
			//			errorsumbound = 100;
			//		}
			//		else if (i == 2 || i == 8) // hip roll
			//		{
			//			kp = 360;
			//			ki = 0;
			//			kd = 0;
			//			errorsumbound = 100;
			//		}
			//		else if (i == 3 || i == 9) // hip pitch
			//		{
			//			kp = 330;
			//			ki = 0;
			//			kd = 0;
			//			errorsumbound = 100;
			//		}
			//		else if (i == 4 || i == 10) // knee
			//		{
			//			kp = 300;
			//			ki = 0;
			//			kd = 0;
			//			errorsumbound = 100;
			//		}
			//		else if (i == 5 || i == 11) // ankle pitch
			//		{
			//			kp = 330;
			//			ki = 0;
			//			kd = 0;
			//			errorsumbound = 100;
			//		}
			//		else if (i == 6 || i == 12) // ankle roll
			//		{
			//			kp = 340;
			//			ki = 0;
			//			kd = 0;
			//			errorsumbound = 100;
			//		}


			//		gpDP->TxData[0]=ID_SetPID+CMD_SET*i;
			//		gpDP->TxData[1]=(unsigned char)kp;
			//		gpDP->TxData[2]=(unsigned char)(kp>>8);
			//		gpDP->TxData[3]=(unsigned char)ki;
			//		gpDP->TxData[4]=(unsigned char)(ki>>8);
			//		gpDP->TxData[5]=(unsigned char)kd;
			//		gpDP->TxData[6]=(unsigned char)(kd>>8);
			//		gpDP->TxData[7]=(unsigned char)errorsumbound;
			//		gpDP->TxData[8]=(unsigned char)(errorsumbound>>8);

			//		sprintf_s(gGlobalMessage,"Sending Axis %d PID\n",i);
			//		printf(gGlobalMessage);

			//		///gpPortLL->_write(gpDP->TxData,9); 
			//		///gpPortRL->_write(gpDP->TxData,9); 

			//		Sleep(25);

			//	}
			//}


			for (int i = 13 ; i < 27 ; i++)
			{

				if (i == 13) // torso
				{
					kp = 250;
					ki = 0;
					kd = 0;
					errorsumbound = 100;
				}
				else if (i == 14 ) // torso
				{
					kp = 250;
					ki = 0;
					kd = 0;
					errorsumbound = 100;
				}
				else if (i == 15 || i == 21) // hip pitch
				{
					kp = 400*PIDRatio;
					ki = 0;
					kd = 0;
					errorsumbound = 100;
				}
				else if (i == 16 || i == 22) // knee
				{
					kp = 500*PIDRatio;
					ki = 0;
					kd = 100;
					errorsumbound = 100;
				}
				else if (i == 17 || i == 23) // ankle pitch
				{
					kp = 300*PIDRatio;
					ki = 0;
					kd = 1;
					errorsumbound = 100;
				}
				else if (i == 18 || i == 24) // ankle roll
				{
					kp = 800*PIDRatio;
					ki = 0;
					kd = 250;
					errorsumbound = 100;
				}
				else if (i == 19 || i == 25) // ankle roll
				{
					kp = 300*PIDRatio;
					ki = 0;
					kd = 100;
					errorsumbound = 100;
				}
				else if (i == 20 || i == 26) // ankle roll
				{
					kp = 300*PIDRatio;
					ki = 0;
					kd = 0;
					errorsumbound = 100;
				}

				if (i >= 15 && i <= 20)
				{
					gpDP->TxData[0]=ID_SetPID+CMD_SET*(i-14);
					gpDP->TxData[1]=(unsigned char)kp;
					gpDP->TxData[2]=(unsigned char)(kp>>8);
					gpDP->TxData[3]=(unsigned char)ki;
					gpDP->TxData[4]=(unsigned char)(ki>>8);
					gpDP->TxData[5]=(unsigned char)kd;
					gpDP->TxData[6]=(unsigned char)(kd>>8);
					gpDP->TxData[7]=(unsigned char)errorsumbound;
					gpDP->TxData[8]=(unsigned char)(errorsumbound>>8);

					sprintf_s(gGlobalMessage,"Sending Axis %d PID\n",i);
					printf(gGlobalMessage);
					gpPortLA->_write(gpDP->TxData,9); 

					Sleep(25);
				}
				else if (i >= 21 && i <= 26)
				{
					gpDP->TxData[0]=ID_SetPID+CMD_SET*(i-20);
					gpDP->TxData[1]=(unsigned char)kp;
					gpDP->TxData[2]=(unsigned char)(kp>>8);
					gpDP->TxData[3]=(unsigned char)ki;
					gpDP->TxData[4]=(unsigned char)(ki>>8);
					gpDP->TxData[5]=(unsigned char)kd;
					gpDP->TxData[6]=(unsigned char)(kd>>8);
					gpDP->TxData[7]=(unsigned char)errorsumbound;
					gpDP->TxData[8]=(unsigned char)(errorsumbound>>8);

					sprintf_s(gGlobalMessage,"Sending Axis %d PID\n",i);
					printf(gGlobalMessage);
					gpPortRA->_write(gpDP->TxData,9); 

					Sleep(25);
				}


			}

			gSetPIDDone = true;


			break;


		case SetPWMLims:  // 
			//	SetPWMLims();
			// 設定PWM上下限值
			//for (int i = 1 ; i < 13 ; i++)
			//{
			//	if (AxisOnBus[i] == 1)
			//	{

			//		if (i == 4 || i == 10)
			//		{
			//			gpDP->TxData[0]=ID_SetPWMLimit+CMD_SET*i;
			//			gpDP->TxData[1]=95;
			//			gpDP->TxData[2]=LowerLim;
			//			gpDP->TxData[3]=OkErrBound%256;
			//			gpDP->TxData[4]=OkErrBound/256;
			//			gpDP->TxData[5]=0;
			//			gpDP->TxData[6]=0;
			//			gpDP->TxData[7]=0;
			//			gpDP->TxData[8]=0;
			//		}
			//		else
			//		{
			//			gpDP->TxData[0]=ID_SetPWMLimit+CMD_SET*i;
			//			gpDP->TxData[1]=UpperLim;
			//			gpDP->TxData[2]=LowerLim;
			//			gpDP->TxData[3]=OkErrBound%256;
			//			gpDP->TxData[4]=OkErrBound/256;
			//			gpDP->TxData[5]=0;
			//			gpDP->TxData[6]=0;
			//			gpDP->TxData[7]=0;
			//			gpDP->TxData[8]=0;
			//		}

			//		sprintf_s(gGlobalMessage,"Sending Axis %d PWM Limit\n",i);
			//		printf(gGlobalMessage);
			//		///gpPortLL->_write(gpDP->TxData,9);
			//		///gpPortRL->_write(gpDP->TxData,9);



			//		Sleep(25);
			//	}
			//}

			break;

		case SetCali:
			// set all mechanism calibration

		//if (gSetPIDDone == false)
		//{
		//	cout << "先設定PID用以解鎖，不然無法連續控制" << endl;
		//}
		//else
		//{
		//	// 設定Calibration值 由於機器人的安裝與加工上有誤差 所以需要這些微調值
		//	for (int CaliLoop = 1 ; CaliLoop < CaliTotalCount+1 ; CaliLoop++)
		//	{
		//		printf("CaliLoop = %d\n",CaliLoop);
		//		jointNum = 3;
		//		gpDP->State[2]=ID_SetCaliENC;
		//		///gpPortLL->_write(gpDP->State,5);
		//		///gpPortRL->_write(gpDP->State,5);
		//		//encoder= (long int)((-1.1*500*4*160*50)/360.0/34.0); //!!!!!
		//		//encoder= (long int)((-0.75*500*4*160*50)/360.0/34.0/double(CaliTotalCount)*double(CaliLoop)); //!!!!!
		//		encoder= (long int)((-1.55*500*4*160*50)/360.0/34.0/double(CaliTotalCount)*double(CaliLoop)); //!!!!!
		//		gpDP->TransENC(encoder,  ID_SetCaliENC+jointNum*CMD_SET);
		//		///gpPortLL->_write(gpDP->TxData,9);
		//		///gpPortRL->_write(gpDP->TxData,9);
		//		Sleep(1);

		//		jointNum = 4;
		//		gpDP->State[2]=ID_SetCaliENC;
		//		///gpPortLL->_write(gpDP->State,5);
		//		///gpPortRL->_write(gpDP->State,5);
		//		//encoder= (long int)((0.8*500*4*160*50)/360.0/34.0); //!!!!!
		//		encoder= (long int)((-0.5*500*4*160*50)/360.0/34.0/double(CaliTotalCount)*double(CaliLoop)); //!!!!!
		//		gpDP->TransENC(encoder,  ID_SetCaliENC+jointNum*CMD_SET);
		//		///gpPortLL->_write(gpDP->TxData,9);
		//		///gpPortRL->_write(gpDP->TxData,9);
		//		Sleep(1);

		//		jointNum = 5;
		//		gpDP->State[2]=ID_SetCaliENC;
		//		///gpPortLL->_write(gpDP->State,5);
		//		///gpPortRL->_write(gpDP->State,5);
		//		//encoder= (long int)((0.4*500*4*160*50)/360.0/34.0); //!!!!!
		//		//encoder= (long int)((-0.5*500*4*160*50)/360.0/34.0/double(CaliTotalCount)*double(CaliLoop)); //!!!!!
		//		encoder= (long int)((2.20*500*4*160*50)/360.0/34.0/double(CaliTotalCount)*double(CaliLoop)); //!!!!!
		//		gpDP->TransENC(encoder,  ID_SetCaliENC+jointNum*CMD_SET);
		//		///gpPortLL->_write(gpDP->TxData,9);
		//		///gpPortRL->_write(gpDP->TxData,9);
		//		Sleep(1);

		//		jointNum = 11;
		//		gpDP->State[2]=ID_SetCaliENC;
		//		///gpPortLL->_write(gpDP->State,5);
		//		///gpPortRL->_write(gpDP->State,5);
		//		//encoder= (long int)((0.46*500*4*160*50)/360.0/34.0); //!!!!!
		//		//encoder= (long int)((0.3*500*4*160*50)/360.0/34.0/double(CaliTotalCount)*double(CaliLoop)); //!!!!!
		//		encoder= (long int)((-2.2*500*4*160*50)/360.0/34.0/double(CaliTotalCount)*double(CaliLoop)); //!!!!!
		//		gpDP->TransENC(encoder,  ID_SetCaliENC+jointNum*CMD_SET);
		//		///gpPortLL->_write(gpDP->TxData,9);
		//		///gpPortRL->_write(gpDP->TxData,9);
		//		Sleep(1);

		//		jointNum = 8;
		//		gpDP->State[2]=ID_SetCaliENC;
		//		///gpPortLL->_write(gpDP->State,5);
		//		///gpPortRL->_write(gpDP->State,5);
		//		//encoder= (long int)((0.46*500*4*160*50)/360.0/34.0); //!!!!!
		//		//encoder= (long int)((1.5*500*4*160*50)/360.0/34.0/double(CaliTotalCount)*double(CaliLoop)); //!!!!!
		//		//encoder= (long int)((1.6*500*4*160*50)/360.0/34.0/double(CaliTotalCount)*double(CaliLoop)); //!!!!!
		//		encoder= (long int)((2.00*500*4*160*50)/360.0/34.0/double(CaliTotalCount)*double(CaliLoop)); //!!!!!
		//		gpDP->TransENC(encoder,  ID_SetCaliENC+jointNum*CMD_SET);
		//		///gpPortLL->_write(gpDP->TxData,9);
		//		///gpPortRL->_write(gpDP->TxData,9);
		//		Sleep(1);

		//		jointNum = 12;
		//		gpDP->State[2]=ID_SetCaliENC;
		//		///gpPortLL->_write(gpDP->State,5);
		//		///gpPortRL->_write(gpDP->State,5);
		//		//encoder= (long int)((0.46*500*4*160*50)/360.0/34.0); //!!!!!
		//		//encoder= (long int)((0.3*500*4*160*50)/360.0/34.0/double(CaliTotalCount)*double(CaliLoop)); //!!!!!
		//		encoder= (long int)((0.15*500*4*160*50)/360.0/34.0/double(CaliTotalCount)*double(CaliLoop)); //!!!!!
		//		gpDP->TransENC(encoder,  ID_SetCaliENC+jointNum*CMD_SET);
		//		///gpPortLL->_write(gpDP->TxData,9);
		//		///gpPortRL->_write(gpDP->TxData,9);
		//		Sleep(1);

		//		jointNum = 2;
		//		gpDP->State[2]=ID_SetCaliENC;
		//		///gpPortLL->_write(gpDP->State,5);
		//		///gpPortRL->_write(gpDP->State,5);
		//		//encoder= (long int)((-0.46*500*4*160*50)/360.0/34.0); //!!!!!
		//		//encoder= (long int)((-0.9*500*4*160*50)/360.0/34.0); //!!!!!
		//		//encoder= (long int)((-1.55*500*4*160*50)/360.0/34.0/double(CaliTotalCount)*double(CaliLoop)); //!!!!!
		//		//encoder= (long int)((-1.65*500*4*160*50)/360.0/34.0/double(CaliTotalCount)*double(CaliLoop)); //!!!!!
		//		encoder= (long int)((-2.05*500*4*160*50)/360.0/34.0/double(CaliTotalCount)*double(CaliLoop)); //!!!!!
		//		gpDP->TransENC(encoder,  ID_SetCaliENC+jointNum*CMD_SET);
		//		///gpPortLL->_write(gpDP->TxData,9);
		//		///gpPortRL->_write(gpDP->TxData,9);
		//		Sleep(1);

		//		jointNum = 9;
		//		gpDP->State[2]=ID_SetCaliENC;
		//		///gpPortLL->_write(gpDP->State,5);
		//		///gpPortRL->_write(gpDP->State,5);
		//		//encoder= (long int)((-0.46*500*4*160*50)/360.0/34.0); //!!!!!
		//		//encoder= (long int)((-0.9*500*4*160*50)/360.0/34.0); //!!!!!
		//		//encoder= (long int)((-1.55*500*4*160*50)/360.0/34.0/double(CaliTotalCount)*double(CaliLoop)); //!!!!!
		//		//encoder= (long int)((-1.65*500*4*160*50)/360.0/34.0/double(CaliTotalCount)*double(CaliLoop)); //!!!!!
		//		encoder= (long int)((-0.6*500*4*160*50)/360.0/34.0/double(CaliTotalCount)*double(CaliLoop)); //!!!!!
		//		gpDP->TransENC(encoder,  ID_SetCaliENC+jointNum*CMD_SET);
		//		///gpPortLL->_write(gpDP->TxData,9);
		//		///gpPortRL->_write(gpDP->TxData,9);
		//		Sleep(1);


		//		jointNum = 6;
		//		gpDP->State[2]=ID_SetCaliENC;
		//		///gpPortLL->_write(gpDP->State,5);
		//		///gpPortRL->_write(gpDP->State,5);
		//		//encoder= (long int)((-0.46*500*4*160*50)/360.0/34.0); //!!!!!
		//		//encoder= (long int)((-0.3*500*4*160*50)/360.0/34.0/double(CaliTotalCount)*double(CaliLoop)); //!!!!!
		//		encoder= (long int)((-0.50*500*4*160*50)/360.0/34.0/double(CaliTotalCount)*double(CaliLoop)); //!!!!!
		//		gpDP->TransENC(encoder,  ID_SetCaliENC+jointNum*CMD_SET);
		//		///gpPortLL->_write(gpDP->TxData,9);
		//		///gpPortRL->_write(gpDP->TxData,9);
		//		Sleep(1);

		//		jointNum = 10;
		//		gpDP->State[2]=ID_SetCaliENC;
		//		///gpPortLL->_write(gpDP->State,5);
		//		///gpPortRL->_write(gpDP->State,5);
		//		//encoder= (long int)((-0.46*500*4*160*50)/360.0/34.0); //!!!!!
		//		//encoder= (long int)((-0.3*500*4*160*50)/360.0/34.0/double(CaliTotalCount)*double(CaliLoop)); //!!!!!
		//		encoder= (long int)((-1.30*500*4*160*50)/360.0/34.0/double(CaliTotalCount)*double(CaliLoop)); //!!!!!
		//		gpDP->TransENC(encoder,  ID_SetCaliENC+jointNum*CMD_SET);
		//		///gpPortLL->_write(gpDP->TxData,9);
		//		///gpPortRL->_write(gpDP->TxData,9);
		//		Sleep(1);



		//	}
		//}

			break;


		case SetEncReadFlag:

			//// Set encdor reading status 感應器讀取狀態 (設定是否讀取)

			//gpDP->State[2]=ID_ReadEncoder;
			/////gpPortLL->_write(gpDP->State,5);
			/////gpPortRL->_write(gpDP->State,5);

			////sprintf_s(gGlobalMessage,"Encoders will be read\n");
			////printf(gGlobalMessage);

			//Sleep(25);

			break;

		default: 
			cout << "Without Choosing Sending Mode" << endl; 
		}

		if(mode!=Traj)
		cout<<"System Stand by:"<<endl;
	}
	else
	{
			cout<<"please push initialize"<<endl;
	}
}

// ======================================================================
// log file
// ======================================================================
void gLoadfile()
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 讀取所有需要從.txt .dat 讀取的資料
	******************************************************************/
	int tempData = 0;

	gfLALen.open("DataLen_LA.txt",ios::in);
	gfLALen >> gLAMotionLen;
	gfLALen.close();
	cout << "Continuous Trajectory Length of Left Arm= " << gLAMotionLen << endl;

	gfLATraj.open("DataTraj_LA.txt",ios::in);

	for (int i = 0 ; i < gLAMotionLen ; i++)
	{

		for (int j = 0 ; j < 30 ; j++)
		{
			gfLATraj >> tempData;
			gLAMotion[j][i] = tempData;
		}
	}

	gfLATraj.close();

	cout << "Left Arm Continuous Trajectory Loading Completed " << endl;

	gfRALen.open("DataLen_RA.txt",ios::in);
	gfRALen >> gRAMotionLen;
	gfRALen.close();
	cout << "Continuous Trajectory Length of Right Arm= " << gRAMotionLen << endl;

	gfRATraj.open("DataTraj_RA.txt",ios::in);

	for (int i = 0 ; i < gRAMotionLen ; i++)
	{
		for (int j = 0 ; j < 30 ; j++)
		{
			gfRATraj >> tempData;
			gRAMotion[j][i] = tempData;
		}
	}

	gfRATraj.close();

	cout << "Right Arm Continuous Trajectory Loading Completed " << endl;

	gfTorsoLen.open("DataLen_Torso.txt",ios::in);
	gfTorsoLen >> gTorsoMotionLen;
	gfTorsoLen.close();
	cout << "Continuous Trajectory Length of Torso= " << gTorsoMotionLen << endl;

	gfTorsoTraj.open("DataTraj_Torso.txt",ios::in);

	for (int i = 0 ; i < gTorsoMotionLen ; i++)
	{

		for (int j = 0 ; j < 30 ; j++)
		{
			gfTorsoTraj >> tempData;
			gTorsoMotion[j][i] = tempData;
		}
	}

	gfTorsoTraj.close();

	cout << "Left Arm Continuous Trajectory Loading Completed " << endl;


}


void gLoadfileHand() // 至峻 20130410
{
		// ================================== 以上不用宣告，手掌程式中已經宣告過 =================================================================
	int tempData = 0;
	int ContTrajRow = -1;
	int ContTrajCol = 0;
	int ContTrajRow1 = -1;
	int ContTrajCol1 = 0;
	
	char fileR[] = "ALLR_H.txt";
	char fileL[] = "ALLL_H.txt";

	//f load datalength

	bool getcolumn = 1;

	string line;
	string word;

	//===================================== Record the number of Row, Column for Right hand =================================================
	gfLen.open(fileR,ios::in);

	if (gfLen.is_open())
	{
		while (!gfLen.eof())
		{
			getline(gfLen,line);
			ContTrajRow++;

			if (getcolumn)
			{
				istringstream ss(line);

				while (ss>>word)
				{
					ContTrajCol++;
				}
				getcolumn=0;
			}
		}
	}
	gfLen.close();

	// ==================================== Record the number of Row, Column for Left hand========================================================

	getcolumn = 1;
	gfLen.open(fileL,ios::in);

	if (gfLen.is_open())
	{
		while (!gfLen.eof())
		{
			getline(gfLen,line);
			ContTrajRow1++;

			if (getcolumn)
			{
				istringstream ss(line);

				while (ss>>word)
				{
					ContTrajCol1++;
				}
				getcolumn=0;
			}
		}
	}
	gfLen.close();

	ContTrajCol = ContTrajCol + 6;								// +6 為補ID的個數
	ContTrajCol1 = ContTrajCol1 + 6;							// +6 為補ID的個數

	gfContTrajLenRH = ContTrajRow;								//	資料筆數
	gfContTrajLenLH = ContTrajRow1;


	gfContTrajDataRH = new unsigned char [ContTrajRow*ContTrajCol];
	gfContTrajDataLH = new unsigned char [ContTrajRow1*ContTrajCol1];

	int count_L1 = 0;
	int ID_L = 7;						//補ID的變數，初始值設定為需要補上的ID (L與左右手掌無關)

	gfRH.open(fileR,ios::in);//"handTrajactory_1_5_pic.txt"
	for (int i = 0 ; i < ContTrajRow*ContTrajCol ; i++)
	{
		if ( count_L1==0 )
			gfContTrajDataRH[i] = ID_L;
		else
		{
			gfRH >> tempData;
			gfContTrajDataRH[i] = tempData;
		}
		count_L1++;

		//===========Defined by user (ID是由7~12，若以後需要改變手掌馬達ID，此判斷式需要改變(for Right Hand))============
		if (count_L1==5)
		{
			count_L1 = 0;
			ID_L++;
		}
		if (ID_L==13)		
			ID_L = 7;
		//===============================================================================================================
	}

	gfRH.close();

	count_L1 = 0;
	ID_L = 7;				//需要隨著ID初始值而跟著改變

	//f4 load left hand trajectory

	gfLH.open(fileL,ios::in);//"handTrajactory_1_5_pic.txt"
	for (int i = 0 ; i < ContTrajRow1*ContTrajCol1; i++)
	{
		if (count_L1 == 0)
			gfContTrajDataLH[i] = ID_L;
		else
		{
			gfLH >> tempData;
			gfContTrajDataLH[i] = tempData;
		}
		count_L1++;

		//===========Defined by user (ID是由7~12，若以後需要改變手掌馬達ID，此判斷式需要改變(for Left Hand))=============
		if (count_L1==5)
		{
			count_L1 = 0;
			ID_L++;
		}
		if (ID_L==13)
			ID_L = 7;
		//===============================================================================================================
	}

	gfLH.close();

	//ContTrajLoaded = 1;//沒用到

	cout<<gfContTrajLenRH<<endl;
	cout<<gfContTrajLenLH<<endl;


	cout << "Continuous Trajectory Loading Completed " << endl;


	//int tempData = 0;
	//gfLen.open("DataLen.txt",ios::in);
	//gfLen >> gfContTrajLenRH;
	//gfLen.close();

	//gfLen.open("DataLen1.txt",ios::in);
	//gfLen >> gfContTrajLenLH;
	//gfLen.close();

	//gfContTrajDataRH = new unsigned char [gfContTrajLenRH*5*12];
	//gfContTrajDataLH = new unsigned char [gfContTrajLenLH*5*12];
	////f2 load right hand trajectory
	//int count_L1 = 0;
	//int ID_L = 7;						//補ID的變數，初始值設定為需要補上的ID (L與左右手掌無關)

	//gfRH.open("RightHand_A.txt",ios::in);//"handTrajactory_1_5_pic.txt"
	//for (int i = 0 ; i < gfContTrajLenRH*5*12 ; i++)
	//{
	//	if ( count_L1==0 )
	//		gfContTrajDataRH[i] = ID_L;
	//	else
	//	{
	//		gfRH >> tempData;
	//		gfContTrajDataRH[i] = tempData;
	//	}
	//	count_L1++;

	//	//===========Defined by user (ID是由7~12，若以後需要改變手掌馬達ID，此判斷式需要改變(for Right Hand))============
	//	if (count_L1==5)
	//	{
	//		count_L1 = 0;
	//		ID_L++;
	//	}
	//	if (ID_L==13)		
	//		ID_L = 7;
	//	//===============================================================================================================
	//}

	//gfRH.close();

	//count_L1 = 0;
	//ID_L = 7;				//需要隨著ID初始值而跟著改變

	////f4 load left hand trajectory

	//gfLH.open("LeftHand_A.txt",ios::in);//"handTrajactory_1_5_pic.txt"
	//for (int i = 0 ; i < gfContTrajLenLH*5*12 ; i++)
	//{
	//	if (count_L1 == 0)
	//		gfContTrajDataLH[i] = ID_L;
	//	else
	//	{
	//		gfLH >> tempData;
	//		gfContTrajDataLH[i] = tempData;
	//	}
	//	count_L1++;
	//	
	//	//===========Defined by user (ID是由7~12，若以後需要改變手掌馬達ID，此判斷式需要改變(for Left Hand))=============
	//	if (count_L1==5)
	//	{
	//		count_L1 = 0;
	//		ID_L++;
	//	}
	//	if (ID_L==13)
	//		ID_L = 7;
	//	//===============================================================================================================
	//}

	//gfLH.close();

	//cout << "Continuous Trajectory Hand Loading Completed " << endl;



}


void gRenderSceneThread(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 繪製機器人棒棒人模型 可以設定FPS 
	// 最主要的繪圖function
	******************************************************************/


	LARGE_INTEGER StartTime_Render;
	LARGE_INTEGER CurrentTime_Render;
	LARGE_INTEGER FreqSys;
	double SysTime_Render = 0;

	double Freq_;

	QueryPerformanceFrequency(&FreqSys);
	Freq_ = FreqSys.QuadPart;

	// 設定 framerate
	float FPS = 30.0;
	if (gFlagSimulation == RealExp)
	{
		FPS = 10;
	}

	double TimeDiff;
	double Now_Time;

	char time[60];
	char *c;

	float w_scale = 640.0f;

	//_______________________________________Added by Slongz_______________________________________
	#if CheckingMode 
	double GLCOG[20000][3];
	double GLZMP[20000][3];
	int tempindex=1;
	#endif
	//_______________________________________Added by Slongz_______________________________________
	while(1)
	{
		if (gRenderLife)
		{
			if (gRenderPMSWorking	== false)
			{
				gRenderKineWorking = true;
				QueryPerformanceCounter(&StartTime_Render);
				glutSetWindow(gGLID);
				
				glEnable(GL_COLOR_MATERIAL );
				//glDisable(GL_COLOR_MATERIAL );
				glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

				glLoadIdentity();
				glViewport(0,0,640,640);
				glMatrixMode(GL_PROJECTION);						
				glLoadIdentity();
				glOrtho(-w_scale,w_scale,-w_scale,w_scale,-w_scale,w_scale);

				if (gStartTimeAcquired)
				{
					Now_Time = (StartTime_Render.QuadPart-gStartTime.QuadPart)/double(FreqSys.QuadPart);
				}
				else
				{
					Now_Time = 0.0;
				}

				sprintf_s(time,"Time = %6.3f s ", Now_Time);

				glPushMatrix();
					glTranslatef(0.0,0.0,0);		
					glColor3f(1.0f,1.0f,1.0f);

					glRasterPos2f(-w_scale,w_scale-40);
					for ( c = time ; *c!= '\0';c++)
						glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24 ,*c);

				glPopMatrix();

				glPushMatrix();
					glTranslatef(0.0,0.0,0);		
					glColor3f(1.0f,1.0f,1.0f);

					glRasterPos2f(-w_scale,w_scale-90);
					for ( c = gGlobalMessage ; *c!= '\0';c++)
						glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18 ,*c);

				glPopMatrix();
	//_______________________________________Added by Slongz_______________________________________
				glPushMatrix();
					glTranslatef(0.0,0.0,0);		
					glColor3f(1,1,1);

					glRasterPos2f(-w_scale,w_scale-140);
					for ( c = gBipedInfo1 ; *c!= '\0';c++)
						glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18 ,*c);

				glPopMatrix();

				glPushMatrix();
					glTranslatef(0.0,0.0,0);		
					glColor3f(1,1,1);

					glRasterPos2f(-w_scale,w_scale-190);
					for ( c = gBipedInfo2 ; *c!= '\0';c++)
						glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18 ,*c);

				glPopMatrix();
	//_______________________________________Added by Slongz_______________________________________



				// 設定 Viewport 
				// glLoadIdentity();
				// glViewport(0, 0, 640, 640);

				// 重新設定 Projection matrix
				glMatrixMode(GL_PROJECTION);
				glLoadIdentity();

				// 作 Perspective 投射
				gluPerspective(75.0f, 1.0, gViewRangeNear, gViewRangeFar);
				// 重新設定 View matrix


				glMatrixMode(GL_MODELVIEW);
				glLoadIdentity();

				gluLookAt(gEyeCenter[0],		gEyeCenter[1],		gEyeCenter[2],
						  gViewCenter[0],	gViewCenter[1],		gViewCenter[2],
						  gCamUp[0],			gCamUp[1],			gCamUp[2]);

				glColor3f(1.0f,1.0f,1.0f);
				//glutWireTeapot(2.0f);
				glEnable(GL_COLOR_MATERIAL );
				glBegin(GL_LINES);

					glColor3f(0.0f,1.0f,0.0f);
					glVertex3f(0.0f,0.0f,0.0f);
					glVertex3f(50.0f,0.0f,0.0f);

					glColor3f(0.0f,0.0f,1.0f);
					glVertex3f(0.0f,0.0f,0.0f);
					glVertex3f(0.0f,50.0f,0.0f);

					glColor3f(1.0f,0.0f,0.0f);
					glVertex3f(0.0f,0.0f,0.0f);
					glVertex3f(0.0f,0.0f,50.0f);

				glEnd();
				////____________________________________Dora Laser
				////eye->transfer();
				#if LaserCatch
					glPushMatrix();
					glTranslatef(gKineAll.CrdAll->data[1],gKineAll.CrdAll->data[2],gKineAll.CrdAll->data[0]);
					for(int i=0;i<681;i++)
						{
						glPushMatrix();
						glTranslatef((float)laserbufY[i]*-1,(float)20,(float)laserbufX[i]);
						glColor3f(1.0f,0.0f,0.0f);
						glutSolidSphere(10.0f,20,20);				
						glPopMatrix();
						}
					glPopMatrix();
				#endif
				////____________________________________Dora Laser

				if(gFlagGLMode==0)//mode change//dora
				{
					
				// 畫出棒棒人
				// 繪圖變數會自動在算完FK以後自動更新完成 這樣就不會搶 thread
				gGLDrawLinePoint(gpRobotDrawingBuffer,gDrawingSeqSize/3);
				gGLDrawLinePoint(gpRobotDrawingBufferArm,gDrawingSeqSizeArm/3);


				//// 畫出 COG
				//glPushMatrix();

				//glTranslatef((float)gKineAll.COG[1],(float)gKineAll.COG[2],(float)gKineAll.COG[0]);
				//glColor3f(0.0f,1.0f,0.0f);
				//glutSolidSphere(30.0f,20,20);
				#if CheckingMode 
				// 畫一大顆COG
				glPushMatrix();
				glTranslatef((float)gKineAll.COG[1],(float)gKineAll.COG[2],(float)gKineAll.COG[0]);
				glColor3f(0.0f,1.0f,0.0f);
				glutSolidSphere(30.0f,20,20);
				glPopMatrix();
				#endif
				
				//glPopMatrix();
				
				#if CheckingMode //(畫COG軌跡，ZMP軌跡，Grid)
				  if(gFlagReadForceSensor==0)
				  {
						// 畫出 COG
						GLCOG[tempindex][0]=gKineAll.COG[1];
						GLCOG[tempindex][1]=gKineAll.COG[2];
						GLCOG[tempindex][2]=gKineAll.COG[0];
						tempindex++;

						for(int i=0;i<tempindex;i++)
						{
						glPushMatrix();
						glTranslatef((float)GLCOG[i][0],(float)GLCOG[i][1],(float)GLCOG[i][2]);
						glColor3f(0.0f,1.0f,0.0f);
						glutSolidSphere(5.0f,20,20);				
						glPopMatrix();
						}
						// 畫出 ZMP 
						if(gIthIK>0)
						{
							GLZMP[tempindex][0]=gLQs.XState[gIthIK].data[2];
							GLZMP[tempindex][1]=gInpZMPHeight[gIthIK];
							GLZMP[tempindex][2]=gLQs.YState[gIthIK].data[2];
						}
						else
						{
							GLZMP[tempindex][0]=gLQs.XState[0].data[2];
							GLZMP[tempindex][1]=gInpZMPHeight[0];
							GLZMP[tempindex][2]=gLQs.YState[0].data[2];
						}
						for(int i=0;i<tempindex;i++)
						{
						glPushMatrix();
						glTranslatef((float)GLZMP[i][0],(float)GLZMP[i][1],(float)GLZMP[i][2]);
						glColor3f(0.0f,0.7f,0.9f);
						glutSolidSphere(5.0f,20,20);				
						glPopMatrix();
						}
						// 畫出 Grid
						const float sizeL = 0.f;
						const float grid = 50.f;
						glPushMatrix();
						//glPointSize(sizeL);
						glColor3f(0.6f,0.6f,0.6f);
						glLineWidth (0.01);
						glBegin(GL_LINES);
						for(float z=-2000; z<2000; z+=grid)
						for(float i=-2000; i<2000; i+=grid)
						{
							glVertex3f(-2000.f,0, i);
							glVertex3f(2000.f,0, i);
 
							glVertex3f(i, 0,-2000.f);
							glVertex3f(i,0, 2000.f);
						}
						glEnd();
						glPopMatrix();
				  }
				#endif
				}
				else//dora
				{
					glDisable(GL_COLOR_MATERIAL );//dora
					gGLModel.gGLWholeBody();//class//dora
					glEnable(GL_COLOR_MATERIAL );//dora
				}


				//// 身體質量塊連線
				//gpRobotDrawingBuffer[0] = gKineAll.DHOrigin[1];
				//gpRobotDrawingBuffer[1] = gKineAll.DHOrigin[2];
				//gpRobotDrawingBuffer[2] = gKineAll.DHOrigin[0];
				//gpRobotDrawingBuffer[3] = gKineAll.pv_stack[28];
				//gpRobotDrawingBuffer[4] = gKineAll.pv_stack[29];
				//gpRobotDrawingBuffer[5] = gKineAll.pv_stack[27];
				//gGLDrawLinePoint(gpRobotDrawingBuffer,2);

				////// 身體質量塊表示
				//for  (int i=0;i<16;i++)
				//{
				//glPushMatrix();

				////int axis = 7;
				//glTranslatef((float)gKineAll.pv_stack[3*i+1],(float)gKineAll.pv_stack[3*i+2],(float)gKineAll.pv_stack[3*i+0]);
				//glColor3f(0.0f,1.0f,0.0f);
				//glutSolidSphere(20.0f,20,20);
				//
				//glPopMatrix();
				//}
				// 繪製ZMP位置


				if (gFlagZMPPlot)
				{ 
					if (gIthIK > 0)
					{
						glPushMatrix();
							glColor3f(1.0,0.0,0.0);
							//glTranslatef((float)(gLQs.XState[gIthIK].data[2]),(float)(gLQs.YState[gIthIK].data[2]),20);
							//glTranslatef((float)(gLQs.XState[gIthIK].data[2]),(float)gInpZMPHeight[gIthIK],(float)(gLQs.YState[gIthIK].data[2]));
							if(gFlagReadForceSensor)
							glTranslatef(  (float)(gKineAll.FS_ZMP[1]),   (float)gInpZMPHeight[gIthIK],   (float)(gKineAll.FS_ZMP[0])   );
							else
							glTranslatef((float)(gLQs.XState[gIthIK].data[2]),(float)gInpZMPHeight[gIthIK],(float)(gLQs.YState[gIthIK].data[2]));
							glutSolidSphere(15.0f,15.0f,15.0f);
						glPopMatrix();
					}
					else
					{
						glPushMatrix();
							glColor3f(1.0,0.0,0.0);
							//glTranslatef((float)(gLQs.XState[gIthIK].data[2]),(float)(gLQs.YState[gIthIK].data[2]),20);
							glTranslatef((float)(gLQs.XState[0].data[2]),(float)gInpZMPHeight[0],(float)(gLQs.YState[0].data[2]));
							glutSolidSphere(15.0f,15.0f,15.0f);
						glPopMatrix();
					}
				}

				// 地板繪圖
				double CubeSize = 2000;
				
				//#if OfflineTraj
				//	glPushMatrix();
				//		glTranslatef(0,0,-230+114);
				//		glColor3f(0.0,0.3,0.0);
				//		glPushMatrix();
				//			glTranslatef(0,-CubeSize/2.0,0);
				//			glutSolidCube(CubeSize);
				//		glPopMatrix();
				//	glPopMatrix();	
				//#else
					// Normal
					glPushMatrix();
						glTranslatef(0,-6,-230+114);
						glColor3f(0.0,0.3,0.0);
							glPushMatrix();
								glTranslatef(0,-CubeSize/2.0,0);
								glutSolidCube(CubeSize);
							glPopMatrix();
					glPopMatrix();
					// Normal

					//// 上下樓梯用
					//CubeSize = 460;
					//glPushMatrix();
					//	glTranslatef(0,-6,114+20); //  在此調整方塊位置 z方向為一開始auto完出螢幕位置 y為正向上 請注意方塊原點為雙腳中間 z=114(腳底前半部)+20(樓梯空隙)
					//	//glTranslatef(0,-6,-230+114);
					//	for (int i = 1; i < 8 ; i++)
					//	{				
					//		glColor3f(1.0-i*0.1,0.1+i*0.1,0.1);
					//		glTranslatef(0,gGroundHeight[i]-gGroundHeight[i-1],gFstpY[i]-gFstpY[i-1]);

					//		if (i == 5)
					//		{
					//			glTranslatef(0,0,CubeSize/2);
					//		}

					//		glPushMatrix();
					//			glTranslatef(0,-CubeSize/2.0,0);
					//			glutSolidCube(CubeSize);
					//		glPopMatrix();
					//	}
					//glPopMatrix();
					//// 上下樓梯用
				
					//// 轉彎用場景
					//double CubeSize = 4000;

					//glColorMaterial(GL_FRONT_AND_BACK, GL_EMISSION );
					//glEnable(GL_COLOR_MATERIAL );

					//glPushMatrix();
					//		
					//glTranslatef(0.0,-CubeSize/2.0-10.0,0.0);

					//glColor3f(0.0,0.3,0.0);
					//glutSolidCube(CubeSize);

					//glPopMatrix();
					//// 轉彎用場景
				//#endif

				glutSwapBuffers();
		
				QueryPerformanceCounter(&CurrentTime_Render);
				SysTime_Render = (CurrentTime_Render.QuadPart - StartTime_Render.QuadPart)/Freq_;

				TimeDiff = (1.0/FPS - SysTime_Render)*1000.0;

				//printf("Render time cost = %f \n",SysTime_Render);

				gRenderKineWorking = false;

				if (TimeDiff < 2.0) // waiting for expected FPS
				{
					// do nothing
					// speed lower than expected
					//printf("time cost = %f \n",SysTime_Render);
				}
				else
				{
					Sleep(DWORD(TimeDiff));
				}
			}
		}
		else
		{
			break;
		}
	}
}




void gRenderSceneEmpty(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 空的function, 讓GL不要在被更動的時候自動繪圖\
	// 這樣比較好自己控制畫圖
	******************************************************************/
}



void gMouseFunc(int button, int state, int x, int y)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 偵測滑鼠是否在GL視窗中被壓下
	******************************************************************/
	if (button== GLUT_LEFT_BUTTON) {
		gLeftMouse = !gLeftMouse;
	}
	if (button == GLUT_RIGHT_BUTTON) {
		gRightMouse = !gRightMouse;
	}

}

void gMousePressedMove(int x, int y)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 偵測滑鼠是否在GL視窗中被壓下後移動
	// 有的話，就會進來這個function
	******************************************************************/
	int dx = x-gLastMx;
	int dy = y-gLastMy;
	double tempA[3];
	double VLineCam[3];

	if (gLeftMouse && !gRightMouse) // space-fixed rotation
	{

		// y rotation
		gRotYMat(gRotSensivity*dx,&gTempM);
		MatMulAB(gRotMat.data,3,3,gTempM.data,3,3,gTempMRes.data);
		for (int i = 0 ; i < 9 ; i ++)
			gRotMat.data[i] = gTempMRes.data[i];

		// x rotation
		gRotXMat(gRotSensivity*dy,&gTempM);
		MatMulAB(gRotMat.data,3,3,gTempM.data,3,3,gTempMRes.data);
		for (int i = 0 ; i < 9 ; i ++)
			gRotMat.data[i] = gTempMRes.data[i];

		VLineCam[0] = gEyeCenterInit[0]-gViewCenterInit[0]; 	
		VLineCam[1] = gEyeCenterInit[1]-gViewCenterInit[1]; 	
		VLineCam[2] = gEyeCenterInit[2]-gViewCenterInit[2]; 
		MatMulAB(gRotMat.data,3,3,VLineCam,3,1,tempA);
		gEyeCenter[0] = tempA[0]+gViewCenterInit[0];
		gEyeCenter[1] = tempA[1]+gViewCenterInit[1];
		gEyeCenter[2] = tempA[2]+gViewCenterInit[2];

		MatMulAB(gRotMat.data,3,3,gCamUpInit,3,1,tempA);
		gCamUp[0] = tempA[0];
		gCamUp[1] = tempA[1];
		gCamUp[2] = tempA[2];


	}
	else if (!gLeftMouse && gRightMouse)
	{

		float px;
		float py;
		float pz;

		if (dx > 0)
		{
			px = gMoveSensivity*gRotMat.data[0];
			py = gMoveSensivity*gRotMat.data[3];
			pz = gMoveSensivity*gRotMat.data[6];

			gViewCenterInit[0] -= px;
			gViewCenterInit[1] -= py;
			gViewCenterInit[2] -= pz;
			gViewCenter[0] = gViewCenterInit[0];
			gViewCenter[1] = gViewCenterInit[1];
			gViewCenter[2] = gViewCenterInit[2];

			gEyeCenterInit[0] -= px;
			gEyeCenterInit[1] -= py;
			gEyeCenterInit[2] -= pz;

			gEyeCenter[0] -= px;
			gEyeCenter[1] -= py;
			gEyeCenter[2] -= pz;


		}
		else if (dx <0)
		{

			px = gMoveSensivity*gRotMat.data[0];
			py = gMoveSensivity*gRotMat.data[3];
			pz = gMoveSensivity*gRotMat.data[6];

			gViewCenterInit[0] += px;
			gViewCenterInit[1] += py;
			gViewCenterInit[2] += pz;
			gViewCenter[0] = gViewCenterInit[0];
			gViewCenter[1] = gViewCenterInit[1];
			gViewCenter[2] = gViewCenterInit[2];

			gEyeCenterInit[0] += px;
			gEyeCenterInit[1] += py;
			gEyeCenterInit[2] += pz;

			gEyeCenter[0] += px;
			gEyeCenter[1] += py;
			gEyeCenter[2] += pz;

		}
		if (dy > 0)
		{

			px = gMoveSensivity*gRotMat.data[1];
			py = gMoveSensivity*gRotMat.data[4];
			pz = gMoveSensivity*gRotMat.data[7];

			gViewCenterInit[0] += px;
			gViewCenterInit[1] += py;
			gViewCenterInit[2] += pz;
			gViewCenter[0] = gViewCenterInit[0];
			gViewCenter[1] = gViewCenterInit[1];
			gViewCenter[2] = gViewCenterInit[2];

			gEyeCenterInit[0] += px;
			gEyeCenterInit[1] += py;
			gEyeCenterInit[2] += pz;

			gEyeCenter[0] += px;
			gEyeCenter[1] += py;
			gEyeCenter[2] += pz;

		}
		else if (dy <0)
		{
			px = gMoveSensivity*gRotMat.data[1];
			py = gMoveSensivity*gRotMat.data[4];
			pz = gMoveSensivity*gRotMat.data[7];

			gViewCenterInit[0] -= px;
			gViewCenterInit[1] -= py;
			gViewCenterInit[2] -= pz;
			gViewCenter[0] = gViewCenterInit[0];
			gViewCenter[1] = gViewCenterInit[1];
			gViewCenter[2] = gViewCenterInit[2];

			gEyeCenterInit[0] -= px;
			gEyeCenterInit[1] -= py;
			gEyeCenterInit[2] -= pz;

			gEyeCenter[0] -= px;
			gEyeCenter[1] -= py;
			gEyeCenter[2] -= pz;
		}

	}
	else if (gLeftMouse && gRightMouse)
	{

		float Vector_2P[3];
		double VLineCam[3];
		double NormVal = gNorm2Pd(gEyeCenterInit,gViewCenterInit);
		double Temp;
		bool ExeScale = false;

		Vector_2P[0] = gEyeCenterInit[0] - gViewCenterInit[0];
		Vector_2P[1] = gEyeCenterInit[1] - gViewCenterInit[1];
		Vector_2P[2] = gEyeCenterInit[2] - gViewCenterInit[2];

		if (dy > 0)
		{
			if (NormVal > 0.2)
			{
				Temp = NormVal/gScaleSensivity;
				ExeScale = true;
			}		
		}
		else if (dy < 0)
		{
			Temp = NormVal*gScaleSensivity;
			ExeScale = true;
		}

		if (ExeScale == true)
		{

			Vector_2P[0] = Vector_2P[0]*Temp/NormVal;
			Vector_2P[1] = Vector_2P[1]*Temp/NormVal;
			Vector_2P[2] = Vector_2P[2]*Temp/NormVal;	

			gEyeCenterInit[0] = gViewCenterInit[0] + Vector_2P[0];
			gEyeCenterInit[1] = gViewCenterInit[1] + Vector_2P[1];
			gEyeCenterInit[2] = gViewCenterInit[2] + Vector_2P[2];

			VLineCam[0] = gEyeCenterInit[0]-gViewCenterInit[0]; 	
			VLineCam[1] = gEyeCenterInit[1]-gViewCenterInit[1]; 	
			VLineCam[2] = gEyeCenterInit[2]-gViewCenterInit[2]; 
			MatMulAB(gRotMat.data,3,3,VLineCam,3,1,tempA);
			gEyeCenter[0] = tempA[0]+gViewCenterInit[0];
			gEyeCenter[1] = tempA[1]+gViewCenterInit[1];
			gEyeCenter[2] = tempA[2]+gViewCenterInit[2];
		}

	}

	gLastMx = x;
	gLastMy = y;

}
void gMouseNotPressedMove(int x, int y)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 偵測滑鼠是否在GL視窗中沒有被壓下時移動
	// 若是的話，就會進來這個function
	******************************************************************/
	gLastMx = x;
	gLastMy = y;

	gLeftMouse = false;
	gRightMouse = false;

}



void gOnSizeMyGL(int w, int h)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 偵測GL視窗大小是否改變
	// 若是的話，就會進來這個function
	******************************************************************/

	GLfloat fAspect;

    // Prevent a divide by zero
    if(h == 0)
        h = 1;

    // 設定 Viewport 
    glViewport(0, 0, w, h);

	fAspect = (GLfloat)w /(GLfloat)h;
	// 顯像的步驟
    // 重新設定 Projection matrix
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

	// 作 Perspective 投射
	//gluPerspective(proj_ang, fAspect, 1.0, 425.0);
	gluPerspective(45.0f, 1.0, gViewRangeNear, gViewRangeFar);
	// 重新設定 View matrix
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

}

float gNorm2Pf(float* p1, float* p2)
{
	/******************************************************************
	input: p1 p2 都是三維座標點
	output: 距離

	Note:
	// 取出這兩的輸入點的距離
	******************************************************************/
	return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1])+(p1[2]-p2[2])*(p1[2]-p2[2]));
}

double gNorm2Pd(double* p1, double* p2)
{
	/******************************************************************
	input: p1 p2 都是三維座標點
	output: 距離

	Note:
	// 取出這兩的輸入點的距離
	******************************************************************/
	return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1])+(p1[2]-p2[2])*(p1[2]-p2[2]));
}

float gNorm1Pf(float* p1)
{
	/******************************************************************
	input: p1 是三維向量
	output: 長度

	Note:
	// 取出這向量的長度
	******************************************************************/
	return sqrt(p1[0]*p1[0]+p1[1]*p1[1]+p1[2]*p1[2]);
}

float gNorm1Pd(double* p1)
{
	/******************************************************************
	input: p1 是三維向量
	output: 長度

	Note:
	// 取出這向量的長度
	******************************************************************/
	return sqrt(p1[0]*p1[0]+p1[1]*p1[1]+p1[2]*p1[2]);
}


void gCross2Vf(float* v1, float* v2, float* v3)
{
	/******************************************************************
	input: p1 p2 p3都是三維向量
	output: void

	Note:
	// 向量外積 v3 = v2 x v1
	******************************************************************/
	v3[0] = v1[1]*v2[2]-v1[2]*v2[1];
	v3[1] = v1[2]*v2[0]-v1[0]*v2[2];
	v3[2] = v1[0]*v2[1]-v1[1]*v2[0];
}

void gCross2Vd(double* v1, double* v2, double* v3)
{
	/******************************************************************
	input: p1 p2 p3都是三維向量
	output: void

	Note:
	// 向量外積 v3 = v2 x v1
	******************************************************************/
	v3[0] = v1[1]*v2[2]-v1[2]*v2[1];
	v3[1] = v1[2]*v2[0]-v1[0]*v2[2];
	v3[2] = v1[0]*v2[1]-v1[1]*v2[0];
}



void gRotXMat(float th, YMatLite* Rxn)
{
	/******************************************************************
	input: th 旋轉角度, Rxn 相對應的旋轉矩陣
	output: void

	Note:
	// 計算對於X軸旋轉th度的旋轉矩陣
	******************************************************************/
	Rxn->data[0] = 1;
	Rxn->data[1] = 0;
	Rxn->data[2] = 0;
	Rxn->data[3] = 0;
	Rxn->data[4] = cos(th);
	Rxn->data[5] = sin(th);
	Rxn->data[6] = 0;
	Rxn->data[7] = -sin(th);
	Rxn->data[8] = cos(th);
}


void gRotYMat(float th, YMatLite* Ryn)
{
	/******************************************************************
	input: th 旋轉角度, Ryn 相對應的旋轉矩陣
	output: void

	Note:
	// 計算對於Y軸旋轉th度的旋轉矩陣
	******************************************************************/
	Ryn->data[0] = cos(th);
	Ryn->data[1] = 0;
	Ryn->data[2] = -sin(th);
	Ryn->data[3] = 0;
	Ryn->data[4] = 1;
	Ryn->data[5] = 0;
	Ryn->data[6] = sin(th);
	Ryn->data[7] = 0;
	Ryn->data[8] = cos(th);
}

void gGLDrawLinePoint(float* CrdDraw,int Len)
{
	/******************************************************************
	input: CrdDraw 要被畫出來的點線資料, Len 總點數
	output: void

	Note:
	// 依照輸入點線資料，自動畫出預設的點線樣式
	******************************************************************/

	// CrdDraw is a Nx3 matrix, each row is a point vector in world

	int indexI = 3;
	int indexP = 0;

	//glColor4f(1.0f,0.0f,0.0f,1.0f);
	glColor3f(1.0f,0.0f,0.0f);
	glPushMatrix();

		glTranslatef(CrdDraw[0],CrdDraw[1],CrdDraw[2]);
		glutSolidSphere(8.0f,20,20);

		for (int i = 1; i < Len ; i++)
		{
			//indexP = (i-1)*3;
			//indexI = i*3;
			glTranslatef(CrdDraw[indexI]-CrdDraw[indexP],CrdDraw[indexI+1]-CrdDraw[indexP+1],CrdDraw[indexI+2]-CrdDraw[indexP+2]);
			glutSolidSphere(8.0f,20,20);
			indexP = indexI;
			indexI += 3;
		}

	glPopMatrix();


	indexI = 0;
	//glColor4f(0.0f,0.0f,1.0f,1.0f);
	glColor3f(0.0f,0.0f,1.0f);
	glLineWidth(5.0f);
	glBegin(GL_LINE_STRIP);
		
		for (int i = 0; i < Len ; i++)
		{
			glVertex3f(CrdDraw[indexI],CrdDraw[indexI+1],CrdDraw[indexI+2]);
			indexI += 3;
		}

	glEnd();

}


void C2MWrite2Txt(int datasize, double data[], int dataindexstart, int switchnumb , fstream &tempfile)
{
//fstream aces;
	int wait_loop=0;  //switch 內不能放變數宣告
	double M2C_ID=0;

	switch(switchnumb)
	{
		case 0://writefile portotype
			tempfile.open("tempsaving.txt",ios::out| ios::trunc);
			tempfile<<255<<"\t";
			for(int i=0;i<datasize;i++)
			{
				tempfile<<data[dataindexstart+i]<<"\t";
			}
			tempfile<<"\n";
			tempfile.close();
			break;

		case 1://writingfile test (offlin ver.)
			tempfile.open("tempsaving2.txt",ios::out| ios::app);
			tempfile<<255<<"\t";
			for(int i=0;i<datasize;i++)
			{
				tempfile<<data[dataindexstart+i]<<"\t";
			}
			tempfile<<"\n";
			tempfile.close();
			break;

		case 2://writingfile test (offlin ver.)
			tempfile.open("Qth.txt",ios::out| ios::app);

			//20121218doratom為了要看腳尖的軌跡(走斜坡用)//
			//tempfile.open("slopedora.txt",ios::out| ios::app);
			//20121218doratom為了要看腳尖的軌跡(走斜坡用)//

			tempfile<<123<<"\t";
			for(int i=0;i<datasize;i++)
			{
				tempfile<<data[dataindexstart+i]<<"\t";
			}
			tempfile<<"\n";
			tempfile.close();
			break;

		case 3://formal communication
			while(1)
			{	
				tempfile.open("C:/Users/user/Desktop/MatlabAdamsSimu_original/test.txt",ios::in);
				for(int g=0;g<datasize+1;g++)
				{
					if(g < 12)
					tempfile>>gKineAll.AdamsFS[g];
					else
					tempfile>>M2C_ID;
				}
				//printf("M2C_ID: %f\t",M2C_ID);
				tempfile.close();

				if(M2C_ID==127)
				{
					//fstream temp;
					//temp.open("CC.txt",ios::out|ios::app);
					//for(int i=0;i<12;i++)
					//	{
					//		temp<<(gKineAll.AdamsTorque[i]/1000/gKineAll.RatedTorque[i]/ gKineAll.MotorRated[i]) <<" ";
					//	}
					//temp<<endl;
					//temp.close();
					
					tempfile.open("C:/Users/user/Desktop/MatlabAdamsSimu_original/test.txt",ios::out| ios::trunc);
					for(int i=0;i<datasize;i++)
					{
						tempfile<<data[dataindexstart+i]<<"\t";
					}
					tempfile<<255<<"\t";
					tempfile<<"\n";
					tempfile.close();	
					printf("wait_loop: %d\n",wait_loop);
					wait_loop=0;
					break;
				}
				//printf("wait_loop: %d\n",wait_loop);
				wait_loop++;
				Sleep(5);
			}
			break;

		case 4:   //using error command to stop Matlab
			tempfile.open("C:/Users/user/Desktop/MatlabAdamsSimu_original/test.txt",ios::out| ios::trunc);
			for(int i=0;i<datasize;i++)
			{
				tempfile<<0<<"\t";
			}
			tempfile<<511<<"\t";
			tempfile<<"\n";
			tempfile.close();	
			break;

		case 5://formal communicationz
			double tempZMP[2];
			while(1)
			{	
				tempfile.open("C:/Users/user/Desktop/MatlabAdamsSimu_original/test.txt",ios::in);
				for(int g=0;g<datasize+1;g++)
				{
					if(g==0)
						tempfile>>tempZMP[0];
					else if(g==1)
						tempfile>>tempZMP[1];
					else
					tempfile>>M2C_ID;
				}
				printf("M2C_ID: %f\t",M2C_ID);
				tempfile.close();

				if(M2C_ID==127)
				{
					gLQs.YState[dataindexstart].data[2]=tempZMP[0]*0.75+gLQs.YState[dataindexstart].data[2]*0.25;
					gLQs.XState[dataindexstart].data[2]=tempZMP[1]*0.25+gLQs.XState[dataindexstart].data[2]*0.75;

					tempfile.open("C:/Users/user/Desktop/MatlabAdamsSimu/test.txt",ios::out| ios::trunc);
					for(int i=0;i<datasize;i++)
					{
						tempfile<<data[i]<<"\t";
					}
					tempfile<<255<<"\t";
					tempfile<<"\n";
					tempfile.close();	
					wait_loop=0;
					break;
				}
				printf("wait_loop: %d\n",wait_loop);
				wait_loop++;
				Sleep(5);
			}
			break;

		default:
			break;
   }
}

void C2MLoadTraj(int datanumb, double data[], int dataindexstart, fstream &tempfile)
{
	switch(datanumb) 
	{ 
		case 6 :
		  //20121218doratom為了要看腳尖的軌跡(走斜坡用)//
			for(int h = 0; h < 3 ; h++)
			{
				data[h]=gKineAll.CrdAll->data[36+h];//左腳腳尖
				data[h+3]=gKineAll.CrdAll->data[75+h];//右腳腳尖

				//data[h]=gKineAll.CrdAll->data[33+h];//左腳腳跟
				//data[h+3]=gKineAll.CrdAll->data[72+h];//右腳腳跟
			}
			//20121218doratom為了要看腳尖的軌跡(走斜坡用)//
			break;
	
		case 12:  // Biped Traj
			for (int h = 0 ; h < 6 ; h++)
			{
				data[h]=gKineAll.FKLLeg->theta[h+1];
				data[h+6]=gKineAll.FKRLeg->theta[h+1];
			}
			break;

		case 18:  // Biped Traj & Footpad Location (projection of ankle posi. on the ground)
			for (int h = 0 ; h < 6 ; h++)
			{
				data[h]=gKineAll.FKLLeg->theta[h+1];
				data[h+6]=gKineAll.FKRLeg->theta[h+1];
			}
			for (int h = 0 ; h < 3 ; h++)
			{
				data[h+12]=gKineAll.CrdAll->data[h+7*3];
				data[h+15]=gKineAll.CrdAll->data[h+7*3+13*3];									
			}
			break;

		case 20:  // Biped Traj & Footpad Location (projection of ankle posi. on the ground)
			for (int h = 0 ; h < 6 ; h++)
			{
				data[h]=gKineAll.FKLLeg->theta[h+1];
				data[h+6]=gKineAll.FKRLeg->theta[h+1];
			}
			for (int h = 0 ; h < 3 ; h++)
			{
				data[h+12]=gKineAll.CrdAll->data[h+7*3];
				data[h+15]=gKineAll.CrdAll->data[h+7*3+13*3];									
			}
			data[18]=gLQs.YState[dataindexstart].data[0];
			data[19]=gLQs.XState[dataindexstart].data[0];
			
			break;

		case 34:  // Biped Traj & Footpad Location (projection of ankle posi. on the ground)
			for (int h = 0 ; h < 6 ; h++)
			{
				data[h]=gKineAll.FKLLeg->theta[h+1];
				data[h+6]=gKineAll.FKRLeg->theta[h+1];
				data[h+14]=gKineAll.FKLArm->theta[h+4];
				data[h+20]=gKineAll.FKRArm->theta[h+4];
			}
				data[12]=gKineAll.FKLArm->theta[1];
				data[13]=gKineAll.FKLArm->theta[2];		
			for (int h = 0 ; h < 3 ; h++)
			{
				data[h+26]=gKineAll.CrdAll->data[h+7*3];
				data[h+29]=gKineAll.CrdAll->data[h+7*3+13*3];									
			}
			data[32]=gLQs.YState[dataindexstart].data[0];
			data[33]=gLQs.XState[dataindexstart].data[0];
			
			break;

		case 35:  // Biped Traj & Footpad Location (projection of ankle posi. on the ground)
			for (int h = 0 ; h < 6 ; h++)
			{
				if(h==4)
				{
					if(gKineAll.selIK == 0) //left support
					{
						data[h+6]=gKineAll.FKRLeg->theta[5]-gKineAll.AnklePitchRef[gIthIK%gStepSample]/cos(gKineAll.FKRLeg->theta[6]);
						data[h]=gKineAll.FKLLeg->theta[h+1];
					}
					else if (gKineAll.selIK == 1 || gKineAll.selIK == 2)
					{
						data[h]=gKineAll.FKLLeg->theta[5]+gKineAll.AnklePitchRef[gIthIK%gStepSample]/cos(gKineAll.FKLLeg->theta[6]);	
						data[h+6]=gKineAll.FKRLeg->theta[h+1];
					}
				}
				else
				{
				data[h]=gKineAll.FKLLeg->theta[h+1];
				data[h+6]=gKineAll.FKRLeg->theta[h+1];
				}
				data[h+12]=gKineAll.FKLArm->theta[h+4];
				data[h+18]=gKineAll.FKRArm->theta[h+4];
			}
				data[24]=gKineAll.FKLArm->theta[1];
				data[25]=gKineAll.FKLArm->theta[2];		
			for (int h = 0 ; h < 3 ; h++)
			{
				data[h+26]=gKineAll.CrdAll->data[h+7*3];
				data[h+29]=gKineAll.CrdAll->data[h+7*3+13*3];									
			}
			data[32]=gLQs.YState[dataindexstart].data[0];
			data[33]=gLQs.XState[dataindexstart].data[0];
			break;

		case 456:  // Freely assigned for data savingr
				//data[0]=gKineAll.CrdAll->data[0];
				//data[1]=gKineAll.CrdAll->data[1];
				//data[2]=gKineAll.CrdAll->data[2];


				//data[0]=gKineAll.COG[0];
				//data[1]=gKineAll.COG[1];
				//data[2]=gKineAll.COG[2];

			data[0]=gKineAll.COGDev[0];
				data[1]=gKineAll.COGDev[1];
				data[2]=gKineAll.COGDev[2];
				//for (int h = 0 ; h < 12 ; h++)
				//	data[h]=gKineAll.Ja->data[21*24+h];
				//for (int h = 0 ; h < 12 ; h++)	
				//	data[h+12]=gKineAll.Ja->data[22*24+h];
				//for (int h = 0 ; h < 12 ; h++)
				//	data[h+24]=gKineAll.Ja->data[23*24+h];

				
			break;

		case 46:  // Biped Traj & Footpad Location (projection of ankle posi. on the ground) &Foot Torque Trajectory
			for (int h = 0 ; h < 6 ; h++)
			{
				data[h]=gKineAll.FKLLeg->theta[h+1];
				data[h+6]=gKineAll.FKRLeg->theta[h+1];
				data[h+12]=gKineAll.FKLArm->theta[h+4];
				data[h+18]=gKineAll.FKRArm->theta[h+4];
				data[h+34]=gKineAll.MotorTorq[h];
				data[h+40]=gKineAll.MotorTorq[h+6];
			}
				data[24]=gKineAll.FKLArm->theta[1];
				data[25]=gKineAll.FKLArm->theta[2];		
			for (int h = 0 ; h < 3 ; h++)
			{
				data[h+26]=gKineAll.CrdAll->data[h+7*3];
				data[h+29]=gKineAll.CrdAll->data[h+7*3+13*3];									
			}
			data[32]=gLQs.YState[dataindexstart].data[0];
			data[33]=gLQs.XState[dataindexstart].data[0];
			
			break;

		default:
			break;
	}

}

void gCoPCali(int* FlagL, int* FlagR)
{
	/******************************************************************
	input: FlagL, FlagR 
	output: CaliPitchAngL CaliPitchAngR CaliRollAngL CaliRollAngR 
			CaliRollHipL CaliRollHipR CaliPitchHipL CaliPitchHipR
			CaliFlagR CaliFlagL

	Note:
	// 藉由力規值得到的兩腳CoP位置調整Cali的腳底板角度
	// 旗標與校正的角度分別以指標和全域變數控制
	// 角度須為累加 因為EPOS要給的為絕對的ENC數
	// COP位置為機器人座標之COP 前方為x 左為y
	// 注意在此COP的準位已為+-80 若腳的起始寬度有改變 deadzone要手動調整
	// 以ZMP為準位(0,0)
	// x方向的準為仍舊為0 y方向依左右腳則有+-80的差別
	// 注意 機器人座標與OpenGL座標不同
	// 注意 如果在gCoPCali有調整ZMP原點 在gTrackingZMP也要一並調整
	// WZ 20130301
	******************************************************************/
#if TwinCAT_Mode	
	//double error;
	double deadzone = 5;
	double deadzoneCoP = 10;
	long Ang = long(74896.4438101*3.14159265/180/500);	
	long nError;
	long nError2;

	if(gKineAll.FS_ZMP[1] > deadzone) // 表示重心在左腳 左腳為support
		{
			CaliRollAngL += Ang;
			CaliRollAngR += Ang;
			CaliRollHipL -= Ang;
			CaliRollHipR -= Ang;
		}
	else if(gKineAll.FS_ZMP[1] < -deadzone) // 表示重心在右腳 右腳為support
		{
			CaliRollAngL -= Ang;
			CaliRollAngR -= Ang;
			CaliRollHipL += Ang;
			CaliRollHipR += Ang;
		}

	// 左右已經調完了 所以重心分配完成 然後調x方向
	else if(gKineAll.FS_ZMP[0] > deadzone)// COP(ZMP)在前 腳板往下給反作用力 髖關節也往下 將身體擺正
		{
			CaliPitchHipL += Ang;
			CaliPitchHipR -= Ang;
			CaliPitchAngL -= Ang;
			CaliPitchAngR += Ang;
		}
	else if(gKineAll.FS_ZMP[0] < -deadzone)
		{
			CaliPitchHipL -= Ang;
			CaliPitchHipR += Ang;
			CaliPitchAngL += Ang;
			CaliPitchAngR -= Ang;
		}
	else
		{
			CaliCount += 1;
		}

	///////////////////////////////////////////////////////////
	// ZMP調完 調兩腳COP 兩腳皆當作swing腳來調
	if(*FlagL==2)
		CaliCountL += 1;

	if(*FlagR==2)
		CaliCountR += 1;

	// 左腳
	if(CaliCountL < 200)
	{
		if(gKineAll.CoPL[0] > deadzoneCoP)
		{	
			CaliPitchAngL += Ang;
			*FlagL = 0;
		}
		else if (gKineAll.CoPL[0] < -deadzoneCoP)
		{
			CaliPitchAngL -= Ang;
			*FlagL = 0;
		}
		else
			*FlagL = 1;

		if(gKineAll.CoPL[1] > (80) && *FlagL ==1)
			CaliRollAngL += Ang;
		else if (gKineAll.CoPL[1] < (-deadzoneCoP+80) && *FlagL ==1)
			CaliRollAngL -= Ang;
		else if(*FlagL ==1)
			*FlagL = 2;
	}

	//右腳
	else if(CaliCountL > 200)
	{
		if(gKineAll.CoPR[0] > deadzoneCoP)
		{
			CaliPitchAngR -= Ang;
			*FlagR = 0;	
		}
		else if (gKineAll.CoPR[0] < -deadzoneCoP)
		{
			CaliPitchAngR += Ang;
			*FlagR = 0;
		}
		else
			*FlagR = 1;

		if(gKineAll.CoPR[1] > (deadzoneCoP-80) && *FlagR == 1)
			CaliRollAngR += Ang;
		else if (gKineAll.CoPR[1] < -80 && *FlagR == 1)
			CaliRollAngR -= Ang;
		else if(*FlagR == 1)
			*FlagR = 2;
		//if(CoPRy > deadzoneCoP && *FlagR == 1)
		//	CaliRollAngR += Ang;
		//else if (CoPRy < -deadzoneCoP && *FlagR == 1)
		//	CaliRollAngR -= Ang;
		//else if(*FlagR == 1)
		//	*FlagR = 2;
	}

	// PitchAngL
			TCAT->pTLL1msOut->LL_O_TarEnc_05=CaliPitchAngL;

	// RollAngL	
			TCAT->pTLL1msOut->LL_O_TarEnc_06=CaliRollAngL;

	// PitchHipL
			TCAT->pTLL1msOut->LL_O_TarEnc_03=CaliPitchHipL;

	// RollHipL
			TCAT->pTLL1msOut->LL_O_TarEnc_02=CaliRollHipL;	
			
	// PitchAngR
			TCAT->pTRL1msOut->RL_O_TarEnc_05=CaliPitchAngR;

	// RollAngR
			TCAT->pTRL1msOut->RL_O_TarEnc_06=CaliRollAngR;

	// RollHipR
			TCAT->pTRL1msOut->RL_O_TarEnc_02=CaliRollHipR;

	// PitchHipR
			TCAT->pTRL1msOut->RL_O_TarEnc_03=CaliPitchHipR;

	if ( ((nError = TCatIoInputUpdate( TASK_RLEG_PORTNUMBER )) == 0 )&& ((nError2 = TCatIoOutputUpdate( TASK_RLEG_PORTNUMBER )) == 0 )&&
				 (( nError = TCatIoInputUpdate( TASK_LLEG_PORTNUMBER )) == 0 )&& ( (nError2 = TCatIoOutputUpdate( TASK_LLEG_PORTNUMBER )) == 0 )) 
	{ 
					// do your calculation and logic 
					//pT1msOut->AnalogOut = pT1msIn->AnalogIn; 
					// start the I/O update and field bus cycle 
										
					TCatIoOutputUpdate( TASK_RLEG_PORTNUMBER ); 
					TCatIoOutputUpdate( TASK_LLEG_PORTNUMBER ); 
					//CC.open("CC.txt",ios::out| ios::app);
					//CC<<pTRL1msIn->RL_I_Enc_04<<"\t";
					//CC.close();
	}

	if(CaliCountR > 200)
	{
		*FlagL = 3;
		*FlagR = 3;
		CaliCountR = 0;
		CaliCountL = 0;
	}

	if(CaliCount==100) //調完第一次ZMP之後還是會繼續調ZMP 只是不會再把FLAG變成4
	{
		*FlagL = 4;
		*FlagR = 4;	
	}
					
#endif
}

void gCaliManual(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 手動調整個軸角度
	// WZ 20130401
	******************************************************************/
#if TwinCAT_Mode
	long Ang = long(74896.4438101*3.14159265/180);	//角度*ANG=ENC
	long AngYaw = long(78353.2027529*3.14159265/180);	
	//double Angle = 0;
	long nError;
	long nError2;
	double weight = 500;
	long Enc = ( Ang / weight );
	long EncYaw = ( AngYaw / weight);
	long Encoder = 0;
	double error = 10;	//Encoder
	//for(int i = 0 ; i < 12 ; i++)
	//	CaliTemp[i] = 0;
	while(1)
	{
		cout<<"左腳上到下1~6"<<"\t"<<"右腳上到下7~12"<<"輸入 0 結束調整"<<endl;
		cin>> axisNumber;
		
		if(axisNumber==0)
			break;
		else if(axisNumber<13 && axisNumber>0)
		{
			cout<<"請輸入欲調整 相對 角度(不得超過+-5度)"<<endl;
			cin>> CaliAngle;	//(角度)
			Encoder = CaliTemp[axisNumber-1];	// 將該軸最後的值取出(ENC)
			if(axisNumber==1 || axisNumber ==7)	
				CaliTemp[axisNumber-1] += CaliAngle*EncYaw*weight;	// 決定最終要追到的ENC數
			else
				CaliTemp[axisNumber-1] += CaliAngle*Enc*weight;
		}
		else
			cout<<"左腳上到下1~6"<<"\t"<<"右腳上到下7~12"<<"\t"<<"輸入 0 結束調整"<<endl;

		while(1)
		{
			
			if(CaliAngle < 5 && CaliAngle > -5)	
			{
				// 決定丟入TWINCAT的Count數
				if(CaliAngle > 0)
				{
					if(axisNumber==1 || axisNumber ==7)	
					{
						Encoder += EncYaw;	//ENC
						if(CaliTemp[axisNumber-1]-Encoder < error)// 追到了(Encoder)
						{
							//CaliTemp[axisNumber] = Angle;
							//Angle = 0;
							break;	
						}
					}
					else
					{
						Encoder += Enc;				
						if(CaliTemp[axisNumber-1]-Encoder < error)// 追到了
						{
							//CaliTemp[axisNumber] = Angle;
							//Angle = 0;
							break;	
						}
					}

				}
				else if(CaliAngle < 0)
				{
					if(axisNumber==1 || axisNumber ==7)
					{
						Encoder -= EncYaw;
						if(Encoder-CaliTemp[axisNumber-1] < error)// 追到了
						{
							//Angle = 0;
							break;	
						}
					}	
					else
					{
						Encoder -= Enc;
						if(Encoder-CaliTemp[axisNumber-1] < error)// 追到了
						{
							//Angle = 0;
							break;	
						}
					}
				}
				else//輸入調整角度CaliAngle==0
				{
					cout<<"不要鬧囉^___________^"<<endl;
					break;
				}

				// Count數決定後 開始調
				if(axisNumber==1)
				{
					//cout<<"左腳Hip Yaw "<<Angle<<"度"<<endl;
					TCAT->pTLL1msOut->LL_O_TarEnc_01=Encoder;
				}
				else if(axisNumber==2)
				{
					//cout<<"左腳Hip Roll "<<Angle<<"度"<<endl;
					TCAT->pTLL1msOut->LL_O_TarEnc_02=Encoder;							
				}

				else if(axisNumber==3)
				{
					//cout<<"左腳Hip Pitch "<<Angle<<"度"<<endl;
					TCAT->pTLL1msOut->LL_O_TarEnc_03=Encoder;								
				}
	
				else if(axisNumber==4)
				{
					//cout<<"左腳Knee Pitch "<<Angle<<"度"<<endl;
					TCAT->pTLL1msOut->LL_O_TarEnc_04=Encoder;								
				}

				else if(axisNumber==5)
				{
					//cout<<"左腳Ankle Pitch "<<Angle<<"度"<<endl;
					TCAT->pTLL1msOut->LL_O_TarEnc_05=Encoder;								
				}
	
				else if(axisNumber==6)
				{
					//cout<<"左腳Ankle Roll "<<Angle<<"度"<<endl;
					TCAT->pTLL1msOut->LL_O_TarEnc_06=Encoder;						
				}

				else if(axisNumber==7)
				{
					//cout<<"右腳Hip Yaw "<<Angle<<"度"<<endl;
					TCAT->pTRL1msOut->RL_O_TarEnc_01=Encoder;								
				}

				else if(axisNumber==8)
				{
					//cout<<"右腳Hip Roll "<<Angle<<"度"<<endl;
					TCAT->pTRL1msOut->RL_O_TarEnc_02=Encoder;								
				}

				else if(axisNumber==9)
				{
					//cout<<"右腳Hip Pitch "<<Angle<<"度"<<endl;
					TCAT->pTRL1msOut->RL_O_TarEnc_03=Encoder;								
				}

				else if(axisNumber==10)
				{
					//cout<<"右腳Knee Pitch "<<Angle<<"度"<<endl;
					TCAT->pTRL1msOut->RL_O_TarEnc_04=Encoder;								
				}

				else if(axisNumber==11)
				{
					//cout<<"右腳Ankle Pitch "<<Angle<<"度"<<endl;
					TCAT->pTRL1msOut->RL_O_TarEnc_05=Encoder;									
				}

				else if(axisNumber==12)
				{
					//cout<<"右腳Ankle Roll "<<Angle<<"度"<<endl;
					TCAT->pTRL1msOut->RL_O_TarEnc_06=Encoder;									
				}

			
				if ( ((nError = TCatIoInputUpdate( TASK_RLEG_PORTNUMBER )) == 0 )&& ((nError2 = TCatIoOutputUpdate( TASK_RLEG_PORTNUMBER )) == 0 )&&
										 (( nError = TCatIoInputUpdate( TASK_LLEG_PORTNUMBER )) == 0 )&& ( (nError2 = TCatIoOutputUpdate( TASK_LLEG_PORTNUMBER )) == 0 )) 
				{ 
					// do your calculation and logic 
					//pT1msOut->AnalogOut = pT1msIn->AnalogIn; 
					// start the I/O update and field bus cycle 
										
					TCatIoOutputUpdate( TASK_RLEG_PORTNUMBER ); 
					TCatIoOutputUpdate( TASK_LLEG_PORTNUMBER ); 
					//CC.open("CC.txt",ios::out| ios::app);
					//CC<<pTRL1msIn->RL_I_Enc_04<<"\t";
					//CC.close();
				}
			}
			else
			{
				cout<<"請輸入欲調整角度(不得超過5度)"<<endl;
				break;
			}
			
		Sleep(10);
		}		
		CaliTemp[axisNumber-1] = Encoder; //記錄該軸最後Encoder絕對位置
		Encoder = 0;
	}

#endif
}


void gPreProcessDataHand(unsigned int mode,unsigned int stimes,float interval ,unsigned int axinumb) // 至峻 20130410
{
	////////////////////////////////////////
	////////////////////////////////////////
	//if(	 g_start_sensing == true)
	//{
		commmode=mode;
		
		//ID
		int number1=0;
		//set PID
		int kp=0;int ki=0;int kd=0;int errorsumbound=0;
		//set encoder
		float ang=0; long encoder=0;float ang1=0;long encoder1=0;
		int timeout=0;
		int waittimes=80;
		////////////Test///////////
		int nn=0;
		///////////////////////////
		int whand=0;

		//int AxisNum = 0;

		// 設定哪些軸要Active
		//unsigned char AxisOnBus[13] = {253,0,1,1,1,1,1,0,1,1,1,1,1}; // 第一個是識別碼，從第二個開始是馬達各軸要不要active
		//                            // cmd,1,2,3,4,5,6,7,8,9,0,1,2
		unsigned char AxisOnBus[13] = {253,1,1,1,1,1,1,1,1,1,1,1,1}; // 第一個是識別碼，從第二個開始是馬達各軸要不要active 第1~12軸
									// cmd,1,2,3,4,5,6,7,8,9,0,1,2
		unsigned char AxisOnBus2[13] = {254,1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // 第一個是識別碼，從第二個開始是馬達各軸要不要active 第13~24軸
									 // cmd,13,14,15,16,17,18,19,20,21,22,23,24

		unsigned int NumOfTrajSample = 0;
		unsigned int NumOfTrajSample1 = 0;

		//set PWM 用到
		unsigned char UpperLim = 99;
		unsigned char LowerLim = 10;
		unsigned int  OkErrBound = 6000; // 800/2000/160/1.47*360= 0.61度


		switch(mode)
		{
		case 0: // 機構歸home

			//設定7~12軸Go Home(右手)
			//設for loop 一次Go Home一軸

			//for (int axNum = 1; axNum <= 12; axNum++)
			//{
			//		g_InitOneAxis(axNum);
			//		Sleep(200);
			//}

			//for (int axNum = 1; axNum <= 6; axNum++)
			//{
			//		g_InitOneAxis(axNum);
			//		Sleep(200);
			//}

			break; 

		case 1:
			//	SetPID(); 
			//#define ID_SetPID1	
			cout<<"Position PID Parameter Setting Mode"<<endl;
			cout<<"Which hand would be control : (Right=0 , Left=1) ";
				scanf("%d", &whand);
			cout<<"Please Type the Axis Number You Want to Modify: ";
				scanf("%d", &number1); // %d 代表十進位整數
			cout<<"Please Type the Kp: ";
				scanf("%d", &kp);
			cout<<"Please Type the Ki: ";
				scanf("%d", &ki);
			cout<<"Please Type the Kd: ";
				scanf("%d", &kd);
			cout<<"Please Type the Bound of ErrorSum: ";
				scanf("%d", &errorsumbound);

				DP->TxData[0]=ID_SetPID+CMD_SET*number1;
				DP->TxData[1]=(unsigned char)kp;
				DP->TxData[2]=(unsigned char)(kp>>8);//位移8bits，因為unsigned char只有0~255
				DP->TxData[3]=(unsigned char)ki;
				DP->TxData[4]=(unsigned char)(ki>>8);
				DP->TxData[5]=(unsigned char)kd;
				DP->TxData[6]=(unsigned char)(kd>>8);
				DP->TxData[7]=(unsigned char)errorsumbound;
				DP->TxData[8]=(unsigned char)(errorsumbound>>8);	

				if(whand=1)
				{
					gpPortLH->_write(DP->TxData,9);   
				}
				else if(whand=0)
				{
					gpPortRH->_write(DP->TxData,9);   
				}
				else
				{
					cout<<"The Axis isn't exist."<<endl;
				}

			cout<<"Potencial PID Parameter Setting Mode"<<endl;			

			cout<<"Please Type the Kp: ";
				scanf("%d", &kp);
			cout<<"Please Type the Ki: ";
				scanf("%d", &ki);
			cout<<"Please Type the Kd: ";
				scanf("%d", &kd);
			cout<<"Please Type the Bound of ErrorSum: ";
				scanf("%d", &errorsumbound);

				DP->TxData[0]=ID_SetpotPID+CMD_SET*number1;
				DP->TxData[1]=(unsigned char)kp;
				DP->TxData[2]=(unsigned char)(kp>>8);
				DP->TxData[3]=(unsigned char)ki;
				DP->TxData[4]=(unsigned char)(ki>>8);
				DP->TxData[5]=(unsigned char)kd;
				DP->TxData[6]=(unsigned char)(kd>>8);
				DP->TxData[7]=(unsigned char)errorsumbound;
				DP->TxData[8]=(unsigned char)(errorsumbound>>8);

				if(whand==1)
				{
					gpPortLH->_write(DP->TxData,9);   
				}
				else if(whand==0)
				{
					gpPortRH->_write(DP->TxData,9);   
				}
				else
				{
					cout<<"The Axis does not exist."<<endl;
				}	

			break;

		case 2:
			//	Manual();

			//if (SetPID_Done == false)
			//{
			//	cout << "先設定PID用以解鎖，不然無法連續控制" << endl;
			//	system("pause");
			//}
			//else
			//{
				DP->State[2]=ID_SetOneAxis;
				gpPortRH->_write(DP->State,5);	
				cout<<"Which hand to be control : (Right=0 , Left=1) ";
				scanf("%d", &whand);
				cout<<"Please Type the Axis Number You Want to Modify: ";
					scanf("%d", &number1);
				cout<<"Please Assign the Potentical Turning Angle: ";
					scanf("%f", &ang);
				cout<<"Please Assign the Position Turning Angle: ";
					scanf("%f", &ang1);

				//cout<<"Manual Mode"<<endl;
				//乘上減速比換成encoder
				//pot
				encoder= ang;//(long int)((ang*500*4*160*50)/360.0/34.0); //!!!!!
				//pos
				encoder1= (long int)(ang1*3000/99);

				DP->TxData[0] = ID_SendReadENC+CMD_SET*number1;
				DP->TxData[1] = (signed long)encoder;
                DP->TxData[2] = (signed long)encoder>>8;
				DP->TxData[3] = (signed long)encoder1;
                DP->TxData[4] = (signed long)encoder1>>8;
				DP->TxData[5] = 0;
				DP->TxData[6] = 0;
				DP->TxData[7] = 0;
				DP->TxData[8] = 0; 

				//DP->TransENC(encoder,ID_SetENC+number1*CMD_SET);
				if(whand==1)
				{
					gpPortLH->_write(DP->TxData,9);
					buffer[0] = 0;

					Sleep(500);
					//回傳值，確認是否有傳到C30
					if(gpPortLH->read(buffer, 5,100,NULL))
					{
						if (int(buffer[0])%16 == IDR_LastCMD_Succ)
						{
							cout<<"Homing Aaxis " << int(buffer[0])/16 << " Completed, " << int(buffer[1])+int(buffer[2])*256 <<"   "<< int(buffer[3])+int(buffer[4])*256 << " Returned "<<endl;
						}
						else
						{
							cout << "Return " << int(buffer[0]) << " wrong response" << endl;
						}
					}
					else
					{
						cout << "axis " << number1 << " no response" << endl;
					}

					buffer[0] = 0;

					Sleep(100);
				}
				else if(whand==0)
				{
					gpPortRH->_write(DP->TxData,9);
					buffer[0] = 0;

					Sleep(500);
					//回傳值，確認是否有傳到C30
					if(gpPortRH->read(buffer, 5,100,NULL))
					{
						if (int(buffer[0])%16 == IDR_LastCMD_Succ)
						{
							cout<<"Homing Aaxis " << int(buffer[0])/16 << " Completed, " << int(buffer[1])+int(buffer[2])*256 <<"   "<< int(buffer[3])+int(buffer[4])*256 << " Returned "<<endl;
						}
						else
						{
							cout << "Return " << int(buffer[0]) << " wrong response" << endl;
						}
					}
					else
					{
						cout << "axis " << number1 << " no response" << endl;
					}

					buffer[0] = 0;
					buffer[1] = 0;
					buffer[2] = 0;
					buffer[3] = 0;
					buffer[4] = 0;
					Sleep(100);
				}
				else
				{
					cout<<"The Axis isn't exist."<<endl;
				}
			//}

			break; 

		case 3:
			//	Set Idle();  ok
 			DP->State[2]=SP_IDLE;
			gpPortLH->_write(DP->State,5); 

			break; 

		case 4:
			////	Sinusoid();
			//DP->NodeWathcing(axinumb);//準備軌跡
			//g_ss=true;
			//g_sf=true;
			//g_stimes=stimes;
			//cominterval=interval;

			//SendContTraj=true; 		//g_SendData(1,0)~g_SendData(1,9)

			//DP->State[2]=ID_SetTrajPacket; //ID_SetTrajPacket
			//gpPortLH->_write(DP->State,5);

			break;

		case 5:
			//	Traj();	

			//if (SetPID_Done == false)
			//{
			//	cout << "先設定PID用以解鎖，不然無法連續控制" << endl;
			//}
			//else
			//{

				gpPortLH->_write(AxisOnBus,13); // 設定需要啟動的軸 1~12
				Sleep(150);

				//gpPortLH->_write(AxisOnBus2,13); // 設定需要啟動的軸 13~24
				//Sleep(150);

				DP->State[2]=ID_SetTrajPacket; //ID_SetTrajPacket
				gpPortLH->_write(DP->State,5);

				Sleep(150);

				NumOfTrajSample = gfContTrajLenLH;

				DP->TxData[0]=0; // reserved
				DP->TxData[1]=5; // sapling time  = TxData[1] + TxData[2] * 256
				DP->TxData[2]=0;
				DP->TxData[3]=12; // num of axis = TxData[3] + TxData[4] * 256
				DP->TxData[4]=0;
				DP->TxData[5]=(unsigned char)NumOfTrajSample;
				DP->TxData[6]=(unsigned char)(NumOfTrajSample>>8); // NumOfTrajSample = TxData[5] + TxData[6] * 256
				// TxData[5] 還有 TxData[6] 先不用到，也保留，PIC32會先直接四筆四筆抓
				DP->TxData[7]=0; // reserved
				DP->TxData[8]=0; // reserved
				gpPortLH->_write(DP->TxData,9); 

				Sleep(100);

				////////////////////////////////////////////////////////////////////

				gpPortRH->_write(AxisOnBus,13); // 設定需要啟動的軸 1~12
				Sleep(150);
				
				//gpPortRH->_write(AxisOnBus2,13); // 設定需要啟動的軸 13~24
				//Sleep(150);
								
				DP1->State[2]=ID_SetTrajPacket; //ID_SetTrajPacket
				gpPortRH->_write(DP1->State,5);
								
				Sleep(150);

				NumOfTrajSample1 = gfContTrajLenRH;

				DP1->TxData[0]=0; // reserved
				DP1->TxData[1]=5; // sapling time  = TxData[1] + TxData[2] * 256
				DP1->TxData[2]=0;
				DP1->TxData[3]=12; // num of axis = TxData[3] + TxData[4] * 256
				DP1->TxData[4]=0;
				DP1->TxData[5]=(unsigned char)NumOfTrajSample;
				DP1->TxData[6]=(unsigned char)(NumOfTrajSample>>8); // NumOfTrajSample = TxData[5] + TxData[6] * 256
				// TxData[5] 還有 TxData[6] 先不用到，也保留，PIC32會先直接四筆四筆抓
				DP1->TxData[7]=0; // reserved
				DP1->TxData[8]=0; // reserved
				gpPortRH->_write(DP1->TxData,9); 
				//system("pause");
				Sleep(100);

				gSendContTraj=true;
				gContTrajLock = false; // 解鎖


			//}
			break; 


		case 6:
			//	SetPID();



			for (int i = 1 ; i < 13 ; i++)
			{
				if (AxisOnBus[i] == 1)
				{

					if (i == 1 || i == 7) // hip yaw
					{
						kp = 300;
						ki = 0;
						kd = 0;
						errorsumbound = 100;
					}
					else if (i == 2 || i == 8) // hip roll
					{
						kp = 1100;
						ki = 0;
						kd = 150;
						errorsumbound = 100;
					}
					else if (i == 3 || i == 9) // hip pitch
					{
						kp = 800;
						ki = 0;
						kd = 100;
						errorsumbound = 100;
					}
					else if (i == 4 || i == 10) // knee
					{
						kp = 500;
						ki = 0;
						kd = 100;
						errorsumbound = 100;
					}
					else if (i == 5 || i == 11) // ankle pitch
					{
						kp = 700;
						ki = 0;
						kd = 100;
						errorsumbound = 100;
					}
					else if (i == 6 || i == 12) // ankle roll
					{
						kp = 800;
						ki = 0;
						kd = 100;
						errorsumbound = 100;
					}


					DP->TxData[0]=ID_SetPID+CMD_SET*i;
					DP->TxData[1]=(unsigned char)kp;
					DP->TxData[2]=(unsigned char)(kp>>8);
					DP->TxData[3]=(unsigned char)ki;
					DP->TxData[4]=(unsigned char)(ki>>8);
					DP->TxData[5]=(unsigned char)kd;
					DP->TxData[6]=(unsigned char)(kd>>8);
					DP->TxData[7]=(unsigned char)errorsumbound;
					DP->TxData[8]=(unsigned char)(errorsumbound>>8);
					gpPortLH->_write(DP->TxData,9); 

					Sleep(150);

					DP1->TxData[0]=ID_SetPID+CMD_SET*i;
					DP1->TxData[1]=(unsigned char)kp;
					DP1->TxData[2]=(unsigned char)(kp>>8);
					DP1->TxData[3]=(unsigned char)ki;
					DP1->TxData[4]=(unsigned char)(ki>>8);
					DP1->TxData[5]=(unsigned char)kd;
					DP1->TxData[6]=(unsigned char)(kd>>8);
					DP1->TxData[7]=(unsigned char)errorsumbound;
					DP1->TxData[8]=(unsigned char)(errorsumbound>>8);
					gpPortRH->_write(DP1->TxData,9); 


					cout << "Sending Axis " << i << endl;
					Sleep(150);

				}
			}

			//SetPID_Done = true;

			//printf("沒有資料送出!! 為了debug!!! 請改回來 上面三行");


			break;


		case 7:
			//	SetPWMLims();

			

			for (int i = 1 ; i < 13 ; i++)
			{
				if (AxisOnBus[i] == 1)
				{

					if (i == 4 || i == 10)
					{
						DP->TxData[0]=ID_SetPWMLimit+CMD_SET*i;
						DP->TxData[1]=80;
						DP->TxData[2]=LowerLim;
						DP->TxData[3]=OkErrBound%256;
						DP->TxData[4]=OkErrBound/256;
						DP->TxData[5]=0;
						DP->TxData[6]=0;
						DP->TxData[7]=0;
						DP->TxData[8]=0;

						DP1->TxData[0]=ID_SetPWMLimit+CMD_SET*i;
						DP1->TxData[1]=80;
						DP1->TxData[2]=LowerLim;
						DP1->TxData[3]=OkErrBound%256;
						DP1->TxData[4]=OkErrBound/256;
						DP1->TxData[5]=0;
						DP1->TxData[6]=0;
						DP1->TxData[7]=0;
						DP1->TxData[8]=0;
						
					}
					else
					{
						DP->TxData[0]=ID_SetPWMLimit+CMD_SET*i;
						DP->TxData[1]=UpperLim;
						DP->TxData[2]=LowerLim;
						DP->TxData[3]=OkErrBound%256;
						DP->TxData[4]=OkErrBound/256;
						DP->TxData[5]=0;
						DP->TxData[6]=0;
						DP->TxData[7]=0;
						DP->TxData[8]=0;

						DP1->TxData[0]=ID_SetPWMLimit+CMD_SET*i;
						DP1->TxData[1]=UpperLim;
						DP1->TxData[2]=LowerLim;
						DP1->TxData[3]=OkErrBound%256;
						DP1->TxData[4]=OkErrBound/256;
						DP1->TxData[5]=0;
						DP1->TxData[6]=0;
						DP1->TxData[7]=0;
						DP1->TxData[8]=0;
						
					}

					gpPortLH->_write(DP->TxData,9); 
					Sleep(150);
					gpPortRH->_write(DP1->TxData,9); 
					cout << "Sending Axis Limit" << i << endl;
					Sleep(150);
				}
			}

			break;

		case 8:
		// set all mechanism calibration

		//if (SetPID_Done == false)
		//{
		//	cout << "先設定PID用以解鎖，不然無法連續控制" << endl;
		//}
		//else
		{
			int jointNum = 0;
			jointNum = 3;
			DP->State[2]=ID_SetCaliENC;
			gpPortLH->_write(DP->State,5);
			encoder= (long int)((-1.1*500*4*160*50)/360.0/34.0); //!!!!!
			DP->TransENC(encoder,  ID_SetCaliENC+jointNum*CMD_SET);
			gpPortLH->_write(DP->TxData,9);
			Sleep(30);

			jointNum = 4;
			DP->State[2]=ID_SetCaliENC;
			gpPortLH->_write(DP->State,5);
			encoder= (long int)((0.8*500*4*160*50)/360.0/34.0); //!!!!!
			DP->TransENC(encoder,  ID_SetCaliENC+jointNum*CMD_SET);
			gpPortLH->_write(DP->TxData,9);
			Sleep(30);

			jointNum = 5;
			DP->State[2]=ID_SetCaliENC;
			gpPortLH->_write(DP->State,5);
			encoder= (long int)((0.4*500*4*160*50)/360.0/34.0); //!!!!!
			DP->TransENC(encoder,  ID_SetCaliENC+jointNum*CMD_SET);
			gpPortLH->_write(DP->TxData,9);
			Sleep(30);

			jointNum = 8;
			DP->State[2]=ID_SetCaliENC;
			gpPortLH->_write(DP->State,5);
			encoder= (long int)((0.46*500*4*160*50)/360.0/34.0); //!!!!!
			DP->TransENC(encoder,  ID_SetCaliENC+jointNum*CMD_SET);
			gpPortLH->_write(DP->TxData,9);
			Sleep(30);

			jointNum = 2;
			DP->State[2]=ID_SetCaliENC;
			gpPortLH->_write(DP->State,5);
			encoder= (long int)((-0.46*500*4*160*50)/360.0/34.0); //!!!!!
			DP->TransENC(encoder,  ID_SetCaliENC+jointNum*CMD_SET);
			gpPortLH->_write(DP->TxData,9);
			Sleep(30);

		}

			break;

		//合併到case1
		case 9:
			//	SetpotPID(); 
			//#define ID_SetPID1	
			cout<<"Potencial PID Parameter Setting Mode"<<endl;			
			cout<<"Please Type the Axis Number You Want to Modify: ";
				scanf("%d", &number1); // %d 代表十進位整數
			cout<<"Please Type the Kp: ";
				scanf("%d", &kp);
			cout<<"Please Type the Ki: ";
				scanf("%d", &ki);
			cout<<"Please Type the Kd: ";
				scanf("%d", &kd);
			cout<<"Please Type the Bound of ErrorSum: ";
				scanf("%d", &errorsumbound);

				DP->TxData[0]=ID_SetpotPID+CMD_SET*number1;
				DP->TxData[1]=(unsigned char)kp;
				DP->TxData[2]=(unsigned char)(kp>>8);
				DP->TxData[3]=(unsigned char)ki;
				DP->TxData[4]=(unsigned char)(ki>>8);
				DP->TxData[5]=(unsigned char)kd;
				DP->TxData[6]=(unsigned char)(kd>>8);
				DP->TxData[7]=(unsigned char)errorsumbound;
				DP->TxData[8]=(unsigned char)(errorsumbound>>8);

				if(whand==1)
				{
					gpPortLH->_write(DP->TxData,9);   
				}
				else if(whand==0)
				{
					gpPortRH->_write(DP->TxData,9);   
				}
				else
				{
					cout<<"The Axis does not exist."<<endl;
				}	
			

			break;

			case 10:
			//	TEST();
				cout<<"Which hand to be control : (Right=0 , Left=1) ";
				scanf("%d", &whand);
				cout << "Type testing axis : ";
					scanf("%d", &number1);
				cout << "Type testing times : ";				
					scanf("%d", &nn);

			if(whand==0)
			{
				for(int i = 0; i<nn; i++)
				{
					DP->State[2]=ID_SetOneAxis;
					gpPortRH->_write(DP->State,5);

						DP->TxData[0] = ID_SetENC+CMD_SET*number1;
						DP->TxData[1] = 0;
						DP->TxData[2] = 0;
						DP->TxData[3] = 0;
						DP->TxData[4] = 0;
						DP->TxData[5] = 0;
						DP->TxData[6] = 0;
						DP->TxData[7] = 0;
						DP->TxData[8] = 0; 
					gpPortRH->_write(DP->TxData,9);
					cout << "OK " << i+1 <<endl;
					Sleep(5);					
				}

				DP->TxData[0] = ID_ReadENCcount+CMD_SET*number1;
				DP->TxData[1] = 0;
				DP->TxData[2] = 0;
				DP->TxData[3] = 0;
				DP->TxData[4] = 0;
				DP->TxData[5] = 0;
				DP->TxData[6] = 0;
				DP->TxData[7] = 0;
				DP->TxData[8] = 0; 
				gpPortRH->_write(DP->TxData,9);
				buffer[0]=0;
				buffer[1]=0;
				buffer[2]=0;
				buffer[3]=0;
  				Sleep(500);
				if(gpPortRH->read(buffer, 5,100,NULL))
				{
					if (int(buffer[0])%16 == IDR_LastCMD_Succ)
					{
						cout<<"Testing Aaxis " << int(buffer[0])/16 << " Completed, " << "Total Count : "<<int(buffer[1])+int(buffer[2])*256+int(buffer[3])*256*256 << " Returned "<<endl;
					}
					else
					{
							cout << "Return " << int(buffer[0]) << " wrong response" << endl;
					}
				}

				else
				{
					cout << "axis " << number1 << " no response" << endl;
				}
			}

			else if (whand==1)
			{
				for(int i = 0; i<nn; i++)
				{
					DP->State[2]=ID_SetOneAxis;
					gpPortLH->_write(DP->State,5);

						DP->TxData[0] = ID_SetENC+CMD_SET*number1;
						DP->TxData[1] = 0;
						DP->TxData[2] = 0;
						DP->TxData[3] = 0;
						DP->TxData[4] = 0;
						DP->TxData[5] = 0;
						DP->TxData[6] = 0;
						DP->TxData[7] = 0;
						DP->TxData[8] = 0; 
					gpPortLH->_write(DP->TxData,9);
					cout << "OK " << i+1 <<endl;
					Sleep(5);					
				}

				DP->TxData[0] = ID_ReadENCcount+CMD_SET*number1;
				DP->TxData[1] = 0;
				DP->TxData[2] = 0;
				DP->TxData[3] = 0;
				DP->TxData[4] = 0;
				DP->TxData[5] = 0;
				DP->TxData[6] = 0;
				DP->TxData[7] = 0;
				DP->TxData[8] = 0; 
				gpPortLH->_write(DP->TxData,9);
				buffer[0]=0;
				buffer[1]=0;
				buffer[2]=0;
				buffer[3]=0;
  				Sleep(500);
				if(gpPortLH->read(buffer, 5,100,NULL))
				{
					if (int(buffer[0])%16 == IDR_LastCMD_Succ)
					{
						cout<<"Testing Aaxis " << int(buffer[0])/16 << " Completed, " << "Total Count : "<<int(buffer[1])+int(buffer[2])*256+int(buffer[3])*256*256 << " Returned "<<endl;
					}
					else
					{
							cout << "Return " << int(buffer[0]) << " wrong response" << endl;
					}
				}

				else
				{
					cout << "axis " << number1 << " no response" << endl;
				}
			}

			else
			{
				cout << "Hand does not exist." << endl;
			}


				break;

						//DP->TxData[0] = ID_SendReadENC+CMD_SET*number1;
						//DP->TxData[1] = 0;
						//DP->TxData[2] = 0;
						//DP->TxData[3] = 0;
						//DP->TxData[4] = 0;
						//DP->TxData[5] = 0;
						//DP->TxData[6] = 0;
						//DP->TxData[7] = 0;
						//DP->TxData[8] = 0; 



						//DP->TxData[0] = ID_SetENC+CMD_SET*number1;
						//DP->TxData[1] = 0;
						//DP->TxData[2] = 0;
						//DP->TxData[3] = 0;
						//DP->TxData[4] = 0;
						//DP->TxData[5] = 0;
						//DP->TxData[6] = 0;
						//DP->TxData[7] = 0;
						//DP->TxData[8] = 0; 





				//for(int i = 0; i<100; i++)
				//{
				//		//if(number1>=1 && number1<=6)
				//		//{
				//		//	gpPortLH->_write(DP->TxData,9);   
				//		//}
				//		//if(number1>=7 && number1<=12)
				//		//{
				//	gpPortRH->_write(DP->TxData,9);   
				//		//}
				//	Sleep(10);
				//	//buffer[0] = 0;

				//	//if(number1>=1 && number1<=6)
				//	//{
				//	//	if(gpPortLH->read(buffer, 5,100,NULL))
				//	//	{
				//	//		cout << "OK" <<endl;
				//	//		testcount++;

				//	//	}
				//	//	else
				//	//	{
				//	//		cout << "Fail" <<endl;
				//	//		cout << "Total times : "<< testcount<<endl;
				//	//		break;
				//	//	}

				//	//}
				//	//if(number1>=7 && number1<=12)
				//	//{
				//		if(gpPortRH->read(buffer, 5,100,NULL))
				//		{
				//			cout << "OK" << testcount<<endl;
				//			testcount++;

				//		}
				//		else
				//		{
				//			cout << "Fail" <<endl;
				//			cout << "Total times : "<< testcount<<endl;
				//			testcount=0;
				//			break;
				//		}  
				//	//}
				//	//else
				//	//{
				//	//	cout<<"The Axis does not exist."<<endl;
				//	//}	
				//}

			//testcount=0;
			
			//break;

		default: 
		
			cout << "Without Choosing Sending Mode" << endl; 
		}

		if(mode!=4&&mode!=5)
		cout<<"System Stand by:"<<endl;
	/*}
	else
	{
			cout<<"please push initialize"<<endl;
	}*/
}


void gTestHand()
{
	//cout<<"Left Hand Axis Test:"<<endl;
	//for(int AxisNumber = 7; AxisNumber < 13 ; AxisNumber++)
	//{
	//	DP->State[2]=ID_InitMech;
	//	gpPortLH->_write(DP->State,5);
	//	Sleep(100);
	//	DP->TxData[0] = AxisNumber;
	//	gpPortLH->_write(DP->TxData,9);
	//	buffer[2] = 0;
	//	Sleep(1000);
	//	//回傳值，確認是否有傳到C30
	//	if(gpPortLH->read(buffer, 5,500,NULL))
	//	{
	//		if (int(buffer[2])%16 == IDR_LastCMD_Succ)
	//		{
	//			cout<<"Homing Aaxis " << int(buffer[2])/16 << " Completed, " << int(buffer[2]) << " Returned "<<endl;
	//		}
	//		else
	//		{
	//			cout << "Return " << int(buffer[2]) << " wrong response" << endl;
	//		}
	//	}
	//	else
	//	{
	//		cout << "axis " << AxisNumber << " no response" << endl;
	//	}
	//	buffer[2] = 0;
	//	Sleep(100); 
	//}
	//
	//cout<<"Right Hand Axis Test:"<<endl;
	//for(int AxisNumber = 7; AxisNumber < 13 ; AxisNumber++)
	//{		
	//	DP->State[2]=ID_InitMech;
	//	gpPortRH->_write(DP->State,5);
	//	Sleep(100);
	//	DP->TxData[0] = AxisNumber;
	//	gpPortRH->_write(DP->TxData,9);
	//	buffer[2] = 0;
	//	Sleep(1000);
	//	//回傳值，確認是否有傳到C30
	//	if(gpPortRH->read(buffer, 5,500,NULL))
	//	{
	//		if (int(buffer[2])%16 == IDR_LastCMD_Succ)
	//		{
	//			cout<<"Homing Aaxis " << int(buffer[2])/16 << " Completed, " << int(buffer[2]) << " Returned "<<endl;
	//		}
	//		else
	//		{
	//			cout << "Return " << int(buffer[2]) << " wrong response" << endl;
	//		}
	//	}
	//	else
	//	{
	//		cout << "axis " << AxisNumber << " no response" << endl;
	//	}
	//	buffer[2] = 0;
	//	Sleep(100); 
	//}
	int nn=5;
	for(int whand = 0 ;whand<2;whand++) 
	{
		for(int number1 = 7; number1<14; number1++)
		{


			if(whand==0)
			{
				for(int i = 0; i<nn; i++)
				{
					DP->State[2]=ID_SetOneAxis;
					gpPortRH->_write(DP->State,5);

					DP->TxData[0] = ID_SetENC+CMD_SET*number1;
					DP->TxData[1] = 20;
					DP->TxData[2] = 0;
					DP->TxData[3] = 0;
					DP->TxData[4] = 0;
					DP->TxData[5] = 0;
					DP->TxData[6] = 0;
					DP->TxData[7] = 0;
					DP->TxData[8] = 0; 
					gpPortRH->_write(DP->TxData,9);
					//cout << "OK " << i+1 <<endl;
					Sleep(5);					
				}

				DP->TxData[0] = ID_ReadENCcount+CMD_SET*number1;
				DP->TxData[1] = 0;
				DP->TxData[2] = 0;
				DP->TxData[3] = 0;
				DP->TxData[4] = 0;
				DP->TxData[5] = 0;
				DP->TxData[6] = 0;
				DP->TxData[7] = 0;
				DP->TxData[8] = 0; 
				gpPortRH->_write(DP->TxData,9);
				buffer[0]=0;
				buffer[1]=0;
				buffer[2]=0;
				buffer[3]=0;
				Sleep(100);
				if(gpPortRH->read(buffer, 5,100,NULL))
				{
					if (int(buffer[0])%16 == IDR_LastCMD_Succ)
					{
						cout<<"Testing Aaxis " << int(buffer[0])/16 << " Completed, " << "Total Count : "<<int(buffer[1])+int(buffer[2])*256+int(buffer[3])*256*256 << " Returned "<<endl;
					}
					else
					{
						cout << "Return " << int(buffer[0]) << " wrong response" << endl;
					}
				}

				else
				{
					cout << "axis " << number1-1 << " no response" << endl;
				}
			}

			else if (whand==1)
			{
				for(int i = 0; i<nn; i++)
				{
					DP->State[2]=ID_SetOneAxis;
					gpPortLH->_write(DP->State,5);

					DP->TxData[0] = ID_SetENC+CMD_SET*number1;
					DP->TxData[1] = 20;
					DP->TxData[2] = 0;
					DP->TxData[3] = 0;
					DP->TxData[4] = 0;
					DP->TxData[5] = 0;
					DP->TxData[6] = 0;
					DP->TxData[7] = 0;
					DP->TxData[8] = 0; 
					gpPortLH->_write(DP->TxData,9);
					//cout << "OK " << i+1 <<endl;
					Sleep(5);					
				}

				DP->TxData[0] = ID_ReadENCcount+CMD_SET*number1;
				DP->TxData[1] = 0;
				DP->TxData[2] = 0;
				DP->TxData[3] = 0;
				DP->TxData[4] = 0;
				DP->TxData[5] = 0;
				DP->TxData[6] = 0;
				DP->TxData[7] = 0;
				DP->TxData[8] = 0; 
				gpPortLH->_write(DP->TxData,9);
				buffer[0]=0;
				buffer[1]=0;
				buffer[2]=0;
				buffer[3]=0;
				Sleep(100);
				if(gpPortLH->read(buffer, 5,100,NULL))
				{
					if (int(buffer[0])%16 == IDR_LastCMD_Succ)
					{
						cout<<"Testing Aaxis " << int(buffer[0])/16 << " Completed, " << "Total Count : "<<int(buffer[1])+int(buffer[2])*256+int(buffer[3])*256*256 << " Returned "<<endl;
					}
					else
					{
						cout << "Return " << int(buffer[0]) << " wrong response" << endl;
					}
				}

				else
				{
					cout << "axis " << number1-1 << " no response" << endl;
				}
			}

			else
			{
				cout << "Hand does not exist." << endl;
			}
			Sleep(1000);
		}
	}
}

void posturecalibration(float *IMUpitch, float *IMUroll, int count)
{
	//input 為弧度 rad

//#if TwinCAT_Mode	
//	
//	//double error;
//	double deadzone = 5;
//	double deadzoneCoP = 10;
//	
//	double Pgain = 0.1;
//	
//	long Ang = long(74896.4438101*3.14159265/180/500);	//固定角度?
//	long nError;
//	long nError2;
//
//	///正負號表示軸方向不同
// 	
//	
//	
//	CaliPitchHipR = Pgain*(0-IMUpitch[count]); 
//	CaliPitchHipL = -Pgain*(0-IMUpitch[count]); 
//	
//	CaliRollHipR  =  -Pgain*(0-IMUroll[count])/2;
//	CaliRollHipL  =  Pgain*(0-IMUroll[count])/2;
//	
//
//
//
//	//// PitchAngL
//	//		TCAT->pTLL1msOut->LL_O_TarEnc_05=CaliPitchAngL;
//
//	//// RollAngL	
//	//		TCAT->pTLL1msOut->LL_O_TarEnc_06=CaliRollAngL;
//
//	// PitchHipL
//			TCAT->pTLL1msOut->LL_O_TarEnc_03=CaliPitchHipL;
//
//	// RollHipL
//			TCAT->pTLL1msOut->LL_O_TarEnc_02=CaliRollHipL;	
//			
//	//// PitchAngR
//	//		TCAT->pTRL1msOut->RL_O_TarEnc_05=CaliPitchAngR;
//
//	//// RollAngR
//	//		TCAT->pTRL1msOut->RL_O_TarEnc_06=CaliRollAngR;
//
//	// RollHipR
//			TCAT->pTRL1msOut->RL_O_TarEnc_02=CaliRollHipR;
//
//	// PitchHipR
//			TCAT->pTRL1msOut->RL_O_TarEnc_03=CaliPitchHipR;
//
//	if ( ((nError = TCatIoInputUpdate( TASK_RLEG_PORTNUMBER )) == 0 )&& ((nError2 = TCatIoOutputUpdate( TASK_RLEG_PORTNUMBER )) == 0 )&&
//				 (( nError = TCatIoInputUpdate( TASK_LLEG_PORTNUMBER )) == 0 )&& ( (nError2 = TCatIoOutputUpdate( TASK_LLEG_PORTNUMBER )) == 0 )) 
//	{ 
//					// do your calculation and logic 
//					//pT1msOut->AnalogOut = pT1msIn->AnalogIn; 
//					// start the I/O update and field bus cycle 
//										
//					TCatIoOutputUpdate( TASK_RLEG_PORTNUMBER ); 
//					TCatIoOutputUpdate( TASK_LLEG_PORTNUMBER ); 
//					//CC.open("CC.txt",ios::out| ios::app);
//					//CC<<pTRL1msIn->RL_I_Enc_04<<"\t";
//					//CC.close();
//	}
//
//					
//#endif

}
