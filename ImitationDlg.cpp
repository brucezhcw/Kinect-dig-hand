
// ImitationDlg.cpp : 实现文件
//

#include "stdafx.h"
#include "Imitation.h"
#include "ImitationDlg.h"
#include "afxdialogex.h"
#include <opencv.hpp>
#include <Kinect.h>
#include <thread>
#include <time.h>
#include "Motor.h"
#include "myVector.h"
#include "client.h"
#include <io.h> 
#include <fcntl.h>
#include <conio.h>
#include <vector>
#include <fstream>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

using namespace std;
using namespace cv;

#pragma region Global Variables
const double pi = 3.1415926;
double theta[7];
int controlTheta[7];
float pSize;
bool Run = false;
bool track = false;
Motor *myMotor;
int startTime;
bool enableMotor = false;
bool openedMotor = false;
int outputPeriod = 4;
int filtersize = 100;

IKinectSensor *myKinectSensor;
ICoordinateMapper *myCoordinateMapper;
//Color Frame Variables
IColorFrameSource *myColorFrameSource;
IColorFrameReader *myColorFrameReader;
IColorFrame *myColorFrame;
Mat img_rgb(1080, 1920, CV_8UC4);
Mat new_rgb(1080, 1920, CV_8UC4);
Mat new1_rgb(96, 96, CV_8UC4);
//Depth Frame Variables
IDepthFrameSource *myDepthFrameSource;
IDepthFrameReader *myDepthFrameReader;
IDepthFrame *myDepthFrame;
Mat Depth_data(424, 512, CV_16UC1);
Mat img_depth(424, 512, CV_8UC1);
Mat new_depth(424, 512, CV_16UC1);
Mat new1_depth(96, 96, CV_16UC1);
//Body Frame Variables
IBodyFrameSource *myBodyFrameSource;
IBodyFrameReader *myBodyFrameReader;
IBodyFrame *myBodyFrame;
CameraSpacePoint TIMpoints[3];
CameraSpacePoint Fingerpoints[5];
CameraSpacePoint points[JointType_Count];
ColorSpacePoint Cpoints[JointType_Count];
DepthSpacePoint Dpoints[JointType_Count];
//Point Cdraw[JointType_Count];
Point Ddraw[JointType_Count];
thread *ColorThread;
//thread *DepthThread;
//thread *BodyThread;
//vector<double*> thetaRobot;
//vector<double*> thetaProcess;
vector<float>X_vector;
vector<float>Y_vector;
vector<float>Z_vector;
vector<Vector3D*> elbowRaw;
vector<Vector3D*> wristRaw;
Vector3D elbowTotal(0);
Vector3D wristTotal(0);

#pragma endregion
#pragma region Global Function
// Define Realease Function
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}
//Inverse Kinematics
void InverseKinematics(double *theta, Vector3D shoulder, Vector3D elbow, Vector3D wrist, Vector3D handnormalvector)
{
	Vector3D Vec1 = wrist - elbow;
	norm(Vec1);
	norm(elbow);

	theta[1] = acos(elbow.z);
	if (elbow.x > 0)
		theta[1] = -theta[1];
	if (theta[1] > 11.0 / 18 * pi)
		theta[1] = 11.0 / 18 * pi;
	if (theta[1] < - 11.0 / 18 * pi)
		theta[1] = - 11.0 / 18 * pi;

	theta[0] = acos(elbow.x / sin(theta[1]));
	//theta[0] = atan(elbow.y / elbow.x);

	//theta[1] = atan( - elbow.y / elbow.z / sin(theta[0]));

	theta[3] = pi / 2 - angle(elbow, Vec1);

	Vector3D nVec = cross(elbow, Vec1);
	norm(nVec);
	theta[2] = asin(nVec.z / sin(theta[1]));
	if (theta[2] > 17.0 / 18 * pi)
		theta[2] = 17.0 / 18 * pi;
	if (theta[2] < -17.0 / 18 * pi)
		theta[2] = -17.0 / 18 * pi;

}
// Delay Function
void Delay(int ms)
{
	int endtime = ms + clock();
	while (clock() < endtime);
}
// Function to Draw lines
inline void DrawHelper(Mat &img, Point *&DrawPoints, const int *Index, int length)
{
	Point point2draw[10] = {};
	for (int i = 0; i < length; ++i)
		point2draw[i] = DrawPoints[Index[i]];
	for (int i = 0; i < length - 1; ++i)
		line(img, point2draw[i], point2draw[i + 1], Scalar(0, 255, 0), 2);
}
// Function to Draw Skeleton
inline void DrawSkeleton(Mat &img, Point DrawPoints[25], int r = 20, const int length = JointType_Count)  // inline here may improve the speed
{
	//Index of the Skeleton data
	const int LeftArm[] = { 20, 4, 5, 6, 7, 21 };
	const int RightArm[] = { 20, 8, 9, 10, 11, 23 };
	const int LeftThumb[] = { 6, 22 };
	const int RightThumb[] = { 10, 24 };
	const int LeftLeg[] = { 0, 12, 13, 14, 15 };
	const int RightLeg[] = { 0, 16, 17, 18, 19 };
	const int Spinal[] = { 3, 2, 20, 1, 0 };
	//--------------------------------------------
	for (int i = 0; i < length; ++i)
		circle(img, DrawPoints[i], r, Scalar(0, 255, 0), -1);
	DrawHelper(img, DrawPoints, LeftArm, 6);
	DrawHelper(img, DrawPoints, RightArm, 6);
	DrawHelper(img, DrawPoints, LeftThumb, 2);
	DrawHelper(img, DrawPoints, RightThumb, 2);
	DrawHelper(img, DrawPoints, LeftLeg, 5);
	DrawHelper(img, DrawPoints, RightLeg, 5);
	DrawHelper(img, DrawPoints, Spinal, 5);
}
// Process TIM state
void processTIM(CameraSpacePoint right_S)
{
	Vector3D c, v1, v2, i, j, k;
	Vector3D right_shoulder(right_S);
	Vector3D T(TIMpoints[0]);
	Vector3D I(TIMpoints[1]);
	Vector3D M(TIMpoints[2]);

	right_shoulder = rightTrans(right_shoulder);
	T = rightTrans(T) - right_shoulder;
	I = rightTrans(I) - right_shoulder;
	M = rightTrans(M) - right_shoulder;
	//right_shoulder = right_shoulder - right_shoulder;

	c = (T + I + M) / 3;
	v1 = I - M;
	v2 = T - M;
	i = cross(v1, v2) / length(cross(v1, v2));
	j = (c - T) / length(c - T);
	k = cross(i, j);

	theta[4] = angle(i, Vector3D(1, 0, 0));
	theta[5] = angle(j, Vector3D(0, 1, 0));
	theta[6] = angle(k, Vector3D(0, 0, 1));
}
//useing least square method to find Fitting plane
void cvFitPlane(const CvMat* points, float* plane)
{
	// Estimate geometric centroid.  
	int nrows = points->rows;
	int ncols = points->cols;
	int type = points->type;
	CvMat* centroid = cvCreateMat(1, ncols, type);
	cvSet(centroid, cvScalar(0));
	for (int c = 0; c<ncols; c++){
		for (int r = 0; r < nrows; r++)
		{
			centroid->data.fl[c] += points->data.fl[ncols*r + c];
		}
		centroid->data.fl[c] /= nrows;
	}
	// Subtract geometric centroid from each point.  
	CvMat* points2 = cvCreateMat(nrows, ncols, type);
	for (int r = 0; r<nrows; r++)
		for (int c = 0; c<ncols; c++)
			points2->data.fl[ncols*r + c] = points->data.fl[ncols*r + c] - centroid->data.fl[c];
	// Evaluate SVD of covariance matrix.  
	CvMat* A = cvCreateMat(ncols, ncols, type);
	CvMat* W = cvCreateMat(ncols, ncols, type);
	CvMat* V = cvCreateMat(ncols, ncols, type);
	cvGEMM(points2, points, 1, NULL, 0, A, CV_GEMM_A_T);
	cvSVD(A, W, NULL, V, CV_SVD_V_T);
	// Assign plane coefficients by singular vector corresponding to smallest singular value.  
	plane[ncols] = 0;
	for (int c = 0; c<ncols; c++){
		plane[c] = V->data.fl[ncols*(ncols - 1) + c];
		plane[ncols] += plane[c] * centroid->data.fl[c];
	}
	// Release allocated resources.  
	cvReleaseMat(&centroid);
	cvReleaseMat(&points2);
	cvReleaseMat(&A);
	cvReleaseMat(&W);
	cvReleaseMat(&V);
}

// Process finger state
void Inversehand(CameraSpacePoint right_S)
{
	//Vector3D thumb	= Fingerpoints[0];
	//Vector3D index	= Fingerpoints[1];
	//Vector3D middle	= Fingerpoints[2];
	//Vector3D ring		= Fingerpoints[3];
	//Vector3D little	= Fingerpoints[4];
	
}
// Process Arm state
void processArmState(CameraSpacePoint points[25])
{
	//Vector3D left_shoulder(points[4]);
	//Vector3D left_elbow(points[5]);
	//Vector3D left_wrist(points[6]);
	//Vector3D left_hand(points[7]);

	Vector3D right_shoulder(points[8]);
	Vector3D right_elbow(points[9]);
	Vector3D right_wrist(points[10]);
	Vector3D right_hand(points[11]);

	//left_shoulder = leftTrans(left_shoulder);
	//left_elbow = leftTrans(left_elbow) - left_shoulder;
	//left_wrist = leftTrans(left_wrist) - left_shoulder;
	//left_hand = leftTrans(left_hand) - left_shoulder;
	//left_shoulder = left_shoulder - left_shoulder;

	right_shoulder = rightTrans(right_shoulder);
	right_elbow = rightTrans(right_elbow) - right_shoulder;
	right_wrist = rightTrans(right_wrist) - right_shoulder;
	right_hand = rightTrans(right_hand) - right_shoulder;
	right_shoulder = right_shoulder - right_shoulder;

	int cnt = elbowRaw.size();
	if (cnt < filtersize)
	{
		elbowTotal = elbowTotal + right_elbow;
		wristTotal = wristTotal + right_wrist;
	}
	else
	{
		elbowTotal = elbowTotal + right_elbow - *elbowRaw[cnt - filtersize];
		wristTotal = wristTotal + right_wrist - *wristRaw[cnt - filtersize];
	}
	Vector3D* tem = new Vector3D(right_elbow);
	elbowRaw.push_back(tem);
	tem = new Vector3D(right_wrist);
	wristRaw.push_back(tem);

	for (int i = 0; i < 5; i++)
	{
		X_vector.push_back(Fingerpoints[i].X);
		Y_vector.push_back(Fingerpoints[i].Y);
		Z_vector.push_back(Fingerpoints[i].Z);
	}
	CvMat*points_mat = cvCreateMat(X_vector.size(), 3, CV_32FC1);//定义用来存储需要拟合点的矩阵   
	for (int i = 0; i < X_vector.size(); ++i)
	{
		points_mat->data.fl[i * 3 + 0] = X_vector[i];//矩阵的值进行初始化   X的坐标值  
		points_mat->data.fl[i * 3 + 1] = Y_vector[i];//  Y的坐标值  
		points_mat->data.fl[i * 3 + 2] = Z_vector[i];//  Z的坐标值</span>  

	}
	float plane12[4] = { 0 };//定义用来储存平面参数的数组   
	cvFitPlane(points_mat, plane12);//调用方程
	Vector3D handnormalvector(plane12[0], plane12[1], plane12[2]);
	InverseKinematics(theta, right_shoulder, elbowTotal / filtersize, wristTotal / filtersize, handnormalvector);

	//_cprintf("Inverse Kinematics Result:theta1: %.2f, theta2: %.2f, theta3: %.2f, theta4: %.2f\n", theta[0], theta[1], theta[2], theta[3]);
}

void Segmentation(Mat&color_YCrCb, Mat& new_depth)
{
	IplImage* pinput = &IplImage(color_YCrCb);
	IplImage* poutput = &IplImage(new_depth);
	//深拷贝只要再加一次复制数据：  IplImage *input = cvCloneImage(pBinary);
	int y, cb, cr, cb0, cr0, index;
	int i, j;
	uchar *indata = NULL;
	uchar *outdata = NULL;
	uchar *indatakm = NULL;
	int height = pinput->height;
	int width = pinput->width;
	switch (width * 2 % 4)
	{
	case 0:index = width * 2;
	case 1:index = width * 2 - 1;
	case 2:index = width * 2 - 2;
	case 3:index = width * 2 - 3;
	}
	uchar *data0 = (uchar *)(pinput->imageData + pinput->widthStep*(height / 2));
	//y0 = data0[index];
	cb0 = data0[index + 1];
	cr0 = data0[index + 2];

	for (j = 0; j < height; j++)
	{
		indata = (uchar *)(pinput->imageData + pinput->widthStep*j);
		outdata = (uchar *)(poutput->imageData + poutput->widthStep*j);
		for (i = 0; i < width; i++)
		{
			y = indata[i * 4];
			cr = indata[i * 4 + 1];
			cb = indata[i * 4 + 2];
			//if ((cr > 129) && (cr < 150) && (cb > 105) && (cb < 128))
			if (abs(cr - cr0) <= 125 && abs(cb - cb0) <= 35)
			{
				outdata[i] = outdata[i];
			}
			else
				outdata[i] = 0;
		}
	}
	// can there be simplified ?
	//Mat ImgTemp;
	//ImgTemp = cvarrToMat(poutput);
	//mask = cvarrToMat(poutput);
}


void RGB_YCrCb(Mat& RGB)
{
	IplImage* pdata = &IplImage(RGB);
	int r, g, b;
	int y, cb, cr;
	int i, j;
	uchar *data = NULL;
	int height = pdata->height;
	int width = pdata->width;

	for (j = 0; j < height; j++)
	{
		data = (uchar *)(pdata->imageData + pdata->widthStep*j);
		for (i = 0; i < width; i++)
		{
			//RGB空间向YCrCb空间的转换  
			b = data[i * 4];
			g = data[i * 4 + 1];
			r = data[i * 4 + 2];

			//y = r * 0.299 + g * 0.587 + b * 0.114;
			//cb = -r * 0.1687 - g * 0.3313 + b * 0.5 + 128;
			//cr = r * 0.5 - g * 0.4187 - b * 0.0813 + 128;

			y = (b * 1868 + g * 9617 + r * 4899 + 8192) / 16384.;
			cr = ((b - y) * 9241 + 8192) / 16384. + 128;
			cb = ((r - y) * 11682 + 8192) / 16384. + 128;


			y = y > 255 ? 255 : y;
			cb = cb > 255 ? 255 : cb;
			cr = cr > 255 ? 255 : cr;

			y = y < 0 ? 0 : y;
			cb = cb < 0 ? 0 : cb;
			cr = cr < 0 ? 0 : cr;

			data[i * 4] = y;
			data[i * 4 + 1] = cb;
			data[i * 4 + 2] = cr;
		}
	}
	// Mat ImgTemp;
	// ImgTemp = cvarrToMat(pdata);
	//RGB = cvarrToMat(pdata);
}
void Color_Cut(ColorSpacePoint chand, Mat&new_rgb, Mat&new1_rgb)
{
	Mat img;
	IplImage* src = &IplImage(new_rgb);
	IplImage* dst;
	float pSize = 2.87 * 87 / points[JointType_HandRight].Z;
	int downedge, leftedge;
	leftedge = (chand.X - pSize / 2) < 0 ? 0 : (chand.X - pSize / 2);
	//rightedge = (chand.X + pSize / 2) >= 1920 ? 1919 : (chand.X + pSize / 2);
	//upedge = (chand.Y + pSize / 2) >= 1080 ? 1079 : (chand.Y + pSize / 2);
	downedge = (chand.Y - pSize / 2) < 0 ? 0 : (chand.Y - pSize / 2);

	cvSetImageROI(src, cvRect(leftedge, downedge, pSize, pSize));
	dst = cvCreateImage(cvSize(pSize, pSize), IPL_DEPTH_8U, src->nChannels);
	cvCopy(src, dst, 0);
	cvResetImageROI(src);
	img = cvarrToMat(dst);
	//img = new_rgb(Range(leftedge, rightedge), Range(downedge, upedge));
	resize(img, new1_rgb, Size(96, 96), 0, 0, INTER_LINEAR);
}

void Depth_Cut(DepthSpacePoint dhand, Mat&new_depth, Mat&new1_depth)
{
	Mat img;
	pSize = 87 / points[JointType_HandRight].Z;
	IplImage* src = &IplImage(new_depth);
	IplImage* dst;
	int downedge, leftedge;
	leftedge = (dhand.X - pSize / 2) < 0 ? 0 : (dhand.X - pSize / 2);
	//rightedge = (dhand.X + pSize / 2) >= 512 ? 511 : (dhand.X + pSize / 2);
	//upedge = (dhand.Y + pSize / 2) >= 424 ? 423 : (dhand.Y + pSize / 2);
	downedge = (dhand.Y - pSize / 2) < 0 ? 0 : (dhand.Y - pSize / 2);
	cvSetImageROI(src, cvRect(leftedge, downedge, pSize, pSize));
	dst = cvCreateImage(cvSize(pSize, pSize), IPL_DEPTH_16U, src->nChannels);
	cvCopy(src, dst, 0);
	cvResetImageROI(src);
	img = cvarrToMat(dst);
	resize(img, new1_depth, Size(96, 96), 0, 0, INTER_LINEAR);
}

void Rotate(ColorSpacePoint chand, ColorSpacePoint celbow, DepthSpacePoint dhand, DepthSpacePoint delbow, Mat&i_rgb, Mat& new_rgb, Mat&i_depth, Mat& new_depth, Mat& new1_rgb, Mat& new1_depth)
{
	CvPoint2D32f Rotatecenter;
	//cv::circle(new_rgb, cv::Point(chand.X, chand.Y), 5, cv::Scalar(0, 128, 0), 5, CV_AA);
	//cv::circle(new_rgb, cv::Point(celbow.X, celbow.Y), 5, cv::Scalar(0, 128, 0), 5, CV_AA);
	//line(new_rgb, (cv::Point)(chand.X, chand.Y), (cv::Point)(celbow.X, celbow.Y), cvScalar(0, 0, 255), 2);
	//cv::circle(new_depth, cv::Point(dhand.X, dhand.Y), 5, cv::Scalar(0, 128, 0), 5, CV_AA);
	//cv::circle(new_depth, cv::Point(delbow.X, delbow.Y), 5, cv::Scalar(0, 128, 0), 5, CV_AA);
	//line(new_depth, (cv::Point)(dhand.X, dhand.Y), (cv::Point)(delbow.X, delbow.Y), cvScalar(0, 0, 255), 2);
	float rotateangle;
	//angle
	if ((delbow.X - dhand.X)*(delbow.Y - dhand.Y) >= 0)
		rotateangle = -atan(fabs(delbow.X - dhand.X) / fabs(delbow.Y - dhand.Y));
	else
		rotateangle = atan(fabs(delbow.X - dhand.X) / fabs(delbow.Y - dhand.Y));
	//color
	Rotatecenter.x = chand.X;
	Rotatecenter.y = chand.Y;
	Mat Mcolor = getRotationMatrix2D(Rotatecenter, rotateangle * 180 / 3.141592654, 1);
	warpAffine(i_rgb, new_rgb, Mcolor, cvSize(i_rgb.cols, i_rgb.rows));
	Color_Cut(chand, new_rgb, new1_rgb);
	//depth
	Rotatecenter.x = dhand.X;
	Rotatecenter.y = dhand.Y;
	Mat Mdepth = getRotationMatrix2D(Rotatecenter, rotateangle * 180 / 3.141592654, 1);
	warpAffine(i_depth, new_depth, Mdepth, cvSize(i_depth.cols, i_depth.rows));
	Depth_Cut(dhand, new_depth, new1_depth);
}
/*
void load_python()
{
	Py_Initialize(); // 初始化，这是必须的，用来初始化python所需的环境
	if (!Py_IsInitialized())
		return;
	// 导入模块
	PyObject* pModule = NULL;
	PyRun_SimpleString("import sys");
	PyRun_SimpleString("sys.path.append('E:/Neural-Network/openpose-z/')");
	pModule = PyImport_ImportModule("dynamic_CNN");
	if (!pModule)
	{
		printf("Cant open python file !\n");
		return;
	}
	// 模块的字典列表
	PyObject* pDict = PyModule_GetDict(pModule);
	if (!pDict)
	{
		printf("Cant find dictionary.\n");
		return;
	}
	// 函数调用
	cout << "calling python program..." << endl;
	PyObject* pFunHi = PyDict_GetItemString(pDict, "CNN_caculater");
	PyObject* reward = PyObject_CallFunction(pFunHi, NULL, NULL);
	if (!reward)
	{
		printf("Cant find phi_class class./n");
		return;
	}
	Py_DECREF(reward);
	Py_DECREF(pFunHi);
	Py_DECREF(pModule);
	Py_Finalize(); // 与初始化对应
}
*/
void InitConsoleWindow()
{
	AllocConsole();
	HANDLE handle = GetStdHandle(STD_OUTPUT_HANDLE);
	int hCrt = _open_osfhandle((long)handle, _O_TEXT);
	FILE * hf = _fdopen(hCrt, "w");
	*stdout = *hf;
}
#pragma endregion
//-----------------------------------------
#pragma region Color
//A Clock Function to Find The fps of Color Frame
void countClockColor()
{
	static int time = 0;
	int nowtime = clock();
	fprintf(stdout, "The Color Frame Cost time %d ms\n", nowtime - time);
	time = nowtime;
}
//Color Frame Function
void ColorFrame()
{
	const UINT buffer_sizergb = 1920 * 1080 * 4;
	const UINT buffer_sizedep = 424 * 512;
	IBody* Bodys[BODY_COUNT] = { 0 };
	//namedWindow("ColorFrame", 0);
	while (track)
	{
		if (myColorFrameReader->AcquireLatestFrame(&myColorFrame) == S_OK &&
			myColorFrame->CopyConvertedFrameDataToArray(buffer_sizergb, reinterpret_cast<BYTE*>(img_rgb.data), ColorImageFormat::ColorImageFormat_Bgra) == S_OK)
		{
			//countClockColor();
			//DrawSkeleton(img_rgb, Cdraw);
			//imshow("ColorFrame", img_rgb);
			SafeRelease(myColorFrame);
			if (waitKey(1) == VK_ESCAPE)
				break;
		}
		if (myDepthFrameReader->AcquireLatestFrame(&myDepthFrame) == S_OK &&
			myDepthFrame->CopyFrameDataToArray(buffer_sizedep, reinterpret_cast<UINT16*>(Depth_data.data)) == S_OK)
		{
			//countClockDepth();
			for (int i = 0; i < 424; ++i)
				for (int j = 0; j < 512; ++j)
					img_depth.at<BYTE>(i, j) = Depth_data.at<UINT16>(i, j) % 256;
			Mat idepth(424, 512, CV_8UC1);
			idepth = img_depth.clone();
			DrawSkeleton(idepth, Ddraw, 5);
			imshow("skeleton", idepth);
			//imshow("DepthFrame", img_depth);
			SafeRelease(myDepthFrame);
			if (waitKey(1) == VK_ESCAPE)
				break;
		}
		if (myBodyFrameReader->AcquireLatestFrame(&myBodyFrame) == S_OK)
		{
			myBodyFrame->GetAndRefreshBodyData(_countof(Bodys), Bodys);
			for (int i = 0; i < BODY_COUNT; ++i)
			{
				IBody *pBody = Bodys[i];
				if (pBody)
				{
					BOOLEAN isTracked = false;
					if (pBody->get_IsTracked(&isTracked) == S_OK && isTracked)
					{
						Joint joints[JointType_Count];
						if (pBody->GetJoints(JointType_Count, joints) == S_OK)
						{
							for (int j = 0; j < JointType_Count; ++j)
							{
								points[j] = joints[j].Position;
								//fprintf(stdout, "%f\t%f\t%f\n", points[j].X, points[j].Y, points[j].Z); // cost too much time!
							}
						}
						myCoordinateMapper->MapCameraPointsToColorSpace(JointType_Count, points, JointType_Count, Cpoints);
						myCoordinateMapper->MapCameraPointsToDepthSpace(JointType_Count, points, JointType_Count, Dpoints);
						if (enableMotor)
						{
							int result[10];
							Mat Mask(96, 96, CV_16UC1);
							Rotate(Cpoints[JointType_HandRight], Cpoints[JointType_ElbowRight], Dpoints[JointType_HandRight], Dpoints[JointType_ElbowRight], img_rgb, new_rgb, Depth_data, new_depth, new1_rgb, new1_depth);
							client(new1_depth, Mask, result);
							//RGB_YCrCb(new1_rgb);
							//Segmentation(new1_rgb, new1_depth);
							//imshow("hand_depth", new1_depth);
							imshow("Mask", Mask);
							imshow("hand_rgb", new1_rgb);
							for (int i = 0; i < 10; ++i)
								result[i] *= (pSize / 48);
							DepthSpacePoint depthPoints[5]; 
							for (int m = 0; m < 5; m++)
							{
								depthPoints[m].X = result[2 * m];
								depthPoints[m].Y = result[2 * m + 1];
								depthPoints[m].X = depthPoints[m].X + Dpoints[JointType_HandRight].X - pSize / 2;
								depthPoints[m].Y = depthPoints[m].Y + Dpoints[JointType_HandRight].Y - pSize / 2;
								//if (depthPoints[m].X > 511) depthPoints[m].X = 511;
								//if (depthPoints[m].Y > 423) depthPoints[m].X = 423;
								myCoordinateMapper->MapDepthPointToCameraSpace(depthPoints[m], Depth_data.at<UINT16>(depthPoints[m].X, depthPoints[m].Y), &Fingerpoints[m]);
							}
							processArmState(points);
							//double* tem = new double[7];
							//for (auto i = 0; i < 7; ++i)
								//tem[i] = theta[i];
							//thetaProcess.push_back(tem);
							///double *rtheta = new double[7];
							//myMotor->getTheta(rtheta);
							//thetaRobot.push_back(rtheta);
							//int cnt = elbowRaw.size();
							//if (cnt % outputPeriod == 0 && cnt > filtersize)
							{
								//_cprintf("Inverse Kinematics Result:theta1: %.2f, theta2: %.2f, theta3: %.2f, theta4: %.2f\n", theta[0], theta[1], theta[2], theta[3]);
								//myMotor->Move(Left, theta);
							}

						}
						for (int j = 0; j < JointType_Count; ++j)
						{
							//Cdraw[j].x = int(Cpoints[j].X);
							//Cdraw[j].y = int(Cpoints[j].Y);
							Ddraw[j].x = int(Dpoints[j].X);
							Ddraw[j].y = int(Dpoints[j].Y);
						}
					}
				}
			}
			//countClockBody();
			SafeRelease(myBodyFrame);
		}

	}
	track = false;
	enableMotor = false;
}
#pragma endregion
// 用于应用程序“关于”菜单项的 CAboutDlg 对话框

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

// 实现
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CImitationDlg 对话框



CImitationDlg::CImitationDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_IMITATION_DIALOG, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CImitationDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CImitationDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDOK, &CImitationDlg::OnBnClickedOk)
	ON_BN_CLICKED(openKinect, &CImitationDlg::OnBnClickedopenkinect)
	ON_BN_CLICKED(openMotor, &CImitationDlg::OnBnClickedopenmotor)
	ON_BN_CLICKED(releaseKinect, &CImitationDlg::OnBnClickedreleasekinect)
	ON_BN_CLICKED(startTracking, &CImitationDlg::OnBnClickedstarttracking)
	ON_BN_CLICKED(closeMotor, &CImitationDlg::OnBnClickedclosemotor)
	ON_BN_CLICKED(stopTracking, &CImitationDlg::OnBnClickedstoptracking)
	ON_BN_CLICKED(startImitation, &CImitationDlg::OnBnClickedstartimitation)
	ON_BN_CLICKED(stopImitation, &CImitationDlg::OnBnClickedstopimitation)
	ON_BN_CLICKED(motorHome, &CImitationDlg::OnBnClickedmotorhome)
	ON_BN_CLICKED(Photo, &CImitationDlg::OnBnClickedPhoto)
END_MESSAGE_MAP()


// CImitationDlg 消息处理程序

BOOL CImitationDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();
	elbowTotal.x = 0;
	elbowTotal.y = 0;
	elbowTotal.z = 0;
	wristTotal.x = 0;
	wristTotal.y = 0;
	wristTotal.z = 0;
	InitConsoleWindow();
	_cprintf("Open console OK\n\n");
	// 将“关于...”菜单项添加到系统菜单中。

	// IDM_ABOUTBOX 必须在系统命令范围内。
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 设置此对话框的图标。  当应用程序主窗口不是对话框时，框架将自动
	//  执行此操作
	SetIcon(m_hIcon, TRUE);			// 设置大图标
	SetIcon(m_hIcon, FALSE);		// 设置小图标

	// TODO: 在此添加额外的初始化代码

	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}

void CImitationDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// 如果向对话框添加最小化按钮，则需要下面的代码
//  来绘制该图标。  对于使用文档/视图模型的 MFC 应用程序，
//  这将由框架自动完成。

void CImitationDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 用于绘制的设备上下文

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 使图标在工作区矩形中居中
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 绘制图标
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

//当用户拖动最小化窗口时系统调用此函数取得光标
//显示。
HCURSOR CImitationDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



void CImitationDlg::OnBnClickedOk()
{
	// TODO: 在此添加控件通知处理程序代码
	CDialogEx::OnOK();
}

void CImitationDlg::OnBnClickedopenkinect()
{
	// TODO: 在此添加控件通知处理程序代码
#pragma region Open Sensor
	HRESULT hr;
	// Open Kinect Sensor
	hr = GetDefaultKinectSensor(&myKinectSensor);
	if (SUCCEEDED(hr))
		hr = myKinectSensor->Open();
	if (FAILED(hr)) {
		_cprintf( "Open Kinect Sensor Error!\n");
		return;
	}
	_cprintf( "Kinect Sensor Ready!\n");
	// Get CoordinateMapper
	hr = myKinectSensor->get_CoordinateMapper(&myCoordinateMapper);
	if (FAILED(hr)) {
		_cprintf("Get Coordinate Mapper Error\n");
		return;
	}
	_cprintf("Coordinate Mapper Ready!\n");
	// Get Color Frame Source
	hr = myKinectSensor->get_ColorFrameSource(&myColorFrameSource);
	if (FAILED(hr)) {
		_cprintf("Get Color Frame Source Error\n");
		return;
	}
	_cprintf("Color Frame Source Ready!\n");
	// Open Color Frame Reader
	hr = myColorFrameSource->OpenReader(&myColorFrameReader);
	if (FAILED(hr)) {
		_cprintf("Open Color Reader Error\n");
		return;
	}
	_cprintf("Color Frame Reader Ready!\n");
	// Get Depth Frame Source
	hr = myKinectSensor->get_DepthFrameSource(&myDepthFrameSource);
	if (FAILED(hr)) {
		_cprintf("Get Depth Frame Source Error\n");
		return;
	}
	_cprintf("Depth Frame Source Ready!\n");
	// Open Depth Frame Reader
	hr = myDepthFrameSource->OpenReader(&myDepthFrameReader);
	if (FAILED(hr)) {
		_cprintf("Open Depth Reader Error\n");
		return;
	}
	_cprintf("Depth Frame Reader Ready!\n");
	// Get Body Frame Source
	hr = myKinectSensor->get_BodyFrameSource(&myBodyFrameSource);
	if (FAILED(hr)) {
		_cprintf("Get Body Frame Source Error\n");
		return;
	}
	_cprintf("Body Frame Source Ready!\n");
	// Open Body Frame Reader
	hr = myBodyFrameSource->OpenReader(&myBodyFrameReader);
	if (FAILED(hr)) {
		_cprintf("Open Body Frame Reader Error\n");
		return;
	}
	_cprintf("Body Frame Reader Ready!\n");
	BOOLEAN isAvailable = false;
	do {
		myKinectSensor->get_IsAvailable(&isAvailable);
	} while (!isAvailable);
	_cprintf("Kinect sensor is ready!\n\n");
#pragma endregion
}


void CImitationDlg::OnBnClickedopenmotor()
{
	// TODO: 在此添加控件通知处理程序代码
	myMotor = new Motor(5);
	myMotor->servoOpen();
	openedMotor = true;
	_cprintf("Motor Opend!\n\n");
}


void CImitationDlg::OnBnClickedreleasekinect()
{
	// TODO: 在此添加控件通知处理程序代码
	SafeRelease(myColorFrameReader);
	SafeRelease(myColorFrameSource);
	SafeRelease(myDepthFrameReader);
	SafeRelease(myDepthFrameSource);
	SafeRelease(myBodyFrameReader);
	SafeRelease(myBodyFrameSource);
	SafeRelease(myCoordinateMapper);
	myKinectSensor->Close();
	SafeRelease(myKinectSensor);
	_cprintf("Kinect source released!\n\n");
}


void CImitationDlg::OnBnClickedstarttracking()
{
	// TODO: 在此添加控件通知处理程序代码
	track = true;
	startTime = clock();
	ColorThread = new thread(ColorFrame);
	_cprintf("Is Tracking now....\n\n");
}


void CImitationDlg::OnBnClickedclosemotor()
{
	// TODO: 在此添加控件通知处理程序代码
	myMotor->servoClose();
	openedMotor = false;
	_cprintf("Motor is closed!\n\n");
}


void CImitationDlg::OnBnClickedstoptracking()
{
	// TODO: 在此添加控件通知处理程序代码
	track = false;
	_cprintf("Tracking is stopped!\n");
}


void CImitationDlg::OnBnClickedstartimitation()
{
	// TODO: 在此添加控件通知处理程序代码
	enableMotor = true;
	_cprintf("Motor is imitating.....\n\n");
}


void CImitationDlg::OnBnClickedstopimitation()
{
	// TODO: 在此添加控件通知处理程序代码
	enableMotor = false;
	_cprintf("Motor is not imitating any more\n\n");
}


void CImitationDlg::OnBnClickedmotorhome()
{
	// TODO: 在此添加控件通知处理程序代码
	_cprintf("Motor is going to Home position....");
	myMotor->Home(Left);
	_cprintf("Motor is Home now\n\n");
}


void CImitationDlg::OnBnClickedPhoto()
{
	// TODO: 在此添加控件通知处理程序代码
	_cprintf("Taking a photo....");
	imwrite("rgb.png", new1_rgb);
	imwrite("dep.png", new1_depth);
	_cprintf("Photo is OK\n\n");
}
