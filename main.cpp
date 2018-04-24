#include "kinect.h"
#include <math.h>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
using namespace cv;
using namespace std;

int main()
{
	void Rotate(ColorSpacePoint chand, ColorSpacePoint celbow, DepthSpacePoint dhand, DepthSpacePoint delbow, Mat&i_rgb, Mat& new_rgb, Mat&i_depth, Mat& new_depth, float z, Mat& new1_rgb, Mat& new1_depth);
	void RGB_YCrCb(Mat& RGB);
	void Segmentation(Mat&color_YCrCb, Mat& new_depth);

	// 三个图片格式
	Mat i_rgb(1080, 1920, CV_8UC4);      //注意：这里必须为4通道的图，Kinect的数据只能以Bgra格式传出
	Mat i_depth(424, 512, CV_8UC1);
	Mat new_rgb(1080, 1920, CV_8UC4);      
	Mat new_depth(424, 512, CV_8UC1);
	Mat new1_rgb;
	Mat new1_depth;
	//Mat mask(96, 96, CV_8UC1);
	unsigned short *depthData = new unsigned short[424 * 512];
	char cc;
	FILE *fp;
	cc = fopen_s(&fp, "E:\\Vs_C++\\local_depth_skin\\first_project\\data\\ehpointdata.txt", "r");
	for (int k = 0; k < 8;k ++)
	{
		string img_path = "E:\\Vs_C++\\local_depth_skin\\first_project\\data\\";
		string str = to_string(k);
		i_rgb = cv::imread(img_path + "rgb_" + str + ".png", -1);
		i_depth = cv::imread(img_path + "dep_" + str + ".png", -1);
		if (!i_rgb.data || !i_depth.data)
			std::cout << "Could not open or find picture in this folder\n " << img_path << std::endl;

		ColorSpacePoint chand, celbow; 
		DepthSpacePoint dhand, delbow;
		//ifstream* in;
		float z;
		//ifstream* in;
		//int i, j;
		char buffer[256];
		if (cc == 0)
		{
			fgets(buffer, 256, fp);
			sscanf_s(buffer, "%f%f%f%f%f", &celbow.X, &celbow.Y, &chand.X, &chand.Y, &z);
			fgets(buffer, 256, fp);
			sscanf_s(buffer, "%f%f%f%f", &delbow.X, &delbow.Y, &dhand.X, &dhand.Y);
			//cout << chand.X << '\t' << celbow.Y << '\t' << dhand.X << '\t' << delbow.Y << endl;
			//cout << z << endl;
		}
		else return 0;
		Rotate(chand, celbow, dhand, delbow, i_rgb, new_rgb, i_depth, new_depth, z, new1_rgb, new1_depth);
		RGB_YCrCb(new1_rgb);
		//cvtColor(new1_rgb, new1_rgb, CV_BGR2YCrCb);
		//Segmentation(new1_rgb, new1_depth);
		//hand_depth(new1_depth, mask);
		//string str = to_string(savecount++);
		//imwrite("final_depth_" + str + ".png", new1_depth);
		// 显示
		//imwrite("mask.jpg", mask);
		imwrite("depariginal_" + str + ".png", new1_depth);
		imwrite("YUV_" + str + ".png", new1_rgb);
		imwrite("dep_" + str + ".png", new1_depth);
	}
	fclose(fp);
	// 关闭窗口，设备
	cv::destroyAllWindows();
	std::system("pause");
	return 0;
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
void Color_Cut(ColorSpacePoint chand, Mat&new_rgb, Mat&new1_rgb, float z)
{
	Mat img;
	IplImage* src = &IplImage(new_rgb);
	IplImage* dst;
	float pSize = 2.87 * 87 / z;
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

void Depth_Cut(DepthSpacePoint dhand, Mat&new_depth, Mat&new1_depth, float z)
{
	Mat img;
	float pSize = 87 / z;
	IplImage* src = &IplImage(new_depth);
	IplImage* dst;
	int downedge, leftedge;
	leftedge = (dhand.X - pSize / 2) < 0 ? 0 : (dhand.X - pSize / 2);
	//rightedge = (dhand.X + pSize / 2) >= 512 ? 511 : (dhand.X + pSize / 2);
	//upedge = (dhand.Y + pSize / 2) >= 424 ? 423 : (dhand.Y + pSize / 2);
	downedge = (dhand.Y - pSize / 2) < 0 ? 0 : (dhand.Y - pSize / 2);
	cvSetImageROI(src, cvRect(leftedge, downedge, pSize, pSize));
	dst = cvCreateImage(cvSize(pSize, pSize), IPL_DEPTH_8U, src->nChannels);
	cvCopy(src, dst, 0);
	cvResetImageROI(src);
	img = cvarrToMat(dst);
	resize(img, new1_depth, Size(96, 96), 0, 0, INTER_LINEAR);
}

void Rotate(ColorSpacePoint chand, ColorSpacePoint celbow, DepthSpacePoint dhand, DepthSpacePoint delbow, Mat&i_rgb, Mat& new_rgb, Mat&i_depth, Mat& new_depth, float z, Mat& new1_rgb, Mat& new1_depth)
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
			rotateangle = - atan(fabs(delbow.X - dhand.X) / fabs(delbow.Y - dhand.Y));
		else
			rotateangle = atan(fabs(delbow.X - dhand.X) / fabs(delbow.Y - dhand.Y));
		//color
		Rotatecenter.x = chand.X;
		Rotatecenter.y = chand.Y;
		Mat Mcolor = getRotationMatrix2D(Rotatecenter, rotateangle * 180 / 3.141592654, 1);
		warpAffine(i_rgb, new_rgb, Mcolor, cvSize(i_rgb.cols, i_rgb.rows));
		Color_Cut(chand, new_rgb, new1_rgb, z);
		//depth
		Rotatecenter.x = dhand.X;
		Rotatecenter.y = dhand.Y;
		Mat Mdepth = getRotationMatrix2D(Rotatecenter, rotateangle * 180 / 3.141592654, 1);
		warpAffine(i_depth, new_depth, Mdepth, cvSize(i_depth.cols, i_depth.rows));
		Depth_Cut(dhand, new_depth, new1_depth, z);
}