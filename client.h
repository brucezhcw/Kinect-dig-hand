#include "Detector.h"
#include "floodfill.h"
#include <fstream>
using namespace std;
using namespace cv;
#pragma comment(lib, "ws2_32.lib")
void client(Mat &Original, Mat &Mask, int result[10])
{
	Detector handDetector = Detector();
	Mat Resized(48, 48, CV_16UC1);
	//ifstream f("0_.txt", ios::in);
	for (int i = 0; i < 96; ++i)
		for (int j = 0; j < 96; ++j)
		{
			//f >> Original.at<UINT16>(i, j);
			Mask.at<UINT16>(i, j) = 0;
		}
	floodFillDepth(Original, Mask, Original.at<UINT16>(48, 48), 48, 48, 10);
	for (int i = 0; i < 96; ++i)
		for (int j = 0; j < 96; ++j)
			Original.at<UINT16>(i, j) *= Mask.at<UINT16>(i, j);
	resize(Original, Resized, Size(48, 48));
	for (int i = 0; i < 10; ++i)
		handDetector.detect(Resized, result);
}