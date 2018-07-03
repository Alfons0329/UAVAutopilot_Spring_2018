#ifndef _PID_HPP_
#define _PID_HPP_

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <opencv2/opencv.hpp>

#if defined(_WIN32) || defined(WIN32)
#define WINDOWS_IMPL
#include <windows.h>
#include <time.h>
#include <Mmsystem.h>
#pragma comment(lib, "Winmmlib")
#elif defined(__linux__) || defined(__APPLE__) || defined(__FreeBSD__) || defined(BSD)
#define UNIX_IMPL
#include <sys/time.h>
#endif

using namespace std;
using namespace cv;


class PIDManager {
public:
	PIDManager(Mat& _X, Mat& _Y, Mat& _Z, Mat& _R);
	PIDManager(string _filename);
	void reset();
	void importCoeffsFromFile(string _filename);
	void setCoeffs(Mat& _X, Mat& _Y, Mat& _Z, Mat& _R);
	void getCommand(Mat& _input, Mat& _output);

	double getCurrentTime();

	Mat mR;
	Mat mX;
	Mat mY;
	Mat mZ;
	bool mInit;

	long previous_time;
	Mat previous_error;
	Mat error_integral;

};


#endif
