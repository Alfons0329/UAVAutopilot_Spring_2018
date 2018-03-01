#include <iostream>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>

using namespace std;
using namespace cv;

void bilinear_interpolation(Mat& input, Mat& output, float scalingFactor);

int main(int argc, char** argv)
{

	Mat inputImg = imread(argv[1]);
	float scalingFactor = atof(argv[2]);

	int scaledWidth = round(1. * inputImg.cols * scalingFactor);
	int scaledHeight = round(1. * inputImg.rows * scalingFactor);

	Mat outputImg1 = Mat(scaledHeight, scaledWidth, inputImg.type());
	Mat outputImg2; // for opencv build-in function

	// resize the input image by your bilinear_interpolation funcion
	bilinear_interpolation(inputImg, outputImg1, scalingFactor);
	// resize the input image by opencv
	// outputImg2 = ....

	imshow("My Interpolation", outputImg1);
	imshow("Opencv build-in function", outputImg2);
	waitKey(0);

	imwrite("output.jpg", outputImg1);

	return 0;
}


void bilinear_interpolation(Mat& input, Mat& output, float scalingFactor)
{

	// write down your code here

}
