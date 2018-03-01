#include <iostream>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>

using namespace std;
using namespace cv;

void 	bilinear_interpolation(Mat& input, Mat& output, float scalingFactor);

int 	main(int argc, char** argv) {
	
	Mat inputImg = imread(argv[1]);
	float scalingFactor = atof(argv[2]);
	
	int scaledWidth = round(1. * inputImg.cols * scalingFactor);
	int scaledHeight = round(1. * inputImg.rows * scalingFactor);
	
	Mat outputImg1 = Mat(scaledHeight, scaledWidth, inputImg.type());
	Mat outputImg2 = outputImg1.clone(); // for opencv build-in function
	
	// resize the input image by your bilinear_interpolation funcion
	bilinear_interpolation(inputImg, outputImg1, scalingFactor);
	// resize the input image by opencv
	resize(inputImg, outputImg2, Size(scaledWidth, scaledHeight), 0, 0, INTER_LINEAR);	//resize image

	imshow("My Interpolation", outputImg1);
	imshow("Opencv build-in function", outputImg2);
	waitKey(0);

	imwrite("output.jpg", outputImg1);
	imwrite("output2.jpg", outputImg2);

	return 0;
}


void 	bilinear_interpolation(Mat& input, Mat& output, float scalingFactor) {
	
	// write down your code here
	for(int i=0;i<output.rows;i++){
		for(int j=0;j<output.cols;j++){
			if((int)(i/scalingFactor)*scalingFactor == i && (int)(j/scalingFactor)*scalingFactor == j){
				output.at<Vec3b>(i,j)[0] = input.at<Vec3b>(i/scalingFactor,j/scalingFactor)[0];
				output.at<Vec3b>(i,j)[1] = input.at<Vec3b>(i/scalingFactor,j/scalingFactor)[1];
				output.at<Vec3b>(i,j)[2] = input.at<Vec3b>(i/scalingFactor,j/scalingFactor)[2];
			}
			else{
				output.at<Vec3b>(i,j)[0] = output.at<Vec3b>(i/scalingFactor,j/scalingFactor)[0] * ((int)i/scalingFactor+1-(float)i/scalingFactor) * ((int)j/scalingFactor+1-(float)j/scalingFactor);
				output.at<Vec3b>(i,j)[0]+= output.at<Vec3b>(i/scalingFactor+1,j/scalingFactor)[0] * ((float)i/scalingFactor-(int)i/scalingFactor) * ((int)j/scalingFactor+1-(float)j/scalingFactor);
				output.at<Vec3b>(i,j)[0]+= output.at<Vec3b>(i/scalingFactor,j/scalingFactor+1)[0] * ((int)i/scalingFactor+1-(float)i/scalingFactor) * ((float)j/scalingFactor-(int)j/scalingFactor);
				output.at<Vec3b>(i,j)[0]+= output.at<Vec3b>(i/scalingFactor+1,j/scalingFactor+1)[0] * ((float)i/scalingFactor-(int)i/scalingFactor) * ((float)j/scalingFactor-(int)j/scalingFactor);
			
				output.at<Vec3b>(i,j)[1] = output.at<Vec3b>(i/scalingFactor,j/scalingFactor)[1] * ((int)i/scalingFactor+1-(float)i/scalingFactor) * ((int)j/scalingFactor+1-(float)j/scalingFactor);
				output.at<Vec3b>(i,j)[1]+= output.at<Vec3b>(i/scalingFactor+1,j/scalingFactor)[1] * ((float)i/scalingFactor-(int)i/scalingFactor) * ((int)j/scalingFactor+1-(float)j/scalingFactor);
				output.at<Vec3b>(i,j)[1]+= output.at<Vec3b>(i/scalingFactor,j/scalingFactor+1)[1] * ((int)i/scalingFactor+1-(float)i/scalingFactor) * ((float)j/scalingFactor-(int)j/scalingFactor);
				output.at<Vec3b>(i,j)[1]+= output.at<Vec3b>(i/scalingFactor+1,j/scalingFactor+1)[1] * ((float)i/scalingFactor-(int)i/scalingFactor) * ((float)j/scalingFactor-(int)j/scalingFactor);

				output.at<Vec3b>(i,j)[2] = output.at<Vec3b>(i/scalingFactor,j/scalingFactor)[2] * ((int)i/scalingFactor+1-(float)i/scalingFactor) * ((int)j/scalingFactor+1-(float)j/scalingFactor);
				output.at<Vec3b>(i,j)[2]+= output.at<Vec3b>(i/scalingFactor+1,j/scalingFactor)[2] * ((float)i/scalingFactor-(int)i/scalingFactor) * ((int)j/scalingFactor+1-(float)j/scalingFactor);
				output.at<Vec3b>(i,j)[2]+= output.at<Vec3b>(i/scalingFactor,j/scalingFactor+1)[2] * ((int)i/scalingFactor+1-(float)i/scalingFactor) * ((float)j/scalingFactor-(int)j/scalingFactor);
				output.at<Vec3b>(i,j)[2]+= output.at<Vec3b>(i/scalingFactor+1,j/scalingFactor+1)[2] * ((float)i/scalingFactor-(int)i/scalingFactor) * ((float)j/scalingFactor-(int)j/scalingFactor);
			}
		}	
		
	}
}
