// ConsoleApplication2.cpp : �w�q�D���x���ε{�����i�J�I�C
//

#include <opencv2/opencv.hpp>

using namespace cv;
void  mask(Mat& input, Mat& output);
int main()
{
	namedWindow("demo", 0);
	Mat input_img = imread("mj.tif",0);
	Mat mask_img = input_img.clone();

	mask(input_img, mask_img);

	imshow("origin", input_img);
	imshow("mask", mask_img);
	waitKey(0);

	imwrite("output.jpg", mask_img);

	return 0;
}
void  mask(Mat& input, Mat& output) {

	// write down your code here
	for (int i = 0; i<output.rows; i++) {
		for (int j = 0; j<output.cols; j++) {
			int temp = (-4)*(input.at<uchar>(i, j));
			if (i - 1 >= 0)
				temp += input.at<uchar>(i - 1, j);
			if (j - 1 >= 0)
				temp += input.at<uchar>(i, j - 1);
			if (i + 1 < output.rows)
				temp += input.at<uchar>(i + 1, j);
			if (j + 1 < output.cols)
				temp += input.at<uchar>(i, j + 1);
			if (temp > 255)
				output.at<uchar>(i, j) = 255;
			else if (temp < 0)
				output.at<uchar>(i, j) = 0;
			else
				output.at<uchar>(i, j) = temp;
		}
	}

}
