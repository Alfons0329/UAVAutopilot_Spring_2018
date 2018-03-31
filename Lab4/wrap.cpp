// ConsoleApplication2.cpp : �w�q�D���x���ε{�����i�J�I�C
//
#include<iostream>
#include <opencv2/opencv.hpp>
// #include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;
void warp(Mat input, Mat &output, Mat h);
void onMouse(int event, int x, int y, int flags, void* param) {
	vector<Point2f>* ptr = (vector<Point2f>*) param;
	if (event == CV_EVENT_LBUTTONDOWN) {
		ptr->push_back(Point2f(x, y));
	}
}

int main() {
	VideoCapture cap(0);
	if (!cap.isOpened()) {
		return -1;
	}
	Mat image;
	image = imread("osaka.jpg");

	Mat frame;
	cap >> frame;

	vector<Point2f> cap_corner;
	vector<Point2f> img_corner;
	// add the corner of frame into cap_corner
	cap_corner.push_back(Point2f(0.0, 0.0));
	cap_corner.push_back(Point2f(frame.cols-1, 0.0));
	cap_corner.push_back(Point2f(frame.cols-1, frame.rows-1));
	cap_corner.push_back(Point2f(0.0, frame.rows-1));
	/*
	img_corner.push_back(Point2f(0.0, 0.0));
	img_corner.push_back(Point2f(200.0, 0.0));
	img_corner.push_back(Point2f(200.0, 200.0));
	img_corner.push_back(Point2f(0.0, 200.0));
	*/
	namedWindow("img", CV_WINDOW_AUTOSIZE);
	imshow("img", image);
	setMouseCallback("img", onMouse, &img_corner);

	while (img_corner.size()<4) {

		if (waitKey(1) == 27) break;
	}

	Mat img_out = image.clone();
	Mat img_temp = image.clone();
	Mat h = findHomography(cap_corner, img_corner);
	h.convertTo(h, 5);
	// call your warping function
	warp(frame, img_temp, h);
	imshow("warped", img_temp);
	Point poly[4];
	for (int i = 0; i < img_corner.size(); i++) {
		poly[i] = img_corner[i];
	}

	while (1) {
		cap >> frame;
		// call your warping function
		warp(frame, img_temp, h);
		fillConvexPoly(img_out, poly, 4, Scalar(0), CV_AA);
		img_out = img_out + img_temp;
		imshow("img", img_out);
		if (waitKey(1) == 27) break;
	}
	return 0;
}
void warp(Mat input, Mat &output, Mat h)
{
	output = (output.size(), CV_8UC3, Scalar(0,0,0));
	for (int i = 0; i < input.rows; i++)
	{
		for (int j = 0; j < input.cols; j++)
		{
			Mat p(3,1, CV_32F);
			p.at<float>(0, 0) = (float)j;
			p.at<float>(1, 0) = (float)i;
			p.at<float>(2, 0) = 1.0;
			Mat t = h*p;
			//cout << t << endl;
			t.at<float>(0, 0) /= t.at<float>(2, 0);
			t.at<float>(1, 0) /= t.at<float>(2, 0);
			t.at<float>(2, 0) /= t.at<float>(2, 0);

			if (t.at<float>(0, 0) < 0 || t.at<float>(0, 0) >= output.cols || t.at<float>(1, 0) < 0 || t.at<float>(1, 0) >= output.rows)
			{
			}
			else
			{
				output.at<Vec3b>(t.at<float>(1, 0), t.at<float>(0, 0)) = input.at<Vec3b>(i,j);
			}
		}
	}
}
