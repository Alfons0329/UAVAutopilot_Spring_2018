#include <opencv2/aruco.hpp>
#include "calibration.h"
#include <bits/stdc++.h>
#include <iostream>
#include <fstream>
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

Mat cameraMatrix;
Mat distCoeffs;
double markerLength = 7.05;

int main(void){

	cv::Ptr<aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

	std::vector<int> ids;
	std::vector<std::vector<cv::Point2f> > corners;

	string name = "name";
	my_camera_calibration(cameraMatrix, distCoeffs, name);

	cv::VideoCapture inputVideo;
	inputVideo.open(0);
	int framePerSecond = 20;
	Mat frame;
	getchar();
	while(inputVideo.grab()) {

		//cap >> frame;
		//imshow("webcam", frame);


		cv::Mat image, imageCopy;
	    inputVideo.retrieve(image);
	    image.copyTo(imageCopy);
	    std::vector<int> ids;
	    std::vector<std::vector<cv::Point2f> > corners;
	    cv::aruco::detectMarkers(image, dictionary, corners, ids);
	    // if at least one marker detected
	    if (ids.size() > 0) {
	        cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
	        vector<cv::Vec3d> rvecs, tvecs;
			aruco::estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);
	        // draw axis for each marker
	        cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs, tvecs, 10);
	        cout << tvecs[0] << endl;
	    }
	    cv::imshow("out", imageCopy);

		/*

		cv::aruco::detectMarkers(frame, dictionary, corners, ids);
		cout << "asd\n";

		cv::aruco::drawDetectedMarkers(frame, corners, ids);

		vector<cv::Vec3d> rvecs, tvecs;
		aruco::estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);
		cout << "asd\n";
		cv::aruco::drawAxis(frame, cameraMatrix, distCoeffs, rvecs, tvecs, 0.1);
		cout << corners.size() << endl;
		cout << tvecs[0] << ", " << tvecs[1] << ", " << tvecs[2] << endl;
		cout << "asd\n";
		*/
		waitKey(100);
	}

	return 0;
}
