#include "calibration.h"
#include "opencv2/opencv.hpp"
#include <opencv2/aruco.hpp>
#include <bits/stdc++.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/imgproc.hpp>
/*
Some notes:
arguments in the waitkey is the system delay.
*/
using namespace std;
using namespace cv;

Mat cameraMatrix;
Mat distCoeffs;
double markerLength = 7.05; //in thecentimeter

int main(void){

	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250); //just this

	vector<int> ids;
	vector<vector<Point2f> > corners;

	my_camera_calibration(cameraMatrix, distCoeffs, "config.xml");

	VideoCapture cap(0);
	int framePerSecond = 20;
	Mat frame;
	getchar();
	while(cap.grab())
	{

		cap >> frame;
	    vector<int> ids;
	    vector<vector<Point2f> > corners;
	    aruco::detectMarkers(frame, dictionary, corners, ids);
	    // if at least one marker detected
	    if (ids.size() > 0)
		{
	        aruco::drawDetectedMarkers(frame, corners, ids);
	        vector<Vec3d> rvecs, tvecs;
			aruco::estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);
	        // draw axis for each marker
			//the more the last coefficient , the more obviously the axis will be drawn
			for(int i = 0;i < ids.size(); i++)
			{
				aruco::drawAxis(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 10); //if read
				cout<<"Detected ArUco markers "<<ids.size()<< "x,y,z = " << tvecs[0] << endl; //x,y,z in the space
			}
	    }
	    imshow("Aruco Market Axis", frame);
		waitKey(100);
	}
	return 0;
}
