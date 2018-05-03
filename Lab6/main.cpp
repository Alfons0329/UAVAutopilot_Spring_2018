#include "ardrone/ardrone.h"
#include "pid.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <bits/stdc++.h>

#define PAT_ROWS   (6)                  // Rows of pattern
#define PAT_COLS   (9)                 // Columns of pattern
#define CHESS_SIZE (22.0)              // Size of a pattern [mm]

using namespace std;
using namespace cv;
// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------
//------------------Marker dection board constants--------------------------//
const float markerLength = 12.3f;
const float required_distance = 80.0f;
//------------------Marker dection board constants end here-----------------//
//------------------velocity amplification and disamplification-------------//
const int vx_amp = 1000;
const int vy_amp = 1000;
const int vz_amp = 1000;
const int vr_amp = 1000;
//------------------velocity amplification and disamplification ends here---------//
// AR.Drone class
ARDrone ardrone;
int load_camera_param(string filename_in, Mat& cameraMatrix, Mat& distCoeffs)
{
	FileStorage fs(filename_in, FileStorage::READ);
	if(!fs.isOpened())
	{
		return 0;
	}
	fs["intrinsic"] >> cameraMatrix;
	fs["distortion"] >> distCoeffs;
	if((int)(cameraMatrix.at<double>(0 ,0)) == 0) //fake read file
	{
		return 0;
	}
	fs.release();
	return 1;
}

int main(int argc, char *argv[])
{
	// Initialize
	//-----------------------camera parameter loading for ardrone----------------------------------//
	Mat image;
	Mat cameraMatrix, distCoeffs;
	if(load_camera_param(argv[1], cameraMatrix, distCoeffs))
	{
		//nop;
	}
	else
	{
		cout << "Loading param failed now manaully loading the parameters"<<endl;

	}
	if (!ardrone.open())
	{
		cout << "Failed to initialize." << endl;
		return -1;
	}

	// Battery
	cout << "Battery = " << ardrone.getBatteryPercentage() << "[%]" << endl;

	// Instructions
	cout << "***************************************" << endl;
	cout << "*       CV Drone sample program       *" << endl;
	cout << "*           - How to play -           *" << endl;
	cout << "***************************************" << endl;
	cout << "*                                     *" << endl;
	cout << "* - Controls -                        *" << endl;
	cout << "*    'Space' -- Takeoff/Landing       *" << endl;
	cout << "*    'Up'    -- Move forward          *" << endl;
	cout << "*    'Down'  -- Move backward         *" << endl;
	cout << "*    'Left'  -- Turn left             *" << endl;
	cout << "*    'Right' -- Turn right            *" << endl;
	cout << "*    'Q'     -- Move upward           *" << endl;
	cout << "*    'A'     -- Move downward         *" << endl;
	cout << "*                                     *" << endl;
	cout << "* - Others -                          *" << endl;
	cout << "*    'C'     -- Change camera         *" << endl;
	cout << "*    'Esc'   -- Exit                  *" << endl;
	cout << "*                                     *" << endl;
	cout << "***************************************" << endl;
	//rvec 02 對應vr

	//-----------------------drone aruco checking and detection data structure init------------------------------//
	cv::Ptr<aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    vector<int> ids; //aruco markers
	vector<vector<Point2f> > aruco_corners; //aruco corners
	//-----------------------done data structure init------------------------------------------//
	//
	PIDManager myPID("pid.yaml");
	getchar();// stop a while for changing paper lololol
	while (1)
	{
		// Key input
		int key = waitKey(33);
		if (key == 0x1b) break;

		// Get an image
		image = ardrone.getImage();

		// Take off / Landing
		if (key == ' ')
		{
			if (ardrone.onGround()) ardrone.takeoff();
			else                    ardrone.landing();
		}

		// Move
		double vx = 0.0, vy = 0.0, vz = 0.0, vr = 0.0;
		if (key == 'i' || key == CV_VK_UP)    vx = 1.0;
		if (key == 'k' || key == CV_VK_DOWN)  vx = -1.0;
		if (key == 'u' || key == CV_VK_LEFT)  vr = 1.0;
		if (key == 'o' || key == CV_VK_RIGHT) vr = -1.0;
		if (key == 'j') vy = 1.0;
		if (key == 'l') vy = -1.0;
		if (key == 'q') vz = 1.0;
		if (key == 'a') vz = -1.0;


		// Change camera
		static int mode = 0;
		if (key == 'c') ardrone.setCamera(++mode % 4);

		if (key == 255)
		{
			//---------------------------------Main part of market detection----------------------------------------------//

			aruco::detectMarkers(image, dictionary, aruco_corners, ids);
			// if at least one marker detected
			if (ids.size() > 0)
			{
				aruco::drawDetectedMarkers(image, aruco_corners, ids);
				vector<Vec3d> rvecs, tvecs;
				aruco::estimatePoseSingleMarkers(aruco_corners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);
				//draw axis for each marker
				//the more the last coefficient , the more obviously the axis will be drawn
				for (int i = 0; i < ids.size(); i++)
				{
					aruco::drawAxis(image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 10); //if read
					cout << "Detected ArUco markers " << ids.size() << "x,y,z = " << tvecs[0] << endl; //x,y,z in the space
				}
				if (!ardrone.onGround() && tvecs.size()) //tvecs has something to do
				{
					// implement your autopilot algorithm here
					// only need to modify vx, vy, vz, vr
					//error is {tvec[0][2] - 80.0f (required_distance), tvec[0][0], tvec[0][1], rvec[0][2]}
					//drone x(forward and back) is corresponding to tvec[0][2]
					//drone y(left and right) is corresponding to tvec[0][0]
					//drone z(up and down) is corresponding to tvec[0][1]
					/*
					流程如下：
					
					*/
					Mat error (4, 1, CV_64F);
					Mat output(4, 1, CV_64F);
					//setting distance delta
					error.at<double>(0, 0)=tvecs[0][2] - required_distance;
					error.at<double>(1, 0)=tvecs[0][0] - 0;
					error.at<double>(2, 0)=tvecs[0][1] - 0;
					if(rvecs[0][0]<0)
					{
						error.at<double>(3, 0) = - ( rvecs[0][2] - 0 );
					}
                	else
					{
						error.at<double>(3, 0) = ( rvecs[0][2] - 0 );
					}
					//setting distance delta done
					//now doing pid
					myPID.getCommand(error, output);
					//doing the [P, I , D] by ourselves to implement the smoothing, how to check if it is smoothed?
					//by printing out the v_something to see if it is stable or not
					//pid done
					cout << " Calibrated vx "<< output.at<double>(0, 0) << " Calibrated vr "<< output.ar<double>(3, 0) <<endl;
				}
			}
			imshow("Aruco Market Axis", image);
		}
		ardrone.move3D(vx, vy, vz, vr);
		// Display the image
		imshow("camera", image);


	}
	// See you
	ardrone.close();

	return 0;
}
