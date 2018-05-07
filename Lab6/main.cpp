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
const float markerLength = 9.4f;
const float required_distance = 100.0f;
//------------------Marker dection board constants end here-----------------//
//------------------velocity amplification and disamplification-------------//
const int vx_amp = 10;
const int vy_amp = 1000;
const int vz_amp = 1000;
const int vr_amp = 20;
//------------------velocity amplification and disamplification ends here---------//
//------------------error bounds--------------------------------------------------//
const float vx_error_bound = 1.0f;
const float vr_error_bound = 0.5f;
//------------------error bounds end here----------------------------------------//
//------------------least required speed-----------------------------------------//
const float vx_lower_bound = 0.01f;
const float vr_lower_bound = 0.3f;
//------------------least required speed-end here----------------------------------//
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
	Mat cameraMatrix(3,3,CV_64F), distCoeffs(1,5,CV_64F);
	cameraMatrix.at<double>(0,0) = 5.11897658982521e+02;
	cameraMatrix.at<double>(0,1) = 0.;
	cameraMatrix.at<double>(0,2) = 3.165825647196696e+02;
	cameraMatrix.at<double>(1,0) = 0.;
	cameraMatrix.at<double>(1,1) = 5.081336169143607e+02;
	cameraMatrix.at<double>(1,2) = 1.903117849450484e+02;
	cameraMatrix.at<double>(2,0) = 0.;
	cameraMatrix.at<double>(2,1) = 0.;
	cameraMatrix.at<double>(2,2) = 1.;
	distCoeffs.at<double>(0,0) = 8.7295277725232e-01;
	distCoeffs.at<double>(1,0) = -3.915795335526079e+01;
	distCoeffs.at<double>(2,0) = -1.872194795688528e-02;
	distCoeffs.at<double>(3,0) = 1.353528461095838e-03;
	distCoeffs.at<double>(4,0) = 3.000881704739565e+02;

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
	//-----------------------flying data structure init---------------------------------------//
	bool flags[5] = {false};
	int index[5] = {0};
	int state = 0;
	memset(flags, false, sizeof(flags));


	unsigned long long int counter = 0;
	struct timeval start, end;
	gettimeofday(&start, 0);
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
			//cout << "State now: " << state << endl;
			cout << "See marker: ";
			for (int j = 0; j < ids.size(); j++)
			{
				flags[ids[j]-1] = true;
				cout << " , " << ids[j] << endl;
				index[ids[j]-1] = j;
			}
			cout << "SEE MARKER END " << endl;
			vector<Vec3d> rvecs, tvecs;
			if (ids.size() > 0)
			{
				aruco::drawDetectedMarkers(image, aruco_corners, ids);
				aruco::estimatePoseSingleMarkers(aruco_corners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);
				//draw axis for each marker
				for (int i = 0; i < ids.size(); i++)
				{
					aruco::drawAxis(image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 10); //if read
					cout << "Detected ArUco markers " << ids.size() << "x,y,z = " << tvecs[0] << endl; //x,y,z in the space
				}
			}
			//---------------------------------------missions-----------------------------------------//
			//--------------------------------------可以飛越id1並且航向id2 單元測試-----------------------//
			/*if(state == 0)
			{
				vr = -0.3; //Self rotate till id1 is seen
				if(flags[0]) //看到一 進入狀態一
				{
					state = 1;
				}
			}
			else if(state == 1 && flags[0]) //看到一 朝它飛過去，目前都是看得到一的狀態
			{
				cout << "If block 2 "<<endl;
				vx = 0.5;
				vy = -( fabs(tvecs[0][0]) / fabs(tvecs[0][2]) ) * vx;
			}*/
			if(state == 0)
			{
				cout << " If block 0" <<endl;
				vr = -0.3; //Self rotate till id1 is seen
				if(flags[0]) //看到一 進入狀態一
				{
					state = 1;
				}
			}
			else if(state == 1 && counter <= 60)
			{
				/*if(counter == 10000000)
				{
					gettimeofday(&end, 0);
					ardrone.landing();
					int sec =abs(end.tv_sec - start.tv_sec);
					cout << "elapsed time "<<sec <<endl;
					exit(0);
				}*/
				cout << "If block 1 "<<endl;
				cout << "Counter " << counter++ << endl;
				vx = 0.5;
				vy = -0.25;//- (0.6 / 4.0f) * 1;
				if(counter >= 60)
				{
					state = 1;
					counter = 0xFFFFFFFF;
				}
			}
			else if(state == 1 && flags[0] == false) //使用上方區塊的 飛一飛id一會飛出視線，所以此時代表要往id二看了
			{
				//cout << "If block 3 "<<endl;
				vx = 0;
				vy = 0;
				vr = -0.3; //Self rotate till id2 is seen
				if(flags[1]) //看到二 進入狀態二 此時也能矯正方向
				{
					state = 2;
				}
			}
			else if(state == 2 && ids.size() > 0 && tvecs[index[1]][2] > required_distance) //持續朝id二飛行
			{
				cout << "If block 4 "<<endl;
				vx = 1;
				vy = 0;
				vr = 0;
			}
			else if(state == 2 && ids.size() > 0 && tvecs[index[1]][2] < required_distance) //快到了 停下
			{
				cout << "If block 5"<<endl;
				vx = 0;//停下來
				state = 3; //進入狀態三
			}
			//--------------------------------------可以飛越id1並且航向id2 單元測試-----------------------//
			//--------------------------------------可以轉向並且飛id2 3 4 單元測試-----------------------//
			/*else if(state == 3 && !flags[2]) //停在id二前面而且看不到id三 就自轉，向右轉會比較快
			{
				cout << "If block 6"<<endl;
				vr = -0.3; //Self rotate till id3 is seen
				if(flags[2]) //看到三 此時也能矯正方向
				{
					state = 4;
				}
			}
			else if(state == 4 && tvecs[index[2]][2] > required_distance)
			{
				vx = 1;
				cout << "If block 7"<<endl;
			}
			else if(state == 4 && tvecs[index[2]][2] < required_distance)
			{
				cout << "If block 8"<<endl;
				vx = 0;
				state = 5;
			}
			else if(state == 5 && !flags[3])
			{
				cout << "If block 9"<<endl;
				vr = -0.3; //Self rotate till id3 is seen
				if(flags[4]) //看到三 此時也能矯正方向
				{
					state = 6;
				}
			}
			//--------------------------------------可以轉向並且飛id2 3 4 單元測試-----------------------//
			//--------------------------------------降落 單元測試--------------------------------------//
			else if(state == 6 && tvecs[index[3]][2] > required_distance)
			{
				vx = 1;
			}
			else if(state == 6 && tvecs[index[3]][2] < required_distance + 20)
			{
				vx = 0;
				ardrone.landing();
			}*/
			//--------------------------------------降落 單元測試--------------------------------------//
			imshow("Aruco Marker Axis", image);
			memset(flags, false, sizeof(flags));

		}
		ardrone.move3D(vx, vy, vz, vr);
		// Display the image
		// imshow("camera", image);
	}
	// See you
	ardrone.close();
	return 0;
}
