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
const int vx_amp = 10;
const int vy_amp = 1000;
const int vz_amp = 1000;
const int vr_amp = 20;
//------------------velocity amplification and disamplification ends here---------//
//------------------error bounds--------------------------------------------------//
const float vx_error_bound = 10.0f;
const float vr_error_bound = 0.5f;
//------------------error bounds end here----------------------------------------//
//------------------least required speed-----------------------------------------//
const float vx_lower_bound = 0.0f;
const float vr_lower_bound = 0.05f;
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
	/*
	if(load_camera_param(argv[1], cameraMatrix, distCoeffs))
	{
		//nop;
		cout << "Loading param SUCCESSFULL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
		cout << "cameraMatrix " << cameraMatrix <<endl;
		cout << "distCoeffs " << distCoeffs <<endl;
	}
	else
	{
		cout << "Loading param failed now manaully loading the parameters"<<endl;
	}
	*/
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
					//cout << "Detected ArUco markers " << ids.size() << "x,y,z = " << tvecs[0] << endl; //x,y,z in the space
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
					vx:
					測一下實際的飛行，然後看看是否有需要調整（放在上面的const 把vx印出來，超過80cm(required_distance)就要加速反之亦然，調整一下放大or縮小係數，）
					vr:
					測一下實際的飛行，于哲說會發現rvec其實很小（轉向的時候） 基本上誤差就可以那樣忽略掉，然後因為它都是1e-3左右，因此放大一千倍較為合適
					test rvecs[0][2] is the r (放在上面的const 調整一下放大or縮小係數，）
					*/
					Mat error (4, 1, CV_64F);
					Mat output(4, 1, CV_64F);
					//setting distance delta
					//---------------------------------setting tvec00 starts here--------------------------------------//
					error.at<double>(0, 0)=tvecs[0][2] - required_distance;
					//---------------------------------setting tvec00 ends here--------------------------------------//
					error.at<double>(1, 0)=tvecs[0][0] - 0;
					error.at<double>(2, 0)=tvecs[0][1] - 0;
					/*朱蝶的code 基本上就是看pid的輸出值在進行倍率調整，太大就除太小就乘，但因為可能他們的跟我們的不一樣 所以就當參考，我直接用係數去改試試看
					time的話我再問問她是幹麻
					if(time>0){
                    time--;
                    vx=tmp;
                	}else{
                	if(_output.at<double>(0, 0)<=5 && _output.at<double>(0,0)>(-5)){
                    	vx=0;
                	}else if(_output.at<double>(0, 0)<=20 && _output.at<double>(0,0)>(-20)){
                    	vx=_output.at<double>(0, 0)/170;
                	}else if(_output.at<double>(0, 0)<=70 && _output.at<double>(0,0)>(-70)){
                    	vx=_output.at<double>(0, 0)/200;
                	}else if(_output.at<double>(0, 0)<=200 && _output.at<double>(0,0)>(-200)){
                    	vx=_output.at<double>(0, 0)/200;
                	}else{
                    	time=1000;
                    	tmp=_output.at<double>(0, 0)/200;
                	}
					*/
					//---------------------------------setting rvec02 starts here--------------------------------------//
					if(rvecs[0][2] < vr_error_bound || rvecs[0][2]> -vr_error_bound) //誤差很小 就忽略誤差
        			{
            			if((rvecs[0][0]>= 0 && rvecs[0][2] >= 0 )|| (rvecs[0][0]<= 0 && rvecs[0][2] <= 0))//markerzai右邊同號
						{
							error.at<double>(3,0) = abs(rvecs[0][2]);
						}
            			else if ((rvecs[0][0] < 0 && rvecs[0][2] > 0) || (rvecs[0][0] > 0 && rvecs[0][2] < 0))//marker zai左邊異號
						{
							error.at<double>(3,0) = - abs(rvecs[0][2]);
						}
        			}
					//---------------------------------setting rvec02 ends here--------------------------------------//
					//now doing pid
					//doing the [P, I , D] by ourselves to implement the smoothing, how to check if it is smoothed?
					myPID.getCommand(error, output);
					//pid done

        			//---------------------------------assign the vx and vr-------------------------------------------//
					//amp代表放大係數
        			if(abs(output.at<double>(3, 0)* vr_amp ) < vr_lower_bound) //too small to rotate no effect
					{
						vr = 0;
					}
        			else
					{
						vr = -output.at<double>(3, 0) * vr_amp;
					}
					if(abs(output.at<double>(0, 0)* vx_amp ) < vx_lower_bound) //too small
					{
						vx = 0;
					}
        			else
					{
						vx = output.at<double>(0, 0) * vx_amp;
					}
					
					//vx = output.at<double>(0, 0) * 0;
					//---------------------------------assign the vx and vr-------------------------------------------//
					//by printing out the v_something to see if it is stable or not
					//cout << "Error at vx: " << error.at<double>(0,0) << ", error at vr: " << error.at<double>(3,0) << endl;
					cout << "Unamplified Calibrated vx "<< output.at<double>(0, 0) << " Calibrated vr "<< output.at<double>(3, 0) <<endl;
				}
			}
			imshow("Aruco Marker Axis", image);
		}
		ardrone.move3D(vx, vy, vz, vr);
		// Display the image
		imshow("camera", image);
	}
	// See you
	ardrone.close();

	return 0;
}
