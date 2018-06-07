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
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------
//------------------Marker dection board constants--------------------------//
const float markerLength = 9.4f;
const float required_distance = 90.0f;
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
	//----------------------face init--------------------------------------------------------------//
	CascadeClassifier face_cascade;
    face_cascade.load( "~opencv/opencv-3.2.0/data/haarcascade_frontalface_alt2.xml" );
    face_cascade.load( "~opencv/opencv-3.2.0/data/haarcascade_frontalface_alt.xml" );
	vector<Rect> faces;
	// Initialize
	//-----------------------camera parameter loading for ardrone----------------------------------//
	Mat image;
	Mat cameraMatrix(3,3,CV_64F), distCoeffs(1,5,CV_64F);
	cameraMatrix.at<double>(0,0) = 5.660229527130537e+02;
	cameraMatrix.at<double>(0,1) = 0.;
	cameraMatrix.at<double>(0,2) = 3.340570397444376e+02;
	cameraMatrix.at<double>(1,0) = 0.;
	cameraMatrix.at<double>(1,1) = 5.664897255182701e+02;
	cameraMatrix.at<double>(1,2) = 1.690819574644224e+02;
	cameraMatrix.at<double>(2,0) = 0.;
	cameraMatrix.at<double>(2,1) = 0.;
	cameraMatrix.at<double>(2,2) = 1.;
	distCoeffs.at<double>(0,0) = -5.060402671581601e-01;
	distCoeffs.at<double>(1,0) = 2.940530016423599e-02;
	distCoeffs.at<double>(2,0) = 1.774436061695057e-04;
	distCoeffs.at<double>(3,0) = -3.99927841858243e-03;
	distCoeffs.at<double>(4,0) = 1.26044448534097;

	Mat cameraMatrix2(3,3,CV_64F), distCoeffs2(1,5,CV_64F);
	cameraMatrix2.at<double>(0,0) = 6.9332526624253353e+02;
	cameraMatrix2.at<double>(0,1) = 0.;
	cameraMatrix2.at<double>(0,2) = 2.9322521936992302e+02;
	cameraMatrix2.at<double>(1,0) = 0.;
	cameraMatrix2.at<double>(1,1) = 7.0206548894894274e+02;
	cameraMatrix2.at<double>(1,2) = 2.1517298545590140e+02;
	cameraMatrix2.at<double>(2,0) = 0.;
	cameraMatrix2.at<double>(2,1) = 0.;
	cameraMatrix2.at<double>(2,2) = 1.;
	distCoeffs2.at<double>(0,0) = 1.5826748207215535e-01;
	distCoeffs2.at<double>(1,0) = 3.3513300662473128e-01;
	distCoeffs2.at<double>(2,0) = 2.7193440572564974e-02;
	distCoeffs2.at<double>(3,0) = -8.7146222209221226e-03;
	distCoeffs2.at<double>(4,0) = -4.1097722303957314e+00;

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
	int final_cnt = 0;
	memset(flags, false, sizeof(flags));


	unsigned long long int counter = 0;
	unsigned long long int landing_counter = 0;
	struct timeval start, end;
	gettimeofday(&start, 0);

	cout << "Start face detection "<< endl;
	while(1)
	{
		Mat image;
		image = ardrone.getImage();
		face_cascade.detectMultiScale( image, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );
		for( int i = 0; i < faces.size(); i++ )
	    {
	        Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
	        ellipse( image, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );
			printf("Find %d faces, face width %lf, face height %lf \n",faces.size() ,faces[i].width , faces[i].height);
	    }
	    // namedWindow("Detected Face", 0);
	    imshow( "Detected Face", image );
	}
	getchar();
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
		if (key == 'f') state = 5;


		// Change camera
		static int mode = 0;
		if (key == 'c') ardrone.setCamera(++mode % 4);

		if (key == 255)
		{
			//---------------------------------Main part of market detection----------------------------------------------//
			aruco::detectMarkers(image, dictionary, aruco_corners, ids);
			//cout << "State now: " << state << endl;
			if(ardrone.onGround())
			{
				state = 0;
			}
			if(ids.size())
			{
				cout << "See marker: ";
			}
			for (int j = 0; j < ids.size(); j++)
			{
				flags[ids[j]-1] = true;
				cout << " , " << ids[j] << endl;
				index[ids[j]-1] = j;
			}
			if(ids.size())
			{
				cout << "SEE MARKER END " << endl;
			}

			vector<Vec3d> rvecs, tvecs;
			if (ids.size() > 0)
			{
				if(mode == 0)
				{
					aruco::drawDetectedMarkers(image, aruco_corners, ids);
					aruco::estimatePoseSingleMarkers(aruco_corners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);
					//draw axis for each marker
					for (int i = 0; i < ids.size(); i++)
					{
						aruco::drawAxis(image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 10); //if read
						cout << "Detected ArUco markers " << ids.size() << "x,y,z = " << tvecs[0] << endl; //x,y,z in the space
						cout << "rvec 0 2 = " << rvecs[0][2] << endl; //x,y,z in the space
					}
				}
				else if(mode == 1)
				{
					aruco::drawDetectedMarkers(image, aruco_corners, ids);
					aruco::estimatePoseSingleMarkers(aruco_corners, markerLength, cameraMatrix2, distCoeffs2, rvecs, tvecs);
					//draw axis for each marker
					for (int i = 0; i < ids.size(); i++)
					{
						aruco::drawAxis(image, cameraMatrix2, distCoeffs2, rvecs[i], tvecs[i], 10); //if read
						cout << "Detected ArUco markers " << ids.size() << "x,y,z = " << tvecs[0] << endl; //x,y,z in the space
						cout << "rvec 0 2 = " << rvecs[0][2] << endl; //x,y,z in the space
					}
				}
			}
			//---------------------------------------missions-----------------------------------------//
			//--------------------------------------可以飛越id1並且航向id2 單元測試-----------------------//

			if(state == 0)
			{
				// cout << " If block 0" <<endl;
				vx = 0;
				vy = 0;
				vr = 0.12; //Self rotate till id1 is seen 正是逆時針，負是順時針
				if(flags[0] && rvecs[0][2] < 0.82 && rvecs[0][2] > -0.7) //看到一 進入狀態一
				{
					state = 1;
				}
			}
			else if(state == 1 && counter <= 140) //counter with trial and error
			{
				cout << "If block 1 "<<endl;
				cout << "Counter " << counter++ << endl;
				vx = 0.4;
				vy = 0.12;//- (0.8 / 4.0f) * 1;
				vr = 0;
				if(counter > 140) //counter with trial and error
				{
					state = 1;
					counter = 0xFFFFFFFF;
				}
			}
			else if(state == 1) //利用counter成功飛越id一之後，就往id二飛行，但在那之前要先找到id2 因此自轉一波
			{
				cout << "If block 3 "<<endl;
				vx = 0;
				vy = 0;
				vr = -0.13; //Self rotate till id2 is seen
				if(flags[1] && rvecs[0][2] < 0.82 && rvecs[0][2] > -0.8 ) //看到二 進入狀態二 此時也能矯正方向
				{
					state = 2;
				}
			}
			else if(state == 2 && ids.size() > 0 && tvecs[index[1]][2] > required_distance && flags[1] == true) //持續朝id二飛行
			{
				cout << "If block 4 "<<endl;
				vx = 0.2;
				vy = 0;
				vr = 0;
			}
			else if(state == 2 && ids.size() > 0 && tvecs[index[1]][2] < required_distance  && flags[1] == true) //快到了，停下，停在id二前面
			{
				cout << "If block 5"<<endl;
				vx = 0;//停下來
				vy = 0;
				vr = 0;
				state = 3; //進入狀態三
			}
			//--------------------------------------可以飛越id1並且航向id2 單元測試-----------------------//
			//--------------------------------------可以飛越id1並且航向id2 單元測試成功 2018/5/7 20:54-----------------------//
			/* 5/8 代辦事項
			1.確認vr 正負號與旋轉的方向，往正確的方向轉可以節省許多時間
			2.把剩下的單元測試跑完
			3.校正landing的下相機 用lab5已經寫好的code直接校正即可
			*/
			//--------------------------------------可以轉向並且飛id2 3 4 單元測試-----------------------//
			else if(state == 3) //停在id二前面而且看不到id三 就自轉，向右轉會比較快
			{
				cout << "If block 6"<<endl;
				vx = 0;
				vy = 0;
				vr = -0.13; //Self rotate till id3 is seen（如果要往另一個方向比較快，就加-號 到時候再看看）
				if(flags[2] && rvecs[0][2] < 0.4 && rvecs[0][2] > -0.4) //看到三 此時也能矯正方向
				{
					state = 4;
				}
			}
			else if(state == 4 && ids.size() > 0 && tvecs[index[2]][2] > required_distance + 20 && flags[2] == true) //持續朝id三飛行
			{
				cout << "If block 7"<<endl;
				vx = 0.25;
				vy = 0;
				vr = 0;
			}
			else if(state == 4 && ids.size() > 0 && tvecs[index[2]][2] < required_distance + 20 && flags[2] == true) //快到了，停下，停在id三前面
			{
				cout << "If block 8"<<endl;
				vx = 0;
				vy = 0;
				vr = 0;
				state = 5;
			}
			else if(state == 5 ) //停在id二前面而且看不到id四 就自轉，向右轉會比較快
			{
				cout << "If block 9"<<endl;
				vx = 0;
				vy = 0;
				vr = -0.2; //Self rotate till id4 is seen
				if(flags[3] && rvecs[0][2] < 0.4 && rvecs[0][2] > -0.4) //看到四 此時也能矯正方向
				{
					state = 6;
				}
			}
			//--------------------------------------可以轉向並且飛id2 3 4 單元測試-----------------------//
			//--------------------------------------降落 單元測試--------------------------------------//
			else if(state == 6 && ids.size() > 0 && tvecs[index[3]][2] > required_distance + 30 && flags[3] == true)
			{
				cout << "If block 10"<<endl;
				vx = 0.32;
				vy = 0;
				vr = 0;
			}
			else if(state == 6 && ids.size() > 0 && tvecs[index[3]][2] > required_distance + 25 && tvecs[index[3]][2] <= required_distance + 30 && flags[3] == true) //因為慣性，要遠一點
			{
				cout << "If block 11"<<endl;
				vx = 0.18;
				vy = 0;
				vr = 0;
			}
			else if(state == 6 && ids.size() > 0 && tvecs[index[3]][2] <= required_distance + 25 && flags[3] == true) //慢慢接近終點
			{
				cout << "If block 12"<<endl;
				final_cnt++;
				if(final_cnt >= 3)
				{
					cout << "Chamge camera !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!-----------   " <<endl;
					ardrone.setCamera(++mode % 4);
					state = 7;
					ids.resize(0);
				}
				else
				{
					vx = 0.15;
					vy = 0;
					vr = 0;
				}
			}
			else if(state == 7) //final landing
			{
				cout << " Landinmg counter "<< landing_counter <<endl;
				if(flags[4] == true || landing_counter == 400)
				{
					cout << "Real landing "<<landing_counter << " ids size " << ids.size() <<endl;
					ardrone.landing();
					exit(0);
				}
				else
				{
					vx = 0;
					vy = 0;
					vr = 0.15;
					++landing_counter;
				}
			}
			else //deafult
			{
				switch(state)
				{
					case 4:
					{
						if(ids.size())
						{
							cout << "Default 4"<<endl;
							vx = 0.32;
							vr = 0;
						}
						else
						{
							vx = 0;
							cout << "Default 4 else"<<endl;
						}
						break;
					}
					case 6:
					{
						if(ids.size())
						{
							cout << "Default 6"<<endl;
							vx = 0.32;
							vr = 0;
						}
						else
						{
							cout << "Default 6 else"<<endl;
							vx = 0;
						}
						break;
					}
					default:
					{
						if(!ids.size())
						{
							vr = -0.15;
						}
						else if (state == 1)
						{
							vr = -0.15;
						}
						cout << "Default"<<endl;
						break;
					}
				}
			}
			//--------------------------------------降落 單元測試--------------------------------------//
			imshow("Aruco Marker Axis", image);
			memset(flags, false, sizeof(flags));

		}
		printf("vx : %f vy : %f vr %f\n", vx, vy, vr);
		ardrone.move3D(vx, vy, vz, vr);
		// Display the image
		// imshow("camera", image);
	}
	// See you
	ardrone.close();
	return 0;
}
/*if(state == 0)
{
	vr = -0.4; //Self rotate till id1 is seen
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
