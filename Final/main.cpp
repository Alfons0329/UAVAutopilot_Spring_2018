#include "ardrone/ardrone.h"
#include "pid.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <bits/stdc++.h>
#include <iostream>
#include <cstdlib>
#include <stdio.h>
#include <vector>
#include <string>

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
//------------------first_counter--------------------------------------------------//
const int first_counter = 60;
const int second_counter = 40;
//------------------face dodge counter--------------------------------------------//
const int face_lr = 40;
const int face_st = 80;
//------------------face width define---------------------------------------------//
const int face_width_todo = 60;
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
		cout << "Read XML failed !! " << endl;
		return 0;
	}
	fs.release();
	return 1;
}

int main(int argc, char *argv[])
{
	//----------------------face init--------------------------------------------------------------//
	CascadeClassifier face_cascade;
    if(!face_cascade.load( "haarcascade_frontalface_alt2.xml" ))
	{
		cout << "Load XML failed !!!! " << endl;
	}
    if(!face_cascade.load( "haarcascade_frontalface_alt.xml" ))
	{
		cout << "Load XML failed !!!! " << endl;
	}
	vector<Rect> faces;
	// Initialize
	//-----------------------camera parameter loading for ardrone----------------------------------//
	Mat image;
	Mat cameraMatrix(3,3,CV_64F), distCoeffs(1,5,CV_64F);
	load_camera_param("ardrone_config.xml",cameraMatrix , distCoeffs);
	cout << "cameraMatrix " <<cameraMatrix << " \n " << distCoeffs << "--------------------- \n";

	Mat cameraMatrix2(3,3,CV_64F), distCoeffs2(1,5,CV_64F);

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

	//-----------------------drone aruco checking and detection data structure init------------------------------//
	cv::Ptr<aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    vector<int> ids; //aruco markers
	vector<vector<Point2f> > aruco_corners; //aruco corners
	//-----------------------done data structure init------------------------------------------//
	//
	PIDManager myPID("pid.yaml");
	//-----------------------flying data structure init---------------------------------------//
	bool flags[50] = {false}; //Alfons: should fix this one ??
	int index[50] = {0}; //Alfons: should fix this one ??
	memset(flags, false, sizeof(flags));
	//----------------------aruco markers part-----------------------------------------------//
	int state = 0;
	unsigned int final_vote_cnt = 0;
	unsigned long long int counter = 0;
	unsigned long long int landing_counter = 0;

	int face_state = 0;
	int face_width = 0, face_height = 0, face_x = 0, face_y = 0;
	unsigned long long int face_counter = 0;


	while (1)
	{
		// Key input
		cout << "State now " << state << endl;
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
		int mode = 0;

		if (key == 'c') ardrone.setCamera(++mode % 4);

		if (key == 255)
		{
			//------------------------------face detection------------------------------------------//
			aruco::detectMarkers(image, dictionary, aruco_corners, ids);
			face_cascade.detectMultiScale( image, faces, 1.1, 12, CV_HAAR_SCALE_IMAGE, Size(30, 30) );
			face_width = 0;
			for( int i = 0; i < faces.size(); i++ )
		    {
		        Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
		        ellipse( image, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );
				printf("Find %d faces, face width %d, face height %d face_x %d face_y %d\n",faces.size() ,faces[i].width , faces[i].height, faces[i].x, faces[i].y);
				// face_width = max(faces[i].width, face_width);// update the max face size for only one face is accepted
				if(faces[i].width > face_width)
				{
					face_width = faces[i].width;
					face_x = faces[i].x;
				}
		    }

			if(ardrone.onGround())
			{
				state = 0;
			}
			//------------------------------------debug output zone----------------------------------//
			if(ids.size())
			{
				cout << "See marker: ";
			}
			for (int j = 0; j < ids.size(); j++) //Alfons: should fix this one ??
			{
				flags[ids[j]-1] = true;
				cout << " , " << ids[j];
				index[ids[j]-1] = j;
			}
			if(ids.size())
			{
				// cout << "\n----------------------- See marker end---------------------- " << endl;
			}

			//---------------------------------Main part of marker detection----------------------------------------------//
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
						// cout << "Detected ArUco markers " << ids.size() << "x,y,z = " << tvecs[0] << endl; //x,y,z in the space
						// cout << "rvec 0 2 = " << rvecs[0][2] << endl; //x,y,z in the space
					}
				}
				else if(mode == 1) //landing
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
			//--------------------------------------face part (priority higher than state)-----------//
			if(state && faces.size() && face_state == 0 && face_width > face_width_todo )
			{
				cout << "Find a face ! close enough, fly right " << endl;
				face_state = 1;
			}
			else if(state && face_state == 1 && face_counter <= face_lr)
			{
				vy = -0.2;
				face_counter++;
				cout << "Fly right of the face face_counter" <<face_counter << endl;
				if(face_counter == face_lr)
				{
					face_state = 2;
					face_counter = 0;
				}
			}
			else if(state && face_state == 2 && face_counter <= face_st)//614
			{
				vy = 0;
				vx = 0.4;
				face_counter++;
				cout << "Fly straight of the face face_counter" <<face_counter << endl;
				if(face_counter == face_st)
				{
					face_state = 3;
					face_counter = 0;
				}
			}
			else if(face_state == 3 && face_counter <= face_lr)
			{
				vx = 0;
				vy = 0.2;
				face_counter++;
				cout << "Fly left of the face face_counter" <<face_counter << endl;
				if(face_counter == face_lr)
				{
					face_state = 0;
					face_counter = 0;
					if(state < 3) //620
					{
						state = 3;
					}
					else if(state > 3 && state < 10)
					{
						state = 10;
					}
					cout << "Finish face dodge !" <<endl;
					// getchar();
					// ardrone.landing();
				}
			}
			//---------------------------------------marker part-------------------------------------//
			else if(state == 0) //最一剛開始
			{
				cout << " If block 0" <<endl;
				vx = 0;
				vy = 0;
				vr = 0.12; //Self rotate till id1 is seen 正是逆時針，負是順時針
				//vr = 0; //Alfons: should fix this one ??
				if(flags[10] && rvecs[0][2] < 0.82 && rvecs[0][2] > -0.7) //看到marker一 進入狀態一
				{
					state = 1;
				}
			}
			else if(state == 1 && counter < first_counter) //側向傾斜的飛行，飛一個指定的counter來逼近人臉，counter要修改 614
			{
				cout << "If block 1 "<<endl;
				cout << "Counter " << counter++ << endl;
				vx = 0.4;
				vy = -0.09;//- (0.8 / 4.0f) * 1;
				vr = 0;
				if(counter >= first_counter) //counter with trial and error
				{
					state = 1;
					counter = 0xFFFFFFFF;
				}
			}
			else if(state == 1) //在飛行counter結束之後，因為不會正向的看著人臉，所以要自轉一下矯正方向以便看到正確的人臉位置，才能直飛 614
			{
				cout << "If block 3 searching for face"<<endl;
				if(faces.size())
				{
					cout << "Face exists !!!! image cols: " << image.cols << "face_XX " <<face_x << "face_YY" <<face_y << endl;
				}
				vx = 0;
				vy = 0;
				vr = 0.13; //Self rotate till id2 is seen
				if(faces.size() && face_x > ( image.cols / 2 ) - 80 && face_x < ( image.cols / 2 ) + 80) //找人臉並且矯正方向 614
				{
					cout << "Next state " <<endl;
					state = 2;
				}
			}
			else if(state == 2 || state == 9) //state = 2 代表朝人臉飛過去(還沒有要進行人臉躲避障礙誤)。
			{
				cout << "Fly closer to the face !!" << endl;
				vx = 0.13;
				if(face_x <= ( image.cols / 2 ) - 50)
				{
					vy = 0.15;
				}
				else if(face_x >= ( image.cols / 2 ) + 50)
				{
					vy = -0.15;
				}
			}
			/*else if(state == 3 && ids.size() == 0)
			{
				cout << "Seeing nothing adter flying over the face, now searching the pattern " << endl;
				vx = 0;
				vy = 0;
				vr = 0.15;
				if()
			}
			else if(state == 3 && ids)*/
			else if(state == 3 && ids.size() > 0 && tvecs[index[1]][2] > required_distance && flags[1] == true) //持續朝id二飛行
			{
				cout << "If block 4 "<<endl;
				vx = 0.2;
				vy = 0;
				vr = 0;
			}
			else if(state == 3 && ids.size() > 0 && tvecs[index[1]][2] < required_distance  && flags[1] == true) //快到了，停下，停在id二前面
			{
				cout << "If block 5 "<<endl;
				vx = 0;//停下來
				vy = 0;
				vr = 0;
				state = 4; //進入狀態四
			}
			else if(state == 4) //停在id二前面而且看不到id三 就自轉，向右轉會比較快
			{
				cout << "If block 6 "<<endl;
				vx = 0;
				vy = 0;
				vr = -0.13; //Self rotate till id3 is seen（如果要往另一個方向比較快，就加-號 到時候再看看）

				if(flags[2] && rvecs[0][2] < 0.4 && rvecs[0][2] > -0.4) //看到三 此時也能矯正方向
				{
					state = 5;
				}
			}
			//-----------------------------------------613 改到這裡------------------------------------------//
			else if(state == 5 && ids.size() > 0 && tvecs[index[2]][2] > required_distance + 20 && flags[2] == true) //持續朝id三飛行
			{
				cout << "If block 7 "<<endl;
				vx = 0.25;
				vy = 0;
				vr = 0;
			}
			else if(state == 5 && ids.size() > 0 && tvecs[index[2]][2] < required_distance + 20 && flags[2] == true) //快到了，停下，停在id三前面
			{
				cout << "If block 8 "<<endl;
				vx = 0;
				vy = 0;
				vr = 0;
				state = 6;
			}
			else if(state == 6) //停在id二前面而且看不到id四 就自轉，向右轉會比較快
			{
				cout << "If block 9 "<<endl;
				vx = 0;
				vy = 0;
				vr = 0.12; //Self rotate till id4 is seen
				if(flags[20] && rvecs[0][2] < 0.82 && rvecs[0][2] > -0.7) //看到21 此時也能矯正方向 620
				{
					counter = 0;
					state = 7;
					cout << "See 21 marker !! " << endl;
				}
			}
			//--------------------------------------可以轉向並且飛id2 3 4 單元測試-----------------------//
			//--------------------------------------降落 單元測試--------------------------------------//
			else if(state == 7 && counter < second_counter) //看到21後 飛一些counter 超越它，並且航向人臉，邏輯跟之前的一樣620
			{
				cout << "If block 9 - 2" << endl;
				cout << "Counter" << counter++ << endl;
				vx = 0.4;
				vy = -0.09;
				vr = 0;
				if(counter >= second_counter)
				{
					state = 8; //飛完counter後，就進入狀態八，代表持續搜索人臉貼近人臉（見下 620
					counter = 0xFFFFFFFF;
				}
			}
			else if(state == 8) //原地旋轉跟之前一樣，看到人臉並且矯正方向。620
			{
				cout << "If block 9-3 searching for face"<<endl;
				if(faces.size())
				{
					cout << "Face exists !!!! image cols: " << image.cols << "face_XX " <<face_x << "face_YY" <<face_y << endl;
				}
				vx = 0;
				vy = 0;
				vr = 0.13; //Self rotate till id2 is seen
				if(faces.size() && face_x > ( image.cols / 2 ) - 80 && face_x < ( image.cols / 2 ) + 80) //找人臉並且矯正方向 620
				{
					cout << "Next state " <<endl;
					state = 9;
				}
			}
			else if(state == 10 && ids.size() > 0 && tvecs[index[3]][2] > required_distance + 30 && flags[3] == true) //飛完人臉要到最後了，邏輯跟之前差不多 620
			{
				cout << "If block 10 "<<endl;
				vx = 0.32;
				vy = 0;
				vr = 0;
			}
			else if(state == 10 && ids.size() > 0 && tvecs[index[3]][2] > required_distance + 25 && tvecs[index[3]][2] <= required_distance + 30 && flags[3] == true) //因為慣性，要遠一點
			{
				cout << "If block 11 "<<endl;
				vx = 0.18;
				vy = 0;
				vr = 0;
			}
			else if(state == 10 && ids.size() > 0 && tvecs[index[3]][2] <= required_distance + 25 && flags[3] == true) //慢慢接近終點
			{
				cout << "If block    "<<endl;
				final_vote_cnt++;
				if(final_vote_cnt >= 3)
				{
					cout << "Chamge camera !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!-----------   " <<endl;
					ardrone.setCamera(++mode % 4);
					state = 11;
					ids.resize(0);
				}
				else
				{
					vx = 0.15;
					vy = 0;
					vr = 0;
				}
			}
			else if(state == 11) //final landing
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
					case 5:
					{
						if(ids.size())
						{
							cout << "Default 5"<<endl;
							vx = 0.32;
							vr = 0;
						}
						else
						{
							vx = 0;
							cout << "Default 5 else"<<endl;
						}
						break;
					}
					case 10:
					{
						if(ids.size())
						{
							cout << "Default 10"<<endl;
							vx = 0.32;
							vr = 0;
						}
						else
						{
							cout << "Default 10 else"<<endl;
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
						else if (state == 1 || state == 3)
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
		printf("vx : %f vy : %f vr %f\n\n", vx, vy, vr);
		ardrone.move3D(vx, vy, vz, vr);
	}
	// See you
	ardrone.close();
	return 0;
}
