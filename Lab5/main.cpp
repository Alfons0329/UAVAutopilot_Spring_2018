#include "ardrone/ardrone.h"
#include "pid.hpp"
// #include "calibration.h" I dorectly write in this file
#include "opencv2/opencv.hpp"
#include <opencv2/aruco.hpp>
#include <bits/stdc++.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cstring>

#define PAT_ROWS   (6)                  // Rows of pattern
#define PAT_COLS   (9)                 // Columns of pattern
#define CHESS_SIZE (22.0)               // Size of a pattern [mm]

using namespace std;
using namespace cv;
// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------
//------------------Marker dection board constants--------------------------//
const float markerLength = 7.05f;
//------------------Marker dection board constants end here-----------------//

// AR.Drone class
ARDrone ardrone;

void my_camera_calibration(Mat& cameraMatrix, Mat& distCoeffs,string name_in)
{
        // image_buffer_vector 0 for built-in camera and 1 for the external camera
        // VideoCapture cap(0);
        //Mat image = cap.read();
        // cap >> image;
        // Open XML file
        // cout << image.size() << endl;
        //-------------------------------Data structure init-------------------------//
        Mat image;


        string filename(name_in );
        FileStorage fs(filename, FileStorage::READ);
        //-------------------------------Data structure init end here---------------------//

        // Not found
        if (1)
        {
            // Image buffer
            vector<Mat> image_buffer_vector;
            cout << "Press Space key to capture an image" << endl;
            cout << "Press Esc to exit" << endl;

            // Calibration loop
            while (1)
            {
                // Key iput
                int key = waitKey(1) & 0xFF;
                if (key == 27) break;

                // Get an image
                image = ardrone.getImage();

                // Convert to grayscale
                Mat gray;
                cvtColor(image, gray, COLOR_BGR2GRAY);

                // Detect a chessboard
                Size size(PAT_COLS, PAT_ROWS);
                vector<Point2f> corners;
                bool found = findChessboardCorners(gray, size, corners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FAST_CHECK);

                // Chessboard detected
                if (found)
                {
                    // Draw it
                    drawChessboardCorners(image, size, corners, found);

                    // Space key was pressed
                    if (key == ' ')
                    {
                        // Add to buffer
                        image_buffer_vector.push_back(gray);
                        // imshow("Haha this image", image_buffer_vector[image_buffer_vector.size() - 1]);
                    }
                }

                // Show the image
                ostringstream stream;
                stream << "Captured " << image_buffer_vector.size() << " image(s).";
                putText(image, stream.str(), Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1, 1);
                imshow("Camera Calibration", image);
            }

            // We have enough samples
            if (image_buffer_vector.size() > 4)
            {
                Size size(PAT_COLS, PAT_ROWS);
                vector< vector<Point2f> > corners2D;
                vector< vector<Point3f> > corners3D;

                for (size_t i = 0; i < image_buffer_vector.size(); i++)
                {
                    // Detect a chessboard
                    vector<Point2f> tmp_corners2D;
                    // imshow("Haha", image_buffer_vector[i]);
                    // getchar();
                    bool found = findChessboardCorners(image_buffer_vector[i], size, tmp_corners2D, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FAST_CHECK);

                    // Chessboard detected
                    if (found)
                    {
                        // Convert the corners to sub-pixel
                        cornerSubPix(image_buffer_vector[i], tmp_corners2D, cvSize(11, 11), cvSize(-1, -1), TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 30, 0.1));
                        corners2D.push_back(tmp_corners2D);

                        // Set the 3D position of patterns
                        const float squareSize = CHESS_SIZE;
                        vector<Point3f> tmp_corners3D;
                        for (int j = 0; j < size.height; j++)
                        {
                            for (int k = 0; k < size.width; k++)
                            {
                                tmp_corners3D.push_back(Point3f((float)(k*squareSize), (float)(j*squareSize), 0.0));
                            }
                        }
                        corners3D.push_back(tmp_corners3D);
                    }
                }

                // Estimate camera parameters
                //Mat cameraMatrix, distCoeffs;
                vector<Mat> rvec, tvec;
                calibrateCamera(corners3D, corners2D, image_buffer_vector[0].size(), cameraMatrix, distCoeffs, rvec, tvec);
                cout << cameraMatrix << endl;
                cout << distCoeffs << endl;

                // Save them
                FileStorage tmp(filename, FileStorage::WRITE);
                tmp << "intrinsic" << cameraMatrix;
                tmp << "distortion" << distCoeffs;
                tmp.release();

                // Reload
                fs.open(filename, FileStorage::READ);
            }

            // Destroy windows
            destroyAllWindows();
        }
}
int main(int argc, char *argv[])
{
    // AR.Drone class
    //ARDrone ardrone;

    // Initialize
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
    Mat image;
    //-----------------------camera calibration for ardrone----------------------------------//
    Mat cameraMatrix, distCoeffs;
    my_camera_calibration(cameraMatrix, distCoeffs, "config.xml");
    //-----------------------drone aruco checking and detection data structure init------------------------------//
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250); //just this
    vector<int> ids; //aruco markers
    vector<vector<Point2f> > aruco_corners; //aruco corners
    //-----------------------done data structure init------------------------------------------//
    //
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
        if (key == 'i' || key == CV_VK_UP)    vx =  1.0;
        if (key == 'k' || key == CV_VK_DOWN)  vx = -1.0;
        if (key == 'u' || key == CV_VK_LEFT)  vr =  1.0;
        if (key == 'o' || key == CV_VK_RIGHT) vr = -1.0;
        if (key == 'j') vy =  1.0;
        if (key == 'l') vy = -1.0;
        if (key == 'q') vz =  1.0;
        if (key == 'a') vz = -1.0;
        ardrone.move3D(vx, vy, vz, vr);

        // Change camera
        static int mode = 0;
        if (key == 'c') ardrone.setCamera(++mode % 4);

        // Display the image
        imshow("camera", image);

        //---------------------------------Main part of market detection----------------------------------------------//
        aruco::detectMarkers(image, dictionary, aruco_corners, ids);
           // if at least one marker detected
        vector<Vec3d> rvecs, tvecs;
        if (ids.size() > 0)
        {
            aruco::drawDetectedMarkers(image, aruco_corners, ids);
            aruco::estimatePoseSingleMarkers(aruco_corners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);
            // draw axis for each marker
            //the more the last coefficient , the more obviously the axis will be drawn
            for(int i = 0;i < ids.size(); i++)
            {
                aruco::drawAxis(image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 10); //if read
                cout<<"Detected ArUco markers "<<ids.size()<< "x,y,z = " << tvecs[0] << endl; //x,y,z in the space
            }
        }
        imshow("Aruco Market Axis", image);
        if (key == -1 && tvecs.size())
        {
    		//---------------------------------Main part of market detection----------------------------------------------//
            // aruco::detectMarkers(image, dictionary, aruco_corners, ids);
            //    // if at least one marker detected
            // vector<Vec3d> rvecs, tvecs;
            // if (ids.size() > 0)
            // {
            //     aruco::drawDetectedMarkers(image, aruco_corners, ids);
            //
            //     aruco::estimatePoseSingleMarkers(aruco_corners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);
            //     // draw axis for each marker
            //     //the more the last coefficient , the more obviously the axis will be drawn
            //     for(int i = 0;i < ids.size(); i++)
            //     {
            //         aruco::drawAxis(image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 10); //if read
            //         cout<<"Detected ArUco markers "<<ids.size()<< "x,y,z = " << tvecs[0] << endl; //x,y,z in the space
            //     }
            // }
            // imshow("Aruco Market Axis", image);

            PIDManager myPID("pid.yaml");
            //myPID.importCoeffsFromFile("pid.yaml");
            // implement your autopilot algorithm here
    		// only need to modify vx, vy, vz, vr

            double previous_error_x = 0;
            double integral_x = 0;
            double previous_error_y = 0;
            double integral_y = 0;
            double previous_error_z = 0;
            double integral_z = 0;
            double previous_error_r = 0;
            double integral_r = 0;

            double dt_x = 0.1;
            double dt_r = 0.1;



            double actual_x = tvecs[0][2];
            double error_x = 80 - actual_x;
            integral_x = integral_x + error_x*dt_x;
            double derivative_x = (error_x - previous_error_x)/dt_x;
            double output_x = myPID.mX.at<double>(0, 0)*error_x + myPID.mX.at<double>(1, 0)*integral_x + myPID.mX.at<double>(2, 0)*derivative_x;
            previous_error_x = error_x;

            //myPID.setThrottleLevel(output_x);
            // wait((int)dt_x);
            double actual_r = tvecs[0][2];
            double error_r = 80 - actual_r;
            integral_r = integral_r + error_r*dt_r;
            double derivative_r = (error_r - previous_error_r)/dt_r;
            double output_r = myPID.mR.at<double>(0, 0)*error_r + myPID.mR.at<double>(1, 0)*integral_r + myPID.mR.at<double>(2, 0)*derivative_r;
            previous_error_r = error_r;

            //myPID.setThrottleLevel(output_r);
            // wait((int)dt_r);
            double error_y = 0;
            double error_z = 0;
            Mat input = Mat::zeros(4, 1, CV_64F);
            Mat out = Mat::zeros(4, 1, CV_64F);
            out.at<double>(0, 0) = error_x;
            out.at<double>(1, 0) = error_y;
            out.at<double>(2, 0) = error_z;
            out.at<double>(3, 0) = error_r;
            myPID.getCommand(input, out);
            cout << out.at<double>(0, 0) << ", " << out.at<double>(3, 0);


            if(out.at<double>(0, 0) <= 0.5 && out.at<double>(0, 0) >= -0.5)
                vx = 0;
            else if(out.at<double>(0, 0) < -0.5){
                vx = 1;
            }
            else{
                vx = -1;
            }

	   }

	   ardrone.move3D(vx, vy, vz, vr);
       waitKey(100);
    }
    // See you
    ardrone.close();

    return 0;
}
