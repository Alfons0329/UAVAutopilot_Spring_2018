#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

const float EDGE_LEN = 22;
const int COLS = 9;
const int ROWS = 6;

Size mychessboard(COLS, ROWS);

int main(int argc, char const *argv[])
{
    VideoCapture camera(0);
    Mat frame;
    vector<Mat> images;

    string config_filename(argv[1]);
    FileStorage fs(config_filename, FileStorage::READ); //read camera configuration

    Mat camera_matrix, distortion_coef;//this will be used in the future

    if(!fs.isOpened())//if a configuration file is not found, then run calibration
    {
        cout << "Press Space key to capture an image" << endl;
        cout << "Press Esc to exit" << endl;

        while(1) //capturing the image for calibration
        {
            int key = waitKey(1) & 0xFF;
            if (key == 27) break;

            camera >> frame;
            cvtColor(frame, frame, COLOR_BGR2GRAY);//convert to gray since the findChessboardCorners only accepts the grayscale
            vector<Point2f> corners; //the corners found on chessboard, used in the output

            /*
            bool findChessboardCorners(InputArray image, Size patternSize, OutputArray corners, int flags = CALIB_CB_ADAPTIVE_THRESH+CALIB_CB_NORMALIZE_IMAGE)
            image：輸入圖，必須為8位元的灰階或彩色影像。
            patternSize：棋盤的尺寸，patternSize = Size(points_per_row,points_per_colum) = Size(columns,rows)。
            corners：輸出角點。
            flags：旗標，CV_CALIB_CB_ADAPTIVE_THRESH表示使用區域自適應濾波進行二值化，CV_CALIB_CB_NORMALIZE_IMAGE表示二值化前先呼叫equalizeHist()。
            有找到棋盤角點返回true，否則返回false。
            */
            bool found = findChessboardCorners(frame, mychessboard, corners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FAST_CHECK);
            if(found)
            {
                cout<<"Corners of chessboard are: ";
                for(auto it = corners.begin(); it != corners.end(); ++it )
                {
                    cout<<*it<<" ";
                }
                drawChessboardCorners(frame, mychessboard, corners, found);
                if(key == 32)
                {
                    images.push_back(frame);
                }
            }
            imshow("Doing calibration ",frame);
        }

        cout<<"Sample enough, doing calibration "<<endl;
        if(images.size() >= 20)
        {
            vector<vector<Point3f> > all3d_points;
            vector<vector<Point2f> > all2d_points;
            for(int i = 0;i < images.size();i++) //calibrate pic by pic
            {
                vector<Point3f> one3d_points;
                vector<Point2f> one2d_points; //more detailed points to be output for more precise calibration

                cornerSubPix(images[i], one2d_points, cvSize(8, 8), cvSize(-1, -1), TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 50, 0.1));
                all2d_points.push_back(one2d_points);

                for(int j = 0;j < mychessboard.height;j++) //put into real world 3d space with z axis being 0 and unit as millimeter
                {
                    for(int k = 0;k < mychessboard.width;k++)
                    {
                        one3d_points.push_back(Point3f((float) j * EDGE_LEN,(float) k * EDGE_LEN, 0.0f));
                    }
                }
                all3d_points.push_back(one3d_points); //push back to the real world coordinate
            }

            vector<Mat> rvec, tvec; //radial vector and tangantial vectors in the correction math expression
            calibrateCamera(all3d_points, all2d_points, images[0].size(), camera_matrix, distortion_coef, rvec, tvec);

            cout<<camera_matrix<<endl;
            cout<<distortion_coef<<endl;

            // Save the camera configuration after calibration
            FileStorage tmp(config_filename, FileStorage::WRITE);
            tmp << "intrinsic" << camera_matrix;
            tmp << "distortion" << distortion_coef;
            tmp.release(); //kind of fclose

            // Reload the configuration data of this calibrated camera
            fs.open(config_filename, FileStorage::READ);
        }

    }

    fs["intrinsic"] >> camera_matrix;
    fs["distortion"] >> distortion_coef;

    // Create undistorted ramappoing with that matrix operation mentioned in pdf
    /*
    OpenCV 校正矩陣
    void initUndistortRectifyMap(InputArray cameraMatrix, InputArray distCoeffs, InputArray R, InputArray newCameraMatrix, Size size, int m1type, OutputArray map1, OutputArray map2)

    cameraMatrix：輸入的相機矩陣。
    distCoeffs：輸入的畸變參數。
    newCameraMatrix：新的相機矩陣。
    size：沒有畸變影像的尺寸。
    m1type：map1的型態，可以為CV_32FC1或CV_16SC2。
    map1：第一個輸出矩陣。
    map2：第二個輸出矩陣
    */
    Mat remap_matrix1, remap_matrix2;
    initUndistortRectifyMap(camera_matrix, distortion_coef, cv::Mat(), camera_matrix, frame.size(), CV_32FC1, remap_matrix1, remap_matrix2);

    // Main loop
    while (1)
    {
        // Key input
        int key = waitKey(33);
        if (key == 27) break;
	    Mat image_raw, calibrated_image;
	    camera >> image_raw;
        remap(image_raw, calibrated_image, remap_matrix1, remap_matrix2, INTER_LINEAR /*LINEAR TRANSFORM*/);

        // Display the image
        imshow("camera", calibrated_image);
    }

    return 0;
}
