#include <bits/stdc++.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#define SQUARE_EDGE_LEN 2.20
#define FPS 20
#define pb push_back
using namespace cv;
using namespace std;

const Size chessboard_dim = Size(9 ,6); //x times y in Eucalidian coordinate
void create_known_board_pos(Size boardsize,vector<Point3f>& corners)
{
    for(int i = 0;i < boardsize.height;i++)
    {
        for(int j  = 0;i < boardsize.height;i++)
        {
            corners.pb(Point3f(i * SQUARE_EDGE_LEN, j * SQUARE_EDGE_LEN, ));
        }
    }
}
void get_chessboard_corners(vector<Mat> images, vector<vector <Point2f>> & all_found_corners, bool draw_result)
{
    for(vector<Mat>::iterator it = images.begin(), it != imahes.end(), it++)
    {
        vector<Point2f> point_buffer;

        bool found_corner = findChessboardCorners(*iter, Size(9, 6), point_buffer, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_NORMALIZE_IMAGE);
        if(found_corner)
        {
            all_found_corners.pb(pointbuf);
        }
        if(draw_result)
        {
            drawChessboardCorners(*iter, Size(9, 6), point_buffer, found);
            imshow("Corner searching", *iter);
            waitKey(30);
        }
    }
}
void camera_calibration_main(vector<Mat> images_for_calibration, Size boardsize,Mat& camera_matrix ,Mat& dist_coef)
{
    vector<vector <Point2f>> chessboard_image_space_points;
    get_chessboard_corners(images_for_calibration, chessboard_image_space_points, false);

    vector<vector <Point3f>> realworld_space_points(1); //space matrix in the real world
    create_known_board_pos(boardsize, SQUARE_EDGE_LEN, realworld_space_points[0]); //put the first in
    realworld_space_points.resize(images_for_calibration.size(), realworld_space_points[0]);

    vector<Mat> rvecs, tvecs;
    dist_coef = Mat::zeros(0, 1, CV_64F); //zeromatrix row col
    calibrateCamera(realworld_space_points, chessboard_image_space_points, boardsize, camera_matrix, dist_coef);
}

void show_camera_matrix(Mat camera_matrix, Mat dist_coef)
{
    cout<<"Current camera coefficient matrix: \n";
    double extracted_val;
    cout<<"Camera matrix : "
    for(int i = 0;i < camera_matrix.rows;i++)
    {
        for(int j = 0;j < camera_matrix.cols;j++)
        {
            extracted_val = camera_matrix.at<double>(i, j);
            cout<<extracted_val<<" ";
        }
        cout<<endl;
    }

    cout<<" \n Distance coefficient : "
    for(int i = 0;i < dist_coef.rows; i++)
    {
        for(int j = 0;j < dist_coef.cols; j++)
        {
            extracted_val = dist_coef.at<double>(i, j);
            cout<<extracted_val<<" ";
        }
        cout<<endl;
    }
}
int main(int argc, char const *argv[])
{
    //reference https://blog.csdn.net/jianguo_cui/article/details/7387169
    VideoCapture cap(1);
    Mat frame, drawto_frame, dist_coef, tmp;
    Mat camera_matrix = Mat::eye(3, 3, CV_64F); //identity matrix zero
    vector<Mat> saved_images;
    while (1)
    {
        cap >> frame;
        if(!cap.read(frame))
        {
            break;
        }
        imshow("webcam", frame);
        bool found = false;

        found = findChessboardCorners(frame, chessboard_dim, found_points, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_NORMALIZE_IMAGE);
        frame.cppyTo(drawto_frame);
        drawChessboardCorners(drawto_frame, chessboard_dim, found_points, found);

        if(found)
        {
            imshow("Webcam", drawto_frame);
        }
        else
        {
            imshow("Webcam", drawto_frame);
            imshow("Webcam", frame);
        }

        char input_char = waitKey(1000 / FPS);

        switch(input_char)
        {
            case ' '://save image till enough
                if(found)
                {
                    frame.copyTo(tmp);
                    saved_images.push_back(frame);
                }
                break;
            case 13: //enter start calibration
            {
                if(saved_images.size() > 20) //20 is enough
                {
                    camera_calibration_main(saved_images, chessboard_dim, camera_matrix, dist_coef);
                    show_camera_matrix(camera_matrix, dist_coef);
                }
            }

            case 27:
                exit(1);
        }
    }
    return 0;
}
