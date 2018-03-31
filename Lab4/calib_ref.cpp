#include <cstring>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <cstdio>
#include <vector>

// Parameter for calibration pattern
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
int main(int argc, char *argv[])
{

    if(argc != 2) {
	cout << "[Usage] ./calib <XML FILE>" << endl;
	return 0;
    }

    // Images
    VideoCapture cap(0);
    Mat frame;
    //Mat frame = cap.read();
     cap >> frame;
	// Open XML file
    cout << frame.size() << endl;
    string filename(argv[1]);
    FileStorage fs(filename, FileStorage::READ);

    // Not found
    if (1) {
        // Image buffer
        vector<Mat> images;
        cout << "Press Space key to capture an image" << endl;
        cout << "Press Esc to exit" << endl;

        // Calibration loop
        while (1) {
            // Key iput
            int key = waitKey(1) & 0xFF;
            printf("Keyval %d\n",key);
            if (key == 27) break;

            // Get an image
	    //frame = cap.read();
	    cap >> frame;

            // Convert to grayscale
            cv::Mat gray;
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

            // Detect a chessboard
            cv::Size size(PAT_COLS, PAT_ROWS);
            std::vector<cv::Point2f> corners;
            bool found = cv::findChessboardCorners(gray, size, corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);

            // Chessboard detected
            if (found) {
                // Draw it
                cv::drawChessboardCorners(frame, size, corners, found);

                // Space key was pressed
                if (key == ' ') {
                    // Add to buffer
                    images.push_back(gray);
                }
            }

            // Show the image
            std::ostringstream stream;
            stream << "Captured " << images.size() << " image(s).";
            cv::putText(frame, stream.str(), cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, 1);
            cv::imshow("Camera Calibration", frame);
        }

        // We have enough samples
        if (images.size() > 4) {
            cv::Size size(PAT_COLS, PAT_ROWS);
            std::vector< std::vector<cv::Point2f> > corners2D;
            std::vector< std::vector<cv::Point3f> > corners3D;

            for (size_t i = 0; i < images.size(); i++) {
                // Detect a chessboard
                std::vector<cv::Point2f> tmp_corners2D;
                bool found = cv::findChessboardCorners(images[i], size, tmp_corners2D);

                // Chessboard detected
                if (found) {
                    // Convert the corners to sub-pixel
                    cv::cornerSubPix(images[i], tmp_corners2D, cvSize(11, 11), cvSize(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.1));
                    corners2D.push_back(tmp_corners2D);

                    // Set the 3D position of patterns
                    const float squareSize = CHESS_SIZE;
                    std::vector<cv::Point3f> tmp_corners3D;
                    for (int j = 0; j < size.height; j++) {
                        for (int k = 0; k < size.width; k++) {
                            tmp_corners3D.push_back(cv::Point3f((float)(k*squareSize), (float)(j*squareSize), 0.0));
                        }
                    }
                    corners3D.push_back(tmp_corners3D);
                }
            }

            // Estimate camera parameters
            cv::Mat cameraMatrix, distCoeffs;
            std::vector<cv::Mat> rvec, tvec;
            cv::calibrateCamera(corners3D, corners2D, images[0].size(), cameraMatrix, distCoeffs, rvec, tvec);
            std::cout << cameraMatrix << std::endl;
            std::cout << distCoeffs << std::endl;

            // Save them
            cv::FileStorage tmp(filename, cv::FileStorage::WRITE);
            tmp << "intrinsic" << cameraMatrix;
            tmp << "distortion" << distCoeffs;
            tmp.release();

            // Reload
            fs.open(filename, cv::FileStorage::READ);
        }

        // Destroy windows
        cv::destroyAllWindows();
    }

    // Load camera parameters
    cv::Mat cameraMatrix, distCoeffs;
    fs["intrinsic"] >> cameraMatrix;
    fs["distortion"] >> distCoeffs;

    // Create undistort map
    cv::Mat mapx, mapy;
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cameraMatrix, frame.size(), CV_32FC1, mapx, mapy);

    // Main loop
    while (1) {
        // Key input
        int key = cv::waitKey(33);
        if (key == 0x41) break;

        // Get an image
        //cv::Mat image_raw = GP.getImage();
	//Mat image_raw = cap.read();
	Mat image_raw;
	cap >> image_raw;

        // Undistort
        cv::Mat image;
        cv::remap(image_raw, image, mapx, mapy, cv::INTER_LINEAR);

        // Display the image
        cv::imshow("camera", image);
    }

    return 0;
}
