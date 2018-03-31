#include "cameracalib.h"

using namespace std;
using namespace cv;

const float EDGE_LEN = 2.2;
const int COLS = 9;
const int ROWS = 6;

Size mychessboard(COLS, ROWS);

int main(int argc, char const *argv[])
{
    VideoCapture camera(0);
    Mat frame;
    vector<Mat> images;

    string config_filename(argv[1]);
    FileStorage fs(config_filename. FileStorage::READ); //read camera configuration

    if(!fs.isOpened())//if a configuration file is not found, then run calibration
    {
        cout << "Press Space key to capture an image" << endl;
        cout << "Press Esc to exit" << endl;

        while(1)
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
            for(int i = 0;i < image.size();i++) //calibrate pic by pic
            {
                vector<Point3f> one3d_points;
                vector<Point2f> one2d_points; //more detailed points to be output for more precise calibration

                cornerSubPix(images[i], one2d_points, cvSize(8, 8), cvSize(-1, -1), TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 50, 0.1));
                all2d_points.push_back(one2d_points);

                for(int j = 0;j < mychessboard.height;j++) //put into real world 3d space with z axis being 0 and unit as centimeter
                {
                    for(int k = 0;k < mychessboard.width;k++)
                    {

                    }
                }
            }
        }
    }

    return 0;
}
