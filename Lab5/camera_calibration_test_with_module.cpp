#include "camera_calibration_module.h"

int main(int argc, char const *argv[])
{
    /* code */
    if(argc != 2)
    {
        cout << "[Usage] ./calib <XML FILE>" << endl;
        return 0;
    }
    cv::Mat cameraMatrix, distCoeffs;
    my_camera_calibration(cameraMatrix, distCoeffs, argv[1]);
    return 0;
}
