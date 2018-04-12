#include "camera_calibration_module.h"

int main(int argc, char const *argv[])
{
    /* code */
    cv::Mat cameraMatrix, distCoeffs;
    my_camera_calibration(cameraMatrix, distCoeffs);
    return 0;
}
