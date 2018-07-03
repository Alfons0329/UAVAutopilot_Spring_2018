#include <bits/stdc++.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

void histogram_equal(Mat& input, Mat& output);

int main(int argc, char** argv)
{

    Mat input_img = imread(argv[1]);
    //since the bgr channel is used for default action, then the BGR 3 channel image must be converted to GREY channel
    cvtColor(input_img, input_img, CV_BGR2GRAY);
    Mat output_img = input_img.clone();
    histogram_equal(input_img, output_img);

    imshow("origin", input_img);
    imshow("histogram_equal", output_img);
    waitKey(0);

    imwrite("output_result.jpg", output_img);

  return 0;
}

void histogram_equal(Mat& input, Mat& output)
{
    vector<int> hash_distribution;
    vector<double> intensity_cdf;
    hash_distribution.resize(256);
    intensity_cdf.resize(256);
    for(int i=0;i<input.rows;i++)
    {
        for(int j=0;j<input.cols;j++)
        {
            hash_distribution[(int) input.at<uchar>(i,j)]++;
        }
    }

    //search the maxium value
    int max_value = 0, cnt=0;
    double cumulative_cnt = 0.0f;
    for(int i=0;i<hash_distribution.size();i++)
    {
        if(hash_distribution[i]!=0)
        {
            max_value = max(max_value, i);
            cumulative_cnt += (double) hash_distribution[i] / (double)(input.rows * input.cols);
            cnt += hash_distribution[i];
            intensity_cdf[i] = cumulative_cnt;
        }
    }
    for(int i=0;i<input.rows;i++)
    {
        for(int j=0;j<input.cols;j++)
        {
            output.at<uchar>(i,j) = (intensity_cdf[input.at<uchar>(i,j)] * max_value );
        }
    }
}
