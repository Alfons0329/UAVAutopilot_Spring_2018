//

#include <opencv2/opencv.hpp>
#include <bits/stdc++.h>

using namespace cv;
void  mask(Mat& input, Mat& output);
int main()
{
	namedWindow("demo", 0);
	Mat input_img = imread("output(otsu).jpg",0);
	cvtColor(input_img, input_img, CV_BGR2GRAY);
	Mat connected_comp = input_img.clone();

	ccmp(input_img, connected_comp);

	imshow("origin", input_img);
	imshow("processed", connected_comp);
	waitKey(0);

	imwrite("result.jpg", connected_comp);

	return 0;
}
void ccmp(Mat& input, Mat& output)
{
	//round one, padding, using seed filling
	unsigned char label_num = 2;
	int passes_level = 0
	vector<vector <int> >connect_components;
	connect_components.resize(100000);
	for(int i = 1;i<input.rows - 1;i++)
    {
        for(int j = 1;j<input.cols - 1;j++)
        {
			if(input.at<uchar>(i,j) == 255 || input.at<uchar>(i,j) == 0) //unlabeled area
			{
				//U R D L searcing the neighboring
				if(input.at<uchar>(i - 1, j) > 0
				&& input.at<uchar>(i - 1, j) <255
				&& input.at<uchar>(i - 1, j) == input.at<uchar>(i, j))
				{
					passes_level++;
				}
				else
				{

				}

				if(input.at<uchar>(i, j + 1) > 0
				&& input.at<uchar>(i, j + 1) <255
				&& input.at<uchar>(i, j + 1) == input.at<uchar>(i, j)
				&& passes_level == 1)
				{}
				else
				{

				}

				if(input.at<uchar>(i + 1, j) > 0
				&& input.at<uchar>(i + 1, j) <255
				&& input.at<uchar>(i + 1, j) == input.at<uchar>(i, j))
				{}
				else
				{

				}

				if(input.at<uchar>(i, j - 1) > 0
				&& input.at<uchar>(i, j - 1) <255
				&& input.at<uchar>(i, j - 1) == input.at<uchar>(i, j))
				{}
				else
				{

				}
			}
        }
    }

}
