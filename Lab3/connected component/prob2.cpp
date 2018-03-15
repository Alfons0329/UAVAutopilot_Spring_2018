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
	//https://ccw1986.blogspot.tw/2016/03/connected-component-labeling-using.html

	unsigned char label_num = 2;
	int passes_level = 0, cnt = 1;
	vector<vector <int> >connect_components;
	connect_components.resize(input.rows);
	for(int i = 0;i < connect_components.size();i++)
	{
		for(int j = 0;j < input.cols;j++)
		{
			connect_components.push_back(input.at<uchar>(i,j));
			cout<<connect_components[i][j]<<endl;
		}
	}

	for(int i = 0;i < input.rows;i++)
	{
		for(int j = 0;j < input.cols;j++)
		{
			if(connect_components[i][j] == 255)
			{
				connect_components[i][j] = cnt;
				if(cnt == 255)
				{
					cnt+=2;
				}
				else
				{
					cnt++;
				}
			}
		}
	}

	//left to right
	for(int i = 0;i < input.rows;i++)
	{
		for(int j = 1;j < input.cols;j++)
		{
			if(connect_components[i][j] != 255 && connect_components[i][j])
			{
				if(connect_components[i][j - 1])
				{
					connect_components[i][j] = connect_components[i][j - 1];
				}
			}
		}
	}

	for(int j = 1;j < input.cols;j++)
	{
		for(int i = 0;i < input.rows; i++)
		{
			if(connect_components[i][j] != 255 && connect_components[i][j])
			{
				if(connect_components[i - 1][j])
				{
					connect_components[i][j] = connect_components[i - 1][j];
				}
			}
		}
	}

	cvtColor(input, input, CV_GRAY2BGR);
	for(int i = 0;i < input.rows;i++)
	{
		for(int j = 1;j < input.cols;j++)
		{
			output.at<Vec3b>(i,j)[0] = connect_components[i][j];
			output.at<Vec3b>(i,j)[1] = (unsigned char)(255 - connect_components[i][j]);
			output.at<Vec3b>(i,j)[2] = (unsigned char)(125 - connect_components[i][j])
		}
	}
	//up to down
	/*for(int i = 1;i<input.rows - 1;i++)
    {
        for(int j = 1;j<input.cols - 1;j++)
        {
			if(connect_components[i][j] > 0) //unlabeled area. white area to be processed
			{
				//U R D L searcing the neighboring
				if(connect_components[i - 1][j] > 0
				&& connect_components[i - 1][j] == connect_components[i][j])
				{
					passes_level++;
				}
				else
				{
					connect_components[i][j] = connect_components[i - 1][j];
					i--; //re check
					passes_level=0;
				}

				if(connect_components[i][j + 1] > 0
				&& connect_components[i][j + 1] == connect_components[i][j]
				&& passes_level == 1)
				{
					passes_level++;
				}
				else
				{
					connect_components[i][j] = connect_components[i][j + 1];
					j++; //re check
					passes_level=0;
				}

				if(connect_components[i + 1][j] > 0
				&& connect_components[i + 1][j] == connect_components[i][j]
				&& passes_level == 2)
				{
					passes_level++;
				}
				else
				{
					connect_components[i][j] = connect_components[i + 1][j];
					i++; //re check
					passes_level=0;
				}

				if(connect_components[i][j - 1] > 0
				&& connect_components[i][j - 1] == connect_components[i][j]
				&& passes_level == 3)
				{
					passes_level;
				}
				else
				{
					connect_components[i][j] = connect_components[i][j - 1];
					i++; //re check
					passes_level=0;
				}
			}
        }
    }*/

}
