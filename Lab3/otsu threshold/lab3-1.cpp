#include <bits/stdc++.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

void otsuThreshold(Mat& input, Mat& output);
void findHistogram(Mat& input, vector<int>& histo);
float variance(vector<int> values, float valuesMean, int NUM);
float average(vector<int> vec);

int main(int argc, char **argv){
	
	Mat input_img = imread(argv[1]);
    
	//since the bgr channel is used for default action, then the BGR 3 channel image must be converted to GREY channel
	cvtColor(input_img, input_img, CV_BGR2GRAY);

	Mat output_img = input_img.clone();
	otsuThreshold(input_img, output_img);	


	//imshow("origin", input_img);
    imshow("otsu", output_img);
    waitKey(0);

    imwrite("output_result.jpg", output_img);

    return 0; 
}

//Function for variance
float variance(vector<int> values, float valuesMean, int NUM){
	if(NUM == 0)
		return 0;

    int sum = 0;
    for (int i = 0; i < values.size(); i++)
    {
        sum += (values[i] - valuesMean) * (values[i] - valuesMean);
    }
    return (float)sum / NUM;
}


void otsuThreshold(Mat& input, Mat& output){

	float sum = 9999;
	int bestThreshold;
	
	vector<int> histo(256,0);
	findHistogram(input, histo);
	
	
	for(int i=0;i<256;i++){

		vector<int> small;
		vector<int> big;
		for(int j=0;j<i;j++){
			for(int k=0;k<histo[j];k++)
				small.push_back(j);
		}
		for(int j=i;j<256;j++){
			for(int k=0;k<histo[j];k++)
				big.push_back(j);
		}

		float averageS = average(small);
		float averageB = average(big);

		
		float newSum = small.size() * variance(small, averageS, small.size()) + big.size() * variance(big, averageB, big.size());
		if (sum == 9999){
			sum = newSum;
			bestThreshold = i;
		}
		else if(newSum <= sum){
			sum = newSum;
			bestThreshold = i;
		}
		//cout << sum << endl;
		

		/*
		float newSum = small.size() * big.size() * (averageS - averageB) * (averageS - averageB);
		if (sum == 9999){
			sum = newSum;
			bestThreshold = i;
		}
		else if(newSum >= sum){
			sum = newSum;
			bestThreshold = i;
		}
		//cout << sum << endl;
		*/

	}

	for(int i=0;i<input.rows;i++){
		for(int j=0;j<input.cols;j++){
			if(input.at<uchar>(i,j) < bestThreshold)
				output.at<uchar>(i,j) = 0;
			else
				output.at<uchar>(i,j) = 255;
		}
	}

	cout << "Threshold: " << bestThreshold << endl;
}

void findHistogram(Mat& input, vector<int>& histo){
	for(int i=0;i<input.rows;i++){
		for(int j=0;j<input.cols;j++){
			histo[input.at<uchar>(i,j)]++;
		}
	}
}

float average(vector<int> vec){
	if(vec.size() == 0)
		return 0;

	int sum = 0;
	for(int i=0;i<vec.size();i++){
		sum += vec[i];
	}
	return (float)((float)sum / vec.size());
}