#include <bits/stdc++.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

#define N 10000

int array[N][3];

void connectedComponents(Mat& input, Mat& output);
void findNext(int i, int j, int label, int rows, int cols, Mat& input);
void buildColor(void);


int main(int argc, char **argv){
	
	Mat input_img = imread(argv[1]);
    
	//since the bgr channel is used for default action, then the BGR 3 channel image must be converted to GREY channel
	cvtColor(input_img, input_img, CV_BGR2GRAY);

	Mat output_img = input_img.clone();
	cvtColor(output_img, output_img, CV_GRAY2BGR);
	
	//buildColor();
	connectedComponents(input_img, output_img);	


	//imshow("origin", input_img);
    imshow("connectedComponents", output_img);
    waitKey(0);

    imwrite("output_result.jpg", output_img);

    return 0; 
}

void buildColor(void){
	for(int i=0;i<N;i++){

		while(1){
			srand(time(NULL));
			int random_integer = 1 + rand() % 254;
			array[i][0] = random_integer;

			array[i][1] = (255 - random_integer) & 0xFF;

			array[i][2] = (125 - random_integer) & 0xFF;

			int j;
			for(j=0;j<i;j++){
				if(array[j][0] == array[i][0] && array[j][1] == array[i][1] && array[j][2] == array[i][2])
					break;
			}
			if(j == i)
				break;
		}
	}
}

void connectedComponents(Mat& input, Mat& output){
	for(int i=0;i<input.rows;i++){
		for(int j=0;j<input.cols;j++){
			if(input.at<uchar>(i,j) < 200){
				input.at<uchar>(i,j) = 0;
			}
			else 
				input.at<uchar>(i,j) = 255;
		}
	}


	int label = 50;
	for(int i=0;i<input.rows;i++){
		for(int j=0;j<input.cols;j++){
			if(input.at<uchar>(i,j) == 255){
				input.at<uchar>(i,j) = label;
				findNext(i, j, label, input.rows, input.cols, input);
				label += 10;
			}
		}
	}

	for(int i=0;i<input.rows;i++){
		for(int j=0;j<input.cols;j++){
			if(input.at<uchar>(i,j) != 0){
				//cout << input.at<uchar>(i,j) << endl;
				int label2 = input.at<uchar>(i,j);
				output.at<Vec3b>(i,j)[0] = (label2%45 * 531)% 255;
				output.at<Vec3b>(i,j)[1] = 255 - label2;
				output.at<Vec3b>(i,j)[2] = (label2%30 * 35)% 255;
			}
			else{
				output.at<Vec3b>(i,j)[0] = 0;
				output.at<Vec3b>(i,j)[1] = 0;
				output.at<Vec3b>(i,j)[2] = 0;	
			}
		}
	}

}

void findNext(int i, int j, int label, int rows, int cols, Mat& input){
	if(i-1 >= 0){
		if(input.at<uchar>(i-1,j) == 255){
			input.at<uchar>(i-1,j) = label;
			findNext(i-1, j, label, input.rows, input.cols, input);
		}
		if(j-1 >= 0){
			if(input.at<uchar>(i-1,j-1) == 255){
				input.at<uchar>(i-1,j-1) = label;
				findNext(i-1, j-1, label, input.rows, input.cols, input);
			}
		}
		if(j+1 < cols){
			if(input.at<uchar>(i-1,j+1) == 255){
				input.at<uchar>(i-1,j+1) = label;
				findNext(i-1, j+1, label, input.rows, input.cols, input);
			}
		}
	}
	if(i+1 < rows){
		if(input.at<uchar>(i+1,j) == 255){
			input.at<uchar>(i+1,j) = label;
			findNext(i+1, j, label, input.rows, input.cols, input);
		}
		if(j-1 >= 0){
			if(input.at<uchar>(i+1,j-1) == 255){
				input.at<uchar>(i+1,j-1) = label;
				findNext(i+1, j-1, label, input.rows, input.cols, input);
			}
		}
		if(j+1 < cols){
			if(input.at<uchar>(i+1,j+1) == 255){
				input.at<uchar>(i+1,j+1) = label;
				findNext(i+1, j+1, label, input.rows, input.cols, input);
			}
		}
	}
	if(j-1 >= 0){
		if(input.at<uchar>(i,j-1) == 255){
			input.at<uchar>(i,j-1) = label;
			findNext(i, j-1, label, input.rows, input.cols, input);
		}
	}
	if(j+1 < cols){
		if(input.at<uchar>(i,j+1) == 255){
			input.at<uchar>(i,j+1) = label;
			findNext(i, j+1, label, input.rows, input.cols, input);
		}
	}
}