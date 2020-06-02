#define _GLIBCXX_USE_CXX11_ABI 0
#include<opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include<vector>
#include<string>
// #include
#include<iostream>
#include"keypoints_tracking.h"

using namespace std;
using namespace cv;
int main()
{
	int keypoints_num= count_keypoints_num();

	
	cv::Mat img;
	cv::VideoCapture cap(0);
	string s="hello";
	std::vector<string> v;
	for(int i=0;i<10;i++)
		v.push_back(s);
	for(int i=0;i<10;++i)
		cout<<v[i]<<endl;
	// cv::namedWindow("test");
	while(cap.read(img))
	{
		cv::imshow("test",img);
		cv::waitKey(1);
		cout<<"hello,world!!"<<endl;
	}
	
	return 0;
}