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
const static int keypoints_num=14;//在删减不必要的关键点之后，还剩下14个关键点
int main()
{
	int keypoints_num= count_keypoints_num();
	cout<<"keypoints_num"<<keypoints_num<<endl;
	//Keypoints Keypoints1(keypoints_num);
	//cout<<"Keypoints1.size()"<<Keypoints1.X.size()<<endl;
	while(true);
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
