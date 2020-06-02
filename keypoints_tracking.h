#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
extern int count_keypoints_num();
const int keypoints_num=15;//在删减不必要的关键点之后，还剩下15个关键点
class Keypoints
{
public:
	Keypoints():X(keypoints_num),Y(keypoints_num),C(keypoints_num)
	{

	}
	std::vector<double> X;
	std::vector<double> Y;
	std::vector<double> C;
};
double calculate_OKS(Keypoints skeleton1,Keypoints skeleton2);
class Single_Skeleton
{
public:
	Single_Skeleton():skeleton(keypoints_num)
	{

	}
	cv::Scalar color;//颜色
	int id;//id号
	// Skeleton(int id,cv::Scalar color,坐标置信度);
	std::vector<cv::KalmanFilter>skeleton;
};
class Skeleton_Tracking  
{
public:
	
	std::vector<Single_Skeleton>people_skeletons;
	void skeletons_correct()
	{
		//使用已经匈牙利匹配完成的骨架来对历史骨架进行更新

	}
	void skeletons_predict();//使用kalman来根据历史骨架进行此刻的预测
	void skeletons_match();//使用OKS来进行匈牙利匹配
	void track();//跟踪！
	std::vector<bool> idTabel;
    void idTabelUpdate(int id);
	int idCreator();
	cv::Scalar colorCreator();
};