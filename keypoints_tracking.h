#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
extern int count_keypoints_num();

class Keypoints
{
public:
	Keypoints(int keypoints_num,std::vector<double>keypoints;)
	{
		//根据关键点的数量和输入数据类型，将其进行转换
		this->keypoints_num=keypoints_num;
		this->X.reserve(keypoints_num);
		this->Y.reserve(keypoints_num);
		this->C.reserve(keypoints_num);
	}
	int keypoints_num;
	std::vector<double> X;
	std::vector<double> Y;
	std::vector<double> C;
};
double calculate_OKS(Keypoints skeleton1,Keypoints skeleton2);
class Single_Skeleton
{
	//单人骨架跟踪类
public:
	Single_Skeleton(int keypoints_num)
	{
		this->keypoints_kalmanfilter.reserve(keypoints_num);
		this->keypoints_num=keypoints_num;
	}
	int keypoints_num;
	cv::Scalar color;//颜色
	int id;//id号
	// Skeleton(int id,cv::Scalar color,坐标置信度);
	Keypoints single_tracking_keypoints;//单人的跟踪关键点结果
	std::vector<cv::KalmanFilter>keypoints_kalmanfilter;//单人的所有关键点kalman跟踪
};
class Skeleton_Tracking  
{
public:
	Skeleton_Tracking(int keypoints_num)
	{
		this->keypoints_num=keypoints_num;
	}
	int keypoints_num;
	std::vector<Single_Skeleton>people_skeletons;//包含了多个人
	void skeletons_correct()
	{
		//使用已经匈牙利匹配完成的骨架来对历史骨架进行更新

	}
	void skeletons_predict();//使用kalman来根据历史骨架进行此刻的预测
	void skeletons_match();//使用OKS来进行匈牙利匹配
	void track(std::vector<std::vector<double>>detected_skeletons);//跟踪！
	std::vector<bool> idTabel;
    void idTabelUpdate(int id);
	int idCreator();
	cv::Scalar colorCreator();
};