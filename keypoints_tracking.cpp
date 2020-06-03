#define _GLIBCXX_USE_CXX11_ABI 0

#include<iostream>
#include<unordered_map>
#include<vector>
#include<keypoints_tracking.h>
using namespace std;



double calculate_OKS(Keypoints skeleton1,Keypoints skeleton2)
{
	//计算oks，但由于是跟踪用，并没有真实值，因此对OKS进行了自定义，用另一种距离方式来计算两个骨架之间的距离，详情见小论文
	return 1.0;
}
int count_keypoints_num(){
	//根据骨架点连线，来数一下有多少个关键点
	const std::vector<int>bones={1,8,   1,2,   1,5,   2,3,   3,4,   5,6,   6,7,   8,9,   9,10,  10,11, 8,12,  12,13, 13,14,  1,0,   0,15, 15,17,  0,16, 16,18,   14,19,19,20,14,21, 11,22,22,23,11,24};
	std::unordered_map<int,bool>m;
	for(int i=0;i<bones.size();++i)
	{
		if(m.find(bones[i])==m.end())
			m.insert(pair<int, bool>(bones[i], true));
	}
	cout<<"size"<<m.size()<<endl;
	return m.size();
	// cout<<"test in  keypoints_tracking.cpp"<<endl;
}
void Skeleton_Tracking::skeletons_predict()
{
	const int people_num=people_skeletons.size();
//使用kalman滤波来对关键点坐标进行预测
	for(int i=0;i<people_num;i++)
	{
		static int keypoints_num=people_skeletons[0].keypoints_num;
		for(int j=0;j<keypoints_num;j++)
		{
			(people_skeletons[i].keypoints_kalmanfilter)[j].predict();
			//还得对类内的关键点进行更新
		}
        
	}
}
void Skeleton_Tracking::track(std::vector<std::vector<double>>detected_skeletons)
{
	skeletons_predict();
	std::vector<Keypoints> detected_keypoints;
	for(int i=0;i<detected_skeletons.size();i++)
	{
		//将二维vector变成Keypoints类型，方便后面求取oks
		detected_keypoints.push_back(Keypoints(this->keypoints_num,detected_skeletons[i]));
	}


}