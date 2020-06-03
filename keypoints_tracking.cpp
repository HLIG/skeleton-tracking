#define _GLIBCXX_USE_CXX11_ABI 0

#include<iostream>
#include<unordered_map>
#include<vector>
#include<keypoints_tracking.h>
using namespace std;


void calculate_Distance_matrix(std::vector<Keypoints>detected_keypoints,std::vector<Keypoints>tracking_keypoints,vector<vector<double>>&Distance_Matrix,vector<vector<double>>&Distance_Matrix_Reverse)
{
    for(int row=0;row<Distance_Matrix.size();row++)
    {
        for(int col=0;col<Distance_Matrix[0].size();col++)
        {
            Distance_Matrix[row][col]=1-calculate_OKS(detected_keypoints[row],tracking_keypoints[col]);//oks越小,距离越大
            Distance_Matrix_Reverse[col][row]=Distance_Matrix[row][col];
        }
    }

}
double calculate_OKS(Keypoints skeleton1,Keypoints skeleton2)
{
	//计算oks，但由于是跟踪用，并没有真实值，因此对OKS进行了自定义，用另一种距离方式来计算两个骨架之间的距离，详情见小论文
	return 1.0;
}
int count_keypoints_num(){
	//根据骨架点连线，来数一下有多少个关键点
	const std::vector<int>bones={  1,2,   1,5,   2,3,   3,4,   5,6,   6,7,   2,9,   5,12,     9,10,    10,11,    12,13, 13,14,  1,0  };
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
void Single_Skeleton::person_predict()
{
    for(int i=0;i<this->keypoints_kalmanfilter.size();++i)
    {//对每个关键点进行卡尔曼预测
        this->keypoints_kalmanfilter[i].predict();
        (person_keypoints->X)[i]=int(this->keypoints_kalmanfilter[i].statePost.at<float>(0));
        (person_keypoints->Y)[i]=int(this->keypoints_kalmanfilter[i].statePost.at<float>(1));
    }
    
}
void Skeleton_Tracking::skeletons_predict()
{
	const int people_num=people_skeletons.size();
//使用kalman滤波来对关键点坐标进行预测
	for(int i=0;i<people_num;i++)
	{//对每个人进行卡尔曼预测
		people_skeletons[i].person_predict();
	}
}
void Skeleton_Tracking::track(std::vector<std::vector<double>>detected_skeletons)
{
	skeletons_predict();//卡尔曼预测
	const int detected_num=detected_skeletons.size();
	const int tracker_num=people_skeletons.size();
	std::vector<Keypoints> detected_keypoints;
	std::vector<Keypoints>tracking_keypoints;
	for(int i=0;i<detected_num;i++)
	{
		//将二维vector变成Keypoints类型，方便后面求取oks
		detected_keypoints.push_back(Keypoints(this->keypoints_num,detected_skeletons[i]));
	}
	for(int i=0;i<tracker_num;i++)
	{
		//将二维vector变成Keypoints类型，方便后面求取oks
		tracking_keypoints.push_back(*(people_skeletons[i].person_keypoints));
	}
    //匈牙利匹配
    const int rows=detected_num;
    const int cols=tracker_num;
    static std::vector<std::vector<double>>Distance_Matrix(rows,std::vector<double>(cols));
    static std::vector<std::vector<double>>Distance_Matrix_reverse(cols,std::vector<double>(rows));//转置矩阵
    

}
