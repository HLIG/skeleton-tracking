/* 
Author:HLG
Date:2020年6月5日
huang.liguang@qq.com
*/
#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
extern int count_keypoints_num();

static const std::vector<cv::Scalar>color_list={
    cv::Scalar(0,255,0),
    cv::Scalar(0,0,255),
    cv::Scalar(255,0,0),
    cv::Scalar(255,255,0),
    cv::Scalar(0,255,255),
    cv::Scalar(255,0,255),
    cv::Scalar(255,255,255)
};
class Keypoints
{
public:
    Keypoints(const Keypoints & keypoints)
    {
        
        this->keypoints_num=keypoints.keypoints_num;
        this->X=keypoints.X;
        this->Y=keypoints.Y;
        this->C=keypoints.C;
        // for (int i = 0; i < keypoints_num; ++i)
        // {
        //     std::cout<<"C in copy:"<<this->C[i]<<std::endl;
        // }
        
        center_point_update();
    }
    Keypoints(int keypoints_num,std::vector<double>keypoints)
    {
        //根据关键点的数量和输入数据类型，将其进行转换 keypoints为数组（x,y,c）*关键点数目
        this->keypoints_num=keypoints_num;
        for(int i=0;i<keypoints_num;i++)
        {
            this->X.push_back(keypoints[3*i]);
            this->Y.push_back(keypoints[3*i+1]);
            this->C.push_back(keypoints[3*i+2]);
            // std::cout<<"C in constructor:"<<this->C[i]<<std::endl;

        }
        center_point_update();
    }
    cv::Point center_point_update()
    {
        //对骨架外界矩形进行更新
        this->center_point.x=(*std::min_element(X.begin(),X.end())+
                            *std::max_element(X.begin(),X.end()))/2;
        this->center_point.y=(*std::min_element(Y.begin(),Y.end())+
                            *std::max_element(Y.begin(),Y.end()))/2;
        return center_point;
    }
    int keypoints_num;
    std::vector<double> X;
    std::vector<double> Y;
    std::vector<double> C;
    cv::Point center_point;//外接矩形框中心
};

class Single_Skeleton
{
    //单人骨架跟踪类
public:
    int keypoints_num;//每个骨架的关键点数量
    cv::Scalar color;//颜色
    int id;//id号
    int track_frame;//成功跟踪的帧数
    double confidence;//置信度
    int confidence_inc;//连续匹配成功的帧数
    int confidence_dec;//连续匹配失败的帧数
    int confidence_linear_decrease_frame ;//置信度分段递减阈值
    // Skeleton(int id,cv::Scalar color,坐标置信度);
    Keypoints person_keypoints;//单人的跟踪关键点结果
    std::vector<cv::KalmanFilter>keypoints_kalmanfilter;//单人的所有关键点kalman跟踪
    
    //外接矩形的中心轨迹
    int max_trajectory_size=50;//最多保留历史50帧的中心轨迹
    std::vector<cv::Point> trajectory;//历史轨迹
    void add_trajectory(cv::Point point);//把点添加到轨迹中

    //构造函数
    Single_Skeleton(int id,int keypoints_num,Keypoints keypoints);
    ~Single_Skeleton(){}
    void confidenceIncrease()
    {
        confidence_inc++;
        confidence_dec = 0;
        confidence +=confidence_inc;
        // cout<<"in confidence:"<<confidence<<endl;
        // confidence += log(confidence_inc + 1) / log(1.5f);
    }

    bool confidenceDecrease()
    {
        confidence_dec++;
        confidence_inc = 0;
        if(confidence_dec<confidence_linear_decrease_frame)//分段递减
            confidence-=confidence_dec;
        else
        {
            confidence -= pow(2.0f, (confidence_dec-6));
        }
        // cout<<"de confidence:"<<confidence<<endl;
        if (confidence < 0)
        {
            confidence = 0;
            return false;
        }

        return true;
    }
    void person_predict();//对单人的卡尔曼结果进行预测
    void person_correct(Keypoints detected_keypoint);//对单人的卡尔曼结果进行更新
};
class Skeleton_Tracking  
{
public:
    Skeleton_Tracking(int keypoints_num,const std::vector<int>&bones)
    {
        idTabel.push_back(true);    //id 0 is always used
        this->keypoints_num=keypoints_num;
        this->bones=bones;
        bones_num=bones.size()/2;
    }
    std::vector<int>bones;//骨架连接
    int keypoints_num;
    int bones_num;
    std::vector<Single_Skeleton>people_skeletons;//包含了多个人
    
    void skeletons_track(std::vector<std::vector<double>>detected_skeletons);//跟踪！
    void draw_skeletons(cv::Mat img,double confidence_thres=0.05,bool draw_trajectory=false);
    std::vector<bool> idTabel;
    void idTabelUpdate(int id);
    int idCreator();
    cv::Scalar colorCreator();
};
