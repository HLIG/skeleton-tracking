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
    Keypoints(int keypoints_num,std::vector<double>keypoints)
    {
        //根据关键点的数量和输入数据类型，将其进行转换 keypoints为数组（x,y,c）*关键点数目
        this->keypoints_num=keypoints_num;
        for(int i=0;i<keypoints_num;i++)
        {
            this->X.push_back(keypoints[i]);
            this->Y.push_back(keypoints[i+1]);
            this->C.push_back(keypoints[i+2]);

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
    Keypoints *person_keypoints;//单人的跟踪关键点结果
    std::vector<cv::KalmanFilter>keypoints_kalmanfilter;//单人的所有关键点kalman跟踪
    
    //中心轨迹
    int max_trajectory_size=50;//最多保留历史50帧的中心轨迹
    std::vector<cv::Point> trajectory;//历史轨迹
    void add_trajectory(cv::Point point);//把点添加到轨迹中
    Single_Skeleton(int id,int keypoints_num,Keypoints keypoints)
    {
        static Keypoints keypoints_static(keypoints);
        this->keypoints_num=keypoints_num;
        this->id=id;
        this->color=color_list[id%color_list.size()];
        // this->max_trajectory_size=50;
        //置信度参数初始化
        confidence = 15;//初始置信度给15
        confidence_inc = 0;
        confidence_dec = 0;
        confidence_linear_decrease_frame=10;
        //关键点坐标置信度初始化
        person_keypoints=&keypoints_static;
        //卡尔曼跟踪参数初始化
        for (int i = 0; i < keypoints_num; ++i)
        {
            this->keypoints_kalmanfilter.push_back(cv::KalmanFilter(4,2,0));//状态变量、观测变量、控制变量的维度
        }
        
        for(int i=0;i<keypoints_num;i++)
        {
            keypoints_kalmanfilter[i].statePre = cv::Mat::zeros(4, 1, CV_32F);
            keypoints_kalmanfilter[i].statePost = cv::Mat::zeros(4, 1, CV_32F);   //x
            keypoints_kalmanfilter[i].statePost.at<float>(0) = (person_keypoints->X)[i];
            keypoints_kalmanfilter[i].statePost.at<float>(1) = (person_keypoints->Y)[i];

        //    transitionMatrix = *(Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);//opencv2
            keypoints_kalmanfilter[i].transitionMatrix=(cv::Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);//opencv4
            keypoints_kalmanfilter[i].controlMatrix.release();    //B
            keypoints_kalmanfilter[i].measurementMatrix = cv::Mat::zeros(2, 4, CV_32F);   //H
            setIdentity(keypoints_kalmanfilter[i].measurementMatrix);
            keypoints_kalmanfilter[i].gain = cv::Mat::zeros(4, 2, CV_32F);    //K

            keypoints_kalmanfilter[i].errorCovPre = cv::Mat::zeros(4, 4, CV_32F);
            keypoints_kalmanfilter[i].errorCovPost = cv::Mat::zeros(4, 4, CV_32F);    //P
            setIdentity(keypoints_kalmanfilter[i].errorCovPost, cv::Scalar::all(1));
            keypoints_kalmanfilter[i].processNoiseCov = cv::Mat::eye(4, 4, CV_32F);   //Q
            setIdentity(keypoints_kalmanfilter[i].processNoiseCov, cv::Scalar::all(1e-3));
            keypoints_kalmanfilter[i].measurementNoiseCov = cv::Mat::eye(2, 2, CV_32F);   //R
            setIdentity(keypoints_kalmanfilter[i].measurementNoiseCov, cv::Scalar::all(1e-6));

            keypoints_kalmanfilter[i].temp1.create(4, 4, CV_32F);
            keypoints_kalmanfilter[i].temp2.create(2, 4, CV_32F);
            keypoints_kalmanfilter[i].temp3.create(2, 2, CV_32F);
            keypoints_kalmanfilter[i].temp4.create(2, 4, CV_32F);
            keypoints_kalmanfilter[i].temp5.create(2, 1, CV_32F);
        }

        
        
        // 
        
    }
    ~Single_Skeleton()
    {
        delete person_keypoints;
    }
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
    Skeleton_Tracking(int keypoints_num)
    {
        this->keypoints_num=keypoints_num;
        bones_num=bones.size()/2;
    }
    const std::vector<int>bones={1,2, 1,5, 2,3, 3,4, 5,6, 6,7, 2,8, 5,11, 8,9, 9,10, 11,12, 12,13, 1,0};//骨架连接
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
