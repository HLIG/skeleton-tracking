/* 
Author:HLG
Date:2020年6月5日
huang.liguang@qq.com
*/
#define _GLIBCXX_USE_CXX11_ABI 0
#include<iostream>
#include<unordered_map>
#include<munkres.h>
#include<adapters/adapter.h>
#include<adapters/std2dvectordapter.h>
#include<keypoints_tracking.h>
// using namespace std;
static bool isequal(double num,double target)
{
    if(abs(num-target)<0.000001)
        return true;
    else
        return false;
}
static double oks_max=DBL_MIN;
static double calculate_OKS(Keypoints skeleton1,Keypoints skeleton2)
{
    //计算oks，但由于是跟踪用，并没有真实值，因此对OKS进行了自定义，用另一种距离方式来计算两个骨架之间的距离，详情见小论文
    //当两个骨架越不相似时，自定义的oks越大 oks最大时为keypoints_num*keypoint_oks_max+box_oks_max=14×1.0+5.0=19.0
    int keypoints_num=skeleton1.keypoints_num;
    double distance_square_thres=30*30;//距离平方阈值，如果两个骨架的相同关键点欧式距离平方超过这个阈值，则oks+=1
    double confidence_thres=0.05;//置信度阈值，如果两个骨架的相同关键点中的某一个置信度小于这个阈值，则oks+=1
    double keypoint_oks_max=1.0;
    double box_square_thres=60*60;//矩形框中心距离阈值，如果超过这个阈值，则oks+=box_oks_max
    double box_oks_max=5.0;
    oks_max=keypoints_num*keypoint_oks_max+box_oks_max;
    double oks=0.0;
    for(int i=0;i<keypoints_num;i++)
    {
        if((skeleton1.C)[i]>confidence_thres&&(skeleton2.C)[i]>confidence_thres)
        {
            double euclidean_distance=((skeleton1.X)[i]-(skeleton2.X)[i])*((skeleton1.X)[i]-(skeleton2.X)[i])+
                                      ((skeleton1.Y)[i]-(skeleton2.Y)[i])*((skeleton1.Y)[i]-(skeleton2.Y)[i]);
            oks+=(keypoint_oks_max*std::min(euclidean_distance,distance_square_thres)/distance_square_thres);//即此时每个关键点距离最多使得oks增加keypoint_oks_max，在distance_square_thres取得keypoint_oks_max。距离越远，增加越快，要增加得更快可以使用三次方      
        }
        else
        {
            oks+=keypoint_oks_max;//有至少1个关键点置信度太低，直接oks+=keypoint_oks_max，
        }           
    }
    //使用二次函数，这样可以满足距离远的时候，oks增加地快一些，距离近的时候oks增加地慢一些
    double center_distance_square=(skeleton1.center_point.x-skeleton2.center_point.x)*(skeleton1.center_point.x-skeleton2.center_point.x)+
                           (skeleton1.center_point.y-skeleton2.center_point.y)*(skeleton1.center_point.y-skeleton2.center_point.y);
    oks+=(box_oks_max*std::min(center_distance_square,box_square_thres)/box_square_thres);//即此时矩形框中心最多使得oks增加box_oks_max，距离越远，增加越快，要增加得更快可以使用三次方
    
    return oks;
}
static void calculate_Distance_matrix(std::vector<Keypoints>detected_keypoints,std::vector<Keypoints>tracking_keypoints,std::vector<std::vector<double>>&Distance_Matrix,std::vector<std::vector<double>>&Distance_Matrix_Reverse)
{
    for(int row=0;row<Distance_Matrix.size();row++)
    {
        for(int col=0;col<Distance_Matrix[0].size();col++)
        {
            Distance_Matrix[row][col]=calculate_OKS(detected_keypoints[row],tracking_keypoints[col]);//oks越小,距离越大
            Distance_Matrix_Reverse[col][row]=Distance_Matrix[row][col];
        }
    }

}
void Distance_matrix_Filter(std::vector<std::vector<double>>&Distance_Matrix,std::vector<std::vector<double>>&Distance_Matrix_Reverse,
                            double threshold,std::vector<int>&choose_rows,std::vector<int>&choose_cols)
{


    if(Distance_Matrix.size()==0||Distance_Matrix[0].size()==0)
    {
        choose_cols.clear();
        choose_rows.clear();
    }
    else
    {
        for(int i=Distance_Matrix.size()-1;i>=0;--i)
        {
            if(*std::min_element(Distance_Matrix[i].begin(),Distance_Matrix[i].end())>threshold)
            {
                choose_rows.erase(choose_rows.begin()+i);
            }
        }
        for(int i=Distance_Matrix_Reverse.size()-1;i>=0;--i)
        {
            if(*std::min_element(Distance_Matrix_Reverse[i].begin(),Distance_Matrix_Reverse[i].end())>threshold)
            {
                choose_cols.erase(choose_cols.begin()+i);
            }
        }
    }
}
std::vector<std::vector<double>> get_filtered_Matrix(std::vector<std::vector<double>>Matrix_origin,std::vector<int>choose_rows,std::vector<int>choose_cols)
{
    int rows=0,cols=0;
    if(choose_rows.size()>0&&choose_cols.size()>0)
    {
        rows=choose_rows.size();
        cols=choose_cols.size();
    }
    std::vector<std::vector<double>>filtered_Matrix(rows,std::vector<double>(cols));
    for(int row=0;row<rows;row++)
    {
        for(int col=0;col<cols;col++)
        {
            filtered_Matrix[row][col]=Matrix_origin[choose_rows[row]][choose_cols[col]];
        }
    }
    return filtered_Matrix;
}
int count_keypoints_num(){
    //根据骨架点连线，来数一下有多少个关键点,顺便将关键点的序号进行重排，写来一开始调试用的。。后来并没有用上 
    const std::vector<int>bones={  1,2,   1,5,   2,3,   3,4,   5,6,   6,7,   2,9,   5,12,     9,10,    10,11,    12,13, 13,14,  1,0  };
    std::unordered_map<int,bool>m;
    for(int i=0;i<bones.size();++i)
    {
        if(m.find(bones[i])==m.end())
            m.insert(std::pair<int, bool>(bones[i], true));
    }

    //重排顺序
    std::vector<int>bones_resort;
    std::unordered_map<int,bool> ::iterator it;
    it=m.begin();
    while(it != m.end())
    {
        bones_resort.push_back(it->first);
        // cout<<"first->:"<<it->first<<endl;
        // cout<<"second->:"<<it->second<<endl;
        it++;
    } 
    sort(bones_resort.begin(),bones_resort.end());
    std::unordered_map<int,int>remap;
    for(int i=0;i<bones_resort.size();i++)
    {
        if(remap.find(bones_resort[i])==remap.end())
        {

            remap.insert(std::pair<int, int>(bones_resort[i], i));
            // printf("%d,%d,%d\n",bones_resort[i], i,remap[bones_resort[i]] );
        }
    }
   
    for(int i=0;i<bones.size();i++)
    {
        
        std::cout<<remap[bones[i]]<<',';
    }
    std::cout<<std::endl;


    std::cout<<"size"<<m.size()<<std::endl;
    return m.size();
    // cout<<"test in  keypoints_tracking.cpp"<<endl;
}
void Single_Skeleton::add_trajectory(cv::Point point)
{
    if(trajectory.size()>=max_trajectory_size)
    {
        trajectory.erase(trajectory.begin());
    }
    trajectory.push_back(point);
}
Single_Skeleton::Single_Skeleton(int id,int keypoints_num,Keypoints keypoints):person_keypoints(keypoints)
{
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
    //卡尔曼跟踪参数初始化
    for (int i = 0; i < keypoints_num; ++i)
    {
        this->keypoints_kalmanfilter.push_back(cv::KalmanFilter(4,2,0));//状态变量、观测变量、控制变量的维度
    }
    for(int i=0;i<keypoints_num;i++)
    {
        keypoints_kalmanfilter[i].statePre = cv::Mat::zeros(4, 1, CV_32F);
        keypoints_kalmanfilter[i].statePost = cv::Mat::zeros(4, 1, CV_32F);   //x
        keypoints_kalmanfilter[i].statePost.at<float>(0) = (person_keypoints.X)[i];
        keypoints_kalmanfilter[i].statePost.at<float>(1) = (person_keypoints.Y)[i];

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

 
    
}
void Single_Skeleton::person_predict()
{
    for(int i=0;i<this->keypoints_kalmanfilter.size();++i)
    {//对每个关键点进行卡尔曼预测
        this->keypoints_kalmanfilter[i].predict();     
        (person_keypoints.X)[i]=double(this->keypoints_kalmanfilter[i].statePost.at<float>(0));
        (person_keypoints.Y)[i]=double(this->keypoints_kalmanfilter[i].statePost.at<float>(1));
      
    } 
}

void Single_Skeleton::person_correct(Keypoints detected_keypoint)
{
    //匹配成功时，使用检测的关键点对跟踪关键点进行更新

    for(int i=0;i<keypoints_num;i++)
    {
        cv::Mat point_mat = (cv::Mat_<float>(2, 1) << (detected_keypoint.X)[i], (detected_keypoint.Y)[i]);
        keypoints_kalmanfilter[i].correct(point_mat);
        (person_keypoints.X)[i]=(detected_keypoint.X)[i];
        (person_keypoints.Y)[i]=(detected_keypoint.Y)[i];
    }
    add_trajectory(person_keypoints.center_point_update());//对骨架外界矩形进行更新
}

void Skeleton_Tracking::idTabelUpdate(int id)
{
    idTabel[id] = false;

    while (!idTabel[idTabel.size() - 1])
        idTabel.pop_back();
}

int Skeleton_Tracking::idCreator()
{
    unsigned int id = 0;
    while (idTabel[id])
    {
        id++;
        if (id == idTabel.size())
        {
            idTabel.push_back(true);
            break;
        }
    }
    idTabel[id] = true;
    
    return id;
}

void Skeleton_Tracking::skeletons_track(std::vector<std::vector<double>>detected_skeletons)
{
    const int detected_num=detected_skeletons.size();
    const int tracker_num=people_skeletons.size();
    for(int i=0;i<tracker_num;i++)
    {//对每个人进行卡尔曼预测
        this->people_skeletons[i].person_predict();
    }

    
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
        tracking_keypoints.push_back(people_skeletons[i].person_keypoints);
    }
    //距离矩阵计算及滤波
    const int rows=detected_num;
    const int cols=tracker_num;
    static std::vector<std::vector<double>>Distance_Matrix(rows,std::vector<double>(cols));//骨架距离矩阵
    static std::vector<std::vector<double>>Distance_Matrix_reverse(cols,std::vector<double>(rows));//骨架距离矩阵转置，以空间换时间
    calculate_Distance_matrix(detected_keypoints,tracking_keypoints,Distance_Matrix,Distance_Matrix_reverse);
    //首先根据距离来对矩形框进行滤波,如果骨架距离超过阈值，需要另外计算，如果在检测结果中，则当作新检测目标加入，如果在跟踪列表中，则当作离开的目标进行去除
    std::vector<int>choose_rows,choose_cols;
    const double distance_threshold=oks_max*4.0/10.0;//距离超过distance_threshold的将其另外考虑
    for(int i=0;i<rows;i++)//先把所有检测结果的序号都放到choose_rows中，后续再进行剔除
        choose_rows.push_back(i);
    for(int i=0;i<cols;i++)//先把所有跟踪目标的序号都放到choose_cols中，后续再进行剔除
        choose_cols.push_back(i);
    Distance_matrix_Filter(Distance_Matrix,Distance_Matrix_reverse,distance_threshold,
                           choose_rows,choose_cols);
    std::vector<std::vector<double>>filtered_distanceMatrix;//得到剔除之后的矩阵
    filtered_distanceMatrix=get_filtered_Matrix(Distance_Matrix,choose_rows,choose_cols);
    //匈牙利匹配
    std::vector<std::vector<double>>match_result_Matrix;
    std::vector<std::pair<int,int>>pairs;//匹配成功的对
    std::set<int>rows_set;//匹配成功的行
    std::set<int>cols_set;//匹配成功的列
    if(filtered_distanceMatrix.size()>0)
    {
        Std2dVectorAdapter<double>std2matrix;
        Matrix<double> matrix=std2matrix.convertToMatrix(filtered_distanceMatrix);
        Munkres<double> m;
        m.solve(matrix);
        std2matrix.convertFromMatrix(match_result_Matrix,matrix);

        //匹配之后需要找出匹配成功的对
        for(int row=0;row<match_result_Matrix.size();++row)
        {
            for(int col=0;col<match_result_Matrix[0].size();++col)
            {
                if(isequal(match_result_Matrix[row][col],0.0))//结果矩阵中=0的代表匹配成功
                {
                    rows_set.insert(choose_rows[row]);
                    cols_set.insert(choose_cols[col]);
                    pairs.push_back({choose_rows[row],choose_cols[col]});
                    break;
                }
            }
        }
    }

    //对于匹配成功的乘客骨架,需要更新其坐标,以及提升其置信度  
    for(auto &iter:pairs)
    {
        this->people_skeletons[iter.second].track_frame++;//跟踪帧数增加
        this->people_skeletons[iter.second].confidenceIncrease();
        this->people_skeletons[iter.second].person_correct(detected_keypoints[iter.first]);//对卡尔曼滤波的坐标进行更新
    }
    //对于距离太远或者匈牙利匹配失败的跟踪目标,需要降低其置信度,置信度低于一定值,将其从后往前删除
    for (std::vector<Single_Skeleton>::iterator k = people_skeletons.end()-1; k != people_skeletons.begin()-1; k--)
    {
        if(cols_set.find(std::distance(people_skeletons.begin(),k))!=cols_set.end())//如果匈牙利匹配成功
            continue;
        else if (!(*k).confidenceDecrease())//置信度减到0以下,剔除该目标
        {
            idTabelUpdate((*k).id);
            people_skeletons.erase(k);
        }
        else//置信度没减到0以下,用上次的跟踪结果更新卡尔曼滤波器
        {
            (*k).track_frame++;//跟踪帧数增加
            (*k).person_correct((*k).person_keypoints);// 使用预测结果来进行更新
        }

    }

    // //对于距离太远的乘客骨架,将其作为新目标,加入到跟踪列表中
    for(int i=0;i<detected_num;i++)
    {   
        // std::cout<<"detected_num:"<<detected_num<<std::endl;
        #ifdef DEBUG
        cout<<"产生新目标"<<endl;
        #endif
        if(rows_set.find(i)==rows_set.end())//没有匈牙利匹配成功的目标
        {

            // std::cout<<id<<" "<<this->keypoints_num<<std::endl;
            // printf("%d,%d\n",id, this->keypoints_num);
            people_skeletons.push_back(Single_Skeleton(idCreator(),this->keypoints_num, detected_keypoints[i]));
        }

    }
}
void Skeleton_Tracking::draw_skeletons(cv::Mat img,double confidence_thres,bool draw_trajectory)
{//画出置信度超过阈值的关键点所在的骨架
    int people_num=people_skeletons.size();
    for(int i=0;i<people_num;++i) {
        for(int j=0;j<bones_num;++j)
        {
            const int index1=bones[j*2];
            const int index2=bones[j*2+1];
            
            const double confidence1=((people_skeletons[i].person_keypoints).C)[index1];//第i个人的第index1个关键点的置信度
            const double confidence2=((people_skeletons[i].person_keypoints).C)[index2];//第i个人的第index2个关键点的置信度
            // std::cout<<confidence1<<" "<<confidence2<<std::endl;
            if(std::min(confidence1,confidence2)>confidence_thres)
            {
                const double point1_x=((people_skeletons[i].person_keypoints).X)[index1];
                const double point1_y=((people_skeletons[i].person_keypoints).Y)[index1];
                const double point2_x=((people_skeletons[i].person_keypoints).X)[index2];
                const double point2_y=((people_skeletons[i].person_keypoints).Y)[index2];
                const cv::Point point1(point1_x,point1_y);                       
                const cv::Point point2(point2_x,point2_y);
                cv::line(img,point1,point2, people_skeletons[i].color, 2 );
            }
            
            // cv::circle(dst,cv::Point(result[i*3],result[i*3+1]),2,cv::Scalar(0,255,0),-1);
            // putText(dst, to_string(i), cv::Point(result[i*3],result[i*3+1]), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 1);
        }
        //如果需要画出行人中心轨迹
        if(draw_trajectory)
        {
            if(people_skeletons[i].trajectory.size()>0)
            {
                cv::Scalar color=people_skeletons[i].color;
                cv::circle(img,people_skeletons[i].trajectory.back(),1,people_skeletons[i].color,-1);
                for(int k=people_skeletons[i].trajectory.size()-1;k>0;--k)
                {
                    cv::Point point1=people_skeletons[i].trajectory[k];
                    cv::Point point2=people_skeletons[i].trajectory[k-1];
                    cv::line(img,point1,point2,color,1);
                }
            }
        }
    }
    
}
