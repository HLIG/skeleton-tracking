#define _GLIBCXX_USE_CXX11_ABI 0

#include<iostream>
#include<unordered_map>
#include<vector>

using namespace std;
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