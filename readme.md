## 写在前面

本项目使用卡尔曼滤波和匈牙利匹配来对关键点进行跟踪

## 需要依赖库
```
OpenCV4.2(其他版本如果报错，请自行对相应函数做调整)
```

## 安装&运行

```
cd keypoints_tracking
mkdir build
cd build
cmake ..
make
./keypoints_tracking
```

## 可能出现的问题
1.如果电脑的OpenCV路径找不到的话，可以在CMakeLists.txt中自己指定路径，如果找得到，就不需要下面这一行
```
set( OpenCV_DIR Your_OpenCV_DIR)
```

2.如果电脑的gcc版本比opencv编译的版本低，则需要使用这一行来避免string编译出错，gcc版本与opencv版本一致，将其注释即可
```
define _GLIBCXX_USE_CXX11_ABI 0
```
