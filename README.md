# long_term_slam

This repo is about the code dof Silin Li's Master Thesis.

[TOC]

# 1 Dependencies
- System: ubuntu 16.04
- C++ version: c++17
- g++/gcc >= 7.0
    - [How to upgrade your g++ and gcc?](https://www.zybuluo.com/iStarLee/note/1260368)
- cmake >= 3.10
- clang-format-6.0
```bash
sudo apt-get install clang-format-6.0
```
- Third Parties
    - glog
    - gflag
    ```bash
    git clone xx_gfalgs
    mkdir build 
    cd build
    export CXXFLAGS="-fPIC" && cmake .. -DBUILD_SHARED_LIBS=ON && make VERBOSE=1
    sudo make install
    ```
    - gtest[future]
    - ros-kinetic
    - eigen 3.2.92
    - opencv 3.3.1
    - pcl 1.7
    - Boost 1.58
    - absl(compiled with c++17 settings)
    ```bash
    # 1. add `set(CMAKE_CXX_STANDARD 17)` to CMakeLists.txt of abseil-cpp
    # 2. compile 
    mkdir build
    cd build
    cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
    make
    sudo make install
    ```
    - libnabo(https://github.com/ethz-asl/libnabo)
    ```bash
    git clone https://github.com/ethz-asl/libnabo
    cd libnabo
    mkdir build
    cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX=/usr ..
    make
    sudo make install
    ```
    
# 2 Long Term Localization 数据处理流程
## 2.1 语义点云处理
- 跑lio_sam 保存keyframe, pose, timestamp
    - 参数调整注意
    `src/third_parties/lio_sam/config/params.yaml` 中`savePCD`打开；`savePCDDirectory`设置保存位置；`surroundingkeyframeAddingDistThreshold`设置关键帧之间的距离。以`savePCDDirectory = ~/offline_process/`为例。
    - 在三个终端内先后执行以下命令
    ```bash
    # terminal1
    roslaunch lio_sam run.launch #会自动生成savePCDDirectory文件夹

    # terminal2
    ./sh/record.sh
    
    # terminal3
    rosbag play xx.bag --clock # 一定要带clock
    ```
注意在lio_sam回环之前关闭，因为回环会造成位姿误差，导致建图有重影，这个问题未来可以考虑使用`src/third_parties/interactive_slam`来解决

- 离线后处理
    ```bash
    ./sh/offline_process.sh
    ```
    至此我们的semantic.bag中有了以下topic:
    - /lio_sam/mapping/odometry 用于长航时定位和NDT定位
    - /navgps 用于定位的evaluation
    - /navodom 保留
    - /imu 用于NDT定位预测
    - /odom 用于NDT定位预测

- 使用`pcl_viewer`打开生成的全局点云
```bash
pcl_viewer cloudGlobal.pcd
```
如果发现打不开，报错如下
```
> Loading cloudGlobal.pcd [pcl::PCDReader::read] Number of points read (2801499) is different than expected (16759678)
```
采用如下[方法](http://www.pcl-users.org/Can-t-read-pcd-file-td4044649.html)解决:
使用编辑器打开该pcd文件，修改`POINTS`为终端提示的2801499，如下所示
```
# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH 2801499
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 2801499
DATA ascii
```
## 2.2 Evalution
- 评估轨迹
```
cd PATH/long_term_slam/src/long_term_slam/data

evo_traj tum localization.txt --ref=gt.txt -p --plot_mode=xy --align
```
- 评估误差
```
evo_ape tum ground_true.txt proposed.txt -a -p --plot_mode=xy
```
- 评估重定位距离
P99
- 评估全局聚类数量（x）与重定位距离（y）
    - 路径周围的聚类数量，
- 聚类缺失到什么程度，重定位会失败（20m之内无法重定位）
- 重定位的得分
    - 聚类匹配数量
    - 参与匹配数量
    - 匹配得到的距离
    - 匹配中聚类物体的被观测次数
