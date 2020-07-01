# map_transmission
地图传输（ROS/PYTHON3）

## 1、ROS : 话题通信 (Python)
https://blog.csdn.net/betterzl/article/details/105835403

## 2、ROS 编译 Python 文件（利用 Python 解析 bag 文件）
https://blog.csdn.net/betterzl/article/details/105965504

## 3、Python 调用 OccupancyGrid 处理栅格地图（一）
https://blog.csdn.net/betterzl/article/details/106070071

## 4、Python 调用 OccupancyGrid 处理栅格地图（二）
https://blog.csdn.net/betterzl/article/details/106122621

## 5、部署

    # 打开新终端
    ssh ***@***
    roslaunch mybot bringup.launch
    
    # 打开新终端
    ssh ***@***
    roslaunch mybot lidar_slam.launch 
    
    # 打开新终端
    ssh ***@***
    roscd commu2app/src/
    ./Map_Reducing.py 
    
    # 打开新终端
    ssh ***@***
    rostopic list
    rostopic echo /map_reducing
    rostopic echo /map_reducing >map_reducing.txt
    vim map_reducing.txt 
