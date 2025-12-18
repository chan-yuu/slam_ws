# 功能包结构

```text
.
├── lio_sam # slam建图定位
│   ├── CMakeLists.txt
│   ├── config
│   ├── include
│   ├── install.md
│   ├── launch
│   ├── msg
│   ├── package.xml
│   ├── README.md
│   ├── src
│   └── srv
├── rslidar_sdk # lidar sdk ros1版本
│   ├── CHANGELOG.md
│   ├── CMakeLists.txt
│   └── src
├── sensor_pkg
│   ├── lidar_transform_pkg # 标定lidar与imu关系
│   └── gps_imu # gps imu数据
└── utils
│   ├── pcd2pgm # 2d 地图的构建
    └── map_load # 加载3d地图到rviz中
```