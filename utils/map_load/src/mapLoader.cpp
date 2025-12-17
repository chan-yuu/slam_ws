/*
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2024-07-30 02:35:01
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2024-12-19 22:00:25
 * @FilePath: /undefined/home/cyun/Documents/panel_ws/src/map_load/src/mapLoader.cpp
 * @Description: 
 * 
 * Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
 */
#include "map_load/mapLoader.h"

MapLoader::MapLoader(ros::NodeHandle &nh){
    std::string pcd_file_path, map_topic;
    nh.param<std::string>("pcd_path", pcd_file_path, "");
    nh.param<std::string>("map_topic", map_topic, "point_map");

    pc_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>(map_topic, 10, true);
	//latched话题：你创建的发布者使用了 latching（持久化）机制 (advertise<sensor_msgs::PointCloud2>(map_topic, 10, true) 中的 true 参数）。Latching 意味着最后一次发布的信息会被保存下来，新的订阅者连接时会立即接收到这条消息。因此，每当一个新的订阅者（如 rostopic echo）连接到这个话题时，它都会立即接收到那条被 latched 的消息。

    file_list_.push_back(pcd_file_path);

    auto pc_msg = CreatePcd();
	std::cout << "pc_msg.width" << "\n" << pc_msg.width << std::endl;

    if (pc_msg.width != 0) {
		pc_msg.header.frame_id = "map";
		pc_map_pub_.publish(pc_msg);
	}
}


sensor_msgs::PointCloud2 MapLoader::CreatePcd()
{
	sensor_msgs::PointCloud2 pcd, part;
	for (const std::string& path : file_list_) {
		if (pcd.width == 0) {
			if (pcl::io::loadPCDFile(path.c_str(), pcd) == -1) {
				std::cerr << "load failed " << path << std::endl;
			}
		}
		else {
			if (pcl::io::loadPCDFile(path.c_str(), part) == -1) {
				std::cerr << "load failed " << path << std::endl;
			}
			pcd.width += part.width;
			pcd.row_step += part.row_step;
			pcd.data.insert(pcd.data.end(), part.data.begin(), part.data.end());
		}
		std::cerr << "load " << path << std::endl;
		if (!ros::ok()) break;
	}

	return pcd;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_loader");

    ROS_INFO("\033[1;32m---->\033[0m Map Loader Started.");

    ros::NodeHandle nh("~");

    MapLoader map_loader(nh);

    ros::spin();

    return 0;
}
