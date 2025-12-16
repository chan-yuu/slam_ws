#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from sklearn.linear_model import RANSACRegressor
import math

class LidarCalibrator:
    def __init__(self):
        rospy.init_node('lidar_ground_calibrator', anonymous=True)
        
        # 订阅雷达点云
        self.sub = rospy.Subscriber("/rslidar_points", pc2.PointCloud2, self.callback)
        
        # --- 配置 ROI (感兴趣区域) ---
        self.x_min, self.x_max = 3.0, 15.0   
        self.y_min, self.y_max = -3.0, 3.0   
        self.z_min, self.z_max = -3.0, -0.5  
        
        # --- 用于存储历史数据以计算平均值 ---
        self.pitch_history = []
        self.roll_history = []
        self.sample_count = 0
        
        print("正在等待点云数据，保持车辆静止...")

    def callback(self, msg):
        # 1. 解析点云
        points_gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = np.array(list(points_gen))

        if points.shape[0] < 100:
            return

        # 2. ROI 过滤
        mask_x = (points[:, 0] > self.x_min) & (points[:, 0] < self.x_max)
        mask_y = (points[:, 1] > self.y_min) & (points[:, 1] < self.y_max)
        mask_z = (points[:, 2] > self.z_min) & (points[:, 2] < self.z_max)
        
        mask = mask_x & mask_y & mask_z
        ground_points = points[mask]

        if ground_points.shape[0] < 500:
            rospy.logwarn_throttle(2, "地面点太少，无法拟合...")
            return

        # 3. RANSAC 平面拟合
        X_train = ground_points[:, :2] 
        y_train = ground_points[:, 2]  

        ransac = RANSACRegressor(residual_threshold=0.05) 
        ransac.fit(X_train, y_train)

        # 获取法向量 n = (a, b, -1)
        a = ransac.estimator_.coef_[0]
        b = ransac.estimator_.coef_[1]
        c = -1.0
        
        # 归一化
        norm_val = math.sqrt(a*a + b*b + c*c)
        nx, ny, nz = a/norm_val, b/norm_val, c/norm_val

        # 4. 计算瞬时欧拉角 (Roll, Pitch) - 弧度
        # Pitch (绕 Y 轴)
        curr_pitch_rad = math.atan2(nx, -nz) 
        
        # Roll (绕 X 轴)
        curr_roll_rad = math.atan2(ny, -nz) 
        
        # --- 5. 存储并计算平均值 ---
        self.pitch_history.append(curr_pitch_rad)
        self.roll_history.append(curr_roll_rad)
        self.sample_count += 1
        
        # 计算历史平均 (rad)
        avg_pitch_rad = np.mean(self.pitch_history)
        avg_roll_rad = np.mean(self.roll_history)

        # 转换为角度用于显示 (deg)
        # 注意：这里我们保留原始 rad 用于 config，转 deg 仅供人眼查看
        # 之前的逻辑中 Roll 取反了，这里保持一致用于显示
        avg_pitch_deg = math.degrees(avg_pitch_rad)
        avg_roll_deg = math.degrees(avg_roll_rad) 

        # --- 6. 格式化输出 ---
        # 清屏 (可选，让输出不刷屏)
        print("\033[H\033[J") 
        
        print("================================================")
        print(f"已采样帧数: {self.sample_count}")
        print(f"拟合平面法向量: ({nx:.3f}, {ny:.3f}, {nz:.3f})")
        print("-" * 30)
        print(f"【建议修正参数 (平均值)】 -> 填入 config.yaml")
        print(f"  Mount Pitch (rad):  {avg_pitch_rad:.6f}  (约 {avg_pitch_deg:.3f}°)")
        # 注意：这里根据你之前的习惯，Roll 显示时取了反，但填入 config 的 rad 通常直接用计算值即可
        # 如果你之前验证过需要取反，请使用 -avg_roll_rad
        print(f"  Mount Roll  (rad):  {avg_roll_rad:.6f}  (约 {-math.degrees(avg_roll_rad):.3f}° [显示取反])") 
        print("-" * 30)
        print(f"瞬时波动: P:{math.degrees(curr_pitch_rad):.2f}° | R:{-math.degrees(curr_roll_rad):.2f}°")

if __name__ == '__main__':
    try:
        LidarCalibrator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass