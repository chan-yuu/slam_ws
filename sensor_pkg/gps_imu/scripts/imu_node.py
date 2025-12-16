#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU 数据发布节点 (ROS 1 版本) - 带坐标系校正功能
发布话题：
  - /imu/data    (sensor_msgs/Imu): 原始IMU数据 (imu_link)
  - /imu/correct (sensor_msgs/Imu): 校正到车身坐标系的数据 (base_link)
"""

import rospy
import can
import cantools
import math
import os
import threading
import rospkg
import numpy as np
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler, euler_matrix, quaternion_multiply

class ImuNode:
    def __init__(self):
        rospy.init_node('imu_node', anonymous=True)

        # ------ 参数配置 (默认值) ------
        # IMU 相对于 base_link 的安装误差/变换
        # 旋转 (Roll, Pitch, Yaw) 单位: 弧度
        self.transform_r = rospy.get_param('~imu_mount_r', 0.0) 
        self.transform_p = rospy.get_param('~imu_mount_p', 0.0)
        self.transform_y = rospy.get_param('~imu_mount_y', -1.57)
        
        # 平移 (X, Y, Z) 单位: 米 (忽略)
        self.transform_x = rospy.get_param('~imu_mount_x', 0.0)
        self.transform_y_trans = rospy.get_param('~imu_mount_y', 0.0)
        self.transform_z = rospy.get_param('~imu_mount_z', 0.0)

        # ------ 计算旋转矩阵 (用于加速度和角速度变换) ------
        # euler_matrix 返回 4x4 矩阵，我们只需要左上角 3x3
        self.rot_matrix = euler_matrix(self.transform_r, self.transform_p, self.transform_y, 'sxyz')[:3, :3]
        rospy.loginfo(f"IMU 安装变换矩阵:\n{self.rot_matrix}")

        # ------ 发布者 ------
        self.pub_imu = rospy.Publisher('/imu/data', Imu, queue_size=10)
        self.pub_imu_correct = rospy.Publisher('/imu/correct', Imu, queue_size=10)

        # ------ 数据缓存 ------
        self.imu_data = {
            'heading': 0.0, 'pitch': 0.0, 'roll': 0.0,
            'ang_rate_x': 0.0, 'ang_rate_y': 0.0, 'ang_rate_z': 0.0,
            'accel_x': 0.0, 'accel_y': 0.0, 'accel_z': 0.0,
            'valid': False
        }

        # ------ 初始化 CAN + DBC ------
        self.can_bus = None
        self.db = None
        self.init_can()
        self.init_dbc()

        self.rate = rospy.Rate(100)

    def init_can(self):
        try:
            self.can_bus = can.Bus(channel='can2', interface='socketcan', bitrate=500000)
            rospy.loginfo('CAN(IMU)初始化成功: can2')
        except Exception as e:
            rospy.logerr(f'CAN(IMU)初始化失败: {e}')

    def init_dbc(self):
        try:
            rospack = rospkg.RosPack()
            # 注意：请确保包名正确，之前是 gps_imu，这里你给的是 gps_imu
            # 如果报错，请改回 gps_imu
            package_path = rospack.get_path('gps_imu') 
            dbc_path = os.path.join(package_path, 'config', 'CAN+Protocol+2.0_20210507-intel.dbc')
            
            if os.path.exists(dbc_path):
                self.db = cantools.database.load_file(dbc_path)
                rospy.loginfo(f'DBC(IMU) 加载成功: {dbc_path}')
                t = threading.Thread(target=self.loop)
                t.daemon = True
                t.start()
            else:
                rospy.logerr(f'❌ DBC 文件不存在: {dbc_path}')
        except Exception as e:
            rospy.logerr(f'DBC 解析失败: {e}')

    def loop(self):
        while not rospy.is_shutdown():
            try:
                msg = self.can_bus.recv(timeout=1.0)
                if msg:
                    can_id = msg.arbitration_id
                    if not self.db: continue
                    try:
                        decoded = self.db.decode_message(can_id, msg.data)
                        if can_id == 810:
                            self.imu_data['heading'] = decoded.get("AngleHeading", 0)
                            self.imu_data['pitch']  = decoded.get('AnglePitch', 0)
                            self.imu_data['roll']   = decoded.get('AngleRoll', 0)
                            self.imu_data['valid'] = True
                        elif can_id == 801:
                            self.imu_data['ang_rate_x'] = decoded.get('AngRateRawX', 0)
                            self.imu_data['ang_rate_y'] = decoded.get('AngRateRawY', 0)
                            self.imu_data['ang_rate_z'] = decoded.get('AngRateRawZ', 0)
                        elif can_id == 802:
                            self.imu_data['accel_x'] = decoded.get('AccelRawX', 0) * 9.8
                            self.imu_data['accel_y'] = decoded.get('AccelRawY', 0) * 9.8
                            self.imu_data['accel_z'] = decoded.get('AccelRawZ', 0) * 9.8
                    except Exception:
                        pass
            except Exception as e:
                pass

    def get_imu_msg(self):
        """构造基础 IMU 消息"""
        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        
        # 1. 原始角速度 (转弧度)
        wx = self.imu_data['ang_rate_x'] * math.pi/180.0
        wy = self.imu_data['ang_rate_y'] * math.pi/180.0
        wz = self.imu_data['ang_rate_z'] * math.pi/180.0
        msg.angular_velocity.x = wx
        msg.angular_velocity.y = wy
        msg.angular_velocity.z = wz

        # 2. 原始线加速度
        ax = self.imu_data['accel_x']
        ay = self.imu_data['accel_y']
        az = self.imu_data['accel_z']
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        # 3. 原始姿态 (欧拉角转四元数)
        r = math.radians(self.imu_data['roll'])
        p = math.radians(self.imu_data['pitch'])
        y = math.radians(self.imu_data['heading'])
        q = quaternion_from_euler(r, p, y) # 使用 tf 库的函数更稳定
        msg.orientation.x = q[0]
        msg.orientation.y = q[1]
        msg.orientation.z = q[2]
        msg.orientation.w = q[3]

        return msg

    def publish_imu(self):
        # --- 1. 发布原始数据 (/imu/data) ---
        imu_raw = self.get_imu_msg()
        imu_raw.header.frame_id = "imu_link"
        self.pub_imu.publish(imu_raw)

        # --- 2. 发布校正数据 (/imu/correct) ---
        imu_corr = Imu()
        imu_corr.header.stamp = imu_raw.header.stamp
        imu_corr.header.frame_id = "base_link" # 转换到了车身坐标系

        # A. 变换角速度 (Vector3 旋转)
        # [wx', wy', wz']^T = R * [wx, wy, wz]^T
        raw_w = np.array([imu_raw.angular_velocity.x, imu_raw.angular_velocity.y, imu_raw.angular_velocity.z])
        corr_w = np.dot(self.rot_matrix, raw_w)
        imu_corr.angular_velocity.x = corr_w[0]
        imu_corr.angular_velocity.y = corr_w[1]
        imu_corr.angular_velocity.z = corr_w[2]

        # B. 变换线加速度 (Vector3 旋转)
        # 注意：严格来说还要加上角加速度引起的切向加速度和向心加速度 (omega x (omega x r))
        # 但通常做简单的旋转对齐时忽略平移引起的向心力，直接旋转向量
        raw_a = np.array([imu_raw.linear_acceleration.x, imu_raw.linear_acceleration.y, imu_raw.linear_acceleration.z])
        corr_a = np.dot(self.rot_matrix, raw_a)
        imu_corr.linear_acceleration.x = corr_a[0]
        imu_corr.linear_acceleration.y = corr_a[1]
        imu_corr.linear_acceleration.z = corr_a[2]

        # C. 变换姿态 (四元数乘法)
        # q_correct = q_mount * q_raw
        # 注意 ROS tf 的 quaternion_multiply 顺序: q2 * q1 意味着先旋转 q1 再旋转 q2
        # 我们希望把传感器本身的读数 (q_raw) 再叠加上安装误差 (q_mount)
        q_raw = [imu_raw.orientation.x, imu_raw.orientation.y, imu_raw.orientation.z, imu_raw.orientation.w]
        q_mount = quaternion_from_euler(self.transform_r, self.transform_p, self.transform_y)
        
        # 旋转顺序视具体定义而定，通常是 q_mount * q_raw (父系转子系) 或者 q_raw * q_mount
        # 这里假设 imu_link 转到 base_link 的旋转是 q_mount
        q_final = quaternion_multiply(q_mount, q_raw)

        imu_corr.orientation.x = q_final[0]
        imu_corr.orientation.y = q_final[1]
        imu_corr.orientation.z = q_final[2]
        imu_corr.orientation.w = q_final[3]
        
        # 处理协方差 (可选，直接复制原始的或者放大)
        imu_corr.orientation_covariance = imu_raw.orientation_covariance
        imu_corr.angular_velocity_covariance = imu_raw.angular_velocity_covariance
        imu_corr.linear_acceleration_covariance = imu_raw.linear_acceleration_covariance

        self.pub_imu_correct.publish(imu_corr)

    def run(self):
        while not rospy.is_shutdown():
            self.publish_imu()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = ImuNode()
        node.run()
    except rospy.ROSInterruptException:
        pass