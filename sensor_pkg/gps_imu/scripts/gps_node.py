#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GPS 数据发布节点 (ROS 1 版本)
发布话题：
  - /gps/fix      (sensor_msgs/NavSatFix): 经纬度、海拔、状态
  - /gps/vel      (geometry_msgs/TwistStamped): 东向/北向/天向速度
  - /gps/imu      (sensor_msgs/Imu): GPS内部INS输出的姿态、角速度、加速度
"""

import rospy
import can
import cantools
import threading
import os
import math
import rospkg
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from geometry_msgs.msg import TwistStamped

class GpsNode:
    def __init__(self):
        rospy.init_node('gps_node', anonymous=True)

        # ------ 发布者 (使用标准消息) ------
        self.pub_fix = rospy.Publisher('/gps/fix', NavSatFix, queue_size=10)
        self.pub_vel = rospy.Publisher('/gps/vel', TwistStamped, queue_size=10)
        # self.pub_imu = rospy.Publisher('/gps/imu', Imu, queue_size=10)

        # ------ 数据缓存 ------
        self.gps_data = {
            'latitude': 0.0, 'longitude': 0.0, 'altitude': 0.0,
            'eastvelocity': 0.0, 'northvelocity': 0.0, 'skyvelocity': 0.0,
            'heading': 0.0, 'pitch': 0.0, 'roll': 0.0,
            'ang_rate_x': 0.0, 'ang_rate_y': 0.0, 'ang_rate_z': 0.0,
            'accel_x': 0.0, 'accel_y': 0.0, 'accel_z': 0.0,
            'valid': False
        }

        # ------ 初始化 CAN + DBC ------
        self.can_bus = None
        self.db = None
        self.init_can_interface()
        self.init_dbc_parser()

        # ------ 设置循环频率 (100Hz) ------
        self.rate = rospy.Rate(100) 

    def init_can_interface(self):
        try:
            self.can_bus = can.Bus(channel='can2', interface='socketcan', bitrate=500000)
            rospy.loginfo('CAN初始化成功: can2')
        except Exception as e:
            rospy.logerr(f'CAN初始化失败: {e}')

    def init_dbc_parser(self):
        try:
            # ROS 1 使用 rospkg 获取包路径
            rospack = rospkg.RosPack()
            package_path = rospack.get_path('gps_imu')
            dbc_path = os.path.join(package_path, 'config', 'CAN+Protocol+2.0_20210507-intel.dbc')
            
            if os.path.exists(dbc_path):
                self.db = cantools.database.load_file(dbc_path)
                rospy.loginfo(f'DBC 文件加载成功: {dbc_path}')
                
                # 开启守护线程接收 CAN 数据
                t = threading.Thread(target=self.can_receive_loop)
                t.daemon = True
                t.start()
            else:
                rospy.logerr(f'❌ DBC 文件不存在: {dbc_path}')
        except Exception as e:
            rospy.logerr(f'DBC 解析失败: {e}')

    def can_receive_loop(self):
        while not rospy.is_shutdown():
            try:
                msg = self.can_bus.recv(timeout=1.0)
                if msg:
                    self.process_can_message(msg)
            except Exception as e:
                rospy.logwarn(f"CAN Receive Error: {e}")

    def process_can_message(self, message):
        if not self.db:
            return

        can_id = message.arbitration_id
        try:
            decoded = self.db.decode_message(can_id, message.data)

            if can_id == 814:
                self.gps_data['latitude'] = decoded.get('PosLat2', 0.0)
                self.gps_data['valid'] = True

            elif can_id == 813:
                self.gps_data['longitude'] = decoded.get('PosLon2', 0.0)
                self.gps_data['valid'] = True

            elif can_id == 805:
                self.gps_data['altitude'] = decoded.get('PosAlt', 0.0)
                self.gps_data['valid'] = True

            elif can_id == 807:
                self.gps_data['eastvelocity'] = decoded.get('VelE', 0.0)
                self.gps_data['northvelocity'] = decoded.get('VelN', 0.0)
                self.gps_data['skyvelocity'] = decoded.get('VelU', 0.0)
                self.gps_data['valid'] = True

            elif can_id == 810:
                # 坐标系转换: 假设原始数据是北偏东，转为 ROS ENU (East=0, North=90)
                heading = 90.0 - decoded.get("AngleHeading", 0.0)
                if heading > 180: heading -= 360
                if heading < -180: heading += 360

                self.gps_data['heading'] = heading
                self.gps_data['pitch'] = decoded.get('AnglePitch', 0.0)
                self.gps_data['roll']  = decoded.get('AngleRoll', 0.0)
                self.gps_data['valid'] = True

            elif can_id == 812:
                self.gps_data['ang_rate_x'] = decoded.get('AngRateX', 0.0)
                self.gps_data['ang_rate_y'] = decoded.get('AngRateY', 0.0)
                self.gps_data['ang_rate_z'] = decoded.get('AngRateZ', 0.0)

            elif can_id == 809:
                # 假设 DBC 中单位是 g，这里转换为 m/s^2
                self.gps_data['accel_x'] = decoded.get('AccelX', 0.0) * 9.8
                self.gps_data['accel_y'] = decoded.get('AccelY', 0.0) * 9.8
                self.gps_data['accel_z'] = decoded.get('AccelZ', 0.0) * 9.8

        except Exception:
            pass

    def euler_to_quaternion(self, r, p, y):
        # 输入为弧度
        qx = math.sin(r/2)*math.cos(p/2)*math.cos(y/2)-math.cos(r/2)*math.sin(p/2)*math.sin(y/2)
        qy = math.cos(r/2)*math.sin(p/2)*math.cos(y/2)+math.sin(r/2)*math.cos(p/2)*math.sin(y/2)
        qz = math.cos(r/2)*math.cos(p/2)*math.sin(y/2)-math.sin(r/2)*math.sin(p/2)*math.cos(y/2)
        qw = math.cos(r/2)*math.cos(p/2)*math.cos(y/2)+math.sin(r/2)*math.sin(p/2)*math.sin(y/2)
        return qx, qy, qz, qw

    def publish_gps(self):
        current_time = rospy.Time.now()

        # 1. 发布 NavSatFix (位置)
        fix_msg = NavSatFix()
        fix_msg.header.stamp = current_time
        fix_msg.header.frame_id = "gps_link"
        
        if self.gps_data['valid']:
            fix_msg.status.status = NavSatStatus.STATUS_FIX
        else:
            fix_msg.status.status = NavSatStatus.STATUS_NO_FIX
            
        fix_msg.status.service = NavSatStatus.SERVICE_GPS
        fix_msg.latitude = self.gps_data['latitude']
        fix_msg.longitude = self.gps_data['longitude']
        fix_msg.altitude = self.gps_data['altitude']
        # 如果不知道协方差，可设为 0 或 -1 (unknown)
        fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.pub_fix.publish(fix_msg)

        # 2. 发布 TwistStamped (速度 - ENU系)
        vel_msg = TwistStamped()
        vel_msg.header.stamp = current_time
        vel_msg.header.frame_id = "gps_link" # 速度通常相对于世界系或BaseLink，根据实际情况调整
        vel_msg.twist.linear.x = self.gps_data['eastvelocity']
        vel_msg.twist.linear.y = self.gps_data['northvelocity']
        vel_msg.twist.linear.z = self.gps_data['skyvelocity']
        self.pub_vel.publish(vel_msg)

        # 3. 发布 Imu (GPS内部INS数据)
        imu_msg = Imu()
        imu_msg.header.stamp = current_time
        imu_msg.header.frame_id = "gps_link"
        
        # 角度转四元数 (标准 IMU 消息使用四元数)
        r = math.radians(self.gps_data['roll'])
        p = math.radians(self.gps_data['pitch'])
        y = math.radians(self.gps_data['heading'])
        qx, qy, qz, qw = self.euler_to_quaternion(r, p, y)
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw

        # 角速度 (度/秒 -> 弧度/秒)
        imu_msg.angular_velocity.x = self.gps_data['ang_rate_x'] * math.pi / 180.0
        imu_msg.angular_velocity.y = self.gps_data['ang_rate_y'] * math.pi / 180.0
        imu_msg.angular_velocity.z = self.gps_data['ang_rate_z'] * math.pi / 180.0

        # 线加速度 (m/s^2)
        imu_msg.linear_acceleration.x = self.gps_data['accel_x']
        imu_msg.linear_acceleration.y = self.gps_data['accel_y']
        imu_msg.linear_acceleration.z = self.gps_data['accel_z']

        # self.pub_imu.publish(imu_msg)

    def run(self):
        while not rospy.is_shutdown():
            self.publish_gps()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = GpsNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
