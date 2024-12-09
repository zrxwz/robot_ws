#!/usr/bin/env python3

import rospy
import message_filters
from sensor_msgs.msg import PointCloud2, Imu

def callback(lidar_msg, imu_msg):
    # 在此处理同步后的数据
    rospy.loginfo("Synchronized LiDAR and IMU data received")
    # 处理 lidar_msg 和 imu_msg

def sync_data():
    rospy.init_node('data_sync_node')

    # 创建消息订阅者
    lidar_sub = message_filters.Subscriber('/rslidar_points', PointCloud2)
    imu_sub = message_filters.Subscriber('/imu', Imu)

    # 创建时间同步的缓存和同步对象
    ts = message_filters.ApproximateTimeSynchronizer([lidar_sub, imu_sub], queue_size=10, slop=0.1)
    ts.registerCallback(callback)

    rospy.spin()

if __name__ == '__main__':
    sync_data()

