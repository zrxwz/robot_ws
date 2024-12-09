#!/usr/bin/env python3
import rospy
import rosbag
import message_filters
from sensor_msgs.msg import PointCloud2, Imu

def callback(lidar_msg, imu_msg, bag_writer):
    # 同步后的数据处理
    rospy.loginfo(f"Synchronized IMU and LiDAR data at time: {lidar_msg.header.stamp}, {imu_msg.header.stamp}")
    
    # 将同步后的数据写入新的 rosbag 文件
    bag_writer.write('/rslidar_points', lidar_msg)
    bag_writer.write('/imu', imu_msg)

def sync_and_save_bag(input_bag_file, output_bag_file):
    # 打开原始 rosbag 文件进行读取
    input_bag = rosbag.Bag(input_bag_file, 'r')
    
    # 打开新 rosbag 文件进行写入
    output_bag = rosbag.Bag(output_bag_file, 'w')

    # 创建消息同步订阅者
    lidar_sub = message_filters.Subscriber('/rslidar_points', PointCloud2)
    imu_sub = message_filters.Subscriber('/imu', Imu)

    # 使用 ApproximateTimeSynchronizer 进行时间同步
    ts = message_filters.ApproximateTimeSynchronizer([lidar_sub, imu_sub], queue_size=10, slop=0.1)
    ts.registerCallback(lambda lidar, imu: callback(lidar, imu, output_bag))

    # 初始化 ROS 节点
    rospy.init_node('sync_bag_node')

    # 读取原始 bag 文件的所有消息，并将同步的消息传递给同步器
    for topic, msg, t in input_bag.read_messages(topics=['/rslidar_points', '/imu']):
        if topic == '/rslidar_points':
            lidar_sub.callback(msg)
        elif topic == '/imu':
            imu_sub.callback(msg)

    # 关闭 rosbag 文件
    input_bag.close()
    output_bag.close()

if __name__ == "__main__":
    input_bag_file = '/home/zhurui/bagfiles/imulidar/1_2024-12-05-14-12-51.bag'  # 输入的原始 bag 文件
    output_bag_file = '/home/zhurui/bagfiles/sysimu.bag'  # 输出的同步后的 bag 文件

    sync_and_save_bag(input_bag_file, output_bag_file)
