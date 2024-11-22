#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Point, Twist

# 速度调节系数
k_v = 0.5       # 线速度比例因子
k_omega = 1.0   # 角速度比例因子

# 发布器
cmd_vel_pub = None

def target_position_callback(target_position):
    # 提取目标位置
    x_target = target_position.x
    y_target = target_position.y

    # 计算距离和角度
    distance_to_target = math.sqrt(x_target ** 2 + y_target ** 2)
    angle_to_target = math.atan2(y_target, x_target)

    # 计算线速度和角速度
    v_linear = k_v * distance_to_target
    v_angular = k_omega * angle_to_target

    # 创建并发布 cmd_vel 消息
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = v_linear
    cmd_vel_msg.angular.z = v_angular

    cmd_vel_pub.publish(cmd_vel_msg)

def main():
    global cmd_vel_pub

    # 初始化ROS节点
    rospy.init_node('target_follow_node')

    # 初始化cmd_vel发布器
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # 订阅目标位置信息
    rospy.Subscriber('/target_position', Point, target_position_callback)

    # 保持运行
    rospy.spin()

if __name__ == '__main__':
    main()

