#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Point, Twist

# 速度调节系数
k_omega = 1.0   # 角速度比例因子

# PID 参数
Kp = 0.5        # 比例增益
Ki = 0.1        # 积分增益
Kd = 0.05       # 微分增益
target_distance = 1.0  # 目标距离（单位：米）

# 全局变量
cmd_vel_pub = None
previous_error = 0.0
integral = 0.0

# 跟随目标的 PID 控制器
def pid_control(current_distance, dt):
    global previous_error, integral

    # 计算误差
    error = current_distance - target_distance

    # 积分项
    integral += error * dt

    # 微分项
    derivative = (error - previous_error) / dt if dt > 0 else 0

    # PID 输出
    output = Kp * error + Ki * integral + Kd * derivative

    # 更新前一个误差
    previous_error = error

    return output

# 目标位置回调函数
def target_position_callback(target_position):
    global cmd_vel_pub, previous_error

    # 提取目标位置
    x_target = target_position.x
    y_target = target_position.y

    # 获取当前时间
    current_time = rospy.Time.now().to_sec()

    # 计算目标距离和角度
    distance_to_target = math.sqrt(x_target ** 2 + y_target ** 2)
    angle_to_target = math.atan2(y_target, x_target)

    # 时间间隔（用于微分项计算）
    dt = rospy.Time.now().to_sec() - current_time

    # 使用 PID 控制调整线速度
    v_linear = pid_control(distance_to_target, dt)

    # 限制线速度，避免过快
    v_linear = max(min(v_linear, 1.0), -1.0)

    # 角速度控制
    v_angular = k_omega * angle_to_target

    # 创建并发布 cmd_vel 消息
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = v_linear
    cmd_vel_msg.angular.z = v_angular

    cmd_vel_pub.publish(cmd_vel_msg)

def main():
    global cmd_vel_pub

    # 初始化 ROS 节点
    rospy.init_node('target_follow_node')

    # 初始化 cmd_vel 发布器
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # 订阅目标位置信息
    rospy.Subscriber('/target_position', Point, target_position_callback)

    # 保持运行
    rospy.spin()

if __name__ == '__main__':
    main()
