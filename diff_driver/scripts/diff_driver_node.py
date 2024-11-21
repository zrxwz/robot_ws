#!/usr/bin/env python3

import rospy
import serial
from geometry_msgs.msg import Twist

# 串口初始化，根据实际情况修改串口号和波特率
ser_A = serial.Serial(port='/dev/ttyUSB1', baudrate=9600, timeout=1)  # A电机串口
ser_B = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=1)  # B电机串口

# 假设轮子间距为 0.37 米，轮子半径为 0.06 米，减速比为 50:1
wheel_separation = 0.37  # 轮子之间的距离
wheel_radius = 0.06  # 轮子的半径
gear_ratio = 50  # 电机的减速比

# 电机控制函数
def send_motor_command(left_speed, right_speed):
    # 将电机速度转为控制命令
    left_command = f'V{left_speed}\r'  # 生成左电机命令
    right_command = f'V{right_speed}\r'  # 生成右电机命令

    # 发送命令到电机
    rospy.loginfo(f"Sending commands: Left (A): {left_command}, Right (B): {right_command}")
    
    # 将命令以ASCII格式发送到串口
    ser_A.write(left_command.encode('ascii'))
    ser_B.write(right_command.encode('ascii'))

# 回调函数：接收 cmd_vel 消息并转换为电机速度
def cmd_vel_callback(msg):
    v_linear = msg.linear.x  # 获取线速度
    v_angular = msg.angular.z  # 获取角速度

    # 计算左右轮子的速度（单位：m/s）
    wheel_speed_left = v_linear - (v_angular * wheel_separation / 2)
    wheel_speed_right = v_linear + (v_angular * wheel_separation / 2)

    # 转换为电机轴的速度（单位：RPM），并考虑减速比
    motor_speed_left = (wheel_speed_left / (2 * 3.14159 * wheel_radius)) * 60 * gear_ratio
    motor_speed_right = (wheel_speed_right / (2 * 3.14159 * wheel_radius)) * 60 * gear_ratio

    # 打印调试信息，确认电机速度计算是否正确
    rospy.loginfo(f"Linear Speed: {v_linear}, Angular Speed: {v_angular}")
    rospy.loginfo(f"Wheel Speeds: Left: {wheel_speed_left}, Right: {wheel_speed_right}")
    rospy.loginfo(f"Motor Speeds: Left: {motor_speed_left}, Right: {motor_speed_right}")

    # 处理旋转和前进/后退的逻辑
    if v_angular == 0 and v_linear == 0:
        # 如果没有线速度和角速度，停止电机
        send_motor_command(0, 0)
    elif v_angular != 0 and v_linear == 0:
        # 如果只有角速度，两个电机转向相反，执行原地旋转
        send_motor_command(int(-motor_speed_left), int(motor_speed_right))  # 左电机反转，右电机正转
    elif v_linear != 0 and v_angular == 0:
        # 如果只有线速度，两个电机转向相反，执行前进或后退
        send_motor_command(int(motor_speed_left), int(-motor_speed_right))
    else:
        # 同时有线速度和角速度时，两个电机转向相反，产生角速度
        send_motor_command(int(motor_speed_right), int(-motor_speed_left))

def main():
    # 初始化 ROS 节点
    rospy.init_node('diff_drive_node')

    # 订阅 cmd_vel 消息
    rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)

    # 启动节点
    rospy.spin()

if __name__ == '__main__':
    main()

# rostopic pub /cmd_vel geometry_msgs/Twist --rate 10 "
# linear:
#   x: 0.1
#   y: 0.0
#   z: 0.0
# angular:
#   x: 0.0
#   y: 0.0
#   z: 0.5"
