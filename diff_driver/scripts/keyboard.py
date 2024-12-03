#!/usr/bin/env python3

import rospy
import serial
from geometry_msgs.msg import Twist
from teleop_twist_keyboard import get_key

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

# 键盘控制回调函数
def keyboard_control():
    rospy.init_node('keyboard_control_node', anonymous=True)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    rospy.loginfo("Use WASD keys to control robot")
    rospy.loginfo("Press Q to quit")

    msg = Twist()

    while not rospy.is_shutdown():
        key = get_key()

        # 根据按键设置速度
        if key == 'w':  # 向前
            msg.linear.x = 0.3
            msg.angular.z = 0.0
        elif key == 's':  # 向后
        
            msg.linear.x = -0.3
            msg.angular.z = 0.0
        elif key == 'a':  # 向左
            msg.linear.x = 0.0
            msg.angular.z = 0.2
        elif key == 'd':  # 向右
            msg.linear.x = 0.0
            msg.angular.z = -0.2
        elif key == 'q':  # 退出
            break
        else:  # 无按键时停止
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        
        pub.publish(msg)  # 发布速度命令
        rospy.sleep(0.1)

# 启动键盘控制节点
if __name__ == '__main__':
    try:
        keyboard_control()
    except rospy.ROSInterruptException:
        pass
# rosrun teleop_twist_keyboard teleop_twist_keyboard.py

