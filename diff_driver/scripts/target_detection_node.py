#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from ultralytics import YOLO
import pykinect_azure as pykinect
from pykinect_azure import K4A_CALIBRATION_TYPE_COLOR
import numpy as np

class TargetDetectionNode:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('target_detection_node')

        # 创建目标位置发布器
        self.target_pub = rospy.Publisher('/target_position', Point, queue_size=10)

        # 初始化 Azure Kinect
        pykinect.initialize_libraries()
        device_config = pykinect.default_configuration
        device_config.color_format = pykinect.K4A_IMAGE_FORMAT_COLOR_BGRA32
        device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_720P
        device_config.depth_mode = pykinect.K4A_DEPTH_MODE_NFOV_UNBINNED
        self.device = pykinect.start_device(config=device_config)

        # 加载 YOLO 模型
        self.yolo_model_path = rospy.get_param("~yolo_model_path", "/path/to/yolov8n.pt")
        self.device = rospy.get_param("~device", "cpu")

        # 获取用户选择的目标类别
        self.target_class = rospy.get_param('~target_class', 'person')  # 默认检测 "person"
        self.tracked_target_id = None

        # 获取类别到 ID 的映射（根据模型的类别）
        self.class_map = self.model.names
        self.target_class_id = self.get_class_id(self.target_class)

    def get_class_id(self, class_name):
        """根据类别名称获取类别 ID"""
        for class_id, name in self.class_map.items():
            if name == class_name:
                return class_id
        rospy.logwarn(f"Class '{class_name}' not found in model. Using 'person' by default.")
        return 0  # 默认类别为 "person"

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            capture = self.device.update()
            ret_color, color_image = capture.get_color_image()
            ret_depth, transformed_depth_image = capture.get_transformed_depth_image()

            if not ret_color or not ret_depth:
                continue

            # 提取 RGB 通道
            color_image_rgb = color_image[:, :, :3]

            # 使用 YOLO 进行检测
            results = self.model(color_image_rgb, conf=0.5)

            # 寻找目标
            target_found = False
            for result in results:
                for i, (box, cls) in enumerate(zip(result.boxes.xywh, result.boxes.cls)):
                    if int(cls) != self.target_class_id:  # 忽略不符合目标类别的检测
                        continue

                    x = int(box[0].item())
                    y = int(box[1].item())

                    # 获取深度值
                    rgb_depth = transformed_depth_image[y, x]
                    if rgb_depth == 0:  # 无效深度值
                        continue

                    # 转换到 3D 空间坐标
                    pixels = pykinect.k4a_float2_t((x, y))
                    pos3d_color = self.device.calibration.convert_2d_to_3d(
                        pixels, rgb_depth, K4A_CALIBRATION_TYPE_COLOR, K4A_CALIBRATION_TYPE_COLOR
                    )

                    if not isinstance(pos3d_color, pykinect.k4a_float3_t):
                        continue

                    x3d, y3d, z3d = pos3d_color.v
                    if self.tracked_target_id is None or i == self.tracked_target_id:
                        self.tracked_target_id = i  # 锁定目标
                        target_found = True

                        # 发布目标位置
                        target_position_msg = Point()
                        target_position_msg.x = x3d
                        target_position_msg.y = y3d
                        target_position_msg.z = z3d
                        self.target_pub.publish(target_position_msg)

                        rospy.loginfo(f"Published target position: ({x3d}, {y3d}, {z3d})")
                        break
                if target_found:
                    break

            # 如果未找到目标，清除跟踪
            if not target_found:
                rospy.loginfo("Target lost.")
                self.tracked_target_id = None

            rate.sleep()

        self.device.stop_device()


if __name__ == '__main__':
    try:
        node = TargetDetectionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
