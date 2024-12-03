#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, Point

class GenericCoordinateTransformer:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('generic_coordinate_transformer')

        # 从参数获取话题名和坐标系
        self.input_topic = rospy.get_param('~input_topic', '/input_point')
        self.output_topic = rospy.get_param('~output_topic', '/output_point')
        self.source_frame = rospy.get_param('~source_frame', 'source_frame')
        self.target_frame = rospy.get_param('~target_frame', 'target_frame')

        # 订阅输入点
        self.input_sub = rospy.Subscriber(self.input_topic, Point, self.point_callback)

        # 发布转换后的点
        self.output_pub = rospy.Publisher(self.output_topic, Point, queue_size=10)

        # tf2 缓存
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def point_callback(self, input_point):
        """
        将输入点从 source_frame 转换到 target_frame
        :param input_point: geometry_msgs/Point
        """
        try:
            # 构建 PointStamped 消息
            point_in_source = PointStamped()
            point_in_source.header.frame_id = self.source_frame
            point_in_source.header.stamp = rospy.Time.now()
            point_in_source.point = input_point

            # 获取 source_frame 到 target_frame 的变换
            transform = self.tf_buffer.lookup_transform(self.target_frame, self.source_frame, rospy.Time(0), rospy.Duration(1.0))

            # 进行坐标变换
            point_in_target = tf2_geometry_msgs.do_transform_point(point_in_source, transform)

            # 发布转换后的点
            transformed_point = Point()
            transformed_point.x = point_in_target.point.x
            transformed_point.y = point_in_target.point.y
            transformed_point.z = point_in_target.point.z
            self.output_pub.publish(transformed_point)

            rospy.loginfo(f"Transformed point: {transformed_point}")

        except tf2_ros.LookupException as e:
            rospy.logwarn(f"Transform lookup failed: {e}")
        except tf2_ros.ExtrapolationException as e:
            rospy.logwarn(f"Transform extrapolation failed: {e}")

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = GenericCoordinateTransformer()
        node.run()
    except rospy.ROSInterruptException:
        pass
