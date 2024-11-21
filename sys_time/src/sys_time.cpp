#include <ros/ros.h>
#include "sys_time/sys_time.h"
 
// # 重写头文件中的构造函数subscriberANDpublisher()
// # main函数初始化对象(subscriberANDpublisher sp）时自动调用构造函数
subscriberANDpublisher::subscriberANDpublisher()
{
    //订阅话题
    lidar_sub.subscribe(nh, "/rslidar_points", 1);
    camera_sub.subscribe(nh, "/rgb/image_raw", 1); 

    //消息过滤器，使用 ApproximateTime 进行时间同步（允许一定程度的时间误差）
    sync_.reset(new message_filters::Synchronizer<syncpolicy>(syncpolicy(10), camera_sub, lidar_sub));
    sync_->registerCallback(boost::bind(&subscriberANDpublisher::callback, this, _1, _2));

    //发布者
    camera_pub = nh.advertise<sensor_msgs::Image>("sync/img", 10);
    lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("sync/lidar", 10);
}

void subscriberANDpublisher::callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::PointCloud2ConstPtr& pointcloud) {
    ROS_INFO("Received synchronized message!");
    camera_pub.publish(image);
    lidar_pub.publish(pointcloud);
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "node");
    subscriberANDpublisher sp;
    ROS_INFO("main done! ");
    ros::spin();
}

