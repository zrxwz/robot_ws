#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class SyncTimestamp {
public:
    SyncTimestamp() {
        // 订阅各个话题（删除了IMU相关订阅）
        image_sub.subscribe(nh, "/rgb/image_raw", 1);
        camera_info_sub.subscribe(nh, "/rgb/camera_info", 1);
        pointcloud_sub.subscribe(nh, "/velodyne_points", 1);

        // 使用 ApproximateTime 策略同步时间戳（删除了IMU相关同步）
        sync_.reset(new message_filters::Synchronizer<SyncPolicy>(
            SyncPolicy(10), image_sub, camera_info_sub, pointcloud_sub));
        sync_->setMaxIntervalDuration(ros::Duration(0.1));  // 可调节时间容忍度
        sync_->registerCallback(boost::bind(&SyncTimestamp::callback, this, _1, _2, _3));

        // 创建发布者，修改同步后的话题名称
        image_pub = nh.advertise<sensor_msgs::Image>("sync/rgb/image_raw", 10);
        camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("sync/rgb/camera_info", 10);
        pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("sync/velodyne_points", 10);
    }

    void callback(const sensor_msgs::ImageConstPtr& image_msg,
                  const sensor_msgs::CameraInfoConstPtr& camera_info_msg,
                  const sensor_msgs::PointCloud2ConstPtr& pointcloud_msg) {
        ROS_INFO("Received synchronized messages!");

        // 发布同步后的消息到新的话题
        image_pub.publish(image_msg);
        camera_info_pub.publish(camera_info_msg);
        pointcloud_pub.publish(pointcloud_msg);

        // 创建ros::Rate对象，设置频率为10Hz
        ros::Rate rate(10);  // 10Hz

        // 在回调函数的末尾执行一次休眠，控制发布频率
        rate.sleep();
    }

private:
    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::Image> image_sub;
    message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub;

    ros::Publisher image_pub;
    ros::Publisher camera_info_pub;
    ros::Publisher pointcloud_pub;

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image,
        sensor_msgs::CameraInfo,
        sensor_msgs::PointCloud2
    > SyncPolicy;

    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "sync_timestamp_node");
    SyncTimestamp sync_timestamp;
    ros::spin();
    return 0;
}
