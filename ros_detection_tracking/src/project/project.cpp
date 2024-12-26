#include"../../include/ros_detection_tracking/project.h"

namespace DetectandTract {

    // 初始化参数函数
    void projector::initParams()
    {
        // 获取 ROS 包的路径
        std::string pkg_loc = ros::package::getPath("ros_detection_tracking");
        // 打开配置文件读取参数
        std::ifstream infile(pkg_loc + "/cfg/initial_params.txt");
        infile >> i_params.camera_topic;  // 读取相机话题
        infile >> i_params.lidar_topic;   // 读取激光雷达话题
        
        // 定义矩阵用于存储相机内外参数
        double_t camtocam[12];
        double_t cameraIn[16];
        double_t RT[16];

        // 读取相机间转换矩阵（camtocam）参数
        for (int i = 0; i < 16; i++) {
            infile >> camtocam[i];
        }
        // 将读取的参数转换为 OpenCV 矩阵
        cv::Mat(4, 4, 6, &camtocam).copyTo(i_params.camtocam_mat);

        // 读取相机内参（cameraIn）矩阵
        for (int i = 0; i < 12; i++) {
            infile >> cameraIn[i];
        }
        // 将读取的参数转换为 OpenCV 矩阵
        cv::Mat(4, 4, 6, &cameraIn).copyTo(i_params.cameraIn);

        // 读取激光雷达到相机坐标系的转换矩阵（RT）参数
        for (int i = 0; i < 16; i++) {
            infile >> RT[i];
        }
        // 将读取的参数转换为 OpenCV 矩阵
        cv::Mat(4, 4, 6, &RT).copyTo(i_params.RT);

        // 打印 RT 矩阵以供调试
        std::cout << i_params.RT << std::endl;
    }

    // 点云与图像数据的投影回调函数
    void projector::projection_callback(const sensor_msgs::Image::ConstPtr &img, 
                                        const sensor_msgs::PointCloud2::ConstPtr &pc)
    {
        // 将 ROS 图像消息转换为 OpenCV 图像
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(img, "bgr8");
        }
        catch (cv_bridge::Exception &e) {
            return;  // 如果转换失败，则退出
        }
        
        // 获取原始图像
        cv::Mat raw_img = cv_ptr->image;
        
        // 创建一个指向点云的指针
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*pc, *cloud);  // 将 ROS 点云消息转换为 PCL 点云格式
        
        // 创建可视化图像和覆盖图像
        cv::Mat visImg = raw_img.clone();
        cv::Mat overlay = visImg.clone();
        
        // 定义 4x1 矩阵 X 和 3x1 矩阵 Y，用于存储点云转换后的坐标
        cv::Mat X(4, 1, cv::DataType<double>::type);
        cv::Mat Y(3, 1, cv::DataType<double>::type);

        // 遍历点云中的每个点
        for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = cloud->points.begin(); it != cloud->points.end(); it++) {
            // 根据条件过滤掉无效的点（如超出最大范围或低于最小 Z 值）
            if (it->x > maxX || it->x < 0.0 || abs(it->y) > maxY || it->z < minZ) {
                continue;
            }

            // 将点的坐标赋值到 X 矩阵中
            X.at<double>(0, 0) = it->x;
            X.at<double>(1, 0) = it->y;
            X.at<double>(2, 0) = it->z;
            X.at<double>(3, 0) = 1;  // 同次坐标

            // 将点云坐标转换为相机坐标系中的坐标
            Y = i_params.cameraIn * i_params.camtocam_mat * i_params.RT * X;

            // 计算点的二维图像坐标
            cv::Point pt;
            pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2);  // x = X / Z
            pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);  // y = Y / Z

            // 根据点的 x 坐标值设置颜色（通过值来设定红绿通道）
            float val = it->x;
            float maxVal = 20.0;
            int red = std::min(255, (int)(255 * abs((val - maxVal) / maxVal)));
            int green = std::min(255, (int)(255 * (1 - abs((val - maxVal) / maxVal))));

            // 在图像上绘制投影点
            cv::circle(overlay, pt, 1, cv::Scalar(0, green, red), -1);  // 用不同颜色标记点
        }

        // 发布投影后的图像
        ros::Time time = ros::Time::now();
        cv_ptr->encoding = "bgr8";  // 设置图像编码格式
        cv_ptr->header.stamp = time;  // 设置时间戳
        cv_ptr->header.frame_id = "/camera_main";  // 设置图像帧 ID
        cv_ptr->image = overlay;  // 设置图像内容为投影后的图像
        image_publisher.publish(cv_ptr->toImageMsg());  // 发布图像消息
    }

    // 矩阵转换为 tf::Transform 类型的函数（目前没有实现）
    void projector::matrix_to_transfrom(Eigen::MatrixXf &matrix, tf::Transform &trans)
    {
        // 此函数暂时未实现
    }

    // 构造函数
    projector::projector() 
    {
        ros::NodeHandle nh("~");  // 创建 ROS 节点句柄
        initParams();  // 初始化参数

        ros::Publisher project_img_pub;  // 创建发布图像的发布器

        // 创建消息过滤器订阅器，订阅摄像头图像和激光雷达点云数据
        message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, i_params.camera_topic, 5);
        message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(nh, i_params.lidar_topic, 5);

        // 定义同步策略：近似时间同步
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
        message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(5), image_sub, pcl_sub);
        
        // 注册回调函数，当数据同步时调用 projection_callback
        sync.registerCallback(boost::bind(&projector::projection_callback, this, _1, _2));

        // 创建图像传输对象并发布图像
        image_transport::ImageTransport imageTransport(nh);
        image_publisher = imageTransport.advertise("/project_pc_image", 20);

        ros::spin();  // 启动 ROS 事件循环
    }

}

// 主函数
int main(int argc, char **argv){
    ros::init(argc, argv, "project_pc_to_image");  // 初始化 ROS 节点
    DetectandTract::projector projector;  // 创建 projector 对象，启动点云投影功能
    return 0;
}
