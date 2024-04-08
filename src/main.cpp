
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/io/ply_io.h>
void callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
       for (size_t i = 0; i < msg->ranges.size(); ++i) {
        // 判断每个点的距离是否越界
        if (msg->ranges[i] < 0.1|| msg->ranges[i] > 30) 
            continue;

        // 获取第i个点与车辆自身的角度
        double real_angle = msg->angle_min + i * msg->angle_increment; 

        // 将雷达的距离和角度转换为笛卡尔坐标
        double x = msg->ranges[i] * std::cos(real_angle);  
        double y = msg->ranges[i] * std::sin(real_angle);
        pcl::PointXYZI p;
        p.x = x;
        p.y = y;
        p.z = 0;
        p.intensity = 0;
        cloud->points.push_back(p);
       }
         cloud->width = cloud->points.size();
            cloud->height = 1;
            cloud->is_dense = false;
            //ply保存
            pcl::io::savePLYFile("scan.ply", *cloud);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan2pcd");
    ros::NodeHandle nh;

    std::string cloud_topic;
    nh.param<std::string>("cloud_topic", cloud_topic, "/front_scan");
    ros::Subscriber sub = nh.subscribe(cloud_topic, 10, callback);
    // 开始循环等待回调函数
    ros::spin();

    return 0;
}