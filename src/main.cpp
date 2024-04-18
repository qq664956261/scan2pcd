
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/ply_io.h>
 #include <pcl_conversions/pcl_conversions.h>
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
int num = 0;
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_all(new pcl::PointCloud<pcl::PointXYZI>);
void pointcloud2_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);
    *cloud_all += *cloud;
    num++;
    if(num > 100)
    {
        cloud_all->width = cloud_all->points.size();
        cloud_all->height = 1;
        cloud_all->is_dense = false;
        pcl::io::savePLYFile("pointcloud_all.ply", *cloud_all);
        num = 0;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan2pcd");
    ros::NodeHandle nh;

    std::string cloud_topic;
    nh.param<std::string>("cloud_topic", cloud_topic, "/front_scan");
    ros::Subscriber sub = nh.subscribe(cloud_topic, 10, callback);
    std::string pointcloud2_topic;
    nh.param<std::string>("pointcloud2_topic", pointcloud2_topic, "/scan_matched_points2");
    ros::Subscriber sub2 = nh.subscribe(pointcloud2_topic, 10, pointcloud2_callback);
    // 开始循环等待回调函数
    ros::spin();

    return 0;
}