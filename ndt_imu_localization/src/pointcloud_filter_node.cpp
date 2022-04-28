//c++
#include <iostream>
#include <functional>
//ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include "pointcloud_filter.h"

ros::Publisher pointcloud_pub;
std::string lidar_frame;

void PointCloudHandler(const sensor_msgs::PointCloud2ConstPtr &pointcloud_in,FilterParam& filter_param){
    
    if(pointcloud_in->header.stamp.toSec() < 60){
        return ;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<PointT>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_filtered(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*pointcloud_in,*pointcloud);

    double start_time = ros::Time::now().toSec();
    PointCloudFilter(pointcloud,pointcloud_filtered,filter_param);

    sensor_msgs::PointCloud2 pointcloud_out;
    pcl::toROSMsg(*pointcloud_filtered,pointcloud_out);
    pointcloud_out.header = pointcloud_in->header;
    pointcloud_out.header.frame_id = lidar_frame;
    pointcloud_pub.publish(pointcloud_out);

    std::cout << "filter cost time is " << ros::Time::now().toSec() - start_time << std::endl;
}

int main(int argc,char** argv){
    ros::init(argc,argv,"point_cloud_filter");
    ros::NodeHandle nh;
    ros::Subscriber pointcloud_sub;  
    FilterParam filter_param;   

    std::string point_cloud_topic;
    ros::NodeHandle private_nh("~");
    private_nh.param("point_cloud_topic",point_cloud_topic,std::string("/kitti/velo/pointcloud"));
    private_nh.param("lidar_frame",lidar_frame,std::string("lidar_link"));
    // pointcloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/kitti/velo/pointcloud",10,boost::bind(&PointCloudHandler,_1,std::ref(filter_param)));
    pointcloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(point_cloud_topic,10,boost::bind(&PointCloudHandler,_1,std::ref(filter_param)));
    pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("filtered_points",10);


    ros::Rate loop(50);
    while(ros::ok()){
        ros::spinOnce();
        loop.sleep();
    }
    return  0;
}