#include "ros/ros.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_ros/point_cloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "ros/package.h"



int main(int argc, char** argv){
    ros::init(argc,argv,"pub_pcd_map");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("/global_map",1);

    std::string map_path;
    std::string workspace_path = ros::package::getPath("ndt_imu_localization");
    map_path = workspace_path + "/map/kitti/filtered_map.pcd";
    sensor_msgs::PointCloud2 point_cloud;
    pcl::io::loadPCDFile(map_path,point_cloud);
    point_cloud.header.frame_id = "world";
    point_cloud.header.stamp = ros::Time::now();

    ros::Rate loop(1);
    while(ros::ok()){
        
        map_pub.publish(point_cloud);
        
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}