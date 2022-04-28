#include "ros/ros.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/common/transforms.h"
#include "string"
#include "deque"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Imu.h"
#include "tf/transform_listener.h"

std::deque<sensor_msgs::PointCloud2> pointcloud_ver;

void PointCloudCB(const sensor_msgs::PointCloud2ConstPtr &msg){
    pointcloud_ver.push_back(*msg);
}

void Splicing(  pcl::PointCloud<pcl::PointXYZI>::Ptr total_pointcloud, 
                sensor_msgs::PointCloud2 current_pointcloud,
                tf::StampedTransform trans){
                
                //transform from rosmsg to pcl
                pcl::PointCloud<pcl::PointXYZI> current_pointcloud_ptr;
                pcl::fromROSMsg(current_pointcloud,current_pointcloud_ptr);

                //create transform matrix
                double yaw,pitch,roll;
                double x,y,z;
                x = trans.getOrigin().getX();
                y = trans.getOrigin().getY();
                z = trans.getOrigin().getZ();
                tf::Matrix3x3(trans.getRotation()).getRPY(roll,pitch,yaw);
                Eigen::Affine3f transform =  pcl::getTransformation(x,y,z,roll,pitch,yaw);

                //transform pointcloud
                pcl::PointCloud<pcl::PointXYZI> pointcloud_transformed;
                pcl::transformPointCloud(current_pointcloud_ptr,pointcloud_transformed,transform);

                //
                *total_pointcloud = *total_pointcloud + pointcloud_transformed;
                }

void PublishPointcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud, ros::Publisher pub){
    sensor_msgs::PointCloud2 output_pointcloud;
    pcl::toROSMsg(*pointcloud,output_pointcloud);
    output_pointcloud.header.frame_id = "simple_cave_02";
    output_pointcloud.header.stamp = ros::Time::now();
    pub.publish(output_pointcloud);
}



int main(int argc,char** argv){
    ros::init(argc,argv,"pointcloud_splicing");
    ros::NodeHandle nh;

    std::string pointcloud_topic("/X1/points");
    ros::Subscriber pointcloud_cb = nh.subscribe(pointcloud_topic,10,&PointCloudCB);
    ros::Publisher pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/splicing_pointcloud",10);
    ros::Rate loop(20);

    tf::TransformListener listener;
    tf::StampedTransform trans;

    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZI>); 

    int max_pointcloud{0};
    while(ros::ok()){
        ros::spinOnce();
        //if no pointcloud
        if(pointcloud_ver.size() == 0){
            loop.sleep();
            continue;
        }
        //frop beginning wrong data
        if(pointcloud_ver[0].header.stamp.toSec() < 24){
            pointcloud_ver.pop_front();
            loop.sleep();
            continue;
        }
        //find tranform from baselink to map
        //NOTE 发现了一个奇怪的现象，为什么tf发布的最新的时刻是14s，我去获取14.5s的trans，他会把14s的trans发给我？
        try{
            listener.lookupTransform("/simple_cave_02","/X1/base_link"
                                        ,pointcloud_ver[0].header.stamp,trans);  
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(0.2).sleep();
            continue;
        }
        //drop old transform
        // std::cout << "trans stamp is " << trans.stamp_.toNSec()
        //           << "pointcloud stamp is " << pointcloud_ver[0].header.stamp.toNSec() << std::endl;
        double time_now = pointcloud_ver[0].header.stamp.toSec();
        if(time_now - abs(time_now) != 0){
            // std::cout << "grop old frame" << std::endl;
            pointcloud_ver.pop_front();
            loop.sleep();
            continue;
        }
        // std::cout << "time is " << trans.stamp_.toSec() << std::endl;
        // std::cout << " x is " << trans.getOrigin().getX() << std::endl;
        if(max_pointcloud++ > 200){
            break;
        }
        Splicing(pointcloud_ptr,pointcloud_ver[0],trans);
        PublishPointcloud(pointcloud_ptr,pointcloud_pub);
        pointcloud_ver.pop_front();
        loop.sleep();
    }

    pcl::io::savePCDFile("/home/lovebc/Desktop/sbut.pcd",*pointcloud_ptr);
    std::cout << "save pcd file success" << std::endl;
    std::cout << "total point num is " << pointcloud_ptr->size() << std::endl;
    return 0;
    
}