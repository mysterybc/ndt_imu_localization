#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pclomp/ndt_omp.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/impl/filter.hpp>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <thread>
#include <mutex>
#include "ros/package.h"

#include <ndt_imu_localization/ndt_match.h>
// #include "Geocentric/LocalCartesian.hpp"
#include "sensor_msgs/NavSatFix.h"
#include "deque"

typedef pcl::PointXYZ PointT;

//params
bool need_initialization;
bool imu_enable;
bool save_downsample_map;
bool downsample_map;
std::string map_source;
bool get_rviz_pose{false};
double score_threshold;
double score;
double initial_x;
double initial_y;
double initial_z;
Eigen::Matrix3f init_rotation;
double downsample_resolution;
std::string world_frame,lidar_frame,imu_frame;


std::deque<nav_msgs::Odometry> imu_odometry;
std::deque<nav_msgs::Odometry> gps_odometry;
std::deque<sensor_msgs::PointCloud2::ConstPtr> pointcloud_que;
Eigen::Matrix4f gps2lidar;

//rospub
ros::Publisher sub_map_pub;
ros::Publisher pubPoseForKITTI;

ros::Time pointcloud_time;
pcl::Filter<PointT>::Ptr downsample_filter;


//局部地图的参数
int laserCloudCenX = 12;
int laserCloudCenY = 12;

const int laserCloudX = 25;
const int laserCloudY = 25;

double map_segment_size = 35.0;//meter

const int laserCloudNum = laserCloudX * laserCloudY;

pcl::PointCloud<PointT>::Ptr laserCloudMapArray[laserCloudNum];


//点云指针
pcl::PointCloud<PointT>::ConstPtr  prev_cloud_Ptr(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr  cur_cloud_Ptr(new pcl::PointCloud<PointT>);
pcl::PointCloud<pcl::PointXYZ>::ConstPtr  point_cloud_map_Ptr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<PointT>::Ptr sub_map(new pcl::PointCloud<PointT>);


geometry_msgs::TransformStamped odom_trans;

Eigen::Matrix4f prev_trans;                  // previous estimated transform from keyframe
Eigen::Matrix4f cur_trans;                       // current estimated transform from keyframe

Eigen::Quaternionf cur_imu_quaternion;
Eigen::Quaternionf prev_imu_quaternion;


Eigen::Matrix4f error_trans;

//验证一下自己的想法，把旋转和平移分开
Eigen::Vector3f pre_translation;
Eigen::Vector3f cur_translation;

void Odometry2Matrix4f(nav_msgs::Odometry odom, Eigen::Matrix4f &trans){
    //set xyz
    trans(0,3) =  odom.pose.pose.position.x;
    trans(1,3) =  odom.pose.pose.position.y;
    trans(2,3) =  odom.pose.pose.position.z;

    //set rotation
    Eigen::Quaternionf quaternion(  odom.pose.pose.orientation.w,odom.pose.pose.orientation.x,
                                    odom.pose.pose.orientation.y,odom.pose.pose.orientation.z); 
    trans.block(0,0,3,3) = quaternion.matrix();
}


void RosQ2RPY(const geometry_msgs::Quaternion& quaternion ,double& roll,double& pitch,double& yaw)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(quaternion, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
}

//下采样滤波器
pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud)
{
    if(!downsample_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
}

//submap生成： 输入：1、预估的变换关系 2、完整的点云地图（或者分割好的点云cube） 输出：当前预估关系的map下的submap
void sub_map_generation(const Eigen::Matrix4f transform,const pcl::PointCloud<PointT>::Ptr laserCloudMapArray[] , pcl::PointCloud<PointT>::Ptr sub_map)
{
    double sub_map_start_time = ros::Time::now().toSec();
    int cur_I,cur_J;
    int trans_to_map_X,trans_to_map_Y;

    //lidar视域范围内(FOV)的点云集索引
    int laserCloudValidInd[125];
    //lidar周围的点云集索引
    int laserCloudSurroundInd[125];

    trans_to_map_X = transform(0,3);
    trans_to_map_Y = transform(1,3);

    //确定x,y坐标对应的cube索引
    cur_I = int((trans_to_map_X + map_segment_size/2.0) / map_segment_size) + laserCloudCenX;
    cur_J = int((trans_to_map_Y + map_segment_size/2.0) / map_segment_size) + laserCloudCenY;


    if (trans_to_map_X + map_segment_size/2.0 < 0) cur_I--;
    if (trans_to_map_Y + map_segment_size/2.0 < 0) cur_J--;

    int cubeInd = cur_I + laserCloudX * cur_J; //ind = I + laserCloudX * J


    int laserCloudValidNum = 0;

    for (int i = cur_I - 1; i <= cur_I + 1; i++) 
    {
        for (int j = cur_J - 1; j <= cur_J + 1; j++) 
        {
            if (i >= 0 && i < laserCloudX && 
                j >= 0 && j < laserCloudY ) 
            {
                //如果索引合法
                //记住视域范围内的cube索引，匹配用
                laserCloudValidInd[laserCloudValidNum] = i + laserCloudX * j;

                laserCloudValidNum++;
            }
        }
    }

    sub_map->clear();

    for (int i = 0; i < laserCloudValidNum; i++) 
    {
        *sub_map += *laserCloudMapArray[laserCloudValidInd[i]];
    }
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg (*sub_map, output);
    output.header.frame_id = world_frame;


    sub_map_pub.publish(output);
}


void publish_odometry(const ros::Time& stamp, const Eigen::Matrix4f& pose) 
{
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "world";
    Eigen::Matrix3f rotation_matrix = pose.block(0,0,3,3);
    Eigen::Quaternionf tem_q = Eigen::Quaternionf(rotation_matrix);

    geometry_msgs::Quaternion q_geo;
    q_geo.x = tem_q.x();
    q_geo.y = tem_q.y();
    q_geo.z = tem_q.z();
    q_geo.w = tem_q.w();

    double tmp_roll,tmp_pitch,tmp_yaw;
    RosQ2RPY(q_geo,tmp_roll,tmp_pitch,tmp_yaw);

    odom.header.stamp = stamp;
    odom.pose.pose.position.x = pose(0, 3);
    odom.pose.pose.position.y = pose(1, 3);
    odom.pose.pose.position.z = pose(2, 3);


    odom.pose.pose.orientation.x = tem_q.x();
    odom.pose.pose.orientation.y = tem_q.y();
    odom.pose.pose.orientation.z = tem_q.z();
    odom.pose.pose.orientation.w = tem_q.w();


    odom.child_frame_id = "lidar_link";
    //odom.twist.twist.linear.x = tmp_roll*180/M_PI;//;
    //odom.twist.twist.linear.y = tmp_pitch*180/M_PI;
    //odom.twist.twist.linear.z = tmp_yaw*180/M_PI;
    //odom.twist.twist.angular.z = tmp_yaw;

    pubPoseForKITTI.publish(odom);

    odom_trans.header.stamp = stamp;
    // odom_trans.header.frame_id = "world";
    // odom_trans.child_frame_id = "lidar_link";
    odom_trans.header.frame_id = world_frame;
    odom_trans.child_frame_id = lidar_frame;
    odom_trans.transform.translation.x = odom.pose.pose.position.x;
    odom_trans.transform.translation.y = odom.pose.pose.position.y;
    odom_trans.transform.translation.z = odom.pose.pose.position.z;
    odom_trans.transform.rotation = q_geo;

    //odom_broadcaster.sendTransform(odom_trans);
}


//最初设备这个函数的意义是使用imu姿态
Eigen::Matrix4f matching_scan_to_map_fix(double& score,const pcl::PointCloud<PointT>::ConstPtr& cur_point,const pcl::PointCloud<PointT>::ConstPtr& map_point)
{
    NdtMatch ndtmatch;
    //估计当前相对于上一帧增加的变换矩阵
    if( need_initialization )
    {
        prev_trans.setIdentity();   
        cur_trans.setIdentity();
        error_trans.setIdentity();
        
        cur_trans(0,3) = initial_x;
        cur_trans(1,3) = initial_y;
        cur_trans(2,3) = initial_z;

        // geometry_msgs::Quaternion q_first;//定义四元数
        // q_first = tf::createQuaternionMsgFromRollPitchYaw(0, 0, initial_yaw * M_PI / 180); //欧拉角转为4元数
        // Eigen::Quaternionf first_Eq(q_first.w, q_first.x, q_first.y, q_first.z);
        // cur_trans.block(0, 0, 3, 3) = first_Eq.matrix();
        cur_trans.block(0, 0, 3, 3)  = init_rotation;
        sub_map_generation(cur_trans,laserCloudMapArray, sub_map);

        //如果说不在数据集的起始位置
        cur_trans = ndtmatch.matching(score,cur_point,sub_map,cur_trans);

        geometry_msgs::Quaternion quaternion;//定义四元数
	    // quaternion=tf::createQuaternionMsgFromRollPitchYaw(roll*M_PI/180.0,pitch*M_PI/180,(yaw+360.0)*M_PI/180); //欧拉角转为4元数 
        Eigen::Quaternionf q(quaternion.w,quaternion.x, quaternion.y, quaternion.z);
        prev_imu_quaternion = q;


        prev_trans = cur_trans;
        publish_odometry(pointcloud_time, cur_trans);

        return cur_trans;
    } 
    //确定imu引入的旋转矩阵
    geometry_msgs::Quaternion cur_quaternion;//定义四元数
    // cur_quaternion=tf::createQuaternionMsgFromRollPitchYaw(roll*M_PI/180.0,pitch*M_PI/180,(yaw+360.0)*M_PI/180); //欧拉角转为4元数
    Eigen::Quaternionf cur_q(cur_quaternion.w,cur_quaternion.x, cur_quaternion.y, cur_quaternion.z);
    cur_imu_quaternion = cur_q;



    Eigen::Matrix3f imu_prev_rotation_matrix;
    Eigen::Matrix3f imu_cur_rotation_matrix;
    Eigen::Matrix3f imu_err_rotation_matrix;
    imu_prev_rotation_matrix =  prev_imu_quaternion.matrix();
    imu_cur_rotation_matrix = cur_imu_quaternion.matrix();
    imu_err_rotation_matrix = imu_cur_rotation_matrix * imu_prev_rotation_matrix.transpose(); //确定旋转增量

    //确定当前的旋转量
    Eigen::Matrix3f prev_rotation;
    Eigen::Matrix3f cur_rotation;
    prev_rotation = prev_trans.block(0,0,3,3);
    cur_rotation  = imu_err_rotation_matrix * prev_rotation;

    cur_trans.block(0,0,3,3) = cur_rotation.matrix();

    //确定当前的平移量
    Eigen::Vector3f err_translation(error_trans(0,3),error_trans(1,3),error_trans(2,3)); //确定平移增量
    Eigen::Vector3f prev_translation(cur_trans(0,3),cur_trans(1,3),cur_trans(2,3)); //先前平移相对于平移增量
    Eigen::Vector3f cur_translation;
    cur_translation = err_translation+prev_translation;

    cur_trans(0,3) =  cur_translation(0);
    cur_trans(1,3) =  cur_translation(1);
    cur_trans(2,3) =  cur_translation(2);

    //当前相对于世界坐标系的位移和旋转

    sub_map_generation(cur_trans,laserCloudMapArray, sub_map);


    //该函数可以读一帧，匹配一帧
        // double matching_start_time = ros::Time::now().toSec();
    cur_trans = ndtmatch.matching(score,cur_point,sub_map,cur_trans);
    // std::cout << "scan matching cost time is " << ros::Time::now().toSec() - matching_start_time << std::endl;

    error_trans.block(0,0,3,3) = cur_trans.block(0,0,3,3) * prev_trans.block(0,0,3,3).transpose();
    error_trans.block<3, 1>(0, 3) = cur_trans.block<3, 1>(0, 3) - prev_trans.block<3, 1>(0, 3);


    double dx = error_trans.block<3, 1>(0, 3).norm();

    double da = std::acos(Eigen::Quaternionf(error_trans.block<3, 3>(0, 0)).w());

    if(dx > 2 || da > M_PI_2||score >= score_threshold)
    {
        Eigen::Matrix3f R;
        R.setIdentity();
        R.matrix()= imu_err_rotation_matrix.matrix();
        Eigen::Matrix3f p_R;
        p_R.matrix() = prev_trans.block(0,0,3,3);
        R = R*p_R;
        cur_trans.block(0,0,3,3) = R;
        ROS_ERROR("score >= score_threshold");
    }

    prev_trans = cur_trans;

    prev_imu_quaternion = cur_imu_quaternion;

    publish_odometry(pointcloud_time, cur_trans);

    return cur_trans;

}

Eigen::Matrix4f matching_scan_to_map_fix_without_imu(double& score,const pcl::PointCloud<PointT>::ConstPtr& cur_point,const pcl::PointCloud<PointT>::ConstPtr& map_point)
{
    NdtMatch ndtmatch;

    if( need_initialization ) {
        prev_trans.setIdentity();
        error_trans.setIdentity();

        sub_map_generation(cur_trans,laserCloudMapArray, sub_map);

        cur_trans = ndtmatch.matching(score, cur_point, sub_map, cur_trans);

        publish_odometry(pointcloud_time, cur_trans);

        prev_trans = cur_trans;

        return cur_trans;
    }

    bool get_imu_pose{false};
    nav_msgs::Odometry imu_odometry_front;
    for(;;){
        if(imu_odometry.size() == 0){
            break;
        }
        imu_odometry_front = imu_odometry.front();
        imu_odometry.pop_front();
        //find latest imu pose
        if( (pointcloud_time - imu_odometry_front.header.stamp).toSec() > 0.013  ){
            continue;
        }
        ros::Time imu_timestamp = imu_odometry_front.header.stamp;
        // if imu pose time is later that pointcloud,break
        if( imu_odometry_front.header.stamp > pointcloud_time){
            break;
        }

        Odometry2Matrix4f(imu_odometry_front,cur_trans);
        get_imu_pose = true;
        break;
    }
    if(!get_imu_pose){
        //NOTE 这些代码是用的匀速模型
        Eigen::Matrix3f err_rotation;
        Eigen::Matrix3f prev_rotation;
        Eigen::Matrix3f cur_rotation;
        prev_rotation = prev_trans.block(0,0,3,3);
        err_rotation = error_trans.block(0,0,3,3);
        cur_rotation  = err_rotation*prev_rotation;

        cur_trans.block(0,0,3,3) = cur_rotation.matrix();

        //确定当前的平移量
        Eigen::Vector3f err_translation(error_trans(0,3),error_trans(1,3),error_trans(2,3)); //确定平移增量
        Eigen::Vector3f prev_translation(cur_trans(0,3),cur_trans(1,3),cur_trans(2,3)); //先前平移相对于平移增量
        Eigen::Vector3f cur_translation;
        cur_translation = err_translation+prev_translation;
        //当前相对于世界坐标系的位移和旋转
        cur_trans(0,3) =  cur_translation(0);
        cur_trans(1,3) =  cur_translation(1);
        cur_trans(2,3) =  cur_translation(2);
    }

    // std::cout <<"has imu pose:  " << get_imu_pose ;

    sub_map_generation(cur_trans,laserCloudMapArray, sub_map);

    // std::cout << "imu prior is x: " << cur_trans(0,3)  << " y: " << cur_trans(1,3) << " z: " << cur_trans(2,3) << std::endl;

    cur_trans = ndtmatch.matching(score,cur_point,sub_map,cur_trans);

    // std::cout << "after ndt is x: " << cur_trans(0,3)  << " y: " << cur_trans(1,3) << " z: " << cur_trans(2,3) << std::endl;

    error_trans.block(0,0,3,3) = cur_trans.block(0,0,3,3) * prev_trans.block(0,0,3,3).transpose();
    error_trans.block<3, 1>(0, 3) = cur_trans.block<3, 1>(0, 3) - prev_trans.block<3, 1>(0, 3);

    prev_trans = cur_trans;

    publish_odometry(pointcloud_time, cur_trans);

    return cur_trans;

}

Eigen::Vector2f offset_p;
Eigen::Matrix2f offset_r;
void localize(){
    if(pointcloud_que.size() >= 2){
        pointcloud_time = pointcloud_que.front()->header.stamp;
        pcl::fromROSMsg(*pointcloud_que.front(), *cur_cloud_Ptr);
        pointcloud_que.pop_front();
    }
    else{
        return ;
    }
    
    ros::Time begin = ros::Time::now();

    bool get_init_pose{false};
    if(need_initialization){
        std::cout << "waiting for initialition" << std::endl;
        //如果得到rviz初始位置
        if(get_rviz_pose == true){
            cur_trans(0, 3) = initial_x;
            cur_trans(1, 3) = initial_y;
            cur_trans(2, 3) = initial_z;
            cur_trans.block(0, 0, 3, 3)  = init_rotation;
            get_rviz_pose = false;
            get_init_pose = true;
            std::cout << "lidar pose is" << std::endl;
            std::cout << cur_trans << std::endl;
            std::cout << "lidar need init , get rviz pose" << std::endl;
        }
        else{
            for(;;)
            {
                nav_msgs::Odometry gps_odometry_front;
                if(gps_odometry.size() == 0){
                    std::cout << "no gps odom" << std::endl;
                    break;
                }
                gps_odometry_front = gps_odometry.front();
                //find latest imu pose
                if( (pointcloud_time - gps_odometry_front.header.stamp).toSec() > 0.15  ){
                    std::cout << "gps odom too early!! pop" << std::endl;
                    gps_odometry.pop_front();
                    continue;
                }
                // if imu pose time is later that pointcloud,break
                if( gps_odometry_front.header.stamp > pointcloud_time){
                    std::cout << "gps odom too old !!" <<  gps_odometry_front.header.stamp.toSec() << " " << pointcloud_time.toSec() << std::endl;
                    break;
                }
                gps_odometry.pop_front();
                Odometry2Matrix4f(gps_odometry_front,cur_trans);
                cur_trans(2,3) = 0;
                cur_trans.block(0,3,2,1) = offset_r * (cur_trans.block(0,3,2,1) - offset_p);
                cur_trans.block(0,0,2,2) = offset_r * cur_trans.block(0,0,2,2);
                get_init_pose = true;
                std::cout << std::endl;
                std::cout << "current prior is" << std::endl;
                std::cout << cur_trans << std::endl;
                break;
            }
            
        }
        if(!get_init_pose){
            return ;
        }
    }
    if(imu_enable)
        matching_scan_to_map_fix(score,cur_cloud_Ptr,point_cloud_map_Ptr);
    else
        matching_scan_to_map_fix_without_imu(score,cur_cloud_Ptr,point_cloud_map_Ptr);
    // Eigen::Matrix4f gps_trans;
    // for(;;)
    // {
    //     nav_msgs::Odometry gps_odometry_front;
    //     if(gps_odometry.size() == 0){
    //         std::cout << "no gps odom" << std::endl;
    //         break;
    //     }
    //     gps_odometry_front = gps_odometry.front();
    //     //find latest imu pose
    //     if( (pointcloud_time - gps_odometry_front.header.stamp).toSec() > 0.15  ){
    //         std::cout << "gps odom too early!! pop" << std::endl;
    //         gps_odometry.pop_front();
    //         continue;
    //     }
    //     // if imu pose time is later that pointcloud,break
    //     if( gps_odometry_front.header.stamp > pointcloud_time){
    //         std::cout << "gps odom too old !!" <<  gps_odometry_front.header.stamp.toSec() << " " << pointcloud_time.toSec() << std::endl;
    //         break;
    //     }
    //     gps_odometry.pop_front();
    //     Odometry2Matrix4f(gps_odometry_front,gps_trans);
    //     gps_trans(2,3) = 0;
    //     std::cout << std::endl;
    //     gps_trans.block(0,3,2,1) = offset_r * (gps_trans.block(0,3,2,1) - offset_p);
    //     gps_trans.block(0,0,2,2) = offset_r * gps_trans.block(0,0,2,2);
    //     std::cout << "lidar angle from gps is" << gps_trans.block(0,0,2,2) << std::endl;
    //     std::cout << "lidar pose is" << cur_trans.block(0,0,2,2) << std::endl;
    // }
    ros::Time end = ros::Time::now();


    std::cout<<"   cost_time: "<< (end - begin).toSec() <<"   score: "<<score<<std::endl;
    need_initialization = false;
    static int count{0};
    if(score > score_threshold){
        count++;
        if(count > 2){
            need_initialization = true;
            ROS_ERROR("lidar localization error!!!");
        }
    }else{
        count = 0;
        need_initialization = false;
    }
    
}

//对地图进行分割 输入：点云地图 输出：分块后的点云地图
void map_segmentation(const pcl::PointCloud<PointT>::ConstPtr& cloud, pcl::PointCloud<PointT>::Ptr laserCloudMapArray[])
{
    int map_points_num = cloud->points.size();
    for (int i = 0; i < map_points_num; i++) {
        
        PointT map_point = cloud->points[i];
        //按50的比例尺缩小，四舍五入，偏移laserCloudCen*的量，计算索引
        int cubeI = int((map_point.x + map_segment_size/2.0) / map_segment_size) + laserCloudCenX;
        int cubeJ = int((map_point.y + map_segment_size/2.0) / map_segment_size) + laserCloudCenY;
        

        if (map_point.x + map_segment_size/2.0 < 0) cubeI--;
        if (map_point.y + map_segment_size/2.0 < 0) cubeJ--;

        if (cubeI >= 0 && cubeI < laserCloudX && 
            cubeJ >= 0 && cubeJ < laserCloudY ) {//只挑选-laserCloudCenWidth * 50.0 < point.x < laserCloudCenWidth * 50.0范围内的点，y和z同理
            //按照尺度放进不同的组，每个组的点数量各异
        int cubeInd = cubeI + laserCloudX * cubeJ; //ind = I + laserCloudX * J
        laserCloudMapArray[cubeInd]->push_back(map_point);
        }
    }
}

//输出读取的点云地图（并对其进行滤波） 依赖的全局参数：downsample_resolution
void map_input(pcl::PointCloud<PointT>::ConstPtr& filtered_map_ptr)
{
    pcl::PointCloud<PointT>::Ptr map_Ptr(new pcl::PointCloud<PointT>);
    //读地图pcd文件，储存到map
    pcl::io::loadPCDFile (map_source, *map_Ptr);
    ROS_INFO("point_cloud_map points is receiving ");

    //显示地图的点云数量
    std::cerr << "map_PointCloud : " << map_Ptr->width * map_Ptr->height
    << " data points (" << pcl::getFieldsList(*map_Ptr) << ")."<<std::endl;

    boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    downsample_filter = voxelgrid;
    filtered_map_ptr = downsample(map_Ptr);
    ROS_INFO("downsample");

    //save downsample map
    if(save_downsample_map){
        int idx = map_source.find_last_of('/');
        std::string filtered_map = map_source.substr(0,idx);
        std::cout << "filtered map is " << filtered_map << std::endl;
        pcl::io::savePCDFile(filtered_map+"/filtered_map.pcd",*filtered_map_ptr);
    }
    //显示滤波后地图的点云数量
    std::cerr << "downsample_map_PointCloud : " << filtered_map_ptr->width * filtered_map_ptr->height
    << " data points (" << pcl::getFieldsList(*filtered_map_ptr) << ")."<<std::endl;

}

//输出读取的点云地图（并对其进行滤波） 依赖的全局参数：downsample_resolution
void map_input_without_downsample(pcl::PointCloud<PointT>::ConstPtr& filtered_map_ptr)
{
    pcl::PointCloud<PointT>::Ptr map_Ptr(new pcl::PointCloud<PointT>);
    //读地图pcd文件，储存到map
    //std::string file_name = "/home/robot/catkin_ws/src/ndt_localization/ndt_localization/lio_sam_filtered.pcd";
    pcl::io::loadPCDFile (map_source, *map_Ptr);
    ROS_INFO("point_cloud_map points is receiving ");

    //显示地图的点云数量
    std::cerr << "map_PointCloud : " << map_Ptr->width * map_Ptr->height
    << " data points (" << pcl::getFieldsList(*map_Ptr) << ")."<<std::endl;

    filtered_map_ptr = map_Ptr;
}

//用rviz初始化激光定位
void initialpose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial_pose_in)
{
    initial_x = initial_pose_in->pose.pose.position.x;
    initial_y = initial_pose_in->pose.pose.position.y;
    initial_z = initial_pose_in->pose.pose.position.z;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(initial_pose_in->pose.pose.orientation, quat);

    double roll, pitch, yaw;//定义存储r\p\y的容器
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换

    geometry_msgs::Quaternion q_first;//定义四元数
    q_first = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw); //欧拉角转为4元数
    Eigen::Quaternionf first_Eq(q_first.w, q_first.x, q_first.y, q_first.z);
    init_rotation = first_Eq.matrix();

    get_rviz_pose = true;
    ROS_INFO("get_rviz_initial_pose");

}

void imu_odometry_cb(const nav_msgs::OdometryConstPtr& msg){
    imu_odometry.push_back(*msg);
}

void gps_odom_cb(const  nav_msgs::OdometryConstPtr& msg){
    gps_odometry.push_back(*msg);
    //维持gps_odometry队列大小小于100
    while(gps_odometry.size() > 300){
        gps_odometry.pop_front();
    }
}

void velodyne_points_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pointcloud_que.push_back(msg);
}

// void gps_cb(const sensor_msgs::NavSatFixConstPtr& msg){
//     double x,y,z;
//     GeographicLib::LocalCartesian geo_converter;
//     geo_converter.Forward(msg->latitude, msg->longitude, msg->altitude, x, y, z);
//     // std::cout << "pose x is " << x  << "  pose y is " << y << std::endl;
// }

//主函数
int main (int argc, char** argv)
{
    ros::init (argc, argv, "pc_fliter");

    ros::NodeHandle nh; 
    ros::NodeHandle nh_p("~");

    tf::TransformBroadcaster odom_broadcaster;
    ros::Time odom_time = ros::Time::now();
    odom_trans.header.stamp = odom_time;

    score = 0;

    //get params
    need_initialization = true;
    get_rviz_pose = true;
    Eigen::Matrix4f lidar2gps;
    //科技园的参数
    // offset_p << 118.74, 101.673;
    // offset_r << 0.0455654,  -0.998961 ,  
    //             0.998961,  0.0455654;
    //学校的参数
    offset_p << 234.611, -155.922;
    offset_r << -0.720988,  0.692947 ,  
                -0.692947,  -0.720988;
    offset_r = offset_r.inverse().eval();
    nh_p.param<double>("score_threshold",score_threshold,2.00);
    nh_p.param<bool>("imu_enable",imu_enable,false);
    nh_p.param<bool>("downsample_map",downsample_map,false);
    nh_p.param<bool>("save_downsample_map",save_downsample_map,false);
    
    std::string map_path;
    nh_p.param("world_frame",world_frame,std::string("world"));
    nh_p.param("lidar_frame",lidar_frame,std::string("lidar_link"));
    nh_p.param("imu_frame",imu_frame,std::string("imu_link"));
    nh_p.param("map_path",map_path,std::string("school"));
    map_source = ros::package::getPath("ndt_imu_localization");
    if(map_path=="school"){
        downsample_resolution = 0.5;
        map_source = map_source + "/map/" + map_path + "/filtered_map.pcd";
        if(downsample_map){
            map_input(point_cloud_map_Ptr);//读入地图并滤波
        }else{
            map_input_without_downsample(point_cloud_map_Ptr);//读入地图并滤波
        }
        Eigen::Quaternionf quaternion(1,0,0,0);
        Eigen::Matrix3f rotation_matrix(quaternion);
        initial_x = 0;
        initial_y = 0;
        initial_z = 0;
        init_rotation = rotation_matrix;
        // while (get_rviz_pose != true)  {  ros::spinOnce();  ros::Duration(0.1).sleep();}
    } 
    else if(map_path=="kitti"){
        downsample_resolution = 0.5;
        map_source = map_source + "/map/" + map_path + "/filtered_map.pcd";
        if(downsample_map){
            map_input(point_cloud_map_Ptr);//读入地图并滤波
        }else{
            map_input_without_downsample(point_cloud_map_Ptr);//读入地图并滤波
        }
        Eigen::Quaternionf quaternion(0.867789133081,0.0127812016969,0.0201438931533,0.496359632684);
        Eigen::Matrix3f rotation_matrix(quaternion);
        initial_x = 0;
        initial_y = 0;
        initial_z = 0;
        init_rotation = rotation_matrix;
        //y p r
        // Eigen::Vector3d eularAngle = rotation_matrix.eulerAngles(2,1,0);
        // initial_yaw = eularAngle(0) * 180 / M_PI;
    }
    else if(map_path=="subt"){
        downsample_resolution = 0.2;
        map_source = map_source + "/map/" + map_path + "/subt.pcd";
        if(downsample_map){
            map_input(point_cloud_map_Ptr);//读入地图并滤波
        }else{
            map_input_without_downsample(point_cloud_map_Ptr);//读入地图并滤波
        }
        Eigen::Quaternionf quaternion(0.677623920649,0.0040147710178,0.000206005245468,-0.73539762125);
        Eigen::Matrix3f rotation_matrix(quaternion);
        initial_x = 26.068;
        initial_y = -27.847;
        initial_z = -0.371;
        init_rotation = rotation_matrix;
        //y p r
        // Eigen::Vector3d eularAngle = rotation_matrix.eulerAngles(2,1,0);
        // initial_yaw = eularAngle(0) * 180 / M_PI;
    }
    else if(map_path=="xiangshan"){
        downsample_resolution = 0.5;
        map_source = map_source + "/map/" + map_path + "/xiangshan4th.pcd";
        if(downsample_map){
            map_input(point_cloud_map_Ptr);//读入地图并滤波
        }else{
            map_input_without_downsample(point_cloud_map_Ptr);//读入地图并滤波
        }
        Eigen::Quaternionf quaternion(1,0,0,0);
        Eigen::Matrix3f rotation_matrix(quaternion);
        initial_x = 0;
        initial_y = 0;
        initial_z = 0;
        init_rotation = rotation_matrix;
    }
    else{
        ROS_WARN("map path error !! please input correct path!!");
        ros::shutdown();
        return 0;
    }
    
    //地图分块
    for (int i = 0; i < laserCloudNum; i++)
	{
		laserCloudMapArray[i].reset(new pcl::PointCloud<PointT>());
	}
    map_segmentation(point_cloud_map_Ptr, laserCloudMapArray);

    ROS_INFO("map segmentation finish");

    ros::Subscriber gps_odom_sub = nh.subscribe("/gps_odom",50,gps_odom_cb);
    ros::Subscriber imu_odometry_sub = nh.subscribe("/odometry/imu",50,&imu_odometry_cb);
    ros::Subscriber pointcloud_sub = nh.subscribe ("filtered_points", 1, velodyne_points_cb);
    sub_map_pub = nh.advertise<sensor_msgs::PointCloud2>("sub_map_points", 1);
    pubPoseForKITTI = nh.advertise<nav_msgs::Odometry>("/odom", 1);
    ros::Subscriber sub_initial_pose = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped> 
                                    ("/initialpose", 50, initialpose_cb);
    
    
    ros::Rate r(100.0);
    while(nh.ok()){
        ros::spinOnce();
        localize();
        if(odom_trans.header.stamp.toSec() != odom_time.toSec())
        {
            odom_broadcaster.sendTransform(odom_trans);
            odom_time = odom_trans.header.stamp;
        }
        r.sleep();
    }
    

    return 0;
}

