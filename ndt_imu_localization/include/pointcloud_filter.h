#pragma once
//c++
#include <iostream>
//ros
#include <ros/ros.h>
//pcl
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>


typedef pcl::PointXYZ PointT;
struct FilterParam{
    FilterParam();

    bool FILTER_DISTANCE;
    bool FILTER_HEIGHT;
    bool FILTER_OUTLIER;
    bool FILTER_DOWNSAMPLE;

    double max_distance;
    double min_distance;
    double max_height;
    double min_height;
    double downsample_resolution;
    double outlier_radius;
    double outlier_neighbors;

    std::shared_ptr<pcl::RadiusOutlierRemoval<PointT>> outlier_filter;
    std::shared_ptr<pcl::VoxelGrid<PointT>> downsample_filter;
};

void PointCloudFilter(pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<PointT>::Ptr, FilterParam&);
void DistanceFilter(pcl::PointCloud<PointT>::Ptr  , pcl::PointCloud<PointT>::Ptr, FilterParam&);
void HeightFilter(pcl::PointCloud<PointT>::Ptr    , pcl::PointCloud<PointT>::Ptr, FilterParam&);
void DownSampleFilter(pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<PointT>::Ptr, FilterParam&);
void OutlierFilter(pcl::PointCloud<PointT>::Ptr   , pcl::PointCloud<PointT>::Ptr, FilterParam&);