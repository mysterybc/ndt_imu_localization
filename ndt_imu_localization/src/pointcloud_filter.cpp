#include "pointcloud_filter.h"

// NOTE 默认只有downsample_filter
FilterParam::FilterParam(){
    ros::NodeHandle nh("~");

    //distance filter param
    if(nh.param("filter_distance",FILTER_DISTANCE,false)){
        if(!nh.getParam("max_distance",max_distance)){
            ROS_WARN("LACK OF PARAM!");
            ros::shutdown();
            return ;
        }
        if(!nh.getParam("min_distance",min_distance)){
            ROS_WARN("LACK OF PARAM!");
            ros::shutdown();
            return ;
        }
    }

    //height filter param
    if(nh.param("filter_height",FILTER_HEIGHT,false)){
        if(!nh.getParam("max_height",max_height)){
            ROS_WARN("LACK OF PARAM!");
            ros::shutdown();
            return ;
        }
        if(!nh.getParam("min_height",min_height)){
            ROS_WARN("LACK OF PARAM!");
            ros::shutdown();
            return ;
        }
    }

    //downsample filter 
    if(nh.param("filter_downsample",FILTER_DOWNSAMPLE,true)){
        if(!nh.getParam("downsample_resolution",downsample_resolution)){
            ROS_WARN("LACK OF PARAM!");
            ros::shutdown();
        }
    }

    //outlier filter param
    if(nh.param("filter_outlier",FILTER_OUTLIER,false)){
        if(!nh.getParam("outlier_radius",outlier_radius)){
            ROS_WARN("LACK OF PARAM!");
            ros::shutdown();
        }
        if(!nh.getParam("outlier_neighbors",outlier_neighbors)){
            ROS_WARN("LACK OF PARAM!");
            ros::shutdown();
        }
    }

    outlier_filter = std::make_shared<pcl::RadiusOutlierRemoval<PointT>>();
    outlier_filter->setRadiusSearch(outlier_radius);
    outlier_filter->setMinNeighborsInRadius(outlier_neighbors);

    downsample_filter = std::make_shared<pcl::VoxelGrid<PointT>>();
    downsample_filter->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);


    ROS_INFO("Pointcloud Filter init success!!");
}


void DistanceFilter(pcl::PointCloud<PointT>::Ptr pointcloud_in, 
                    pcl::PointCloud<PointT>::Ptr pointcloud_out,
                    FilterParam& param)
{
  pointcloud_out->reserve(pointcloud_in->size());

    std::copy_if(pointcloud_in->begin(), pointcloud_in->end(), std::back_inserter(pointcloud_out->points),
      [&](const PointT& p) {
        double d = p.getVector3fMap().norm();
        return d > param.min_distance && d < param.max_distance;
      }
    );

    pointcloud_out->width = pointcloud_out->size();
    pointcloud_out->height = 1;
    pointcloud_out->is_dense = false;
    pointcloud_out->header = pointcloud_in->header;

}

void HeightFilter(pcl::PointCloud<PointT>::Ptr pointcloud_in, 
                    pcl::PointCloud<PointT>::Ptr pointcloud_out,
                    FilterParam& param)
{
    pointcloud_out->reserve(pointcloud_in->size());

    std::copy_if(pointcloud_in->begin(), pointcloud_in->end(), std::back_inserter(pointcloud_out->points),
      [&](const PointT& p) {
        double d = p.z;
        return d > param.min_height && d < param.max_height;
      }
    );

    pointcloud_out->width = pointcloud_out->size();
    pointcloud_out->height = 1;
    pointcloud_out->is_dense = false;
    pointcloud_out->header = pointcloud_in->header;
}

void DownSampleFilter(pcl::PointCloud<PointT>::Ptr pointcloud_in, 
                    pcl::PointCloud<PointT>::Ptr pointcloud_out,
                    FilterParam& param)
{
    if(!param.downsample_filter) {
        return ;
    }

    param.downsample_filter->setInputCloud(pointcloud_in->makeShared());
    param.downsample_filter->filter(*pointcloud_out);
    pointcloud_out->header = pointcloud_in->header;
}

void OutlierFilter(pcl::PointCloud<PointT>::Ptr pointcloud_in, 
                    pcl::PointCloud<PointT>::Ptr pointcloud_out,
                    FilterParam& param)
{
    if(!param.outlier_filter) {
      return ;
    }
    
    param.outlier_filter->setInputCloud(pointcloud_in->makeShared());
    param.outlier_filter->filter(*pointcloud_out);
    pointcloud_out->header = pointcloud_in->header;

}

void PointCloudFilter(pcl::PointCloud<PointT>::Ptr pointcloud_in, 
                    pcl::PointCloud<PointT>::Ptr pointcloud_out,
                    FilterParam& param)
{
    // if(param.FILTER_HEIGHT){
    //     HeightFilter(pointcloud_in,pointcloud_out,param);
    //     pointcloud_in = pointcloud_out;
    // }
    
    // if(param.FILTER_DISTANCE){
    //     DistanceFilter(pointcloud_in,pointcloud_out,param);
    //     pointcloud_in = pointcloud_out;
    // }

    if(param.FILTER_DOWNSAMPLE){
        DownSampleFilter(pointcloud_in,pointcloud_out,param);
        pointcloud_in = pointcloud_out;
    }

    if(param.FILTER_OUTLIER){
        OutlierFilter(pointcloud_in,pointcloud_out,param);
        pointcloud_in = pointcloud_out;
    }
}



