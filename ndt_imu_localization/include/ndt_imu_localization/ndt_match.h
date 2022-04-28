//
// Created by robot on 2020/9/15.
//

#ifndef NDT_MATCH_H
#define NDT_MATCH_H

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pclomp/ndt_omp.h>

typedef pcl::PointXYZ PointT;

class NdtMatch {

public:
    NdtMatch():
    numThreads_(omp_get_max_threads()),
    epsilon_(0.01),
    maximumIterations_(30),
    resolution_(1.0),
    neighborhoodSearchMethod_(pclomp::DIRECT7)
    {};

    ~NdtMatch(){};

    Eigen::Matrix4f matching(double& score,const pcl::PointCloud<PointT>::ConstPtr& source_point,const pcl::PointCloud<PointT>::ConstPtr& target_point,const Eigen::Matrix4f trans)
        {
            Eigen::Matrix4f esti_trans;

            esti_trans.setIdentity();

            boost::shared_ptr<pclomp::NormalDistributionsTransform<PointT, PointT>> ndt_omp(new pclomp::NormalDistributionsTransform<PointT, PointT>());

            ndt_omp->setNumThreads(omp_get_max_threads());
            ndt_omp->setTransformationEpsilon(0.01);
            ndt_omp->setMaximumIterations(30);
            ndt_omp->setResolution(1.0);
            ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);

            boost::shared_ptr<pcl::Registration<PointT, PointT>> registration;

            registration = ndt_omp;

            registration->setInputTarget(target_point);
            registration->setInputSource(source_point);


            pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
            registration->align(*aligned,trans);


            if(!registration->hasConverged()) {
                ROS_INFO("scan matching has not converged!!");
                return trans;
            }

            esti_trans = registration->getFinalTransformation();

//    std::cout << "Normal Distributions Transform has converged:/n" << esti_trans
//            << " score: " << registration->getFitnessScore() << std::endl;

            score = registration->getFitnessScore(0.5);

            return esti_trans;
        };

private:

    int numThreads_;//ndtomp使用的线程数
    double epsilon_;//ndtomp 偏差 达到偏差即停止
    int maximumIterations_;//ndtomp 最大迭代次数
    double resolution_;//ndt使用的体素分辨率
    pclomp::NeighborSearchMethod neighborhoodSearchMethod_; //ndtomp提供的搜索最近邻算法，有以下集中选择
    //    KDTREE
    //    DIRECT26
    //    DIRECT7
    //    DIRECT1



};


#endif //NDT_LOCALIZATION_NDT_MATCH_H
