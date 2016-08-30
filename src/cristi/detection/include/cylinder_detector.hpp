#ifndef CYLINDER_DETECTOR_H_
#define CYLINDER_DETECTOR_H_

#include "bottle_detector.hpp"
#include "detection/ParametersConfig.h"

#include <pcl/visualization/pcl_visualizer.h>

#define MAXIMUM_NUMBER_OF_CLUSTERS 10

void reconfigureCallback(detection::ParametersConfig &config, uint32_t level);

class CylinderDetector: public BottleDetector {
private:
    ros::NodeHandle nh;
    pcl::visualization::PCLVisualizer::Ptr visualizer;
    ros::Publisher publisher_cloud_removed_outliers;
    ros::Publisher publisher_cloud_without_planes;
    ros::Publisher publisher_cloud_projected;
    ros::Publisher publisher_clusters[MAXIMUM_NUMBER_OF_CLUSTERS];
    ros::Publisher publisher_rviz_marker;
    geometry_msgs::Pose findBottlePose(pcl::PointCloud<PointType>::Ptr cylinder,
    		pcl::ModelCoefficients::Ptr coefficients,
    		pcl::PointCloud<PointType>::Ptr projected_points);
    Bottle detectUsingPlaneExtraction(pcl::PointCloud<PointType>::Ptr cloud);
    Bottle detectUsingClustering(pcl::PointCloud<PointType>::Ptr cloud);
    void showCloudWithNormals(pcl::PointCloud<PointType>::Ptr cloud,
            const char cloud_id[], pcl::PointCloud<pcl::Normal>::Ptr normals,
            const char normals_id[]);
public:
    CylinderDetector(pcl::visualization::PCLVisualizer::Ptr visualizer);
    Bottle detect(pcl::PointCloud<PointType>::Ptr cloud);
};

#endif
