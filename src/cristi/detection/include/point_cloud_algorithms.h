#ifndef POINT_CLOUD_ALGORITHMS_H_
#define POINT_CLOUD_ALGORITHMS_H_

#include "common.h"

// For ModelCoefficients class
#include <pcl/segmentation/sac_segmentation.h>

// For centroid:
#include <geometry_msgs/Point.h>

pcl::PointIndices::Ptr getPointsInInterval(pcl::PointCloud<PointType>::Ptr cloud,
		const std::string field_name,
		const double value_min,
		const double value_max);

pcl::PointIndices::Ptr getOutliers(pcl::PointCloud<PointType>::Ptr cloud,
        const int number_of_neighbors,
        const double treshold_mean_multiplier);

pcl::PointCloud<pcl::Normal>::Ptr computeNormalsOfOrganizedCloud(
        pcl::PointCloud<PointType>::Ptr cloud, const double radius_search);

void removeNaNs(pcl::PointCloud<PointType>::Ptr cloud,
        pcl::PointCloud<PointType>::Ptr result);

void removeNaNs(pcl::PointCloud<PointType>::Ptr cloud,
        pcl::PointCloud<pcl::Normal>::Ptr normals,
        pcl::PointCloud<PointType>::Ptr result,
        pcl::PointCloud<pcl::Normal>::Ptr result_normals);

/**
 * Removes the NaN normals and also the corresponding points from the cloud.
 */
void removeNormalNaNs(pcl::PointCloud<PointType>::Ptr cloud,
        pcl::PointCloud<pcl::Normal>::Ptr normals,
        pcl::PointCloud<PointType>::Ptr cloud_result,
        pcl::PointCloud<pcl::Normal>::Ptr normals_result);

std::vector<pcl::PointIndices> getClustersIndices(
        pcl::PointCloud<PointType>::Ptr cloud,
        pcl::PointCloud<pcl::Normal>::Ptr normals,
        const int cluster_min_size,
        const int cluster_max_size,
        const int number_of_neighbors,
        const double smoothness_treshold,
        const double curvature_treshold);

pcl::PointIndices::Ptr findPlaneOfOrganizedCloud(pcl::PointCloud<PointType>::Ptr cloud,
        pcl::ModelCoefficients::Ptr coefficients,
        const double distance_treshold);

bool findRadiusBasedModel(pcl::PointCloud<PointType>::Ptr cloud,
        pcl::PointCloud<pcl::Normal>::Ptr normals,
        pcl::PointCloud<PointType>::Ptr result_model,
        pcl::ModelCoefficients::Ptr coefficients,
        const int model_type,
        const double distance_treshold,
        const double min_radius,
        const double max_radius);

void smoothen(pcl::PointCloud<PointType>::Ptr cloud,
		pcl::PointCloud<PointType>::Ptr result,
		const double search_radius);



void downsamplePointCloud(pcl::PointCloud<PointType>::Ptr cloud,
		pcl::PointCloud<PointType>::Ptr result,
		const double downsampling_size);

geometry_msgs::Point computeCentroid(pcl::PointCloud<PointType>::Ptr cloud);

void projectOnLine(pcl::PointCloud<PointType>::Ptr cloud, double point_x,
		double point_y, double point_z, double direction_x, double direction_y,
		double direction_z, pcl::PointCloud<PointType>::Ptr result);

#endif
