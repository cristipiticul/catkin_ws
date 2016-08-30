#include "point_cloud_algorithms.h"

#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

// For normal estimation:
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/organized.h>

// For clustering:
#include <pcl/segmentation/region_growing.h>

// For RANSAC:
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

// For smoothing:
#include <pcl/surface/mls.h>

// For downsampling:
#include <pcl/filters/voxel_grid.h>

// For projecting on line:
#include <pcl/filters/project_inliers.h>

#define MAX_ITERATIONS 100


pcl::PointIndices::Ptr getPointsInInterval(pcl::PointCloud<PointType>::Ptr cloud,
        const std::string field_name,
        const double value_min,
        const double value_max) {
    pcl::PointIndices::Ptr point_indices(new pcl::PointIndices);
    pcl::PassThrough<PointType> filter;
    filter.setFilterFieldName(field_name);
    filter.setFilterLimits(value_min, value_max);
    filter.setInputCloud(cloud);
    filter.filter(point_indices->indices);
    return point_indices;
}



pcl::PointIndices::Ptr getOutliers(pcl::PointCloud<PointType>::Ptr cloud,
		const int number_of_neighbors,
		const double treshold_mean_multiplier) {
    pcl::PointIndices::Ptr point_indices(new pcl::PointIndices);
    pcl::StatisticalOutlierRemoval<PointType> filter;
    filter.setInputCloud(cloud);
    filter.setMeanK(number_of_neighbors);
    filter.setStddevMulThresh(treshold_mean_multiplier);
    filter.filter(point_indices->indices);
    return point_indices;
}

pcl::PointCloud<pcl::Normal>::Ptr computeNormalsOfOrganizedCloud(pcl::PointCloud<PointType>::Ptr cloud,
        const double radius_search) {
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimationOMP<PointType, pcl::Normal> normal_estimation;
    pcl::search::OrganizedNeighbor<PointType>::Ptr tree(new pcl::search::OrganizedNeighbor<PointType>);
    normal_estimation.setInputCloud(cloud);
    normal_estimation.setRadiusSearch(radius_search);
    normal_estimation.setSearchMethod(tree);
    normal_estimation.compute(*normals);
    return normals;
}

void removeNaNs(pcl::PointCloud<PointType>::Ptr cloud,
        pcl::PointCloud<PointType>::Ptr result) {
    std::vector<int> good_indices;
    pcl::removeNaNFromPointCloud(*cloud, *result, good_indices);
    result->is_dense = false;
}

/**
 * Removes the NaNs of the point cloud, and also deletes the
 * normals for the points that had NaN value. It does not
 * remove the NaNs in the normals point cloud (to do that,
 * use removeNormalNaNs).
 */
void removeNaNs(pcl::PointCloud<PointType>::Ptr cloud,
        pcl::PointCloud<pcl::Normal>::Ptr normals,
        pcl::PointCloud<PointType>::Ptr result,
        pcl::PointCloud<pcl::Normal>::Ptr result_normals) {
    pcl::PointIndices indices;
    pcl::removeNaNFromPointCloud(*cloud, *result, indices.indices);
    pcl::copyPointCloud(*normals, indices.indices, *result_normals);
    result->is_dense = false;
    result_normals->is_dense = false;
}

void removeNormalNaNs(pcl::PointCloud<PointType>::Ptr cloud,
        pcl::PointCloud<pcl::Normal>::Ptr normals,
        pcl::PointCloud<PointType>::Ptr cloud_result,
        pcl::PointCloud<pcl::Normal>::Ptr normals_result) {
    pcl::PointIndices::Ptr good_indices(new pcl::PointIndices);
    pcl::removeNaNNormalsFromPointCloud(*normals, *normals_result, good_indices->indices);
    normals_result->is_dense = false;

    pcl::ExtractIndices<PointType> extract;
    extract.setIndices(good_indices);
    extract.setInputCloud(cloud);
    extract.setNegative(false);
    extract.filter(*cloud_result);
    cloud_result->is_dense = false;
}

std::vector<pcl::PointIndices> getClustersIndices(
        pcl::PointCloud<PointType>::Ptr cloud,
        pcl::PointCloud<pcl::Normal>::Ptr normals,
        const int cluster_min_size,
        const int cluster_max_size,
        const int number_of_neighbors,
        const double smoothness_treshold,
        const double curvature_treshold) {
    std::vector<pcl::PointIndices> clusters_indices;
    pcl::search::KdTree<PointType>::Ptr kdtree(
            new pcl::search::KdTree<PointType>);
    kdtree->setInputCloud(cloud);

    // Region growing clustering object.
    pcl::RegionGrowing<PointType, pcl::Normal> clustering;
    clustering.setMinClusterSize(cluster_min_size);
    clustering.setMaxClusterSize(cluster_max_size);
    clustering.setSearchMethod(kdtree);
    clustering.setNumberOfNeighbours(number_of_neighbors);
    clustering.setInputCloud(cloud);
    clustering.setInputNormals(normals);
    // Set the angle in radians that will be the smoothness threshold
    // (the maximum allowable deviation of the normals).
    clustering.setSmoothnessThreshold(smoothness_treshold / 180.0 * M_PI);
    // Set the curvature threshold. The disparity between curvatures will be
    // tested after the normal deviation check has passed.
    clustering.setCurvatureThreshold(curvature_treshold);

    clustering.extract(clusters_indices);
    return clusters_indices;
}

pcl::PointIndices::Ptr findPlaneIndices(pcl::PointCloud<PointType>::Ptr cloud,
        pcl::ModelCoefficients::Ptr coefficients,
        const double distance_treshold) {
    pcl::PointIndices::Ptr plane_indices(new pcl::PointIndices);
    pcl::SACSegmentation<PointType> segmentation;
    segmentation.setInputCloud(cloud);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setMaxIterations(MAX_ITERATIONS);
    // Maximum distance allowed to the model
    segmentation.setDistanceThreshold(distance_treshold);
//    segmentation.setOptimizeCoefficients(true);

    segmentation.segment(*plane_indices, *coefficients);

    return plane_indices;
}

pcl::PointIndices::Ptr findPlaneOfOrganizedCloud(pcl::PointCloud<PointType>::Ptr cloud,
        pcl::ModelCoefficients::Ptr coefficients,
        const double distance_treshold) {
    pcl::PointIndices::Ptr final_plane_indices(new pcl::PointIndices); // The point indices in the _initial_ cloud (with NaNs)
    pcl::PointIndices::Ptr not_nan_indices(new pcl::PointIndices);
    pcl::PointIndices::Ptr plane_indices;
    pcl::PointCloud<PointType>::Ptr cloud_without_nans(new pcl::PointCloud<PointType>);

    pcl::removeNaNFromPointCloud(*cloud, *cloud_without_nans, not_nan_indices->indices);
    plane_indices = findPlaneIndices(cloud_without_nans, coefficients, distance_treshold);
    for (std::vector<int>::iterator it = plane_indices->indices.begin();
            it != plane_indices->indices.end(); it++) {
        final_plane_indices->indices.push_back(not_nan_indices->indices[*it]);
    }

    return final_plane_indices;
}


bool findRadiusBasedModel(pcl::PointCloud<PointType>::Ptr cloud,
        pcl::PointCloud<pcl::Normal>::Ptr normals,
        pcl::PointCloud<PointType>::Ptr result_model,
        pcl::ModelCoefficients::Ptr coefficients,
        const int model_type,
		const double distance_treshold,
		const double min_radius,
		const double max_radius) {
    // Create the segmentation object.
    pcl::SACSegmentationFromNormals<PointType, pcl::Normal> segmentation;

    segmentation.setInputCloud(cloud);
    segmentation.setInputNormals(normals);
    segmentation.setModelType(model_type);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setDistanceThreshold(
            distance_treshold);
    segmentation.setNormalDistanceWeight(0.1);
    segmentation.setMaxIterations(MAX_ITERATIONS);
    segmentation.setOptimizeCoefficients(true);
    segmentation.setRadiusLimits(min_radius,
            max_radius);

    pcl::PointIndices inlierIndices;
    segmentation.segment(inlierIndices, *coefficients);
    if (inlierIndices.indices.size() == 0) {
        return false;
    }
    pcl::copyPointCloud(*cloud, inlierIndices, *result_model);
    return true;
}

void smoothen(pcl::PointCloud<PointType>::Ptr cloud,
		pcl::PointCloud<PointType>::Ptr result,
		const double search_radius) {
	// Smoothing object (we choose what point types we want as input and output).
	pcl::MovingLeastSquares<PointType, PointType> filter;
	filter.setInputCloud(cloud);
	// Use all neighbors in this radius.
	filter.setSearchRadius(search_radius);
	filter.setPolynomialFit(false);
	filter.setComputeNormals(false);
	// kd-tree object for performing searches.
	pcl::search::KdTree<PointType>::Ptr kdtree2;
	filter.setSearchMethod(kdtree2);

	filter.process(*result);
}

void downsamplePointCloud(pcl::PointCloud<PointType>::Ptr cloud,
		pcl::PointCloud<PointType>::Ptr result,
		const double downsampling_size) {
	pcl::VoxelGrid<PointType> grid;
	grid.setInputCloud(cloud);
	grid.setLeafSize(downsampling_size, downsampling_size, downsampling_size);
	grid.filter(*result);
}


geometry_msgs::Point computeCentroid(pcl::PointCloud<PointType>::Ptr cloud) {
	geometry_msgs::Point result;
	Eigen::Vector4f centroid;

	pcl::compute3DCentroid(*cloud, centroid);
	result.x = centroid[0];
	result.y = centroid[1];
	result.z = centroid[2];

	return result;
}

void projectOnLine(pcl::PointCloud<PointType>::Ptr cloud, double point_x,
		double point_y, double point_z, double direction_x, double direction_y,
		double direction_z, pcl::PointCloud<PointType>::Ptr result) {
	pcl::ModelCoefficients::Ptr line(new pcl::ModelCoefficients);
	line->values.resize(6);

	line->values[0] = point_x;
	line->values[1] = point_y;
	line->values[2] = point_z;
	line->values[3] = direction_x;
	line->values[4] = direction_y;
	line->values[5] = direction_z;

	pcl::ProjectInliers<PointType> proj;
	proj.setModelType(pcl::SACMODEL_LINE);
	proj.setInputCloud(cloud);
	proj.setModelCoefficients(line);
	proj.filter(*result);
}
