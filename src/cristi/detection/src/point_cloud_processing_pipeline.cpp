#include "point_cloud_processing_pipeline.hpp"
#include "point_cloud_algorithms.h"

// For pcl::copyPointCloud
#include <pcl/io/io.h>

// For working with indices
#include <pcl/filters/extract_indices.h>

PointCloudProcessingPipeline::PointCloudProcessingPipeline(
		const CloudProcessingParameters& processing_parameters) :
		cloud_1(new pcl::PointCloud<PointType>), cloud_2(
				new pcl::PointCloud<PointType>), normals_1(
				new pcl::PointCloud<pcl::Normal>), normals_2(
				new pcl::PointCloud<pcl::Normal>) {

	params = processing_parameters;

	cloud_current = &cloud_1;
	cloud_tmp = &cloud_2;
	normals_current = &normals_1;
	normals_tmp = &normals_2;
}

void PointCloudProcessingPipeline::setInputCloud(
		pcl::PointCloud<PointType>::Ptr cloud_input) {
	pcl::copyPointCloud(*cloud_input, **cloud_current);
}

void PointCloudProcessingPipeline::setInputCloud(
        pcl::PointCloud<PointType>::Ptr cloud_input,
        const pcl::PointIndices &indices) {
    pcl::copyPointCloud(*cloud_input, indices, **cloud_current);
}

void PointCloudProcessingPipeline::setInputNormals(pcl::PointCloud<pcl::Normal>::Ptr normals_input) {
    pcl::copyPointCloud(*normals_input, **normals_current);
}

void PointCloudProcessingPipeline::setInputNormals(
        pcl::PointCloud<pcl::Normal>::Ptr normals_input,
        const pcl::PointIndices &indices) {
    pcl::copyPointCloud(*normals_input, indices, **normals_current);
}

pcl::PointCloud<PointType>::Ptr PointCloudProcessingPipeline::getCurrentCloud() {
	pcl::PointCloud<PointType>::Ptr cloud_current_copy(
			new pcl::PointCloud<PointType>);
	pcl::copyPointCloud(**cloud_current, *cloud_current_copy);
	return cloud_current_copy;
}

pcl::PointCloud<pcl::Normal>::Ptr PointCloudProcessingPipeline::getCurrentNormals() {
	pcl::PointCloud<pcl::Normal>::Ptr normals_current_copy(
			new pcl::PointCloud<pcl::Normal>);
	pcl::copyPointCloud(**normals_current, *normals_current_copy);
	return normals_current_copy;
}

void PointCloudProcessingPipeline::pipelineSwap() {
	pcl::PointCloud<PointType>::Ptr *tmp;
	tmp = cloud_current;
	cloud_current = cloud_tmp;
	cloud_tmp = tmp;
	(*cloud_tmp)->clear();
}

void PointCloudProcessingPipeline::pipelineSwapNormals() {
	pcl::PointCloud<pcl::Normal>::Ptr *tmp;
	tmp = normals_current;
	normals_current = normals_tmp;
	normals_tmp = tmp;
	(*normals_tmp)->clear();
}

inline void extractIndicesDirectly(pcl::PointCloud<PointType>::Ptr cloud,
        pcl::PointIndices::Ptr point_indices, bool negative) {
    pcl::ExtractIndices<PointType> extract_indices;
    extract_indices.setInputCloud(cloud);
    extract_indices.setIndices(point_indices);
    extract_indices.setNegative(negative);
    extract_indices.filterDirectly(cloud);
}

void PointCloudProcessingPipeline::filterZ(double z_min, double z_max) {
	pcl::PointIndices::Ptr point_indices = getPointsInInterval(*cloud_current, "z", z_min, z_max);
	extractIndicesDirectly(*cloud_current, point_indices, false);
}

void PointCloudProcessingPipeline::removeOutliers() {
    pcl::PointIndices::Ptr point_indices = getOutliers(*cloud_current,
			params.statistical_outlier_mean_k,
			params.statistical_outlier_mul_tresh);
    extractIndicesDirectly(*cloud_current, point_indices, false);
}

void PointCloudProcessingPipeline::computeNormals() {
    *normals_current = ::computeNormalsOfOrganizedCloud(*cloud_current,
			params.normal_estimation_radius_search);
}

void PointCloudProcessingPipeline::removeNaNs() {
	::removeNaNs(*cloud_current, *cloud_tmp);
	pipelineSwap();
}

void PointCloudProcessingPipeline::removeNormalNaNs() {
	::removeNormalNaNs(*cloud_current, *normals_current, *cloud_tmp,
			*normals_tmp);
	pipelineSwap();
	pipelineSwapNormals();
}

void PointCloudProcessingPipeline::removeNaNsWithNormals() {
    ::removeNaNs(*cloud_current, *normals_current, *cloud_tmp, *normals_tmp);
    pipelineSwap();
    pipelineSwapNormals();
    this->removeNormalNaNs();
}

bool PointCloudProcessingPipeline::removePlaneOfOrganizedCloud() {
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr point_indices = findPlaneOfOrganizedCloud(*cloud_current, coefficients,
			params.plane_segmentation_distance_treshold);
	if (point_indices->indices.size() > params.plane_segmentation_minimum_points) {
	    extractIndicesDirectly(*cloud_current, point_indices, true);
	    return true;
	}
	return false;
}

bool PointCloudProcessingPipeline::getCylinder(
		pcl::ModelCoefficients::Ptr coefficients) {
	bool success = findRadiusBasedModel(*cloud_current, *normals_current, *cloud_tmp,
			coefficients, pcl::SACMODEL_CYLINDER, params.cylinder_segmentation_distance_treshold,
			params.cylinder_min_radius, params.cylinder_max_radius);
	if (success) {
		pipelineSwap();
	}
	return success;
}

bool PointCloudProcessingPipeline::getSphere(
        pcl::ModelCoefficients::Ptr coefficients) {
    bool success = findRadiusBasedModel(*cloud_current, *normals_current, *cloud_tmp,
            coefficients, pcl::SACMODEL_SPHERE, params.sphere_segmentation_distance_treshold,
            params.sphere_min_radius, params.sphere_max_radius);
    if (success) {
        pipelineSwap();
    }
    return success;
}

void PointCloudProcessingPipeline::smoothen() {
	::smoothen(*cloud_current, *cloud_tmp, params.smoothing_search_radius);
	pipelineSwap();
}

void PointCloudProcessingPipeline::projectOnLine(double point_x, double point_y, double point_z,
			double direction_x, double direction_y, double direction_z) {
	::projectOnLine(*cloud_current, point_x, point_y, point_z, direction_x,
			direction_y, direction_z, *cloud_tmp);
	pipelineSwap();
}

int PointCloudProcessingPipeline::cloudSize() {
	return (*cloud_current)->size();
}
