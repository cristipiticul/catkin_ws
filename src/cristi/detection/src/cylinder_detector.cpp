// Many thanks to http://robotica.unileon.es/mediawiki/index.php/PCL/OpenNI_tutorial_3:_Cloud_processing_%28advanced%29
#include "ros/ros.h"
#include <eigen_conversions/eigen_msg.h>

#include <pcl/filters/project_inliers.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

#include "point_cloud_algorithms.h"
#include "point_cloud_processing_pipeline.hpp"

// For Rviz markers:
#include <visualization_msgs/Marker.h>

#include <detection/ParametersConfig.h>

#include "cylinder_detector.hpp"

#include <geometry_msgs/Vector3.h>

CloudProcessingParameters global_parameters;

geometry_msgs::Quaternion quaternionFromTwoVectors(tf::Vector3 from,
		tf::Vector3 to) {
	geometry_msgs::Quaternion result;
	tf::Vector3 orthogonal = from.cross(to);
	orthogonal.normalize();
	tf::Quaternion q(orthogonal, -1.0 * acos(from.dot(to)));
	q.normalize();
	tf::quaternionTFToMsg(q, result);
	return result;
}

void publishPointCloud(ros::Publisher publisher,
		pcl::PointCloud<PointType>::Ptr cloud) {
	sensor_msgs::PointCloud2 msg_cloud;
	pcl::toROSMsg(*cloud, msg_cloud);
	msg_cloud.header.frame_id = "camera_rgb_optical_frame";
	msg_cloud.header.stamp = ros::Time::now();
	publisher.publish(msg_cloud);
}

Bottle getResultFromPipelineWithCylinder(PointCloudProcessingPipeline pipeline,
		pcl::ModelCoefficients::Ptr coefficients) {
	Bottle result;
	result.success = true;

	pcl::PointCloud<PointType>::Ptr cylinder = pipeline.getCurrentCloud();
	pcl::toROSMsg(*cylinder, result.cloud);
	pipeline.projectOnLine(coefficients->values[0], coefficients->values[1],
			coefficients->values[2], coefficients->values[3],
			coefficients->values[4], coefficients->values[5]);

	pcl::PointCloud<PointType>::Ptr projected_points =
			pipeline.getCurrentCloud();

    double radius = coefficients->values[6];
	ROS_WARN("Cylinder radius: %f", radius);

	result.pose.position = computeCentroid(projected_points);

    double dx = coefficients->values[3];
    double dy = coefficients->values[4];
    double dz = coefficients->values[5];

    if (dy < 0) {
        dx = -dx;
        dy = -dy;
        dz = -dz;
    }

	result.pose.orientation = quaternionFromTwoVectors(
			tf::Vector3(dx, dy, dz), tf::Vector3(0.0, 1.0, 0.0));

	return result;
}

Bottle getResultFromPipelineWithSphere(PointCloudProcessingPipeline pipeline,
        pcl::ModelCoefficients::Ptr coefficients) {
    Bottle result;
    result.success = true;

    pcl::PointCloud<PointType>::Ptr sphere = pipeline.getCurrentCloud();
    pcl::toROSMsg(*sphere, result.cloud);

    result.pose.position.x = coefficients->values[0];
    result.pose.position.y = coefficients->values[1];
    result.pose.position.z = coefficients->values[2];
    result.pose.orientation.w = 1.0;

    return result;
}

void reconfigureCallback(detection::ParametersConfig &config, uint32_t level) {
	if (config.groups.statistical_outlier.mean_k != 0) {
		ROS_INFO("Reconfiguring...");

		switch (config.detection_method) {
		case 0:
		    global_parameters.detection_method = EXTRACTION;
		    break;
		case 1:
		    global_parameters.detection_method = CLUSTERING;
		    break;
		}

		switch (config.shape) {
		case 0:
		    global_parameters.shape = CYLINDER;
		    break;
		case 1:
            global_parameters.shape = SPHERE;
		    break;
		}

		global_parameters.normal_estimation_radius_search =
				config.groups.normal_estimation.radius_search;
		global_parameters.statistical_outlier_mean_k =
				config.groups.statistical_outlier.mean_k;
		global_parameters.statistical_outlier_mul_tresh =
				config.groups.statistical_outlier.mul_tresh;

		global_parameters.plane_segmentation_distance_treshold =
				config.groups.segmentation_plane.plane_segmentation_distance_treshold;
		global_parameters.plane_segmentation_minimum_points =
				config.groups.segmentation_plane.plane_segmentation_minimum_points;

		global_parameters.cylinder_segmentation_distance_treshold =
				config.groups.segmentation_cylinder.cylinder_segmentation_distance_treshold;
		global_parameters.cylinder_min_radius =
				config.groups.segmentation_cylinder.cylinder_min_radius;
		global_parameters.cylinder_max_radius =
				config.groups.segmentation_cylinder.cylinder_max_radius;

	    global_parameters.sphere_segmentation_distance_treshold =
	            config.groups.segmentation_sphere.sphere_segmentation_distance_treshold;
	    global_parameters.sphere_min_radius =
	            config.groups.segmentation_sphere.sphere_min_radius;
	    global_parameters.sphere_max_radius =
	            config.groups.segmentation_sphere.sphere_max_radius;

		global_parameters.smoothing_search_radius =
				config.groups.smoothing.smoothing_search_radius;
		global_parameters.smoothing_enabled = config.groups.smoothing.enabled;

		global_parameters.clustering_cluster_min_size =
				config.groups.clustering.cluster_min_size;
		global_parameters.clustering_cluster_max_size =
				config.groups.clustering.cluster_max_size;
		global_parameters.clustering_neighbors =
				config.groups.clustering.neighbors;
		global_parameters.clustering_smoothness_treshold =
				config.groups.clustering.smoothness_treshold;
		global_parameters.clustering_curvature_treshold =
				config.groups.clustering.curvature_treshold;
		//update();
	} else {
		ROS_WARN("Reconfigure had parameters set to 0! Nothing will be done.");
	}
}

CylinderDetector::CylinderDetector(pcl::visualization::PCLVisualizer::Ptr visualizer) :
		nh() {
    this->visualizer = visualizer;

	ROS_INFO("Initializing...");
	global_parameters.detection_method = EXTRACTION;
	global_parameters.shape = CYLINDER;
	global_parameters.normal_estimation_radius_search = 0.015;
	global_parameters.statistical_outlier_mean_k = 100;
	global_parameters.statistical_outlier_mul_tresh = 1.0;
	global_parameters.plane_segmentation_distance_treshold = 0.01;
	global_parameters.plane_segmentation_minimum_points = 8000;
	global_parameters.cylinder_segmentation_distance_treshold = 0.02;
	global_parameters.smoothing_enabled = true;
	global_parameters.smoothing_search_radius = 0.01;
	global_parameters.cylinder_min_radius = 0.01;
	global_parameters.cylinder_max_radius = 0.05;
	global_parameters.sphere_segmentation_distance_treshold = 0.02;
	global_parameters.sphere_min_radius = 0.05;
	global_parameters.sphere_max_radius = 0.10;
	global_parameters.clustering_cluster_min_size = 1000;
	global_parameters.clustering_cluster_max_size = 100000;
	global_parameters.clustering_neighbors = 100;
	global_parameters.clustering_smoothness_treshold = 3.5;
	global_parameters.clustering_curvature_treshold = 0.07;

	publisher_cloud_removed_outliers = nh.advertise<sensor_msgs::PointCloud2>(
			"cloud_removed_outliers", 1);
	publisher_cloud_without_planes = nh.advertise<sensor_msgs::PointCloud2>(
			"cloud_without_planes", 1);
	publisher_cloud_projected = nh.advertise<sensor_msgs::PointCloud2>(
			"cloud_projected", 1);
	publisher_rviz_marker = nh.advertise<visualization_msgs::Marker>(
			"visualization_marker", 1);

	for (int i = 0; i < MAXIMUM_NUMBER_OF_CLUSTERS; i++) {
		std::stringstream ss;
		ss << "cluster" << i;
		publisher_clusters[i] = nh.advertise<sensor_msgs::PointCloud2>(
				ss.str().c_str(), 1);
	}
}

void CylinderDetector::showCloudWithNormals(
        pcl::PointCloud<PointType>::Ptr cloud, const char cloud_id[],
        pcl::PointCloud<pcl::Normal>::Ptr normals, const char normals_id[]) {
    pcl::visualization::PointCloudColorHandlerRandom<PointType> color_handler(cloud);
    visualizer->addPointCloud<PointType>(cloud, color_handler, cloud_id);
    visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4.0, cloud_id);
    visualizer->addPointCloudNormals<PointType, pcl::Normal>(cloud, normals, 50,
            0.05f, normals_id);
}

const double Z_MIN = 0.5;
const double Z_MAX = 1.0;
const double MIN_MATCHING_RATIO = 0.3;

Bottle CylinderDetector::detectUsingClustering(
		pcl::PointCloud<PointType>::Ptr cloud) {
	Bottle result;
	result.success = false; // Assume that the algorithm will fail.
	pcl::PointCloud<PointType>::Ptr cloud_pipeline_result;
	pcl::PointCloud<pcl::Normal>::Ptr normals_pipeline_result;

	PointCloudProcessingPipeline cloud_processing_pipeline(global_parameters);
	cloud_processing_pipeline.setInputCloud(cloud);
	cloud_processing_pipeline.filterZ(Z_MIN, Z_MAX);
	cloud_processing_pipeline.computeNormals();
    cloud_processing_pipeline.removeNaNsWithNormals();
	cloud_pipeline_result = cloud_processing_pipeline.getCurrentCloud();
	normals_pipeline_result = cloud_processing_pipeline.getCurrentNormals();

	std::vector<pcl::PointIndices> clusters_indices = getClustersIndices(
			cloud_pipeline_result, normals_pipeline_result,
			global_parameters.clustering_cluster_min_size,
			global_parameters.clustering_cluster_max_size,
			global_parameters.clustering_neighbors,
			global_parameters.clustering_smoothness_treshold,
			global_parameters.clustering_curvature_treshold);

	ROS_INFO("Found %d clusters", (int ) clusters_indices.size());
	pcl::PointCloud<PointType>::Ptr best_match;
	pcl::ModelCoefficients::Ptr best_match_coefficients;
	double best_matching_ratio = 0.0;
	for (int i = 0; i < clusters_indices.size(); i++) {
		pcl::PointCloud<PointType>::Ptr cylinder;
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		PointCloudProcessingPipeline cluster_processing_pipeline(
				global_parameters);
		cluster_processing_pipeline.setInputCloud(cloud_pipeline_result, clusters_indices[i]);
		cluster_processing_pipeline.setInputNormals(normals_pipeline_result, clusters_indices[i]);

		// Visualization:
		std::stringstream name_points;
		name_points << "cluster_" << i;
        std::stringstream name_normals;
        name_normals << "normals_" << i;
        showCloudWithNormals(cluster_processing_pipeline.getCurrentCloud(),
                name_points.str().c_str(), cluster_processing_pipeline.getCurrentNormals(),
                name_normals.str().c_str());

		int size_before_segmentation = cluster_processing_pipeline.cloudSize();
		bool segmentation_success;
		switch (global_parameters.shape) {
		case CYLINDER:
		    ROS_INFO("Cylinder detection");
            segmentation_success = cluster_processing_pipeline.getCylinder(coefficients);
            break;
		case SPHERE:
            ROS_INFO("Sphere detection");
            segmentation_success = cluster_processing_pipeline.getSphere(coefficients);
            break;
	    default:
	        ROS_ERROR("Unknown shape");
	        exit(-1);
		}
		if (segmentation_success) {
            int size_after_segmentation =
                    cluster_processing_pipeline.cloudSize();
            double matching_ratio = (double) size_after_segmentation / size_before_segmentation;
            if (matching_ratio >= MIN_MATCHING_RATIO
                    && matching_ratio >= best_matching_ratio) {
                best_matching_ratio = matching_ratio;
                best_match = cluster_processing_pipeline.getCurrentCloud();
                best_match_coefficients = coefficients;
            }
		}
	}

	if (best_matching_ratio >= MIN_MATCHING_RATIO) {
	    PointCloudProcessingPipeline result_processing_pipeline(global_parameters);
	    result_processing_pipeline.setInputCloud(best_match);
	    switch (global_parameters.shape) {
	    case CYLINDER:
	        result = getResultFromPipelineWithCylinder(result_processing_pipeline, best_match_coefficients);
	        break;
	    case SPHERE:
	        result = getResultFromPipelineWithSphere(result_processing_pipeline, best_match_coefficients);
	        break;
	    }
	}

	return result;
}

Bottle CylinderDetector::detectUsingPlaneExtraction(
		pcl::PointCloud<PointType>::Ptr cloud) {
	Bottle result;
    result.success = false; // Assume that the algorithm will fail.

	pcl::PointCloud<PointType>::Ptr cloud_without_planes;
	pcl::PointCloud<PointType>::Ptr cloud_removed_outliers;
	pcl::PointCloud<pcl::Normal>::Ptr normals;
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

	PointCloudProcessingPipeline cloud_processing_pipeline(global_parameters);

	cloud_processing_pipeline.setInputCloud(cloud);
	cloud_processing_pipeline.filterZ(Z_MIN, Z_MAX);
	cloud_processing_pipeline.removeOutliers();

    cloud_removed_outliers = cloud_processing_pipeline.getCurrentCloud();
	publishPointCloud(publisher_cloud_removed_outliers, cloud_removed_outliers);

	int planes_found = 0;
	while (cloud_processing_pipeline.removePlaneOfOrganizedCloud()) {
		planes_found++;
	}
	ROS_INFO("%d planes found", planes_found);

	cloud_processing_pipeline.removeOutliers();
	cloud_processing_pipeline.computeNormals();
	cloud_without_planes = cloud_processing_pipeline.getCurrentCloud();
	normals = cloud_processing_pipeline.getCurrentNormals();

	publishPointCloud(publisher_cloud_without_planes, cloud_without_planes);

	cloud_processing_pipeline.removeNaNsWithNormals();

	switch (global_parameters.shape) {
	case CYLINDER:
        ROS_INFO("Cylinder detection");
	    if (!cloud_processing_pipeline.getCylinder(coefficients)) {
            ROS_WARN("Cylinder was not found in the cloud!");
        } else {
            result = getResultFromPipelineWithCylinder(cloud_processing_pipeline,
                    coefficients);
        }
	    break;
	case SPHERE:
        ROS_INFO("Sphere detection");
        if (!cloud_processing_pipeline.getSphere(coefficients)) {
            ROS_WARN("Sphere was not found in the cloud!");
        } else {
            result = getResultFromPipelineWithSphere(cloud_processing_pipeline,
                    coefficients);
        }
        break;
	default:
	    ROS_ERROR("Unknown shape");

	}

	return result;
}

Bottle CylinderDetector::detect(pcl::PointCloud<PointType>::Ptr cloud) {
	Bottle result;
	ros::Time start_detection = ros::Time::now();
	ROS_INFO("Started Bottle detection");

	switch (global_parameters.detection_method) {
	case EXTRACTION:
	    ROS_INFO("Extraction-type detection");
		result = detectUsingPlaneExtraction(cloud);
		break;
	case CLUSTERING:
        ROS_INFO("Clustering-type detection");
		result = detectUsingClustering(cloud);
		break;
	default:
		ROS_ERROR("Unknown detection method requested.");
		exit(-1);
	}

	ros::Time end_detection = ros::Time::now();
	ros::Duration detection_duration = end_detection - start_detection;
	ROS_INFO("Finished Bottle detection in %f",
			(float ) detection_duration.sec
					+ (float ) (detection_duration.nsec) / 1000000000.0f);

	return result;
}
