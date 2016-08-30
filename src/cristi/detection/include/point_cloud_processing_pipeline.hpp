#include "common.h"

// For ModelCoefficients class
#include <pcl/segmentation/sac_segmentation.h>

enum DetectionMethod {
    EXTRACTION, CLUSTERING
};

enum DetectionShapeType {
    CYLINDER, SPHERE
};

struct CloudProcessingParameters {
    DetectionMethod detection_method;
    DetectionShapeType shape;
    double normal_estimation_radius_search;
    int statistical_outlier_mean_k;
    double statistical_outlier_mul_tresh;
    double plane_segmentation_distance_treshold;
    int plane_segmentation_minimum_points;
    double cylinder_segmentation_distance_treshold;
    bool smoothing_enabled;
    double smoothing_search_radius;
    double cylinder_min_radius;
    double cylinder_max_radius;
    double sphere_segmentation_distance_treshold;
    double sphere_min_radius;
    double sphere_max_radius;
    int clustering_cluster_min_size;
    int clustering_cluster_max_size;
    int clustering_neighbors;
    double clustering_smoothness_treshold;
    double clustering_curvature_treshold;
};

class PointCloudProcessingPipeline {
private:
	CloudProcessingParameters params;
	pcl::PointCloud<PointType>::Ptr cloud_1;
	pcl::PointCloud<PointType>::Ptr cloud_2;
	pcl::PointCloud<PointType>::Ptr *cloud_current;
	pcl::PointCloud<PointType>::Ptr *cloud_tmp;
	pcl::PointCloud<pcl::Normal>::Ptr normals_1;
    pcl::PointCloud<pcl::Normal>::Ptr normals_2;
    pcl::PointCloud<pcl::Normal>::Ptr *normals_current;
    pcl::PointCloud<pcl::Normal>::Ptr *normals_tmp;
	void pipelineSwap();
    void pipelineSwapNormals();
public:
	PointCloudProcessingPipeline(const CloudProcessingParameters& processing_parameters);
	void setInputCloud(pcl::PointCloud<PointType>::Ptr cloud_input);
	void setInputCloud(pcl::PointCloud<PointType>::Ptr cloud_input, const pcl::PointIndices &indices);
    void setInputNormals(pcl::PointCloud<pcl::Normal>::Ptr normals_input);
    void setInputNormals(pcl::PointCloud<pcl::Normal>::Ptr normals_input, const pcl::PointIndices &indices);
	pcl::PointCloud<PointType>::Ptr getCurrentCloud();
    pcl::PointCloud<pcl::Normal>::Ptr getCurrentNormals();
	void filterZ(double z_min, double z_max);
	void removeOutliers();
	void updateParams(const CloudProcessingParameters& new_params);
	void computeNormals();
	void removeNaNs();
	void removeNormalNaNs();
	void removeNaNsWithNormals();
	bool removePlaneOfOrganizedCloud();
	bool getCylinder(pcl::ModelCoefficients::Ptr coefficients);
    bool getSphere(pcl::ModelCoefficients::Ptr coefficients);
	void smoothen();
	void projectOnLine(double point_x, double point_y, double point_z,
			double direction_x, double direction_y, double direction_z);
	int cloudSize();
};
