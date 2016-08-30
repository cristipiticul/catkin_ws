#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

typedef pcl::PointXYZRGB PointType;

// For organizing the point cloud
#define CLOUD_WIDTH 800
#define CLOUD_HEIGHT 600
#define RESULT_TOPIC "/camera/depth_registered/points"

ros::Publisher pub;

void callback(const sensor_msgs::PointCloud2& cloud_msg) {
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr result(new pcl::PointCloud<PointType>);
	sensor_msgs::PointCloud2 result_cloud_msg;

	pcl::fromROSMsg(cloud_msg, *cloud);

	assert(cloud->size() == CLOUD_WIDTH * CLOUD_HEIGHT);

	for (int i = 0; i < cloud->points.size(); i++) {
		PointType point = cloud->points[i];

		// Mirror on the yOz plane
		//point.x = -point.x;

		result->points.push_back(point);
	}
	// Make it organized
	result->width = CLOUD_WIDTH;
	result->height = CLOUD_HEIGHT;

	pcl::toROSMsg(*result, result_cloud_msg);
	result_cloud_msg.header.frame_id = cloud_msg.header.frame_id;
	result_cloud_msg.header.stamp = ros::Time::now();
	pub.publish(result_cloud_msg);

	ros::Duration(0.5).sleep();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "simulated_pointcloud_corrector");

    if (argc < 2) {
    	ROS_ERROR("Usage: rosrun arm_simulation pointcloud_corrector <simulated_cloud_topic>");
    	return -1;
    }

    ros::NodeHandle n;
    ros::Subscriber sub;
    sub = n.subscribe(argv[1], 1, callback);
    pub = n.advertise<sensor_msgs::PointCloud2>(RESULT_TOPIC, 1);

    ros::spin();

    ros::shutdown();

    return 0;
}
