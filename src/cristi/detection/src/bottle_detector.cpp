#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <tf/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>
#include "bottle_detector.hpp"
#include "color_detector.hpp"
#include "cylinder_detector.hpp"

/// FOR TEST ONLY:
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>

pcl::visualization::PCLVisualizer::Ptr visualizer;

class PointCloudSubscriber {
private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    BottleDetector* detector;
    ros::Publisher pub;
    ros::Duration time_between_detections;
    ros::Duration delay_after_detection;
    ros::Time time_last_detection;
    ros::Time time_last_detection_finished;
public:
    PointCloudSubscriber(BottleDetector* detector): time_between_detections(5.0), delay_after_detection(1.0), time_last_detection(ros::TIME_MIN) {
        this->detector = detector;
    }

    void start(char cloud_topic[]) {
        sub = n.subscribe(cloud_topic, 1,
                   &PointCloudSubscriber::subscriberCallback, this);
        pub = n.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);
    }

    void subscriberCallback(const sensor_msgs::PointCloud2& cloud_msg) {
        ros::Time now = ros::Time::now();
        if (now - time_last_detection >= time_between_detections && now - time_last_detection_finished >= delay_after_detection) {
            time_last_detection = now;

            pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
            pcl::fromROSMsg(cloud_msg, *cloud);

            visualizer->removeAllPointClouds();
            visualizer->removeAllShapes();
            visualizer->addCoordinateSystem(0.5);
            Bottle result = detector->detect(cloud);
            if (result.success) {
                ROS_INFO("Publishing point cloud!");
                result.cloud.header.frame_id = "camera_rgb_optical_frame";
                result.cloud.header.stamp = ros::Time::now();
                pub.publish(result.cloud);
                broadcastPoseToTf(result.pose);
            }
            else {
                ROS_ERROR("The bottle was not found!");
            }

            time_last_detection_finished = ros::Time::now();
        }
    }

    void broadcastPoseToTf(geometry_msgs::Pose bottlePose) {
        static tf::TransformBroadcaster br;
        tf::Transform transform;

        transform.setOrigin(
                tf::Vector3(bottlePose.position.x, bottlePose.position.y,
                        bottlePose.position.z));
        tf::Quaternion q;
        tf::quaternionMsgToTF(bottlePose.orientation, q);
        transform.setRotation(q);
        br.sendTransform(
                tf::StampedTransform(transform, ros::Time::now(), "camera_rgb_optical_frame",
                        "bottle"));
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "bottle_detector");

    visualizer.reset(new pcl::visualization::PCLVisualizer("Intermediary clouds"));

    dynamic_reconfigure::Server<detection::ParametersConfig> srv;
    dynamic_reconfigure::Server<detection::ParametersConfig>::CallbackType f;

    if (argc <= 2) {
        ROS_ERROR("Usage: rosrun detection bottle_detector <detection_type> <cloud_topic>");
        ROS_ERROR("Detection type is \"color\" or \"cylinder\"");
        exit(-1);
    }

    BottleDetector* detector;
    if (0 == strcmp(argv[1], "color")) {
        detector = new ColorDetector;
    }
    else if (0 == strcmp(argv[1], "cylinder")) {
        CylinderDetector* cylinder_detector = new CylinderDetector(visualizer);
        detector = cylinder_detector;
        f = boost::bind(reconfigureCallback, _1, _2);
        srv.setCallback(f);
        ROS_INFO("Dynamic reconfigure callback is set!");
    }
    else {
        ROS_ERROR("detection_type must be: \"color\" or \"cylinder\"");
        exit(-1);
    }

    ROS_INFO("Starting!");
    PointCloudSubscriber subscriber(detector);
    subscriber.start(argv[2]);

    while(ros::ok()) {
        ros::spinOnce();
        visualizer->spinOnce();
        usleep(10000);
    }

    return 0;

}
