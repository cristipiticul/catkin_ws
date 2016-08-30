#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "ros/ros.h"

typedef pcl::PointXYZRGB PointType;
#include "cloud_callback.h"

class PointCloudSubscriber {
private:
    ros::NodeHandle n;
    CloudCallback cloud_callback;
    ros::Subscriber sub;
public:
    PointCloudSubscriber() {
    }

    void start() {
        sub = n.subscribe("/kinect_ir/points", 1,
                   &PointCloudSubscriber::subscriberCallback, this);
    }

    void subscriberCallback(const sensor_msgs::PointCloud2& cloud_msg) {
        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
        pcl::fromROSMsg(cloud_msg, *cloud);
        cloud_callback.callback(cloud);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "bottle_position_finder");
    PointCloudSubscriber subscriber;
    subscriber.start();

    ros::spin();

    return 0;

}
