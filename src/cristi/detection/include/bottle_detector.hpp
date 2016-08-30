#ifndef BOTTLE_DETECTOR_H_
#define BOTTLE_DETECTOR_H_

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>
#include "common.h"

struct Bottle {
    bool success;
    sensor_msgs::PointCloud2 cloud;
    geometry_msgs::Pose pose;
};

class BottleDetector {
public:
    virtual Bottle detect(pcl::PointCloud<PointType>::Ptr cloud) = 0;
    virtual ~BottleDetector() {}
};

#endif
