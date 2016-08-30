#ifndef COLOR_DETECTOR_H_
#define COLOR_DETECTOR_H_

#include "bottle_detector.hpp"

class ColorDetector: public BottleDetector {
public:
    Bottle detect(pcl::PointCloud<PointType>::Ptr cloud);
};

#endif
