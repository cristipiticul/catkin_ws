#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <geometry_msgs/Pose.h>
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include <pcl/visualization/pcl_visualizer.h>

bool isNaN(float f) {
    if (f != f) {
        return true;
    }
    return false;
}
/*
 int rMax = 90;
 int rMin = 55;
 int gMax = 80;
 int gMin = 50;
 int bMax = 25;
 int bMin = 0;*/
int rMax = 0;
int rMin = 255;
int gMax = 0;
int gMin = 255;
int bMax = 0;
int bMin = 255;

float hMax = 0;
float hMin = 360;
float sMax = 0;
float sMin = 1;
float vMax = 0;
float vMin = 1;

// 0 => RGB filter
// 1 => HSV filter
int color_filter_type = 1;

int min(int x, int y) {
    return (x <= y) ? x : y;
}
int max(int x, int y) {
    return (x >= y) ? x : y;
}

float minf(float x, float y) {
    return (x <= y) ? x : y;
}
float maxf(float x, float y) {
    return (x >= y) ? x : y;
}
// Source: https://www.cs.rit.edu/~ncs/color/t_convert.html
// r,g,b values are from 0 to 1
// h = [0,360], s = [0,1], v = [0,1]
//		if s == 0, then h = -1 (undefined)
void rgbToHsv(int r, int g, int b, float& h, float& s, float& v) {
    float rf = (float) r / 255.0f;
    float gf = (float) g / 255.0f;
    float bf = (float) b / 255.0f;

    float max_color;
    float min_color;

    if (rf >= gf && rf >= bf) {
        max_color = rf;
    } else if (gf >= bf) {
        max_color = gf;
    } else {
        max_color = bf;
    }

    if (rf <= gf && rf <= bf) {
        min_color = rf;
    } else if (gf <= bf) {
        min_color = gf;
    } else {
        min_color = bf;
    }

    v = max_color;

    float delta = max_color - min_color;
    if (max_color != 0) {
        s = delta / max_color;
    } else {
        s = 0;
        h = -1;
        return;
    }

    if (rf == max_color) {
        h = (gf - bf) / delta;
    } else if (gf == max_color) {
        h = 2 + (bf - rf) / delta;
    } else {
        h = 4 + (rf - gf) / delta;
    }

    h = h * 60.0f;
    if (h < 0.0f) {
        h += 360.0f;
    }
}

void filterCloudByColor(pcl::PointCloud<PointType>::Ptr cloud,
        pcl::PointCloud<PointType>::Ptr result) {
    if (color_filter_type == 1) {
        result->clear();

        PointType new_point;
        pcl::PointCloud<PointType>::iterator iter;

        for (iter = cloud->points.begin(); iter != cloud->points.end();
                iter++) {
            new_point.x = iter->x;
            new_point.y = iter->y;
            new_point.z = iter->z;
            int r = iter->r;
            int g = iter->g;
            int b = iter->b;
            new_point.r = r;
            new_point.g = g;
            new_point.b = b;

            float h, s, v;
            rgbToHsv(r, g, b, h, s, v);

            if (h >= hMin && h <= hMax && s >= sMin && s <= sMax && v >= vMin
                    && v <= vMax) {
                result->push_back(new_point);
            }
        }
    } else {
        // Color filtering - accept only white colored points
        // Source: http://www.pcl-users.org/How-to-filter-based-on-color-using-PCL-td2791524.html
        // Thank you, beaverfan

        // build the condition
        /*int rMax = 210;
         int rMin = 130;
         int gMax = 210;
         int gMin = 130;
         int bMax = 110;
         int bMin = 50;*/
        pcl::ConditionAnd<PointType>::Ptr color_cond(
                new pcl::ConditionAnd<PointType>());
        color_cond->addComparison(
                pcl::PackedRGBComparison<PointType>::Ptr(
                        new pcl::PackedRGBComparison<PointType>("r",
                                pcl::ComparisonOps::LT, rMax)));
        color_cond->addComparison(
                pcl::PackedRGBComparison<PointType>::Ptr(
                        new pcl::PackedRGBComparison<PointType>("r",
                                pcl::ComparisonOps::GT, rMin)));
        color_cond->addComparison(
                pcl::PackedRGBComparison<PointType>::Ptr(
                        new pcl::PackedRGBComparison<PointType>("g",
                                pcl::ComparisonOps::LT, gMax)));
        color_cond->addComparison(
                pcl::PackedRGBComparison<PointType>::Ptr(
                        new pcl::PackedRGBComparison<PointType>("g",
                                pcl::ComparisonOps::GT, gMin)));
        color_cond->addComparison(
                pcl::PackedRGBComparison<PointType>::Ptr(
                        new pcl::PackedRGBComparison<PointType>("b",
                                pcl::ComparisonOps::LT, bMax)));
        color_cond->addComparison(
                pcl::PackedRGBComparison<PointType>::Ptr(
                        new pcl::PackedRGBComparison<PointType>("b",
                                pcl::ComparisonOps::GT, bMin)));

        // build the filter
        pcl::ConditionalRemoval<PointType> condrem(color_cond);
        condrem.setInputCloud(cloud);
        condrem.setKeepOrganized(true);

        // apply filter
        condrem.filter(*result);
    }
}

class CloudCallback {
private:
    ros::NodeHandle n;
    ros::ServiceServer command_service;
    ros::Publisher bottle_position_pub;
    ros::Publisher filtered_point_cloud_publisher;
    ros::Publisher point_cloud_publisher;
    bool center;
    pcl::visualization::CloudViewer viewer;
    //pcl::visualization::PCLVisualizer visualizer;
    pcl::PointCloud<PointType>::Ptr cloud_input;
    pcl::PointCloud<PointType>::Ptr cloud_filtered;
    pcl::PointCloud<PointType>::Ptr cloud_filtered2;
    pcl::PointCloud<PointType>::Ptr cloud_filtered3;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudcenter;
    pcl::PointCloud<PointType>::Ptr ui_cloud;

    ros::Time last_ui_update;
    // The delay is meant to be sure that the PCL Viewer has the
    // same cloud as ui_cloud. Only then we can use point picking
    // to extract the colors.
    ros::Duration UI_UPDATE_DELAY;
public:
    CloudCallback() :
            viewer("PCL OpenNI Viewer"), /*visualizer("Cloud Visualizer"),*/ cloud_input(
                    new pcl::PointCloud<PointType>), cloud_filtered(
                    new pcl::PointCloud<PointType>), cloud_filtered2(
                    new pcl::PointCloud<PointType>), cloud_filtered3(
                    new pcl::PointCloud<PointType>), cloudcenter(
                    new pcl::PointCloud<pcl::PointXYZ>), last_ui_update(0.0), UI_UPDATE_DELAY(
                    5.0), ui_cloud(new pcl::PointCloud<PointType>) {
        center = true;
        command_service = n.advertiseService("reset_color_filter",
                &CloudCallback::resetColorFilter, this);
        bottle_position_pub = n.advertise<geometry_msgs::Pose>(
                "bottle_position_camera", 10);
        filtered_point_cloud_publisher = n.advertise<sensor_msgs::PointCloud2>(
                "filtered_cloud", 1);
        point_cloud_publisher = n.advertise<sensor_msgs::PointCloud2>("cloud",
                1);
        /*visualizer.registerPointPickingCallback(&CloudCallback::pointPickingCallback, *this, NULL);
        visualizer.addCoordinateSystem(1.0);*/
        viewer.registerPointPickingCallback(
                &CloudCallback::pointPickingCallback, *this, NULL);
    }

    bool resetColorFilter(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &res) {
        ROS_INFO("Resetting the color filter values!");

        rMax = 0;
        rMin = 255;
        gMax = 0;
        gMin = 255;
        bMax = 0;
        bMin = 255;

        hMax = 0;
        hMin = 360;
        sMax = 0;
        sMin = 1;
        vMax = 0;
        vMin = 1;

        return true;
    }

    void pointPickingCallback(
            const pcl::visualization::PointPickingEvent &event, void *cookie) {
        ROS_INFO("Point picked!");
        float x, y, z;
        int index = event.getPointIndex();
        event.getPoint(x, y, z);
        ROS_INFO("%.2f\t%.2f\t%.2f\n", x, y, z);
        PointType selectedPoint = ui_cloud->points[index];
        ROS_INFO("%.2f\t%.2f\t%.2f\n", selectedPoint.x, selectedPoint.y,
                selectedPoint.z);
        ROS_INFO("%d %d %d %d", selectedPoint.r, selectedPoint.g,
                selectedPoint.b, selectedPoint.a);

        int newRMin = selectedPoint.r - 5 > 0 ? selectedPoint.r - 5 : 0;
        int newGMin = selectedPoint.g - 5 > 0 ? selectedPoint.g - 5 : 0;
        int newBMin = selectedPoint.b - 5 > 0 ? selectedPoint.b - 5 : 0;
        int newRMax = selectedPoint.r + 5 < 256 ? selectedPoint.r + 5 : 0;
        int newGMax = selectedPoint.g + 5 < 256 ? selectedPoint.g + 5 : 0;
        int newBMax = selectedPoint.b + 5 < 256 ? selectedPoint.b + 5 : 0;

        rMin = min(newRMin, rMin);
        gMin = min(newGMin, gMin);
        bMin = min(newBMin, bMin);
        rMax = max(newRMax, rMax);
        gMax = max(newGMax, gMax);
        bMax = max(newRMax, bMax);

        float h, s, v;
        rgbToHsv(selectedPoint.r, selectedPoint.g, selectedPoint.b, h, s, v);
        hMin = minf(h, hMin);
        sMin = minf(s, sMin);
        vMin = minf(v, vMin);
        hMax = maxf(h, hMax);
        sMax = maxf(s, sMax);
        vMax = maxf(v, vMax);

        ROS_INFO(
                "New values: \nminR: %d\tmaxR: %d\nminG: %d\tmaxG: %d\nminB: %d\tmaxB: %d\n",
                rMin, rMax, gMin, gMax, bMin, bMax);
        ROS_INFO(
                "New values: \nminH: %f\tmaxH: %f\nminS: %f\tmaxS: %f\nminV: %f\tmaxV: %f\n",
                hMin, hMax, sMin, sMax, vMin, vMax);
    }

    void callback(const pcl::PointCloud<PointType>::ConstPtr &cloud) {
        Eigen::Vector4f centroid;

        // Important!!!! The PCL Viewer removes all NaN points before processing.
        // When the callback is called, the event data contains the index of the clicked point in the
        // PCL Viewer cloud (with the data points that contain NaN coordinates eliminated).
        // In order for everything to be OK, we remove all the NaN-coordinates-containing points from the ui_cloud.
        // Furthermore, removing all the NaN values will speed up the processing.

        // This variable is unused. It is a mapping function from the cloud point indices to the cloud_input indices.
        // It is not required since we don't care about the old indices.
        std::vector<int> newIndices2;
        pcl::removeNaNFromPointCloud(*cloud, *cloud_input, newIndices2);
//        ROS_INFO("Removed %d NaN points from the initial cloud.", (int) (cloud->points.size() - cloud_input->points.size()));

        if (!viewer.wasStopped()) {
            {
                // Create the filtering object
                pcl::PassThrough<PointType> pass;
                pass.setInputCloud(cloud_input);
                pass.setFilterFieldName("z");
                pass.setFilterLimits(0.5, 1.0);
                pass.setFilterLimitsNegative(false);
                pass.filter(*cloud_filtered);
            }
            {
                pcl::PassThrough<PointType> pass;
                pass.setInputCloud(cloud_filtered);
                pass.setFilterFieldName("x");
                pass.setFilterLimits(-0.5, 0.5);
                pass.setFilterLimitsNegative(false);
                pass.filter(*cloud_filtered2);
            }
            {
                filterCloudByColor(cloud_filtered2, cloud_filtered3);
            }
            if (ros::Time::now() - last_ui_update >= UI_UPDATE_DELAY) {
                last_ui_update = ros::Time::now();
                //std::vector<int> newIndices;
                //pcl::removeNaNFromPointCloud(*cloud_input, *ui_cloud, newIndices);
                pcl::copyPointCloud(*cloud_input, *ui_cloud);
                viewer.showCloud(ui_cloud);

                /* Oh, boy, this crashes a lot
                visualizer.removeAllPointClouds();
                pcl::visualization::PointCloudColorHandlerRGBField<PointType> cloud_color_handler;
                pcl::visualization::PointCloudColorHandlerCustom<PointType> color_b(0, 0, 255);
                cloud_color_handler.setInputCloud(ui_cloud);
                if (!visualizer.updatePointCloud<PointType>(ui_cloud, cloud_color_handler, "cloud")) {
                    visualizer.addPointCloud<PointType>(ui_cloud, cloud_color_handler, "cloud");
                }
                if (!visualizer.updatePointCloud<PointType>(cloud_filtered3, color_b, "cloud_filtered")) {
                    visualizer.addPointCloud<PointType>(cloud_filtered3, color_b, "cloud_filtered");
                }
                visualizer.spinOnce();
                */
            }
            //viewer.showCloud(cloud);
            publishPointCloud(filtered_point_cloud_publisher, *cloud_filtered3);
            publishPointCloud(point_cloud_publisher, *cloud_input);
        }
        if (center) {
            geometry_msgs::Pose msg;

            //Object to store the centroid coordinates.
            pcl::compute3DCentroid(*cloud_filtered3, centroid);
            msg.position.x = centroid[0];
            msg.position.y = centroid[1];
            msg.position.z = centroid[2];

            if (!isNaN(msg.position.x) && !isNaN(msg.position.y)
                    && !isNaN(msg.position.z)) {
                /*std::cout << "The XYZ coordinates of the centroid are: ("
                 << centroid[0] << ", " << centroid[1] << ", " << centroid[2]
                 << ").\n"; */
                bottle_position_pub.publish(msg);
            }
        }
    }

    void publishPointCloud(ros::Publisher publisher,
            const pcl::PointCloud<PointType> &cloud) {
        sensor_msgs::PointCloud2 ros_cloud;
        pcl::toROSMsg(cloud, ros_cloud);
        ros_cloud.header.frame_id = "camera";
        publisher.publish(ros_cloud);
    }
};
