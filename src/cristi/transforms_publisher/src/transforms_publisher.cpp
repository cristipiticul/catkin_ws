#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/Pose.h"

class BottlePositionListener {
public:
    BottlePositionListener();
    void bottlePositionCallback(const geometry_msgs::Pose& bottlePose) {

        static tf::TransformBroadcaster br;
        tf::Transform transform;

        transform.setOrigin(
                tf::Vector3(bottlePose.position.x, bottlePose.position.y,
                        bottlePose.position.z));
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
        br.sendTransform(
                tf::StampedTransform(transform, ros::Time::now(), "camera",
                        "bottle"));
    }
private:
    ros::Subscriber sub;
    ros::NodeHandle n;
    double cap_dx;
    double cap_dy;
    double cap_dz;
};

BottlePositionListener::BottlePositionListener(): n() {
    n.getParam("bottle_dx", cap_dx);
    n.getParam("bottle_dy", cap_dy);
    n.getParam("bottle_dz", cap_dz);
    sub = n.subscribe("bottle_position_camera", 10,
               &BottlePositionListener::bottlePositionCallback, this);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "bottle_position_listener");

    BottlePositionListener listener;

    ros::spin();

    return 0;
}
