#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/Pose.h"
#include <tf/transform_datatypes.h>

void pose2Callback(const geometry_msgs::PoseStamped pose) {

    static tf::TransformBroadcaster br;
    tf::Transform transform;

    transform.setOrigin(
            tf::Vector3(pose.pose.position.x, pose.pose.position.y,
                    pose.pose.position.z));
    tf::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
    transform.setRotation(q);

    tf::Transform transform_inverse = transform.inverse();
    br.sendTransform(
            tf::StampedTransform(transform_inverse, ros::Time::now(), "gripper",
                    pose.header.frame_id));
}

void pose1Callback(const geometry_msgs::PoseStamped pose) {

    static tf::TransformBroadcaster br;
    tf::Transform transform;

    transform.setOrigin(
            tf::Vector3(pose.pose.position.x, pose.pose.position.y,
                    pose.pose.position.z));
    tf::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
    transform.setRotation(q);
    br.sendTransform(
            tf::StampedTransform(transform, ros::Time::now(), pose.header.frame_id,
                    "checkerboard"));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mocap_transforms_publisher");
    ros::NodeHandle n;

    ros::Subscriber sub1 = n.subscribe("/pose1", 1, pose1Callback);
    ros::Subscriber sub2 = n.subscribe("/pose2", 1, pose2Callback);

    ros::spin();

    return 0;
}
