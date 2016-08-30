#include "utils.h"
#include "moveit/move_group_interface/move_group.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "main");

    ros::NodeHandle node_handle;

    ros::Duration sleep_time(1.0);

    moveit::planning_interface::MoveGroup group("arm");

    while (ros::ok()) {
        geometry_msgs::Pose pose = group.getCurrentPose().pose;
        printPose(pose);
        ROS_INFO("Without stuff: %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n",
                    pose.position.x, pose.position.y, pose.position.z,
                    pose.orientation.x, pose.orientation.y, pose.orientation.z,
                    pose.orientation.w);
        ros::spinOnce();
        sleep_time.sleep();
    }
    ros::shutdown();
    return 0;
}
