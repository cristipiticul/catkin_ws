#include <ros/ros.h>
#include "gripper_command_listener.h"
#include "std_msgs/Float64.h"

class GripperControl {
private:
    ros::NodeHandle n;
    GripperCommandListener gripperCommandListener;
    ros::Publisher gripper_joint_publisher;
    ros::Publisher gripper_joint2_publisher;
public:
    GripperControl() :
            n(), gripperCommandListener() {
        gripper_joint_publisher = n.advertise<std_msgs::Float64>(
                "/gripper_joint_position_controller/command", 10);
        gripper_joint2_publisher = n.advertise<std_msgs::Float64>(
                "/gripper_joint2_position_controller/command", 10);
    }

    void loop() {
        while (ros::ok()) {
            Command currentCommand = gripperCommandListener.getCurrentCommand();
            if (currentCommand == OPEN) {
                //ROS_INFO("Opening the gripper...");
                std_msgs::Float64 joint1;
                joint1.data = -1.5;
                std_msgs::Float64 joint2;
                joint2.data = 1.5;
                gripper_joint_publisher.publish(joint1);
                gripper_joint2_publisher.publish(joint2);
            } else if (currentCommand == CLOSE) {
                //ROS_INFO("Closing the gripper...");
                std_msgs::Float64 joint1;
                joint1.data = 1.5;
                std_msgs::Float64 joint2;
                joint2.data = -1.5;
                gripper_joint_publisher.publish(joint1);
                gripper_joint2_publisher.publish(joint2);
            } else {
                //ROS_INFO("IDLE");
            }
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "gripper_control");

    GripperControl gripper_control;
    gripper_control.loop();

    ros::spin();

    return 0;
}
