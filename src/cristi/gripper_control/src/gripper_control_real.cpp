#include <ros/ros.h>
#include "gripper_command_listener.h"
#include "std_msgs/Float64.h"

/**
 * gripper state message:
 * Topic: /gripper_controller/state
 * header:
  seq: 357
  stamp:
    secs: 1449740451
    nsecs: 831737995
  frame_id: ''
name: gripper_joint
motor_ids: [7]
motor_temps: [33]
goal_pos: 0.782330201822
current_pos: -0.98634964661    -> between CLOSED: -1.01396130079, OPEN: 0.78386418261
error: -1.76867984843
velocity: 0.0
load: 0.0
is_moving: False

 * Topic for publishing the command: /gripper_controller/command
 */

class GripperControl {
private:
    ros::NodeHandle n;
    GripperCommandListener gripperCommandListener;
    ros::Publisher gripper_joint_publisher;
public:
    GripperControl() :
            n(), gripperCommandListener() {
        gripper_joint_publisher = n.advertise<std_msgs::Float64>(
                "/gripper_controller/command", 10);
    }

    void loop() {
        while (ros::ok()) {
            Command currentCommand = gripperCommandListener.getCurrentCommand();
            if (currentCommand == OPEN) {
                std_msgs::Float64 joint;
                joint.data = 0.78386418261;
                gripper_joint_publisher.publish(joint);
            } else if (currentCommand == CLOSE) {
                std_msgs::Float64 joint;
                // For fully-closed gripper:
                //joint.data = -1.01396130079;
                // For the bottle cap:
                joint.data = -0.882038953034;
                gripper_joint_publisher.publish(joint);
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
