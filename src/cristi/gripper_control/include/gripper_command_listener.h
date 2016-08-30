#include <ros/ros.h>
#include "gripper_control/GripperCommand.h"

typedef gripper_control::GripperCommand::Request Request;
typedef gripper_control::GripperCommand::Response Response;

enum Command {
    OPEN, CLOSE, IDLE
};

class GripperCommandListener {
public:
    GripperCommandListener();
    bool callback(Request &request, Response &response);
    Command getCurrentCommand();
private:
    Command currentCommand;
    ros::NodeHandle n;
    ros::ServiceServer service;

};
