#include "gripper_command_listener.h"

GripperCommandListener::GripperCommandListener(): n() {
    currentCommand = IDLE;
    service = n.advertiseService("command_gripper",
                    &GripperCommandListener::callback, this);
            ROS_INFO("Ready to receive commands.");
}

bool GripperCommandListener::callback(Request &request, Response &response) {
    switch (request.action) {
    case 0:
        currentCommand = IDLE;
        break;
    case 1:
        currentCommand = OPEN;
        break;
    case 2:
        currentCommand = CLOSE;
        break;
    default:
        ROS_ERROR("Unknown command received: %d! (should be 0,1 or 2)", request.action);
    }
    return true;
}

Command GripperCommandListener::getCurrentCommand() {
    return currentCommand;
}
