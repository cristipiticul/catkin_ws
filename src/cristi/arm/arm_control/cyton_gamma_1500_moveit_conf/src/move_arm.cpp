#include "utils.h"

#include "ros/ros.h"
#include "ros/publisher.h"
#include "moveit/move_group_interface/move_group.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/DisplayTrajectory.h"
#include "tf/transform_datatypes.h"

#include <moveit_msgs/DisplayRobotState.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <shape_msgs/SolidPrimitive.h>

void badumtsss(moveit::planning_interface::MoveGroup& group) {
    std::vector<double> group_variable_values;
    group.getCurrentState()->copyJointGroupPositions(
            group.getCurrentState()->getRobotModel()->getJointModelGroup(
                    group.getName()), group_variable_values);
    for (int i = 0; i < 7; i++) {
        if (i % 2 == 0) {
            group_variable_values[i] = -3.0;
        } else {
            group_variable_values[i] = 3.0;
        }
    }
    group.setJointValueTarget(group_variable_values);

    bool success;
    moveit::planning_interface::MoveGroup::Plan my_plan;
    success = group.plan(my_plan);

    sleep(0.5);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "main");

    ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    int ok;

    ROS_INFO("Working!");

    moveit::planning_interface::MoveGroup group("arm");
    group.setPlannerId("RRTConnectkConfigDefault");

    ROS_INFO("MoveGroup instantiated!");

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO("EEF link: %s", group.getEndEffectorLink().c_str());

    ROS_INFO("Waiting 2 seconds to initialize...");
    sleep(2.0);

//    std::vector<moveit_msgs::CollisionObject> collision_objects;
//
//    tf::Quaternion q = tf::createQuaternionFromRPY(0.0, 0.0, 0.0);
//    collision_objects.push_back(
//            createCollisionBox(group, "ground", 1.5, 1.5, 0.5, 0.0, 0.0, -0.25,
//                    q.getX(), q.getY(), q.getZ(), q.getW()));
//    collision_objects.push_back(
//            createCollisionBox(group, "box", 0.5, 0.5, 1.0, -0.34, 0.45, 0.25,
//                    q.getX(), q.getY(), q.getZ(), q.getW()));
//
//    ROS_INFO("Add an object into the world");
//    planning_scene_interface.addCollisionObjects(collision_objects);

    group.setPlanningTime(1.0);

//    if (!moveTo(group, 0.0, -0.30, 0.30, -M_PI / 2, 0.0, M_PI)) {
//        return -1;
//    }
//    if (!moveTo(group, -0.43, 0.0, 0.30, 0.0, -M_PI / 2, 0.0)) {
//        return -1;
//    }
//  if (!moveTo(group, -0.43, 0.0, 0.28, 0.0, -M_PI / 2, 0.0, 0.08f, 0.08f,
//      M_PI)) {
//    return -1;
//  }
//  if (!moveTo(group, -0.43, 0.0, 0.26, 0.0, -M_PI / 2, 0.0, 0.08f, 0.08f,
//      M_PI)) {
//    return -1;
//  }
//    geometry_msgs::Pose next_pose = group.getCurrentPose("virtual_endeffector").pose;
//    next_pose.position.z -= 0.1;
//    if (!moveOnAxisTo(group, next_pose, Z_AXIS)) {
//        return -1;
//    }
//
//    if (!moveTo(group, 0.0, 0.43, 0.30, -M_PI / 2, 0.0, 0.0)) {
//        return -1;
//    }
    
    while (true) {
        geometry_msgs::Pose pose;
        scanf("%lf %lf %lf %lf %lf %lf %lf", &pose.position.x, &pose.position.y,
                &pose.position.z, &pose.orientation.x, &pose.orientation.y,
                &pose.orientation.z, &pose.orientation.w);
        moveTo(group, pose);
    }  

/*
    double distance = 0.05;
    bool restrictions = true;
    while (1) {
        char c;
        scanf("%c", &c);
        geometry_msgs::Pose current_pose = group.getCurrentPose(group.getEndEffectorLink()).pose;
        switch (c) {
        case '+':
            distance += 0.01;
            ROS_INFO("The new move distance is %f meters", distance);
            break;
        case '-':
            distance -= 0.01;
            ROS_INFO("The new move distance is %f meters", distance);
            break;
        case 'r':
            restrictions = !restrictions;
            ROS_INFO("Restrictions are now %s.", (restrictions ? "enabled" : "disabled"));
            break;
        case 'o':
            moveTo(group, current_pose.position.x, current_pose.position.y, current_pose.position.z, -M_PI / 2, 0.0, M_PI);
            break;
        case 'q':
            return 0;
        }
        if (restrictions) {
            switch (c) {
            case 'w':
                moveForward(group, distance);
                break;
            case 'a':
                moveLeft(group, distance);
                break;
            case 's':
                moveBack(group, distance);
                break;
            case 'd':
                moveRight(group, distance);
                break;
            case 'z':
                moveUp(group, distance);
                break;
            case 'x':
                moveDown(group, distance);
                break;
            }
        }
        else {
            geometry_msgs::Pose next_pose = current_pose;
            switch (c) {
            case 'w':
                next_pose.position.y -= distance;
                moveTo(group, next_pose);
                break;
            case 'a':
                next_pose.position.x += distance;
                moveTo(group, next_pose);
                break;
            case 's':
                next_pose.position.y += distance;
                moveTo(group, next_pose);
                break;
            case 'd':
                next_pose.position.x -= distance;
                moveTo(group, next_pose);
                break;
            case 'z':
                next_pose.position.z += distance;
                moveTo(group, next_pose);
                break;
            case 'x':
                next_pose.position.z -= distance;
                moveTo(group, next_pose);
                break;
            }
        }
    }
*/
    ros::shutdown();

    return 0;
}
