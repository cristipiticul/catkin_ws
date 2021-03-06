#include "ros/ros.h"
#include "ros/publisher.h"
#include "moveit/move_group_interface/move_group.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/DisplayTrajectory.h"

#include <moveit_msgs/DisplayRobotState.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <stdio.h>

moveit_msgs::CollisionObject createCollisionBox(
		moveit::planning_interface::MoveGroup& group, std::string object_id,
		double size_x, double size_y, double size_z, double pos_x, double pos_y,
		double pos_z, double orientation_x, double orientation_y,
		double orientation_z, double orientation_w) {
	moveit_msgs::CollisionObject collision_object;
	collision_object.header.frame_id = group.getPlanningFrame();

	/* The id of the object is used to identify it. */
	collision_object.id = object_id;

	/* Define a box to add to the world. */
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = size_x;
	primitive.dimensions[1] = size_y;
	primitive.dimensions[2] = size_z;

	/* A pose for the box (specified relative to frame_id) */
	geometry_msgs::Pose box_pose;
	box_pose.orientation.x = orientation_x;
	box_pose.orientation.y = orientation_y;
	box_pose.orientation.z = orientation_z;
	box_pose.orientation.w = orientation_w;
	box_pose.position.x = pos_x;
	box_pose.position.y = pos_y;
	box_pose.position.z = pos_z;

	collision_object.primitives.push_back(primitive);
	collision_object.primitive_poses.push_back(box_pose);
	collision_object.operation = collision_object.ADD;

	return collision_object;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "main");
	int ok;

	ros::NodeHandle node_handle;

	ros::AsyncSpinner spinner(1);
	spinner.start();

	ROS_INFO("Working!");

	bool success;
	moveit::planning_interface::MoveGroup::Plan my_plan;

	moveit::planning_interface::MoveGroup gripper_group("gripper");

	std::vector<double> group_variable_values;
	gripper_group.getCurrentState()->copyJointGroupPositions(
			gripper_group.getCurrentState()->getRobotModel()->getJointModelGroup(
					gripper_group.getName()), group_variable_values);
	do {
	  double dx;
	  ROS_INFO("DX:");
	  scanf("%lf", &dx);
	  group_variable_values[0] = dx;
	  gripper_group.setJointValueTarget(group_variable_values);
	  ROS_INFO("planning the gripper action");
	  success = gripper_group.plan(my_plan);
	  if (!success) {
		  ROS_ERROR("Gripper plan failed!");
		  //return -1;
	  }
	  ROS_INFO("OK? (1 - yes, 0 - no)");
	  scanf("%d", &ok);
	  
	} while (!ok);

	ROS_INFO("Move? (1 - yes, 0 - no)");
	scanf("%d", &ok);

	if (ok == 1) {
		ROS_INFO("moving the gripper");
		gripper_group.execute(my_plan);
	}
	// For finding coordinates in RViz
//	std::vector<moveit_msgs::CollisionObject> collision_objects;
//
//	collision_objects.push_back(
//			createCollisionBox(group, "testbox", 0.01, 0.01, 0.01, 0.0, 0.0,
//					1.0, 0.0, 0.0, 0.0, 1.0));
//
//	planning_scene_interface.addCollisionObjects(collision_objects);

	ROS_INFO("shutting down");
	ros::shutdown();

	return 0;
}
