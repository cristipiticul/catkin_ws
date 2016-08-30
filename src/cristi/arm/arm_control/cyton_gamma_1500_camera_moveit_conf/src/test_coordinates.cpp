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

	ros::NodeHandle node_handle;

	ros::AsyncSpinner spinner(1);
	spinner.start();

	ROS_INFO("Working!");

	moveit::planning_interface::MoveGroup group("arm");

	ROS_INFO("MoveGroup instantiated!");

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	ros::Publisher display_publisher = node_handle.advertise<
			moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path",
			1, true);
	moveit_msgs::DisplayTrajectory display_trajectory;

	ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
	ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

	// Planning to a Pose goal
	// ^^^^^^^^^^^^^^^^^^^^^^^
	// We can plan a motion for this group to a desired pose for the
	// end-effector.
	geometry_msgs::Pose target_pose1;
	target_pose1.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0,
	M_PI / 2, 0.0);
	target_pose1.position.x = 0.43;
	target_pose1.position.y = 0.0;
	target_pose1.position.z = 0.30;

	double y;
	bool success;
	int ok;

	// Plan 1
	moveit::planning_interface::MoveGroup::Plan my_plan;
	moveit::planning_interface::MoveGroup::Plan my_plan2;
	group.setStartStateToCurrentState();
	group.setPoseTarget(target_pose1);
	ROS_INFO(
			"Pose #1: \n  x: %lf\n  y: %lf\n  z: %lf\n  ox: %lf\n  oy: %lf\n  oz: %lf\n  ow: %lf\n",
			target_pose1.position.x, target_pose1.position.y,
			target_pose1.position.z, target_pose1.orientation.x,
			target_pose1.orientation.y, target_pose1.orientation.z,
			target_pose1.orientation.w);
	ROS_INFO("Planning to pose #1");
	success = group.plan(my_plan);
	if (!success) {
		ROS_ERROR("Plan #1 failed!");
		return -1;
	}

	ROS_INFO("Move? (1 - yes, 0 - no)");
	scanf("%d", &ok);

	if (ok == 1) {
		ROS_INFO("Moving to pose #1");
		group.execute(my_plan);
	}

	ROS_INFO("press any key and enter to continue");
	scanf("%*s");

	// Plan 2 - relative to the first position
	geometry_msgs::Pose target_pose2;
	target_pose2.orientation = target_pose1.orientation;
	target_pose2.position.x = target_pose1.position.x - 0.1;
	target_pose2.position.y = target_pose1.position.y;
	target_pose2.position.z = target_pose1.position.z;
	ROS_INFO(
			"Pose #2: \n  x: %lf\n  y: %lf\n  z: %lf\n  ox: %lf\n  oy: %lf\n  oz: %lf\n  ow: %lf\n",
			target_pose2.position.x, target_pose2.position.y,
			target_pose2.position.z, target_pose2.orientation.x,
			target_pose2.orientation.y, target_pose2.orientation.z,
			target_pose2.orientation.w);
	group.setStartStateToCurrentState();
	group.setPoseTarget(target_pose2);
	ROS_INFO("Planning to pose #2");
	success = group.plan(my_plan2);
	if (!success) {
		ROS_ERROR("Plan #2 failed!");
		return -1;
	}

	ROS_INFO("Move? (1 - yes, 0 - no)");
	scanf("%d", &ok);

	if (ok == 1) {
		ROS_INFO("Moving to pose #2");
		group.execute(my_plan2);
	}

	// For finding coordinates in RViz
//	std::vector<moveit_msgs::CollisionObject> collision_objects;
//
//	collision_objects.push_back(
//			createCollisionBox(group, "testbox", 0.01, 0.01, 0.01, 0.0, 0.0,
//					1.0, 0.0, 0.0, 0.0, 1.0));
//
//	planning_scene_interface.addCollisionObjects(collision_objects);

	ros::shutdown();

	return 0;
}
