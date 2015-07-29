#include "ros/ros.h"
#include "ros/publisher.h"
#include "moveit/move_group_interface/move_group.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/DisplayTrajectory.h"

#include <moveit_msgs/DisplayRobotState.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

geometry_msgs::Quaternion createQuaternion(double angle, double nx, double ny,
		double nz) {
	geometry_msgs::Quaternion quaternion;
	quaternion.w = cos(angle);
	quaternion.x = nx * sin(angle);
	quaternion.y = ny * sin(angle);
	quaternion.z = nz * sin(angle);
	return quaternion;
}

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
	target_pose1.orientation = createQuaternion(M_PI / 4, -1.0, 0.0, 0.0);
	target_pose1.position.x = 0.0;
	target_pose1.position.y = 0.40;
	target_pose1.position.z = 0.35;

	double y;
	bool success;
	moveit::planning_interface::MoveGroup::Plan my_plan;
	group.setStartStateToCurrentState();
	group.setPoseTarget(target_pose1);

	// Now, we call the planner to compute the plan
	// and visualize it.
	// Note that we are just planning, not asking move_group
	// to actually move the robot.
	success = group.plan(my_plan);
	ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

	sleep(5.0);

	std::vector<moveit_msgs::CollisionObject> collision_objects;

	collision_objects.push_back(
			createCollisionBox(group, "box2", 0.25, 0.05, 0.25, 0.0, 0.2,
					0.125 + 0.3, 0.0, 0.0, 0.0, 1.0));

	tf::Quaternion q = tf::createQuaternionFromRPY(0.0, 0.0, M_PI / 2);
	collision_objects.push_back(
			createCollisionBox(group, "box1", 0.25, 0.05, 0.25, -0.125,
					0.2 - 0.125, 0.125 + 0.3, q.getX(), q.getY(), q.getZ(),
					q.getW()));
	collision_objects.push_back(
			createCollisionBox(group, "box3", 0.25, 0.05, 0.25, 0.0, 0.35, 0.13,
					0.0, 0.0, 0.0, 1.0));
	tf::Quaternion q2 = tf::createQuaternionFromRPY(M_PI / 4, 0.0, 0.0);
	collision_objects.push_back(
			createCollisionBox(group, "box4", 0.25, 0.05, 0.25, 0.25, 0.3, 0.0,
					q2.getX(), q2.getY(), q2.getZ(), q2.getW()));
	q2 *= tf::createQuaternionFromRPY(0.0, M_PI / 4, 0.0);
	collision_objects.push_back(
			createCollisionBox(group, "box5", 0.25, 0.05, 0.25, 0.5, 0.3, 0.0,
					q2.getX(), q2.getY(), q2.getZ(), q2.getW()));
	q2 *= tf::createQuaternionFromRPY(0.0, 0.0, M_PI / 4);
	collision_objects.push_back(
			createCollisionBox(group, "box6", 0.25, 0.05, 0.25, 0.75, 0.3, 0.0,
					q2.getX(), q2.getY(), q2.getZ(), q2.getW()));
	collision_objects.push_back(
			createCollisionBox(group, "boss", 0.01, 0.01, 0.1, 1.0, 1.0, 1.0,
					0.0, 0.0, 0.0, 1.0));

//	std::vector<geometry_msgs::Pose> waypoints;
//
//	geometry_msgs::Pose target_pose3;
//	target_pose3.position.x = 0.0;
//	target_pose3.position.y = -0.4;
//	target_pose3.position.z = 0.27;
//	target_pose3.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.54,
//			0.0, 0.0);
//	waypoints.push_back(target_pose3);
//	waypoints.push_back(target_pose1);

	ROS_INFO("Add an object into the world");
	planning_scene_interface.addCollisionObjects(collision_objects);

	sleep(2.0);

	group.setPlanningTime(10.0);

//	moveit_msgs::RobotTrajectory trajectory;
//	double fraction = group.computeCartesianPath(waypoints, 0.01, 0.0,
//			trajectory);
//
//	ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
//			fraction * 100.0);
	do {
		success = group.plan(my_plan);
		ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
	} while (!success);



	ROS_INFO("Move? (1 - yes, 0 - no)");
	scanf("%d", &ok);

	if (ok == 1) {
		ROS_INFO("Moving the group");
		group.execute(my_plan);
	}

	ros::shutdown();

	return 0;
}
