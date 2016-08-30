/**
 * NOT WORKING!
 * Goal: move the arm using cartesian paths.
 */

#include "ros/ros.h"
#include "ros/publisher.h"
#include "moveit/move_group_interface/move_group.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/DisplayTrajectory.h"

#include <moveit_msgs/DisplayRobotState.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "main");

	ros::NodeHandle node_handle;

	ros::AsyncSpinner spinner(1);
	spinner.start();

	bool success;
	int ok;

	ROS_INFO("Working!");

	moveit::planning_interface::MoveGroup group("arm");

	ROS_INFO("MoveGroup instantiated!");

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
	ROS_INFO("EEF: %s", group.getEndEffectorLink().c_str());

	std::vector<geometry_msgs::Pose> waypoints;

	geometry_msgs::Pose target_pose1;
	target_pose1.orientation = tf::createQuaternionMsgFromRollPitchYaw(-1.54,
			0.0, 0.0);
	target_pose1.position.x = 0.0;
	target_pose1.position.y = 0.40;
	target_pose1.position.z = 0.35;

	moveit::planning_interface::MoveGroup::Plan my_plan;
	group.setStartState(*group.getCurrentState());
	group.setPoseTarget(target_pose1);

	success = group.plan(my_plan);
	ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

	printf("Press any key and press enter to continue.");
	scanf("%*s");

	group.execute(my_plan);




	geometry_msgs::Pose target_pose3;
	target_pose3.position.x = 0.0;
	target_pose3.position.y = -0.4;
	target_pose3.position.z = 0.27;
	target_pose3.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.54,
			0.0, 0.0);

    group.setStartState(*group.getCurrentState());
	group.setPoseTarget(target_pose3);

	success = group.plan(my_plan);
	ROS_INFO("Visualizing plan 2 (pose goal) %s", success ? "" : "FAILED");

	printf("Press any key and press enter to continue.");
	scanf("%*s");

	group.execute(my_plan);




	waypoints.push_back(target_pose1);
	waypoints.push_back(target_pose3);

	moveit_msgs::RobotTrajectory trajectory;
	double fraction = group.computeCartesianPath(waypoints,
	                                             0.01,  // eef_step
	                                             0.0,   // jump_threshold
	                                             trajectory);

	ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
	  fraction * 100.0);

//	group.execute(trajectory);

	spinner.stop();

	return 0;
}

