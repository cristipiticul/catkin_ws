/*
 * pose_reconfigure.cpp
 *
 *  Created on: May 6, 2015
 *      Author: levente
 */

#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <shape_tools/solid_primitive_dims.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <dynamic_reconfigure/server.h>
#include <pick_place_demo/PositionConfig.h>

static const std::string ROBOT_DESCRIPTION = "robot_description";

geometry_msgs::Pose callback_pose;

bool do_plan;

void callback(pick_place_demo::PositionConfig &config, uint32_t level) {
	ROS_INFO("Reconfigure Request: %f %f %f %f %f %f %f", config.orientation_w,
			config.orientation_x, config.orientation_y, config.orientation_z,
			config.position_x, config.position_y, config.position_z);

	callback_pose.orientation.w = config.orientation_w;
	callback_pose.orientation.x = config.orientation_x;
	callback_pose.orientation.y = config.orientation_y;
	callback_pose.orientation.z = config.orientation_z;

	callback_pose.position.x = config.position_x;
	callback_pose.position.y = config.position_y;
	callback_pose.position.z = config.position_z;

	do_plan = config.plan;

}

int main(int argc, char **argv) {
	ros::init(argc, argv, "arm_pick_place");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::NodeHandle nh;
	ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>(
			"collision_object", 10);
	ros::Publisher pub_aco = nh.advertise<moveit_msgs::AttachedCollisionObject>(
			"attached_collision_object", 10);

	dynamic_reconfigure::Server<pick_place_demo::PositionConfig> server;
	dynamic_reconfigure::Server<pick_place_demo::PositionConfig>::CallbackType f;

	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	ros::WallDuration(1.0).sleep();

	moveit::planning_interface::MoveGroup group("arm");
	group.setPlanningTime(45.0);

	moveit::planning_interface::MoveGroup::Plan my_plan;
	bool success = group.plan(my_plan);

	// (Optional) Create a publisher for visualizing plans in Rviz.
	ros::Publisher display_publisher = node_handle.advertise<
			moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path",
			1, true);
	moveit_msgs::DisplayTrajectory display_trajectory;

	// Getting Basic Information
	// ^^^^^^^^^^^^^^^^^^^^^^^^^
	//
	// We can print the name of the reference frame for this robot.
	ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());

	// We can also print the name of the end-effector link for this group.
	ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

	// Planning to a joint-space goal
	// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	//
	// Let's set a joint space goal and move towards it.  This will replace the
	// pose target we set above.
	//
	// First get the current set of joint values for the group.
	/*
	std::vector<double> group_variable_values;
	group.getCurrentState()->copyJointGroupPositions(
			group.getCurrentState()->getRobotModel()->getJointModelGroup(
					group.getName()), group_variable_values);

	 // Now, let's modify one of the joints, plan to the new joint
	 // space goal and visualize the plan.
	 group_variable_values[1] = -1.0;
	 group_variable_values[3] = -1.0;
	 group_variable_values[5] = -1.0;
	 group.setJointValueTarget(group_variable_values);
	 success = group.plan(my_plan);*/

	 ROS_INFO("Visualizing plan 2 (joint space goal) %s",success?"":"FAILED");
	 sleep(3.0);
	 group.move() ;

	/*
	 // Planning to a Pose goal
	 // ^^^^^^^^^^^^^^^^^^^^^^^
	 // We can plan a motion for this group to a desired pose for the
	 // end-effector.*/
	/*geometry_msgs::Pose target_pose1;
	target_pose1.orientation.w = 1.0;
	target_pose1.position.x = 0.2;
	target_pose1.position.y = 0.2;
	target_pose1.position.z = 0.3;
	group.setPoseTarget(target_pose1);

	// Now, we call the planner to compute the plan
	// and visualize it.
	// Note that we are just planning, not asking move_group
	// to actually move the robot.

	success = group.plan(my_plan);

	ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
	sleep(1.0);
	group.move();*/

	ros::Rate r(0.25);
	while (1) {
		if (do_plan) {
			group.setPoseTarget(callback_pose);
			group.move();
		}

		ros::spinOnce();
		r.sleep();
	}

	ros::waitForShutdown();
	return 0;

}
