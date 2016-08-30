#include "utils.h"

#include "moveit/move_group_interface/move_group.h"
#include "moveit_msgs/PlanningScene.h"
#include "ros/ros.h"
#include "ros/publisher.h"
#include "gripper_control/GripperCommand.h"
#include <tf/transform_listener.h>
#include <iostream>

double to_radians(double angle) {
	return M_PI * angle / 180.0;
}

const double ARM_ROTATION_RADIUS = 0.25;
const double ARM_ROTATION_ANGLE = to_radians(22);
const double ARM_ROTATION_CENTER_Y = -0.12;
const double ARM_POSITION_Z = 0.35;

using namespace std;
using namespace geometry_msgs;

typedef struct {
	geometry_msgs::Pose bottlePose;
	geometry_msgs::Pose bottleCapPose;
} BottlePosition;

geometry_msgs::Point tfVector3ToPoint(tf::Vector3 vector) {
	geometry_msgs::Point position;
	position.x = vector.getX();
	position.y = vector.getY();
	position.z = vector.getZ();
	return position;
}

moveit_msgs::CollisionObject createBottleObject(geometry_msgs::Point position) {
	moveit_msgs::CollisionObject bottle;

	/* The header must contain a valid TF frame*/
	bottle.header.frame_id = "base_footprint";
	/* The id of the object */
	bottle.id = "bottle";

	/* A default pose */
	geometry_msgs::Pose pose;
	pose.orientation.w = 1.0;
	pose.position = position;

	/* Define a box to be attached */
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 0.1;
	primitive.dimensions[1] = 0.1;
	primitive.dimensions[2] = 0.1;

	bottle.primitives.push_back(primitive);
	bottle.primitive_poses.push_back(pose);

	return bottle;
}

// get both poses from TF at the same time
bool getBottlePositionFromTf(BottlePosition &result) {
	bool success = true;

	tf::TransformListener listener;
	tf::StampedTransform bottleTransform;
	tf::StampedTransform bottleCapTransform;
	try {
		ros::Time now = ros::Time(0);
		listener.waitForTransform("/base_footprint", "/bottle", now,
				ros::Duration(5.0));
		listener.waitForTransform("/base_footprint", "/bottle_cap", now,
				ros::Duration(5.0));
		listener.lookupTransform("/base_footprint", "/bottle", now,
				bottleTransform);
		listener.lookupTransform("/base_footprint", "/bottle_cap", now,
				bottleCapTransform);
	} catch (tf::TransformException &ex) {
		success = false;
		ROS_WARN("The bottle was not found!");
	}

	if (success) {
		result.bottlePose.position.x = bottleTransform.getOrigin().getX();
		result.bottlePose.position.y = bottleTransform.getOrigin().getY();
		result.bottlePose.position.z = bottleTransform.getOrigin().getZ();
		tf::quaternionTFToMsg(bottleTransform.getRotation(),
				result.bottlePose.orientation);

		result.bottleCapPose.position.x = bottleCapTransform.getOrigin().getX();
		result.bottleCapPose.position.y = bottleCapTransform.getOrigin().getY();
		result.bottleCapPose.position.z = bottleCapTransform.getOrigin().getZ();
		tf::quaternionTFToMsg(bottleCapTransform.getRotation(),
				result.bottleCapPose.orientation);

		return true;
	} else {
		return false;
	}
}

const ros::Duration stabilisation_duration(3.0);

void testForPose(moveit::planning_interface::MoveGroup &group,
		geometry_msgs::Pose initial_pose) {
	geometry_msgs::Pose destination;
	BottlePosition result;

	if (!moveTo(group, initial_pose)) {
		ROS_ERROR("No plan was found!");
		return;
	}
	stabilisation_duration.sleep();

	bool found_bottle = getBottlePositionFromTf(result);
	geometry_msgs::Point bottleCapPosition = result.bottleCapPose.position;
	ROS_INFO("Found bottle cap at: %f %f %f", bottleCapPosition.x,
			bottleCapPosition.y, bottleCapPosition.z);

	destination.position.x = bottleCapPosition.x;
	destination.position.y = bottleCapPosition.y;
	destination.position.z = bottleCapPosition.z + 0.03;
	destination.orientation = initial_pose.orientation;

	if (!moveTo(group, destination)) {
		ROS_ERROR("No plan was found!");
		return;
	}

	stabilisation_duration.sleep();

	if (!moveTo(group, initial_pose)) {
		ROS_ERROR("No plan was found!");
		return;
	}
	stabilisation_duration.sleep();
}

vector<Pose> computeArmPosesForDetection() {
	vector<Pose> arm_poses;
	Pose arm_pose_front;
	Pose arm_pose_left;
	Pose arm_pose_right;

	arm_pose_front.position.x = 0.0;
	arm_pose_front.position.y = ARM_ROTATION_RADIUS + ARM_ROTATION_CENTER_Y;
	arm_pose_front.position.z = ARM_POSITION_Z;
	tf::Quaternion q_front = tf::createQuaternionFromRPY(-M_PI / 2, 0.0, M_PI);
	tf::quaternionTFToMsg(q_front, arm_pose_front.orientation);

	arm_pose_left.position.x = ARM_ROTATION_RADIUS * sin(ARM_ROTATION_ANGLE);
	arm_pose_left.position.y = ARM_ROTATION_CENTER_Y
			+ ARM_ROTATION_RADIUS * cos(ARM_ROTATION_ANGLE);
	arm_pose_left.position.z = ARM_POSITION_Z;
	tf::Quaternion q_left = tf::createQuaternionFromRPY(-M_PI / 2, 0.0,
	M_PI - ARM_ROTATION_ANGLE);
	tf::quaternionTFToMsg(q_left, arm_pose_left.orientation);

	arm_pose_right.position.x = ARM_ROTATION_RADIUS * sin(-ARM_ROTATION_ANGLE);
	arm_pose_right.position.y = ARM_ROTATION_CENTER_Y
			+ ARM_ROTATION_RADIUS * cos(-ARM_ROTATION_ANGLE);
	arm_pose_right.position.z = ARM_POSITION_Z;
	tf::Quaternion q_right = tf::createQuaternionFromRPY(-M_PI / 2, 0.0,
	M_PI + ARM_ROTATION_ANGLE);
	tf::quaternionTFToMsg(q_right, arm_pose_right.orientation);

	arm_poses.push_back(arm_pose_front);
	arm_poses.push_back(arm_pose_left);
	arm_poses.push_back(arm_pose_right);

	return arm_poses;
}

BottlePosition computeBottlePoseFromMeasurements(
		vector<BottlePosition> measurements) {
	BottlePosition result;
	result.bottlePose.position.x = result.bottlePose.position.y =
			result.bottlePose.position.z = 0.0;
	result.bottlePose.orientation.x = result.bottlePose.orientation.y =
			result.bottlePose.orientation.z = 0.0;
	result.bottlePose.orientation.w = 1.0;

	result.bottleCapPose.position.x = result.bottleCapPose.position.y =
			result.bottleCapPose.position.z = 0.0;
	result.bottleCapPose.orientation.x = result.bottleCapPose.orientation.y =
			result.bottleCapPose.orientation.z = 0.0;
	result.bottleCapPose.orientation.w = 1.0;

	int number_of_measurements = measurements.size();

	for (vector<BottlePosition>::iterator measurement = measurements.begin();
			measurement != measurements.end(); measurement++) {
		result.bottlePose.position.x += measurement->bottlePose.position.x;
		result.bottlePose.position.y += measurement->bottlePose.position.y;
		result.bottlePose.position.z += measurement->bottlePose.position.z;

		result.bottleCapPose.position.x +=
				measurement->bottleCapPose.position.x;
		result.bottleCapPose.position.y +=
				measurement->bottleCapPose.position.y;
		result.bottleCapPose.position.z +=
				measurement->bottleCapPose.position.z;
	}
	result.bottlePose.position.x /= number_of_measurements;
	result.bottlePose.position.y /= number_of_measurements;
	result.bottlePose.position.z /= number_of_measurements;

	result.bottleCapPose.position.x /= number_of_measurements;
	result.bottleCapPose.position.y /= number_of_measurements;
	result.bottleCapPose.position.z /= number_of_measurements;

	return result;
}

BottlePosition findBottlePose(moveit::planning_interface::MoveGroup &group) {
	BottlePosition result;
	vector<Pose> arm_poses = computeArmPosesForDetection();
	vector<BottlePosition> measurements;

	for (vector<Pose>::iterator pose = arm_poses.begin();
			pose != arm_poses.end(); pose++) {
		if (!moveTo(group, *pose)) {
			throw std::runtime_error(
					"Error in determining bottle pose: The arm could not move to the given pose.");
		}
		stabilisation_duration.sleep();

		BottlePosition bottle_position;
		bool found_bottle = getBottlePositionFromTf(bottle_position);
		if (found_bottle) {
			measurements.push_back(bottle_position);

			ROS_INFO("Found bottle at:");
			printPose(measurements.back().bottlePose);
			ROS_INFO("And cap at:");
			printPose(measurements.back().bottleCapPose);
		} else {
			ROS_WARN("Measurement is ignored");
		}
	}
	if (measurements.size() == 0) {
		ROS_ERROR("The bottle was not found from any pose. The program will shut down!");
		exit(-1);
	}
	result = computeBottlePoseFromMeasurements(measurements);

	ROS_INFO("Average pose:");
	printPose(result.bottlePose);
	ROS_INFO("Average cap pose:");
	printPose(result.bottleCapPose);

	return result;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "main");

	ros::NodeHandle node_handle;

	ros::Duration sleep_time(10.0);
	ros::Duration gripper_command_sleep_time(1.0);
	const double distance_to_bottle = 0.05;

	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::ServiceClient gripper_service_client = node_handle.serviceClient<
			gripper_control::GripperCommand>("command_gripper");

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	std::vector<moveit_msgs::CollisionObject> collision_objects;
	int ok;
	BottlePosition bottlePosition;

	ROS_INFO("Working!");

	moveit::planning_interface::MoveGroup group("arm");
	ROS_INFO("MoveGroup instantiated!");

	group.setPlannerId("RRTConnectkConfigDefault");

	ROS_INFO("Waiting 5 seconds to initialize...");
	sleep(5.0);

	group.setPlanningTime(0.1);

	ros::Publisher planning_scene_diff_publisher = node_handle.advertise<
			moveit_msgs::PlanningScene>("planning_scene", 1);
	while (planning_scene_diff_publisher.getNumSubscribers() < 1) {
		ros::WallDuration sleep_t(0.5);
		sleep_t.sleep();
	}

	tf::Quaternion q = tf::createQuaternionFromRPY(0.0, 0.0, 0.0);
	collision_objects.push_back(
			createCollisionBox(group, "ground", 1.5, 1.5, 0.5, 0.0, 0.0, -0.25,
					q.getX(), q.getY(), q.getZ(), q.getW()));

	ROS_INFO("Add the ground to the world");
	planning_scene_interface.addCollisionObjects(collision_objects);

	bottlePosition = findBottlePose(group);

	geometry_msgs::Pose goal_pose = bottlePosition.bottleCapPose;
	// NOT ANYMORE Because the planner moves the "wrist_roll" to the specified coordinates,
	// add 1cm to the X coordinate.
	// NO LONGER NEEDED: goal_pose.position.x += 0.1;
	// Go above the bottle cap
	goal_pose.position.y += distance_to_bottle;

	ROS_INFO("Add the bottle to the planning scene.");
	geometry_msgs::Pose bottle_pose = bottlePosition.bottlePose;
	moveit_msgs::CollisionObject bottle = createBottleObject(
			bottle_pose.position);
	moveit_msgs::PlanningScene planning_scene;
	/*
	 TODO: Put these back (if we want no bottle collision)
	 planning_scene.world.collision_objects.push_back(bottle);
	 planning_scene.is_diff = true;
	 planning_scene_diff_publisher.publish(planning_scene);
	 sleep_time.sleep();
	 */

	gripper_control::GripperCommand open_gripper_command;
	open_gripper_command.request.action = 1;

	if (gripper_service_client.call(open_gripper_command)) {
		ROS_INFO("Opening gripper...");
	} else {
		ROS_ERROR("Failed to call the command_gripper service!");
		return -1;
	}

	gripper_command_sleep_time.sleep();
	ROS_INFO("The gripper is opened.");

	tf::Quaternion orientation = tf::createQuaternionFromRPY(-M_PI / 2, 0.0,
			M_PI);
	tf::quaternionTFToMsg(orientation, goal_pose.orientation);
	moveTo(group, goal_pose);

	moveForward(group, distance_to_bottle);

	gripper_control::GripperCommand close_gripper_command;
	close_gripper_command.request.action = 2;

	if (gripper_service_client.call(close_gripper_command)) {
		ROS_INFO("Closing gripper...");
	} else {
		ROS_ERROR("Failed to call the command_gripper service!");
		return -1;
	}

	gripper_command_sleep_time.sleep();
	ROS_INFO("The gripper is closed.");

	/*
	 * TODO: Put these back (if we want no bottle collision)
	moveit_msgs::CollisionObject remove_object;
	remove_object.id = "bottle";
	remove_object.header.frame_id = "base_footprint";
	remove_object.operation = remove_object.REMOVE;

	moveit_msgs::AttachedCollisionObject attached_object;
	attached_object.link_name = "virtual_endeffector";
	attached_object.object = bottle;
	attached_object.object.operation = attached_object.object.ADD;

	ROS_INFO("Attaching the bottle to the arm, removing it from the world.");
	planning_scene.world.collision_objects.clear();
	planning_scene.world.collision_objects.push_back(remove_object);
	planning_scene.robot_state.attached_collision_objects.push_back(
			attached_object);
	planning_scene_diff_publisher.publish(planning_scene);
	*/

//	moveTo(group, 0.04, 0.0, 0.63, 0.0, 0.0, 0.0);

	return 0;
}
