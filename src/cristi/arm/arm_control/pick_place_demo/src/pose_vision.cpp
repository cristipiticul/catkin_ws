/*
 ` * pose_reconfigure.cpp
 *
 *  Created on: May 6, 2015
 *      Author: levente
 */

#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/move_group_interface/move_group.h>
#include <shape_tools/solid_primitive_dims.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <dynamic_reconfigure/server.h>
#include <pick_place_demo/PositionConfig.h>

//TF listener
#include <tf/transform_listener.h>

//Math 
#include <math.h>
//#include <Quaternion.h>
#include <iostream>

using namespace std;

static const std::string ROBOT_DESCRIPTION = "robot_description";

geometry_msgs::Pose callback_pose;

// Planning constants
const double POS_TOLARENCE = 0.01, ANG_TOLARENCE = 0.1, PLANING_TIME = 20;

// Relative position and orientation constants
const double roll_ref = -M_PI / 2.0, pitch_ref = M_PI / 2.0, yaw_ref = 0;
const double ox_dist = 0.0, oy_dist = 0.0, oz_dist = 0.0;


template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
// working OK
void printPosition(geometry_msgs::Pose p, string lable) {
	tf::Pose p_;
	double roll, pitch, yaw, ox, oy, oz;
	tf::poseMsgToTF(p, p_);

	ox = p_.getOrigin().x();
	oy = p_.getOrigin().y();
	oz = p_.getOrigin().z();

	p_.getBasis().getEulerYPR(yaw, pitch, roll);

	cout << lable << " : ";
	printf("pos [x,y,z]=[%f, %f, %f] ori [y, p, r]=[%f, %f, %f] \n", ox, oy, oz,
			yaw, roll, pitch);
}

double radToDeg(double radian) {
	return radian * 180.0 / M_PI;
}

double degToRad(double radian) {
	return radian / 180.0 * M_PI;
}

void getTargetPoseFromChessBoard(geometry_msgs::Pose &current_pose,
		tf::StampedTransform chessB_pose) {

	tf::Pose p_;
	tf::Quaternion q_, q_err;
	tf::Vector3 c_;
	double roll, pitch, yaw, ox, oy, oz, target_y, target_p, target_r; //orientation vars
	double target_oy, target_ox, target_oz;	 // position vars

	// get current orientation and pose in p_
	tf::poseMsgToTF(current_pose, p_);
	/*
	 // calculate desired orientatin
	 chessB_pose.getBasis().getEulerYPR(yaw, pitch, roll); // get measurmetns
	 target_y = pitch;
	 target_r = roll;
	 target_p = yaw;
	 q_.setEuler(target_y, target_p, target_r);
	 q_.normalize();

	 cout << "Board orientation " << radToDeg(yaw) << ", " << radToDeg(pitch)
	 << ", " << radToDeg(roll) << endl;
	 */
	// calculate translation
	target_ox = chessB_pose.getOrigin().x() - ox_dist;
	target_oy = chessB_pose.getOrigin().y() - oy_dist;
	target_oz = chessB_pose.getOrigin().z() - oz_dist;

	cout << "Distance x,y,z: "
			<< abs(chessB_pose.getOrigin().x() - p_.getOrigin().getX()) << " "
			<< abs(chessB_pose.getOrigin().y() - p_.getOrigin().getY()) << " "
			<< abs(chessB_pose.getOrigin().z() - p_.getOrigin().getZ()) << endl
			<< endl;

	c_.setValue(target_ox, target_oy, target_oz);
	p_.setOrigin(c_);
	q_ = p_.getRotation(); // FOR TESTING DISTANCE
	p_.setRotation(q_);
	tf::poseTFToMsg(p_, current_pose);
	printPosition(current_pose, "Target: ");
}

geometry_msgs::Pose moveToInitialPos(
		moveit::planning_interface::MoveGroup &group) {
	geometry_msgs::Pose target_init;
	target_init.orientation.w = 0.7071;
	target_init.orientation.x = -0.7071;
	target_init.orientation.y = 0.0;
	target_init.orientation.z = 0.0;
	target_init.position.x = 0.1;
	target_init.position.y = 0.1;
	target_init.position.z = 0.3;
	group.setPoseTarget(target_init);

	moveit::planning_interface::MoveGroup::Plan my_plan;

	bool success = group.plan(my_plan);
	ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
	sleep(5.0);

	if (success) {
		//group.move();
		group.execute(my_plan);
		//printPosition(target_init, "init");
	} else
		printf(
				"Error while planning initial position of arm, no move command was sent!");
	sleep(2.0);
	return target_init;
}

std::vector<moveit_msgs::CollisionObject> creatCollisionObjChessboard(
		tf::StampedTransform cameraGrid) {

	tf::Quaternion q_;
	moveit_msgs::CollisionObject obj;
	std::vector<moveit_msgs::CollisionObject> collision_objects;
	obj.header.frame_id = "base_footprint"; //group.getPlanningFrame();

	/* The id of the object is used to identify it. */
	obj.id = "ChessBoard";

	/* Define a box to add to the world. */
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 0.06;  // seams to be x
	primitive.dimensions[1] = 0.08; // seams to be y
	primitive.dimensions[2] = 0.005;  // seams to be z

	/* A pose for the box (specified relative to frame_id) */
	geometry_msgs::Pose box_pose;

	cameraGrid.getBasis().getRotation(q_);
	q_.normalize();
	box_pose.orientation.w = cameraGrid.getRotation().getW();
	box_pose.orientation.x = cameraGrid.getRotation().getX();
	box_pose.orientation.y = cameraGrid.getRotation().getY();
	box_pose.orientation.z = cameraGrid.getRotation().getZ();
	box_pose.position.x = cameraGrid.getOrigin().x();
	box_pose.position.y = cameraGrid.getOrigin().y();
	box_pose.position.z = cameraGrid.getOrigin().z();

	obj.primitives.push_back(primitive);
	obj.primitive_poses.push_back(box_pose);
	obj.operation = obj.ADD;

	collision_objects.push_back(obj);

	return collision_objects;
}

void pauseIN(string a) {
	cout << " PRESS ENTER TO CONTINUE " << a;
	cin >> a;
	cout << " ..." << endl;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "move_group_interface_tutorial");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();
	bool success;
	geometry_msgs::Pose target_init;
	geometry_msgs::Pose target_current;

	tf::TransformListener listener_chessBoard, tf_listener_;
	tf::Quaternion target_orientation, chessboard_quater;
	tf::StampedTransform transform_cameraToGrid;
	std::vector<std::string> object_ids;
	object_ids.push_back("ChessBoard");

	// The :move_group_interface:`MoveGroup` ("group name")
	moveit::planning_interface::MoveGroup group("arm");
	group.setPlanningTime(PLANING_TIME);
	group.setGoalOrientationTolerance(ANG_TOLARENCE);
	group.setGoalPositionTolerance(POS_TOLARENCE);

	// We will use the :planning_scene_interface:`PlanningSceneInterface`
	// class to deal directly with the world.
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	moveit::planning_interface::MoveGroup::Plan main_plan;

	// (Optional) Create a publisher for visualizing plans in Rviz.
	ros::Publisher display_publisher = node_handle.advertise<
			moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path",
			1, true);
	moveit_msgs::DisplayTrajectory display_trajectory;

	// We can print the name of the reference frame for this robot.
	ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());

	// We can also print the name of the end-effector link for this group.
	ROS_INFO("Reference eef: %s", group.getEndEffectorLink().c_str());

	group.setStartState(*group.getCurrentState());
	// Initial position to look on forward to the chess board
	target_current = moveToInitialPos(group);
	target_init = target_current;
	//group.setStartState(*group.getCurrentState());
	ros::Rate rate(10.0);

	while (node_handle.ok()) {
		//=== Listener to TF chess board topic

		try {
			//listener_chessBoard.
			cout << "  -->> Read chess pose" << endl;
			listener_chessBoard.lookupTransform("/base_footprint",
					"/calibration_grid", ros::Time(0), transform_cameraToGrid);

			// remove old chess board collision object
			cout << "  -->>  Add collision box" << endl;
			planning_scene_interface.removeCollisionObjects(object_ids);
			sleep(2.0);
			// get desired position relative to chess board position
			getTargetPoseFromChessBoard(target_current, transform_cameraToGrid);
			//add chess board as a collision object
			planning_scene_interface.addCollisionObjects(
					creatCollisionObjChessboard(transform_cameraToGrid));
			sleep(2.0);



			//Path Constrain
			// define an orientation constrain on the wrist roll link
			/*
			moveit_msgs::OrientationConstraint ocm;
			ocm.link_name = "wrist_roll";
			ocm.header.frame_id = "base_link";
			ocm.orientation.w = target_current.orientation.w;
			ocm.orientation.x = target_current.orientation.x;
			ocm.orientation.y = target_current.orientation.y;
			ocm.orientation.z = target_current.orientation.z;
			ocm.absolute_x_axis_tolerance = 0.1;
			ocm.absolute_y_axis_tolerance = 0.1;
			ocm.absolute_z_axis_tolerance = 0.1;
			ocm.weight = 1.0;
			// set to the group
			moveit_msgs::Constraints test_constraints;
			test_constraints.orientation_constraints.push_back(ocm);
			group.setPathConstraints(test_constraints);



			//=== Plane relative positioning from chess board
			cout << "  -->>   Plan to target chessB ";
			group.setPoseTarget(target_current);
			success = group.plan(main_plan);
			sleep(2.0);
			(cout << (success)) ? "" : "FAILD";
			cout << endl;
*/

			// Cartesian Paths
			std::vector<geometry_msgs::Pose> waypoints;
			geometry_msgs::Pose target_waypoint = target_init;

			double signX, signY, signZ, step = 0.08;

			signX = sgn(target_current.position.x - target_init.position.x);
			signY = sgn(target_current.position.y - target_init.position.y);
			signZ = sgn(target_current.position.z - target_init.position.z);

			cout << "sign " << signX << " " << signY << " " << signZ << endl;
			int i = 0;
			cout << "start: " << target_init.position.z << " " << target_init.position.y << " " << target_init.position.x << endl;
			cout << "end: " << target_current.position.z << " " << target_current.position.y << " " << target_current.position.x << endl;
			//pauseIN("do traj");

			do{

				target_waypoint.position.z = (abs(target_current.position.z - (target_waypoint.position.z )) <= step)
								? target_current.position.z
										: target_waypoint.position.z + signZ*step ;

				target_waypoint.position.y = (abs(target_current.position.y - (target_waypoint.position.y )) <= step)
								? target_current.position.y
										: target_waypoint.position.y + signY*step ;

				target_waypoint.position.x = (abs(target_current.position.x - (target_waypoint.position.x )) <= step)
								? target_current.position.x
										: target_waypoint.position.x + signX*step ;

				waypoints.push_back(target_waypoint);

				i++;
				cout << "step " << i << ": ";
				cout << target_waypoint.position.z << " " << target_waypoint.position.y << " " << target_waypoint.position.x << endl;


			}while(target_waypoint.position.x != target_current.position.x ||
					target_waypoint.position.y != target_current.position.y ||
					target_waypoint.position.z != target_current.position.z);



			moveit_msgs::RobotTrajectory trajectory;

			double fraction = group.computeCartesianPath(waypoints, 0.02, // eef_step
					0.0,   // jump_threshold
					trajectory);

			ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)",
					fraction * 100.0);
			/* Sleep to give Rviz time to visualize the plan. */
			sleep(5.0);

			main_plan.trajectory_=trajectory;
			//pauseIN("move to chess board");
			//group.move();
			//if (success) {
			if (0.25 <= fraction) {
				cout << "  -->>  Move to target" << endl;
				group.execute(main_plan);
				target_init = target_current;
				sleep(10.0);
			}
			//pauseIN("read chess pose");

		} catch (tf::TransformException ex) {
			ROS_ERROR("%s", ex.what());

			target_current = moveToInitialPos(group);
			target_init = target_current;
			cout << "  -->>  Initial pose" << endl;
			sleep(5.0);

			pauseIN(" reSTART");
			ros::Duration(1.0).sleep();

		}

		rate.sleep();
	}

	ros::waitForShutdown();
	return 0;

}
