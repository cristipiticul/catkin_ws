#include "moveit/move_group_interface/move_group.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"

#include "tf/transform_datatypes.h"

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>

enum Axis {
  X_AXIS, Y_AXIS, Z_AXIS
};

geometry_msgs::Quaternion createQuaternion(double angle, double nx, double ny,
    double nz) {
  geometry_msgs::Quaternion quaternion;
  quaternion.w = cos(angle);
  quaternion.x = nx * sin(angle);
  quaternion.y = ny * sin(angle);
  quaternion.z = nz * sin(angle);
  return quaternion;
}


void printPose(geometry_msgs::Pose pose) {
    ROS_INFO("X Y Z RX RY RZ RW:\n%.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z,
            pose.orientation.w);
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

bool moveTo(moveit::planning_interface::MoveGroup& group,
    geometry_msgs::Pose target_pose) {
  bool success;
  int ok;
  moveit::planning_interface::MoveGroup::Plan my_plan;

  group.setStartState(*group.getCurrentState());

  group.setPoseTarget(target_pose);

  ROS_INFO("Target pose:");
  printPose(target_pose);

  success = false;
  for (int i = 1; i <= 3 && !success; i++) {
      ROS_INFO("Planning try #%d...", i);
      success = group.plan(my_plan);
  }
  if (!success) {
    ROS_INFO("Planning failed!");
    return false;
  }

  ROS_INFO("Move? (1 - yes, 0 - no)");
  scanf("%d", &ok);
//  ok = 1;

  if (ok == 1) {
    ROS_INFO("Moving the group");
    group.execute(my_plan);

    ros::Duration(1.0).sleep();
    geometry_msgs::PoseStamped current_pose = group.getCurrentPose(
        "virtual_endeffector");
    ROS_INFO("Reached pose:");
    printPose(current_pose.pose);

    return true;
  } else {
    return false;
  }
}

bool moveTo(moveit::planning_interface::MoveGroup& group, double x, double y,
    double z, double roll, double pitch, double yaw) {
  geometry_msgs::Pose target_pose;

  tf::Quaternion orientation = tf::createQuaternionFromRPY(roll, pitch, yaw);
  tf::quaternionTFToMsg(orientation, target_pose.orientation);
  target_pose.position.x = x;
  target_pose.position.y = y;
  target_pose.position.z = z;

  return moveTo(group, target_pose);
}

/**
 * When we need to keep the "virtual_endeffector" link inside a specific volume during
 * the path execution, use this kind of constraint. It will not allow the
 * "virtual_endeffector" link to exit the volume in its motion. Useful when we need
 * the arm to move on a linear trajectory along one of the axis.
 *
 * Parameters:
 *  - x, y, z: the position of the center of the box.
 *  - lx, ly, lz: the size of the box.
 */
moveit_msgs::PositionConstraint createPositionConstraint(
    geometry_msgs::Point position, double lx, double ly, double lz) {
  moveit_msgs::PositionConstraint position_constraint =
      moveit_msgs::PositionConstraint();
  position_constraint.header.frame_id = "base_footprint";
  shape_msgs::SolidPrimitive bounding_box;
  bounding_box.type = bounding_box.BOX;
  bounding_box.dimensions.resize(3);
  bounding_box.dimensions[bounding_box.BOX_X] = lx;
  bounding_box.dimensions[bounding_box.BOX_Y] = ly;
  bounding_box.dimensions[bounding_box.BOX_Z] = lz;

  geometry_msgs::Pose pose;
  pose.position.x = position.x;
  pose.position.y = position.y;
  pose.position.z = position.z;
  pose.orientation.w = 1.0;

  position_constraint.constraint_region.primitives.push_back(bounding_box);
  position_constraint.constraint_region.primitive_poses.push_back(pose);

  position_constraint.weight = 1.0f;
  position_constraint.link_name = "virtual_endeffector";

  return position_constraint;
}

/**
 * Constraint for the "virtual_endeffector" link to move along the OX axis.
 */
moveit_msgs::PositionConstraint createXPositionConstraint(
    geometry_msgs::Point position) {
  return createPositionConstraint(position, 1.0, 0.1, 0.1);
}

/**
 * Constraint for the "virtual_endeffector" link to move along the OY axis.
 */
moveit_msgs::PositionConstraint createYPositionConstraint(
    geometry_msgs::Point position) {
  return createPositionConstraint(position, 0.1, 1.0, 0.1);
}

/**
 * Constraint for the "virtual_endeffector" link to move along the OZ axis.
 */
moveit_msgs::PositionConstraint createZPositionConstraint(
    geometry_msgs::Point position) {
  return createPositionConstraint(position, 0.1, 0.1, 1.0);
}

/**
 * Constraint for the "virtual_endeffector" link to keep its orientation during the
 * movement.
 */
moveit_msgs::OrientationConstraint createOrientationConstraint(
    geometry_msgs::Quaternion orientation) {
  moveit_msgs::OrientationConstraint orientation_constraint =
      moveit_msgs::OrientationConstraint();
  orientation_constraint.header.frame_id = "base_footprint";
  orientation_constraint.orientation = orientation;
  orientation_constraint.absolute_x_axis_tolerance = 0.125;
  orientation_constraint.absolute_y_axis_tolerance = 0.125;
  orientation_constraint.absolute_z_axis_tolerance = 0.125;
  orientation_constraint.weight = 1.0f;
  orientation_constraint.link_name = "virtual_endeffector";

  return orientation_constraint;
}

bool moveOnAxisTo(moveit::planning_interface::MoveGroup& group,
    geometry_msgs::Pose target_pose, Axis axis) {
  moveit_msgs::Constraints path_constraints = moveit_msgs::Constraints();
  path_constraints.name = "tcp_orientation_constraint";

  /** Orientation constraint: the arm orientation must not change. */
  moveit_msgs::OrientationConstraint orientation_constraint =
      createOrientationConstraint(target_pose.orientation);
  /** End of orientation constraint definition. */

  path_constraints.orientation_constraints.push_back(orientation_constraint);

  /** Position constraint: move only on OZ axis */
  moveit_msgs::PositionConstraint position_constraint;
  if (axis == X_AXIS) {
    position_constraint = createXPositionConstraint(target_pose.position);
  } else if (axis == Y_AXIS) {
    position_constraint = createYPositionConstraint(target_pose.position);
  } else if (axis == Z_AXIS) {
    position_constraint = createZPositionConstraint(target_pose.position);
  }
  /** End of position constraint definition. */

  path_constraints.position_constraints.push_back(position_constraint);

  group.setPathConstraints(path_constraints);

  bool success = moveTo(group, target_pose);

  // Remove the path constraints after planning in order to leave
  // the MoveGroup to the same state as the initial state.
  group.clearPathConstraints();

  return success;
}

bool moveOnAxisTo(moveit::planning_interface::MoveGroup& group, double x,
    double y, double z, double roll, double pitch, double yaw, Axis axis) {
  geometry_msgs::Pose target_pose;

  tf::Quaternion orientation = tf::createQuaternionFromRPY(roll, pitch, yaw);
  tf::quaternionTFToMsg(orientation, target_pose.orientation);
  target_pose.position.x = x;
  target_pose.position.y = y;
  target_pose.position.z = z;

  return moveOnAxisTo(group, target_pose, axis);
}

void moveLeft(moveit::planning_interface::MoveGroup &group, double distance) {
  geometry_msgs::Pose next_pose = group.getCurrentPose("virtual_endeffector").pose;
//  tf::Quaternion orientation = tf::createQuaternionFromRPY(-M_PI / 2, 0.0,
//      0.0);
//  tf::quaternionTFToMsg(orientation, next_pose.orientation);
  next_pose.position.x += distance;

  moveOnAxisTo(group, next_pose, X_AXIS);
}
void moveRight(moveit::planning_interface::MoveGroup &group, double distance) {
  geometry_msgs::Pose next_pose = group.getCurrentPose("virtual_endeffector").pose;
//  tf::Quaternion orientation = tf::createQuaternionFromRPY(-M_PI / 2, 0.0,
//      0.0);
//  tf::quaternionTFToMsg(orientation, next_pose.orientation);
  next_pose.position.x -= distance;

  moveOnAxisTo(group, next_pose, X_AXIS);
}
void moveForward(moveit::planning_interface::MoveGroup &group, double distance) {
  geometry_msgs::Pose next_pose = group.getCurrentPose("virtual_endeffector").pose;
//  tf::Quaternion orientation = tf::createQuaternionFromRPY(-M_PI / 2, 0.0,
//      0.0);
//  tf::quaternionTFToMsg(orientation, next_pose.orientation);
  next_pose.position.y -= distance;

  moveOnAxisTo(group, next_pose, Y_AXIS);
}
void moveBack(moveit::planning_interface::MoveGroup &group, double distance) {
  geometry_msgs::Pose next_pose = group.getCurrentPose("virtual_endeffector").pose;
//  tf::Quaternion orientation = tf::createQuaternionFromRPY(-M_PI / 2, 0.0,
//      0.0);
//  tf::quaternionTFToMsg(orientation, next_pose.orientation);
  next_pose.position.y += distance;

  moveOnAxisTo(group, next_pose, Y_AXIS);
}
void moveUp(moveit::planning_interface::MoveGroup &group, double distance) {
  geometry_msgs::Pose next_pose = group.getCurrentPose("virtual_endeffector").pose;
//  tf::Quaternion orientation = tf::createQuaternionFromRPY(-M_PI / 2, 0.0,
//      0.0);
//  tf::quaternionTFToMsg(orientation, next_pose.orientation);
  next_pose.position.z += distance;

  moveOnAxisTo(group, next_pose, Z_AXIS);
}
void moveDown(moveit::planning_interface::MoveGroup &group, double distance) {
  geometry_msgs::Pose next_pose = group.getCurrentPose("virtual_endeffector").pose;
//  tf::Quaternion orientation = tf::createQuaternionFromRPY(-M_PI / 2, 0.0,
//      0.0);
//  tf::quaternionTFToMsg(orientation, next_pose.orientation);
  next_pose.position.z -= distance;

  moveOnAxisTo(group, next_pose, Z_AXIS);
}
