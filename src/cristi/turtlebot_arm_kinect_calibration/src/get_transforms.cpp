#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>



using namespace std;
using namespace Eigen;


Eigen::Matrix4f EigenFromTF(tf::Transform trans)
{
  Eigen::Matrix4f out;
  tf::Quaternion quat = trans.getRotation();
  tf::Vector3 origin = trans.getOrigin();

  Eigen::Quaternionf quat_out(quat.w(), quat.x(), quat.y(), quat.z());
  Eigen::Vector3f origin_out(origin.x(), origin.y(), origin.z());

  out.topLeftCorner<3, 3>() = quat_out.toRotationMatrix();
  out.topRightCorner<3, 1>() = origin_out;
  out(3, 3) = 1;

  return out;
}



int main(int argc, char** argv) {
	ros::init(argc, argv, "my_tf_listener");

	ros::NodeHandle node;

	tf::TransformListener listener;

	ros::Rate rate(10.0);
	tf::StampedTransform transform_BG; //Base-Gripper
	try {
		ros::Time now = ros::Time::now();
		listener.waitForTransform("/base_link", "/gripper_finger1", ros::Time(0),
				ros::Duration(100.0));
		listener.lookupTransform("/base_link", "/gripper_finger1",ros::Time(0), transform_BG);
		ROS_INFO("Got the first transform from base-->gripper!\n");
	} catch (tf::TransformException &ex) {
		ROS_ERROR("%s", ex.what());
		ros::Duration(1.0).sleep();
	}

	tf::StampedTransform transform_BC; //Base-Camera
	try {
		ros::Time now = ros::Time::now();
		listener.waitForTransform( "/base_link","/camera_link", ros::Time(0),
				ros::Duration(100.0));
		listener.lookupTransform("/base_link","/camera_link",ros::Time(0), transform_BC);
		ROS_INFO("Got the second transform from base-->camera!\n");
	} catch (tf::TransformException &ex) {
		ROS_ERROR("%s", ex.what());
		ros::Duration(1.0).sleep();
	}

	tf::StampedTransform transform_GC; //Gripper-Camera
	transform_GC.setData(transform_BG.inverse()); //
	transform_GC *= transform_BC;

	Eigen::Matrix4f t_full = EigenFromTF(transform_GC);
	cout << t_full << std::endl;

	t_full = EigenFromTF(transform_BC);
	cout << "BC" <<  std::endl << t_full << std::endl;

	t_full = EigenFromTF(transform_BG);
	cout << "BG" <<  std::endl <<  t_full << std::endl;



	return 0;
}

