#include <stdio.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>


int main( int argc, char** argv )
{
  ros::init(argc, argv, "rviz_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::ARROW;

  double x;
  double y;
  double z;
  double rx;
  double ry;
  double rz;
  double rw;

  printf("Input: x y z rx ry rz rw\n");
  scanf("%lf", &x);
  scanf("%lf", &y);
  scanf("%lf", &z);
  scanf("%lf", &rx);
  scanf("%lf", &ry);
  scanf("%lf", &rz);
  scanf("%lf", &rw);

  tf::Quaternion q = tf::createQuaternionFromRPY(0.0, 0.0, M_PI / 2);
  tf::Quaternion q2 = q * tf::Quaternion(rx, ry, rz, rw);

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/base_footprint";
    marker.header.stamp = ros::Time::now();

    marker.ns = "arrow_marker";
    marker.id = 0;

    marker.type = shape;

    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    tf::quaternionTFToMsg(q2, marker.pose.orientation);

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.1;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);

    r.sleep();
  }
}
