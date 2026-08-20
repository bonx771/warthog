#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

#include <geometry_msgs/PointStamped.h>
#include <robot_localization/navsat_conversions.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

std::string utm_zone;

geometry_msgs::PointStamped latLongToUTM(double latitude, double longitude)
{
  double utm_x = 0.0;
  double utm_y = 0.0;
  geometry_msgs::PointStamped utm_point;

  RobotLocalization::NavsatConversions::LLtoUTM(latitude, longitude, utm_y, utm_x, utm_zone);
  utm_point.header.frame_id = "utm";
  utm_point.header.stamp = ros::Time(0);
  utm_point.point.x = utm_x;
  utm_point.point.y = utm_y;
  utm_point.point.z = 0.0;
  return utm_point;
}

bool transformPointToFrame(
  tf::TransformListener& listener,
  const geometry_msgs::PointStamped& input_point,
  const std::string& target_frame,
  geometry_msgs::PointStamped& output_point)
{
  try
  {
    listener.waitForTransform(target_frame, input_point.header.frame_id, ros::Time(0), ros::Duration(3.0));
    listener.transformPoint(target_frame, input_point, output_point);
    return true;
  }
  catch (tf::TransformException& ex)
  {
    ROS_WARN("Could not transform waypoint marker into %s: %s", target_frame.c_str(), ex.what());
    return false;
  }
}

void publishDeleteAll(ros::Publisher& marker_pub, const std::string& marker_frame)
{
  visualization_msgs::Marker clear_marker;
  clear_marker.header.frame_id = marker_frame;
  clear_marker.header.stamp = ros::Time::now();
  clear_marker.action = visualization_msgs::Marker::DELETEALL;
  marker_pub.publish(clear_marker);
}

void publishWaypointMarker(
  ros::Publisher& marker_pub,
  const geometry_msgs::PointStamped& marker_point,
  int marker_id,
  double marker_scale)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = marker_point.header.frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = "collected_waypoints";
  marker.id = marker_id;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.pose.position.x = marker_point.point.x;
  marker.pose.position.y = marker_point.point.y;
  marker.pose.position.z = marker_point.point.z + 0.2;
  marker.scale.x = marker_scale;
  marker.scale.y = marker_scale;
  marker.scale.z = marker_scale;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.lifetime = ros::Duration(0.0);
  marker_pub.publish(marker);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "publish_waypoints_from_file");
  ros::NodeHandle nh;

  std::string path_local;
  std::string marker_frame;
  std::string marker_topic;
  double marker_scale = 0.7;

  ros::param::param<std::string>("/outdoor_waypoint_nav/coordinates_file", path_local, "/waypoint_files/points_sim.txt");
  ros::param::param<std::string>("/outdoor_waypoint_nav/waypoint_marker_frame", marker_frame, "map");
  ros::param::param<std::string>("/outdoor_waypoint_nav/waypoint_marker_topic", marker_topic, "/outdoor_waypoint_nav/collected_waypoints");
  ros::param::param<double>("/outdoor_waypoint_nav/waypoint_marker_scale", marker_scale, 0.7);

  const std::string path_abs = ros::package::getPath("outdoor_waypoint_nav") + path_local;
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>(marker_topic, 100, true);
  tf::TransformListener listener;

  ROS_INFO("Publishing waypoint markers from file: %s", path_abs.c_str());
  publishDeleteAll(marker_pub, marker_frame);
  ros::Duration(0.1).sleep();

  std::ifstream waypoint_file(path_abs.c_str());
  if (!waypoint_file.is_open())
  {
    ROS_ERROR("Unable to open waypoint file for marker publishing: %s", path_abs.c_str());
    ros::spin();
    return 1;
  }

  double latitude = 0.0;
  double longitude = 0.0;
  int marker_id = 0;
  while (waypoint_file >> latitude >> longitude)
  {
    marker_id++;
    geometry_msgs::PointStamped marker_point;
    if (transformPointToFrame(listener, latLongToUTM(latitude, longitude), marker_frame, marker_point))
    {
      publishWaypointMarker(marker_pub, marker_point, marker_id, marker_scale);
      ROS_INFO(
        "Loaded waypoint marker %d from file at (lat, lon) = (%.8f, %.8f)",
        marker_id,
        latitude,
        longitude);
    }
  }

  ROS_INFO("Published %d waypoint marker(s) from file.", marker_id);
  ros::spin();
  return 0;
}
