#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// The 28 points (x,y) that define the field
double field_points[28][2] = {
  { 0.680000,  0.650000},
  { 0.000000,  0.650000},
  {-0.680000,  0.650000},
  {-0.750000,  0.580000},
  {-0.750000,  0.350000},
  {-0.750000,  0.200000},
  {-0.850000,  0.200000},
  {-0.850000, -0.200000},
  {-0.750000, -0.200000},
  {-0.750000, -0.350000},
  {-0.750000, -0.580000},
  {-0.680000, -0.650000},
  { 0.000000, -0.650000},
  { 0.680000, -0.650000},
  { 0.750000, -0.580000},
  { 0.750000, -0.350000},
  { 0.750000, -0.200000},
  { 0.850000, -0.200000},
  { 0.850000,  0.200000},
  { 0.750000,  0.200000},
  { 0.750000,  0.350000},
  { 0.750000,  0.580000},
  {-0.600000,  0.350000},
  {-0.600000, -0.350000},
  { 0.000000,  0.200000},
  { 0.000000, -0.200000},
  { 0.600000,  0.350000},
  { 0.600000, -0.350000}
};

int main(int argc, char **argv)
{
	// Init ROS
	ros::init(argc, argv, "field_visualization");

	// Init a node handler
	ros::NodeHandle n;

	// Set up publisher. Uses 'true' to set a latched topic.
	ros::Publisher pub = n.advertise<visualization_msgs::MarkerArray>("field_visualization", 1, true); 

  // Set common information for all the markers composing the field
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  // marker.header.seq = ??;
  marker.header.frame_id = "origin";
  marker.ns = "field_lines";
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.005;
  marker.scale.y = 0.01; // not used by LINE_STRIP
  marker.scale.z = 0.01; // not used by LINE_STRIP
  marker.color.a = 1.0; // Alpha shoud be set if not itś  invisible.
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.lifetime = ros::Duration(0); // 0 means forever
  marker.frame_locked = true; // marker should be retransformed into its frame every step

  // Compose the marker array msg with several markers
	visualization_msgs::MarkerArray msg;
  // line around the field
  marker.id = 0;
  for(unsigned i=0; i<22; i++)
  {
    // All points in sequence
    geometry_msgs::Point point;
    point.x = field_points[i][0];
    point.y = field_points[i][1];
    point.z = 0.0;
    marker.points.push_back(point);
  }
  {
    // Finish with first point again to close loop
    geometry_msgs::Point point;
    point.x = field_points[0][0];
    point.y = field_points[0][1];
    point.z = 0.0;
    marker.points.push_back(point);
  }  
  msg.markers.push_back(marker);
  marker.points.clear();
  // left area
  marker.id = 1;
  {
    unsigned area_indexes[4] = {4, 22, 23, 9};
    for(unsigned i=0; i<4; i++)
    {
      geometry_msgs::Point point;
      point.x = field_points[area_indexes[i]][0];
      point.y = field_points[area_indexes[i]][1];
      point.z = 0.0;
      marker.points.push_back(point);
    }
  }
  msg.markers.push_back(marker);
  marker.points.clear();
  // right area
  marker.id = 2;
  {
    unsigned area_indexes[4] = {20, 26, 27, 15};
    for(unsigned i=0; i<4; i++)
    {
      geometry_msgs::Point point;
      point.x = field_points[area_indexes[i]][0];
      point.y = field_points[area_indexes[i]][1];
      point.z = 0.0;
      marker.points.push_back(point);
    }
  }
  msg.markers.push_back(marker);
  marker.points.clear();
  // middle line
  marker.id = 3;
  {
    unsigned area_indexes[4] = {1, 24, 25, 12};
    for(unsigned i=0; i<4; i++)
    {
      geometry_msgs::Point point;
      point.x = field_points[area_indexes[i]][0];
      point.y = field_points[area_indexes[i]][1];
      point.z = 0.0;
      marker.points.push_back(point);
    }
  }
  msg.markers.push_back(marker);
  marker.points.clear();

  // Publish marker array
  pub.publish(msg);

	// The publisher is latched, so we only need to publish once. 
  // There's nothing else to do. Wait until the end.
	ros::Rate r(10);
	while(ros::ok())
		r.sleep();

	return 0;
}
