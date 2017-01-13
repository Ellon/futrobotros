#include <string>
#include <sstream>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <futrobotros/TeamPose.h>

using namespace std;

std::string my_namespace, prefix;

void teamPoseCallback(const futrobotros::TeamPose::ConstPtr& msg) {
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  for (int i = 0; i < 3; i++) {
    transform.setOrigin( tf::Vector3(msg->robot_pose[i].x, msg->robot_pose[i].y, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, msg->robot_pose[i].theta);
    transform.setRotation(q);
    ostringstream oss;
    oss << my_namespace << "_" << prefix << "_" << i;
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "origin", oss.str()));
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "team_pose_tf_broadcaster");

  ros::NodeHandle n;

  // recover current namespace
  my_namespace = ros::this_node::getNamespace();
  if (my_namespace[0] == '/')
    my_namespace.erase(0, 1); // remove leading slash

  // recover prefix from the private parameter
  ros::NodeHandle pn("~"); // node to recover private parameters
  pn.param<std::string>("prefix", prefix, "");

  ros::Subscriber sub = n.subscribe("team_poses_input", 10, &teamPoseCallback);

  ros::spin();
  return 0;
};