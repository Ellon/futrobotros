#include <string>
#include <sstream>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <futrobotros/TeamPose.h>

using namespace std;

void teamPoseCallback(const futrobotros::TeamPose::ConstPtr& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  for (int i = 0; i < 3; i++) {
    transform.setOrigin( tf::Vector3(msg->robot_pose[i].x, msg->robot_pose[i].y, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, msg->robot_pose[i].theta);
    transform.setRotation(q);
    ostringstream oss;
    oss << "robot" << i;
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "origin", oss.str()));
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "team_pose_tf_broadcaster");

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("team_pose", 10, &teamPoseCallback);

  ros::spin();
  return 0;
};