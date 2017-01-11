#include <ros/ros.h>

#include <geometry_msgs/Point.h>

#include <futrobotros/TeamPose.h>
#include <futrobotros/TeamPWM.h>

void yellowTeamControlCallback(const futrobotros::TeamPWM::ConstPtr& msg)
{

}

void blueTeamControlCallback(const futrobotros::TeamPWM::ConstPtr& msg)
{

}

int main(int argc, char **argv)
{
	// Init ROS
	ros::init(argc, argv, "simulator");

	// Init a node handler
	ros::NodeHandle n;

	// Set up topics to publish simulated data
	ros::Publisher yellow_team_poses_pub = n.advertise<futrobotros::TeamPose>("yellow_team_poses", 1000);
	ros::Publisher blue_team_poses_pub = n.advertise<futrobotros::TeamPose>("blue_team_poses", 1000);
	ros::Publisher ball_pub = n.advertise<geometry_msgs::Point>("ball_position", 1000);

	// Subscribe to the topic with the acquired images
	ros::Subscriber sub_yellow_team_control = n.subscribe("yellow_team_pwms", 1000, yellowTeamControlCallback);
	ros::Subscriber sub_blue_team_control = n.subscribe("blue_team_pwms", 1000, blueTeamControlCallback);

	// Spin until the end
	ros::spin();

	return 0;
}
