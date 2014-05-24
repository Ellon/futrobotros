#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <futrobotros/Poses.h>

// Declare individual robot poses publishers
ros::Publisher robot_pose_pub[3];
// Declare publisher of all robot poses at the same time
ros::Publisher robot_poses_pub;
// Declare publisher for the ball
ros::Publisher ball_pos_pub;

/** \brief Localization callback
 *
 * This callback should receive an image as a message and publish the
 * pose of the robots and the position of the ball as a tf::Transform
 */
void localizationCallback(const sensor_msgs::Image::ConstPtr& msg)
{
	/// \todo Substitute the next line by your image processing functions.
	ROS_INFO("Image processing not implemented yet!");

	// Publish robot poses
	/// \todo Change here for values obtained from your image processing
	geometry_msgs::Pose2D pose2d_msg;
	for(unsigned i=0; i<3; ++i){
		pose2d_msg.x = 0;
		pose2d_msg.y = 0;
		pose2d_msg.theta = 0;
		robot_pose_pub[i].publish(pose2d_msg);
	}

	// Publish ball position
	/// \todo Change here for values obtained from your image processing
	geometry_msgs::Point point_msg;
	point_msg.x = 0;
	point_msg.y = 0;
	point_msg.z = 0;
	ball_pos_pub.publish(point_msg);
}

int main(int argc, char **argv)
{
	// Init ROS
	ros::init(argc, argv, "localization");

	// Init a node handler
	ros::NodeHandle n;

	// Subscribe to the topic with the acquired images
	ros::Subscriber sub = n.subscribe("image_input", 1000, localizationCallback);

	// Set up individual robot pose publishers
	for(unsigned i=0; i<3; ++i){
		std::ostringstream oss; oss << "robot/" << i << "/pose";
		robot_pose_pub[i] = n.advertise<geometry_msgs::Pose2D>(oss.str(), 1000);
	}

	// Set up publisher for all robot poses
	robot_poses_pub = n.advertise<futrobotros::Poses>("poses", 1000);

	// Set up ball position publisher
	ball_pos_pub = n.advertise<geometry_msgs::Point>("ball/position", 1000);

	// Spin until the end
	ros::spin();

	return 0;
}
