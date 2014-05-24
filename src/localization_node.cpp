#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <futrobotros/State.h>

// Declare publisher for localization result
ros::Publisher localization_pub;

/** \brief Localization callback
 *
 * This callback should receive an image as a message and publish the
 * pose of the robots and the position of the ball as a futrobotros::State
 */
void localizationCallback(const sensor_msgs::Image::ConstPtr& msg)
{
	/// \todo Substitute the next line by your image processing functions.
	ROS_INFO("Image processing not implemented yet!");

	// Publish localization result as a state message
	/// \todo Change here for values obtained from your image processing
	futrobotros::State state_msg;
	for(unsigned i=0; i<3; ++i){
		state_msg.robots[i].x = 0;
		state_msg.robots[i].y = 0;
		state_msg.robots[i].theta = 0;
	}
	state_msg.ball.x = 0;
	state_msg.ball.y = 0;
	state_msg.ball.z = 0;

	localization_pub.publish(state_msg);
}

int main(int argc, char **argv)
{
	// Init ROS
	ros::init(argc, argv, "localization");

	// Init a node handler
	ros::NodeHandle n;

	// Subscribe to the topic with the acquired images
	ros::Subscriber sub = n.subscribe("localization_input", 1000, localizationCallback);

	// Set up ball position publisher
	localization_pub = n.advertise<futrobotros::State>("localization_output", 1000);

	// Spin until the end
	ros::spin();

	return 0;
}
