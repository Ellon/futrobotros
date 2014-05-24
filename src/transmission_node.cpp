#include <ros/ros.h>
#include <futrobotros/Controls.h>

/** \brief Transmission callback
 *
 * This callback should receive the control signals to be sent to the
 * robot and then access the hardware to actually send them
 */
void transmissionCallback(const futrobotros::Controls::ConstPtr& msg)
{
	/// \todo Substitute the next line by the function to access your hardware and realize the transmission to the robots.
	ROS_INFO("Transmission not implemented yet!");
}

int main(int argc, char **argv)
{
	// Init ROS
	ros::init(argc, argv, "transmission");

	// Init a node handler
	ros::NodeHandle n;

	// Subscribe to the topic with the localization state
	ros::Subscriber sub = n.subscribe("transmission_input", 1000, transmissionCallback);

	// Spin until the end
	ros::spin();

	return 0;
}
