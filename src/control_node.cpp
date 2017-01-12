#include <ros/ros.h>
#include <futrobotros/TeamPose.h>
#include <futrobotros/TeamPWM.h>

// Declare the publisher of the control signals
ros::Publisher control_pub;

/** \brief Control callback
 *
 * This callback should receive the reference for the robots as a futrobotros::TeamPose
 * message and publish the control signals the robots should apply to reach the
 * reference
 */
void controlCallback(const futrobotros::TeamPose::ConstPtr& msg)
{
	/// \todo Substitute the next line by your control function.
	ROS_INFO("Control not implemented yet!");

	// Publish robot control signals
	/// \todo Change here for values obtained from your control function
	futrobotros::TeamPWM control_msg;
	for(unsigned i=0; i<3; ++i){
		control_msg.robot_pwm[i].left = 0.5;
		control_msg.robot_pwm[i].right = 0.7;
	}

	control_pub.publish(control_msg);
}

int main(int argc, char **argv)
{
	// Init ROS
	ros::init(argc, argv, "control");

	// Init a node handler
	ros::NodeHandle n;

	// Subscribe to the topic with the localization state
	ros::Subscriber sub = n.subscribe("control_input", 1000, controlCallback);

	// Set up strategy publisher
	control_pub = n.advertise<futrobotros::TeamPWM>("control_output", 1000);

	// Spin until the end
	ros::spin();

	return 0;
}
