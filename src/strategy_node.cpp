#include <ros/ros.h>
#include <futrobotros/State.h>
#include <futrobotros/TeamPose.h>

// Declare the robot poses publishers
ros::Publisher strategy_pub;

/** \brief Strategy callback
 *
 * This callback should receive futrobotros::State as a message and publish
 * the references the robots should go following the defined strategy
 */
void strategyCallback(const futrobotros::TeamPose::ConstPtr& msg)
{
	/// \todo Substitute the next line by your strategy function.
	ROS_INFO("Strategy not implemented yet!");

	// Publish robot references
	/// \todo Change here for values obtained from your strategy
	futrobotros::TeamPose reference_msg;
	for(unsigned i=0; i<3; ++i){
		reference_msg.robot_pose[i].x = 0;
		reference_msg.robot_pose[i].y = 0;
		reference_msg.robot_pose[i].theta = 0;
	}

	strategy_pub.publish(reference_msg);
}

int main(int argc, char **argv)
{
	// Init ROS
	ros::init(argc, argv, "strategy");

	// Init a node handler
	ros::NodeHandle n;

	// Subscribe to the topic with the localization state
	ros::Subscriber sub = n.subscribe("strategy_input", 1000, strategyCallback);

	// Set up strategy publisher
	strategy_pub = n.advertise<futrobotros::TeamPose>("strategy_output", 1000);

	// Spin until the end
	ros::spin();

	return 0;
}
