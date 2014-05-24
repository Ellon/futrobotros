#include <ros/ros.h>
#include <futrobotros/Poses.h>

// Declare the publisher for robot poses after corrections
ros::Publisher obstacle_avoidance_pub;

/** \brief Obstacle Avoidance callback
 *
 * This callback should receive futrobotros::Poses as a message and publish
 * the references the robots should go to avoid possible obstacles.
 */
void obstacleAvoidanceCallback(const futrobotros::Poses::ConstPtr& msg)
{
	/// \todo Substitute the next line by your strategy function.
	ROS_INFO("Obstacle avoidance not implemented yet!");

	// Publish robot references
	/// \todo Change here for values obtained from your routine to avoid obstacles
	futrobotros::Poses new_reference_msg;
	for(unsigned i=0; i<3; ++i){
		new_reference_msg.robots[i].x = 0;
		new_reference_msg.robots[i].y = 0;
		new_reference_msg.robots[i].theta = 0;
	}

	obstacle_avoidance_pub.publish(new_reference_msg);
}

int main(int argc, char **argv)
{
	// Init ROS
	ros::init(argc, argv, "obstacle_avoidance");

	// Init a node handler
	ros::NodeHandle n;

	// Subscribe to the topic with the localization state
	ros::Subscriber sub = n.subscribe("obstacle_avoidance_input", 1000, obstacleAvoidanceCallback);

	// Set up obstacle avoidance publisher
	obstacle_avoidance_pub = n.advertise<futrobotros::Poses>("obstacle_avoidance_output", 1000);

	// Spin until the end
	ros::spin();

	return 0;
}
