#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <futrobotros/State.h>
#include <futrobotros/TeamPose.h>
#include <geometry_msgs/PointStamped.h>

// Declare the robot poses publishers
ros::Publisher strategy_pub;

/** \brief Strategy callback
 *
 * This callback should receive futrobotros::State as a message and publish
 * the references the robots should go following the defined strategy
 */
void strategyCallback(
	const futrobotros::TeamPose::ConstPtr& team_msg,
	const futrobotros::TeamPose::ConstPtr& opponent_msg,
	const geometry_msgs::PointStamped::ConstPtr& ball_msg)
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

	// Declare subscribers using message_filters for synchronization
	message_filters::Subscriber<futrobotros::TeamPose> team_pose_sub(n, "team_poses", 1000);
	message_filters::Subscriber<futrobotros::TeamPose> opponent_pose_sub(n, "opponent_poses", 1000);
	message_filters::Subscriber<geometry_msgs::PointStamped> ball_position_sub(n, "ball_position", 1000);
	// Synchronize and register the callback
	message_filters::TimeSynchronizer<futrobotros::TeamPose, futrobotros::TeamPose, geometry_msgs::PointStamped> sync(team_pose_sub, opponent_pose_sub, ball_position_sub, 10);
	sync.registerCallback(boost::bind(&strategyCallback, _1, _2, _3));
 
	// Set up strategy publisher
	strategy_pub = n.advertise<futrobotros::TeamPose>("strategy_output", 1000);

	// Spin until the end
	ros::spin();

	return 0;
}
