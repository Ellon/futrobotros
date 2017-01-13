#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <futrobotros/State.h>
#include <futrobotros/TeamPose.h>
#include <geometry_msgs/PointStamped.h>

#include "strategy.h"

// Declare the robot poses publishers
ros::Publisher strategy_pub;
Strategy * p_strategy;
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
	// Store msgs in a CONFIG variable
	CONFIG pos;
	for(unsigned i=0; i<3; ++i){
		pos.me[i].x() = team_msg->robot_pose[i].x;
		pos.me[i].y() = team_msg->robot_pose[i].y;
		pos.me[i].theta() = team_msg->robot_pose[i].theta;
		pos.op[i].x() = opponent_msg->robot_pose[i].x;
		pos.op[i].y() = opponent_msg->robot_pose[i].y;
		pos.op[i].theta() = opponent_msg->robot_pose[i].theta;
	}
	pos.ball.x() = ball_msg->point.x;
	pos.ball.y() = ball_msg->point.y;
	pos.future_ball.x() = ball_msg->point.x; // \todo Change that.
	pos.future_ball.y() = ball_msg->point.y; // \todo Change that.
	pos.vel_ball.mod = 0.0;
	pos.vel_ball.ang = 0.0;

	// Set new pos and process strategy
	p_strategy->set_pos(pos);
	p_strategy->strategy();

	// Recover references and convert to msg
	REFERENCES ref;
	p_strategy->get_ref(ref);

	/// \todo Change here for values obtained from your strategy
	futrobotros::TeamPose reference_msg;
	reference_msg.header.stamp = team_msg->header.stamp;
	// reference_msg.header.seq = team_msg->header.seq;
	reference_msg.header.frame_id = team_msg->header.frame_id;
	for(unsigned i=0; i<3; ++i){
		reference_msg.robot_pose[i].x = ref.me[i].x();
		reference_msg.robot_pose[i].y = ref.me[i].y();
		reference_msg.robot_pose[i].theta = ref.me[i].theta();
	}

	// Publish team reference msg
	strategy_pub.publish(reference_msg);
}

int main(int argc, char **argv)
{
	// Init ROS
	ros::init(argc, argv, "strategy");

	// Init a node handler
	ros::NodeHandle n;

	// Create the strategy object that holds all strategy logic
	Strategy strategy(n);

	// Store a pointer to the strategy. Used in the callback
	// \todo change this workaround for something proper
	p_strategy = &strategy;

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
