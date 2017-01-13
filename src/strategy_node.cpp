#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/PointStamped.h>

#include <futrobotros/BallVelocity.h>
#include <futrobotros/TeamBlocked.h>
#include <futrobotros/TeamBypassControl.h>
#include <futrobotros/TeamPose.h>

#include "strategy.h"

// Declare the robot poses publishers
ros::Publisher references_pub;
ros::Publisher bypass_control_pub;
ros::Publisher blocked_pub;

// Pointer to strategy object. Global to provide access from the strategyCallback
// \todo This workaround is ugly. Change it to a class.
Strategy * p_strategy;

/** \brief Strategy callback
 *
 * This callback should receive futrobotros::State as a message and publish
 * the references the robots should go following the defined strategy
 */
void strategyCallback(
	const futrobotros::TeamPose::ConstPtr& team_msg,
	const futrobotros::TeamPose::ConstPtr& opponent_msg,
	const geometry_msgs::PointStamped::ConstPtr& ball_msg,
	const geometry_msgs::PointStamped::ConstPtr& future_ball_msg,
	const futrobotros::BallVelocity::ConstPtr& vel_ball_msg)
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
	pos.future_ball.x() = future_ball_msg->point.x;
	pos.future_ball.y() = future_ball_msg->point.y;
	pos.vel_ball.mod = vel_ball_msg->module;
	pos.vel_ball.ang = vel_ball_msg->angle;

	// Set new pos and process strategy
	p_strategy->set_pos(pos);
	p_strategy->strategy();

	// Recover references and convert to msg
	REFERENCES ref;
	p_strategy->get_ref(ref);
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
	references_pub.publish(reference_msg);

	// Recover bypassControl state, convert to message and publish
	bool bypassControl[3];
	PWM_ROBOTS pwm;
	p_strategy->get_bypass_control(bypassControl, pwm);
	futrobotros::TeamBypassControl bypass_control_msg;
	bypass_control_msg.header.stamp = team_msg->header.stamp;
	// bypass_control_msg.header.seq = team_msg->header.seq;
	bypass_control_msg.header.frame_id = team_msg->header.frame_id; // Should be the same?
	for(unsigned i=0; i<3; ++i){
		bypass_control_msg.bypass[i] = bypassControl[i];
		bypass_control_msg.robot_pwm[i].left = pwm.me[i].left;
		bypass_control_msg.robot_pwm[i].right = pwm.me[i].right;
	}
	bypass_control_pub.publish(bypass_control_msg);

	// Recover blocked state, convert to message and publish
	bool bloqueado[3];
	p_strategy->get_bloqueado(bloqueado);
	futrobotros::TeamBlocked blocked_msg;
	blocked_msg.header.stamp = team_msg->header.stamp;
	// blocked_msg.header.seq = team_msg->header.seq;
	blocked_msg.header.frame_id = team_msg->header.frame_id; // Should be the same?
	for(unsigned i=0; i<3; ++i){
		blocked_msg.blocked[i] = bloqueado[i];
	}
	blocked_pub.publish(blocked_msg);
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
	message_filters::Subscriber<futrobotros::TeamPose> team_pose_sub(n, "team_poses", 10);
	message_filters::Subscriber<futrobotros::TeamPose> opponent_pose_sub(n, "opponent_poses", 10);
	message_filters::Subscriber<geometry_msgs::PointStamped> ball_position_sub(n, "ball_position", 10);
	message_filters::Subscriber<geometry_msgs::PointStamped> future_ball_position_sub(n, "future_ball_position", 10);
	message_filters::Subscriber<futrobotros::BallVelocity> ball_velocity_sub(n, "ball_velocity", 10);
	// Synchronize and register the callback
	message_filters::TimeSynchronizer<futrobotros::TeamPose, futrobotros::TeamPose, geometry_msgs::PointStamped, geometry_msgs::PointStamped, futrobotros::BallVelocity>
		sync(team_pose_sub, opponent_pose_sub, ball_position_sub, future_ball_position_sub, ball_velocity_sub, 10);
	sync.registerCallback(boost::bind(&strategyCallback, _1, _2, _3, _4, _5));

	// Set up strategy publisher
	references_pub = n.advertise<futrobotros::TeamPose>("team_references", 10);
	bypass_control_pub = n.advertise<futrobotros::TeamBypassControl>("team_bypass_control", 10);
	blocked_pub = n.advertise<futrobotros::TeamBlocked>("team_blocked", 10);

	// Spin until the end
	ros::spin();

	return 0;
}
