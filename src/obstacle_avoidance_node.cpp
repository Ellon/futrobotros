#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/PointStamped.h>

#include <futrobotros/TeamPose.h>

#include "obstacles.h"

// Declare the publisher for robot poses after corrections
ros::Publisher corrected_references_pub;

Obstacles *p_obstacles;


/** \brief Obstacle Avoidance callback
 *
 * This callback should receive futrobotros::TeamPose as a message and publish
 * the references the robots should go to avoid possible obstacles.
 */
void obstacleAvoidanceCallback(
	const futrobotros::TeamPose::ConstPtr& team_poses_msg,
	const futrobotros::TeamPose::ConstPtr& opponent_poses_msg,
	const futrobotros::TeamPose::ConstPtr& team_ref_msg,
	const geometry_msgs::PointStamped::ConstPtr& ball_msg)
{
	// Convert team pose msg to TPOS_ROBO
	TPOS_ROBO pos_me;
	for(unsigned i=0; i<3; ++i){
		pos_me[i].x() = team_poses_msg->robot_pose[i].x;
		pos_me[i].y() = team_poses_msg->robot_pose[i].y;
		pos_me[i].theta() = team_poses_msg->robot_pose[i].theta;
	}

	// Convert opponent pose msg to TPOS_ROBO
	TPOS_ROBO pos_op;
	for(unsigned i=0; i<3; ++i){
		pos_op[i].x() = opponent_poses_msg->robot_pose[i].x;
		pos_op[i].y() = opponent_poses_msg->robot_pose[i].y;
		pos_op[i].theta() = opponent_poses_msg->robot_pose[i].theta;
	}

	// Convert team ref msg to REFERENCES
	REFERENCES ref;
	for(unsigned i=0; i<3; ++i){
		ref.me[i].x() = team_ref_msg->robot_pose[i].x;
		ref.me[i].y() = team_ref_msg->robot_pose[i].y;
		ref.me[i].theta() = team_ref_msg->robot_pose[i].theta;
	}

	// Convert ball position msg to POS_BOLA
	POS_BOLA pos_ball;
	pos_ball.x() = ball_msg->point.x;
	pos_ball.y() = ball_msg->point.y;

	// Set state in obstacles
	p_obstacles->set_pos_me(pos_me);
	p_obstacles->set_pos_op(pos_op);
	p_obstacles->set_ref(ref);
	p_obstacles->set_pos_ball(pos_ball);

	// Compute references to avoid obstacles
	p_obstacles->obstacles();

	// Recover the computed references
	REFERENCES computed_ref;
	p_obstacles->get_ref(computed_ref);

	// Publish references that avoid obstacles
	futrobotros::TeamPose computed_reference_msg;
	computed_reference_msg.header.stamp = team_poses_msg->header.stamp;
	// computed_reference_msg.header.seq = team_poses_msg->header.seq;
	computed_reference_msg.header.frame_id = team_poses_msg->header.frame_id;
	for(unsigned i=0; i<3; ++i){
		computed_reference_msg.robot_pose[i].x = computed_ref.me[i].x();
		computed_reference_msg.robot_pose[i].y = computed_ref.me[i].y();
		computed_reference_msg.robot_pose[i].theta = computed_ref.me[i].theta();
	}
	corrected_references_pub.publish(computed_reference_msg);
}

int main(int argc, char **argv)
{
	// Init ROS
	ros::init(argc, argv, "obstacle_avoidance");

	// Init a node handler
	ros::NodeHandle n;

	// Init a private node handler
	ros::NodeHandle pn("~");

	// Create the obstacles object that holds all obstacle avoidance logic
	Obstacles obstacles(n);

	// Store a pointer to the obstacles. Used in the callback
	// \todo change this workaround for something proper
	p_obstacles = &obstacles;

	// Declare subscribers using message_filters for synchronization
	message_filters::Subscriber<futrobotros::TeamPose> team_poses_sub(n, "team_poses", 10);
	message_filters::Subscriber<futrobotros::TeamPose> opponent_poses_sub(n, "opponent_poses", 10);
	message_filters::Subscriber<futrobotros::TeamPose> team_references_sub(n, "team_references", 10);
	message_filters::Subscriber<geometry_msgs::PointStamped> ball_position_sub(n, "ball_position", 10);

	// Synchronize and register the callback
	message_filters::TimeSynchronizer<futrobotros::TeamPose, futrobotros::TeamPose, futrobotros::TeamPose, geometry_msgs::PointStamped>
		sync(team_poses_sub, opponent_poses_sub, team_references_sub, ball_position_sub, 10);
	sync.registerCallback(boost::bind(&obstacleAvoidanceCallback, _1, _2, _3, _4));

	// Set up reference publisher
	corrected_references_pub = n.advertise<futrobotros::TeamPose>("team_corrected_references", 10);

	// Spin until the end
	ros::spin();

	return 0;
}
