#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <futrobotros/TeamBlocked.h>
#include <futrobotros/TeamBypassControl.h>
#include <futrobotros/TeamPose.h>
#include <futrobotros/TeamPWM.h>

#include "control.h"

// Declare the publisher of the control signals
ros::Publisher control_pub;

// Pointer to control object. Global to provide access from the controlCallback
// \todo This workaround is ugly. Change it to a class.
Control *p_control;

/** \brief Control callback
 *
 * This callback should receive the reference for the robots as a futrobotros::TeamPose
 * message and publish the control signals the robots should apply to reach the
 * reference
 */
void controlCallback(
	const futrobotros::TeamPose::ConstPtr& team_pose_msg,
	const futrobotros::TeamPose::ConstPtr& team_ref_msg,
	const futrobotros::TeamBypassControl::ConstPtr& bypass_control_msg,
	const futrobotros::TeamBlocked::ConstPtr& blocked_msg)
{
	// Convert team pose msg to TPOS_ROBO
	TPOS_ROBO pos_me;
	for(unsigned i=0; i<3; ++i){
		pos_me[i].x() = team_pose_msg->robot_pose[i].x;
		pos_me[i].y() = team_pose_msg->robot_pose[i].y;
		pos_me[i].theta() = team_pose_msg->robot_pose[i].theta;
	}

	// Convert team ref msg to REFERENCES
	REFERENCES ref;
	for(unsigned i=0; i<3; ++i){
		ref.me[i].x() = team_ref_msg->robot_pose[i].x;
		ref.me[i].y() = team_ref_msg->robot_pose[i].y;
		ref.me[i].theta() = team_ref_msg->robot_pose[i].theta;
	}

	// Convert bypass control msg to bool[3] and PWM_ROBOTS
	bool bypassControl[3];
	PWM_ROBOTS pwm;
	for(unsigned i=0; i<3; ++i){
		bypassControl[i] = bypass_control_msg->bypass[i];
		pwm.me[i].left = bypass_control_msg->robot_pwm[i].left;
		pwm.me[i].right = bypass_control_msg->robot_pwm[i].right;
	}

	// Convert blocked msg to bool[3]
	bool bloqueado[3];
	for(unsigned i=0; i<3; ++i){
		bloqueado[i] = blocked_msg->blocked[i];
	}

	// Set state in the controle
	p_control->set_pos_me(pos_me);
	p_control->set_ref(ref);
	p_control->set_bypass_control(bypassControl, pwm);
	p_control->set_bloqueado(bloqueado);

	// Compute control signals
	p_control->control();

	// Recover the computed pwms
	PWM_ROBOTS computed_pwm;
	p_control->get_pwm(computed_pwm);

	// Publish robot control signals
	futrobotros::TeamPWM control_msg;
	for(unsigned i=0; i<3; ++i){
		control_msg.robot_pwm[i].left = computed_pwm.me[i].left;
		control_msg.robot_pwm[i].right = computed_pwm.me[i].right;
	}

	control_pub.publish(control_msg);
}

int main(int argc, char **argv)
{
	// Init ROS
	ros::init(argc, argv, "control");

	// Init a node handler
	ros::NodeHandle n;

	// Init a private node handler
	ros::NodeHandle pn("~");

	// Create the control object that holds all control logic
	Control control(n);

	// Store a pointer to the control. Used in the callback
	// \todo change this workaround for something proper
	p_control = &control;

	// Declare subscribers using message_filters for synchronization
	message_filters::Subscriber<futrobotros::TeamPose> team_poses_sub(n, "team_poses", 10);
	message_filters::Subscriber<futrobotros::TeamPose> team_references_sub(n, "team_references", 10);
	message_filters::Subscriber<futrobotros::TeamBypassControl> bypass_control_sub(n, "team_bypass_control", 10);
	message_filters::Subscriber<futrobotros::TeamBlocked> blocked_sub(n, "team_blocked", 10);

	// Synchronize and register the callback
	message_filters::TimeSynchronizer<futrobotros::TeamPose, futrobotros::TeamPose, futrobotros::TeamBypassControl, futrobotros::TeamBlocked>
		sync(team_poses_sub, team_references_sub, bypass_control_sub, blocked_sub, 10);
	sync.registerCallback(boost::bind(&controlCallback, _1, _2, _3, _4));

	// Set up control publisher
	control_pub = n.advertise<futrobotros::TeamPWM>("team_pwms", 10);

	// Spin until the end
	ros::spin();

	return 0;
}
