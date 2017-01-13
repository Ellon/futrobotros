#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

#include <futrobotros/BallVelocity.h>
#include <futrobotros/TeamPose.h>

#include "data.h"
#include "functions.h"
#include "parameters.h"

using namespace std;

// \todo Change these defines to somewhere else.
#define TEMPO_BOLA_FUTURA 0.1
#define DT_AMOSTR_INICIAL 1.0/30.0 

// Declare publisher for localization result
ros::Publisher future_ball_position_pub;
ros::Publisher ball_velocity_pub;

POS_BOLA last_ball;

/** \brief Localization callback
 *
 * This callback should receive an image as a message and publish the
 * pose of the robots and the position of the ball as a futrobotros::State
 */
void ballPositionCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
	/// \todo Substitute the next line by your image processing functions.
	ROS_INFO("Localization not implemented yet!");

	POS_BOLA ball;
	ball.x() = msg->point.x;
	ball.y() = msg->point.y;

	double dt_amostr = DT_AMOSTR_INICIAL;

	//**********************************************************
	//********** CALCULO DA VELOCIDADE DA BOLA *****************
	//**********************************************************
	double vbola, thetabola;
	VELOCITY vel_ball;
	if (ball.x() != POSITION_UNDEFINED && ball.y() != POSITION_UNDEFINED) {
		vbola = hypot(ball.y() - last_ball.y(),
		              ball.x() - last_ball.x()) / dt_amostr;
		thetabola = arc_tang(ball.y() - last_ball.y(),
		                     ball.x() - last_ball.x());
		if (vbola > VEL_BOLA_PARADA) {
			vel_ball.mod = (vel_ball.mod + vbola) / 2.0;
			vel_ball.ang = (vel_ball.ang + thetabola) / 2.0;
		}
		else {
			vel_ball.mod = vel_ball.mod / 2.0;
			vel_ball.ang  = 0.0;
		}
	}
	else {
		ball = last_ball;
		vel_ball.mod = 0.0;
		vel_ball.ang = 0.0;
	}
	if (isnan(ball.x()) || isnan(ball.y())) {
		cerr << "POS ATUAL DA BOLA EH NAN!!! "
		     << __FILE__ << " " << __LINE__ << endl;
	}
	if (isnan(vel_ball.mod) || isnan(vel_ball.ang)) {
		cerr << "VELOCIDADE DA BOLA EH NAN!!! "
		     << __FILE__ << " " << __LINE__ << endl;
	}

	POS_BOLA future_ball;
	future_ball.x() = ball.x() + vel_ball.mod * cos(vel_ball.ang) * TEMPO_BOLA_FUTURA;
	future_ball.y() = ball.y() + vel_ball.mod * sin(vel_ball.ang) * TEMPO_BOLA_FUTURA;

	if (isnan(future_ball.x()) || isnan(future_ball.y())) {
		cerr << "POS_FUT DA BOLA EH NAN!!! "
		     << __FILE__ << " " << __LINE__ << endl;
		cerr << "\t" << ball.x() << " " << ball.y() << " "
		     << vel_ball.mod << " " << vel_ball.ang << " "
		     << future_ball.x() << " " << future_ball.y() << endl;
	}

	// Publish future ball position
	geometry_msgs::PointStamped future_ball_msg;
	future_ball_msg.header.stamp = msg->header.stamp;
	// future_ball_msg.header.seq = msg->header.seq;
	future_ball_msg.header.frame_id = msg->header.frame_id;
	future_ball_msg.point.x = future_ball.x();
	future_ball_msg.point.y = future_ball.y();

	future_ball_position_pub.publish(future_ball_msg);

	// Publish ball velocity
	futrobotros::BallVelocity ball_velocity_msg;
	ball_velocity_msg.header.stamp = msg->header.stamp;
	// ball_velocity_msg.header.seq = msg->header.seq;
	ball_velocity_msg.header.frame_id = msg->header.frame_id;
	ball_velocity_msg.module = vel_ball.mod;
	ball_velocity_msg.angle = vel_ball.ang;
	ball_velocity_pub.publish(ball_velocity_msg);

	last_ball = ball;
}

int main(int argc, char **argv)
{
	// Init ROS
	ros::init(argc, argv, "localization");

	// Init a node handler
	ros::NodeHandle n;

	last_ball.x() = 0.0;
	last_ball.y() = 0.0;

	// Subscribe to the topic with the acquired images
	ros::Subscriber ball_position_sub = n.subscribe("ball_position", 10, ballPositionCallback);

	// Set future ball position publisher
	future_ball_position_pub = n.advertise<geometry_msgs::PointStamped>("future_ball_position", 10);
	ball_velocity_pub = n.advertise<futrobotros::BallVelocity>("ball_velocity", 10);

	// Spin until the end
	ros::spin();

	return 0;
}
