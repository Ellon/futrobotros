#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>

#include <futrobotros/TeamPose.h>

#include "acquisition.h"

// Declare publisher for acquisition result
ros::Publisher team_poses_pub;
ros::Publisher opponent_poses_pub;
ros::Publisher ball_pub;

// Pointer to acquisition object. Global to provide access from the acquisitionCallback
// \todo This workaround is ugly. Change it to a class.
Acquisition * p_acquisition;

/** \brief Localization callback
 *
 * This callback should receive an image as a message and publish the
 * pose of the robots and the position of the ball as a futrobotros::State
 */
void acquisitionCallback(const sensor_msgs::Image::ConstPtr& image_msg)
{
	// THIS IS A UGLY HACK!
	// Create a cv::Mat using the pointer to the data of ImagemRGB inside acquisition
	cv::Mat myimage(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3, p_acquisition->getImgPointer());
	try
	{
		// Resize image, storing it into myimage (and thus, inside the ImagemRGB inside acquisition because they use the same pointer)
		cv::resize(cv_bridge::toCvShare(image_msg, "rgb8")->image, myimage, myimage.size(), 0, 0, cv::INTER_AREA);	
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'rgb8'.", image_msg->encoding.c_str());
	}

	// Perform the image processing
	p_acquisition->acquisition();

	// Recover the configuration from the image processing
	CONFIG pos;
	p_acquisition->get_pos(pos);

	// compose team pose message
	futrobotros::TeamPose me_msg;
	me_msg.header.stamp = image_msg->header.stamp;
	// me_msg.header.seq = ?;
	me_msg.header.frame_id = "origin";
	for (int i = 0; i < 3; i++) {
		me_msg.robot_pose[i].x = pos.me[i].x();
		me_msg.robot_pose[i].y = pos.me[i].y();
		me_msg.robot_pose[i].theta = pos.me[i].theta();
	}

	// compose opponent pose message
	futrobotros::TeamPose op_msg;
	op_msg.header.stamp = image_msg->header.stamp;
	// op_msg.header.seq = ?;
	op_msg.header.frame_id = "origin";
	for (int i = 0; i < 3; i++) {
		op_msg.robot_pose[i].x = pos.op[i].x();
		op_msg.robot_pose[i].y = pos.op[i].y();
		op_msg.robot_pose[i].theta = pos.op[i].theta();
	}

	// compose ball msg 
	geometry_msgs::PointStamped ball_msg;
	ball_msg.header.stamp = image_msg->header.stamp;
	// ball_msg.header.seq = ??;
	ball_msg.header.frame_id = "origin";
	ball_msg.point.x = pos.ball.x();
	ball_msg.point.y = pos.ball.y();

	// publish messages
	team_poses_pub.publish(me_msg);
	opponent_poses_pub.publish(op_msg);
	ball_pub.publish(ball_msg);
}

int main(int argc, char **argv)
{
	// Init ROS
	ros::init(argc, argv, "acquisition");

	// Init a node handler
	ros::NodeHandle n;

	// Init a private node handler
	ros::NodeHandle pn("~");

	// Create the acquisition object that holds all image processing logic
	Acquisition acquisition(n, pn);

	// Store a pointer to the acquisition. Used in the callback
	// \todo change this workaround for something proper
	p_acquisition = &acquisition;

	image_transport::ImageTransport it(n);
	image_transport::Subscriber sub = it.subscribe("image", 1, acquisitionCallback);

	// Set up publishers
	team_poses_pub = n.advertise<futrobotros::TeamPose>("team_poses", 10);
	opponent_poses_pub = n.advertise<futrobotros::TeamPose>("opponent_poses", 10);
	ball_pub = n.advertise<geometry_msgs::PointStamped>("ball_position", 10);

	// Spin until the end
	ros::spin();

	return 0;
}
