/*
 * OdometryROS.cpp
 *
 *  Created on: 07.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#include "OdometryROS.h"
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include "robotino_msgs/MotorReadings.h"

OdometryROS::OdometryROS()
{

	odometry_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1, true);
	robot_state_.loadURDF("robotino.urdf");
	odometry_initialized_ = false;
	reset_odometry_server_ = nh_.advertiseService("reset_odometry",
												  &OdometryROS::resetOdometryCallback, this);
	motor_readings_sub_ = nh_.subscribe("motor_readings", 1, &OdometryROS::motorReadingsEvent, this);
}

OdometryROS::~OdometryROS()
{
	odometry_pub_.shutdown();
	reset_odometry_server_.shutdown();
	motor_readings_sub_.shutdown();
}

void OdometryROS::setTimeStamp(ros::Time stamp)
{
	stamp_ = stamp;
}

void OdometryROS::motorReadingsEvent(const robotino_msgs::MotorReadingsConstPtr &msg)
{

	const unsigned int motor_readings_threshold = 3;
	// to avoid initial odometry jump since the encoders are not zeroed
	if (!odometry_initialized_ && robot_state_.getSequence() <= motor_readings_threshold)
	{
		robot_state_.setOdometry(0.0, 0.0, 0.0);
		odometry_initialized_ = robot_state_.getSequence() > motor_readings_threshold;
	}

	// only gets called when motor readings change

	std::vector<float> motor_velocities(4);
	std::vector<int> motor_positions(4);

	motor_velocities[0] = msg->velocities[0];
	motor_velocities[1] = msg->velocities[1];
	motor_velocities[2] = msg->velocities[2];
	motor_velocities[3] = msg->velocities[3];

	motor_positions[0] = msg->positions[0];
	motor_positions[1] = msg->positions[1];
	motor_positions[2] = msg->positions[2];
	motor_positions[3] = msg->positions[3];

	ros::Time stamp = msg->stamp;

	// Calculate odometry
	robot_state_.update(motor_velocities, motor_positions);

	// Construct messages
	geometry_msgs::Quaternion phi_quat = tf::createQuaternionMsgFromYaw(robot_state_.getPhi());
	odometry_msg_.header.seq = robot_state_.getSequence();
	odometry_msg_.header.frame_id = "odom";
	odometry_msg_.header.stamp = stamp;
	odometry_msg_.child_frame_id = "base_link";
	odometry_msg_.pose.pose.position.x = robot_state_.getX();
	odometry_msg_.pose.pose.position.y = robot_state_.getY();
	odometry_msg_.pose.pose.position.z = 0.0;
	odometry_msg_.pose.pose.orientation = phi_quat;
	odometry_msg_.twist.twist.linear.x = robot_state_.getVx();
	odometry_msg_.twist.twist.linear.y = robot_state_.getVy();
	odometry_msg_.twist.twist.linear.z = 0.0;
	odometry_msg_.twist.twist.angular.x = 0.0;
	odometry_msg_.twist.twist.angular.y = 0.0;
	odometry_msg_.twist.twist.angular.z = robot_state_.getOmega();

	// Publish the msg
	odometry_pub_.publish(odometry_msg_);
}

void OdometryROS::readingsEvent(double x, double y, double phi,
								float vx, float vy, float omega, unsigned int sequence)
{

	// gets called periodically compared to motorReadingsEvent
	geometry_msgs::Quaternion phi_quat = tf::createQuaternionMsgFromYaw(robot_state_.getPhi());
	odometry_transform_.header.frame_id = "odom";
	odometry_transform_.header.stamp = ros::Time::now();
	odometry_transform_.child_frame_id = "base_link";
	odometry_transform_.transform.translation.x = robot_state_.getX();
	odometry_transform_.transform.translation.y = robot_state_.getY();
	odometry_transform_.transform.translation.z = 0.0;
	odometry_transform_.transform.rotation = phi_quat;
	odometry_transform_broadcaster_.sendTransform(odometry_transform_);
}

bool OdometryROS::resetOdometryCallback(
	robotino_msgs::ResetOdometry::Request &req,
	robotino_msgs::ResetOdometry::Response &res)
{
	set(req.x, req.y, req.phi, true);				 // reset internal odometry (not used in this example)
	robot_state_.setOdometry(req.x, req.y, req.phi); // reset odometry in RobotState

	return true;
}
