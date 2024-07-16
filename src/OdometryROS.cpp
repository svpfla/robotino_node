/*
 * OdometryROS.cpp
 *
 *  Created on: 07.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#include "OdometryROS.h"
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>

OdometryROS::OdometryROS()
{

	odometry_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1, true);
	robot_state_.loadURDF("robotino.urdf");

	reset_odometry_server_ = nh_.advertiseService("reset_odometry",
												  &OdometryROS::resetOdometryCallback, this);
}

OdometryROS::~OdometryROS()
{
	odometry_pub_.shutdown();
	reset_odometry_server_.shutdown();
}

void OdometryROS::setTimeStamp(ros::Time stamp)
{
	stamp_ = stamp;
}
void OdometryROS::readingsEvent(double x, double y, double phi,
								float vx, float vy, float omega, unsigned int sequence)
{

	// calculate odometry from a different source
	motor_array_.getMotorReadings(motor_velocities_, motor_positions_);
	robot_state_.update(motor_velocities_, motor_positions_);

	geometry_msgs::Quaternion phi_quat = tf::createQuaternionMsgFromYaw(robot_state_.getPhi());

	// Construct messages
	odometry_msg_.header.seq = robot_state_.getSequence();
	odometry_msg_.header.frame_id = "odom";
	odometry_msg_.header.stamp = stamp_;
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

	odometry_transform_.header.frame_id = "odom";
	odometry_transform_.header.stamp = odometry_msg_.header.stamp;
	odometry_transform_.child_frame_id = "base_link";
	odometry_transform_.transform.translation.x = robot_state_.getX();
	odometry_transform_.transform.translation.y = robot_state_.getY();
	odometry_transform_.transform.translation.z = 0.0;
	odometry_transform_.transform.rotation = phi_quat;

	odometry_transform_broadcaster_.sendTransform(odometry_transform_);

	// Publish the msg
	odometry_pub_.publish(odometry_msg_);
}

bool OdometryROS::resetOdometryCallback(
	robotino_msgs::ResetOdometry::Request &req,
	robotino_msgs::ResetOdometry::Response &res)
{
	set(req.x, req.y, req.phi, true);
	robot_state_.setOdometry(req.x, req.y, req.phi);

	return true;
}
