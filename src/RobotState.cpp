#include "RobotState.h"
#include <cmath>

#include <ros/ros.h>
#include <ros/package.h>
#include <urdf/model.h>

#define CPR 98000
#define WHEEL_SLIP_CORRECTION 0.413

bool RobotState::loadURDF(const std::string &urdf_file_name)
{
    urdf::Model model;
    std::string package_path = ros::package::getPath("robotino_description");
    std::string urdf_path = package_path + "/" + urdf_file_name;

    if (!model.initFile(urdf_path))
    {
        ROS_ERROR("Failed to parse urdf file");
        return false;
    }

    // Retrieve the wheel separation distance
    auto frontLeftWheel = model.getLink("wheel0_link");
    auto frontRightWheel = model.getLink("wheel1_link");
    if (frontLeftWheel && frontRightWheel)
    {
        wheel_distance_ = std::abs(frontLeftWheel->parent_joint->parent_to_joint_origin_transform.position.y -
                                   frontRightWheel->parent_joint->parent_to_joint_origin_transform.position.y);
        ROS_INFO("Wheel separation distance: %f", wheel_distance_);
    }
    else
    {
        ROS_ERROR("Failed to retrieve wheel positions");
        return false;
    }

    // Retrieve the wheel radius
    auto wheelLink = model.getLink("wheel0_link"); // Assuming all wheels have the same radius
    if (wheelLink && wheelLink->visual && wheelLink->visual->geometry)
    {
        auto wheelGeometry = std::dynamic_pointer_cast<urdf::Cylinder>(wheelLink->visual->geometry);
        if (wheelGeometry)
        {
            wheel_radius_ = wheelGeometry->radius;
            ROS_INFO("Wheel radius: %f", wheel_radius_);
        }
        else
        {
            ROS_ERROR("Failed to retrieve wheel radius");
            return false;
        }
    }
    else
    {
        ROS_ERROR("Failed to retrieve wheel link");
        return false;
    }

    return true;
}

void RobotState::update(std::vector<float> motor_velocities, std::vector<int> motor_positions)
{

    // Calculate encoder positions
    int left_motor_position = motor_positions[0]; // front left
    int right_motor_position = motor_positions[2]; // back right


    // Calculate the change in encoder positions
    int delta_left = -(left_motor_position - last_left_motor_position); // negative sign because the encoder values are decreasing
    int delta_right = right_motor_position - last_right_motor_position;

    // conversion to radians
    double delta_left_rad = delta_left * 2 * M_PI / CPR; 
    double delta_right_rad = delta_right * 2 * M_PI / CPR;

    // Calculate the distance traveled by each wheel
    double delta_left_distance = delta_left_rad * wheel_radius_;
    double delta_right_distance = delta_right_rad * wheel_radius_;

    ROS_INFO("delta_left: %f, delta_right: %f", delta_left_distance, delta_right_distance);
    
    float deltaS = (delta_left_distance + delta_right_distance) / 2;
    float deltaPhi = (float)(delta_right_distance - delta_left_distance) / wheel_distance_;

    // since we don't have a 100% correct differential drive platform, the rotation is corrected with a constant factor
    deltaPhi *= WHEEL_SLIP_CORRECTION;    

    // Calculate the new position and orientation of the robot
    x += deltaS * cos(phi + deltaPhi / 2);
    y += deltaS * sin(phi + deltaPhi / 2);
    phi += deltaPhi;

    // save the velocities
    last_left_motor_position = left_motor_position;
    last_right_motor_position = right_motor_position;

    sequence++;
}

void RobotState::setOdometry(double x, double y, double phi)
{
    ROS_INFO("Setting odometry to x: %f, y: %f, phi: %f", x, y, phi);
    this->x = x;
    this->y = y;
    this->phi = phi;

    vx = 0.0;
    vy = 0.0;
    omega = 0.0;

    sequence = 0;

    // last_left_motor_position = 0.0;
    // last_right_motor_position = 0.0;
}