#include "RobotState.h"
#include <cmath>

#include <ros/ros.h>
#include <ros/package.h>
#include <urdf/model.h>

bool RobotState::loadURDF(const std::string &urdf_file_name)
{
    urdf::Model model;
    std::string package_path = ros::package::getPath("robotino_description");
    std::string urdf_path = package_path + "/urdf/" + urdf_file_name;

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
    // Calculate velocities
    float v_left = wheel_radius_ * motor_velocities[0];
    float v_right = wheel_radius_ * motor_velocities[2];

    vx = (v_right + v_left) / 2.0;
    vy = 0.0;
    omega = (v_right - v_left) / wheel_distance_;

    // Calculate position
    int left_motor_position = motor_positions[0];
    int right_motor_position = motor_positions[2];

    int delta_left = left_motor_position - angle_left;
    int delta_right = right_motor_position - angle_right;

    float deltaS = (delta_left + delta_right) / 2.0 * wheel_radius_;
    float deltaPhi = (delta_right - delta_left) / wheel_distance_;

    float R = deltaS / deltaPhi;
    x += R * (sin(phi + deltaPhi) - sin(phi));
    y += -R * (cos(phi + deltaPhi) - cos(phi));
    phi += deltaPhi;

    angle_left = left_motor_position;
    angle_right = right_motor_position;

    sequence++;
}

void RobotState::setOdometry(double x, double y, double phi)
{
    this->x = x;
    this->y = y;
    this->phi = phi;
    
    vx = 0.0;
    vy = 0.0;
    omega = 0.0;

    sequence = 0;

    angle_left = 0.0;
    angle_right = 0.0;
}